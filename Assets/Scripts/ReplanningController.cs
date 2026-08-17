using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Navigation state of the autonomous Tactical UAV.
/// </summary>
public enum NavigationState
{
    Normal,          // Cruising along verified safe trajectory
    ThreatDetected,  // Obstacle detected in forward sector
    Replanning,      // Active dynamic A* path recalculation in progress
    Rerouting,       // Trajectory updated; navigating around obstacle
    NoSafePath       // Target unreachable; UAV entered emergency safe hold
}

/// <summary>
/// Autonomous Dynamic Replanning Controller.
/// Listens to ThreatAssessment telemetry and orchestrates real-time A* path replanning
/// from the UAV's current flight position to the mission target when collision risks arise.
/// </summary>
[RequireComponent(typeof(PathFollower))]
[RequireComponent(typeof(ThreatAssessment))]
[RequireComponent(typeof(UAVPerception))]
public class ReplanningController : MonoBehaviour
{
    [Header("Replanning Parameters")]
    [Tooltip("Cooldown time between consecutive replanning calculations in seconds.")]
    [SerializeField] private float replanCooldown = 1.0f;

    [Tooltip("Minimum threat level required to trigger dynamic replanning.")]
    [SerializeField] private ThreatLevel triggerThreshold = ThreatLevel.Warning;

    [Tooltip("Maximum Time-To-Collision (TTC) threshold to force emergency replanning in seconds.")]
    [SerializeField] private float emergencyTtcThreshold = 4.0f;

    [Header("Telemetry & Diagnostics")]
    [SerializeField] private bool logReplanningEvents = true;
    [SerializeField] private bool showGizmos = true;

    // Public Read-Only Telemetry State
    public NavigationState State { get; private set; } = NavigationState.Normal;
    public int ReplanCount => replanCount;
    public float LastReplanTime => lastReplanTime;
    public Vector3 LastReplanPosition => lastReplanPosition;

    // Reactive Events for External Systems
    public event Action<NavigationState> OnStateChanged;
    public event Action<List<Node>> OnPathReplanned;
    public event Action OnNoSafePathFound;

    private PathFollower pathFollower;
    private ThreatAssessment threatAssessment;
    private UAVPerception perception;
    private Pathfinding pathfinding;

    private float lastReplanTime = -10f;
    private int replanCount = 0;
    private Vector3 lastReplanPosition;
    private GameObject currentlyAvoidingObstacle = null;

    private void Awake()
    {
        pathFollower = GetComponent<PathFollower>();
        threatAssessment = GetComponent<ThreatAssessment>();
        perception = GetComponent<UAVPerception>();
        pathfinding = FindFirstObjectByType<Pathfinding>();
    }

    private void Start()
    {
        if (threatAssessment != null)
        {
            threatAssessment.OnThreatEvaluated += HandleThreatEvaluated;
            threatAssessment.OnCriticalThreatDetected += HandleCriticalThreat;
        }
    }

    private void OnDestroy()
    {
        if (threatAssessment != null)
        {
            threatAssessment.OnThreatEvaluated -= HandleThreatEvaluated;
            threatAssessment.OnCriticalThreatDetected -= HandleCriticalThreat;
        }
    }

    private void Update()
    {
        UpdateNavigationState();
    }

    private void HandleCriticalThreat(ThreatReport report)
    {
        if (State == NavigationState.Replanning)
            return;

        if (State == NavigationState.Rerouting &&
            report.ThreateningObstacle.GameObject != null &&
            report.ThreateningObstacle.GameObject == currentlyAvoidingObstacle)
        {
            return;
        }

        TryExecuteReplan("Critical Collision Risk Detected", report);
    }

    private void HandleThreatEvaluated(ThreatReport report)
    {
        if (report.ThreatLevel >= triggerThreshold && report.TimeToCollision <= emergencyTtcThreshold)
        {
            if (State == NavigationState.Replanning)
                return;

            if (State == NavigationState.Rerouting &&
                report.ThreateningObstacle.GameObject != null &&
                report.ThreateningObstacle.GameObject == currentlyAvoidingObstacle)
            {
                return;
            }

            TryExecuteReplan($"Threat Level {report.ThreatLevel} within TTC threshold ({report.TimeToCollision:F2}s)", report);
        }
    }

    /// <summary>
    /// Attempts to dynamically replan the UAV's flight route from its current position to the mission target.
    /// </summary>
    public bool TryExecuteReplan(string triggerReason, ThreatReport report)
    {
        if (pathFollower == null || !pathFollower.IsFollowing)
            return false;

        if (State == NavigationState.Replanning)
            return false;

        if (State == NavigationState.Rerouting &&
            report.ThreateningObstacle.GameObject != null &&
            report.ThreateningObstacle.GameObject == currentlyAvoidingObstacle)
        {
            return false;
        }

        if (Time.time - lastReplanTime < replanCooldown)
            return false;

        // Strictly validate threat metrics before allowing dynamic replan
        if (!float.IsFinite(report.DistanceToCollision) || !float.IsFinite(report.TimeToCollision) || report.ObstructedWaypointIndex < 0)
            return false;

        if (pathfinding == null || pathfinding.targetTransform == null)
        {
            pathfinding = FindFirstObjectByType<Pathfinding>();
            if (pathfinding == null || pathfinding.targetTransform == null)
                return false;
        }

        SetState(NavigationState.Replanning);

        Vector3 currentPos = transform.position;
        Vector3 targetPos = pathfinding.targetTransform.position;
        lastReplanPosition = currentPos;
        lastReplanTime = Time.time;
        replanCount++;
        currentlyAvoidingObstacle = report.ThreateningObstacle.GameObject;

        // Execute A* search from the current physical UAV coordinates to the mission target
        pathfinding.FindPath(currentPos, targetPos);

        if (pathfinding.path != null && pathfinding.path.Count > 0)
        {
            // Transition active flight path to the newly calculated safe detour
            pathFollower.StartFollowing(pathfinding.path);
            SetState(NavigationState.Rerouting);

            if (logReplanningEvents)
            {
                string obsName = report.ThreateningObstacle.GameObject != null
                    ? report.ThreateningObstacle.GameObject.name
                    : "Obstacle";

                Debug.Log(
                    $"<color=#00FFFF><b>[ReplanningController] DYNAMIC REPLAN SUCCESS (Replan #{replanCount})</b></color>\n" +
                    $"  • Reason: {triggerReason}\n" +
                    $"  • Threat: {report.ThreatLevel} | Obstacle: {obsName}\n" +
                    $"  • TTC: {report.TimeToCollision:F2}s | Distance: {report.DistanceToCollision:F2}m | Waypoint: {report.ObstructedWaypointIndex}\n" +
                    $"  • Replan Origin: {currentPos:F2} -> Target: {targetPos:F2}\n" +
                    $"  • New Waypoints: {pathfinding.path.Count} nodes (smoothed from {pathfinding.rawPath?.Count ?? pathfinding.path.Count} raw nodes)");
            }

            OnPathReplanned?.Invoke(pathfinding.path);
            return true;
        }
        else
        {
            // If no valid path exists around the obstacle, enter emergency safe hold
            pathFollower.StopFollowing();
            SetState(NavigationState.NoSafePath);

            Debug.LogWarning(
                $"<color=#FF4500><b>[ReplanningController] DYNAMIC REPLAN FAILED</b></color>\n" +
                $"  • No safe A* path found from current position {currentPos:F2} to target {targetPos:F2}.\n" +
                $"  • UAV halted in emergency Safe Hold.");

            OnNoSafePathFound?.Invoke();
            return false;
        }
    }

    private void UpdateNavigationState()
    {
        if (State == NavigationState.Replanning)
            return;

        if (pathFollower == null || !pathFollower.IsFollowing)
        {
            if (State != NavigationState.NoSafePath)
            {
                currentlyAvoidingObstacle = null;
                SetState(NavigationState.Normal);
            }
            return;
        }

        ThreatLevel currentThreat = threatAssessment != null ? threatAssessment.CurrentThreatLevel : ThreatLevel.None;

        if (State == NavigationState.Rerouting)
        {
            // Transition back to Normal cruising once clear of threat and cooldown expired
            if (currentThreat <= ThreatLevel.Advisory && Time.time - lastReplanTime >= replanCooldown)
            {
                currentlyAvoidingObstacle = null;
                SetState(NavigationState.Normal);
            }
        }
        else if (State == NavigationState.Normal)
        {
            if (currentThreat >= ThreatLevel.Advisory)
            {
                SetState(NavigationState.ThreatDetected);
            }
        }
        else if (State == NavigationState.ThreatDetected)
        {
            if (currentThreat == ThreatLevel.None)
            {
                SetState(NavigationState.Normal);
            }
        }
    }

    private void SetState(NavigationState newState)
    {
        if (State == newState)
            return;

        State = newState;
        OnStateChanged?.Invoke(State);
    }

    private void OnDrawGizmos()
    {
        if (!showGizmos)
            return;

        Vector3 uavPos = transform.position;
        Color stateColor = GetStateColor(State);

        // Draw State Indicator Sphere above UAV
        Gizmos.color = stateColor;
        Gizmos.DrawWireSphere(uavPos + Vector3.up * 1.5f, 0.4f);

        // Draw Marker at Last Replan Trigger Point
        if (replanCount > 0 && lastReplanPosition != Vector3.zero)
        {
            Gizmos.color = Color.magenta;
            Gizmos.DrawWireCube(lastReplanPosition, Vector3.one * 0.5f);
            Gizmos.DrawLine(lastReplanPosition, uavPos);
        }
    }

    private static Color GetStateColor(NavigationState state)
    {
        switch (state)
        {
            case NavigationState.Normal: return Color.green;
            case NavigationState.ThreatDetected: return Color.yellow;
            case NavigationState.Replanning: return Color.magenta;
            case NavigationState.Rerouting: return Color.cyan;
            case NavigationState.NoSafePath: return Color.red;
            default: return Color.white;
        }
    }
}
