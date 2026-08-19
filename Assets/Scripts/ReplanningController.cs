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
/// Categorizes the physical rationale and tactical mechanism chosen by the 3-axis evasion hierarchy.
/// </summary>
public enum TacticalDecisionReason
{
    None,
    VOPacingApplied,
    VerticalStepClimbed,
    VerticalRejectedCeilingExceeded,
    VerticalRejectedClimbTimeInfeasible,
    VerticalRejectedMultiThreatConflict,
    SpatialDetourExecuted,
    NoSafePathHold
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
    public TacticalDecisionReason LatestDecisionReason => latestDecisionReason;

    // Reactive Events for External Systems
    public event Action<NavigationState> OnStateChanged;
    public event Action<List<Node>> OnPathReplanned;
    public event Action OnNoSafePathFound;
    public event Action<float, float> OnSpeedPacingApplied;
    public event Action<float> OnVerticalEvasionExecuted;
    public event Action<TacticalDecisionReason, string> OnTacticalDecisionMade;

    [Header("Altitude Recovery Configuration")]
    [SerializeField] private float nominalAltitude = 1.0f;

    public float NominalAltitude
    {
        get => nominalAltitude;
        set => nominalAltitude = Mathf.Max(0.5f, value);
    }

    private PathFollower pathFollower;
    private ThreatAssessment threatAssessment;
    private UAVPerception perception;
    private Pathfinding pathfinding;

    private float lastReplanTime = -10f;
    private int replanCount = 0;
    private int speedPacingCount = 0;
    private int verticalEvasionCount = 0;
    private int spatialReplanCount = 0;
    private int peakSimultaneousThreats = 0;
    private int voPacingDecisions = 0;
    private int verticalStepClimbs = 0;
    private int verticalCeilingRejections = 0;
    private int verticalClimbTimeRejections = 0;
    private int verticalMultiThreatRejections = 0;
    private int safeHoldDecisions = 0;
    private TacticalDecisionReason latestDecisionReason = TacticalDecisionReason.None;
    private Vector3 lastReplanPosition;
    private GameObject currentlyAvoidingObstacle = null;
    private readonly HashSet<GameObject> currentlyAvoidingObstacles = new HashSet<GameObject>();

    public GameObject CurrentlyAvoidingObstacle => currentlyAvoidingObstacle;
    public IReadOnlyCollection<GameObject> CurrentlyAvoidingObstacles => currentlyAvoidingObstacles;
    public int SpeedPacingCount => speedPacingCount;
    public int VerticalEvasionCount => verticalEvasionCount;
    public int SpatialReplanCount => spatialReplanCount;
    public int PeakSimultaneousThreats => peakSimultaneousThreats;
    public int VoPacingDecisions => voPacingDecisions;
    public int VerticalStepClimbs => verticalStepClimbs;
    public int VerticalCeilingRejections => verticalCeilingRejections;
    public int VerticalClimbTimeRejections => verticalClimbTimeRejections;
    public int VerticalMultiThreatRejections => verticalMultiThreatRejections;
    public int SafeHoldDecisions => safeHoldDecisions;

    private void Awake()
    {
        pathFollower = GetComponent<PathFollower>();
        threatAssessment = GetComponent<ThreatAssessment>();
        perception = GetComponent<UAVPerception>();
        pathfinding = FindFirstObjectByType<Pathfinding>();
        stateProvider = GetComponent<IEstimatedStateProvider>();
    }

    private Vector3 GetEstimatedPosition()
    {
        return (stateProvider != null && stateProvider.IsEstimatorReady)
            ? stateProvider.CurrentState.Position
            : transform.position;
    }

    private float GetEstimatedAltitude()
    {
        return (stateProvider != null && stateProvider.IsEstimatorReady)
            ? stateProvider.CurrentState.Position.y
            : transform.position.y;
    }

    private void Start()
    {
        if (threatAssessment != null)
        {
            threatAssessment.OnThreatEvaluated += HandleThreatEvaluated;
            threatAssessment.OnCriticalThreatDetected += HandleCriticalThreat;
        }

        PathfindingRuntimeSetup setup = FindFirstObjectByType<PathfindingRuntimeSetup>();
        if (setup != null && setup.ScenarioConfig != null)
        {
            nominalAltitude = setup.ScenarioConfig.nominalFlightAltitude;
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
            currentlyAvoidingObstacles.Contains(report.ThreateningObstacle.GameObject))
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
                currentlyAvoidingObstacles.Contains(report.ThreateningObstacle.GameObject))
            {
                return;
            }

            TryExecuteReplan($"Threat Level {report.ThreatLevel} within TTC threshold ({report.TimeToCollision:F2}s)", report);
        }
    }

    /// <summary>
    /// Attempts to dynamically replan the UAV's flight route from its current position to the mission target
    /// using the 3-Axis Tactical Evasion Hierarchy:
    /// Stage 1 (VO Speed Pacing) -> Stage 2 (Vertical Step Climb) -> Stage 3 (Spatial Re-A*) -> Stage 4 (Fail-Safe).
    /// </summary>
    public bool TryExecuteReplan(string triggerReason, ThreatReport report)
    {
        if (pathFollower == null || !pathFollower.IsFollowing)
            return false;

        if (State == NavigationState.Replanning)
            return false;

        if (State == NavigationState.Rerouting &&
            report.ThreateningObstacle.GameObject != null &&
            currentlyAvoidingObstacles.Contains(report.ThreateningObstacle.GameObject) &&
            (threatAssessment == null || threatAssessment.ActiveThreatReports == null || threatAssessment.ActiveThreatReports.Count <= 1))
        {
            return false;
        }

        if (Time.time - lastReplanTime < replanCooldown)
            return false;

        int currentActiveCount = threatAssessment != null && threatAssessment.ActiveThreatReports != null
            ? threatAssessment.ActiveThreatReports.Count
            : 0;
        if (currentActiveCount > peakSimultaneousThreats)
        {
            peakSimultaneousThreats = currentActiveCount;
        }

        // Stage 1: Try Velocity Obstacle (VO) Tactical Speed Modulation for moving dynamic threats
        if (report.ThreateningObstacle.IsDynamic && TryTacticalSpeedModulation(report, out float speedRatio))
        {
            float overrideDuration = float.IsFinite(report.TimeToCollision) ? report.TimeToCollision + 1.5f : 4.0f;
            pathFollower.ApplyTacticalSpeedOverride(speedRatio, overrideDuration);
            lastReplanPosition = GetEstimatedPosition();
            lastReplanTime = Time.time;
            replanCount++;
            speedPacingCount++;
            voPacingDecisions++;
            latestDecisionReason = TacticalDecisionReason.VOPacingApplied;
            currentlyAvoidingObstacle = report.ThreateningObstacle.GameObject;
            currentlyAvoidingObstacles.Clear();
            if (currentlyAvoidingObstacle != null) currentlyAvoidingObstacles.Add(currentlyAvoidingObstacle);

            if (threatAssessment != null && threatAssessment.ActiveThreatReports != null)
            {
                for (int i = 0; i < threatAssessment.ActiveThreatReports.Count; i++)
                {
                    if (threatAssessment.ActiveThreatReports[i].ThreateningObstacle.GameObject != null)
                    {
                        currentlyAvoidingObstacles.Add(threatAssessment.ActiveThreatReports[i].ThreateningObstacle.GameObject);
                    }
                }
            }

            SetState(NavigationState.Rerouting);
            OnSpeedPacingApplied?.Invoke(speedRatio, overrideDuration);
            OnTacticalDecisionMade?.Invoke(latestDecisionReason, $"VO Speed Pacing applied at {speedRatio:P0} cruise speed for {overrideDuration:F1}s");

            if (logReplanningEvents)
            {
                string obsName = report.ThreateningObstacle.GameObject != null
                    ? report.ThreateningObstacle.GameObject.name
                    : "Dynamic Obstacle";

                Debug.Log(
                    $"<color=#00FF99><b>[ReplanningController] STAGE 1: VO TACTICAL SPEED PACING APPLIED (Replan #{replanCount} | Pacing #{speedPacingCount})</b></color>\n" +
                    $"  • Obstacle: {obsName} (Dynamic)\n" +
                    $"  • Speed Override: {speedRatio:P0} of cruise speed for {overrideDuration:F1}s\n" +
                    $"  • TTC: {report.TimeToCollision:F2}s | Distance: {report.DistanceToCollision:F2}m\n" +
                    $"  • Evasion: Pacing cleared VO collision cone without spatial path detour.");
            }

            return true;
        }

        // Stage 2: Try Tactical Vertical Step Climb / Descent
        if (TryTacticalVerticalEvasion(report, out float targetAltitude, out TacticalDecisionReason verticalRejectionReason))
        {
            pathFollower.SetTargetAltitude(targetAltitude);
            lastReplanPosition = GetEstimatedPosition();
            lastReplanTime = Time.time;
            replanCount++;
            verticalEvasionCount++;
            verticalStepClimbs++;
            latestDecisionReason = TacticalDecisionReason.VerticalStepClimbed;
            currentlyAvoidingObstacle = report.ThreateningObstacle.GameObject;
            currentlyAvoidingObstacles.Clear();
            if (currentlyAvoidingObstacle != null) currentlyAvoidingObstacles.Add(currentlyAvoidingObstacle);

            if (threatAssessment != null && threatAssessment.ActiveThreatReports != null)
            {
                for (int i = 0; i < threatAssessment.ActiveThreatReports.Count; i++)
                {
                    if (threatAssessment.ActiveThreatReports[i].ThreateningObstacle.GameObject != null)
                    {
                        currentlyAvoidingObstacles.Add(threatAssessment.ActiveThreatReports[i].ThreateningObstacle.GameObject);
                    }
                }
            }

            SetState(NavigationState.Rerouting);
            OnVerticalEvasionExecuted?.Invoke(targetAltitude);
            OnTacticalDecisionMade?.Invoke(latestDecisionReason, $"Vertical Step Climb commanded to {targetAltitude:F2}m");

            if (logReplanningEvents)
            {
                string obsName = report.ThreateningObstacle.GameObject != null
                    ? report.ThreateningObstacle.GameObject.name
                    : "Obstacle";

                Debug.Log(
                    $"<color=#33CCFF><b>[ReplanningController] STAGE 2: VERTICAL STEP CLIMB APPLIED (Replan #{replanCount} | Vertical #{verticalEvasionCount})</b></color>\n" +
                    $"  • Obstacle: {obsName}\n" +
                    $"  • Target Altitude: {targetAltitude:F2}m (Current: {GetEstimatedAltitude():F2}m)\n" +
                    $"  • TTC: {report.TimeToCollision:F2}s | Distance: {report.DistanceToCollision:F2}m\n" +
                    $"  • Evasion: Vertical step-climb cleared 3D obstacle volume without spatial detour.");
            }

            return true;
        }

        // Record why Stage 2 was rejected before escalating to Stage 3
        if (verticalRejectionReason != TacticalDecisionReason.None)
        {
            switch (verticalRejectionReason)
            {
                case TacticalDecisionReason.VerticalRejectedCeilingExceeded:
                    verticalCeilingRejections++;
                    break;
                case TacticalDecisionReason.VerticalRejectedClimbTimeInfeasible:
                    verticalClimbTimeRejections++;
                    break;
                case TacticalDecisionReason.VerticalRejectedMultiThreatConflict:
                    verticalMultiThreatRejections++;
                    break;
            }
            latestDecisionReason = verticalRejectionReason;
            OnTacticalDecisionMade?.Invoke(latestDecisionReason, $"Vertical Evasion rejected: {verticalRejectionReason}");
        }

        // Stage 3: Fall back to Spatial A* Dynamic Replanning
        if (pathfinding == null || pathfinding.targetTransform == null)
        {
            pathfinding = FindFirstObjectByType<Pathfinding>();
            if (pathfinding == null || pathfinding.targetTransform == null)
                return false;
        }

        SetState(NavigationState.Replanning);

        Vector3 currentPos = GetEstimatedPosition();
        Vector3 targetPos = pathfinding.targetTransform.position;
        lastReplanPosition = currentPos;
        lastReplanTime = Time.time;
        replanCount++;
        spatialReplanCount++;
        currentlyAvoidingObstacle = report.ThreateningObstacle.GameObject;
        currentlyAvoidingObstacles.Clear();
        if (currentlyAvoidingObstacle != null) currentlyAvoidingObstacles.Add(currentlyAvoidingObstacle);

        // Collect active dynamic hazards to avoid simultaneously (bounded to top 5)
        List<DynamicHazard> compoundHazards = new List<DynamicHazard>(8);
        float baseBuffer = threatAssessment != null ? threatAssessment.SafetyRadius + 1.2f : 2.2f;

        if (threatAssessment != null && threatAssessment.ActiveThreatReports != null && threatAssessment.ActiveThreatReports.Count > 0)
        {
            for (int i = 0; i < threatAssessment.ActiveThreatReports.Count && compoundHazards.Count < 5; i++)
            {
                ThreatReport r = threatAssessment.ActiveThreatReports[i];
                if (r.ThreateningObstacle.GameObject != null)
                {
                    currentlyAvoidingObstacles.Add(r.ThreateningObstacle.GameObject);
                    float projHorizon = (r.ThreateningObstacle.IsDynamic && float.IsFinite(r.TimeToCollision) && r.TimeToCollision > 0f)
                        ? Mathf.Min(3.0f, r.TimeToCollision + 0.5f)
                        : 0f;

                    compoundHazards.Add(new DynamicHazard(
                        r.ThreateningObstacle.WorldPosition,
                        baseBuffer,
                        r.ThreateningObstacle.Velocity,
                        r.ThreateningObstacle.IsDynamic,
                        projHorizon));
                }
            }
        }

        // Ensure primary report obstacle is included if not already present
        if (report.ThreateningObstacle.GameObject != null)
        {
            bool alreadyInList = false;
            for (int i = 0; i < compoundHazards.Count; i++)
            {
                if (Vector3.Distance(compoundHazards[i].Position, report.ThreateningObstacle.WorldPosition) < 0.1f)
                {
                    alreadyInList = true;
                    break;
                }
            }
            if (!alreadyInList)
            {
                float primaryProjHorizon = (report.ThreateningObstacle.IsDynamic && float.IsFinite(report.TimeToCollision) && report.TimeToCollision > 0f)
                    ? Mathf.Min(3.0f, report.TimeToCollision + 0.5f)
                    : 0f;

                compoundHazards.Insert(0, new DynamicHazard(
                    report.ThreateningObstacle.WorldPosition,
                    baseBuffer,
                    report.ThreateningObstacle.Velocity,
                    report.ThreateningObstacle.IsDynamic,
                    primaryProjHorizon));
            }
        }

        // Execute A* search avoiding all compound hazard footprints simultaneously
        pathfinding.FindPath(currentPos, targetPos, compoundHazards);

        if (pathfinding.path != null && pathfinding.path.Count > 0)
        {
            // Transition active flight path to the newly calculated safe detour
            pathFollower.StartFollowing(pathfinding.path);
            SetState(NavigationState.Rerouting);
            latestDecisionReason = TacticalDecisionReason.SpatialDetourExecuted;
            OnTacticalDecisionMade?.Invoke(latestDecisionReason, $"Spatial A* Detour executed with {pathfinding.path.Count} waypoints");

            if (logReplanningEvents)
            {
                string obsName = report.ThreateningObstacle.GameObject != null
                    ? report.ThreateningObstacle.GameObject.name
                    : "Obstacle";

                Debug.Log(
                    $"<color=#00FFFF><b>[ReplanningController] DYNAMIC REPLAN SUCCESS (Replan #{replanCount} | Spatial #{spatialReplanCount})</b></color>\n" +
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
            safeHoldDecisions++;
            latestDecisionReason = TacticalDecisionReason.NoSafePathHold;
            OnTacticalDecisionMade?.Invoke(latestDecisionReason, "Emergency Safe Hold commanded (No safe path found)");

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
                currentlyAvoidingObstacles.Clear();
                SetState(NavigationState.Normal);
            }
            return;
        }

        ThreatLevel currentThreat = threatAssessment != null ? threatAssessment.CurrentThreatLevel : ThreatLevel.None;

        bool hasActiveThreats = false;
        if (threatAssessment != null)
        {
            if (threatAssessment.ActiveThreatReports != null && threatAssessment.ActiveThreatReports.Count > 0)
            {
                for (int i = 0; i < threatAssessment.ActiveThreatReports.Count; i++)
                {
                    if (threatAssessment.ActiveThreatReports[i].ThreatLevel >= ThreatLevel.Warning)
                    {
                        hasActiveThreats = true;
                        break;
                    }
                }
            }
            else if (currentThreat >= ThreatLevel.Warning)
            {
                hasActiveThreats = true;
            }
        }

        if (State == NavigationState.Rerouting)
        {
            // Transition back to Normal cruising only once ALL active threats have cleared and cooldown expired
            if (!hasActiveThreats && currentThreat <= ThreatLevel.Advisory && Time.time - lastReplanTime >= replanCooldown)
            {
                currentlyAvoidingObstacle = null;
                currentlyAvoidingObstacles.Clear();
                SetState(NavigationState.Normal);
                RecoverNominalAltitude();
            }
        }
        else if (State == NavigationState.Normal)
        {
            if (currentThreat >= ThreatLevel.Advisory)
            {
                SetState(NavigationState.ThreatDetected);
            }
            else if (!hasActiveThreats && Time.time - lastReplanTime >= replanCooldown)
            {
                RecoverNominalAltitude();
            }
        }
        else if (State == NavigationState.ThreatDetected)
        {
            if (currentThreat == ThreatLevel.None && !hasActiveThreats)
            {
                SetState(NavigationState.Normal);
                if (Time.time - lastReplanTime >= replanCooldown)
                {
                    RecoverNominalAltitude();
                }
            }
        }
    }

    /// <summary>
    /// Smoothly commands target altitude recovery toward nominal flight altitude
    /// once the UAV has safely cleared all relevant obstacle volumes.
    /// </summary>
    public void RecoverNominalAltitude()
    {
        if (pathFollower == null)
            return;

        if (pathFollower.TargetAltitude > nominalAltitude + 0.05f)
        {
            float recoveryAlt = Mathf.Clamp(nominalAltitude, pathFollower.MinFlightAltitude, pathFollower.MaxFlightAltitude);
            pathFollower.SetTargetAltitude(recoveryAlt);
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

    /// <summary>
    /// Checks whether tactical speed modulation can safely clear ALL active dynamic obstacles' Velocity Obstacle (VO) cones.
    /// A candidate speed is accepted if and only if it is outside the VO cone of every relevant dynamic threat.
    /// </summary>
    public bool TryTacticalSpeedModulation(ThreatReport primaryReport, out float recommendedSpeedRatio)
    {
        recommendedSpeedRatio = 1.0f;

        if (pathFollower == null || !pathFollower.IsFollowing)
            return false;

        Vector3 uavPos = GetEstimatedPosition();
        float combinedRadius = threatAssessment != null ? threatAssessment.SafetyRadius + 0.5f : 1.5f;

        // 1. Gather all active dynamic threats to evaluate (bounded to top 5)
        List<ThreatReport> dynamicThreats = new List<ThreatReport>(8);

        if (threatAssessment != null && threatAssessment.ActiveThreatReports != null && threatAssessment.ActiveThreatReports.Count > 0)
        {
            for (int i = 0; i < threatAssessment.ActiveThreatReports.Count && dynamicThreats.Count < 5; i++)
            {
                ThreatReport r = threatAssessment.ActiveThreatReports[i];
                if (r.ThreateningObstacle.IsDynamic &&
                    r.ThreateningObstacle.GameObject != null &&
                    r.ThreateningObstacle.Velocity.sqrMagnitude >= 0.04f)
                {
                    dynamicThreats.Add(r);
                }
            }
        }

        // If primaryReport is dynamic and not already included, ensure it is present
        if (primaryReport.ThreateningObstacle.IsDynamic &&
            primaryReport.ThreateningObstacle.GameObject != null &&
            primaryReport.ThreateningObstacle.Velocity.sqrMagnitude >= 0.04f)
        {
            bool alreadyInList = false;
            for (int i = 0; i < dynamicThreats.Count; i++)
            {
                if (dynamicThreats[i].ThreateningObstacle.GameObject == primaryReport.ThreateningObstacle.GameObject)
                {
                    alreadyInList = true;
                    break;
                }
            }
            if (!alreadyInList)
            {
                dynamicThreats.Insert(0, primaryReport);
            }
        }

        // If no dynamic moving threats exist, speed modulation cannot resolve static obstacles
        if (dynamicThreats.Count == 0)
            return false;

        // 2. Build Velocity Obstacle (VO) cones for all active dynamic threats
        List<VelocityObstacle> activeVOs = new List<VelocityObstacle>(dynamicThreats.Count);
        List<float> lookaheads = new List<float>(dynamicThreats.Count);

        for (int i = 0; i < dynamicThreats.Count; i++)
        {
            ThreatReport t = dynamicThreats[i];
            VelocityObstacle vo = CollisionPrediction.CalculateVelocityObstacle(
                uavPos,
                t.ThreateningObstacle.WorldPosition,
                t.ThreateningObstacle.Velocity,
                combinedRadius);

            if (vo.IsValid)
            {
                activeVOs.Add(vo);
                float lookahead = float.IsFinite(t.TimeToCollision) && t.TimeToCollision > 0f
                    ? Mathf.Min(8.0f, t.TimeToCollision + 2.0f)
                    : 8.0f;
                lookaheads.Add(lookahead);
            }
        }

        if (activeVOs.Count == 0)
            return false;

        // 3. Determine candidate velocity along flight direction
        Vector3 flightDir = (pathFollower.TargetWaypoint - uavPos).normalized;
        flightDir.y = 0f;
        if (flightDir.sqrMagnitude < 0.001f)
            return false;

        float cruiseSpeed = pathFollower.MoveSpeed;
        float minAllowedSpeed = 0.5f;

        // Test candidate reduced speed ratios: 50%, 65%, 75%
        float[] candidateRatios = new float[] { 0.50f, 0.65f, 0.75f };

        for (int rIdx = 0; rIdx < candidateRatios.Length; rIdx++)
        {
            float ratio = candidateRatios[rIdx];
            float testSpeed = Mathf.Max(minAllowedSpeed, cruiseSpeed * ratio);
            Vector3 testVel = flightDir * testSpeed;

            // 4. Test candidate velocity against ALL active VO cones (Unanimous Safety Requirement)
            bool isCandidateSafeAgainstAll = true;

            for (int vIdx = 0; vIdx < activeVOs.Count; vIdx++)
            {
                if (activeVOs[vIdx].ContainsVelocity(testVel, lookaheads[vIdx]))
                {
                    isCandidateSafeAgainstAll = false;
                    break; // Unsafe against this threat; reject candidate
                }
            }

            if (isCandidateSafeAgainstAll)
            {
                recommendedSpeedRatio = ratio;
                return true;
            }
        }
    /// <summary>
    /// Evaluates whether a tactical vertical step climb or descent can safely clear all active threats.
    /// Checks flight ceiling/floor bounds, climb-time feasibility at CPA, and multi-threat clearance.
    /// </summary>
    public bool TryTacticalVerticalEvasion(ThreatReport primaryReport, out float targetAltitude)
    {
        return TryTacticalVerticalEvasion(primaryReport, out targetAltitude, out _);
    }

    /// <summary>
    /// Evaluates whether a tactical vertical step climb or descent can safely clear all active threats,
    /// returning the specific TacticalDecisionReason failure code if infeasible.
    /// </summary>
    public bool TryTacticalVerticalEvasion(ThreatReport primaryReport, out float targetAltitude, out TacticalDecisionReason failureReason)
    {
        failureReason = TacticalDecisionReason.None;
        targetAltitude = GetEstimatedAltitude();

        if (pathFollower == null || !pathFollower.IsFollowing)
            return false;

        float currentAltitude = GetEstimatedAltitude();
        float minAltitude = pathFollower.MinFlightAltitude;
        float maxAltitude = pathFollower.MaxFlightAltitude;
        float maxClimbRate = pathFollower.MaxClimbRate;
        float maxDescentRate = pathFollower.MaxDescentRate;
        float verticalSafetyMargin = threatAssessment != null ? threatAssessment.VerticalSafetyMargin : 0.5f;

        // 1. Determine primary obstacle top ceiling
        float primaryTopY = primaryReport.ThreateningObstacle.GameObject != null && primaryReport.ThreateningObstacle.Collider != null
            ? primaryReport.ThreateningObstacle.Collider.bounds.max.y
            : primaryReport.ThreateningObstacle.WorldPosition.y + 0.5f;

        float candidateClimbAltitude = primaryTopY + verticalSafetyMargin;

        // 2. Flight Ceiling Check
        if (candidateClimbAltitude > maxAltitude)
        {
            failureReason = TacticalDecisionReason.VerticalRejectedCeilingExceeded;
        }
        else
        {
            // 3. Climb-Time Feasibility Check at CPA
            float tAvail = float.IsFinite(primaryReport.TimeToCollision) && primaryReport.TimeToCollision > 0f
                ? primaryReport.TimeToCollision
                : (float.IsFinite(primaryReport.DistanceToCollision) && primaryReport.DistanceToCollision > 0f && pathFollower.MoveSpeed > 0.1f
                    ? primaryReport.DistanceToCollision / pathFollower.MoveSpeed
                    : 2.0f);

            float reqClimb = candidateClimbAltitude - currentAltitude;

            // If already at or above altitude, or can achieve climb within tAvail
            bool isClimbFeasible = false;
            if (reqClimb <= 0f)
            {
                isClimbFeasible = true;
            }
            else
            {
                // Maximum climb distance in tAvail
                float achievableClimb = maxClimbRate * Mathf.Max(0f, tAvail - 0.2f);
                if (achievableClimb >= reqClimb || (maxClimbRate * tAvail >= reqClimb))
                {
                    isClimbFeasible = true;
                }
            }

            if (!isClimbFeasible)
            {
                failureReason = TacticalDecisionReason.VerticalRejectedClimbTimeInfeasible;
            }
            else
            {
                // 4. Multi-Threat Clearance Check: candidate altitude must clear all active threats
                bool clearsAllThreats = true;

                if (threatAssessment != null && threatAssessment.ActiveThreatReports != null && threatAssessment.ActiveThreatReports.Count > 0)
                {
                    for (int i = 0; i < threatAssessment.ActiveThreatReports.Count; i++)
                    {
                        ThreatReport r = threatAssessment.ActiveThreatReports[i];
                        float obsTop = r.ThreateningObstacle.GameObject != null && r.ThreateningObstacle.Collider != null
                            ? r.ThreateningObstacle.Collider.bounds.max.y
                            : r.ThreateningObstacle.WorldPosition.y + 0.5f;

                        if (candidateClimbAltitude < obsTop + verticalSafetyMargin)
                        {
                            // Another active threat is too tall for this candidate altitude
                            clearsAllThreats = false;
                            break;
                        }
                    }
                }

                if (!clearsAllThreats)
                {
                    failureReason = TacticalDecisionReason.VerticalRejectedMultiThreatConflict;
                }
                else
                {
                    targetAltitude = Mathf.Clamp(candidateClimbAltitude, minAltitude, maxAltitude);
                    return true;
                }
            }
        }

        // 5. If climb is not feasible or blocked, evaluate Descent (for high overhangs/bridges)
        float primaryBottomY = primaryReport.ThreateningObstacle.GameObject != null && primaryReport.ThreateningObstacle.Collider != null
            ? primaryReport.ThreateningObstacle.Collider.bounds.min.y
            : Mathf.Max(0f, primaryReport.ThreateningObstacle.WorldPosition.y - 0.5f);

        float candidateDescentAltitude = primaryBottomY - verticalSafetyMargin;
        if (candidateDescentAltitude >= minAltitude)
        {
            float tAvail = float.IsFinite(primaryReport.TimeToCollision) && primaryReport.TimeToCollision > 0f
                ? primaryReport.TimeToCollision
                : 2.0f;

            float reqDescent = currentAltitude - candidateDescentAltitude;
            if (reqDescent <= 0f || (maxDescentRate * tAvail >= reqDescent))
            {
                targetAltitude = Mathf.Clamp(candidateDescentAltitude, minAltitude, maxAltitude);
                failureReason = TacticalDecisionReason.None;
                return true;
            }
        }

        return false;
    }
}
