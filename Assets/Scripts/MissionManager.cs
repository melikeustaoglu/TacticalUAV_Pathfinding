using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// High-level lifecycle state of the UAV tactical mission.
/// </summary>
public enum MissionState
{
    Pending,     // Mission initialized; awaiting takeoff / first path execution
    Navigating,  // Cruising along planned mission route towards objective
    Rerouting,   // In-flight tactical detour actively executing around threats
    Completed,   // Mission objective successfully reached (Terminal State)
    Failed       // Mission failed due to unreachable target / no safe path (Terminal State)
}

/// <summary>
/// Comprehensive structured outcome report generated upon mission termination.
/// </summary>
[Serializable]
public struct MissionResult
{
    public bool IsSuccess;
    public MissionState FinalState;
    public float TotalFlightTime;
    public float TotalDistanceTraveled;
    public float PlannedPathDistance;
    public int TotalReplans;
    public int TotalThreatEncounters;
    public int CriticalThreatCount;
    public float MinimumClearanceObserved;
    public float PathEfficiency;

    public MissionResult(
        bool isSuccess,
        MissionState finalState,
        float totalFlightTime,
        float totalDistanceTraveled,
        float plannedPathDistance,
        int totalReplans,
        int totalThreatEncounters,
        int criticalThreatCount,
        float minimumClearanceObserved,
        float pathEfficiency)
    {
        IsSuccess = isSuccess;
        FinalState = finalState;
        TotalFlightTime = totalFlightTime;
        TotalDistanceTraveled = totalDistanceTraveled;
        PlannedPathDistance = plannedPathDistance;
        TotalReplans = totalReplans;
        TotalThreatEncounters = totalThreatEncounters;
        CriticalThreatCount = criticalThreatCount;
        MinimumClearanceObserved = minimumClearanceObserved;
        PathEfficiency = pathEfficiency;
    }
}

/// <summary>
/// High-level Mission Manager and Tactical Lifecycle Observer.
/// Observes underlying navigation, perception, and threat telemetry to track mission progress,
/// accumulate evaluation metrics, detect objective completion, protect terminal outcomes,
/// and dispatch structured mission lifecycle reports.
/// </summary>
[RequireComponent(typeof(PathFollower))]
[RequireComponent(typeof(ReplanningController))]
public class MissionManager : MonoBehaviour
{
    // Public Read-Only State & Telemetry
    public MissionState State { get; private set; } = MissionState.Pending;
    public MissionResult? Result { get; private set; }
    public MissionScore? Score { get; private set; }
    public bool IsActive => State == MissionState.Navigating || State == MissionState.Rerouting;

    public float TotalFlightTime => IsActive ? (Time.time - missionStartTime) : totalFlightTime;
    public float TotalDistanceTraveled => totalDistanceTraveled;
    public float PlannedPathDistance => plannedPathDistance;
    public int TotalReplans => replanningController != null ? replanningController.ReplanCount : 0;
    public int TotalThreatEncounters => totalThreatEncounters;
    public int CriticalThreatCount => criticalThreatCount;
    public float MinimumClearanceObserved => minimumClearanceObserved;
    public float PathEfficiency => totalDistanceTraveled > 0.0001f ? (plannedPathDistance / totalDistanceTraveled) : 0f;

    // Mission Events
    public event Action<MissionState> OnMissionStateChanged;
    public event Action<MissionResult> OnMissionCompleted;

    private PathFollower pathFollower;
    private ReplanningController replanningController;
    private ThreatAssessment threatAssessment;
    private UAVPerception perception;

    // Accumulated Metrics
    private float missionStartTime = -1f;
    private float totalFlightTime = 0f;
    private float totalDistanceTraveled = 0f;
    private float plannedPathDistance = 0f;
    private int totalThreatEncounters = 0;
    private int criticalThreatCount = 0;
    private float minimumClearanceObserved = float.PositiveInfinity;

    // Tracking helpers
    private Vector3 lastPosition;
    private bool hasLastPosition = false;
    private bool initialPathCaptured = false;
    private bool inThreatCondition = false;
    private GameObject lastThreatObstacle = null;

    private void Awake()
    {
        pathFollower = GetComponent<PathFollower>();
        replanningController = GetComponent<ReplanningController>();
        threatAssessment = GetComponent<ThreatAssessment>();
        perception = GetComponent<UAVPerception>();
    }

    private void OnEnable()
    {
        if (pathFollower != null)
        {
            pathFollower.OnDestinationReached += HandleDestinationReached;
        }

        if (replanningController != null)
        {
            replanningController.OnStateChanged += HandleNavigationStateChanged;
            replanningController.OnNoSafePathFound += HandleNoSafePath;
        }

        if (threatAssessment != null)
        {
            threatAssessment.OnThreatEvaluated += HandleThreatEvaluated;
            threatAssessment.OnCriticalThreatDetected += HandleCriticalThreatDetected;
        }
    }

    private void OnDisable()
    {
        if (pathFollower != null)
        {
            pathFollower.OnDestinationReached -= HandleDestinationReached;
        }

        if (replanningController != null)
        {
            replanningController.OnStateChanged -= HandleNavigationStateChanged;
            replanningController.OnNoSafePathFound -= HandleNoSafePath;
        }

        if (threatAssessment != null)
        {
            threatAssessment.OnThreatEvaluated -= HandleThreatEvaluated;
            threatAssessment.OnCriticalThreatDetected -= HandleCriticalThreatDetected;
        }
    }

    private void Update()
    {
        // Detect initial transition from Pending to Navigating once flight commences
        if (State == MissionState.Pending && pathFollower != null && pathFollower.IsFollowing)
        {
            CaptureInitialPlannedPath();
            SetState(MissionState.Navigating);
        }

        if (IsActive)
        {
            // Accumulate physical displacement traveled by UAV
            Vector3 currentPos = transform.position;
            if (hasLastPosition)
            {
                float stepDist = Vector3.Distance(lastPosition, currentPos);
                totalDistanceTraveled += stepDist;
            }
            lastPosition = currentPos;
            hasLastPosition = true;

            // Monitor minimum observed obstacle clearance from perception telemetry
            if (perception != null && perception.HasObstacles)
            {
                float nearestDist = perception.NearestObstacle.Distance;
                if (nearestDist > 0f)
                {
                    minimumClearanceObserved = Mathf.Min(minimumClearanceObserved, nearestDist);
                }
            }
        }
    }

    private void CaptureInitialPlannedPath()
    {
        if (initialPathCaptured || pathFollower == null)
            return;

        IReadOnlyList<Node> currentPath = pathFollower.CurrentPath;
        if (currentPath != null && currentPath.Count > 0)
        {
            plannedPathDistance = CalculatePathDistance(transform.position, currentPath);
            initialPathCaptured = true;
        }
    }

    private static float CalculatePathDistance(Vector3 origin, IReadOnlyList<Node> nodes)
    {
        if (nodes == null || nodes.Count == 0)
            return 0f;

        float totalDist = 0f;
        Vector3 prev = origin;
        for (int i = 0; i < nodes.Count; i++)
        {
            Vector3 next = new Vector3(nodes[i].worldPosition.x, origin.y, nodes[i].worldPosition.z);
            totalDist += Vector3.Distance(prev, next);
            prev = next;
        }
        return totalDist;
    }

    private void HandleNavigationStateChanged(NavigationState navState)
    {
        if (IsTerminalState(State))
            return;

        switch (navState)
        {
            case NavigationState.Normal:
                if (State == MissionState.Rerouting || State == MissionState.Pending)
                {
                    SetState(MissionState.Navigating);
                }
                break;

            case NavigationState.Rerouting:
            case NavigationState.Replanning:
                if (State == MissionState.Navigating || State == MissionState.Pending)
                {
                    SetState(MissionState.Rerouting);
                }
                break;

            case NavigationState.NoSafePath:
                SetState(MissionState.Failed);
                break;

            case NavigationState.ThreatDetected:
                // Retains current high-level state (Navigating or Rerouting) while assessing
                break;
        }
    }

    private void HandleThreatEvaluated(ThreatReport report)
    {
        if (!IsActive)
            return;

        // Count discrete threat encounter episodes (Warning/Critical condition entry or obstacle switch)
        if (report.ThreatLevel >= ThreatLevel.Warning)
        {
            GameObject currentObstacle = report.ThreateningObstacle.GameObject;
            bool isNewThreat = !inThreatCondition;
            bool isDifferentObstacle = inThreatCondition && currentObstacle != null && currentObstacle != lastThreatObstacle;

            if (isNewThreat || isDifferentObstacle)
            {
                inThreatCondition = true;
                lastThreatObstacle = currentObstacle;
                totalThreatEncounters++;
            }
        }
        else if (report.ThreatLevel == ThreatLevel.None)
        {
            inThreatCondition = false;
            lastThreatObstacle = null;
        }
    }

    private void HandleCriticalThreatDetected(ThreatReport report)
    {
        if (!IsActive)
            return;

        criticalThreatCount++;
    }

    private void HandleDestinationReached()
    {
        SetState(MissionState.Completed);
    }

    private void HandleNoSafePath()
    {
        SetState(MissionState.Failed);
    }

    private void SetState(MissionState newState)
    {
        if (IsTerminalState(State))
            return;

        if (State == newState)
            return;

        MissionState previousState = State;
        State = newState;

        Debug.Log($"[MissionManager] Mission State: {previousState} → {newState}");
        OnMissionStateChanged?.Invoke(State);

        // Lifecycle timestamp updates
        if (previousState == MissionState.Pending && newState == MissionState.Navigating)
        {
            missionStartTime = Time.time;
            lastPosition = transform.position;
            hasLastPosition = true;
            CaptureInitialPlannedPath();
        }

        if (IsTerminalState(State))
        {
            totalFlightTime = missionStartTime >= 0f ? (Time.time - missionStartTime) : 0f;
            int replans = replanningController != null ? replanningController.ReplanCount : 0;
            float efficiency = totalDistanceTraveled > 0.0001f ? (plannedPathDistance / totalDistanceTraveled) : 0f;

            MissionResult result = new MissionResult(
                State == MissionState.Completed,
                State,
                totalFlightTime,
                totalDistanceTraveled,
                plannedPathDistance,
                replans,
                totalThreatEncounters,
                criticalThreatCount,
                minimumClearanceObserved,
                efficiency);

            Result = result;

            float nominalSpeed = pathFollower != null ? pathFollower.MoveSpeed : 1.5f;
            MissionScore score = MissionScore.Evaluate(result, nominalSpeed);
            Score = score;

            string clearanceStr = float.IsPositiveInfinity(minimumClearanceObserved)
                ? "N/A"
                : $"{minimumClearanceObserved:F2}m";

            Debug.Log(
                $"[MissionManager] Mission Result\n" +
                $"State={result.FinalState}\n" +
                $"Success={result.IsSuccess}\n" +
                $"FlightTime={result.TotalFlightTime:F2}s\n" +
                $"ActualDistance={result.TotalDistanceTraveled:F2}m\n" +
                $"PlannedDistance={result.PlannedPathDistance:F2}m\n" +
                $"Efficiency={result.PathEfficiency:F3}\n" +
                $"Replans={result.TotalReplans}\n" +
                $"Threats={result.TotalThreatEncounters}\n" +
                $"CriticalThreats={result.CriticalThreatCount}\n" +
                $"MinClearance={clearanceStr}\n" +
                $"Score={score.OverallScore:F1} (Safety:{score.SafetyScore:F1}, Eff:{score.EfficiencyScore:F1}, Nav:{score.NavigationScore:F1}, Threat:{score.ThreatManagementScore:F1}, Time:{score.TimeScore:F1})");

            OnMissionCompleted?.Invoke(result);
        }
    }

    private static bool IsTerminalState(MissionState state)
    {
        return state == MissionState.Completed || state == MissionState.Failed;
    }
}
