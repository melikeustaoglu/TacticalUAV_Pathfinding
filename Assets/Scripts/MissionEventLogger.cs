using System;
using System.Collections.Generic;
using System.Text;
using UnityEngine;

/// <summary>
/// Serializable record representing a discrete tactical or mission lifecycle event.
/// </summary>
[Serializable]
public struct MissionEventRecord
{
    public float Timestamp;
    public string EventType;
    public string Description;

    public MissionEventRecord(float timestamp, string eventType, string description)
    {
        Timestamp = timestamp;
        EventType = eventType;
        Description = description;
    }

    public override string ToString()
    {
        return $"{Timestamp:00.00}s | {EventType} | {Description}";
    }
}

/// <summary>
/// Chronological Tactical Mission Event Logger.
/// Observes lifecycle, navigation, perception, and threat systems to maintain
/// an immutable ordered timeline of mission execution events.
/// </summary>
public class MissionEventLogger : MonoBehaviour
{
    [Header("Logging Configuration")]
    [SerializeField] private bool logToConsoleOnComplete = true;

    private readonly List<MissionEventRecord> events = new List<MissionEventRecord>();
    public IReadOnlyList<MissionEventRecord> Events => events;

    private MissionManager missionManager;
    private PathFollower pathFollower;
    private ReplanningController replanningController;
    private ThreatAssessment threatAssessment;
    private UAVPerception perception;

    private ThreatLevel lastThreatLevel = ThreatLevel.None;
    private GameObject lastObservedObstacle = null;
    private bool inObstacleField = false;

    private void Awake()
    {
        missionManager = GetComponent<MissionManager>() ?? FindFirstObjectByType<MissionManager>();
        pathFollower = GetComponent<PathFollower>() ?? FindFirstObjectByType<PathFollower>();
        replanningController = GetComponent<ReplanningController>() ?? FindFirstObjectByType<ReplanningController>();
        threatAssessment = GetComponent<ThreatAssessment>() ?? FindFirstObjectByType<ThreatAssessment>();
        perception = GetComponent<UAVPerception>() ?? FindFirstObjectByType<UAVPerception>();

        // Record initial state entry
        RecordEvent("MISSION_PENDING", "Mission initialized; awaiting departure");
    }

    private void OnEnable()
    {
        if (missionManager == null)
        {
            missionManager = GetComponent<MissionManager>() ?? FindFirstObjectByType<MissionManager>();
        }

        if (missionManager != null)
        {
            missionManager.OnMissionStateChanged += HandleMissionStateChanged;
            missionManager.OnMissionCompleted += HandleMissionCompleted;
        }

        if (pathFollower != null)
        {
            pathFollower.OnDestinationReached += HandleDestinationReached;
        }

        if (replanningController != null)
        {
            replanningController.OnPathReplanned += HandlePathReplanned;
            replanningController.OnNoSafePathFound += HandleNoSafePathFound;
            replanningController.OnSpeedPacingApplied += HandleSpeedPacingApplied;
            replanningController.OnVerticalEvasionExecuted += HandleVerticalEvasionExecuted;
            replanningController.OnTacticalDecisionMade += HandleTacticalDecisionMade;
        }

        if (threatAssessment != null)
        {
            threatAssessment.OnThreatEvaluated += HandleThreatEvaluated;
            threatAssessment.OnCriticalThreatDetected += HandleCriticalThreatDetected;
        }

        if (perception != null)
        {
            perception.OnObstacleDetected += HandleObstacleDetected;
            perception.OnObstaclesCleared += HandleObstaclesCleared;
        }
    }

    private void OnDisable()
    {
        if (missionManager != null)
        {
            missionManager.OnMissionStateChanged -= HandleMissionStateChanged;
            missionManager.OnMissionCompleted -= HandleMissionCompleted;
        }

        if (pathFollower != null)
        {
            pathFollower.OnDestinationReached -= HandleDestinationReached;
        }

        if (replanningController != null)
        {
            replanningController.OnPathReplanned -= HandlePathReplanned;
            replanningController.OnNoSafePathFound -= HandleNoSafePathFound;
            replanningController.OnSpeedPacingApplied -= HandleSpeedPacingApplied;
            replanningController.OnVerticalEvasionExecuted -= HandleVerticalEvasionExecuted;
            replanningController.OnTacticalDecisionMade -= HandleTacticalDecisionMade;
        }

        if (threatAssessment != null)
        {
            threatAssessment.OnThreatEvaluated -= HandleThreatEvaluated;
            threatAssessment.OnCriticalThreatDetected -= HandleCriticalThreatDetected;
        }

        if (perception != null)
        {
            perception.OnObstacleDetected -= HandleObstacleDetected;
            perception.OnObstaclesCleared -= HandleObstaclesCleared;
        }
    }

    private float GetCurrentTimestamp()
    {
        return missionManager != null ? missionManager.TotalFlightTime : 0f;
    }

    private void RecordEvent(string eventType, string description)
    {
        float timestamp = GetCurrentTimestamp();
        MissionEventRecord record = new MissionEventRecord(timestamp, eventType, description);
        events.Add(record);
    }

    private void HandleMissionStateChanged(MissionState newState)
    {
        switch (newState)
        {
            case MissionState.Navigating:
                RecordEvent("MISSION_NAVIGATING", "UAV underway along verified corridor");
                break;

            case MissionState.Rerouting:
                RecordEvent("MISSION_REROUTING", "Active dynamic detour executing around threat");
                break;

            case MissionState.Completed:
                RecordEvent("MISSION_COMPLETED", "Mission objective achieved successfully");
                break;

            case MissionState.Failed:
                RecordEvent("MISSION_FAILED", "Mission terminated - path blocked or unreachable");
                break;
        }
    }

    private void HandleMissionCompleted(MissionResult result)
    {
        if (logToConsoleOnComplete)
        {
            PrintTimelineSummary();
        }
    }

    private void HandleDestinationReached()
    {
        RecordEvent("DESTINATION_REACHED", "Final target waypoint reached");
    }

    private void HandlePathReplanned(List<Node> newPath)
    {
        int waypointCount = newPath != null ? newPath.Count : 0;
        RecordEvent("PATH_REPLANNED", $"New dynamic detour generated with {waypointCount} waypoints");
    }

    private void HandleNoSafePathFound()
    {
        RecordEvent("NO_SAFE_PATH_FOUND", "A* search failed; no safe detour around obstacle");
    }

    private void HandleSpeedPacingApplied(float speedRatio, float duration)
    {
        RecordEvent("VO_SPEED_PACING_APPLIED",
            $"Tactical speed override applied: {speedRatio:P0} cruise speed for {duration:F1}s");
    }

    private void HandleVerticalEvasionExecuted(float targetAltitude)
    {
        RecordEvent("VERTICAL_EVASION_EXECUTED",
            $"Tactical step-climb commanded to altitude {targetAltitude:F2}m");
    }

    private void HandleTacticalDecisionMade(TacticalDecisionReason reason, string description)
    {
        switch (reason)
        {
            case TacticalDecisionReason.VerticalRejectedCeilingExceeded:
                RecordEvent("VERTICAL_REJECTED_CEILING_EXCEEDED", description);
                break;
            case TacticalDecisionReason.VerticalRejectedClimbTimeInfeasible:
                RecordEvent("VERTICAL_REJECTED_CLIMB_TIME_INFEASIBLE", description);
                break;
            case TacticalDecisionReason.VerticalRejectedMultiThreatConflict:
                RecordEvent("VERTICAL_REJECTED_MULTI_THREAT_CONFLICT", description);
                break;
            case TacticalDecisionReason.SpatialDetourExecuted:
                RecordEvent("SPATIAL_DETOUR_EXECUTED", description);
                break;
            case TacticalDecisionReason.NoSafePathHold:
                RecordEvent("NO_SAFE_PATH_HOLD", description);
                break;
        }
    }

    private void HandleCriticalThreatDetected(ThreatReport report)
    {
        string obstacleName = report.ThreateningObstacle.GameObject != null
            ? report.ThreateningObstacle.GameObject.name
            : "Obstacle";

        RecordEvent("CRITICAL_THREAT_DETECTED",
            $"Critical collision threat from {obstacleName} (TTC: {report.TimeToCollision:F2}s, Dist: {report.DistanceToCollision:F2}m)");
    }

    private void HandleThreatEvaluated(ThreatReport report)
    {
        if (report.ThreatLevel >= ThreatLevel.Warning && lastThreatLevel < ThreatLevel.Warning)
        {
            string obstacleName = report.ThreateningObstacle.GameObject != null
                ? report.ThreateningObstacle.GameObject.name
                : "Obstacle";

            RecordEvent("THREAT_WARNING",
                $"Threat level elevated to {report.ThreatLevel} by {obstacleName}");
        }
        else if (report.ThreatLevel == ThreatLevel.None && lastThreatLevel != ThreatLevel.None)
        {
            RecordEvent("THREAT_CLEARED", "Forward flight corridor cleared of active threats");
        }

        lastThreatLevel = report.ThreatLevel;
    }

    private void HandleObstacleDetected(DetectedObstacle obstacle)
    {
        GameObject obsGo = obstacle.GameObject;
        if (!inObstacleField || (obsGo != null && obsGo != lastObservedObstacle))
        {
            inObstacleField = true;
            lastObservedObstacle = obsGo;
            string name = obsGo != null ? obsGo.name : "Obstacle";
            RecordEvent("OBSTACLE_DETECTED", $"{name} entered sensor cone (Distance: {obstacle.Distance:F2}m)");
        }
    }

    private void HandleObstaclesCleared()
    {
        if (inObstacleField)
        {
            inObstacleField = false;
            lastObservedObstacle = null;
            RecordEvent("OBSTACLES_CLEARED", "Sensor cone clear of perceived obstacles");
        }
    }

    /// <summary>
    /// Outputs the full mission chronological event timeline to the Unity Console.
    /// </summary>
    public void PrintTimelineSummary()
    {
        StringBuilder sb = new StringBuilder();
        sb.AppendLine("[MissionEventLogger] Mission Timeline");
        for (int i = 0; i < events.Count; i++)
        {
            sb.AppendLine($"{events[i].Timestamp:00.00}s | {events[i].EventType,-24} | {events[i].Description}");
        }

        Debug.Log(sb.ToString().TrimEnd());
    }
}
