using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Categorizes the severity of an obstacle threat relative to the active flight path.
/// </summary>
public enum ThreatLevel
{
    None,        // Airspace clear, no obstacle in trajectory corridor
    Advisory,    // Obstacle perceived in FOV, but passing well clear of active path
    Warning,     // Obstacle approaching safety margin of flight path
    Critical     // Imminent collision predicted along active path trajectory
}

/// <summary>
/// Structured telemetry report containing detailed threat and collision forecast data.
/// </summary>
[Serializable]
public struct ThreatReport
{
    public ThreatLevel ThreatLevel { get; }
    public DetectedObstacle ThreateningObstacle { get; }
    public Vector3 EstimatedCollisionPoint { get; }
    public float DistanceToCollision { get; }
    public float TimeToCollision { get; }
    public int ObstructedWaypointIndex { get; }

    public ThreatReport(
        ThreatLevel threatLevel,
        DetectedObstacle threateningObstacle,
        Vector3 estimatedCollisionPoint,
        float distanceToCollision,
        float timeToCollision,
        int obstructedWaypointIndex)
    {
        ThreatLevel = threatLevel;
        ThreateningObstacle = threateningObstacle;
        EstimatedCollisionPoint = estimatedCollisionPoint;
        DistanceToCollision = distanceToCollision;
        TimeToCollision = timeToCollision;
        ObstructedWaypointIndex = obstructedWaypointIndex;
    }

    public static ThreatReport Clear => new ThreatReport(
        ThreatLevel.None,
        default,
        Vector3.zero,
        float.PositiveInfinity,
        float.PositiveInfinity,
        -1);
}

/// <summary>
/// Evaluates obstacles perceived by UAVPerception against the UAV's active flight trajectory
/// to forecast potential collisions and assign threat levels.
/// </summary>
[RequireComponent(typeof(UAVPerception))]
[RequireComponent(typeof(PathFollower))]
public class ThreatAssessment : MonoBehaviour
{
    [Header("Threat Envelope Settings")]
    [Tooltip("Radius of the UAV physical safety clearance envelope in meters.")]
    [SerializeField] private float safetyRadius = 1.0f;

    [Tooltip("Cross-track distance threshold for Warning classification in meters.")]
    [SerializeField] private float warningRadius = 2.2f;

    [Tooltip("Cross-track distance threshold for Advisory classification in meters.")]
    [SerializeField] private float advisoryRadius = 4.0f;

    [Tooltip("Forward trajectory lookahead forecasting time in seconds.")]
    [SerializeField] private float lookaheadTime = 4.5f;

    [Header("Logging & Diagnostics")]
    [SerializeField] private bool logCriticalThreats = true;

    [Header("Gizmo Visualization")]
    [SerializeField] private bool showGizmos = true;

    // Public Read-Only State for downstream subsystems (ReplanningController)
    public ThreatLevel CurrentThreatLevel => currentReport.ThreatLevel;
    public ThreatReport CurrentThreatReport => currentReport;
    public IReadOnlyList<ThreatReport> AllEvaluatedReports => allEvaluatedReports;
    public IReadOnlyList<ThreatReport> ActiveThreatReports => activeThreatReports;
    public float SafetyRadius => safetyRadius;
    public float LookaheadTime => lookaheadTime;

    // Reactive Events for modular subscribers
    public event Action<ThreatReport> OnThreatEvaluated;
    public event Action<ThreatReport> OnCriticalThreatDetected;

    private UAVPerception perception;
    private PathFollower pathFollower;
    private ThreatReport currentReport = ThreatReport.Clear;
    private readonly List<ThreatReport> allEvaluatedReports = new List<ThreatReport>();
    private readonly List<ThreatReport> activeThreatReports = new List<ThreatReport>();
    private ThreatLevel lastThreatLevel = ThreatLevel.None;
    private GameObject lastCriticalObstacle = null;
    private bool wasInCriticalState = false;

    private void Awake()
    {
        perception = GetComponent<UAVPerception>();
        pathFollower = GetComponent<PathFollower>();
    }

    private void Update()
    {
        EvaluateThreats();
    }

    /// <summary>
    /// Evaluates all perceived obstacles against the UAV's active flight trajectory.
    /// </summary>
    public void EvaluateThreats()
    {
        if (perception == null || pathFollower == null)
        {
            currentReport = ThreatReport.Clear;
            allEvaluatedReports.Clear();
            activeThreatReports.Clear();
            return;
        }

        allEvaluatedReports.Clear();
        activeThreatReports.Clear();

        IReadOnlyList<DetectedObstacle> obstacles = perception.DetectedObstacles;
        if (obstacles == null || obstacles.Count == 0)
        {
            currentReport = ThreatReport.Clear;
            NotifyThreatState();
            return;
        }

        Vector3 uavPos = transform.position;
        Vector3 uavVelocity = pathFollower.CurrentVelocity;
        float nominalSpeed = pathFollower.MoveSpeed;
        IReadOnlyList<Node> remainingWaypoints = pathFollower.RemainingPath;
        Vector3 targetWaypoint = pathFollower.TargetWaypoint;

        ThreatReport highestReport = ThreatReport.Clear;

        for (int i = 0; i < obstacles.Count; i++)
        {
            DetectedObstacle obs = obstacles[i];

            // Ignore obstacles positioned behind the UAV
            Vector3 toObs = obs.WorldPosition - uavPos;
            if (Vector3.Dot(toObs, transform.forward) < -0.1f)
                continue;

            CollisionPredictionResult prediction = CollisionPrediction.PredictPathCollision(
                uavPos,
                uavVelocity,
                nominalSpeed,
                remainingWaypoints,
                targetWaypoint,
                obs,
                safetyRadius,
                lookaheadTime);

            bool hasValidCollision = prediction.WillCollide &&
                                    float.IsFinite(prediction.DistanceToCollision) &&
                                    float.IsFinite(prediction.TimeToCollision) &&
                                    prediction.ObstructedWaypointIndex >= 0;

            ThreatLevel evaluatedLevel;

            if (hasValidCollision)
            {
                if (prediction.CrossTrackDistance <= safetyRadius || prediction.TimeToCollision <= lookaheadTime)
                {
                    evaluatedLevel = ThreatLevel.Critical;
                }
                else if (prediction.CrossTrackDistance <= warningRadius)
                {
                    evaluatedLevel = ThreatLevel.Warning;
                }
                else
                {
                    evaluatedLevel = ThreatLevel.Advisory;
                }
            }
            else
            {
                // No direct collision projected within lookahead window
                if (float.IsFinite(prediction.CrossTrackDistance) && prediction.CrossTrackDistance <= warningRadius)
                {
                    evaluatedLevel = ThreatLevel.Warning;
                }
                else if (float.IsFinite(prediction.CrossTrackDistance) && prediction.CrossTrackDistance <= advisoryRadius)
                {
                    evaluatedLevel = ThreatLevel.Advisory;
                }
                else
                {
                    evaluatedLevel = ThreatLevel.None;
                }
            }

            ThreatReport report = new ThreatReport(
                evaluatedLevel,
                obs,
                prediction.EstimatedCollisionPoint,
                prediction.DistanceToCollision,
                prediction.TimeToCollision,
                prediction.ObstructedWaypointIndex);

            allEvaluatedReports.Add(report);
            if (evaluatedLevel >= ThreatLevel.Warning)
            {
                activeThreatReports.Add(report);
            }

            // Keep the most severe valid threat for currentReport
            bool shouldUpdateReport = false;

            if (evaluatedLevel > highestReport.ThreatLevel)
            {
                shouldUpdateReport = true;
            }
            else if (evaluatedLevel == highestReport.ThreatLevel && evaluatedLevel != ThreatLevel.None)
            {
                bool currentIsFinite = float.IsFinite(prediction.DistanceToCollision);
                bool highestIsFinite = float.IsFinite(highestReport.DistanceToCollision);

                if (currentIsFinite && !highestIsFinite)
                {
                    shouldUpdateReport = true;
                }
                else if (currentIsFinite && highestIsFinite && prediction.DistanceToCollision < highestReport.DistanceToCollision)
                {
                    shouldUpdateReport = true;
                }
            }

            if (shouldUpdateReport)
            {
                highestReport = report;
            }
        }

        // Sort active threats with highest severity first
        if (activeThreatReports.Count > 1)
        {
            activeThreatReports.Sort((a, b) => b.ThreatLevel.CompareTo(a.ThreatLevel));
        }

        currentReport = highestReport;
        NotifyThreatState();
    }

    private void NotifyThreatState()
    {
        bool isCritical = currentReport.ThreatLevel == ThreatLevel.Critical;
        bool isValidCritical = isCritical &&
                               float.IsFinite(currentReport.TimeToCollision) &&
                               float.IsFinite(currentReport.DistanceToCollision) &&
                               currentReport.ObstructedWaypointIndex >= 0;

        if (isCritical && !isValidCritical)
        {
            currentReport = ThreatReport.Clear;
            isCritical = false;
        }

        if (isValidCritical)
        {
            GameObject currentObstacle = currentReport.ThreateningObstacle.GameObject;
            bool isNewCriticalEntry = !wasInCriticalState;
            bool isDifferentObstacle = wasInCriticalState && (currentObstacle != null && currentObstacle != lastCriticalObstacle);

            // Raise OnCriticalThreatDetected ONLY on state entry or obstacle change
            if (isNewCriticalEntry || isDifferentObstacle)
            {
                wasInCriticalState = true;
                lastCriticalObstacle = currentObstacle;

                string obsName = currentObstacle != null ? currentObstacle.name : "Obstacle";

                if (logCriticalThreats)
                {
                    Debug.LogWarning(
                        $"[ThreatAssessment] CRITICAL ENTERED | Obstacle={obsName} | " +
                        $"TTC={currentReport.TimeToCollision:F2}s | Dist={currentReport.DistanceToCollision:F2}m | " +
                        $"Wp={currentReport.ObstructedWaypointIndex}");
                }

                OnCriticalThreatDetected?.Invoke(currentReport);
            }
        }
        else
        {
            // Leaving the Critical threat condition
            if (wasInCriticalState)
            {
                string prevObsName = lastCriticalObstacle != null ? lastCriticalObstacle.name : "Obstacle";
                if (logCriticalThreats)
                {
                    Debug.Log($"[ThreatAssessment] CRITICAL CLEARED | Obstacle={prevObsName}");
                }

                wasInCriticalState = false;
                lastCriticalObstacle = null;
            }
        }

        if (currentReport.ThreatLevel != lastThreatLevel)
        {
            lastThreatLevel = currentReport.ThreatLevel;
        }

        OnThreatEvaluated?.Invoke(currentReport);
    }

    private void OnDrawGizmos()
    {
        if (!showGizmos || pathFollower == null)
            return;

        Vector3 uavPos = transform.position;
        IReadOnlyList<Node> remainingWaypoints = pathFollower.RemainingPath;

        Color threatColor = GetThreatColor(currentReport.ThreatLevel);

        // 1. Draw Safety Corridor along Active Waypoints
        if (remainingWaypoints != null && remainingWaypoints.Count > 0)
        {
            Gizmos.color = new Color(threatColor.r, threatColor.g, threatColor.b, 0.35f);
            Vector3 prev = uavPos;

            for (int i = 0; i < remainingWaypoints.Count; i++)
            {
                Vector3 next = new Vector3(
                    remainingWaypoints[i].worldPosition.x,
                    uavPos.y,
                    remainingWaypoints[i].worldPosition.z);

                Gizmos.DrawLine(prev, next);

                // Draw corridor boundary offsets
                Vector3 dir = (next - prev).normalized;
                Vector3 perp = new Vector3(-dir.z, 0f, dir.x) * safetyRadius;
                Gizmos.DrawLine(prev + perp, next + perp);
                Gizmos.DrawLine(prev - perp, next - perp);

                prev = next;
            }
        }

        // 2. Draw Threat Visual Markers
        if (currentReport.ThreatLevel >= ThreatLevel.Warning)
        {
            Gizmos.color = threatColor;

            if (currentReport.EstimatedCollisionPoint != Vector3.zero)
            {
                Gizmos.DrawWireSphere(currentReport.EstimatedCollisionPoint, safetyRadius);
                Gizmos.DrawLine(uavPos, currentReport.EstimatedCollisionPoint);
            }

            if (currentReport.ThreateningObstacle.WorldPosition != Vector3.zero)
            {
                Gizmos.DrawLine(currentReport.EstimatedCollisionPoint, currentReport.ThreateningObstacle.WorldPosition);
                Gizmos.DrawCube(currentReport.ThreateningObstacle.WorldPosition, Vector3.one * 0.3f);
            }
        }
    }

    private static Color GetThreatColor(ThreatLevel level)
    {
        switch (level)
        {
            case ThreatLevel.Critical: return Color.red;
            case ThreatLevel.Warning: return new Color(1f, 0.5f, 0f); // Orange
            case ThreatLevel.Advisory: return Color.yellow;
            default: return Color.green;
        }
    }
}
