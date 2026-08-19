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
/// Supports both legacy DetectedObstacle and multi-target TrackedTarget contracts.
/// </summary>
[Serializable]
public struct ThreatReport
{
    public ThreatLevel ThreatLevel { get; }
    public DetectedObstacle ThreateningObstacle { get; }
    public TrackedTarget ThreateningTrack { get; }
    public bool HasTrack => ThreateningTrack.IsValid;
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
        ThreateningTrack = TrackedTarget.Empty;
        EstimatedCollisionPoint = estimatedCollisionPoint;
        DistanceToCollision = distanceToCollision;
        TimeToCollision = timeToCollision;
        ObstructedWaypointIndex = obstructedWaypointIndex;
    }

    public ThreatReport(
        ThreatLevel threatLevel,
        TrackedTarget threateningTrack,
        Vector3 estimatedCollisionPoint,
        float distanceToCollision,
        float timeToCollision,
        int obstructedWaypointIndex)
    {
        ThreatLevel = threatLevel;
        ThreateningTrack = threateningTrack;
        ThreateningObstacle = new DetectedObstacle(
            null,
            null,
            threateningTrack.EstimatedPosition,
            Vector3.zero,
            Vector3.forward,
            distanceToCollision,
            0f,
            Vector3.up,
            threateningTrack.EstimatedVelocity,
            threateningTrack.Speed > 0.1f);
        EstimatedCollisionPoint = estimatedCollisionPoint;
        DistanceToCollision = distanceToCollision;
        TimeToCollision = timeToCollision;
        ObstructedWaypointIndex = obstructedWaypointIndex;
    }

    public static ThreatReport Clear => new ThreatReport(
        ThreatLevel.None,
        default(DetectedObstacle),
        Vector3.zero,
        float.PositiveInfinity,
        float.PositiveInfinity,
        -1);
}

/// <summary>
/// Evaluates obstacles and multi-target tracked targets against the UAV's active flight trajectory
/// to forecast potential collisions and assign threat levels.
/// Operates strictly on TrackedTarget estimates and EstimatedState with zero ground-truth dependencies.
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

    [Tooltip("Vertical clearance safety margin in meters above obstacle top.")]
    [SerializeField] private float verticalSafetyMargin = 0.5f;

    [Tooltip("Forward trajectory lookahead forecasting time in seconds.")]
    [SerializeField] private float lookaheadTime = 4.5f;

    [Header("Uncertainty-Aware Safety Parameters")]
    [Tooltip("Sigma multiplier for horizontal position uncertainty.")]
    [SerializeField] private float sigmaMultiplier = 2.0f;

    [Tooltip("Sigma multiplier for vertical altitude uncertainty.")]
    [SerializeField] private float verticalSigmaMultiplier = 2.0f;

    [Tooltip("Minimum allowable effective safety radius in meters.")]
    [SerializeField] private float minSafetyRadius = 1.0f;

    [Tooltip("Maximum allowable effective safety radius in meters.")]
    [SerializeField] private float maxSafetyRadius = 2.5f;

    [Tooltip("Minimum allowable effective vertical safety margin in meters.")]
    [SerializeField] private float minVerticalSafetyMargin = 0.5f;

    [Tooltip("Maximum allowable effective vertical safety margin in meters.")]
    [SerializeField] private float maxVerticalSafetyMargin = 1.5f;

    [Tooltip("Minimum allowable effective warning radius in meters.")]
    [SerializeField] private float minWarningRadius = 2.2f;

    [Tooltip("Maximum allowable effective warning radius in meters.")]
    [SerializeField] private float maxWarningRadius = 4.0f;

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
    public float WarningRadius => warningRadius;
    public float AdvisoryRadius => advisoryRadius;
    public float VerticalSafetyMargin
    {
        get => verticalSafetyMargin;
        set => verticalSafetyMargin = Mathf.Max(0f, value);
    }
    public float LookaheadTime => lookaheadTime;

    public float SigmaMultiplier
    {
        get => sigmaMultiplier;
        set => sigmaMultiplier = Mathf.Max(0f, value);
    }

    public float VerticalSigmaMultiplier
    {
        get => verticalSigmaMultiplier;
        set => verticalSigmaMultiplier = Mathf.Max(0f, value);
    }

    public float MinSafetyRadius => minSafetyRadius;
    public float MaxSafetyRadius => maxSafetyRadius;
    public float MinVerticalSafetyMargin => minVerticalSafetyMargin;
    public float MaxVerticalSafetyMargin => maxVerticalSafetyMargin;
    public float MinWarningRadius => minWarningRadius;
    public float MaxWarningRadius => maxWarningRadius;

    /// <summary>
    /// Effective uncertainty-aware horizontal safety radius in meters:
    /// R_eff = clamp(safetyRadius + sigmaMultiplier * horizontalSigma, minSafetyRadius, maxSafetyRadius).
    /// </summary>
    public float EffectiveSafetyRadius
    {
        get
        {
            if (stateProvider == null || !stateProvider.IsEstimatorReady)
                return safetyRadius;

            float horizSigma = stateProvider.CurrentState.HorizontalPositionStandardDeviation;
            return Mathf.Clamp(safetyRadius + sigmaMultiplier * horizSigma, minSafetyRadius, maxSafetyRadius);
        }
    }

    /// <summary>
    /// Effective uncertainty-aware vertical safety clearance margin in meters:
    /// M_eff = clamp(verticalSafetyMargin + verticalSigmaMultiplier * verticalSigma, minVerticalSafetyMargin, maxVerticalSafetyMargin).
    /// </summary>
    public float EffectiveVerticalSafetyMargin
    {
        get
        {
            if (stateProvider == null || !stateProvider.IsEstimatorReady)
                return verticalSafetyMargin;

            float vertSigma = stateProvider.CurrentState.VerticalPositionStandardDeviation;
            return Mathf.Clamp(verticalSafetyMargin + verticalSigmaMultiplier * vertSigma, minVerticalSafetyMargin, maxVerticalSafetyMargin);
        }
    }

    /// <summary>
    /// Effective uncertainty-aware warning radius threshold in meters:
    /// R_warning_eff = clamp(warningRadius + sigmaMultiplier * horizontalSigma, minWarningRadius, maxWarningRadius).
    /// </summary>
    public float EffectiveWarningRadius
    {
        get
        {
            if (stateProvider == null || !stateProvider.IsEstimatorReady)
                return warningRadius;

            float horizSigma = stateProvider.CurrentState.HorizontalPositionStandardDeviation;
            return Mathf.Clamp(warningRadius + sigmaMultiplier * horizSigma, minWarningRadius, maxWarningRadius);
        }
    }

    // Reactive Events for modular subscribers
    public event Action<ThreatReport> OnThreatEvaluated;
    public event Action<ThreatReport> OnCriticalThreatDetected;

    private UAVPerception perception;
    private PathFollower pathFollower;
    private TrackManager trackManager;
    private ThreatReport currentReport = ThreatReport.Clear;
    private readonly List<ThreatReport> allEvaluatedReports = new List<ThreatReport>(16);
    private readonly List<ThreatReport> activeThreatReports = new List<ThreatReport>(16);
    private readonly TrackedTarget[] trackedTargetBuffer = new TrackedTarget[64];

    private ThreatLevel lastThreatLevel = ThreatLevel.None;
    private GameObject lastCriticalObstacle = null;
    private int lastCriticalTrackId = -1;
    private bool wasInCriticalState = false;
    private IEstimatedStateProvider stateProvider;

    public void SetTrackManager(TrackManager manager) => trackManager = manager;
    public void SetStateProvider(IEstimatedStateProvider provider) => stateProvider = provider;
    public void SetPathFollower(PathFollower follower) => pathFollower = follower;

    private void Awake()
    {
        perception = GetComponent<UAVPerception>();
        pathFollower = GetComponent<PathFollower>();
        trackManager = GetComponent<TrackManager>();
        stateProvider = GetComponent<IEstimatedStateProvider>();
    }

    private void Update()
    {
        EvaluateThreats();
    }

    /// <summary>
    /// Evaluates active threats using TrackManager targets if available, or legacy perception fallback.
    /// </summary>
    public void EvaluateThreats()
    {
        // 1. Prefer Multi-Target TrackManager if attached and active
        if (trackManager != null && trackManager.ActiveTrackCount > 0)
        {
            int confirmedCount = trackManager.GetConfirmedTargets(trackedTargetBuffer, 0, 64);
            if (confirmedCount > 0)
            {
                EvaluateTrackedTargets(trackedTargetBuffer, confirmedCount);
                return;
            }
        }

        // 2. Legacy Perception Fallback
        if (perception != null && pathFollower != null)
        {
            IReadOnlyList<DetectedObstacle> obstacles = perception.DetectedObstacles;
            if (obstacles != null && obstacles.Count > 0)
            {
                EvaluateLegacyObstacles(obstacles);
                return;
            }
        }

        // 3. Clear threat state
        currentReport = ThreatReport.Clear;
        allEvaluatedReports.Clear();
        activeThreatReports.Clear();
        NotifyThreatState();
    }

    /// <summary>
    /// Evaluates a collection of TrackedTarget estimates against the UAV's flight trajectory,
    /// combining UAV ego uncertainty with target position uncertainty.
    /// </summary>
    public void EvaluateTrackedTargets(TrackedTarget[] targets, int count)
    {
        allEvaluatedReports.Clear();
        activeThreatReports.Clear();

        if (targets == null || count <= 0)
        {
            currentReport = ThreatReport.Clear;
            NotifyThreatState();
            return;
        }

        Vector3 uavPos = (stateProvider != null && stateProvider.IsEstimatorReady)
            ? stateProvider.CurrentState.Position
            : transform.position;

        Vector3 uavVelocity = (stateProvider != null && stateProvider.IsEstimatorReady)
            ? stateProvider.CurrentState.Velocity
            : (pathFollower != null ? pathFollower.CurrentVelocity : Vector3.zero);

        Vector3 uavForward = (stateProvider != null && stateProvider.IsEstimatorReady)
            ? stateProvider.CurrentState.Forward
            : transform.forward;

        float uavHorizSigma = (stateProvider != null && stateProvider.IsEstimatorReady)
            ? stateProvider.CurrentState.HorizontalPositionStandardDeviation
            : 0f;

        float uavVertSigma = (stateProvider != null && stateProvider.IsEstimatorReady)
            ? stateProvider.CurrentState.VerticalPositionStandardDeviation
            : 0f;

        ThreatReport highestReport = ThreatReport.Clear;

        for (int i = 0; i < count; i++)
        {
            TrackedTarget target = targets[i];

            // Ignore non-active tracking states
            if (target.Status != TrackStatus.Confirmed && target.Status != TrackStatus.Coasting)
            {
                continue;
            }

            Vector3 toTarget = target.EstimatedPosition - uavPos;

            // Ignore targets positioned behind the UAV
            if (Vector3.Dot(toTarget, uavForward) < -0.1f)
            {
                continue;
            }

            // Combine spatial uncertainties: sigma_comb = sqrt(sigma_uav^2 + sigma_target^2)
            float targetHorizSigma = target.HorizontalPositionStdDev;
            float combinedHorizSigma = Mathf.Sqrt(uavHorizSigma * uavHorizSigma + targetHorizSigma * targetHorizSigma);

            float targetVertSigma = target.VerticalPositionStdDev;
            float combinedVertSigma = Mathf.Sqrt(uavVertSigma * uavVertSigma + targetVertSigma * targetVertSigma);

            float effSafetyRadius = Mathf.Clamp(safetyRadius + sigmaMultiplier * combinedHorizSigma, minSafetyRadius, maxSafetyRadius);
            float effVerticalMargin = Mathf.Clamp(verticalSafetyMargin + verticalSigmaMultiplier * combinedVertSigma, minVerticalSafetyMargin, maxVerticalSafetyMargin);
            float effWarningRadius = Mathf.Clamp(warningRadius + sigmaMultiplier * combinedHorizSigma, minWarningRadius, maxWarningRadius);

            // Compute Closest Point of Approach (CPA) and Time to Collision (TTC)
            Vector3 vRel = target.EstimatedVelocity - uavVelocity;
            float distance = toTarget.magnitude;
            float vRelSq = vRel.sqrMagnitude;

            float ttc = float.PositiveInfinity;
            float distanceToCollision = distance;
            Vector3 collisionPoint = Vector3.zero;
            ThreatLevel level = ThreatLevel.None;

            if (distance <= effSafetyRadius)
            {
                // Immediate collision / inside safety margin
                level = ThreatLevel.Critical;
                ttc = 0f;
                distanceToCollision = distance;
                collisionPoint = target.EstimatedPosition;
            }
            else if (vRelSq > 1e-6f)
            {
                // Convergence check
                float tCpa = -Vector3.Dot(toTarget, vRel) / vRelSq;

                if (tCpa > 0f)
                {
                    Vector3 pCpa = toTarget + vRel * tCpa;
                    float dCpa = pCpa.magnitude;
                    float vertSeparationCpa = Mathf.Abs(pCpa.y);

                    if (dCpa <= effSafetyRadius && vertSeparationCpa <= effVerticalMargin && tCpa <= lookaheadTime)
                    {
                        level = ThreatLevel.Critical;
                        ttc = tCpa;
                        distanceToCollision = distance;
                        collisionPoint = uavPos + uavVelocity * tCpa;
                    }
                    else if (dCpa <= effWarningRadius && tCpa <= lookaheadTime)
                    {
                        level = ThreatLevel.Warning;
                        ttc = tCpa;
                        distanceToCollision = distance;
                        collisionPoint = uavPos + uavVelocity * tCpa;
                    }
                    else if (distance <= advisoryRadius)
                    {
                        level = ThreatLevel.Advisory;
                        distanceToCollision = distance;
                    }
                }
                else if (distance <= effWarningRadius)
                {
                    level = ThreatLevel.Warning;
                    distanceToCollision = distance;
                }
                else if (distance <= advisoryRadius)
                {
                    level = ThreatLevel.Advisory;
                    distanceToCollision = distance;
                }
            }
            else
            {
                // Stationary or co-moving target
                if (distance <= effWarningRadius)
                {
                    level = ThreatLevel.Warning;
                    distanceToCollision = distance;
                }
                else if (distance <= advisoryRadius)
                {
                    level = ThreatLevel.Advisory;
                    distanceToCollision = distance;
                }
            }

            ThreatReport report = new ThreatReport(
                level,
                target,
                collisionPoint,
                distanceToCollision,
                ttc,
                0);

            allEvaluatedReports.Add(report);
            if (level >= ThreatLevel.Warning)
            {
                activeThreatReports.Add(report);
            }

            // Deterministic Multi-Target Selection: Severity > TTC > Distance > TrackId
            if (IsMoreSevereThreat(report, highestReport))
            {
                highestReport = report;
            }
        }

        // Sort active threats deterministically
        if (activeThreatReports.Count > 1)
        {
            activeThreatReports.Sort((a, b) =>
            {
                int sev = b.ThreatLevel.CompareTo(a.ThreatLevel);
                if (sev != 0) return sev;

                int ttcComp = a.TimeToCollision.CompareTo(b.TimeToCollision);
                if (ttcComp != 0) return ttcComp;

                int distComp = a.DistanceToCollision.CompareTo(b.DistanceToCollision);
                if (distComp != 0) return distComp;

                return a.ThreateningTrack.TrackId.CompareTo(b.ThreateningTrack.TrackId);
            });
        }

        currentReport = highestReport;
        NotifyThreatState();
    }

    private static bool IsMoreSevereThreat(ThreatReport candidate, ThreatReport current)
    {
        if (candidate.ThreatLevel > current.ThreatLevel) return true;
        if (candidate.ThreatLevel < current.ThreatLevel) return false;
        if (candidate.ThreatLevel == ThreatLevel.None) return false;

        // 1. TTC comparison
        if (float.IsFinite(candidate.TimeToCollision) && !float.IsFinite(current.TimeToCollision)) return true;
        if (!float.IsFinite(candidate.TimeToCollision) && float.IsFinite(current.TimeToCollision)) return false;
        if (float.IsFinite(candidate.TimeToCollision) && float.IsFinite(current.TimeToCollision))
        {
            if (candidate.TimeToCollision < current.TimeToCollision - 0.001f) return true;
            if (candidate.TimeToCollision > current.TimeToCollision + 0.001f) return false;
        }

        // 2. Distance comparison
        if (float.IsFinite(candidate.DistanceToCollision) && !float.IsFinite(current.DistanceToCollision)) return true;
        if (!float.IsFinite(candidate.DistanceToCollision) && float.IsFinite(current.DistanceToCollision)) return false;
        if (float.IsFinite(candidate.DistanceToCollision) && float.IsFinite(current.DistanceToCollision))
        {
            if (candidate.DistanceToCollision < current.DistanceToCollision - 0.001f) return true;
            if (candidate.DistanceToCollision > current.DistanceToCollision + 0.001f) return false;
        }

        // 3. TrackId tie-breaker
        if (candidate.HasTrack && current.HasTrack)
        {
            return candidate.ThreateningTrack.TrackId < current.ThreateningTrack.TrackId;
        }

        return false;
    }

    private void EvaluateLegacyObstacles(IReadOnlyList<DetectedObstacle> obstacles)
    {
        allEvaluatedReports.Clear();
        activeThreatReports.Clear();

        Vector3 uavPos = (stateProvider != null && stateProvider.IsEstimatorReady)
            ? stateProvider.CurrentState.Position
            : transform.position;

        Vector3 uavVelocity = (stateProvider != null && stateProvider.IsEstimatorReady)
            ? stateProvider.CurrentState.Velocity
            : (pathFollower != null ? pathFollower.CurrentVelocity : Vector3.zero);

        Vector3 uavForward = (stateProvider != null && stateProvider.IsEstimatorReady)
            ? stateProvider.CurrentState.Forward
            : transform.forward;

        float nominalSpeed = pathFollower != null ? pathFollower.MoveSpeed : 5.0f;
        IReadOnlyList<Node> remainingWaypoints = pathFollower != null ? pathFollower.RemainingPath : null;
        Vector3 targetWaypoint = pathFollower != null ? pathFollower.TargetWaypoint : uavPos + uavForward * 10f;

        ThreatReport highestReport = ThreatReport.Clear;
        float effSafetyRadius = EffectiveSafetyRadius;
        float effVerticalMargin = EffectiveVerticalSafetyMargin;
        float effWarningRadius = EffectiveWarningRadius;

        for (int i = 0; i < obstacles.Count; i++)
        {
            DetectedObstacle obs = obstacles[i];

            Vector3 toObs = obs.WorldPosition - uavPos;
            if (Vector3.Dot(toObs, uavForward) < -0.1f)
                continue;

            CollisionPredictionResult prediction = CollisionPrediction.PredictPathCollision(
                uavPos,
                uavVelocity,
                nominalSpeed,
                remainingWaypoints,
                targetWaypoint,
                obs,
                effSafetyRadius,
                lookaheadTime,
                effVerticalMargin);

            bool hasValidCollision = prediction.WillCollide &&
                                    float.IsFinite(prediction.DistanceToCollision) &&
                                    float.IsFinite(prediction.TimeToCollision) &&
                                    prediction.ObstructedWaypointIndex >= 0;

            ThreatLevel evaluatedLevel;

            if (hasValidCollision)
            {
                if (prediction.CrossTrackDistance <= effSafetyRadius || prediction.TimeToCollision <= lookaheadTime)
                {
                    evaluatedLevel = ThreatLevel.Critical;
                }
                else if (prediction.CrossTrackDistance <= effWarningRadius)
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
                bool isVerticallyClear = prediction.VerticalSeparation >= effVerticalMargin;

                if (!isVerticallyClear && float.IsFinite(prediction.CrossTrackDistance) && prediction.CrossTrackDistance <= effWarningRadius)
                {
                    evaluatedLevel = ThreatLevel.Warning;
                }
                else if (!isVerticallyClear && float.IsFinite(prediction.CrossTrackDistance) && prediction.CrossTrackDistance <= advisoryRadius)
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

            if (evaluatedLevel > highestReport.ThreatLevel)
            {
                highestReport = report;
            }
            else if (evaluatedLevel == highestReport.ThreatLevel && evaluatedLevel != ThreatLevel.None)
            {
                bool currentIsFinite = float.IsFinite(prediction.DistanceToCollision);
                bool highestIsFinite = float.IsFinite(highestReport.DistanceToCollision);

                if (currentIsFinite && !highestIsFinite)
                {
                    highestReport = report;
                }
                else if (currentIsFinite && highestIsFinite && prediction.DistanceToCollision < highestReport.DistanceToCollision)
                {
                    highestReport = report;
                }
            }
        }

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
                               float.IsFinite(currentReport.DistanceToCollision);

        if (isCritical && !isValidCritical)
        {
            currentReport = ThreatReport.Clear;
            isCritical = false;
        }

        if (isValidCritical)
        {
            GameObject currentObstacle = currentReport.ThreateningObstacle.GameObject;
            int currentTrackId = currentReport.HasTrack ? currentReport.ThreateningTrack.TrackId : -1;

            bool isNewCriticalEntry = !wasInCriticalState;
            bool isDifferentObstacle = wasInCriticalState &&
                ((currentObstacle != null && currentObstacle != lastCriticalObstacle) ||
                 (currentTrackId != -1 && currentTrackId != lastCriticalTrackId));

            if (isNewCriticalEntry || isDifferentObstacle)
            {
                wasInCriticalState = true;
                lastCriticalObstacle = currentObstacle;
                lastCriticalTrackId = currentTrackId;

                string obsName = currentTrackId != -1
                    ? $"Track #{currentTrackId}"
                    : (currentObstacle != null ? currentObstacle.name : "Obstacle");

                if (logCriticalThreats)
                {
                    Debug.LogWarning(
                        $"[ThreatAssessment] CRITICAL ENTERED | Target={obsName} | " +
                        $"TTC={currentReport.TimeToCollision:F2}s | Dist={currentReport.DistanceToCollision:F2}m | " +
                        $"Wp={currentReport.ObstructedWaypointIndex}");
                }

                OnCriticalThreatDetected?.Invoke(currentReport);
            }
        }
        else
        {
            if (wasInCriticalState)
            {
                string prevObsName = lastCriticalTrackId != -1
                    ? $"Track #{lastCriticalTrackId}"
                    : (lastCriticalObstacle != null ? lastCriticalObstacle.name : "Obstacle");

                if (logCriticalThreats)
                {
                    Debug.Log($"[ThreatAssessment] CRITICAL CLEARED | Target={prevObsName}");
                }

                wasInCriticalState = false;
                lastCriticalObstacle = null;
                lastCriticalTrackId = -1;
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

                Vector3 dir = (next - prev).normalized;
                Vector3 perp = new Vector3(-dir.z, 0f, dir.x) * safetyRadius;
                Gizmos.DrawLine(prev + perp, next + perp);
                Gizmos.DrawLine(prev - perp, next - perp);

                prev = next;
            }
        }

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
            case ThreatLevel.Warning: return new Color(1f, 0.5f, 0f);
            case ThreatLevel.Advisory: return Color.yellow;
            default: return Color.green;
        }
    }
}
