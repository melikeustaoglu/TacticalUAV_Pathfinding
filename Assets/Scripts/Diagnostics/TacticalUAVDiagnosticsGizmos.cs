using System;
using System.Collections.Generic;
using UnityEngine;

#if UNITY_EDITOR
using UnityEditor;
#endif

/// <summary>
/// Dedicated 3D Scene View visual diagnostics and Gizmos renderer for the Tactical UAV autonomy stack.
/// Renders sensor geometries (LiDAR, Radar), multi-target tracks, EKF 3-sigma uncertainty ellipsoids,
/// threat envelopes, Velocity Obstacles (VO), and waypoint paths without altering runtime autonomy behavior.
/// Zero per-frame runtime heap allocations.
/// </summary>
[ExecuteInEditMode]
public class TacticalUAVDiagnosticsGizmos : MonoBehaviour
{
    [Header("Master Diagnostics Control")]
    [Tooltip("Master switch to enable or disable all tactical 3D diagnostics.")]
    [SerializeField] private bool showDiagnostics = true;

    [Header("LiDAR Visualization")]
    [Tooltip("Visualize LiDAR sensor origin and horizontal/vertical FOV scanning frustum.")]
    [SerializeField] private bool showLidar = true;

    [Tooltip("Visualize active LiDAR hit points and detection rays.")]
    [SerializeField] private bool showLidarHits = true;

    [Header("Radar Visualization")]
    [Tooltip("Visualize Doppler Radar sensor origin and detection range / FOV cone.")]
    [SerializeField] private bool showRadar = true;

    [Tooltip("Visualize active Radar detections and measured radial/target velocities.")]
    [SerializeField] private bool showRadarDetections = true;

    [Header("Threat & Safety Envelopes")]
    [Tooltip("Visualize active threat levels and collision forecast vectors.")]
    [SerializeField] private bool showThreats = true;

    [Tooltip("Visualize dynamic uncertainty-aware safety radius around threats.")]
    [SerializeField] private bool showSafetyEnvelope = true;

    [Tooltip("Visualize predicted Closest Point of Approach (CPA) and collision points.")]
    [SerializeField] private bool showPredictedCollision = true;

    [Header("Tactical Velocity Obstacles (VO)")]
    [Tooltip("Visualize Velocity Obstacle collision cones and candidate flight velocities.")]
    [SerializeField] private bool showVelocityObstacle = true;

    [Header("EKF State Estimation Uncertainty")]
    [Tooltip("Visualize EKF 3-sigma horizontal and vertical position uncertainty ellipsoid.")]
    [SerializeField] private bool showEkfUncertainty = true;

    [Header("Multi-Target Tracking")]
    [Tooltip("Visualize active target tracks (Tentative, Confirmed, Coasting, Lost).")]
    [SerializeField] private bool showTracks = true;

    [Tooltip("Visualize kinematic trajectory predictions for tracked targets.")]
    [SerializeField] private bool showTrackPredictions = true;

    [Header("Mission Path & Waypoints")]
    [Tooltip("Visualize active A* waypoint path and target waypoint.")]
    [SerializeField] private bool showPath = true;

    [Header("Display Throttling Limits")]
    [Tooltip("Maximum number of active tracks to render simultaneously.")]
    [SerializeField] private int maxDisplayedTracks = 16;

    [Tooltip("Maximum number of LiDAR detection rays to render.")]
    [SerializeField] private int maxDisplayedLidarRays = 32;

    [Tooltip("Maximum number of forward trajectory prediction steps per track.")]
    [SerializeField] private int maxDisplayedPredictions = 10;

    // Component References (Acquired safely at runtime / edit mode)
    private SimulatedLidarSensor lidarSensor;
    private SimulatedRadarSensor radarSensor;
    private TrackManager trackManager;
    private ThreatAssessment threatAssessment;
    private ReplanningController replanningController;
    private EkfStateProvider ekfStateProvider;
    private PathFollower pathFollower;

    // Preallocated non-allocating buffers
    private readonly TargetDetection[] lidarBuffer = new TargetDetection[64];
    private readonly TargetDetection[] radarBuffer = new TargetDetection[64];
    private readonly TrackedTarget[] trackBuffer = new TrackedTarget[64];

    // Public property accessors for testability & external config
    public bool ShowDiagnostics { get => showDiagnostics; set => showDiagnostics = value; }
    public bool ShowLidar { get => showLidar; set => showLidar = value; }
    public bool ShowLidarHits { get => showLidarHits; set => showLidarHits = value; }
    public bool ShowRadar { get => showRadar; set => showRadar = value; }
    public bool ShowRadarDetections { get => showRadarDetections; set => showRadarDetections = value; }
    public bool ShowThreats { get => showThreats; set => showThreats = value; }
    public bool ShowSafetyEnvelope { get => showSafetyEnvelope; set => showSafetyEnvelope = value; }
    public bool ShowPredictedCollision { get => showPredictedCollision; set => showPredictedCollision = value; }
    public bool ShowVelocityObstacle { get => showVelocityObstacle; set => showVelocityObstacle = value; }
    public bool ShowEkfUncertainty { get => showEkfUncertainty; set => showEkfUncertainty = value; }
    public bool ShowTracks { get => showTracks; set => showTracks = value; }
    public bool ShowTrackPredictions { get => showTrackPredictions; set => showTrackPredictions = value; }
    public bool ShowPath { get => showPath; set => showPath = value; }

    private void Awake()
    {
        AcquireReferences();
    }

    private void OnEnable()
    {
        AcquireReferences();
    }

    public void AcquireReferences()
    {
        if (lidarSensor == null) lidarSensor = GetComponent<SimulatedLidarSensor>();
        if (radarSensor == null) radarSensor = GetComponent<SimulatedRadarSensor>();
        if (trackManager == null) trackManager = GetComponent<TrackManager>();
        if (threatAssessment == null) threatAssessment = GetComponent<ThreatAssessment>();
        if (replanningController == null) replanningController = GetComponent<ReplanningController>();
        if (ekfStateProvider == null) ekfStateProvider = GetComponent<EkfStateProvider>();
        if (pathFollower == null) pathFollower = GetComponent<PathFollower>();
    }

#if UNITY_EDITOR
    private void OnDrawGizmos()
    {
        if (!showDiagnostics) return;

        AcquireReferences();
        Vector3 uavPos = transform.position;

        if (showLidar) DrawLidarDiagnostics(uavPos);
        if (showRadar) DrawRadarDiagnostics(uavPos);
        if (showEkfUncertainty) DrawEkfUncertaintyDiagnostics(uavPos);
        if (showTracks) DrawTrackDiagnostics(uavPos);
        if (showThreats) DrawThreatDiagnostics(uavPos);
        if (showVelocityObstacle) DrawVelocityObstacleDiagnostics(uavPos);
        if (showPath) DrawPathDiagnostics(uavPos);
    }

    private void DrawLidarDiagnostics(Vector3 uavPos)
    {
        if (lidarSensor == null) return;

        float range = lidarSensor.DetectionRange;
        float hFov = lidarSensor.FieldOfViewAngle;
        float vFov = lidarSensor.VerticalFovAngle;
        Vector3 forward = lidarSensor.transform.forward;
        Vector3 up = lidarSensor.transform.up;
        Vector3 right = lidarSensor.transform.right;

        // Draw LiDAR FOV boundary lines
        Gizmos.color = new Color(0f, 0.9f, 1f, 0.25f);
        Quaternion leftRot = Quaternion.AngleAxis(-hFov * 0.5f, up);
        Quaternion rightRot = Quaternion.AngleAxis(hFov * 0.5f, up);
        Quaternion topRot = Quaternion.AngleAxis(-vFov * 0.5f, right);
        Quaternion bottomRot = Quaternion.AngleAxis(vFov * 0.5f, right);

        Vector3 leftDir = leftRot * forward;
        Vector3 rightDir = rightRot * forward;
        Vector3 topDir = topRot * forward;
        Vector3 bottomDir = bottomRot * forward;

        Gizmos.DrawLine(uavPos, uavPos + leftDir * range);
        Gizmos.DrawLine(uavPos, uavPos + rightDir * range);
        Gizmos.DrawLine(uavPos, uavPos + topDir * range);
        Gizmos.DrawLine(uavPos, uavPos + bottomDir * range);

        // Draw LiDAR detection hits
        if (showLidarHits && lidarSensor.LastDetectionCount > 0)
        {
            int count = lidarSensor.TryGetDetections(lidarBuffer, 0, Mathf.Min(maxDisplayedLidarRays, lidarBuffer.Length), Time.time);
            Gizmos.color = new Color(0f, 1f, 0.8f, 0.8f);

            for (int i = 0; i < count; i++)
            {
                TargetDetection det = lidarBuffer[i];
                if (det.IsValid)
                {
                    Gizmos.DrawWireCube(det.MeasuredPosition, Vector3.one * 0.25f);
                    Gizmos.color = new Color(0f, 1f, 0.8f, 0.2f);
                    Gizmos.DrawLine(uavPos, det.MeasuredPosition);
                    Gizmos.color = new Color(0f, 1f, 0.8f, 0.8f);
                }
            }
        }
    }

    private void DrawRadarDiagnostics(Vector3 uavPos)
    {
        if (radarSensor == null) return;

        float range = radarSensor.DetectionRange;
        float hFov = radarSensor.FieldOfViewAngle;
        Vector3 forward = radarSensor.transform.forward;
        Vector3 up = radarSensor.transform.up;

        // Draw Radar arc / boundary
        Gizmos.color = new Color(1f, 0.6f, 0f, 0.25f);
        Vector3 leftEdge = Quaternion.AngleAxis(-hFov * 0.5f, up) * forward * range;
        Vector3 rightEdge = Quaternion.AngleAxis(hFov * 0.5f, up) * forward * range;
        Gizmos.DrawLine(uavPos, uavPos + leftEdge);
        Gizmos.DrawLine(uavPos, uavPos + rightEdge);
        Gizmos.DrawLine(uavPos + leftEdge, uavPos + rightEdge);

        // Draw Radar detections with Doppler velocity vectors
        if (showRadarDetections && radarSensor.LastDetectionCount > 0)
        {
            int count = radarSensor.TryGetDetections(radarBuffer, 0, Mathf.Min(maxDisplayedTracks, radarBuffer.Length), Time.time);
            for (int i = 0; i < count; i++)
            {
                TargetDetection det = radarBuffer[i];
                if (det.IsValid)
                {
                    Gizmos.color = new Color(1f, 0.5f, 0f, 0.9f);
                    Gizmos.DrawWireSphere(det.MeasuredPosition, 0.35f);

                    if (det.HasVelocity && det.MeasuredVelocity.sqrMagnitude > 0.01f)
                    {
                        Gizmos.color = Color.yellow;
                        Gizmos.DrawLine(det.MeasuredPosition, det.MeasuredPosition + det.MeasuredVelocity);
                    }
                }
            }
        }
    }

    private void DrawEkfUncertaintyDiagnostics(Vector3 uavPos)
    {
        if (ekfStateProvider == null || !ekfStateProvider.IsEstimatorReady) return;

        EstimatedState state = ekfStateProvider.CurrentState;
        Vector3 estPos = state.Position;
        float sigmaH = state.HorizontalPositionStandardDeviation;
        float sigmaV = state.VerticalPositionStandardDeviation;

        // 3-Sigma Uncertainty Envelope
        float radius3SigmaH = Mathf.Max(0.1f, 3.0f * sigmaH);
        float radius3SigmaV = Mathf.Max(0.1f, 3.0f * sigmaV);

        Gizmos.color = new Color(0.2f, 0.6f, 1f, 0.35f);
        DrawWireEllipsoid(estPos, radius3SigmaH, radius3SigmaV);

        // Draw Estimated Heading
        Gizmos.color = Color.cyan;
        Gizmos.DrawLine(estPos, estPos + state.Forward * 1.5f);
    }

    private void DrawTrackDiagnostics(Vector3 uavPos)
    {
        if (trackManager == null || trackManager.ActiveTrackCount <= 0) return;

        int count = trackManager.GetAllTargets(trackBuffer, 0, Mathf.Min(maxDisplayedTracks, trackBuffer.Length));

        for (int i = 0; i < count; i++)
        {
            TrackedTarget trk = trackBuffer[i];
            if (!trk.IsValid) continue;

            Color trackColor = GetTrackStatusColor(trk.Status);
            Gizmos.color = trackColor;

            // Target bounding representation
            Gizmos.DrawWireCube(trk.EstimatedPosition, Vector3.one * 0.6f);

            // Target velocity vector
            if (trk.Speed > 0.1f)
            {
                Gizmos.color = Color.magenta;
                Gizmos.DrawLine(trk.EstimatedPosition, trk.EstimatedPosition + trk.EstimatedVelocity);
            }

            // Kinematic forward projection steps (1s, 2s, 3s, 4s)
            if (showTrackPredictions && trk.Speed > 0.1f)
            {
                Gizmos.color = new Color(trackColor.r, trackColor.g, trackColor.b, 0.4f);
                Vector3 prevPos = trk.EstimatedPosition;
                int steps = Mathf.Min(maxDisplayedPredictions, 4);

                for (int s = 1; s <= steps; s++)
                {
                    float dt = s * 1.0f;
                    Vector3 predPos = trk.EstimatedPosition + trk.EstimatedVelocity * dt;
                    Gizmos.DrawLine(prevPos, predPos);
                    Gizmos.DrawWireSphere(predPos, 0.15f);
                    prevPos = predPos;
                }
            }
        }
    }

    private void DrawThreatDiagnostics(Vector3 uavPos)
    {
        if (threatAssessment == null) return;

        ThreatReport report = threatAssessment.CurrentThreatReport;
        float safetyRadius = threatAssessment.EffectiveSafetyRadius;
        float verticalMargin = threatAssessment.EffectiveVerticalSafetyMargin;

        IReadOnlyList<ThreatReport> allReports = threatAssessment.AllEvaluatedReports;
        if (allReports != null && allReports.Count > 0)
        {
            for (int i = 0; i < allReports.Count; i++)
            {
                ThreatReport r = allReports[i];
                if (r.ThreatLevel == ThreatLevel.None) continue;

                Color color = GetThreatLevelColor(r.ThreatLevel);
                Vector3 obsPos = r.ThreateningObstacle.WorldPosition;

                // Safety Clearance Envelope around threat
                if (showSafetyEnvelope && obsPos != Vector3.zero)
                {
                    Gizmos.color = new Color(color.r, color.g, color.b, 0.3f);
                    DrawWireCylinder(obsPos, safetyRadius, verticalMargin);
                }

                // Predicted Collision & CPA
                if (showPredictedCollision && r.EstimatedCollisionPoint != Vector3.zero && r.ThreatLevel >= ThreatLevel.Warning)
                {
                    Gizmos.color = color;
                    Gizmos.DrawWireSphere(r.EstimatedCollisionPoint, 0.4f);
                    Gizmos.DrawLine(uavPos, r.EstimatedCollisionPoint);
                }
            }
        }
    }

    private void DrawVelocityObstacleDiagnostics(Vector3 uavPos)
    {
        if (threatAssessment == null || !showVelocityObstacle) return;

        ThreatReport report = threatAssessment.CurrentThreatReport;
        if (!report.ThreateningObstacle.IsDynamic || report.ThreatLevel < ThreatLevel.Warning) return;

        Vector3 obsPos = report.ThreateningObstacle.WorldPosition;
        Vector3 obsVel = report.ThreateningObstacle.Velocity;
        float uavRadius = threatAssessment.EffectiveSafetyRadius;
        float obsRadius = 0.5f;

        VelocityObstacle vo = CollisionPrediction.CalculateVelocityObstacle(uavPos, obsPos, obsVel, uavRadius + obsRadius);
        if (!vo.IsValid) return;

        // Draw VO collision cone in velocity space centered at UAV position + VO apex
        Vector3 apexPos = uavPos + vo.Apex;
        Gizmos.color = new Color(1f, 0f, 0.2f, 0.6f);
        Gizmos.DrawWireSphere(apexPos, 0.25f);

        Vector3 relDir = vo.RelativePosition.normalized;
        Quaternion leftRot = Quaternion.AngleAxis(-vo.HalfAngleDeg, Vector3.up);
        Quaternion rightRot = Quaternion.AngleAxis(vo.HalfAngleDeg, Vector3.up);

        float coneLength = 5.0f;
        Vector3 leftRay = leftRot * relDir * coneLength;
        Vector3 rightRay = rightRot * relDir * coneLength;

        Gizmos.DrawLine(apexPos, apexPos + leftRay);
        Gizmos.DrawLine(apexPos, apexPos + rightRay);
        Gizmos.DrawLine(apexPos + leftRay, apexPos + rightRay);

        // Draw current UAV velocity vector
        if (pathFollower != null && pathFollower.CurrentVelocity.sqrMagnitude > 0.01f)
        {
            Vector3 uavVel = pathFollower.CurrentVelocity;
            bool insideVO = vo.ContainsVelocity(uavVel, 5.0f);
            Gizmos.color = insideVO ? Color.red : Color.green;
            Gizmos.DrawLine(uavPos, uavPos + uavVel);
            Gizmos.DrawWireSphere(uavPos + uavVel, 0.2f);
        }
    }

    private void DrawPathDiagnostics(Vector3 uavPos)
    {
        if (pathFollower == null) return;

        IReadOnlyList<Node> currentPath = pathFollower.CurrentPath;
        if (currentPath == null || currentPath.Count == 0) return;

        Gizmos.color = new Color(0f, 1f, 0.3f, 0.7f);
        Vector3 prev = uavPos;

        for (int i = pathFollower.CurrentWaypointIndex; i < currentPath.Count; i++)
        {
            Vector3 wp = currentPath[i].worldPosition;
            Gizmos.DrawLine(prev, wp);
            Gizmos.DrawWireSphere(wp, 0.2f);
            prev = wp;
        }

        // Highlight current target waypoint
        Vector3 targetWp = pathFollower.TargetWaypoint;
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireCube(targetWp, Vector3.one * 0.4f);
        Gizmos.DrawLine(uavPos, targetWp);
    }

    private static void DrawWireEllipsoid(Vector3 center, float radiusH, float radiusV)
    {
        const int segments = 16;
        float angleStep = 360f / segments;
        Vector3 prevH = center + new Vector3(radiusH, 0f, 0f);

        for (int i = 1; i <= segments; i++)
        {
            float rad = i * angleStep * Mathf.Deg2Rad;
            Vector3 nextH = center + new Vector3(Mathf.Cos(rad) * radiusH, 0f, Mathf.Sin(rad) * radiusH);
            Gizmos.DrawLine(prevH, nextH);
            prevH = nextH;
        }

        Gizmos.DrawLine(center + Vector3.up * radiusV, center - Vector3.up * radiusV);
    }

    private static void DrawWireCylinder(Vector3 center, float radius, float verticalMargin)
    {
        Vector3 topCenter = center + Vector3.up * verticalMargin;
        Vector3 bottomCenter = center - Vector3.up * verticalMargin;

        DrawWireEllipsoid(topCenter, radius, 0.05f);
        DrawWireEllipsoid(bottomCenter, radius, 0.05f);
        Gizmos.DrawLine(topCenter + Vector3.forward * radius, bottomCenter + Vector3.forward * radius);
        Gizmos.DrawLine(topCenter - Vector3.forward * radius, bottomCenter - Vector3.forward * radius);
        Gizmos.DrawLine(topCenter + Vector3.right * radius, bottomCenter + Vector3.right * radius);
        Gizmos.DrawLine(topCenter - Vector3.right * radius, bottomCenter - Vector3.right * radius);
    }

    private static Color GetTrackStatusColor(TrackStatus status)
    {
        switch (status)
        {
            case TrackStatus.Tentative: return Color.cyan;
            case TrackStatus.Confirmed: return new Color(1f, 0.2f, 0.6f);
            case TrackStatus.Coasting: return Color.yellow;
            case TrackStatus.Lost: return Color.gray;
            default: return Color.white;
        }
    }

    private static Color GetThreatLevelColor(ThreatLevel level)
    {
        switch (level)
        {
            case ThreatLevel.Critical: return Color.red;
            case ThreatLevel.Warning: return new Color(1f, 0.5f, 0f);
            case ThreatLevel.Advisory: return Color.yellow;
            default: return Color.green;
        }
    }
#endif
}
