using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Encapsulates detailed geometric and physical data for an obstacle detected by the UAV perception sensor.
/// </summary>
[Serializable]
public struct DetectedObstacle
{
    public GameObject GameObject { get; }
    public Collider Collider { get; }
    public Vector3 WorldPosition { get; }
    public Vector3 RelativePosition { get; }
    public Vector3 Direction { get; }
    public float Distance { get; }
    public float AngleFromHeading { get; }
    public Vector3 SurfaceNormal { get; }

    public DetectedObstacle(
        GameObject gameObject,
        Collider collider,
        Vector3 worldPosition,
        Vector3 relativePosition,
        Vector3 direction,
        float distance,
        float angleFromHeading,
        Vector3 surfaceNormal)
    {
        GameObject = gameObject;
        Collider = collider;
        WorldPosition = worldPosition;
        RelativePosition = relativePosition;
        Direction = direction;
        Distance = distance;
        AngleFromHeading = angleFromHeading;
        SurfaceNormal = surfaceNormal;
    }
}

/// <summary>
/// Onboard forward-looking perception sensor for the Tactical UAV.
/// Simulates LiDAR/Radar detection using physics broadphase queries, angular FOV filtering,
/// and line-of-sight raycast verification.
/// </summary>
public class UAVPerception : MonoBehaviour
{
    [Header("Sensor Configuration")]
    [Tooltip("Maximum detection range in meters.")]
    [SerializeField] private float detectionRange = 10.0f;

    [Tooltip("Horizontal forward Field of View (FOV) in degrees.")]
    [Range(10f, 180f)]
    [SerializeField] private float fieldOfViewAngle = 90.0f;

    [Tooltip("Layer mask specifying obstacle colliders to detect.")]
    [SerializeField] private LayerMask obstacleMask;

    [Header("Debug Visualization")]
    [SerializeField] private bool showGizmos = true;
    [SerializeField] private Color sensorClearColor = new Color(0f, 0.85f, 1f, 0.4f);
    [SerializeField] private Color sensorAlertColor = new Color(1f, 0.2f, 0.2f, 0.6f);
    [SerializeField] private Color contactRayColor = Color.yellow;
    [SerializeField] private Color nearestMarkerColor = Color.red;

    // Public Read-Only State for downstream subsystems (Threat Assessment / Dynamic Replanning)
    public bool HasObstacles => detectedObstacles.Count > 0;
    public int DetectedCount => detectedObstacles.Count;
    public DetectedObstacle NearestObstacle => nearestObstacle;
    public IReadOnlyList<DetectedObstacle> DetectedObstacles => detectedObstacles;

    public float DetectionRange
    {
        get => detectionRange;
        set => detectionRange = Mathf.Max(0.1f, value);
    }

    public float FieldOfViewAngle
    {
        get => fieldOfViewAngle;
        set => fieldOfViewAngle = Mathf.Clamp(value, 1f, 180f);
    }

    public LayerMask ObstacleMask
    {
        get => obstacleMask;
        set => obstacleMask = value;
    }

    // Reactive Events for modular subscribers
    public event Action<UAVPerception> OnPerceptionUpdated;
    public event Action<DetectedObstacle> OnObstacleDetected;
    public event Action OnObstaclesCleared;

    // Internal working buffers (Zero per-frame heap allocations)
    private readonly Collider[] overlapResults = new Collider[32];
    private readonly List<DetectedObstacle> detectedObstacles = new List<DetectedObstacle>(16);
    private DetectedObstacle nearestObstacle;
    private bool hadObstaclesLastFrame;

    private void Awake()
    {
        if (obstacleMask.value == 0)
        {
            obstacleMask = ProceduralObstacleGenerator.GetObstacleMask();
        }
    }

    private void Update()
    {
        PerformScan();
    }

    /// <summary>
    /// Executes a forward sensor scan using non-allocating physics overlap queries,
    /// horizontal FOV angular filtering, and line-of-sight raycasts.
    /// </summary>
    public void PerformScan()
    {
        detectedObstacles.Clear();
        nearestObstacle = default;
        float minDistance = float.MaxValue;
        bool hasNearest = false;

        Vector3 sensorPos = transform.position;
        Vector3 sensorForward = transform.forward;

        int hitCount = Physics.OverlapSphereNonAlloc(sensorPos, detectionRange, overlapResults, obstacleMask);

        for (int i = 0; i < hitCount; i++)
        {
            Collider candidate = overlapResults[i];
            if (candidate == null)
                continue;

            // Defensive check: ignore self if UAV has an attached collider
            if (candidate.transform.root == transform.root)
                continue;

            Vector3 closestPoint = candidate.ClosestPoint(sensorPos);
            Vector3 toObstacle = closestPoint - sensorPos;
            float distance = toObstacle.magnitude;

            if (distance > detectionRange)
                continue;

            Vector3 direction = distance > 0.0001f ? toObstacle / distance : sensorForward;
            float angle = Vector3.Angle(sensorForward, direction);

            // Forward FOV Cone Filtering
            if (angle <= fieldOfViewAngle * 0.5f)
            {
                // Line-of-sight verification via targeted raycast
                if (Physics.Raycast(sensorPos, direction, out RaycastHit hit, detectionRange, obstacleMask))
                {
                    // Confirm the ray hit the candidate collider or its parent/child hierarchy
                    if (hit.collider == candidate ||
                        hit.collider.transform.IsChildOf(candidate.transform) ||
                        candidate.transform.IsChildOf(hit.collider.transform))
                    {
                        Vector3 relativePos = transform.InverseTransformPoint(hit.point);
                        DetectedObstacle obstacle = new DetectedObstacle(
                            hit.collider.gameObject,
                            hit.collider,
                            hit.point,
                            relativePos,
                            direction,
                            hit.distance,
                            angle,
                            hit.normal
                        );

                        detectedObstacles.Add(obstacle);

                        if (hit.distance < minDistance)
                        {
                            minDistance = hit.distance;
                            nearestObstacle = obstacle;
                            hasNearest = true;
                        }
                    }
                }
            }
        }

        // Clean working buffer for next query
        Array.Clear(overlapResults, 0, hitCount);

        // Dispatch state change events
        if (HasObstacles)
        {
            if (hasNearest)
            {
                OnObstacleDetected?.Invoke(nearestObstacle);
            }
            hadObstaclesLastFrame = true;
        }
        else if (hadObstaclesLastFrame)
        {
            hadObstaclesLastFrame = false;
            OnObstaclesCleared?.Invoke();
        }

        OnPerceptionUpdated?.Invoke(this);
    }

    private void OnDrawGizmos()
    {
        if (!showGizmos)
            return;

        Vector3 sensorPos = transform.position;
        Vector3 sensorForward = transform.forward;
        Color activeColor = HasObstacles ? sensorAlertColor : sensorClearColor;

        // 1. Draw Forward Field-of-View Arc & Boundary Lines
        Gizmos.color = activeColor;
        float halfFov = fieldOfViewAngle * 0.5f;
        Vector3 leftDir = Quaternion.Euler(0f, -halfFov, 0f) * sensorForward;
        Vector3 rightDir = Quaternion.Euler(0f, halfFov, 0f) * sensorForward;

        Gizmos.DrawLine(sensorPos, sensorPos + leftDir * detectionRange);
        Gizmos.DrawLine(sensorPos, sensorPos + rightDir * detectionRange);
        Gizmos.DrawLine(sensorPos, sensorPos + sensorForward * detectionRange);

        // Draw Arc Segments
        const int arcSegments = 24;
        Vector3 prevPoint = sensorPos + leftDir * detectionRange;
        for (int i = 1; i <= arcSegments; i++)
        {
            float currentAngle = -halfFov + (fieldOfViewAngle / arcSegments) * i;
            Vector3 currentDir = Quaternion.Euler(0f, currentAngle, 0f) * sensorForward;
            Vector3 nextPoint = sensorPos + currentDir * detectionRange;
            Gizmos.DrawLine(prevPoint, nextPoint);
            prevPoint = nextPoint;
        }

        // 2. Draw Rays and Markers to Detected Obstacles
        if (detectedObstacles != null && detectedObstacles.Count > 0)
        {
            for (int i = 0; i < detectedObstacles.Count; i++)
            {
                DetectedObstacle obstacle = detectedObstacles[i];
                Gizmos.color = contactRayColor;
                Gizmos.DrawLine(sensorPos, obstacle.WorldPosition);
                Gizmos.DrawWireSphere(obstacle.WorldPosition, 0.2f);
            }

            // 3. Highlight Nearest Detected Obstacle
            if (HasObstacles)
            {
                Gizmos.color = nearestMarkerColor;
                Gizmos.DrawLine(sensorPos, nearestObstacle.WorldPosition);
                Gizmos.DrawWireSphere(nearestObstacle.WorldPosition, 0.35f);
                Gizmos.DrawCube(nearestObstacle.WorldPosition, Vector3.one * 0.2f);
            }
        }
    }
}
