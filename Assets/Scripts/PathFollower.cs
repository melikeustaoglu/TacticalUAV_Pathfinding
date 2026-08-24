using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Kinematic UAV waypoint follower with bounded yaw turn rate, smooth linear acceleration/deceleration,
/// adaptive cornering velocity scaling, and zero per-frame garbage collection allocations.
/// </summary>
public class PathFollower : MonoBehaviour
{
    [Header("Movement & Speed Settings")]
    [Tooltip("Nominal cruise flight speed in meters per second.")]
    [SerializeField] private float moveSpeed = 1.5f;

    [Tooltip("Waypoint proximity threshold to consider reached.")]
    [SerializeField] private float nodeReachThreshold = 0.1f;

    [Header("Kinematic Dynamics & Heading Constraints")]
    [Tooltip("Maximum yaw turn rate in degrees per second.")]
    [SerializeField] private float maxYawRate = 120.0f; // deg/s

    [Tooltip("Forward linear acceleration in m/s^2.")]
    [SerializeField] private float acceleration = 2.5f; // m/s^2

    [Tooltip("Linear braking deceleration in m/s^2.")]
    [SerializeField] private float deceleration = 3.5f; // m/s^2

    [Tooltip("Minimum cornering speed multiplier during sharp turns (e.g. 0.5 = 50% cruise speed).")]
    [SerializeField] private float minCornerSpeedRatio = 0.5f;

    [Header("Vertical Flight Kinematics")]
    [Tooltip("Maximum vertical climb rate in meters per second.")]
    [SerializeField] private float maxClimbRate = 1.5f;

    [Tooltip("Maximum vertical descent rate in meters per second.")]
    [SerializeField] private float maxDescentRate = 2.0f;

    [Tooltip("Vertical linear acceleration in m/s^2.")]
    [SerializeField] private float verticalAcceleration = 2.0f;

    [Tooltip("Vertical linear braking deceleration in m/s^2.")]
    [SerializeField] private float verticalDeceleration = 2.5f;

    [Tooltip("Vertical altitude arrival threshold in meters.")]
    [SerializeField] private float altitudeReachThreshold = 0.1f;

    [Tooltip("Minimum allowable flight altitude in meters.")]
    [SerializeField] private float minFlightAltitude = 1.0f;

    [Tooltip("Maximum allowable flight altitude (ceiling) in meters.")]
    [SerializeField] private float maxFlightAltitude = 6.0f;

    [Header("Legacy / Compatibility Settings")]
    [SerializeField] private float rotationSpeed = 8.0f;
    [SerializeField] private bool useRigidbody;
    [SerializeField] private bool showGizmos = true;

    private Rigidbody rb;
    private Pathfinding pathfinding;
    private IEstimatedStateProvider stateProvider;
    private List<Node> currentPath;
    private int pathIndex;
    private bool isFollowing;
    private float currentFlightSpeed = 0f;
    private float currentVerticalSpeed = 0f;
    private float targetAltitude = 1.0f;
    private Vector3 currentVelocity;
    private Vector3 lastPosition;

    // Runtime Telemetry for Autonomous Subsystems
    public bool IsFollowing => isFollowing;
    public int CurrentWaypointIndex => pathIndex;
    public float CurrentFlightSpeed => currentFlightSpeed;
    public float CurrentVerticalSpeed => currentVerticalSpeed;
    public Vector3 CurrentVelocity => isFollowing ? currentVelocity : Vector3.zero;
    public Vector3 TargetWaypoint => (currentPath != null && pathIndex < currentPath.Count) ? GetTargetPosition(currentPath[pathIndex]) : GetEstimatedPosition();
    public IReadOnlyList<Node> RemainingPath => (currentPath != null && pathIndex < currentPath.Count)
        ? currentPath.GetRange(pathIndex, currentPath.Count - pathIndex)
        : (IReadOnlyList<Node>)Array.Empty<Node>();
    public IReadOnlyList<Node> CurrentPath => currentPath != null ? currentPath : (IReadOnlyList<Node>)Array.Empty<Node>();

    // Events
    public event Action OnDestinationReached;

    private Vector3 GetEstimatedPosition()
    {
        return (stateProvider != null && stateProvider.IsEstimatorReady)
            ? stateProvider.CurrentState.Position
            : transform.position;
    }

    public float MoveSpeed
    {
        get => moveSpeed;
        set => moveSpeed = Mathf.Max(0f, value);
    }

    public float MaxYawRate
    {
        get => maxYawRate;
        set => maxYawRate = Mathf.Max(10.0f, value);
    }

    public float Acceleration
    {
        get => acceleration;
        set => acceleration = Mathf.Max(0.1f, value);
    }

    public float Deceleration
    {
        get => deceleration;
        set => deceleration = Mathf.Max(0.1f, value);
    }

    public float MinCornerSpeedRatio
    {
        get => minCornerSpeedRatio;
        set => minCornerSpeedRatio = Mathf.Clamp01(value);
    }

    public float MaxClimbRate
    {
        get => maxClimbRate;
        set => maxClimbRate = Mathf.Max(0.1f, value);
    }

    public float MaxDescentRate
    {
        get => maxDescentRate;
        set => maxDescentRate = Mathf.Max(0.1f, value);
    }

    public float VerticalAcceleration
    {
        get => verticalAcceleration;
        set => verticalAcceleration = Mathf.Max(0.1f, value);
    }

    public float VerticalDeceleration
    {
        get => verticalDeceleration;
        set => verticalDeceleration = Mathf.Max(0.1f, value);
    }

    public float AltitudeReachThreshold
    {
        get => altitudeReachThreshold;
        set => altitudeReachThreshold = Mathf.Max(0.01f, value);
    }

    public float MinFlightAltitude
    {
        get => minFlightAltitude;
        set => minFlightAltitude = Mathf.Max(0.1f, value);
    }

    public float MaxFlightAltitude
    {
        get => maxFlightAltitude;
        set => maxFlightAltitude = Mathf.Max(minFlightAltitude, value);
    }

    public float TargetAltitude
    {
        get => targetAltitude;
        set => targetAltitude = Mathf.Clamp(value, minFlightAltitude, maxFlightAltitude);
    }

    public void SetTargetAltitude(float altitude)
    {
        targetAltitude = Mathf.Clamp(altitude, minFlightAltitude, maxFlightAltitude);
    }

    public float RotationSpeed
    {
        get => rotationSpeed;
        set => rotationSpeed = Mathf.Max(0f, value);
    }

    [Header("Uncertainty-Aware Speed Parameters")]
    [Tooltip("Nominal horizontal uncertainty threshold below which cruise speed is 100% (meters).")]
    [SerializeField] private float nominalUncertaintyThreshold = 0.15f;

    [Tooltip("Sensitivity coefficient for cruise speed scaling under horizontal position uncertainty.")]
    [SerializeField] private float speedScaleSensitivity = 0.60f;

    [Tooltip("Minimum allowable cruise speed scale under high uncertainty.")]
    [SerializeField] private float minCruiseSpeedScale = 0.60f;

    public float NominalUncertaintyThreshold
    {
        get => nominalUncertaintyThreshold;
        set => nominalUncertaintyThreshold = Mathf.Max(0f, value);
    }

    public float SpeedScaleSensitivity
    {
        get => speedScaleSensitivity;
        set => speedScaleSensitivity = Mathf.Max(0f, value);
    }

    public float MinCruiseSpeedScale
    {
        get => minCruiseSpeedScale;
        set => minCruiseSpeedScale = Mathf.Clamp01(value);
    }

    /// <summary>
    /// Effective cruise speed scale based on horizontal position uncertainty:
    /// S_speed = clamp(1.0 - 0.60 * max(0, sigma_horiz - 0.15), 0.60, 1.0).
    /// </summary>
    public float UncertaintySpeedScale
    {
        get
        {
            if (stateProvider == null || !stateProvider.IsEstimatorReady)
                return 1.0f;

            float horizSigma = stateProvider.CurrentState.HorizontalPositionStandardDeviation;
            float excessSigma = Mathf.Max(0f, horizSigma - nominalUncertaintyThreshold);
            return Mathf.Clamp(1.0f - speedScaleSensitivity * excessSigma, minCruiseSpeedScale, 1.0f);
        }
    }

    /// <summary>
    /// Net effective cruise speed in m/s, accounting for base moveSpeed, uncertainty throttling, and tactical overrides.
    /// </summary>
    public float EffectiveCruiseSpeed
    {
        get
        {
            float baseCruise = moveSpeed * UncertaintySpeedScale;
            if (IsSpeedOverrideActive)
            {
                return Mathf.Max(0.5f, baseCruise * speedOverrideRatio);
            }
            return baseCruise;
        }
    }

    public void SetStateProvider(IEstimatedStateProvider provider) => stateProvider = provider;

    [Header("Tactical Speed Override")]
    private bool isSpeedOverrideActive = false;
    private float speedOverrideRatio = 1.0f;
    private float speedOverrideEndTime = 0f;

    public bool IsSpeedOverrideActive => isSpeedOverrideActive && Time.time < speedOverrideEndTime;
    public float CurrentSpeedOverrideRatio => IsSpeedOverrideActive ? speedOverrideRatio : 1.0f;

    /// <summary>
    /// Applies a temporary tactical speed modulation (e.g. slowing down or pacing to avoid dynamic VO collision).
    /// </summary>
    /// <param name="speedRatio">Fraction of cruise speed [0.3, 1.2], clamped to maintain minimum 0.5 m/s.</param>
    /// <param name="duration">Duration of override in seconds (clamped to [0.1, 10.0]).</param>
    public void ApplyTacticalSpeedOverride(float speedRatio, float duration)
    {
        if (duration <= 0f)
        {
            ClearSpeedOverride();
            return;
        }

        // Ensure effective speed never falls below 0.5 m/s
        float minRatio = moveSpeed > 0.001f ? Mathf.Clamp01(0.5f / moveSpeed) : 0.5f;
        speedOverrideRatio = Mathf.Clamp(speedRatio, minRatio, 1.2f);
        speedOverrideEndTime = Time.time + Mathf.Clamp(duration, 0.1f, 10.0f);
        isSpeedOverrideActive = true;
    }

    /// <summary>
    /// Clears any active tactical speed override and restores nominal cruise speed.
    /// </summary>
    public void ClearSpeedOverride()
    {
        isSpeedOverrideActive = false;
        speedOverrideRatio = 1.0f;
        speedOverrideEndTime = 0f;
    }

    private void Awake()
    {
        rb = GetComponent<Rigidbody>();
        pathfinding = FindFirstObjectByType<Pathfinding>();
        stateProvider = GetComponent<IEstimatedStateProvider>();
        lastPosition = transform.position;
        targetAltitude = transform.position.y;
    }

    public void StartFollowing(List<Node> path)
    {
        if (path == null || path.Count == 0)
        {
            StopFollowing();
            return;
        }

        currentPath = path;
        pathIndex = 0;
        isFollowing = true;
        lastPosition = transform.position;

        // If initiating from rest, reset speed to 0; if replanning mid-flight, preserve momentum
        if (currentVelocity.sqrMagnitude < 0.01f)
        {
            currentFlightSpeed = 0f;
            currentVerticalSpeed = 0f;
        }

        UpdateRemainingPathLine();
    }

    public void StopFollowing()
    {
        isFollowing = false;
        currentPath = null;
        pathIndex = 0;
        currentFlightSpeed = 0f;
        currentVerticalSpeed = 0f;
        currentVelocity = Vector3.zero;
    }

    private void Update()
    {
        if (!isFollowing || (useRigidbody && rb != null))
        {
            currentVelocity = Vector3.zero;
            lastPosition = transform.position;
            return;
        }

        MoveAlongPath(
            transform.position,
            Time.deltaTime,
            position => transform.position = position,
            rotation => transform.rotation = rotation);
    }

    private void FixedUpdate()
    {
        if (!isFollowing || !useRigidbody || rb == null)
            return;

        MoveAlongPath(
            rb.position,
            Time.fixedDeltaTime,
            rb.MovePosition,
            rb.MoveRotation);
    }

    [Header("Debug & Telemetry")]
    [SerializeField] private bool enableDebugLogging = true;
    private float nextDebugLogTime = 0f;

    private void MoveAlongPath(
        Vector3 currentPosition,
        float deltaTime,
        Action<Vector3> applyPosition,
        Action<Quaternion> applyRotation)
    {
        if (currentPath == null || currentPath.Count == 0 || pathIndex >= currentPath.Count)
        {
            StopFollowing();
            return;
        }

        Vector3 target = GetTargetPosition(currentPath[pathIndex]);
        float distToActiveWaypoint = Vector3.Distance(currentPosition, target);

        // Advance to next waypoint if already within reach radius
        if (distToActiveWaypoint <= nodeReachThreshold)
        {
            pathIndex++;
            if (pathIndex >= currentPath.Count)
            {
                OnDestinationReached?.Invoke();
                StopFollowing();
                return;
            }

            target = GetTargetPosition(currentPath[pathIndex]);
            distToActiveWaypoint = Vector3.Distance(currentPosition, target);
        }

        // 1. Heading Alignment & Maximum Yaw Rate Clamping (deg/s) (Strictly Horizontal Heading)
        Vector3 toTarget = target - currentPosition;
        toTarget.y = 0f;
        float headingErrorDeg = 0f;
        float targetYawDeg = 0f;

        Quaternion currentRot = (useRigidbody && rb != null) ? rb.rotation : transform.rotation;
        float currentYaw = currentRot.eulerAngles.y;

        if (toTarget.sqrMagnitude > 0.0001f)
        {
            Quaternion targetYawRotation = Quaternion.LookRotation(toTarget.normalized, Vector3.up);
            targetYawDeg = targetYawRotation.eulerAngles.y;
            headingErrorDeg = Quaternion.Angle(Quaternion.Euler(0f, currentYaw, 0f), targetYawRotation);
        }

        // Estimator health check: If estimator failed, halt flight immediately
        if (stateProvider != null && stateProvider.CurrentState.Status == EstimatorStatus.Failed)
        {
            StopFollowing();
            return;
        }

        // Check for speed override expiration
        if (isSpeedOverrideActive && Time.time >= speedOverrideEndTime)
        {
            ClearSpeedOverride();
        }

        float effectiveCruiseSpeed = EffectiveCruiseSpeed;

        // 2. Adaptive Cornering Target Speed (Immediate Heading Deviation + Lookahead Anticipation)
        float cornerFactor = Mathf.Clamp01(Mathf.Cos(headingErrorDeg * Mathf.Deg2Rad));
        float targetSpeed = Mathf.Lerp(effectiveCruiseSpeed * minCornerSpeedRatio, effectiveCruiseSpeed, cornerFactor);

        // Lookahead: Anticipate sharp turns at upcoming waypoints and brake before reaching the corner
        if (pathIndex < currentPath.Count - 1)
        {
            Vector3 nextTarget = GetTargetPosition(currentPath[pathIndex + 1]);
            Vector3 currentSegmentDir = (target - currentPosition).normalized;
            Vector3 nextSegmentDir = (nextTarget - target).normalized;
            currentSegmentDir.y = 0f;
            nextSegmentDir.y = 0f;

            float upcomingTurnAngle = Vector3.Angle(currentSegmentDir, nextSegmentDir);
            if (upcomingTurnAngle > 25f)
            {
                float turnSeverity = Mathf.Clamp01(upcomingTurnAngle / 90f);
                float desiredCornerEntrySpeed = Mathf.Lerp(moveSpeed, moveSpeed * minCornerSpeedRatio, turnSeverity);
                float requiredBrakingDist = Mathf.Max(0.5f, (currentFlightSpeed * currentFlightSpeed - desiredCornerEntrySpeed * desiredCornerEntrySpeed) / (2f * deceleration));

                if (distToActiveWaypoint <= requiredBrakingDist)
                {
                    float brakeProgress = Mathf.Clamp01(distToActiveWaypoint / requiredBrakingDist);
                    float anticipatorySpeed = Mathf.Lerp(desiredCornerEntrySpeed, moveSpeed, brakeProgress);
                    targetSpeed = Mathf.Min(targetSpeed, anticipatorySpeed);
                }
            }
        }

        // 3. Terminal Deceleration Check (smooth arrival at mission destination)
        if (pathIndex == currentPath.Count - 1)
        {
            float stoppingSpeed = Mathf.Sqrt(2f * deceleration * Mathf.Max(0.01f, distToActiveWaypoint));
            targetSpeed = Mathf.Min(targetSpeed, stoppingSpeed);
        }

        // Maintain minimum flight velocity so the UAV never deadlocks
        targetSpeed = Mathf.Max(targetSpeed, 0.25f);

        // 4. Smooth Horizontal Acceleration / Deceleration Integration
        if (currentFlightSpeed < targetSpeed)
        {
            currentFlightSpeed = Mathf.MoveTowards(currentFlightSpeed, targetSpeed, acceleration * deltaTime);
        }
        else
        {
            currentFlightSpeed = Mathf.MoveTowards(currentFlightSpeed, targetSpeed, deceleration * deltaTime);
        }

        // 5. Horizontal Translational Step Integration (Heading-Coupled Translation)
        float forwardAlignment = Mathf.Max(0.20f, Mathf.Cos(headingErrorDeg * Mathf.Deg2Rad));
        float effectiveDisplacementSpeed = currentFlightSpeed * forwardAlignment;
        float horizontalStep = effectiveDisplacementSpeed * deltaTime;

        Vector3 targetHorizontal = new Vector3(target.x, currentPosition.y, target.z);
        Vector3 newHorizontalPos = Vector3.MoveTowards(currentPosition, targetHorizontal, horizontalStep);

        // 6. Vertical Velocity & Altitude Integration (Climb / Descent Dynamics)
        float targetY = Mathf.Clamp(target.y, minFlightAltitude, maxFlightAltitude);
        float altitudeDelta = targetY - currentPosition.y;
        float desiredVy = 0f;

        if (Mathf.Abs(altitudeDelta) > altitudeReachThreshold)
        {
            float maxRate = altitudeDelta > 0f ? maxClimbRate : maxDescentRate;
            float brakingSpeed = Mathf.Sqrt(2f * verticalDeceleration * Mathf.Abs(altitudeDelta));
            desiredVy = Mathf.Sign(altitudeDelta) * Mathf.Min(maxRate, brakingSpeed);
        }

        float vAccel = (Mathf.Abs(desiredVy) > Mathf.Abs(currentVerticalSpeed)) ? verticalAcceleration : verticalDeceleration;
        currentVerticalSpeed = Mathf.MoveTowards(currentVerticalSpeed, desiredVy, vAccel * deltaTime);

        float newY = currentPosition.y + currentVerticalSpeed * deltaTime;
        if ((altitudeDelta > 0f && newY > targetY) || (altitudeDelta < 0f && newY < targetY))
        {
            newY = targetY;
            currentVerticalSpeed = 0f;
        }

        Vector3 newPosition = new Vector3(newHorizontalPos.x, newY, newHorizontalPos.z);
        applyPosition(newPosition);

        // 7. Rotation: Horizontal Yaw Tracking + Visual Pitch Attitude Clamped to [-30, +30] deg
        float newYaw = currentYaw;
        if (toTarget.sqrMagnitude > 0.0001f)
        {
            newYaw = Mathf.MoveTowardsAngle(currentYaw, targetYawDeg, maxYawRate * deltaTime);
        }

        float desiredPitch = 0f;
        if (Mathf.Abs(currentVerticalSpeed) > 0.01f && currentFlightSpeed > 0.01f)
        {
            float pitchRad = Mathf.Atan2(currentVerticalSpeed, currentFlightSpeed);
            desiredPitch = Mathf.Clamp(-pitchRad * Mathf.Rad2Deg, -30f, 30f);
        }

        Quaternion newRotation = Quaternion.Euler(desiredPitch, newYaw, 0f);
        applyRotation(newRotation);

        // 8. 3D Velocity Calculation for Telemetry & Perception
        Vector3 displacement = newPosition - currentPosition;
        currentVelocity = deltaTime > 0.00001f ? (displacement / deltaTime) : Vector3.zero;
        lastPosition = newPosition;

        // Periodic low-frequency debug logging (every 0.5s)
        if (enableDebugLogging && Time.time >= nextDebugLogTime)
        {
            nextDebugLogTime = Time.time + 0.5f;
            Debug.Log($"[PathFollower] Speed: {currentFlightSpeed:F2} m/s (Vert: {currentVerticalSpeed:F2} m/s, Alt: {newPosition.y:F2}m) | HeadingErr: {headingErrorDeg:F1}° | Waypoint: {pathIndex + 1}/{currentPath.Count}");
        }

        if (Vector3.Distance(newPosition, target) <= nodeReachThreshold)
        {
            pathIndex++;
        }

        if (pathIndex >= currentPath.Count)
        {
            OnDestinationReached?.Invoke();
            StopFollowing();
            return;
        }

        UpdateRemainingPathLine();
    }

    private void UpdateRemainingPathLine()
    {
        if (pathfinding == null || currentPath == null || currentPath.Count == 0)
            return;

        int remainingNodeCount = currentPath.Count - pathIndex;
        if (remainingNodeCount <= 0)
        {
            pathfinding.ClearPathLineRenderer();
            return;
        }

        List<Node> remainingPath = currentPath.GetRange(pathIndex, remainingNodeCount);
        pathfinding.UpdatePathLineRenderer(remainingPath, transform.position);
    }

    private Vector3 GetTargetPosition(Node node)
    {
        if (node == null)
            return new Vector3(transform.position.x, targetAltitude, transform.position.z);

        float targetY = targetAltitude;
        if (node.worldPosition.y > 0.001f && Mathf.Abs(node.worldPosition.y - transform.position.y) > 0.001f)
        {
            targetY = Mathf.Clamp(node.worldPosition.y, minFlightAltitude, maxFlightAltitude);
        }

        return new Vector3(node.worldPosition.x, targetY, node.worldPosition.z);
    }

    private void OnDrawGizmos()
    {
        if (!showGizmos)
            return;

        Vector3 uavPos = transform.position;

        // 1. Highlight Active TargetWaypoint and direct steering line
        if (isFollowing && currentPath != null && pathIndex < currentPath.Count)
        {
            Vector3 target = TargetWaypoint;

            // Highlight target waypoint with yellow wire sphere
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireSphere(target, 0.4f);

            // Draw direct line from UAV to active waypoint
            Gizmos.color = new Color(1f, 0.92f, 0.016f, 0.75f);
            Gizmos.DrawLine(uavPos, target);
        }

        // 2. Draw Kinematic Velocity Vector
        if (currentVelocity.sqrMagnitude > 0.01f)
        {
            Vector3 velOrigin = uavPos + Vector3.up * 0.15f;
            Vector3 velEnd = velOrigin + currentVelocity;

            Gizmos.color = Color.green;
            Gizmos.DrawLine(velOrigin, velEnd);
            Gizmos.DrawWireSphere(velEnd, 0.12f);
        }
    }
}
