using System;
using UnityEngine;

/// <summary>
/// Operational health status of the onboard state estimation engine.
/// </summary>
public enum EstimatorStatus
{
    /// <summary>Estimator has not yet received initial sensor alignments.</summary>
    Uninitialized,

    /// <summary>Accumulating initial sensor baseline and zero-velocity biases.</summary>
    Initializing,

    /// <summary>Nominal full multi-sensor fusion with low covariance and high confidence.</summary>
    Nominal,

    /// <summary>Operating under degraded sensor conditions (e.g. GPS denial, dead reckoning).</summary>
    Degraded,

    /// <summary>Estimator divergence or anomaly detected (high innovation residuals).</summary>
    Failed
}

/// <summary>
/// Satellite navigation GNSS/GPS fix quality status.
/// </summary>
public enum GpsFixState
{
    /// <summary>No satellite lock or GPS denied.</summary>
    NoFix,

    /// <summary>2D horizontal fix only (insufficient satellites for vertical).</summary>
    Fix2D,

    /// <summary>Standard 3D satellite position and velocity fix.</summary>
    Fix3D,

    /// <summary>High-precision differential GNSS / RTK lock.</summary>
    Differential,

    /// <summary>Degraded fix due to high dilution of precision (PDOP > 4) or multipath.</summary>
    Degraded
}

/// <summary>
/// Immutable, allocation-free data structure representing the UAV's onboard belief state and uncertainty.
///
/// ====================================================================================================
/// ARCHITECTURAL DOMAIN BOUNDARY DEFINITION:
/// ----------------------------------------------------------------------------------------------------
/// 1. GROUND TRUTH (Simulation Layer):
///    The exact physical reality in the Unity simulation engine (e.g. Transform, Rigidbody, True Obstacles).
///    Direct access is strictly restricted to physical sensor models and evaluation components.
///
/// 2. ESTIMATED STATE (Belief Layer):
///    What the onboard UAV algorithms believe to be true based on fused sensor observations.
///    Represented by this EstimatedState struct, including position, velocity, heading, and covariance.
///
/// 3. AUTONOMY (Planning & Guidance Layer):
///    Downstream decision-making (PathFollower, ThreatAssessment, ReplanningController) that must strictly
///    consume EstimatedState via IEstimatedStateProvider rather than querying Unity ground truth.
///
/// 4. EVALUATION (Benchmarking Layer):
///    Post-mission analysis (MissionManager, BenchmarkReporter) authorized to inspect both Ground Truth
///    and Estimated State to compute estimation error metrics (RMSE, NEES) and authoritative scores.
/// ====================================================================================================
/// </summary>
[Serializable]
public struct EstimatedState
{
    // ------------------------------------------------------------------------------------------------
    // 1. Kinematic State Belief
    // ------------------------------------------------------------------------------------------------

    /// <summary>Estimated 3D position in world navigation frame (meters).</summary>
    public Vector3 Position { get; }

    /// <summary>Estimated 3D velocity vector in world navigation frame (m/s).</summary>
    public Vector3 Velocity { get; }

    /// <summary>Estimated yaw heading angle in degrees [0, 360).</summary>
    public float YawDegrees { get; }

    /// <summary>Estimated yaw heading angle in radians.</summary>
    public float YawRadians => YawDegrees * Mathf.Deg2Rad;

    /// <summary>Estimated pitch attitude angle in degrees [-90, +90].</summary>
    public float PitchDegrees { get; }

    /// <summary>Estimated pitch attitude angle in radians.</summary>
    public float PitchRadians => PitchDegrees * Mathf.Deg2Rad;

    /// <summary>Estimated forward direction unit vector derived from heading and pitch.</summary>
    public Vector3 Forward => Quaternion.Euler(PitchDegrees, YawDegrees, 0f) * Vector3.forward;

    // ------------------------------------------------------------------------------------------------
    // 2. Sensor Bias Estimates
    // ------------------------------------------------------------------------------------------------

    /// <summary>Estimated accelerometer bias vector in body frame (m/s^2).</summary>
    public Vector3 AccelerometerBias { get; }

    /// <summary>Estimated gyroscope yaw rate bias in body frame (rad/s).</summary>
    public float GyroYawBias { get; }

    // ------------------------------------------------------------------------------------------------
    // 3. Covariance & Uncertainty Metrics (Diagonal Approximations & RSS)
    // ------------------------------------------------------------------------------------------------

    /// <summary>Position variance diagonal elements [Var(X), Var(Y), Var(Z)] in m^2.</summary>
    public Vector3 PositionVariance { get; }

    /// <summary>Velocity variance diagonal elements [Var(Vx), Var(Vy), Var(Vz)] in (m/s)^2.</summary>
    public Vector3 VelocityVariance { get; }

    /// <summary>Yaw heading angle variance in rad^2.</summary>
    public float YawVariance { get; }

    /// <summary>Maximum horizontal position variance (m^2) for uncertainty safety margins.</summary>
    public float HorizontalPositionVariance => Mathf.Max(PositionVariance.x, PositionVariance.z);

    /// <summary>Vertical altitude variance (m^2) for vertical clearance checks.</summary>
    public float VerticalPositionVariance => PositionVariance.y;

    /// <summary>Standard deviation of horizontal position estimation error (1-sigma, meters).</summary>
    public float HorizontalPositionStandardDeviation => Mathf.Sqrt(Mathf.Max(0f, HorizontalPositionVariance));

    /// <summary>Standard deviation of vertical altitude estimation error (1-sigma, meters).</summary>
    public float VerticalPositionStandardDeviation => Mathf.Sqrt(Mathf.Max(0f, VerticalPositionVariance));

    /// <summary>Standard deviation of horizontal velocity estimation error (1-sigma, m/s).</summary>
    public float HorizontalVelocityStandardDeviation => Mathf.Sqrt(Mathf.Max(0f, Mathf.Max(VelocityVariance.x, VelocityVariance.z)));

    // ------------------------------------------------------------------------------------------------
    // 4. Metadata, Health & Diagnostics
    // ------------------------------------------------------------------------------------------------

    /// <summary>Simulation timestamp at which this state estimate was computed (seconds).</summary>
    public float Timestamp { get; }

    /// <summary>Operating health status of the state estimator.</summary>
    public EstimatorStatus Status { get; }

    /// <summary>Current GNSS / GPS satellite fix state.</summary>
    public GpsFixState GpsState { get; }

    /// <summary>Composite navigation confidence score in [0.0, 1.0].</summary>
    public float NavigationConfidence { get; }

    /// <summary>Continuous elapsed time in seconds since the last accepted GPS measurement correction.</summary>
    public float DeadReckoningDuration { get; }

    /// <summary>Whether this state estimate is mathematically valid and suitable for navigation.</summary>
    public bool IsValid => Status == EstimatorStatus.Nominal || Status == EstimatorStatus.Degraded;

    public EstimatedState(
        Vector3 position,
        Vector3 velocity,
        float yawDegrees,
        float pitchDegrees,
        Vector3 accelerometerBias,
        float gyroYawBias,
        Vector3 positionVariance,
        Vector3 velocityVariance,
        float yawVariance,
        float timestamp,
        EstimatorStatus status,
        GpsFixState gpsState,
        float navigationConfidence = 1.0f,
        float deadReckoningDuration = 0f)
    {
        Position = position;
        Velocity = velocity;
        YawDegrees = yawDegrees;
        PitchDegrees = pitchDegrees;
        AccelerometerBias = accelerometerBias;
        GyroYawBias = gyroYawBias;
        PositionVariance = positionVariance;
        VelocityVariance = velocityVariance;
        YawVariance = yawVariance;
        Timestamp = timestamp;
        Status = status;
        GpsState = gpsState;
        NavigationConfidence = Mathf.Clamp01(navigationConfidence);
        DeadReckoningDuration = Mathf.Max(0f, deadReckoningDuration);
    }

    /// <summary>
    /// Factory helper creating a nominal baseline state from simulation coordinates.
    /// Used by transitional providers, mock-free tests, and deterministic benchmarks.
    /// </summary>
    public static EstimatedState CreateNominal(
        Vector3 position,
        Vector3 velocity,
        float yawDegrees,
        float pitchDegrees = 0f,
        float timestamp = 0f,
        float baselinePositionVariance = 0f,
        float navigationConfidence = 1.0f,
        float deadReckoningDuration = 0f)
    {
        return new EstimatedState(
            position,
            velocity,
            yawDegrees,
            pitchDegrees,
            Vector3.zero,
            0f,
            Vector3.one * baselinePositionVariance,
            Vector3.zero,
            0f,
            timestamp,
            EstimatorStatus.Nominal,
            GpsFixState.Fix3D,
            navigationConfidence,
            deadReckoningDuration);
    }

    /// <summary>
    /// Uninitialized default state representing an unpowered or pre-takeoff estimator.
    /// </summary>
    public static EstimatedState Uninitialized => new EstimatedState(
        Vector3.zero,
        Vector3.zero,
        0f,
        0f,
        Vector3.zero,
        0f,
        Vector3.one * 9999f,
        Vector3.one * 9999f,
        9999f,
        0f,
        EstimatorStatus.Uninitialized,
        GpsFixState.NoFix,
        0f,
        0f);
}
