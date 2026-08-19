using System;
using UnityEngine;

/// <summary>
/// 11-State Discrete Extended Kalman Filter (EKF) Core Engine for autonomous UAV state estimation.
///
/// ====================================================================================================
/// MATHEMATICAL FORMULATION:
/// ----------------------------------------------------------------------------------------------------
/// State Vector (11x1):
///   x = [ p_x, p_y, p_z,          (0..2: 3D World Position in meters)
///         v_x, v_y, v_z,          (3..5: 3D World Velocity in m/s)
///         yaw,                    (6:    Heading in radians, normalized [-pi, pi])
///         b_ax, b_ay, b_az,       (7..9: 3D Accelerometer body-frame bias in m/s^2)
///         b_gyro_z ]              (10:   Gyroscope yaw rate body-frame bias in rad/s)
///
/// Kinematic Mechanization & Coordinate Convention (Unity NED equivalent):
///   • World X: East (+X)
///   • World Y: Up (+Y, Altitude)
///   • World Z: North (+Z)
///   • Body to World Rotation: R_B_W(yaw) = [ cos(yaw) 0 sin(yaw); 0 1 0; -sin(yaw) 0 cos(yaw) ]
///   • Gravity Vector: g = [0, -9.81, 0]^T
///   • Joseph-Form Covariance Update for numerical stability: P = (I-KH)P(I-KH)^T + KRK^T
/// ====================================================================================================
/// </summary>
public class ExtendedKalmanFilter
{
    private Vector11 state = Vector11.Zero;
    private Matrix11x11 covariance = Matrix11x11.Identity;
    private Matrix11x11 processNoiseQ = Matrix11x11.Zero;

    private float lastPredictionTime = -1.0f;
    private bool hasPredictionTimestamp = false;
    private EstimatorStatus status = EstimatorStatus.Uninitialized;
    private GpsFixState gpsState = GpsFixState.NoFix;

    // Diagnostics & Statistics
    private int acceptedMeasurements = 0;
    private int rejectedMeasurements = 0;
    private int predictionCount = 0;
    private float lastInnovationDistanceSq = 0f;

    // Tuning Parameters
    private float accelProcessNoiseSigma = 0.15f;      // m/s^2 / sqrt(Hz)
    private float gyroProcessNoiseSigma = 0.015f;      // rad/s / sqrt(Hz)
    private float accelBiasDriftSigma = 0.002f;        // m/s^3 / sqrt(Hz)
    private float gyroBiasDriftSigma = 0.0002f;        // rad/s^2 / sqrt(Hz)
    private float mahalanobisGateThresholdSq = 16.0f;  // 4-sigma gating threshold (~99.99%)

    public EstimatorStatus Status => status;
    public GpsFixState GpsState => gpsState;
    public Vector11 StateVector => state;
    public Matrix11x11 CovarianceMatrix => covariance;
    public int AcceptedMeasurementsCount => acceptedMeasurements;
    public int RejectedMeasurementsCount => rejectedMeasurements;
    public int PredictionCount => predictionCount;
    public float LastInnovationDistanceSq => lastInnovationDistanceSq;
    public bool IsInitialized => status == EstimatorStatus.Nominal || status == EstimatorStatus.Degraded;

    public float MahalanobisGateThresholdSq
    {
        get => mahalanobisGateThresholdSq;
        set => mahalanobisGateThresholdSq = Mathf.Max(1.0f, value);
    }

    public ExtendedKalmanFilter()
    {
        Reset();
    }

    /// <summary>
    /// Explicitly initializes the filter state and covariance from initial sensor observations.
    /// Does NOT access Unity Transform or ground truth.
    /// </summary>
    public void Initialize(
        Vector3 initialPosition,
        Vector3 initialVelocity,
        float initialYawRad,
        Vector3 positionVariance,
        Vector3 velocityVariance,
        float yawVariance = 0.1f,
        float timestamp = 0f)
    {
        state = Vector11.Zero;
        state[0] = initialPosition.x;
        state[1] = initialPosition.y;
        state[2] = initialPosition.z;
        state[3] = initialVelocity.x;
        state[4] = initialVelocity.y;
        state[5] = initialVelocity.z;
        state[6] = WrapAngle(initialYawRad);
        state[7] = 0f; // b_ax
        state[8] = 0f; // b_ay
        state[9] = 0f; // b_az
        state[10] = 0f; // b_gyro_z

        covariance = Matrix11x11.Zero;
        covariance[0, 0] = Mathf.Max(0.01f, positionVariance.x);
        covariance[1, 1] = Mathf.Max(0.01f, positionVariance.y);
        covariance[2, 2] = Mathf.Max(0.01f, positionVariance.z);
        covariance[3, 3] = Mathf.Max(0.01f, velocityVariance.x);
        covariance[4, 4] = Mathf.Max(0.01f, velocityVariance.y);
        covariance[5, 5] = Mathf.Max(0.01f, velocityVariance.z);
        covariance[6, 6] = Mathf.Max(0.01f, yawVariance);
        covariance[7, 7] = 0.04f;  // (0.2 m/s^2)^2 initial accel bias uncertainty
        covariance[8, 8] = 0.04f;
        covariance[9, 9] = 0.04f;
        covariance[10, 10] = 0.0025f; // (0.05 rad/s)^2 initial gyro bias uncertainty

        lastPredictionTime = timestamp;
        hasPredictionTimestamp = true;
        status = EstimatorStatus.Nominal;
        gpsState = GpsFixState.Fix3D;
        acceptedMeasurements = 0;
        rejectedMeasurements = 0;
        predictionCount = 0;
    }

    /// <summary>
    /// Resets the filter to uninitialized state.
    /// </summary>
    public void Reset()
    {
        state = Vector11.Zero;
        covariance = Matrix11x11.Identity;
        for (int i = 0; i < 11; i++) covariance[i, i] = 9999f;
        lastPredictionTime = -1f;
        hasPredictionTimestamp = false;
        status = EstimatorStatus.Uninitialized;
        gpsState = GpsFixState.NoFix;
        acceptedMeasurements = 0;
        rejectedMeasurements = 0;
        predictionCount = 0;
    }

    /// <summary>
    /// Time-propagation step driven by high-rate IMU observations.
    /// Integrates specific force and body rates into world position, velocity, and heading.
    /// </summary>
    public bool Predict(ImuMeasurement imu, float currentTime)
    {
        if (!imu.IsValid)
        {
            return false;
        }

        if (status == EstimatorStatus.Uninitialized)
        {
            // Initializing orientation and baseline from IMU at rest
            float initYaw = 0f;
            Initialize(Vector3.zero, Vector3.zero, initYaw, Vector3.one * 100f, Vector3.one * 10f, 1.0f, currentTime);
            status = EstimatorStatus.Initializing;
            return true;
        }

        float dt = hasPredictionTimestamp ? (currentTime - lastPredictionTime) : 0.01f;
        lastPredictionTime = currentTime;
        hasPredictionTimestamp = true;

        if (dt <= 0.0001f || dt > 1.0f)
        {
            dt = 0.01f; // Clamp anomalous time steps
        }

        float yaw = state[6];
        float cosYaw = Mathf.Cos(yaw);
        float sinYaw = Mathf.Sin(yaw);

        // 1. Remove estimated body accelerometer bias
        float fx = imu.LinearAcceleration.x - state[7];
        float fy = imu.LinearAcceleration.y - state[8];
        float fz = imu.LinearAcceleration.z - state[9];

        // 2. Rotate body specific force into World frame (R_B_W) and add gravity
        // R_B_W: [ cos(yaw) 0 sin(yaw); 0 1 0; -sin(yaw) 0 cos(yaw) ]
        float axWorld = cosYaw * fx + sinYaw * fz;
        float ayWorld = fy - 9.81f; // Specific force balances gravity, so a_world = f_world + g (g = -9.81)
        float azWorld = -sinYaw * fx + cosYaw * fz;

        // 3. Propagate kinematic state vector
        state[0] += state[3] * dt + 0.5f * axWorld * dt * dt; // p_x
        state[1] += state[4] * dt + 0.5f * ayWorld * dt * dt; // p_y
        state[2] += state[5] * dt + 0.5f * azWorld * dt * dt; // p_z

        state[3] += axWorld * dt; // v_x
        state[4] += ayWorld * dt; // v_y
        state[5] += azWorld * dt; // v_z

        // 4. Propagate yaw heading with unbiased gyro rate
        float gyroYawRateUnbiased = imu.AngularVelocity.y - state[10];
        state[6] = WrapAngle(state[6] + gyroYawRateUnbiased * dt);

        // Biases state[7..10] remain constant under random walk prediction

        // 5. Construct State Transition Jacobian Matrix F (11x11)
        Matrix11x11 F = Matrix11x11.Identity;
        F[0, 3] = dt;
        F[1, 4] = dt;
        F[2, 5] = dt;

        // Partial derivatives w.r.t yaw (state[6]): d(a_x)/d(yaw) = a_z_specific, d(a_z)/d(yaw) = -a_x_specific
        float dAx_dYaw = -sinYaw * fx + cosYaw * fz;
        float dAz_dYaw = -cosYaw * fx - sinYaw * fz;

        F[0, 6] = 0.5f * dAx_dYaw * dt * dt;
        F[2, 6] = 0.5f * dAz_dYaw * dt * dt;
        F[3, 6] = dAx_dYaw * dt;
        F[5, 6] = dAz_dYaw * dt;

        // Partial derivatives w.r.t accelerometer biases (state[7..9]): d(a_world)/d(b_a) = -R_B_W
        F[3, 7] = -cosYaw * dt;
        F[3, 9] = -sinYaw * dt;
        F[4, 8] = -dt;
        F[5, 7] = sinYaw * dt;
        F[5, 9] = -cosYaw * dt;

        // Partial derivative w.r.t gyro yaw bias (state[10]): d(yaw_dot)/d(b_gyro_z) = -1
        F[6, 10] = -dt;

        // 6. Construct Process Noise Covariance Matrix Q (11x11)
        float qPos = 0.25f * (accelProcessNoiseSigma * accelProcessNoiseSigma) * (dt * dt * dt * dt);
        float qVel = (accelProcessNoiseSigma * accelProcessNoiseSigma) * (dt * dt);
        float qYaw = (gyroProcessNoiseSigma * gyroProcessNoiseSigma) * (dt * dt);
        float qAccelBias = (accelBiasDriftSigma * accelBiasDriftSigma) * dt;
        float qGyroBias = (gyroBiasDriftSigma * gyroBiasDriftSigma) * dt;

        processNoiseQ = Matrix11x11.Zero;
        processNoiseQ[0, 0] = qPos; processNoiseQ[1, 1] = qPos; processNoiseQ[2, 2] = qPos;
        processNoiseQ[3, 3] = qVel; processNoiseQ[4, 4] = qVel; processNoiseQ[5, 5] = qVel;
        processNoiseQ[6, 6] = qYaw;
        processNoiseQ[7, 7] = qAccelBias; processNoiseQ[8, 8] = qAccelBias; processNoiseQ[9, 9] = qAccelBias;
        processNoiseQ[10, 10] = qGyroBias;

        // 7. Propagate Covariance: P = F * P * F^T + Q
        Matrix11x11.PropagateCovariance(in F, in covariance, in processNoiseQ, ref covariance);

        predictionCount++;
        ValidateFilterHealth();
        return true;
    }

    /// <summary>
    /// Corrects estimated state and covariance using a valid satellite navigation (GPS) fix.
    /// Performs sequential scalar Joseph-form updates for position (X, Y, Z) and velocity (Vx, Vy, Vz).
    /// </summary>
    public bool CorrectGps(GpsMeasurement gps)
    {
        if (!gps.IsValid)
        {
            gpsState = GpsFixState.NoFix;
            if (status == EstimatorStatus.Nominal)
            {
                status = EstimatorStatus.Degraded; // Fall back to dead-reckoning
            }
            return false;
        }

        gpsState = gps.FixQuality;

        if (status == EstimatorStatus.Uninitialized || status == EstimatorStatus.Initializing)
        {
            float initYaw = (gps.Velocity.sqrMagnitude > 0.25f)
                ? Mathf.Atan2(gps.Velocity.x, gps.Velocity.z)
                : 0f;

            Initialize(
                gps.Position,
                gps.Velocity,
                initYaw,
                gps.PositionVariance,
                gps.VelocityVariance,
                0.2f,
                gps.Timestamp);
            return true;
        }

        // Sequential scalar corrections for Position X, Y, Z
        bool pxOk = CorrectScalar(0, gps.Position.x, gps.PositionVariance.x);
        bool pyOk = CorrectScalar(1, gps.Position.y, gps.PositionVariance.y);
        bool pzOk = CorrectScalar(2, gps.Position.z, gps.PositionVariance.z);

        // Sequential scalar corrections for Velocity Vx, Vy, Vz
        bool vxOk = CorrectScalar(3, gps.Velocity.x, gps.VelocityVariance.x);
        bool vyOk = CorrectScalar(4, gps.Velocity.y, gps.VelocityVariance.y);
        bool vzOk = CorrectScalar(5, gps.Velocity.z, gps.VelocityVariance.z);

        // Heading observation from ground course if speed is sufficient (> 0.5 m/s)
        if (gps.Velocity.sqrMagnitude >= 0.25f)
        {
            float courseYaw = Mathf.Atan2(gps.Velocity.x, gps.Velocity.z);
            float courseVar = 0.04f; // ~11 deg std dev
            CorrectScalar(6, courseYaw, courseVar, isAngle: true);
        }

        if (pxOk && pyOk && pzOk && vxOk && vyOk && vzOk)
        {
            if (status == EstimatorStatus.Degraded)
            {
                status = EstimatorStatus.Nominal;
            }
            return true;
        }

        return false;
    }

    /// <summary>
    /// Corrects estimated altitude (Y) using a digital barometric altimeter observation.
    /// </summary>
    public bool CorrectBaro(AltimeterMeasurement baro)
    {
        if (!baro.IsValid || status == EstimatorStatus.Uninitialized)
        {
            return false;
        }

        return CorrectScalar(1, baro.Altitude, baro.AltitudeVariance);
    }

    /// <summary>
    /// Performs a single-state scalar measurement correction using Joseph-form covariance updates
    /// with Mahalanobis innovation outlier gating.
    /// </summary>
    private bool CorrectScalar(
        int stateIndex,
        float measuredValue,
        float measurementVariance,
        bool isAngle = false)
    {
        if (!float.IsFinite(measuredValue) || measurementVariance <= 0f)
        {
            return false;
        }

        float stateVal = state[stateIndex];
        float innovation = measuredValue - stateVal;
        if (isAngle)
        {
            innovation = WrapAngle(innovation);
        }

        float stateVariance = covariance[stateIndex, stateIndex];
        float innovationVarianceS = stateVariance + measurementVariance;

        if (innovationVarianceS <= 1e-9f || !float.IsFinite(innovationVarianceS))
        {
            return false;
        }

        // Mahalanobis Innovation Gating: d^2 = y^2 / S
        float d2 = (innovation * innovation) / innovationVarianceS;
        lastInnovationDistanceSq = d2;

        if (d2 > mahalanobisGateThresholdSq)
        {
            rejectedMeasurements++;
            return false;
        }

        // Kalman Gain: K_i = P[i, stateIndex] / S
        Vector11 K = Vector11.Zero;
        float invS = 1.0f / innovationVarianceS;
        for (int i = 0; i < 11; i++)
        {
            K[i] = covariance[i, stateIndex] * invS;
        }

        // State Update: x = x + K * y
        for (int i = 0; i < 11; i++)
        {
            state[i] += K[i] * innovation;
        }

        // Normalize yaw angle
        state[6] = WrapAngle(state[6]);

        // Joseph-Form Covariance Update: P_new = (I-KH) P (I-KH)^T + K * r * K^T
        Matrix11x11.UpdateJosephScalar(in covariance, in K, stateIndex, measurementVariance, ref covariance);

        acceptedMeasurements++;
        ValidateFilterHealth();
        return true;
    }

    private void ValidateFilterHealth()
    {
        if (!state.IsFinite() || !covariance.IsFinite())
        {
            status = EstimatorStatus.Failed;
            return;
        }

        // Check for extreme covariance collapse or explosion
        float posVarMax = Mathf.Max(covariance[0, 0], Mathf.Max(covariance[1, 1], covariance[2, 2]));
        if (posVarMax > 100000f)
        {
            status = EstimatorStatus.Degraded;
        }
    }

    /// <summary>
    /// Constructs an immutable EstimatedState snapshot representing the current onboard belief.
    /// </summary>
    public EstimatedState GetEstimatedState(float timestamp)
    {
        Vector3 pos = new Vector3(state[0], state[1], state[2]);
        Vector3 vel = new Vector3(state[3], state[4], state[5]);
        float yawDeg = state[6] * Mathf.Rad2Deg;
        if (yawDeg < 0f) yawDeg += 360f;

        Vector3 accelBias = new Vector3(state[7], state[8], state[9]);
        float gyroBias = state[10];

        Vector3 posVar = new Vector3(covariance[0, 0], covariance[1, 1], covariance[2, 2]);
        Vector3 velVar = new Vector3(covariance[3, 3], covariance[4, 4], covariance[5, 5]);
        float yawVar = covariance[6, 6];

        return new EstimatedState(
            pos,
            vel,
            yawDeg,
            0f, // Pitch attitude
            accelBias,
            gyroBias,
            posVar,
            velVar,
            yawVar,
            timestamp,
            status,
            gpsState);
    }

    public static float WrapAngle(float angleRad)
    {
        while (angleRad > Mathf.PI) angleRad -= 2.0f * Mathf.PI;
        while (angleRad < -Mathf.PI) angleRad += 2.0f * Mathf.PI;
        return angleRad;
    }
}
