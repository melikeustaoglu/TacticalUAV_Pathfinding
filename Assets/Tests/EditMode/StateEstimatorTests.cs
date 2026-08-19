using System;
using NUnit.Framework;
using UnityEngine;

/// <summary>
/// Phase 11.2.2 State Estimator Core Engine Quantitative Tests.
/// Validates 11-state EKF prediction, correction, Joseph covariance updates, outlier gating,
/// numerical stability, and statistical RMSE / NEES convergence on synthetic trajectories.
/// </summary>
[TestFixture]
public class StateEstimatorTests
{
    private ExtendedKalmanFilter ekf;

    [SetUp]
    public void SetUp()
    {
        ekf = new ExtendedKalmanFilter();
    }

    [Test]
    public void Initialization_FromGps_ProducesValidEstimate()
    {
        Vector3 initialPos = new Vector3(15.0f, 2.0f, 25.0f);
        Vector3 initialVel = new Vector3(2.0f, 0f, 1.0f);
        Vector3 posVar = new Vector3(0.64f, 1.0f, 0.64f);
        Vector3 velVar = Vector3.one * 0.01f;

        GpsMeasurement gps = new GpsMeasurement(initialPos, initialVel, posVar, velVar, 1.0f);
        bool initialized = ekf.CorrectGps(gps);

        Assert.IsTrue(initialized);
        Assert.IsTrue(ekf.IsInitialized);
        Assert.AreEqual(EstimatorStatus.Nominal, ekf.Status);

        EstimatedState state = ekf.GetEstimatedState(1.0f);
        Assert.AreEqual(initialPos.x, state.Position.x, 0.01f);
        Assert.AreEqual(initialPos.y, state.Position.y, 0.01f);
        Assert.AreEqual(initialPos.z, state.Position.z, 0.01f);
        Assert.AreEqual(initialVel.x, state.Velocity.x, 0.01f);
    }

    [Test]
    public void Prediction_WithoutCorrection_PropagatesState()
    {
        // Initialize at origin with forward velocity +2 m/s along Z
        ekf.Initialize(Vector3.zero, new Vector3(0f, 0f, 2.0f), 0f, Vector3.one * 0.1f, Vector3.one * 0.01f, 0.1f, 0f);

        // Constant speed forward (specific force = [0, 9.81, 0] in body frame to cancel gravity)
        ImuMeasurement imu = new ImuMeasurement(new Vector3(0f, 9.81f, 0f), Vector3.zero, Vector3.one * 0.001f, Vector3.one * 0.0001f, 0.1f);

        for (int i = 1; i <= 10; i++)
        {
            ekf.Predict(imu, i * 0.1f);
        }

        EstimatedState state = ekf.GetEstimatedState(1.0f);
        // After 1.0s at 2.0 m/s, position Z should be ~2.0m
        Assert.AreEqual(2.0f, state.Position.z, 0.05f);
        Assert.AreEqual(2.0f, state.Velocity.z, 0.05f);
    }

    [Test]
    public void AccelerometerBias_IsEstimated()
    {
        // Inject a constant +0.5 m/s^2 body Y specific force offset (accel bias)
        ekf.Initialize(new Vector3(0f, 5f, 0f), Vector3.zero, 0f, Vector3.one * 0.1f, Vector3.one * 0.01f, 0.1f, 0f);

        // Persistent IMU reporting extra 0.5 m/s^2 in Y (specific force = 9.81 + 0.5 = 10.31)
        ImuMeasurement imu = new ImuMeasurement(new Vector3(0f, 10.31f, 0f), Vector3.zero, Vector3.one * 0.001f, Vector3.one * 0.0001f, 0.01f);

        // True position is stationary at altitude 5.0m
        for (int step = 1; step <= 200; step++)
        {
            float t = step * 0.02f;
            ekf.Predict(imu, t);

            if (step % 5 == 0) // 10 Hz GPS position & velocity correction
            {
                GpsMeasurement gps = new GpsMeasurement(new Vector3(0f, 5f, 0f), Vector3.zero, Vector3.one * 0.04f, Vector3.one * 0.01f, t);
                ekf.CorrectGps(gps);
            }
        }

        EstimatedState state = ekf.GetEstimatedState(4.0f);
        // Estimated bias Y should track toward +0.5 m/s^2
        Assert.AreEqual(0.5f, state.AccelerometerBias.y, 0.25f, "EKF must estimate positive accelerometer bias from persistent innovation!");
    }

    [Test]
    public void GyroBias_IsEstimated()
    {
        ekf.Initialize(Vector3.zero, Vector3.zero, 0f, Vector3.one * 0.1f, Vector3.one * 0.01f, 0.1f, 0f);

        // Constant gyro bias: gyro reads 0.1 rad/s while true heading remains 0
        ImuMeasurement imu = new ImuMeasurement(new Vector3(0f, 9.81f, 0f), new Vector3(0f, 0.1f, 0f), Vector3.one * 0.001f, Vector3.one * 0.0001f, 0.01f);

        for (int step = 1; step <= 200; step++)
        {
            float t = step * 0.02f;
            ekf.Predict(imu, t);

            if (step % 5 == 0)
            {
                // Moving forward along Z (yaw = 0)
                GpsMeasurement gps = new GpsMeasurement(new Vector3(0f, 1f, t * 2f), new Vector3(0f, 0f, 2f), Vector3.one * 0.04f, Vector3.one * 0.01f, t);
                ekf.CorrectGps(gps);
            }
        }

        EstimatedState state = ekf.GetEstimatedState(4.0f);
        Assert.AreEqual(0.1f, state.GyroYawBias, 0.06f, "EKF must estimate gyro yaw bias!");
    }

    [Test]
    public void GPSPositionCorrection_ReducesPositionError()
    {
        // Initial state with 5.0m position error
        ekf.Initialize(new Vector3(5f, 0f, 0f), Vector3.zero, 0f, Vector3.one * 10f, Vector3.one * 1f, 0.5f, 0f);

        // Correct with high-precision GPS at true position (0, 0, 0)
        GpsMeasurement gps = new GpsMeasurement(Vector3.zero, Vector3.zero, Vector3.one * 0.01f, Vector3.one * 0.01f, 0.1f);
        ekf.CorrectGps(gps);

        EstimatedState state = ekf.GetEstimatedState(0.1f);
        Assert.Less(state.Position.x, 0.5f, "GPS correction must substantially reduce initial position offset!");
    }

    [Test]
    public void GPSVelocityCorrection_ReducesVelocityError()
    {
        ekf.Initialize(Vector3.zero, new Vector3(5f, 0f, 0f), 0f, Vector3.one * 1f, Vector3.one * 10f, 0.5f, 0f);

        GpsMeasurement gps = new GpsMeasurement(Vector3.zero, Vector3.zero, Vector3.one * 0.01f, Vector3.one * 0.01f, 0.1f);
        ekf.CorrectGps(gps);

        EstimatedState state = ekf.GetEstimatedState(0.1f);
        Assert.Less(state.Velocity.x, 0.5f, "GPS velocity correction must reduce velocity error!");
    }

    [Test]
    public void BarometerCorrection_ReducesAltitudeError()
    {
        ekf.Initialize(new Vector3(0f, 10f, 0f), Vector3.zero, 0f, Vector3.one * 10f, Vector3.one * 1f, 0.5f, 0f);

        AltimeterMeasurement baro = new AltimeterMeasurement(2.0f, 0f, 0.04f, 0.1f);
        ekf.CorrectBaro(baro);

        EstimatedState state = ekf.GetEstimatedState(0.1f);
        Assert.Less(state.Position.y, 3.0f, "Barometer correction must pull altitude toward measured 2.0m!");
    }

    [Test]
    public void CovariancePrediction_IncreasesUncertaintyDuringDeadReckoning()
    {
        ekf.Initialize(Vector3.zero, Vector3.zero, 0f, Vector3.one * 0.1f, Vector3.one * 0.01f, 0.01f, 0f);

        float initialPosVar = ekf.CovarianceMatrix[0, 0];

        ImuMeasurement imu = new ImuMeasurement(new Vector3(0f, 9.81f, 0f), Vector3.zero, Vector3.one * 0.01f, Vector3.one * 0.001f, 0.1f);
        for (int i = 1; i <= 50; i++)
        {
            ekf.Predict(imu, i * 0.1f);
        }

        float propagatedPosVar = ekf.CovarianceMatrix[0, 0];
        Assert.Greater(propagatedPosVar, initialPosVar * 5.0f, "Position variance must grow monotonically during dead-reckoning!");
    }

    [Test]
    public void MeasurementCorrection_ReducesRelevantCovariance()
    {
        ekf.Initialize(Vector3.zero, Vector3.zero, 0f, Vector3.one * 5.0f, Vector3.one * 1.0f, 0.5f, 0f);

        float priorVar = ekf.CovarianceMatrix[0, 0];
        GpsMeasurement gps = new GpsMeasurement(Vector3.zero, Vector3.zero, Vector3.one * 0.1f, Vector3.one * 0.05f, 0.1f);
        ekf.CorrectGps(gps);

        float posteriorVar = ekf.CovarianceMatrix[0, 0];
        Assert.Less(posteriorVar, priorVar, "Measurement correction must reduce state covariance!");
    }

    [Test]
    public void InnovationGate_RejectsLargeGpsOutlier()
    {
        ekf.Initialize(Vector3.zero, Vector3.zero, 0f, Vector3.one * 0.04f, Vector3.one * 0.01f, 0.01f, 0f);

        // Glitch of 50 meters with small variance (0.01 m^2) -> Mahalanobis distance >> 4 sigma
        GpsMeasurement outlier = new GpsMeasurement(new Vector3(50f, 0f, 0f), Vector3.zero, Vector3.one * 0.01f, Vector3.one * 0.01f, 0.1f);

        bool accepted = ekf.CorrectGps(outlier);
        Assert.IsFalse(accepted, "EKF innovation gate must reject a 50m outlier!");
        Assert.AreEqual(1, ekf.RejectedMeasurementsCount);

        EstimatedState state = ekf.GetEstimatedState(0.1f);
        Assert.AreEqual(0f, state.Position.x, 0.01f, "Rejected outlier must not corrupt state position!");
    }

    [Test]
    public void YawInnovation_IsWrappedCorrectly()
    {
        // UAV heading is +179 deg (+3.124 rad)
        float headingRad = 179.0f * Mathf.Deg2Rad;
        ekf.Initialize(Vector3.zero, new Vector3(0f, 0f, 2f), headingRad, Vector3.one * 0.1f, Vector3.one * 0.01f, 0.1f, 0f);

        // Course reading slightly crosses boundary to -179 deg (-3.124 rad) -> True difference is 2 deg (0.035 rad)
        float courseRad = -179.0f * Mathf.Deg2Rad;
        float diff = ExtendedKalmanFilter.WrapAngle(courseRad - headingRad);

        Assert.AreEqual(2.0f * Mathf.Deg2Rad, Mathf.Abs(diff), 0.01f, "Wrapped angle difference must be 2 degrees!");
    }

    [Test]
    public void Covariance_RemainsSymmetric()
    {
        ekf.Initialize(Vector3.zero, Vector3.zero, 0.5f, Vector3.one * 0.5f, Vector3.one * 0.1f, 0.1f, 0f);
        ImuMeasurement imu = new ImuMeasurement(new Vector3(0.5f, 9.81f, -0.2f), new Vector3(0.01f, 0.05f, 0f), Vector3.one * 0.01f, Vector3.one * 0.001f, 0.02f);
        GpsMeasurement gps = new GpsMeasurement(new Vector3(1f, 1f, 1f), new Vector3(0.5f, 0f, 0.5f), Vector3.one * 0.2f, Vector3.one * 0.05f, 0.02f);

        for (int i = 1; i <= 20; i++)
        {
            ekf.Predict(imu, i * 0.02f);
            ekf.CorrectGps(gps);
        }

        Matrix11x11 P = ekf.CovarianceMatrix;
        for (int r = 0; r < 11; r++)
        {
            for (int c = 0; c < 11; c++)
            {
                Assert.AreEqual(P[r, c], P[c, r], 1e-5f, $"Covariance must be symmetric at [{r}, {c}]!");
            }
        }
    }

    [Test]
    public void CovarianceDiagonal_RemainsPositive()
    {
        ekf.Initialize(Vector3.zero, Vector3.zero, 0f, Vector3.one * 0.1f, Vector3.one * 0.01f, 0.01f, 0f);
        ImuMeasurement imu = new ImuMeasurement(new Vector3(0f, 9.81f, 0f), Vector3.zero, Vector3.one * 0.001f, Vector3.one * 0.0001f, 0.01f);

        for (int i = 1; i <= 100; i++)
        {
            ekf.Predict(imu, i * 0.01f);
        }

        Matrix11x11 P = ekf.CovarianceMatrix;
        for (int i = 0; i < 11; i++)
        {
            Assert.Greater(P[i, i], 0f, $"Covariance diagonal element [{i}, {i}] must remain strictly positive!");
        }
    }

    [Test]
    public void InvalidMeasurement_DoesNotCorruptEstimator()
    {
        ekf.Initialize(new Vector3(1f, 2f, 3f), Vector3.zero, 0f, Vector3.one * 0.1f, Vector3.one * 0.01f, 0.01f, 0f);

        GpsMeasurement invalidGps = GpsMeasurement.Invalid;
        bool corrected = ekf.CorrectGps(invalidGps);
        Assert.IsFalse(corrected);

        EstimatedState state = ekf.GetEstimatedState(0.1f);
        Assert.AreEqual(1f, state.Position.x, 0.001f);
        Assert.AreEqual(2f, state.Position.y, 0.001f);
        Assert.AreEqual(3f, state.Position.z, 0.001f);
    }

    [Test]
    public void EstimatorNeverProducesNaNOrInfinity()
    {
        ekf.Initialize(Vector3.zero, Vector3.zero, 0f, Vector3.one * 0.1f, Vector3.one * 0.01f, 0.01f, 0f);

        ImuMeasurement imu = new ImuMeasurement(new Vector3(0f, 9.81f, 0f), Vector3.zero, Vector3.one * 0.01f, Vector3.one * 0.001f, 0.01f);
        GpsMeasurement gps = new GpsMeasurement(Vector3.one, Vector3.one, Vector3.one * 0.5f, Vector3.one * 0.1f, 0.1f);

        for (int i = 1; i <= 50; i++)
        {
            ekf.Predict(imu, i * 0.01f);
            if (i % 10 == 0) ekf.CorrectGps(gps);
        }

        EstimatedState state = ekf.GetEstimatedState(0.5f);
        Assert.IsTrue(state.IsValid);
        Assert.IsTrue(float.IsFinite(state.Position.x));
        Assert.IsTrue(float.IsFinite(state.Velocity.x));
        Assert.IsTrue(float.IsFinite(state.YawDegrees));
    }

    [Test]
    public void DeterministicReplay_ProducesIdenticalEstimate()
    {
        ExtendedKalmanFilter ekf1 = new ExtendedKalmanFilter();
        ExtendedKalmanFilter ekf2 = new ExtendedKalmanFilter();

        Vector3 initPos = new Vector3(10f, 1f, 10f);
        ekf1.Initialize(initPos, Vector3.zero, 0f, Vector3.one * 0.1f, Vector3.one * 0.01f, 0.1f, 0f);
        ekf2.Initialize(initPos, Vector3.zero, 0f, Vector3.one * 0.1f, Vector3.one * 0.01f, 0.1f, 0f);

        GaussianNoiseGenerator noise = new GaussianNoiseGenerator(555);

        for (int step = 1; step <= 50; step++)
        {
            float t = step * 0.02f;
            Vector3 accelNoise = noise.SampleVector3(0.02f, 0.02f, 0.02f);
            ImuMeasurement imu = new ImuMeasurement(new Vector3(0f, 9.81f, 0f) + accelNoise, Vector3.zero, Vector3.one * 0.001f, Vector3.one * 0.0001f, t);

            ekf1.Predict(imu, t);
            ekf2.Predict(imu, t);

            if (step % 5 == 0)
            {
                GpsMeasurement gps = new GpsMeasurement(initPos + new Vector3(t, 0f, t * 2f), new Vector3(1f, 0f, 2f), Vector3.one * 0.1f, Vector3.one * 0.02f, t);
                ekf1.CorrectGps(gps);
                ekf2.CorrectGps(gps);
            }
        }

        EstimatedState state1 = ekf1.GetEstimatedState(1.0f);
        EstimatedState state2 = ekf2.GetEstimatedState(1.0f);

        Assert.AreEqual(state1.Position.x, state2.Position.x, 1e-6f);
        Assert.AreEqual(state1.Position.z, state2.Position.z, 1e-6f);
        Assert.AreEqual(state1.Velocity.z, state2.Velocity.z, 1e-6f);
    }

    [Test]
    public void NominalTrajectory_RMSEPositionWithinThreshold()
    {
        RunSyntheticFlightSimulation(out float rmsePos, out float rmseVel, out float avgNees);

        // Position RMSE threshold under standard GNSS noise (sigma = 0.8m horizontal, 1.5m vertical)
        // With 10 Hz GPS + 50 Hz IMU fusion, filtered position RMSE is typically 0.20m - 0.35m
        Assert.Less(rmsePos, 0.40f, $"Filter position RMSE ({rmsePos:F3}m) must be under 0.40m!");
    }

    [Test]
    public void NominalTrajectory_RMSEVelocityWithinThreshold()
    {
        RunSyntheticFlightSimulation(out float rmsePos, out float rmseVel, out float avgNees);

        // Velocity RMSE threshold (GPS sigma = 0.1 m/s)
        Assert.Less(rmseVel, 0.15f, $"Filter velocity RMSE ({rmseVel:F3} m/s) must be under 0.15 m/s!");
    }

    [Test]
    public void NEES_RemainsWithinReasonableStatisticalBounds()
    {
        RunSyntheticFlightSimulation(out float rmsePos, out float rmseVel, out float avgNees);

        // For a 3D position error vector, theoretical expectation of NEES is 3.0.
        // A well-tuned filter maintains average NEES between 0.8 and 5.0.
        Assert.Greater(avgNees, 0.5f, $"Average NEES ({avgNees:F2}) indicates overly conservative covariance!");
        Assert.Less(avgNees, 6.0f, $"Average NEES ({avgNees:F2}) indicates overly optimistic covariance!");
    }

    /// <summary>
    /// Deterministic synthetic trajectory simulation harness.
    /// Simulates a 10-second flight along a curved trajectory with known ground-truth state,
    /// generating noisy IMU (50 Hz) and GPS (10 Hz) measurements.
    /// </summary>
    private void RunSyntheticFlightSimulation(out float rmsePos, out float rmseVel, out float avgNees)
    {
        ExtendedKalmanFilter simEkf = new ExtendedKalmanFilter();
        GaussianNoiseGenerator noise = new GaussianNoiseGenerator(42);

        float duration = 10.0f;
        float dt = 0.02f; // 50 Hz IMU
        int steps = (int)(duration / dt);

        float sumSqPosErr = 0f;
        float sumSqVelErr = 0f;
        float sumNees = 0f;
        int sampleCount = 0;

        Vector3 truePos = new Vector3(0f, 2.0f, 0f);
        Vector3 trueVel = new Vector3(0f, 0f, 3.0f); // 3 m/s cruise
        float trueYaw = 0f;

        simEkf.Initialize(truePos, trueVel, trueYaw, Vector3.one * 0.64f, Vector3.one * 0.01f, 0.1f, 0f);

        for (int i = 1; i <= steps; i++)
        {
            float t = i * dt;

            // Simulated circular curve (yaw rate = 0.2 rad/s)
            float yawRate = 0.2f;
            trueYaw += yawRate * dt;
            trueVel = new Vector3(Mathf.Sin(trueYaw) * 3.0f, 0f, Mathf.Cos(trueYaw) * 3.0f);
            truePos += trueVel * dt;

            // True centripetal acceleration in world frame: a_world = [cos(yaw)*3*0.2, 0, -sin(yaw)*3*0.2]
            Vector3 trueWorldAccel = new Vector3(Mathf.Cos(trueYaw) * 3.0f * yawRate, 0f, -Mathf.Sin(trueYaw) * 3.0f * yawRate);

            // True specific force in body frame: R_W_B * (a_world - g)
            Quaternion rot = Quaternion.Euler(0f, trueYaw * Mathf.Rad2Deg, 0f);
            Vector3 trueSpecificForceBody = Quaternion.Inverse(rot) * (trueWorldAccel - new Vector3(0f, -9.81f, 0f));

            // Noisy IMU reading (sigma = 0.05 m/s^2 accel, 0.005 rad/s gyro)
            Vector3 accelNoise = noise.SampleVector3(0.05f, 0.05f, 0.05f);
            Vector3 gyroNoise = noise.SampleVector3(0.005f, 0.005f, 0.005f);
            ImuMeasurement imu = new ImuMeasurement(
                trueSpecificForceBody + accelNoise,
                new Vector3(0f, yawRate, 0f) + gyroNoise,
                Vector3.one * 0.0025f,
                Vector3.one * 0.000025f,
                t);

            simEkf.Predict(imu, t);

            // 10 Hz GPS corrections (every 5 steps)
            if (i % 5 == 0)
            {
                Vector3 posNoise = noise.SampleVector3(0.8f, 1.5f, 0.8f);
                Vector3 velNoise = noise.SampleVector3(0.1f, 0.1f, 0.1f);
                GpsMeasurement gps = new GpsMeasurement(
                    truePos + posNoise,
                    trueVel + velNoise,
                    new Vector3(0.64f, 2.25f, 0.64f),
                    Vector3.one * 0.01f,
                    t);

                simEkf.CorrectGps(gps);
            }

            // Sample estimation error after initial convergence (t > 2.0s)
            if (t >= 2.0f)
            {
                EstimatedState est = simEkf.GetEstimatedState(t);
                Vector3 posErr = truePos - est.Position;
                Vector3 velErr = trueVel - est.Velocity;

                sumSqPosErr += posErr.sqrMagnitude;
                sumSqVelErr += velErr.sqrMagnitude;

                // 3D NEES = (p_err)^T * P_pos^-1 * (p_err) (diagonal approximation)
                float varX = Mathf.Max(0.001f, simEkf.CovarianceMatrix[0, 0]);
                float varY = Mathf.Max(0.001f, simEkf.CovarianceMatrix[1, 1]);
                float varZ = Mathf.Max(0.001f, simEkf.CovarianceMatrix[2, 2]);
                float nees = (posErr.x * posErr.x / varX) + (posErr.y * posErr.y / varY) + (posErr.z * posErr.z / varZ);

                sumNees += nees;
                sampleCount++;
            }
        }

        rmsePos = Mathf.Sqrt(sumSqPosErr / sampleCount);
        rmseVel = Mathf.Sqrt(sumSqVelErr / sampleCount);
        avgNees = sumNees / sampleCount;
    }
}
