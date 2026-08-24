using System;
using NUnit.Framework;
using UnityEngine;

/// <summary>
/// Phase B Navigation Resilience Core Tests (B1–B4).
/// Deterministically validates system-level state estimation resilience through EkfStateProvider:
/// - B1: GPS Outage & IMU Dead Reckoning Continuity with Covariance Expansion
/// - B2: GPS Outlier / Spoofing Rejection via Mahalanobis Innovation Gating
/// - B3: GPS Reacquisition & Smooth State Convergence (Anti-Teleportation)
/// - B4: Navigation Health & Confidence API Tracking
/// </summary>
[TestFixture]
public class NavigationResilienceTests
{
    private GameObject providerGo;
    private EkfStateProvider provider;

    [SetUp]
    public void SetUp()
    {
        providerGo = new GameObject("Test_EkfStateProvider");
        provider = providerGo.AddComponent<EkfStateProvider>();
        provider.InitializeProvider();
    }

    [TearDown]
    public void TearDown()
    {
        if (providerGo != null)
        {
            UnityEngine.Object.DestroyImmediate(providerGo);
        }
    }

    [Test]
    public void B1_GpsOutage_MaintainsDeadReckoningAndExpandsUncertainty()
    {
        // 1. Initial 1.0s of Nominal flight along +Z at 2.0 m/s
        Vector3 trueVel = new Vector3(0f, 0f, 2.0f);
        Vector3 truePos = Vector3.zero;

        float dt = 0.01f; // 100 Hz IMU
        for (int step = 1; step <= 100; step++)
        {
            float t = step * dt;
            truePos = trueVel * t;

            ImuMeasurement imu = new ImuMeasurement(
                new Vector3(0f, 9.81f, 0f),
                Vector3.zero,
                Vector3.one * 0.001f,
                Vector3.one * 0.0001f,
                t);
            provider.HandleImuUpdated(imu);

            if (step % 10 == 0) // 10 Hz GPS
            {
                GpsMeasurement gps = new GpsMeasurement(
                    truePos,
                    trueVel,
                    new Vector3(0.64f, 1.0f, 0.64f),
                    Vector3.one * 0.01f,
                    t);
                provider.HandleGpsUpdated(gps);
            }
        }

        // Verify pre-outage nominal baseline
        EstimatedState preOutageState = provider.CurrentState;
        Assert.AreEqual(EstimatorStatus.Nominal, preOutageState.Status, "Estimator must be Nominal before outage.");
        Assert.AreEqual(GpsFixState.Fix3D, preOutageState.GpsState, "GPS state must be Fix3D before outage.");
        float preOutageSigma = preOutageState.HorizontalPositionStandardDeviation;
        Assert.Greater(preOutageSigma, 0.01f, "Pre-outage position uncertainty must be positive.");

        // 2. 3.0s sustained GPS Outage (t = 1.0s -> 4.0s) with pure IMU dead-reckoning
        for (int step = 101; step <= 400; step++)
        {
            float t = step * dt;
            truePos = trueVel * t;

            ImuMeasurement imu = new ImuMeasurement(
                new Vector3(0f, 9.81f, 0f),
                Vector3.zero,
                Vector3.one * 0.001f,
                Vector3.one * 0.0001f,
                t);
            provider.HandleImuUpdated(imu);
        }

        // 3. Verify post-outage state and covariance properties
        EstimatedState postOutageState = provider.CurrentState;

        // [DERIVED]: Watchdog timeout (> 0.50s) triggers Degraded status and NoFix
        Assert.AreEqual(EstimatorStatus.Degraded, postOutageState.Status, "Estimator must transition to Degraded during outage.");
        Assert.AreEqual(GpsFixState.NoFix, postOutageState.GpsState, "GpsState must be NoFix during outage.");

        // [DERIVED]: Positional uncertainty expands after sustained dead-reckoning
        float postOutageSigma = postOutageState.HorizontalPositionStandardDeviation;
        Assert.Greater(postOutageSigma, preOutageSigma, "Position uncertainty must increase over 3s GPS outage.");

        // Numerical stability assertions
        Assert.IsTrue(float.IsFinite(postOutageState.Position.x) && float.IsFinite(postOutageState.Position.z), "Position must remain finite.");
        Assert.IsTrue(float.IsFinite(postOutageState.Velocity.z), "Velocity must remain finite.");
        Assert.Greater(postOutageState.Position.z, 7.0f, "Dead reckoning must advance UAV forward along +Z.");
        Assert.Less(postOutageState.Position.z, 9.0f, "Dead reckoning position must remain close to true 8.0m trajectory.");

        // Covariance symmetry check
        Matrix11x11 P = provider.EkfCore.CovarianceMatrix;
        for (int r = 0; r < 11; r++)
        {
            for (int c = 0; c < 11; c++)
            {
                Assert.AreEqual(P[r, c], P[c, r], 1e-5f, $"Covariance matrix must remain symmetric at [{r}, {c}].");
            }
        }
    }

    [Test]
    public void B2_GpsOutlier_RejectedByMahalanobisGate()
    {
        // 1. Establish nominal tracking baseline for 1.0s
        Vector3 trueVel = new Vector3(0f, 0f, 2.0f);
        Vector3 truePos = Vector3.zero;
        float dt = 0.01f;

        for (int step = 1; step <= 100; step++)
        {
            float t = step * dt;
            truePos = trueVel * t;

            ImuMeasurement imu = new ImuMeasurement(
                new Vector3(0f, 9.81f, 0f),
                Vector3.zero,
                Vector3.one * 0.001f,
                Vector3.one * 0.0001f,
                t);
            provider.HandleImuUpdated(imu);

            if (step % 10 == 0)
            {
                GpsMeasurement gps = new GpsMeasurement(
                    truePos,
                    trueVel,
                    new Vector3(0.64f, 1.0f, 0.64f),
                    Vector3.one * 0.01f,
                    t);
                provider.HandleGpsUpdated(gps);
            }
        }

        int preRejections = provider.RejectedMeasurements;
        int preAccepted = provider.AcceptedMeasurements;
        Vector3 preOutlierPos = provider.CurrentState.Position;

        // 2. Inject a 10m spoofing/multipath outlier at t = 1.1s (d^2 ~ 147 >> 16.0 gate)
        float tOutlier = 1.10f;
        Vector3 spoofedPos = truePos + new Vector3(10.0f, 0f, 0f);
        GpsMeasurement outlierGps = new GpsMeasurement(
            spoofedPos,
            trueVel,
            new Vector3(0.64f, 1.0f, 0.64f),
            Vector3.one * 0.01f,
            tOutlier);

        bool outlierAccepted = provider.EkfCore.CorrectGps(outlierGps);
        provider.PublishState(tOutlier);

        // [DERIVED]: Outlier px is rejected by Mahalanobis gate, causing CorrectGps to return false
        Assert.IsFalse(outlierAccepted, "CorrectGps must return false when at least one required scalar dimension is rejected.");
        Assert.AreEqual(preRejections + 1, provider.RejectedMeasurements, "Rejected measurements count must increment by exactly 1 for the corrupted px scalar.");

        // [DERIVED]: State estimate does not jump to the corrupted 10m position
        EstimatedState stateAfterOutlier = provider.CurrentState;
        Assert.Less(Mathf.Abs(stateAfterOutlier.Position.x - preOutlierPos.x), 0.10f, "Estimator position must not jump toward outlier.");

        // 3. Verify normal GPS fixes are accepted immediately afterward
        float tNext = 1.20f;
        Vector3 nextTruePos = trueVel * tNext;
        GpsMeasurement validGps = new GpsMeasurement(
            nextTruePos,
            trueVel,
            new Vector3(0.64f, 1.0f, 0.64f),
            Vector3.one * 0.01f,
            tNext);

        bool nextAccepted = provider.EkfCore.CorrectGps(validGps);
        provider.PublishState(tNext);
        Assert.IsTrue(nextAccepted, "Normal GPS measurement must be accepted.");
    }

    [Test]
    public void B3_GpsReacquisition_SmoothlyConvergesState()
    {
        // 1. Initial 1.0s nominal flight
        Vector3 trueVel = new Vector3(0f, 0f, 2.0f);
        float dt = 0.01f;

        for (int step = 1; step <= 100; step++)
        {
            float t = step * dt;
            ImuMeasurement imu = new ImuMeasurement(new Vector3(0f, 9.81f, 0f), Vector3.zero, Vector3.one * 0.001f, Vector3.one * 0.0001f, t);
            provider.HandleImuUpdated(imu);
            if (step % 10 == 0)
            {
                provider.HandleGpsUpdated(new GpsMeasurement(trueVel * t, trueVel, new Vector3(0.64f, 1.0f, 0.64f), Vector3.one * 0.01f, t));
            }
        }

        // 2. 3.0s GPS Outage (t = 1.0s -> 4.0s)
        for (int step = 101; step <= 400; step++)
        {
            float t = step * dt;
            ImuMeasurement imu = new ImuMeasurement(new Vector3(0f, 9.81f, 0f), Vector3.zero, Vector3.one * 0.001f, Vector3.one * 0.0001f, t);
            provider.HandleImuUpdated(imu);
        }

        EstimatedState outageEndState = provider.CurrentState;
        Assert.AreEqual(EstimatorStatus.Degraded, outageEndState.Status);
        float outageEndSigma = outageEndState.HorizontalPositionStandardDeviation;
        Vector3 outageEndPosErr = (trueVel * 4.0f) - outageEndState.Position;

        // 3. Restore valid GPS updates for 2.0s (t = 4.0s -> 6.0s)
        float maxVelocityDeltaPerFrame = 0f;
        Vector3 lastVel = outageEndState.Velocity;

        for (int step = 401; step <= 600; step++)
        {
            float t = step * dt;
            Vector3 truePos = trueVel * t;

            ImuMeasurement imu = new ImuMeasurement(new Vector3(0f, 9.81f, 0f), Vector3.zero, Vector3.one * 0.001f, Vector3.one * 0.0001f, t);
            provider.HandleImuUpdated(imu);

            if (step % 10 == 0)
            {
                GpsMeasurement gps = new GpsMeasurement(truePos, trueVel, new Vector3(0.64f, 1.0f, 0.64f), Vector3.one * 0.01f, t);
                provider.HandleGpsUpdated(gps);
            }

            Vector3 currentVel = provider.CurrentState.Velocity;
            float velDelta = (currentVel - lastVel).magnitude;
            if (velDelta > maxVelocityDeltaPerFrame)
            {
                maxVelocityDeltaPerFrame = velDelta;
            }
            lastVel = currentVel;
        }

        EstimatedState recoveredState = provider.CurrentState;
        Vector3 recoveredPosErr = (trueVel * 6.0f) - recoveredState.Position;
        float recoveredSigma = recoveredState.HorizontalPositionStandardDeviation;

        // [DERIVED]: Status returns to Nominal and Fix3D
        Assert.AreEqual(EstimatorStatus.Nominal, recoveredState.Status, "Estimator must return to Nominal after reacquisition.");
        Assert.AreEqual(GpsFixState.Fix3D, recoveredState.GpsState, "GPS state must return to Fix3D.");

        // [DERIVED]: Position error and covariance decrease relative to outage end state
        Assert.LessOrEqual(recoveredPosErr.magnitude, outageEndPosErr.magnitude + 0.01f, "Position error must not increase upon reacquisition.");
        Assert.Less(recoveredSigma, outageEndSigma, "Position uncertainty must decrease upon reacquisition.");

        // [DERIVED]: Velocity remains continuous without impulse step spikes
        Assert.Less(maxVelocityDeltaPerFrame, 0.50f, $"Max velocity step per frame ({maxVelocityDeltaPerFrame:F3} m/s) must be physically bounded.");
    }

    [Test]
    public void B4_NavigationHealthAPI_ReflectsEstimatorState()
    {
        Vector3 trueVel = new Vector3(0f, 0f, 2.0f);
        float dt = 0.01f;

        // 1. Nominal Phase: 1.0s with healthy GPS
        for (int step = 1; step <= 100; step++)
        {
            float t = step * dt;
            provider.HandleImuUpdated(new ImuMeasurement(new Vector3(0f, 9.81f, 0f), Vector3.zero, Vector3.one * 0.001f, Vector3.one * 0.0001f, t));
            if (step % 10 == 0)
            {
                provider.HandleGpsUpdated(new GpsMeasurement(trueVel * t, trueVel, new Vector3(0.64f, 1.0f, 0.64f), Vector3.one * 0.01f, t));
            }
        }

        EstimatedState nominalState = provider.CurrentState;
        Assert.GreaterOrEqual(nominalState.NavigationConfidence, 0.85f, "Nominal navigation confidence must be high (>= 0.85).");
        Assert.LessOrEqual(nominalState.DeadReckoningDuration, 0.15f, "Dead reckoning duration must be near zero during nominal GPS.");

        // 2. Outage Phase: 2.5s without GPS (t = 1.0s -> 3.5s)
        for (int step = 101; step <= 350; step++)
        {
            float t = step * dt;
            provider.HandleImuUpdated(new ImuMeasurement(new Vector3(0f, 9.81f, 0f), Vector3.zero, Vector3.one * 0.001f, Vector3.one * 0.0001f, t));
        }

        EstimatedState outageState = provider.CurrentState;
        Assert.AreEqual(EstimatorStatus.Degraded, outageState.Status);
        Assert.Less(outageState.NavigationConfidence, nominalState.NavigationConfidence, "Confidence must drop during outage.");
        Assert.Less(outageState.NavigationConfidence, 0.70f, "Degraded confidence must be <= 0.70.");
        Assert.AreEqual(2.50f, outageState.DeadReckoningDuration, 0.15f, "Dead reckoning duration must reflect elapsed outage time.");

        // 3. Spoofed GPS Outlier: Must NOT reset dead reckoning duration
        GpsMeasurement spoofedGps = new GpsMeasurement(
            new Vector3(50.0f, 0f, 0f),
            trueVel,
            new Vector3(0.64f, 1.0f, 0.64f),
            Vector3.one * 0.01f,
            3.55f);
        provider.HandleGpsUpdated(spoofedGps);

        EstimatedState stateAfterSpoof = provider.CurrentState;
        Assert.GreaterOrEqual(stateAfterSpoof.DeadReckoningDuration, 2.50f, "Rejected spoof measurement must NOT reset dead reckoning duration.");

        // 4. Valid GPS Restoration at t = 4.0s
        for (int step = 351; step <= 400; step++)
        {
            float t = step * dt;
            provider.HandleImuUpdated(new ImuMeasurement(new Vector3(0f, 9.81f, 0f), Vector3.zero, Vector3.one * 0.001f, Vector3.one * 0.0001f, t));
            if (step % 10 == 0)
            {
                provider.HandleGpsUpdated(new GpsMeasurement(trueVel * t, trueVel, new Vector3(0.64f, 1.0f, 0.64f), Vector3.one * 0.01f, t));
            }
        }

        EstimatedState restoredState = provider.CurrentState;
        Assert.AreEqual(EstimatorStatus.Nominal, restoredState.Status);
        Assert.Greater(restoredState.NavigationConfidence, outageState.NavigationConfidence, "Confidence must recover upon GPS fix.");
        Assert.LessOrEqual(restoredState.DeadReckoningDuration, 0.15f, "Dead reckoning duration must reset upon valid GPS fix.");
    }
}
