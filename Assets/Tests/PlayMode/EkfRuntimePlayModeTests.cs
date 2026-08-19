using System.Collections;
using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

/// <summary>
/// Phase 11.2.3 PlayMode Runtime Lifecycle Integration Tests.
/// Executes live frame updates across Unity game loop (Awake/Start/Update/FixedUpdate)
/// validating real sensor sampling, EKF convergence, GPS dropouts, and outlier gating.
/// </summary>
public class EkfRuntimePlayModeTests
{
    private GameObject uavObj;

    [TearDown]
    public void TearDown()
    {
        if (uavObj != null)
        {
            Object.Destroy(uavObj);
        }
    }

    [UnityTest]
    public IEnumerator EkfPlayMode_RuntimeLifecycle_SamplesSensorsAndConvergesNominalState()
    {
        Vector3 spawnPos = new Vector3(8f, 1.5f, 12f);
        uavObj = GameManagerBootstrapper.CreateUav(spawnPos);

        SimulatedGpsSensor gps = uavObj.GetComponent<SimulatedGpsSensor>();
        SimulatedImuSensor imu = uavObj.GetComponent<SimulatedImuSensor>();
        SimulatedBaroAltimeter baro = uavObj.GetComponent<SimulatedBaroAltimeter>();
        EkfStateProvider ekf = uavObj.GetComponent<EkfStateProvider>();
        StateEstimationDiagnostics diag = uavObj.GetComponent<StateEstimationDiagnostics>();
        PathFollower pf = uavObj.GetComponent<PathFollower>();

        Assert.IsNotNull(gps, "UAV must be equipped with SimulatedGpsSensor!");
        Assert.IsNotNull(imu, "UAV must be equipped with SimulatedImuSensor!");
        Assert.IsNotNull(baro, "UAV must be equipped with SimulatedBaroAltimeter!");
        Assert.IsNotNull(ekf, "UAV must be equipped with EkfStateProvider!");
        Assert.IsNull(uavObj.GetComponent<GroundTruthStateProvider>(), "GroundTruthStateProvider must NOT be on runtime UAV!");

        // Run simulation for 0.4 seconds across Unity frames
        yield return new WaitForSeconds(0.4f);

        // Verify sensors have sampled and published at runtime
        Assert.IsTrue(gps.CurrentMeasurement.IsValid, "GPS must have published valid runtime measurement!");
        Assert.IsTrue(imu.CurrentMeasurement.IsValid, "IMU must have published valid runtime measurement!");
        Assert.IsTrue(baro.CurrentMeasurement.IsValid, "Barometer must have published valid runtime measurement!");

        // Verify EKF convergence
        Assert.IsTrue(ekf.IsEstimatorReady, "EKF must be ready and publishing!");
        Assert.AreEqual(EstimatorStatus.Nominal, ekf.Status, "EKF must achieve Nominal status!");

        EstimatedState state = ekf.CurrentState;
        Assert.IsTrue(float.IsFinite(state.Position.x));
        Assert.IsTrue(float.IsFinite(state.Position.y));
        Assert.IsTrue(float.IsFinite(state.Position.z));
        Assert.AreEqual(spawnPos.x, state.Position.x, 1.5f);
        Assert.AreEqual(spawnPos.y, state.Position.y, 1.5f);
        Assert.AreEqual(spawnPos.z, state.Position.z, 1.5f);

        // Verify PathFollower consumes EstimatedState
        Assert.IsTrue(float.IsFinite(pf.TargetWaypoint.x));
    }

    [UnityTest]
    public IEnumerator EkfPlayMode_GpsDropoutAndRecovery_MaintainsEstimatorContinuity()
    {
        uavObj = GameManagerBootstrapper.CreateUav(new Vector3(0f, 1f, 0f));
        SimulatedGpsSensor gps = uavObj.GetComponent<SimulatedGpsSensor>();
        EkfStateProvider ekf = uavObj.GetComponent<EkfStateProvider>();

        // 1. Initial convergence
        yield return new WaitForSeconds(0.3f);
        Assert.AreEqual(EstimatorStatus.Nominal, ekf.Status);
        float initialStdDev = ekf.HorizontalPositionStdDev;

        // 2. GPS Outage (simulated tunnel / jamming)
        gps.SetHealth(SensorHealth.Failed, GpsFixState.NoFix);
        yield return new WaitForSeconds(0.5f);

        // During GPS outage, EKF dead-reckons; covariance must expand
        float outageStdDev = ekf.HorizontalPositionStdDev;
        Assert.Greater(outageStdDev, initialStdDev, "Covariance must grow during GPS outage!");
        Assert.IsTrue(ekf.CurrentState.IsValid, "State must remain valid during dead reckoning!");

        // 3. GPS Recovery
        gps.SetHealth(SensorHealth.Healthy, GpsFixState.Fix3D);
        yield return new WaitForSeconds(0.4f);

        // Covariance must shrink upon GPS fix reacquisition
        float recoveredStdDev = ekf.HorizontalPositionStdDev;
        Assert.Less(recoveredStdDev, outageStdDev, "Covariance must reduce upon GPS restoration!");
        Assert.AreEqual(EstimatorStatus.Nominal, ekf.Status);
    }

    [UnityTest]
    public IEnumerator EkfPlayMode_GpsOutlierRejection_GatesAnomalousGlitch()
    {
        uavObj = GameManagerBootstrapper.CreateUav(new Vector3(0f, 1f, 0f));
        EkfStateProvider ekf = uavObj.GetComponent<EkfStateProvider>();

        // Allow initial convergence
        yield return new WaitForSeconds(0.3f);

        Vector3 posBeforeGlitch = ekf.CurrentState.Position;

        // Inject 100-meter GPS multipath glitch
        GpsMeasurement outlier = new GpsMeasurement(
            new Vector3(100f, 1f, 100f),
            Vector3.zero,
            Vector3.one * 0.01f,
            Vector3.one * 0.01f,
            Time.time);

        ekf.HandleGpsUpdated(outlier);
        yield return null;

        // Verify rejection
        Assert.AreEqual(1, ekf.RejectedMeasurements, "100m outlier must be gated by Mahalanobis filter!");
        Vector3 posAfterGlitch = ekf.CurrentState.Position;
        Assert.Less(Vector3.Distance(posBeforeGlitch, posAfterGlitch), 1.0f, "State position must not jump due to rejected outlier!");
    }
}
