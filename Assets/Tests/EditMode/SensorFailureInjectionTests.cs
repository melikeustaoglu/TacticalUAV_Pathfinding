using System.Collections.Generic;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;

/// <summary>
/// Phase 11.4 Sensor Failure Injection & Watchdog Integration Tests.
/// Validates deterministic GPS denial, IMU loss, Barometer failure, recovery, and autonomy integration.
/// </summary>
[TestFixture]
public class SensorFailureInjectionTests
{
    private GameObject uavObj;
    private SimulatedGpsSensor gpsSensor;
    private SimulatedImuSensor imuSensor;
    private SimulatedBaroAltimeter baroSensor;
    private EkfStateProvider ekfProvider;
    private SensorFailureInjector failureInjector;
    private ThreatAssessment threatAssessment;
    private ReplanningController replanningController;
    private PathFollower pathFollower;
    private UAVPerception perception;

    [SetUp]
    public void SetUp()
    {
        uavObj = new GameObject("SensorFailureUAV");
        gpsSensor = uavObj.AddComponent<SimulatedGpsSensor>();
        imuSensor = uavObj.AddComponent<SimulatedImuSensor>();
        baroSensor = uavObj.AddComponent<SimulatedBaroAltimeter>();
        ekfProvider = uavObj.AddComponent<EkfStateProvider>();
        failureInjector = uavObj.AddComponent<SensorFailureInjector>();
        pathFollower = uavObj.AddComponent<PathFollower>();
        perception = uavObj.AddComponent<UAVPerception>();
        threatAssessment = uavObj.AddComponent<ThreatAssessment>();
        replanningController = uavObj.AddComponent<ReplanningController>();

        typeof(SimulatedGpsSensor).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(gpsSensor, null);
        typeof(SimulatedImuSensor).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(imuSensor, null);
        typeof(SimulatedBaroAltimeter).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(baroSensor, null);
        typeof(EkfStateProvider).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(ekfProvider, null);
        typeof(EkfStateProvider).GetMethod("OnEnable", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(ekfProvider, null);
        typeof(SensorFailureInjector).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(failureInjector, null);
        typeof(PathFollower).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(pathFollower, null);
        typeof(UAVPerception).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(perception, null);
        typeof(ThreatAssessment).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(threatAssessment, null);
        typeof(ReplanningController).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(replanningController, null);
    }

    [TearDown]
    public void TearDown()
    {
        if (uavObj != null) Object.DestroyImmediate(uavObj);
    }

    [Test]
    public void SensorFailure_GpsLoss_CeasesGpsCorrectionsAndExpandsCovariance()
    {
        // 1. Initial convergence
        gpsSensor.UpdateFromSimulationState(Vector3.zero, Vector3.zero, 0.0f);
        float initVar = ekfProvider.CurrentState.HorizontalPositionVariance;

        // 2. Inject complete GPS failure
        failureInjector.InjectFailure(SensorType.GPS);

        // 3. Propagate dead-reckoning via IMU for 20 steps (0.2s)
        for (int i = 1; i <= 20; i++)
        {
            float t = i * 0.01f;
            imuSensor.UpdateFromKinematics(Vector3.zero, Vector3.zero, Quaternion.identity, t);
        }

        float deadReckoningVar = ekfProvider.CurrentState.HorizontalPositionVariance;
        Assert.Greater(deadReckoningVar, initVar, "Position variance must expand monotonically during GPS outage!");
    }

    [Test]
    public void SensorFailure_GpsLoss_TransitionsGpsFixStateToNoFix()
    {
        gpsSensor.UpdateFromSimulationState(Vector3.zero, Vector3.zero, 0.0f);
        Assert.AreEqual(GpsFixState.Fix3D, ekfProvider.GpsState);

        // Inject GPS failure
        failureInjector.InjectFailure(SensorType.GPS);

        // Step time past the 0.50s GPS timeout threshold
        float timeoutTime = 0.60f;
        imuSensor.UpdateFromKinematics(Vector3.zero, Vector3.zero, Quaternion.identity, timeoutTime);

        Assert.AreEqual(GpsFixState.NoFix, ekfProvider.GpsState, "GpsFixState must transition to NoFix after timeout!");
    }

    [Test]
    public void SensorFailure_GpsLoss_TransitionsEstimatorStatusToDegraded()
    {
        gpsSensor.UpdateFromSimulationState(Vector3.zero, Vector3.zero, 0.0f);
        Assert.AreEqual(EstimatorStatus.Nominal, ekfProvider.Status);

        failureInjector.InjectFailure(SensorType.GPS);

        // Propagate via IMU until covariance exceeds degradation threshold
        for (int i = 1; i <= 60; i++)
        {
            float t = i * 0.02f;
            imuSensor.UpdateFromKinematics(Vector3.zero, Vector3.zero, Quaternion.identity, t);
        }

        Assert.AreEqual(EstimatorStatus.Degraded, ekfProvider.Status, "EstimatorStatus must transition to Degraded during extended GPS outage!");
    }

    [Test]
    public void SensorFailure_GpsRecovery_RestoresCorrectionsAndContractsCovariance()
    {
        // 1. Initial fix
        gpsSensor.UpdateFromSimulationState(Vector3.zero, Vector3.zero, 0.0f);

        // 2. Outage
        failureInjector.InjectFailure(SensorType.GPS);
        for (int i = 1; i <= 50; i++)
        {
            imuSensor.UpdateFromKinematics(Vector3.zero, Vector3.zero, Quaternion.identity, i * 0.02f);
        }
        float outageStdDev = ekfProvider.HorizontalPositionStdDev;

        // 3. Recovery
        failureInjector.RecoverSensor(SensorType.GPS);
        for (int i = 51; i <= 70; i++)
        {
            float t = i * 0.02f;
            imuSensor.UpdateFromKinematics(Vector3.zero, Vector3.zero, Quaternion.identity, t);
            if (i % 5 == 0) gpsSensor.UpdateFromSimulationState(Vector3.zero, Vector3.zero, t);
        }

        float recoveredStdDev = ekfProvider.HorizontalPositionStdDev;
        Assert.Less(recoveredStdDev, outageStdDev, "Covariance must shrink upon GPS reacquisition!");
        Assert.AreEqual(EstimatorStatus.Nominal, ekfProvider.Status, "EstimatorStatus must return to Nominal after recovery!");
    }

    [Test]
    public void SensorFailure_BaroLoss_ExpandsVerticalUncertainty()
    {
        // 1. Initial healthy state with Baro + GPS
        for (int i = 0; i < 5; i++)
        {
            float t = i * 0.05f;
            imuSensor.UpdateFromKinematics(Vector3.zero, Vector3.zero, Quaternion.identity, t);
            if (i % 2 == 0) gpsSensor.UpdateFromSimulationState(Vector3.zero, Vector3.zero, t);
            baroSensor.UpdateFromSimulationState(0f, 0f, t);
        }

        float nominalVertStdDev = ekfProvider.VerticalPositionStdDev;

        // 2. Inject Barometer failure (GPS continues, Baro ceases)
        failureInjector.InjectFailure(SensorType.Barometer);

        for (int i = 5; i <= 35; i++)
        {
            float t = i * 0.05f;
            imuSensor.UpdateFromKinematics(Vector3.zero, Vector3.zero, Quaternion.identity, t);
            if (i % 2 == 0) gpsSensor.UpdateFromSimulationState(Vector3.zero, Vector3.zero, t);
            // Baro is failed so it produces no measurements
            baroSensor.UpdateFromSimulationState(0f, 0f, t);
        }

        float degradedVertStdDev = ekfProvider.VerticalPositionStdDev;
        Assert.Greater(degradedVertStdDev, nominalVertStdDev, "Vertical uncertainty must increase when high-rate baro is lost!");
        Assert.AreNotEqual(EstimatorStatus.Failed, ekfProvider.Status, "Estimator must not fail if GPS and IMU remain operational!");
    }

    [Test]
    public void SensorFailure_BaroRecovery_RestoresVerticalClearance()
    {
        // Outage
        failureInjector.InjectFailure(SensorType.Barometer);
        for (int i = 1; i <= 20; i++)
        {
            imuSensor.UpdateFromKinematics(Vector3.zero, Vector3.zero, Quaternion.identity, i * 0.05f);
        }
        float outageMargin = threatAssessment.EffectiveVerticalSafetyMargin;

        // Recovery
        failureInjector.RecoverSensor(SensorType.Barometer);
        for (int i = 21; i <= 30; i++)
        {
            float t = i * 0.05f;
            imuSensor.UpdateFromKinematics(Vector3.zero, Vector3.zero, Quaternion.identity, t);
            baroSensor.UpdateFromSimulationState(0f, 0f, t);
        }

        float recoveredMargin = threatAssessment.EffectiveVerticalSafetyMargin;
        Assert.LessOrEqual(recoveredMargin, outageMargin, "Effective vertical safety margin must contract upon Baro recovery!");
    }

    [Test]
    public void SensorFailure_ImuLoss_TriggersEstimatorFailedStatus()
    {
        gpsSensor.UpdateFromSimulationState(Vector3.zero, Vector3.zero, 0.0f);
        imuSensor.UpdateFromKinematics(Vector3.zero, Vector3.zero, Quaternion.identity, 0.0f);
        Assert.AreEqual(EstimatorStatus.Nominal, ekfProvider.Status);

        // Inject IMU failure
        failureInjector.InjectFailure(SensorType.IMU);

        // Advance time past 0.10s IMU timeout threshold
        float currentTime = 0.20f;
        ekfProvider.CheckTimeouts(currentTime);

        Assert.AreEqual(EstimatorStatus.Failed, ekfProvider.Status, "EstimatorStatus must transition to Failed upon IMU loss!");
    }

    [Test]
    public void SensorFailure_ImuLoss_TriggersAutonomySafeHold()
    {
        gpsSensor.UpdateFromSimulationState(Vector3.zero, Vector3.zero, 0.0f);
        imuSensor.UpdateFromKinematics(Vector3.zero, Vector3.zero, Quaternion.identity, 0.0f);

        pathFollower.StartFollowing(new List<Node> { new Node(true, new Vector3(0f, 1f, 10f), 0, 0) });

        // IMU failure past timeout
        failureInjector.InjectFailure(SensorType.IMU);
        ekfProvider.CheckTimeouts(0.25f);

        ThreatReport threat = new ThreatReport(ThreatLevel.Critical, default(DetectedObstacle), Vector3.forward * 3f, 3f, 1.0f, 0);
        bool replanResult = replanningController.TryExecuteReplan("Critical Threat with IMU Failure", threat);

        Assert.IsFalse(replanResult);
        Assert.AreEqual(NavigationState.NoSafePath, replanningController.State, "UAV must enter NoSafePath Safe Hold on estimator failure!");
    }

    [Test]
    public void SensorFailure_IntermittentGps_MaintainsContinuousEstimate()
    {
        Vector3 truePos = new Vector3(2f, 1f, 3f);
        Vector3 trueVel = new Vector3(0f, 0f, 1f);

        // Cycle GPS failure on and off
        for (int cycle = 0; cycle < 4; cycle++)
        {
            float tBase = cycle * 0.4f;

            // 0.2s ON
            failureInjector.RecoverSensor(SensorType.GPS);
            for (int i = 1; i <= 2; i++)
            {
                float t = tBase + i * 0.1f;
                imuSensor.UpdateFromKinematics(Vector3.zero, Vector3.zero, Quaternion.identity, t);
                gpsSensor.UpdateFromSimulationState(truePos, trueVel, t);
            }

            // 0.2s OFF
            failureInjector.InjectFailure(SensorType.GPS);
            for (int i = 3; i <= 4; i++)
            {
                float t = tBase + i * 0.1f;
                imuSensor.UpdateFromKinematics(Vector3.zero, Vector3.zero, Quaternion.identity, t);
            }
        }

        EstimatedState finalState = ekfProvider.CurrentState;
        Assert.IsTrue(float.IsFinite(finalState.Position.x));
        Assert.IsTrue(float.IsFinite(finalState.Position.y));
        Assert.IsTrue(float.IsFinite(finalState.Position.z));
        Assert.AreEqual(2.0f, finalState.Position.x, 1.0f);
        Assert.AreEqual(3.0f, finalState.Position.z, 1.0f);
    }

    [Test]
    public void SensorFailure_TimeoutDebounce_IgnoresSinglePacketDrop()
    {
        gpsSensor.UpdateFromSimulationState(Vector3.zero, Vector3.zero, 0.0f);

        // Simulate single missed GPS frame (0.15s elapsed since last packet, less than 0.50s timeout threshold)
        float singleMissedFrameTime = 0.15f;
        imuSensor.UpdateFromKinematics(Vector3.zero, Vector3.zero, Quaternion.identity, singleMissedFrameTime);

        // GPS fix should NOT be dropped prematurely
        Assert.AreEqual(GpsFixState.Fix3D, ekfProvider.GpsState, "Debounce watchdog must not declare NoFix on a single dropped packet!");
        Assert.AreEqual(EstimatorStatus.Nominal, ekfProvider.Status);
    }

    [Test]
    public void SensorFailure_UncertaintyAvoidance_ExpandsDynamicRadiiDuringFailure()
    {
        // 1. Converge initial nominal state
        for (int i = 0; i < 5; i++)
        {
            float t = i * 0.1f;
            imuSensor.UpdateFromKinematics(Vector3.zero, Vector3.zero, Quaternion.identity, t);
            gpsSensor.UpdateFromSimulationState(Vector3.zero, Vector3.zero, t);
        }
        float nominalSafetyRadius = threatAssessment.EffectiveSafetyRadius;

        failureInjector.InjectFailure(SensorType.GPS);

        // Propagate dead-reckoning
        for (int i = 5; i <= 45; i++)
        {
            imuSensor.UpdateFromKinematics(Vector3.zero, Vector3.zero, Quaternion.identity, i * 0.05f);
        }

        float degradedSafetyRadius = threatAssessment.EffectiveSafetyRadius;
        Assert.Greater(degradedSafetyRadius, nominalSafetyRadius, "Phase 11.3 EffectiveSafetyRadius must expand during GPS failure!");
        Assert.LessOrEqual(degradedSafetyRadius, threatAssessment.MaxSafetyRadius, "EffectiveSafetyRadius must respect max clamp!");
    }

    [Test]
    public void SensorFailure_Injector_ExecutesTimedFailureScheduleDeterministically()
    {
        // Add schedule: GPS failed from t=1.0s to t=3.0s
        failureInjector.AddSchedule(new SensorFailureSchedule(SensorType.GPS, SensorHealth.Failed, 1.0f, 2.0f, GpsFixState.NoFix));

        // t = 0.5s (Before failure)
        failureInjector.EvaluateSchedules(0.5f);
        Assert.AreEqual(SensorHealth.Healthy, gpsSensor.Health);

        // t = 1.5s (Inside failure window)
        failureInjector.EvaluateSchedules(1.5f);
        Assert.AreEqual(SensorHealth.Failed, gpsSensor.Health);
        Assert.AreEqual(GpsFixState.NoFix, gpsSensor.FixQuality);

        // t = 3.5s (After failure window - recovered)
        failureInjector.EvaluateSchedules(3.5f);
        Assert.AreEqual(SensorHealth.Healthy, gpsSensor.Health);
        Assert.AreEqual(GpsFixState.Fix3D, gpsSensor.FixQuality);
    }
}
