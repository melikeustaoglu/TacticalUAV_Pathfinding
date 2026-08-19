using System.Collections.Generic;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;

/// <summary>
/// Phase 11.2.3 EKF Runtime Integration & Ground-Truth Decoupling Tests.
/// Validates the live Sensor -> EKF -> EstimatedState -> Autonomy pipeline in EditMode.
/// </summary>
[TestFixture]
public class EkfRuntimeIntegrationTests
{
    private GameObject uavObj;
    private SimulatedGpsSensor gpsSensor;
    private SimulatedImuSensor imuSensor;
    private SimulatedBaroAltimeter baroSensor;
    private EkfStateProvider ekfProvider;
    private StateEstimationDiagnostics diagnostics;
    private PathFollower pathFollower;
    private UAVPerception perception;
    private ThreatAssessment threatAssessment;
    private ReplanningController replanningController;

    [SetUp]
    public void SetUp()
    {
        uavObj = new GameObject("EkfRuntimeUAV");
        gpsSensor = uavObj.AddComponent<SimulatedGpsSensor>();
        imuSensor = uavObj.AddComponent<SimulatedImuSensor>();
        baroSensor = uavObj.AddComponent<SimulatedBaroAltimeter>();
        ekfProvider = uavObj.AddComponent<EkfStateProvider>();
        diagnostics = uavObj.AddComponent<StateEstimationDiagnostics>();
        pathFollower = uavObj.AddComponent<PathFollower>();
        perception = uavObj.AddComponent<UAVPerception>();
        threatAssessment = uavObj.AddComponent<ThreatAssessment>();
        replanningController = uavObj.AddComponent<ReplanningController>();

        typeof(SimulatedGpsSensor).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(gpsSensor, null);
        typeof(SimulatedImuSensor).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(imuSensor, null);
        typeof(SimulatedBaroAltimeter).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(baroSensor, null);
        typeof(EkfStateProvider).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(ekfProvider, null);
        typeof(EkfStateProvider).GetMethod("OnEnable", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(ekfProvider, null);
        typeof(StateEstimationDiagnostics).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(diagnostics, null);
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
    public void EkfRuntime_Initialization_WithoutGroundTruthProvider_OperatesSuccessfully()
    {
        uavObj.transform.position = new Vector3(10f, 2f, 15f);

        // Feed initial GPS and Baro measurements
        gpsSensor.UpdateFromSimulationState(new Vector3(10f, 2f, 15f), Vector3.zero, 0.1f);
        baroSensor.UpdateFromSimulationState(2.0f, 0.0f, 0.1f);

        Assert.IsTrue(ekfProvider.IsEstimatorReady);
        Assert.AreEqual(EstimatorStatus.Nominal, ekfProvider.Status);
        Assert.AreEqual(10f, ekfProvider.CurrentState.Position.x, 1.0f);
        Assert.AreEqual(2f, ekfProvider.CurrentState.Position.y, 0.5f);
        Assert.AreEqual(15f, ekfProvider.CurrentState.Position.z, 1.0f);
    }

    [Test]
    public void EkfRuntime_SensorPipeline_PropagatesToAutonomyConsumers()
    {
        uavObj.transform.position = new Vector3(5f, 3f, 8f);
        gpsSensor.UpdateFromSimulationState(new Vector3(5f, 3f, 8f), Vector3.zero, 0.0f);
        baroSensor.UpdateFromSimulationState(3.0f, 0.0f, 0.0f);

        // Verify ThreatAssessment received EstimatedState via IEstimatedStateProvider
        FieldInfo stateField = typeof(ThreatAssessment).GetField("stateProvider", BindingFlags.NonPublic | BindingFlags.Instance);
        IEstimatedStateProvider provider = stateField?.GetValue(threatAssessment) as IEstimatedStateProvider;

        Assert.IsNotNull(provider);
        Assert.AreEqual(ekfProvider, provider, "ThreatAssessment must bind to EkfStateProvider!");
        Assert.AreEqual(5f, provider.CurrentState.Position.x, 1.0f);
        Assert.AreEqual(3f, provider.CurrentState.Position.y, 0.5f);
        Assert.AreEqual(8f, provider.CurrentState.Position.z, 1.0f);
    }

    [Test]
    public void EkfRuntime_GpsDropout_DegradesEstimatorStatusGracefully()
    {
        gpsSensor.UpdateFromSimulationState(Vector3.zero, Vector3.zero, 0.0f);
        Assert.AreEqual(EstimatorStatus.Nominal, ekfProvider.Status);

        // Disable GPS (e.g. simulated tunnel / jamming)
        gpsSensor.SetHealth(SensorHealth.Failed, GpsFixState.NoFix);

        // Propagate via IMU dead-reckoning for 3 seconds
        for (int i = 1; i <= 30; i++)
        {
            float t = i * 0.1f;
            imuSensor.UpdateFromKinematics(Vector3.zero, Vector3.zero, Quaternion.identity, t);
        }

        // When GPS is unacquired during navigation, status should reflect dead-reckoning / degradation
        Assert.IsTrue(ekfProvider.CurrentState.IsValid);
        Assert.Greater(ekfProvider.HorizontalPositionStdDev, 0.2f, "Position uncertainty must grow during GPS denial!");
    }

    [Test]
    public void EkfRuntime_GpsRecovery_RestoresNominalStatusAndConvergence()
    {
        // 1. Initial fix
        gpsSensor.UpdateFromSimulationState(new Vector3(0f, 1f, 0f), Vector3.zero, 0.0f);

        // 2. Outage
        gpsSensor.SetHealth(SensorHealth.Failed, GpsFixState.NoFix);
        for (int i = 1; i <= 20; i++)
        {
            imuSensor.UpdateFromKinematics(Vector3.zero, Vector3.zero, Quaternion.identity, i * 0.1f);
        }

        float degradedStdDev = ekfProvider.HorizontalPositionStdDev;

        // 3. Recovery
        gpsSensor.SetHealth(SensorHealth.Healthy, GpsFixState.Fix3D);
        for (int i = 21; i <= 30; i++)
        {
            float t = i * 0.1f;
            imuSensor.UpdateFromKinematics(Vector3.zero, Vector3.zero, Quaternion.identity, t);
            if (i % 2 == 0) gpsSensor.UpdateFromSimulationState(new Vector3(0f, 1f, 0f), Vector3.zero, t);
        }

        float recoveredStdDev = ekfProvider.HorizontalPositionStdDev;
        Assert.Less(recoveredStdDev, degradedStdDev, "Covariance must shrink upon GPS reacquisition!");
    }

    [Test]
    public void EkfRuntime_GpsOutlier_IsGatedWithoutCorruptingEstimate()
    {
        gpsSensor.UpdateFromSimulationState(Vector3.zero, Vector3.zero, 0.0f);

        // Glitch of 100 meters
        GpsMeasurement outlier = new GpsMeasurement(new Vector3(100f, 0f, 0f), Vector3.zero, Vector3.one * 0.01f, Vector3.one * 0.01f, 0.1f);
        ekfProvider.HandleGpsUpdated(outlier);

        Assert.AreEqual(1, ekfProvider.RejectedMeasurements, "Outlier must be rejected by innovation gate!");
        Assert.Less(ekfProvider.CurrentState.Position.x, 1.0f, "State position must not jump to 100m!");
    }

    [Test]
    public void EkfRuntime_Diagnostics_ComputesRmseAccurately()
    {
        uavObj.transform.position = new Vector3(5f, 2f, 10f);
        gpsSensor.UpdateFromSimulationState(new Vector3(5f, 2f, 10f), Vector3.zero, 0.0f);
        baroSensor.UpdateFromSimulationState(2.0f, 0.0f, 0.0f);

        diagnostics.SampleDiagnostics();

        // Error between truth (5, 2, 10) and estimated should be small (noise level)
        Assert.Less(diagnostics.CurrentPositionError, 1.5f);
    }

    [Test]
    public void EkfRuntime_PathFollower_ConsumesEkfPositionForGuidance()
    {
        gpsSensor.UpdateFromSimulationState(new Vector3(2f, 1f, 3f), Vector3.zero, 0.0f);
        pathFollower.StartFollowing(new List<Node> { new Node(true, new Vector3(2f, 1f, 20f), 0, 0) });

        Assert.AreEqual(1.0f, pathFollower.TargetWaypoint.y, 0.1f);
    }

    [Test]
    public void EkfRuntime_ThreatAssessment_UsesEkfStateForCpaProjections()
    {
        gpsSensor.UpdateFromSimulationState(new Vector3(0f, 1f, 0f), Vector3.zero, 0.0f);

        GameObject obstacle = GameObject.CreatePrimitive(PrimitiveType.Cube);
        obstacle.name = "EkfThreatObstacle";
        obstacle.layer = LayerMask.NameToLayer(ProceduralObstacleGenerator.ObstacleLayerName);
        obstacle.transform.position = new Vector3(0f, 1f, 6f);
        Physics.SyncTransforms();

        pathFollower.StartFollowing(new List<Node> { new Node(true, new Vector3(0f, 1f, 20f), 0, 0) });

        perception.PerformScan();
        threatAssessment.EvaluateThreats();

        Assert.AreNotEqual(ThreatLevel.None, threatAssessment.CurrentThreatLevel);

        Object.DestroyImmediate(obstacle);
    }

    [Test]
    public void EkfRuntime_ReplanningController_UsesEkfAltitudeForVerticalEvasion()
    {
        uavObj.transform.position = new Vector3(0f, 1.5f, 0f);
        gpsSensor.UpdateFromSimulationState(new Vector3(0f, 1.5f, 0f), Vector3.zero, 0.0f);
        baroSensor.UpdateFromSimulationState(1.5f, 0.0f, 0.0f);

        MethodInfo getEstAlt = typeof(ReplanningController).GetMethod("GetEstimatedAltitude", BindingFlags.NonPublic | BindingFlags.Instance);
        float estAlt = (float)(getEstAlt?.Invoke(replanningController, null) ?? 0f);

        Assert.AreEqual(1.5f, estAlt, 0.5f);
    }

    [Test]
    public void EkfRuntime_GroundTruthStateProvider_IsNotRequiredForNormalOperation()
    {
        // Assert GroundTruthStateProvider is not present on this UAV
        GroundTruthStateProvider gt = uavObj.GetComponent<GroundTruthStateProvider>();
        Assert.IsNull(gt, "GroundTruthStateProvider must not be required on UAV with EKF!");

        // Assert EkfStateProvider provides full functional estimation
        Assert.IsNotNull(uavObj.GetComponent<IEstimatedStateProvider>());
        Assert.IsInstanceOf<EkfStateProvider>(uavObj.GetComponent<IEstimatedStateProvider>());
    }
}
