using System.Collections.Generic;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;

/// <summary>
/// Phase 11.1 State Estimation Architecture Boundary Tests.
/// Validates EstimatedState data contracts, IEstimatedStateProvider service abstractions,
/// and transitional GroundTruthStateProvider mapping.
/// </summary>
[TestFixture]
public class EstimatedStateArchitectureTests
{
    private GameObject uavObj;
    private GroundTruthStateProvider stateProvider;
    private PathFollower pathFollower;
    private UAVPerception perception;
    private ThreatAssessment threatAssessment;
    private ReplanningController replanningController;

    [SetUp]
    public void SetUp()
    {
        uavObj = new GameObject("ArchTestUAV");
        stateProvider = uavObj.AddComponent<GroundTruthStateProvider>();
        pathFollower = uavObj.AddComponent<PathFollower>();
        perception = uavObj.AddComponent<UAVPerception>();
        threatAssessment = uavObj.AddComponent<ThreatAssessment>();
        replanningController = uavObj.AddComponent<ReplanningController>();

        typeof(PathFollower).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(pathFollower, null);
        typeof(UAVPerception).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(perception, null);
        typeof(ThreatAssessment).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(threatAssessment, null);
        typeof(ReplanningController).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(replanningController, null);
        typeof(GroundTruthStateProvider).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(stateProvider, null);
    }

    [TearDown]
    public void TearDown()
    {
        if (uavObj != null) Object.DestroyImmediate(uavObj);
    }

    [Test]
    public void EstimatedState_DeterministicInitialization_PopulatesFieldsCorrectly()
    {
        Vector3 pos = new Vector3(10f, 2.5f, -15f);
        Vector3 vel = new Vector3(1.2f, 0f, 0.9f);
        Vector3 posVar = new Vector3(0.04f, 0.01f, 0.09f);
        Vector3 velVar = new Vector3(0.01f, 0.01f, 0.01f);

        EstimatedState state = new EstimatedState(
            pos,
            vel,
            45.0f,
            -10.0f,
            new Vector3(0.02f, -0.01f, 0.0f),
            0.005f,
            posVar,
            velVar,
            0.001f,
            12.5f,
            EstimatorStatus.Nominal,
            GpsFixState.Fix3D);

        Assert.AreEqual(pos, state.Position);
        Assert.AreEqual(vel, state.Velocity);
        Assert.AreEqual(45.0f, state.YawDegrees, 0.001f);
        Assert.AreEqual(45.0f * Mathf.Deg2Rad, state.YawRadians, 0.001f);
        Assert.AreEqual(-10.0f, state.PitchDegrees, 0.001f);
        Assert.AreEqual(0.09f, state.HorizontalPositionVariance, 0.001f);
        Assert.AreEqual(0.3f, state.HorizontalPositionStandardDeviation, 0.001f);
        Assert.AreEqual(0.1f, state.VerticalPositionStandardDeviation, 0.001f);
        Assert.AreEqual(12.5f, state.Timestamp, 0.001f);
        Assert.AreEqual(EstimatorStatus.Nominal, state.Status);
        Assert.AreEqual(GpsFixState.Fix3D, state.GpsState);
        Assert.IsTrue(state.IsValid);
    }

    [Test]
    public void EstimatedState_StatusAndGpsEnums_ReflectValidStates()
    {
        EstimatedState uninit = EstimatedState.Uninitialized;
        Assert.IsFalse(uninit.IsValid);
        Assert.AreEqual(EstimatorStatus.Uninitialized, uninit.Status);
        Assert.AreEqual(GpsFixState.NoFix, uninit.GpsState);

        EstimatedState degraded = new EstimatedState(
            Vector3.zero, Vector3.zero, 0f, 0f, Vector3.zero, 0f, Vector3.one, Vector3.one, 0f, 0f,
            EstimatorStatus.Degraded, GpsFixState.NoFix);
        Assert.IsTrue(degraded.IsValid);

        EstimatedState failed = new EstimatedState(
            Vector3.zero, Vector3.zero, 0f, 0f, Vector3.zero, 0f, Vector3.one, Vector3.one, 0f, 0f,
            EstimatorStatus.Failed, GpsFixState.NoFix);
        Assert.IsFalse(failed.IsValid);
    }

    [Test]
    public void GroundTruthStateProvider_SamplesTransform_AndPublishesNominalState()
    {
        uavObj.transform.position = new Vector3(5f, 3f, -8f);
        uavObj.transform.rotation = Quaternion.Euler(0f, 90f, 0f);

        stateProvider.SampleState();

        EstimatedState state = stateProvider.CurrentState;

        Assert.IsTrue(stateProvider.IsEstimatorReady);
        Assert.AreEqual(5f, state.Position.x, 0.01f);
        Assert.AreEqual(3f, state.Position.y, 0.01f);
        Assert.AreEqual(-8f, state.Position.z, 0.01f);
        Assert.AreEqual(90f, state.YawDegrees, 0.1f);
        Assert.AreEqual(EstimatorStatus.Nominal, state.Status);
        Assert.AreEqual(GpsFixState.Fix3D, state.GpsState);
    }

    [Test]
    public void GroundTruthStateProvider_DispatchesReactiveEvent()
    {
        uavObj.transform.position = new Vector3(12f, 1f, 14f);

        int eventFiredCount = 0;
        EstimatedState capturedState = default;

        stateProvider.OnStateEstimated += s =>
        {
            eventFiredCount++;
            capturedState = s;
        };

        stateProvider.SampleState();

        Assert.AreEqual(1, eventFiredCount);
        Assert.AreEqual(12f, capturedState.Position.x, 0.01f);
        Assert.AreEqual(14f, capturedState.Position.z, 0.01f);
    }

    [Test]
    public void GroundTruthStateProvider_SyntheticVarianceInjection_IncreasesCovariance()
    {
        FieldInfo injectField = typeof(GroundTruthStateProvider).GetField("injectBaselineVariance", BindingFlags.NonPublic | BindingFlags.Instance);
        FieldInfo varField = typeof(GroundTruthStateProvider).GetField("syntheticPositionVariance", BindingFlags.NonPublic | BindingFlags.Instance);

        injectField?.SetValue(stateProvider, true);
        varField?.SetValue(stateProvider, 0.16f); // 0.4m 1-sigma

        stateProvider.SampleState();

        EstimatedState state = stateProvider.CurrentState;
        Assert.AreEqual(0.16f, state.HorizontalPositionVariance, 0.001f);
        Assert.AreEqual(0.4f, state.HorizontalPositionStandardDeviation, 0.001f);
    }

    [Test]
    public void IEstimatedStateProvider_DecouplesThreatAssessment_FromDirectTransform()
    {
        // Position UAV at (0, 1, 0)
        uavObj.transform.position = new Vector3(0f, 1f, 0f);
        stateProvider.SampleState();

        // Create obstacle at (0, 1, 6)
        GameObject obstacle = GameObject.CreatePrimitive(PrimitiveType.Cube);
        obstacle.name = "TestObstacle";
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
    public void IEstimatedStateProvider_DecouplesReplanningController_FromDirectTransform()
    {
        uavObj.transform.position = new Vector3(2f, 3.5f, 4f);
        stateProvider.SampleState();

        MethodInfo getEstPos = typeof(ReplanningController).GetMethod("GetEstimatedPosition", BindingFlags.NonPublic | BindingFlags.Instance);
        MethodInfo getEstAlt = typeof(ReplanningController).GetMethod("GetEstimatedAltitude", BindingFlags.NonPublic | BindingFlags.Instance);

        Vector3 pos = (Vector3)(getEstPos?.Invoke(replanningController, null) ?? Vector3.zero);
        float alt = (float)(getEstAlt?.Invoke(replanningController, null) ?? 0f);

        Assert.AreEqual(2f, pos.x, 0.01f);
        Assert.AreEqual(3.5f, pos.y, 0.01f);
        Assert.AreEqual(4f, pos.z, 0.01f);
        Assert.AreEqual(3.5f, alt, 0.01f);
    }

    [Test]
    public void AutonomyStack_FallbackGracefully_WhenStateProviderNotAttached()
    {
        // Create an un-instrumented UAV without GroundTruthStateProvider
        GameObject rawUav = new GameObject("RawUAV");
        rawUav.transform.position = new Vector3(7f, 2f, 9f);

        PathFollower pf = rawUav.AddComponent<PathFollower>();
        ThreatAssessment ta = rawUav.AddComponent<ThreatAssessment>();
        ReplanningController rc = rawUav.AddComponent<ReplanningController>();

        typeof(PathFollower).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(pf, null);
        typeof(ThreatAssessment).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(ta, null);
        typeof(ReplanningController).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(rc, null);

        MethodInfo getEstPos = typeof(ReplanningController).GetMethod("GetEstimatedPosition", BindingFlags.NonPublic | BindingFlags.Instance);
        Vector3 fallbackPos = (Vector3)(getEstPos?.Invoke(rc, null) ?? Vector3.zero);

        Assert.AreEqual(7f, fallbackPos.x, 0.01f);
        Assert.AreEqual(2f, fallbackPos.y, 0.01f);
        Assert.AreEqual(9f, fallbackPos.z, 0.01f);

        Object.DestroyImmediate(rawUav);
    }

    [Test]
    public void GameManagerBootstrapper_CreateUav_EquipsCompleteSensorAndTrackingStack()
    {
        Vector3 spawnPos = new Vector3(15f, 2f, 25f);
        GameObject runtimeUav = GameManagerBootstrapper.CreateUav(spawnPos);

        // Core Sensors
        SimulatedGpsSensor gps = runtimeUav.GetComponent<SimulatedGpsSensor>();
        SimulatedImuSensor imu = runtimeUav.GetComponent<SimulatedImuSensor>();
        SimulatedBaroAltimeter baro = runtimeUav.GetComponent<SimulatedBaroAltimeter>();
        SimulatedLidarSensor lidar = runtimeUav.GetComponent<SimulatedLidarSensor>();
        SimulatedRadarSensor radar = runtimeUav.GetComponent<SimulatedRadarSensor>();

        // State Estimation & Diagnostics
        EkfStateProvider ekf = runtimeUav.GetComponent<EkfStateProvider>();
        StateEstimationDiagnostics diag = runtimeUav.GetComponent<StateEstimationDiagnostics>();

        // Target Tracking
        TrackManager trackManager = runtimeUav.GetComponent<TrackManager>();

        // Tactical Autonomy & Mission
        PathFollower pf = runtimeUav.GetComponent<PathFollower>();
        UAVPerception perception = runtimeUav.GetComponent<UAVPerception>();
        ThreatAssessment threat = runtimeUav.GetComponent<ThreatAssessment>();
        ReplanningController replanning = runtimeUav.GetComponent<ReplanningController>();
        MissionManager mission = runtimeUav.GetComponent<MissionManager>();
        TacticalHUD hud = runtimeUav.GetComponent<TacticalHUD>();
        MissionEventLogger logger = runtimeUav.GetComponent<MissionEventLogger>();
        BenchmarkReporter reporter = runtimeUav.GetComponent<BenchmarkReporter>();

        Assert.IsNotNull(gps, "Runtime UAV must be equipped with SimulatedGpsSensor!");
        Assert.IsNotNull(imu, "Runtime UAV must be equipped with SimulatedImuSensor!");
        Assert.IsNotNull(baro, "Runtime UAV must be equipped with SimulatedBaroAltimeter!");
        Assert.IsNotNull(lidar, "Runtime UAV must be equipped with SimulatedLidarSensor!");
        Assert.IsNotNull(radar, "Runtime UAV must be equipped with SimulatedRadarSensor!");
        Assert.IsNotNull(ekf, "Runtime UAV must be equipped with EkfStateProvider!");
        Assert.IsNotNull(diag, "Runtime UAV must be equipped with StateEstimationDiagnostics!");
        Assert.IsNotNull(trackManager, "Runtime UAV must be equipped with TrackManager!");
        Assert.IsNotNull(pf, "Runtime UAV must be equipped with PathFollower!");
        Assert.IsNotNull(perception, "Runtime UAV must be equipped with UAVPerception!");
        Assert.IsNotNull(threat, "Runtime UAV must be equipped with ThreatAssessment!");
        Assert.IsNotNull(replanning, "Runtime UAV must be equipped with ReplanningController!");
        Assert.IsNotNull(mission, "Runtime UAV must be equipped with MissionManager!");
        Assert.IsNotNull(hud, "Runtime UAV must be equipped with TacticalHUD!");
        Assert.IsNotNull(logger, "Runtime UAV must be equipped with MissionEventLogger!");
        Assert.IsNotNull(reporter, "Runtime UAV must be equipped with BenchmarkReporter!");

        // Verify sensor masks
        LayerMask expectedMask = ProceduralObstacleGenerator.GetObstacleMask();
        Assert.AreEqual(expectedMask.value, lidar.TargetMask.value, "LiDAR target mask must match obstacle mask!");
        Assert.AreEqual(expectedMask.value, radar.TargetMask.value, "Radar target mask must match obstacle mask!");

        // Verify TrackManager discovered and registered both tracking sensors
        Assert.AreEqual(2, trackManager.SensorCount, "TrackManager must discover both LiDAR and Radar sensors on the UAV!");

        Object.DestroyImmediate(runtimeUav);
    }

    [Test]
    public void GameManagerBootstrapper_CreateUav_WiredTrackManager_FeedsThreatAssessment()
    {
        Vector3 spawnPos = new Vector3(0f, 1f, 0f);
        GameObject runtimeUav = GameManagerBootstrapper.CreateUav(spawnPos);

        TrackManager trackManager = runtimeUav.GetComponent<TrackManager>();
        ThreatAssessment threat = runtimeUav.GetComponent<ThreatAssessment>();
        PathFollower pf = runtimeUav.GetComponent<PathFollower>();

        // Set path for ThreatAssessment trajectory lookup
        pf.StartFollowing(new List<Node>
        {
            new Node(true, new Vector3(0f, 1f, 5f), 0, 0),
            new Node(true, new Vector3(0f, 1f, 15f), 0, 1)
        });

        // 3 consecutive radar hits confirming an approaching target in front of UAV at 4m
        Vector3 targetPos = new Vector3(0f, 1f, 4f);
        Vector3 targetVel = new Vector3(0f, 0f, -2.0f);
        trackManager.ProcessDetections(new TargetDetection[] { new TargetDetection(TargetSensorModality.Radar, 0.00f, targetPos, Vector3.one * 0.04f, 0.95f, 1, targetVel, Vector3.one * 0.04f, true) }, 1, 0.00f);
        trackManager.ProcessDetections(new TargetDetection[] { new TargetDetection(TargetSensorModality.Radar, 0.05f, targetPos, Vector3.one * 0.04f, 0.95f, 1, targetVel, Vector3.one * 0.04f, true) }, 1, 0.05f);
        trackManager.ProcessDetections(new TargetDetection[] { new TargetDetection(TargetSensorModality.Radar, 0.10f, targetPos, Vector3.one * 0.04f, 0.95f, 1, targetVel, Vector3.one * 0.04f, true) }, 1, 0.10f);

        Assert.AreEqual(1, trackManager.ActiveTrackCount);

        // Evaluate threat assessment consuming TrackManager
        threat.EvaluateThreats();

        Assert.AreNotEqual(ThreatLevel.None, threat.CurrentThreatLevel, "ThreatAssessment must detect the confirmed tracked target!");
        Assert.IsTrue(threat.CurrentThreatReport.HasTrack, "ThreatReport must contain valid track reference!");

        Object.DestroyImmediate(runtimeUav);
    }
}
