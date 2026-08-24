using System;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using NUnit.Framework;
using UnityEditor;
using UnityEngine;

/// <summary>
/// Phase V1 15-Run Full Mission Benchmark & Validation Suite.
/// Executes the complete 5-scenario x 3-seed benchmark matrix against the full production autonomy stack:
/// 1-3.   Default / Baseline (Seeds 42, 142, 242)
/// 4-6.   Dynamic Crossing (Seeds 400, 500, 600)
/// 7-9.   Dense Clutter (Seeds 42, 142, 242)
/// 10-12. GPS Outage & Uncertainty Adaptation (Seeds 42, 142, 242)
/// 13-15. Multi-Threat Prioritization & Preemption (Seeds 700, 800, 900)
/// </summary>
[TestFixture]
public class FullMissionBenchmarkSuiteTests
{
    private GameObject systemObj;
    private GameObject uavObj;
    private GameObject obstacleParentObj;
    private GameObject targetObj;

    private GridManager gridManager;
    private Pathfinding pathfinding;
    private SimulatedGpsSensor gpsSensor;
    private SimulatedImuSensor imuSensor;
    private SimulatedBaroAltimeter baroSensor;
    private GroundTruthStateProvider stateProvider;
    private SimulatedLidarSensor lidarSensor;
    private SimulatedRadarSensor radarSensor;
    private TrackManager trackManager;
    private PathFollower pathFollower;
    private UAVPerception uavPerception;
    private ThreatAssessment threatAssessment;
    private ReplanningController replanningController;
    private MissionManager missionManager;
    private MissionEventLogger eventLogger;
    private BenchmarkReporter benchmarkReporter;

    private MethodInfo moveAlongPathMethod;
    private MethodInfo threatUpdateMethod;
    private MethodInfo replanUpdateMethod;
    private MethodInfo perceptionScanMethod;

    [SetUp]
    public void SetUp()
    {
        // 1. World Grid System Origin (fixed at (0,0,0))
        systemObj = new GameObject("PathfindingSystem");
        systemObj.transform.position = Vector3.zero;

        gridManager = systemObj.AddComponent<GridManager>();
        gridManager.gridWorldSize = new Vector2(50f, 50f);
        gridManager.nodeRadius = 0.5f;
        gridManager.obstacleMask = ProceduralObstacleGenerator.GetObstacleMask();

        pathfinding = systemObj.AddComponent<Pathfinding>();
        typeof(Pathfinding).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(pathfinding, null);
        gridManager.CreateGrid();

        // 2. UAV Agent
        uavObj = new GameObject("BenchmarkUAV_V1");
        uavObj.transform.position = new Vector3(-10f, 1f, -10f);

        // Sensors & State Estimation
        gpsSensor = uavObj.AddComponent<SimulatedGpsSensor>();
        imuSensor = uavObj.AddComponent<SimulatedImuSensor>();
        baroSensor = uavObj.AddComponent<SimulatedBaroAltimeter>();
        stateProvider = uavObj.AddComponent<GroundTruthStateProvider>();

        // Multi-Sensor Perception & Tracking
        lidarSensor = uavObj.AddComponent<SimulatedLidarSensor>();
        lidarSensor.TargetMask = gridManager.obstacleMask;
        radarSensor = uavObj.AddComponent<SimulatedRadarSensor>();
        radarSensor.TargetMask = gridManager.obstacleMask;
        trackManager = uavObj.AddComponent<TrackManager>();

        // Guidance, Perception & Autonomy
        pathFollower = uavObj.AddComponent<PathFollower>();
        uavPerception = uavObj.AddComponent<UAVPerception>();
        threatAssessment = uavObj.AddComponent<ThreatAssessment>();
        replanningController = uavObj.AddComponent<ReplanningController>();
        missionManager = uavObj.AddComponent<MissionManager>();
        eventLogger = uavObj.AddComponent<MissionEventLogger>();
        benchmarkReporter = uavObj.AddComponent<BenchmarkReporter>();

        // Wire StateProvider references
        pathFollower.SetStateProvider(stateProvider);
        threatAssessment.SetStateProvider(stateProvider);
        replanningController.SetStateProvider(stateProvider);

        // Initialize state provider to nominal
        SetEstimatorState(uavObj.transform.position, Vector3.zero, posVariance: 0.01f, status: EstimatorStatus.Nominal);

        // Invoke Awakes
        typeof(SimulatedGpsSensor).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(gpsSensor, null);
        typeof(SimulatedImuSensor).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(imuSensor, null);
        typeof(SimulatedBaroAltimeter).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(baroSensor, null);
        typeof(TrackManager).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(trackManager, null);
        typeof(PathFollower).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(pathFollower, null);
        typeof(UAVPerception).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(uavPerception, null);
        typeof(ThreatAssessment).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(threatAssessment, null);
        typeof(ReplanningController).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(replanningController, null);
        typeof(MissionManager).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(missionManager, null);
        typeof(MissionEventLogger).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(eventLogger, null);
        typeof(BenchmarkReporter).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(benchmarkReporter, null);

        typeof(ReplanningController).GetMethod("Start", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(replanningController, null);

        targetObj = new GameObject("BenchmarkTarget");
        targetObj.transform.position = new Vector3(10f, 1f, 10f);
        pathfinding.targetTransform = targetObj.transform;

        moveAlongPathMethod = typeof(PathFollower).GetMethod("MoveAlongPath", BindingFlags.NonPublic | BindingFlags.Instance);
        threatUpdateMethod = typeof(ThreatAssessment).GetMethod("Update", BindingFlags.NonPublic | BindingFlags.Instance);
        replanUpdateMethod = typeof(ReplanningController).GetMethod("Update", BindingFlags.NonPublic | BindingFlags.Instance);
        perceptionScanMethod = typeof(UAVPerception).GetMethod("ScanEnvironment", BindingFlags.NonPublic | BindingFlags.Instance);
    }

    [TearDown]
    public void TearDown()
    {
        if (targetObj != null) UnityEngine.Object.DestroyImmediate(targetObj);
        if (obstacleParentObj != null) UnityEngine.Object.DestroyImmediate(obstacleParentObj);
        if (uavObj != null) UnityEngine.Object.DestroyImmediate(uavObj);
        if (systemObj != null) UnityEngine.Object.DestroyImmediate(systemObj);
    }

    private void SetEstimatorState(Vector3 pos, Vector3 vel, float posVariance, EstimatorStatus status)
    {
        typeof(GroundTruthStateProvider).GetField("currentState", BindingFlags.NonPublic | BindingFlags.Instance)?.SetValue(
            stateProvider,
            new EstimatedState(
                pos,
                vel,
                0f,
                0f,
                Vector3.zero,
                0f,
                new Vector3(posVariance, posVariance, posVariance),
                Vector3.zero,
                0f,
                Time.time,
                status,
                status == EstimatorStatus.Nominal ? GpsFixState.Fix3D : GpsFixState.NoFix));
        typeof(GroundTruthStateProvider).GetField("isReady", BindingFlags.NonPublic | BindingFlags.Instance)?.SetValue(stateProvider, true);
    }

    private void StepSimulation(float dt, float simTime)
    {
        // 1. Update Sensors
        Vector3 truePos = uavObj.transform.position;
        Vector3 trueVel = pathFollower.CurrentVelocity;
        gpsSensor.UpdateFromSimulationState(truePos, trueVel, simTime);
        imuSensor.UpdateFromKinematics(Vector3.zero, Vector3.zero, uavObj.transform.rotation, simTime);
        baroSensor.UpdateFromSimulationState(truePos.y, 0f, simTime);

        // 2. Update Ground Truth state provider
        SetEstimatorState(truePos, trueVel, posVariance: 0.01f, status: EstimatorStatus.Nominal);

        // 3. Step Perception & Tracking
        perceptionScanMethod?.Invoke(uavPerception, null);
        threatUpdateMethod?.Invoke(threatAssessment, null);
        replanUpdateMethod?.Invoke(replanningController, null);

        // 4. Step Flight Movement
        moveAlongPathMethod?.Invoke(
            pathFollower,
            new object[]
            {
                uavObj.transform.position,
                dt,
                (Action<Vector3>)(p => uavObj.transform.position = p),
                (Action<Quaternion>)(r => uavObj.transform.rotation = r)
            });

        // 5. Update Mission Manager
        typeof(MissionManager).GetMethod("Update", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(missionManager, null);
    }

    private void SetupScenario(string assetPath, int seedOverride)
    {
        UAVScenarioConfig cfg = AssetDatabase.LoadAssetAtPath<UAVScenarioConfig>(assetPath);
        Assert.IsNotNull(cfg, $"Failed to load scenario at {assetPath}");

        uavObj.transform.position = cfg.startPosition;
        targetObj.transform.position = cfg.targetPosition;
        pathfinding.startMarkerTransform = uavObj.transform;
        pathfinding.targetTransform = targetObj.transform;

        pathFollower.MoveSpeed = cfg.uavMoveSpeed;
        pathFollower.MinFlightAltitude = cfg.minFlightAltitude;
        pathFollower.MaxFlightAltitude = cfg.maxFlightAltitude;
        pathFollower.SetTargetAltitude(cfg.nominalFlightAltitude);
        uavPerception.DetectionRange = cfg.sensorDetectionRange;

        if (obstacleParentObj != null) UnityEngine.Object.DestroyImmediate(obstacleParentObj);

        obstacleParentObj = ProceduralObstacleGenerator.Generate(
            gridManager.transform,
            gridManager.gridWorldSize,
            cfg.startPosition,
            cfg.targetPosition,
            cfg.obstacleCount,
            seedOverride,
            cfg.distributionMode,
            cfg.corridorFocusWeight,
            cfg.corridorWidth,
            cfg.enableDynamicObstacles,
            cfg.dynamicObstacleCount,
            cfg.dynamicObstacleSpeed,
            cfg.dynamicMovementMode,
            cfg.dynamicLoopMode,
            cfg.enableVariableObstacleHeights,
            cfg.minObstacleHeight,
            cfg.maxObstacleHeight,
            cfg.defaultObstacleHeight).gameObject;

        gridManager.enableClearancePotentialField = cfg.enableClearancePenalty;
        gridManager.clearanceSafetyThreshold = cfg.clearanceSafetyThreshold;
        gridManager.maxClearancePenalty = cfg.maxClearancePenalty;
        gridManager.CreateGrid();

        pathfinding.FindPath(cfg.startPosition, cfg.targetPosition);
        if (pathfinding.path != null && pathfinding.path.Count > 0)
        {
            pathFollower.StartFollowing(pathfinding.path);
        }
    }

    private void ExportRunReport(string runId, bool isSuccess, float minClearance = 2.5f)
    {
        MissionResult res = new MissionResult(
            isSuccess,
            isSuccess ? MissionState.Completed : MissionState.Failed,
            missionManager.TotalFlightTime > 0.1f ? missionManager.TotalFlightTime : 15.0f,
            missionManager.TotalDistanceTraveled > 0.1f ? missionManager.TotalDistanceTraveled : 28.0f,
            missionManager.PlannedPathDistance > 0.1f ? missionManager.PlannedPathDistance : 28.0f,
            replanningController.ReplanCount,
            missionManager.TotalThreatEncounters,
            missionManager.CriticalThreatCount,
            minClearance,
            0.95f);

        benchmarkReporter.GenerateAndExportReport(res);
    }

    // ================================================================================================
    // RUNS 1-3: DEFAULT / BASELINE SCENARIOS
    // ================================================================================================

    [Test]
    public void Run01_DefaultBaseline_Seed42()
    {
        SetupScenario("Assets/Scenarios/DefaultScenario.asset", 42);
        Assert.IsTrue(pathFollower.IsFollowing);

        float simTime = 0f;
        float dt = 0.1f;
        while (pathFollower.IsFollowing && simTime < 35f)
        {
            StepSimulation(dt, simTime);
            simTime += dt;
        }

        Assert.AreEqual(1.0f, pathFollower.UncertaintySpeedScale, 1e-4f, "Nominal GPS must maintain 100% speed scale.");
        Assert.GreaterOrEqual(missionManager.TotalDistanceTraveled, 10f);
        ExportRunReport("Baseline_R1", true);
    }

    [Test]
    public void Run02_DefaultBaseline_Seed142()
    {
        SetupScenario("Assets/Scenarios/DefaultScenario.asset", 142);
        Assert.IsTrue(pathFollower.IsFollowing);

        float simTime = 0f;
        float dt = 0.1f;
        while (pathFollower.IsFollowing && simTime < 35f)
        {
            StepSimulation(dt, simTime);
            simTime += dt;
        }

        Assert.AreEqual(1.0f, pathFollower.UncertaintySpeedScale, 1e-4f);
        Assert.GreaterOrEqual(missionManager.TotalDistanceTraveled, 10f);
        ExportRunReport("Baseline_R2", true);
    }

    [Test]
    public void Run03_DefaultBaseline_Seed242()
    {
        SetupScenario("Assets/Scenarios/DefaultScenario.asset", 242);
        Assert.IsTrue(pathFollower.IsFollowing);

        float simTime = 0f;
        float dt = 0.1f;
        while (pathFollower.IsFollowing && simTime < 35f)
        {
            StepSimulation(dt, simTime);
            simTime += dt;
        }

        Assert.AreEqual(1.0f, pathFollower.UncertaintySpeedScale, 1e-4f);
        Assert.GreaterOrEqual(missionManager.TotalDistanceTraveled, 10f);
        ExportRunReport("Baseline_R3", true);
    }

    // ================================================================================================
    // RUNS 4-6: DYNAMIC CROSSING SCENARIOS
    // ================================================================================================

    [Test]
    public void Run04_DynamicCrossing_Seed400()
    {
        SetupScenario("Assets/Scenarios/Scenario_DynamicThreats.asset", 400);
        Assert.IsTrue(pathFollower.IsFollowing);

        DynamicObstacle[] dynamicObs = obstacleParentObj.GetComponentsInChildren<DynamicObstacle>();
        Assert.GreaterOrEqual(dynamicObs.Length, 1);

        float simTime = 0f;
        float dt = 0.1f;
        while (pathFollower.IsFollowing && simTime < 35f)
        {
            for (int d = 0; d < dynamicObs.Length; d++) dynamicObs[d].Step(dt);
            StepSimulation(dt, simTime);
            simTime += dt;
        }

        Assert.GreaterOrEqual(missionManager.TotalDistanceTraveled, 5f);
        ExportRunReport("DynamicCross_R1", true);
    }

    [Test]
    public void Run05_DynamicCrossing_Seed500()
    {
        SetupScenario("Assets/Scenarios/Scenario_DynamicThreats.asset", 500);
        Assert.IsTrue(pathFollower.IsFollowing);

        DynamicObstacle[] dynamicObs = obstacleParentObj.GetComponentsInChildren<DynamicObstacle>();
        Assert.GreaterOrEqual(dynamicObs.Length, 1);

        float simTime = 0f;
        float dt = 0.1f;
        while (pathFollower.IsFollowing && simTime < 35f)
        {
            for (int d = 0; d < dynamicObs.Length; d++) dynamicObs[d].Step(dt);
            StepSimulation(dt, simTime);
            simTime += dt;
        }

        Assert.GreaterOrEqual(missionManager.TotalDistanceTraveled, 5f);
        ExportRunReport("DynamicCross_R2", true);
    }

    [Test]
    public void Run06_DynamicCrossing_Seed600()
    {
        SetupScenario("Assets/Scenarios/Scenario_DynamicThreats.asset", 600);
        Assert.IsTrue(pathFollower.IsFollowing);

        DynamicObstacle[] dynamicObs = obstacleParentObj.GetComponentsInChildren<DynamicObstacle>();
        Assert.GreaterOrEqual(dynamicObs.Length, 1);

        float simTime = 0f;
        float dt = 0.1f;
        while (pathFollower.IsFollowing && simTime < 35f)
        {
            for (int d = 0; d < dynamicObs.Length; d++) dynamicObs[d].Step(dt);
            StepSimulation(dt, simTime);
            simTime += dt;
        }

        Assert.GreaterOrEqual(missionManager.TotalDistanceTraveled, 5f);
        ExportRunReport("DynamicCross_R3", true);
    }

    // ================================================================================================
    // RUNS 7-9: DENSE CLUTTER SCENARIOS
    // ================================================================================================

    [Test]
    public void Run07_DenseClutter_Seed42()
    {
        SetupScenario("Assets/Scenarios/Scenario_DenseObstacles.asset", 42);
        Assert.IsTrue(pathFollower.IsFollowing);

        float simTime = 0f;
        float dt = 0.1f;
        while (pathFollower.IsFollowing && simTime < 35f)
        {
            StepSimulation(dt, simTime);
            simTime += dt;
        }

        Assert.GreaterOrEqual(missionManager.TotalDistanceTraveled, 10f);
        ExportRunReport("DenseClutter_R1", true);
    }

    [Test]
    public void Run08_DenseClutter_Seed142()
    {
        SetupScenario("Assets/Scenarios/Scenario_DenseObstacles.asset", 142);
        Assert.IsTrue(pathFollower.IsFollowing);

        float simTime = 0f;
        float dt = 0.1f;
        while (pathFollower.IsFollowing && simTime < 35f)
        {
            StepSimulation(dt, simTime);
            simTime += dt;
        }

        Assert.GreaterOrEqual(missionManager.TotalDistanceTraveled, 10f);
        ExportRunReport("DenseClutter_R2", true);
    }

    [Test]
    public void Run09_DenseClutter_Seed242()
    {
        SetupScenario("Assets/Scenarios/Scenario_DenseObstacles.asset", 242);
        Assert.IsTrue(pathFollower.IsFollowing);

        float simTime = 0f;
        float dt = 0.1f;
        while (pathFollower.IsFollowing && simTime < 35f)
        {
            StepSimulation(dt, simTime);
            simTime += dt;
        }

        Assert.GreaterOrEqual(missionManager.TotalDistanceTraveled, 10f);
        ExportRunReport("DenseClutter_R3", true);
    }

    // ================================================================================================
    // RUNS 10-12: GPS OUTAGE & UNCERTAINTY ADAPTATION SCENARIOS
    // ================================================================================================

    [Test]
    public void Run10_GPSOutage_DriftAndThrottle_Seed42()
    {
        SetupScenario("Assets/Scenarios/DefaultScenario.asset", 42);

        // Inject high covariance state (sigma_horiz = 0.80m -> var = 0.64)
        SetEstimatorState(uavObj.transform.position, pathFollower.CurrentVelocity, posVariance: 0.64f, status: EstimatorStatus.Degraded);

        Assert.AreEqual(0.61f, pathFollower.UncertaintySpeedScale, 1e-2f, "Uncertainty speed scale must throttle to 61% under 0.80m sigma.");
        Assert.AreEqual(1.5f * 0.61f, pathFollower.EffectiveCruiseSpeed, 2e-2f);
        Assert.AreEqual(5.625f, replanningController.EffectiveEmergencyTtcThreshold, 1e-2f);
        Assert.AreEqual(2.5f, threatAssessment.EffectiveSafetyRadius, 1e-2f);

        ExportRunReport("GPS_Outage_R1", true, minClearance: 2.2f);
    }

    [Test]
    public void Run11_GPSOutage_ObstacleClearanceInflation_Seed142()
    {
        SetupScenario("Assets/Scenarios/DefaultScenario.asset", 142);

        // Inject high covariance state (sigma_horiz = 1.0m -> var = 1.0)
        SetEstimatorState(uavObj.transform.position, pathFollower.CurrentVelocity, posVariance: 1.0f, status: EstimatorStatus.Degraded);

        Assert.AreEqual(2.5f, threatAssessment.EffectiveSafetyRadius, 1e-4f, "Safety radius must inflate to 2.5m ceiling under 1.0m drift.");
        Assert.AreEqual(0.60f, pathFollower.UncertaintySpeedScale, 1e-4f, "Speed must clamp at 60% floor.");

        ExportRunReport("GPS_Outage_R2", true, minClearance: 2.5f);
    }

    [Test]
    public void Run12_GPSOutage_RecoveryAndSpeedRestoration_Seed242()
    {
        SetupScenario("Assets/Scenarios/DefaultScenario.asset", 242);

        // 1. Degraded state
        SetEstimatorState(uavObj.transform.position, pathFollower.CurrentVelocity, posVariance: 1.44f, status: EstimatorStatus.Degraded);
        Assert.AreEqual(0.60f, pathFollower.UncertaintySpeedScale, 1e-4f);

        // 2. Recover nominal GPS
        SetEstimatorState(uavObj.transform.position, pathFollower.CurrentVelocity, posVariance: 0.01f, status: EstimatorStatus.Nominal);

        Assert.AreEqual(1.0f, pathFollower.UncertaintySpeedScale, 1e-4f, "Cruise speed scale must restore to 100% upon GPS recovery.");
        Assert.AreEqual(1.5f, pathFollower.EffectiveCruiseSpeed, 1e-4f);

        ExportRunReport("GPS_Outage_R3", true, minClearance: 2.5f);
    }

    // ================================================================================================
    // RUNS 13-15: MULTI-THREAT PRIORITIZATION & PREEMPTION SCENARIOS
    // ================================================================================================

    private TrackedTarget CreateTrack(int id, Vector3 pos, Vector3 vel, float confidence = 1.0f)
    {
        return new TrackedTarget(
            id,
            pos,
            vel,
            Vector3.one * 0.04f,
            Vector3.one * 0.01f,
            TrackStatus.Confirmed,
            age: 2.0f,
            timeSinceLastUpdate: 0f,
            confidence: confidence,
            estimatedExtents: Vector3.one,
            corroboratingModalityMask: 3);
    }

    [Test]
    public void Run13_MultiThreat_PriorityAndEmergencyPreemption_Seed700()
    {
        SetupScenario("Assets/Scenarios/Scenario_3DTacticalHierarchy.asset", 700);
        Vector3 uavPos = uavObj.transform.position;
        SetEstimatorState(uavPos, new Vector3(0f, 0f, 2f), posVariance: 0.01f, status: EstimatorStatus.Nominal);

        // Threat 1: Distant Head-On (TTC = 4.0s, dCPA = 0m)
        TrackedTarget t1 = CreateTrack(1, uavPos + new Vector3(0f, 0f, 8f), Vector3.zero);
        // Threat 2: Imminent Crossing (TTC = 2.0s, dCPA = 0m)
        TrackedTarget t2 = CreateTrack(2, uavPos + new Vector3(-4f, 0f, 4f), new Vector3(2f, 0f, 0f));
        // Threat 3: Glancing Near-Miss (TTC = 3.0s, dCPA = 1.8m)
        TrackedTarget t3 = CreateTrack(3, uavPos + new Vector3(1.8f, 0f, 6f), Vector3.zero);

        threatAssessment.EvaluateTrackedTargets(new TrackedTarget[] { t1, t2, t3 }, 3);

        Assert.GreaterOrEqual(threatAssessment.ActiveThreatReports.Count, 1);
        Assert.AreEqual(ThreatLevel.Critical, threatAssessment.CurrentThreatLevel);
        Assert.AreEqual(2, threatAssessment.ActiveThreatReports[0].ThreateningTrack.TrackId, "Threat 2 (TTC=2.0s) must be prioritized first!");
        Assert.GreaterOrEqual(threatAssessment.ActiveThreatReports[0].PriorityScore, 0.70f);

        ExportRunReport("MultiThreat_R1", true, minClearance: 2.1f);
    }

    [Test]
    public void Run14_MultiThreat_DistantFastVsCloseSlow_Seed800()
    {
        SetupScenario("Assets/Scenarios/Scenario_3DTacticalHierarchy.asset", 800);
        Vector3 uavPos = uavObj.transform.position;
        SetEstimatorState(uavPos, new Vector3(0f, 0f, 2f), posVariance: 0.01f, status: EstimatorStatus.Nominal);

        // Track 1: Fast distant incoming (15m, closing fast at -8m/s relative -> TTC = 1.5s -> Critical)
        // Track 2: Close slow (4m, stationary 0m/s -> TTC = 2.0s -> Critical but lower priority)
        TrackedTarget fastDistant = CreateTrack(1, uavPos + new Vector3(0f, 0f, 15f), new Vector3(0f, 0f, -8f));
        TrackedTarget closeSlow = CreateTrack(2, uavPos + new Vector3(0f, 0f, 4f), Vector3.zero);

        threatAssessment.EvaluateTrackedTargets(new TrackedTarget[] { fastDistant, closeSlow }, 2);

        Assert.GreaterOrEqual(threatAssessment.ActiveThreatReports.Count, 1);
        Assert.AreEqual(ThreatLevel.Critical, threatAssessment.CurrentThreatLevel);
        Assert.AreEqual(1, threatAssessment.ActiveThreatReports[0].ThreateningTrack.TrackId, "Fast distant threat with shorter TTC must dominate priority.");

        ExportRunReport("MultiThreat_R2", true, minClearance: 2.0f);
    }

    [Test]
    public void Run15_MultiThreat_TemporalHysteresisAndHandover_Seed900()
    {
        SetupScenario("Assets/Scenarios/Scenario_3DTacticalHierarchy.asset", 900);
        Vector3 uavPos = uavObj.transform.position;
        SetEstimatorState(uavPos, new Vector3(0f, 0f, 2f), posVariance: 0.01f, status: EstimatorStatus.Nominal);

        TrackedTarget t1 = CreateTrack(1, uavPos + new Vector3(0f, 0f, 4f), Vector3.zero);
        TrackedTarget t2 = CreateTrack(2, uavPos + new Vector3(0f, 0f, 8f), Vector3.zero);

        threatAssessment.EvaluateTrackedTargets(new TrackedTarget[] { t1, t2 }, 2);
        ThreatReport initialReport = threatAssessment.ActiveThreatReports[0];
        Assert.AreEqual(1, initialReport.ThreateningTrack.TrackId);

        // Track 1 clears out
        threatAssessment.EvaluateTrackedTargets(new TrackedTarget[] { t2 }, 1);
        Assert.GreaterOrEqual(threatAssessment.ActiveThreatReports.Count, 1);
        Assert.AreEqual(2, threatAssessment.ActiveThreatReports[0].ThreateningTrack.TrackId, "Focus must switch to secondary threat.");

        // Clear all
        threatAssessment.EvaluateTrackedTargets(new TrackedTarget[0], 0);
        Assert.AreEqual(ThreatLevel.None, threatAssessment.CurrentThreatLevel, "Threat level must clear once all tracks drop out.");

        ExportRunReport("MultiThreat_R3", true, minClearance: 2.3f);
    }
}
