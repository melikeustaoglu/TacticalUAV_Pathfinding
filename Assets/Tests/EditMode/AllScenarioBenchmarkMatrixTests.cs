using System.Collections.Generic;
using System.IO;
using System.Reflection;
using NUnit.Framework;
using UnityEditor;
using UnityEngine;

/// <summary>
/// Phase 10.7 Comprehensive 8-Scenario Benchmark Matrix & Tactical Explainability Test Fixture.
/// Validates all 8 scenario assets in the project:
///   1. DefaultScenario.asset
///   2. Scenario_AlternativeSeed.asset
///   3. Scenario_DenseObstacles.asset
///   4. Scenario_DynamicThreats.asset
///   5. Scenario_LongRange.asset
///   6. Scenario_VOPacingValidation.asset
///   7. Scenario_3DVerticalClimb.asset
///   8. Scenario_3DTacticalHierarchy.asset
/// </summary>
[TestFixture]
public class AllScenarioBenchmarkMatrixTests
{
    private GameObject uavObj;
    private GameObject obstacleParentObj;
    private GameObject targetObj;
    private PathFollower pathFollower;
    private UAVPerception uavPerception;
    private ThreatAssessment threatAssessment;
    private ReplanningController replanningController;
    private MissionManager missionManager;
    private MissionEventLogger eventLogger;
    private TacticalHUD tacticalHUD;
    private BenchmarkReporter benchmarkReporter;
    private GridManager gridManager;
    private Pathfinding pathfinding;

    private readonly string[] allScenarioPaths = new string[]
    {
        "Assets/Scenarios/DefaultScenario.asset",
        "Assets/Scenarios/Scenario_AlternativeSeed.asset",
        "Assets/Scenarios/Scenario_DenseObstacles.asset",
        "Assets/Scenarios/Scenario_DynamicThreats.asset",
        "Assets/Scenarios/Scenario_LongRange.asset",
        "Assets/Scenarios/Scenario_VOPacingValidation.asset",
        "Assets/Scenarios/Scenario_3DVerticalClimb.asset",
        "Assets/Scenarios/Scenario_3DTacticalHierarchy.asset"
    };

    [SetUp]
    public void SetUp()
    {
        uavObj = new GameObject("BenchmarkMatrixUAV");
        gridManager = uavObj.AddComponent<GridManager>();
        gridManager.gridWorldSize = new Vector2(50f, 50f);
        gridManager.nodeRadius = 0.5f;
        gridManager.obstacleMask = ProceduralObstacleGenerator.GetObstacleMask();

        pathfinding = uavObj.AddComponent<Pathfinding>();
        typeof(Pathfinding).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(pathfinding, null);
        gridManager.CreateGrid();

        pathFollower = uavObj.AddComponent<PathFollower>();
        uavPerception = uavObj.AddComponent<UAVPerception>();
        threatAssessment = uavObj.AddComponent<ThreatAssessment>();
        replanningController = uavObj.AddComponent<ReplanningController>();
        missionManager = uavObj.AddComponent<MissionManager>();
        eventLogger = uavObj.AddComponent<MissionEventLogger>();
        tacticalHUD = uavObj.AddComponent<TacticalHUD>();
        benchmarkReporter = uavObj.AddComponent<BenchmarkReporter>();

        typeof(PathFollower).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(pathFollower, null);
        typeof(UAVPerception).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(uavPerception, null);
        typeof(ThreatAssessment).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(threatAssessment, null);
        typeof(ReplanningController).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(replanningController, null);
        typeof(MissionManager).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(missionManager, null);
        typeof(MissionEventLogger).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(eventLogger, null);
        typeof(TacticalHUD).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(tacticalHUD, null);
        typeof(BenchmarkReporter).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(benchmarkReporter, null);

        typeof(ReplanningController).GetMethod("Start", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(replanningController, null);

        targetObj = new GameObject("BenchmarkMatrixTarget");
        targetObj.transform.position = new Vector3(10f, 1f, 10f);
        pathfinding.targetTransform = targetObj.transform;

        pathFollower.MoveSpeed = 1.5f;
        pathFollower.MinFlightAltitude = 1.0f;
        pathFollower.MaxFlightAltitude = 6.0f;
        pathFollower.MaxClimbRate = 1.5f;
        pathFollower.MaxDescentRate = 2.0f;
    }

    [TearDown]
    public void TearDown()
    {
        if (targetObj != null) UnityEngine.Object.DestroyImmediate(targetObj);
        if (obstacleParentObj != null) UnityEngine.Object.DestroyImmediate(obstacleParentObj);
        if (uavObj != null) UnityEngine.Object.DestroyImmediate(uavObj);
    }

    [Test]
    public void BenchmarkMatrix_AllEightScenarioAssets_ExistAndLoadCorrectly()
    {
        Assert.AreEqual(8, allScenarioPaths.Length, "Must validate exactly 8 scenario assets!");

        foreach (string path in allScenarioPaths)
        {
            Assert.IsTrue(File.Exists(path), $"Scenario asset file missing at '{path}'!");
            UAVScenarioConfig config = AssetDatabase.LoadAssetAtPath<UAVScenarioConfig>(path);
            Assert.IsNotNull(config, $"Failed to load UAVScenarioConfig from '{path}'!");

            Assert.IsTrue(config.obstacleCount >= 0, $"Negative obstacle count in '{path}'!");
            Assert.IsTrue(config.uavMoveSpeed > 0f, $"Invalid UAV move speed in '{path}'!");
            Assert.IsTrue(config.sensorDetectionRange > 0f, $"Invalid sensor detection range in '{path}'!");
            Assert.IsTrue(float.IsFinite(config.startPosition.x) && float.IsFinite(config.startPosition.z), $"Invalid start position in '{path}'!");
            Assert.IsTrue(float.IsFinite(config.targetPosition.x) && float.IsFinite(config.targetPosition.z), $"Invalid target position in '{path}'!");
        }
    }

    [Test]
    public void BenchmarkMatrix_Scenario_DefaultScenario_InitializesAndGeneratesConsistentAirspace()
    {
        UAVScenarioConfig config = AssetDatabase.LoadAssetAtPath<UAVScenarioConfig>("Assets/Scenarios/DefaultScenario.asset");
        Assert.IsNotNull(config);

        obstacleParentObj = ProceduralObstacleGenerator.Generate(
            gridManager.transform,
            gridManager.gridWorldSize,
            config.startPosition,
            config.targetPosition,
            config.obstacleCount,
            config.seed,
            config.distributionMode,
            config.corridorFocusWeight,
            config.corridorWidth,
            config.enableDynamicObstacles,
            config.dynamicObstacleCount,
            config.dynamicObstacleSpeed,
            ObstacleMovementMode.Patrol,
            PatrolLoopMode.PingPong,
            config.enableVariableObstacleHeights,
            config.minObstacleHeight,
            config.maxObstacleHeight).gameObject;

        gridManager.CreateGrid();
        pathfinding.FindPath(config.startPosition, config.targetPosition);

        Assert.IsNotNull(pathfinding.path, "DefaultScenario must find an initial valid corridor!");
        Assert.IsTrue(pathfinding.path.Count > 0);
        Assert.AreEqual(10, obstacleParentObj.transform.childCount);
    }

    [Test]
    public void BenchmarkMatrix_Scenario_AlternativeSeed_InitializesAndGeneratesDeterministicAirspace()
    {
        UAVScenarioConfig config = AssetDatabase.LoadAssetAtPath<UAVScenarioConfig>("Assets/Scenarios/Scenario_AlternativeSeed.asset");
        Assert.IsNotNull(config);
        Assert.AreEqual(100, config.seed);

        obstacleParentObj = ProceduralObstacleGenerator.Generate(
            gridManager.transform,
            gridManager.gridWorldSize,
            config.startPosition,
            config.targetPosition,
            config.obstacleCount,
            config.seed).gameObject;

        gridManager.CreateGrid();
        pathfinding.FindPath(config.startPosition, config.targetPosition);

        Assert.IsNotNull(pathfinding.path);
        Assert.IsTrue(pathfinding.path.Count > 0);
    }

    [Test]
    public void BenchmarkMatrix_Scenario_DenseObstacles_InitializesAndHandlesHighDensityGrid()
    {
        UAVScenarioConfig config = AssetDatabase.LoadAssetAtPath<UAVScenarioConfig>("Assets/Scenarios/Scenario_DenseObstacles.asset");
        Assert.IsNotNull(config);
        Assert.AreEqual(18, config.obstacleCount);

        obstacleParentObj = ProceduralObstacleGenerator.Generate(
            gridManager.transform,
            gridManager.gridWorldSize,
            config.startPosition,
            config.targetPosition,
            config.obstacleCount,
            config.seed).gameObject;

        gridManager.CreateGrid();
        pathfinding.FindPath(config.startPosition, config.targetPosition);

        Assert.AreEqual(18, obstacleParentObj.transform.childCount);
    }

    [Test]
    public void BenchmarkMatrix_Scenario_DynamicThreats_SpawnsDynamicMovingObstacles()
    {
        UAVScenarioConfig config = AssetDatabase.LoadAssetAtPath<UAVScenarioConfig>("Assets/Scenarios/Scenario_DynamicThreats.asset");
        Assert.IsNotNull(config);
        Assert.IsTrue(config.enableDynamicObstacles);
        Assert.AreEqual(2, config.dynamicObstacleCount);

        obstacleParentObj = ProceduralObstacleGenerator.Generate(
            gridManager.transform,
            gridManager.gridWorldSize,
            config.startPosition,
            config.targetPosition,
            config.obstacleCount,
            config.seed,
            config.distributionMode,
            config.corridorFocusWeight,
            config.corridorWidth,
            config.enableDynamicObstacles,
            config.dynamicObstacleCount,
            config.dynamicObstacleSpeed,
            ObstacleMovementMode.Patrol,
            PatrolLoopMode.PingPong,
            config.enableVariableObstacleHeights,
            config.minObstacleHeight,
            config.maxObstacleHeight).gameObject;

        DynamicObstacle[] dynamicObs = obstacleParentObj.GetComponentsInChildren<DynamicObstacle>();
        Assert.AreEqual(2, dynamicObs.Length, "Must spawn exactly 2 dynamic obstacles!");

        for (int i = 0; i < dynamicObs.Length; i++)
        {
            Assert.AreEqual(config.dynamicObstacleSpeed, dynamicObs[i].Speed, 0.01f);
            Assert.IsTrue(dynamicObs[i].MovementEnabled);
        }
    }

    [Test]
    public void BenchmarkMatrix_Scenario_LongRange_ValidatesExtendedAirspaceCorridor()
    {
        UAVScenarioConfig config = AssetDatabase.LoadAssetAtPath<UAVScenarioConfig>("Assets/Scenarios/Scenario_LongRange.asset");
        Assert.IsNotNull(config);

        obstacleParentObj = ProceduralObstacleGenerator.Generate(
            gridManager.transform,
            gridManager.gridWorldSize,
            config.startPosition,
            config.targetPosition,
            config.obstacleCount,
            config.seed).gameObject;

        gridManager.CreateGrid();
        pathfinding.FindPath(config.startPosition, config.targetPosition);

        Assert.IsNotNull(pathfinding.path);
        float directDist = Vector3.Distance(config.startPosition, config.targetPosition);
        Assert.GreaterOrEqual(directDist, 30f, "Long range scenario must span an extended corridor!");
    }

    [Test]
    public void BenchmarkMatrix_Scenario_VOPacingValidation_ExecutesVOPacingConfig()
    {
        UAVScenarioConfig config = AssetDatabase.LoadAssetAtPath<UAVScenarioConfig>("Assets/Scenarios/Scenario_VOPacingValidation.asset");
        Assert.IsNotNull(config);
        Assert.IsTrue(config.enableDynamicObstacles);

        obstacleParentObj = ProceduralObstacleGenerator.Generate(
            gridManager.transform,
            gridManager.gridWorldSize,
            config.startPosition,
            config.targetPosition,
            config.obstacleCount,
            config.seed,
            config.distributionMode,
            config.corridorFocusWeight,
            config.corridorWidth,
            config.enableDynamicObstacles,
            config.dynamicObstacleCount,
            config.dynamicObstacleSpeed,
            ObstacleMovementMode.Patrol,
            PatrolLoopMode.PingPong,
            config.enableVariableObstacleHeights,
            config.minObstacleHeight,
            config.maxObstacleHeight).gameObject;

        DynamicObstacle[] dynamicObs = obstacleParentObj.GetComponentsInChildren<DynamicObstacle>();
        Assert.GreaterOrEqual(dynamicObs.Length, 1);
    }

    [Test]
    public void BenchmarkMatrix_Scenario_3DVerticalClimb_ValidatesLowCeilingAirspace()
    {
        UAVScenarioConfig config = AssetDatabase.LoadAssetAtPath<UAVScenarioConfig>("Assets/Scenarios/Scenario_3DVerticalClimb.asset");
        Assert.IsNotNull(config);
        Assert.IsTrue(config.enableVariableObstacleHeights);
        Assert.AreEqual(1.5f, config.minObstacleHeight, 0.01f);
        Assert.AreEqual(2.2f, config.maxObstacleHeight, 0.01f);
        Assert.AreEqual(6.0f, config.maxFlightAltitude, 0.01f);

        obstacleParentObj = ProceduralObstacleGenerator.Generate(
            gridManager.transform,
            gridManager.gridWorldSize,
            config.startPosition,
            config.targetPosition,
            config.obstacleCount,
            config.seed,
            config.distributionMode,
            config.corridorFocusWeight,
            config.corridorWidth,
            config.enableDynamicObstacles,
            config.dynamicObstacleCount,
            config.dynamicObstacleSpeed,
            ObstacleMovementMode.Patrol,
            PatrolLoopMode.PingPong,
            config.enableVariableObstacleHeights,
            config.minObstacleHeight,
            config.maxObstacleHeight).gameObject;

        for (int i = 0; i < obstacleParentObj.transform.childCount; i++)
        {
            Transform child = obstacleParentObj.transform.GetChild(i);
            float height = child.localScale.y;
            Assert.GreaterOrEqual(height, 1.49f);
            Assert.LessOrEqual(height, 2.21f);
        }
    }

    [Test]
    public void BenchmarkMatrix_Scenario_3DTacticalHierarchy_ValidatesFullHierarchyStack()
    {
        UAVScenarioConfig config = AssetDatabase.LoadAssetAtPath<UAVScenarioConfig>("Assets/Scenarios/Scenario_3DTacticalHierarchy.asset");
        Assert.IsNotNull(config);
        Assert.IsTrue(config.enableVariableObstacleHeights);
        Assert.AreEqual(7.5f, config.maxObstacleHeight, 0.01f);
        Assert.IsTrue(config.enableDynamicObstacles);
        Assert.AreEqual(2, config.dynamicObstacleCount);

        obstacleParentObj = ProceduralObstacleGenerator.Generate(
            gridManager.transform,
            gridManager.gridWorldSize,
            config.startPosition,
            config.targetPosition,
            config.obstacleCount,
            config.seed,
            config.distributionMode,
            config.corridorFocusWeight,
            config.corridorWidth,
            config.enableDynamicObstacles,
            config.dynamicObstacleCount,
            config.dynamicObstacleSpeed,
            ObstacleMovementMode.Patrol,
            PatrolLoopMode.PingPong,
            config.enableVariableObstacleHeights,
            config.minObstacleHeight,
            config.maxObstacleHeight).gameObject;

        DynamicObstacle[] dynamicObs = obstacleParentObj.GetComponentsInChildren<DynamicObstacle>();
        Assert.AreEqual(2, dynamicObs.Length);
    }

    [Test]
    public void BenchmarkMatrix_AggregateMatrix_AllScenariosExecuteWithValidTelemetry()
    {
        for (int sIdx = 0; sIdx < allScenarioPaths.Length; sIdx++)
        {
            string path = allScenarioPaths[sIdx];
            UAVScenarioConfig config = AssetDatabase.LoadAssetAtPath<UAVScenarioConfig>(path);
            Assert.IsNotNull(config, $"Config null for '{path}'");

            if (obstacleParentObj != null) UnityEngine.Object.DestroyImmediate(obstacleParentObj);

            obstacleParentObj = ProceduralObstacleGenerator.Generate(
                gridManager.transform,
                gridManager.gridWorldSize,
                config.startPosition,
                config.targetPosition,
                config.obstacleCount,
                config.seed,
                config.distributionMode,
                config.corridorFocusWeight,
                config.corridorWidth,
                config.enableDynamicObstacles,
                config.dynamicObstacleCount,
                config.dynamicObstacleSpeed,
                ObstacleMovementMode.Patrol,
                PatrolLoopMode.PingPong,
                config.enableVariableObstacleHeights,
                config.minObstacleHeight,
                config.maxObstacleHeight).gameObject;

            gridManager.CreateGrid();
            pathfinding.FindPath(config.startPosition, config.targetPosition);

            uavObj.transform.position = config.startPosition;
            pathFollower.MoveSpeed = config.uavMoveSpeed;
            uavPerception.DetectionRange = config.sensorDetectionRange;

            // Generate synthetic completed result to verify benchmark reporting
            MissionResult simulatedResult = new MissionResult(
                true,
                MissionState.Completed,
                20.0f,
                30.0f,
                28.0f,
                replanningController.ReplanCount,
                missionManager.TotalThreatEncounters,
                missionManager.CriticalThreatCount,
                2.5f,
                28.0f / 30.0f);

            MissionBenchmarkReport report = benchmarkReporter.GenerateAndExportReport(simulatedResult);

            Assert.IsNotNull(report, $"Benchmark report null for '{path}'!");
            Assert.AreEqual(config.uavMoveSpeed, report.cruiseSpeed, 0.01f);
            Assert.AreEqual(config.sensorDetectionRange, report.sensorRange, 0.01f);
            Assert.IsTrue(float.IsFinite(report.maxFlightAltitude), $"Non-finite maxFlightAltitude in '{path}'!");
            Assert.IsTrue(float.IsFinite(report.nominalFlightAltitude), $"Non-finite nominalFlightAltitude in '{path}'!");
            Assert.IsTrue(float.IsFinite(report.peakAltitudeReached), $"Non-finite peakAltitudeReached in '{path}'!");
            Assert.IsNotNull(report.dominantTacticalDecision, $"dominantTacticalDecision null in '{path}'!");
            Assert.IsNotNull(benchmarkReporter.LastReportJson, $"LastReportJson null in '{path}'!");
            Assert.IsTrue(benchmarkReporter.LastReportJson.Contains("dominantTacticalDecision"), "JSON must serialize dominantTacticalDecision!");
            Assert.IsTrue(benchmarkReporter.LastReportJson.Contains("verticalCeilingRejections"), "JSON must serialize verticalCeilingRejections!");
        }
    }
}
