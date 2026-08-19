using System.Collections.Generic;
using System.IO;
using System.Reflection;
using NUnit.Framework;
using UnityEditor;
using UnityEngine;

[TestFixture]
public class ThreeDimensionalScenarioIntegrationTests
{
    private GameObject uavObj;
    private GameObject obstacleParentObj;
    private GameObject targetObj;
    private PathFollower pathFollower;
    private UAVPerception uavPerception;
    private ThreatAssessment threatAssessment;
    private ReplanningController replanningController;
    private GridManager gridManager;
    private Pathfinding pathfinding;

    [SetUp]
    public void SetUp()
    {
        uavObj = new GameObject("IntegrationTestUAV");
        gridManager = uavObj.AddComponent<GridManager>();
        gridManager.gridWorldSize = new Vector2(40f, 40f);
        gridManager.nodeRadius = 0.5f;
        gridManager.obstacleMask = ProceduralObstacleGenerator.GetObstacleMask();

        pathfinding = uavObj.AddComponent<Pathfinding>();
        typeof(Pathfinding).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(pathfinding, null);
        gridManager.CreateGrid();

        pathFollower = uavObj.AddComponent<PathFollower>();
        uavPerception = uavObj.AddComponent<UAVPerception>();
        threatAssessment = uavObj.AddComponent<ThreatAssessment>();
        replanningController = uavObj.AddComponent<ReplanningController>();

        uavPerception.ObstacleMask = gridManager.obstacleMask;

        typeof(PathFollower).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(pathFollower, null);
        typeof(UAVPerception).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(uavPerception, null);
        typeof(ThreatAssessment).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(threatAssessment, null);
        typeof(ReplanningController).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(replanningController, null);

        typeof(ReplanningController).GetMethod("Start", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(replanningController, null);

        targetObj = new GameObject("IntegrationTestTarget");
        targetObj.transform.position = new Vector3(0f, 1f, 20f);
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
        if (targetObj != null) Object.DestroyImmediate(targetObj);
        if (obstacleParentObj != null) Object.DestroyImmediate(obstacleParentObj);
        if (uavObj != null) Object.DestroyImmediate(uavObj);
    }

    [Test]
    public void Scenario3DClimb_AssetLoading_AndValidation()
    {
        string assetPath = "Assets/Scenarios/Scenario_3DVerticalClimb.asset";
        UAVScenarioConfig config = AssetDatabase.LoadAssetAtPath<UAVScenarioConfig>(assetPath);

        Assert.IsNotNull(config, $"Failed to load UAVScenarioConfig at '{assetPath}'!");
        Assert.IsTrue(config.enableVariableObstacleHeights, "Variable obstacle heights must be enabled!");
        Assert.AreEqual(1.5f, config.minObstacleHeight, 0.01f);
        Assert.AreEqual(2.2f, config.maxObstacleHeight, 0.01f);
        Assert.AreEqual(6.0f, config.maxFlightAltitude, 0.01f);
        Assert.AreEqual(1.0f, config.nominalFlightAltitude, 0.01f);
        Assert.IsFalse(config.enableDynamicObstacles);
    }

    [Test]
    public void Scenario3DTacticalHierarchy_AssetLoading_AndValidation()
    {
        string assetPath = "Assets/Scenarios/Scenario_3DTacticalHierarchy.asset";
        UAVScenarioConfig config = AssetDatabase.LoadAssetAtPath<UAVScenarioConfig>(assetPath);

        Assert.IsNotNull(config, $"Failed to load UAVScenarioConfig at '{assetPath}'!");
        Assert.IsTrue(config.enableVariableObstacleHeights, "Variable obstacle heights must be enabled!");
        Assert.AreEqual(7.5f, config.maxObstacleHeight, 0.01f);
        Assert.IsTrue(config.enableDynamicObstacles, "Dynamic obstacles must be enabled!");
        Assert.AreEqual(2, config.dynamicObstacleCount);
        Assert.AreEqual(1.2f, config.dynamicObstacleSpeed, 0.01f);
    }

    [Test]
    public void Scenario3D_ProceduralGeneration_DualRunDeterminism()
    {
        GameObject gridObjA = new GameObject("GridA");
        GameObject gridObjB = new GameObject("GridB");
        Vector3 start = new Vector3(-10f, 1f, -10f);
        Vector3 target = new Vector3(10f, 1f, 10f);
        Vector2 size = new Vector2(40f, 40f);

        Transform obstaclesA = ProceduralObstacleGenerator.Generate(
            gridObjA.transform, size, start, target, 8, 501,
            ObstacleDistributionMode.CorridorFocused, 0.8f, 8.0f,
            false, 0, 1.0f, ObstacleMovementMode.Patrol, PatrolLoopMode.PingPong,
            true, 1.5f, 2.2f, 2.0f);

        Transform obstaclesB = ProceduralObstacleGenerator.Generate(
            gridObjB.transform, size, start, target, 8, 501,
            ObstacleDistributionMode.CorridorFocused, 0.8f, 8.0f,
            false, 0, 1.0f, ObstacleMovementMode.Patrol, PatrolLoopMode.PingPong,
            true, 1.5f, 2.2f, 2.0f);

        Assert.AreEqual(obstaclesA.childCount, obstaclesB.childCount, "Child obstacle count must match exactly!");
        Assert.Greater(obstaclesA.childCount, 0, "Must generate at least 1 obstacle!");

        for (int i = 0; i < obstaclesA.childCount; i++)
        {
            Transform a = obstaclesA.GetChild(i);
            Transform b = obstaclesB.GetChild(i);

            Assert.AreEqual(a.position.x, b.position.x, 0.0001f);
            Assert.AreEqual(a.position.y, b.position.y, 0.0001f);
            Assert.AreEqual(a.position.z, b.position.z, 0.0001f);

            Assert.AreEqual(a.localScale.x, b.localScale.x, 0.0001f);
            Assert.AreEqual(a.localScale.y, b.localScale.y, 0.0001f);
            Assert.AreEqual(a.localScale.z, b.localScale.z, 0.0001f);

            float bottomY = a.position.y - (a.localScale.y / 2f);
            Assert.AreEqual(0f, bottomY, 0.001f, $"Obstacle '{a.name}' must be ground-aligned at Y = 0!");
        }

        Object.DestroyImmediate(obstaclesA.gameObject);
        Object.DestroyImmediate(obstaclesB.gameObject);
        Object.DestroyImmediate(gridObjA);
        Object.DestroyImmediate(gridObjB);
    }

    [Test]
    public void Scenario3D_Stage1_DynamicThreat_FullPipelineE2E()
    {
        // 1. Setup UAV on flight path towards Z = 20m
        uavObj.transform.position = new Vector3(0f, 1f, 0f);
        uavObj.transform.forward = Vector3.forward;

        List<Node> flightPath = new List<Node>
        {
            new Node(true, new Vector3(0f, 1f, 20f), 0, 0)
        };
        pathFollower.StartFollowing(flightPath);

        // 2. Spawn actual dynamic obstacle on Obstacle layer crossing the corridor at Z = 5m
        obstacleParentObj = GameObject.CreatePrimitive(PrimitiveType.Cube);
        obstacleParentObj.name = "DynamicObstacle_E2E";
        obstacleParentObj.layer = LayerMask.NameToLayer(ProceduralObstacleGenerator.ObstacleLayerName);
        obstacleParentObj.transform.position = new Vector3(-5f, 1f, 5f);
        obstacleParentObj.transform.localScale = Vector3.one * 1.5f;

        DynamicObstacle dynComp = obstacleParentObj.AddComponent<DynamicObstacle>();
        dynComp.Speed = 1.5f;
        dynComp.MovementMode = ObstacleMovementMode.Patrol;
        dynComp.MovementEnabled = true;
        dynComp.SetPatrolWaypoints(new List<Vector3>
        {
            new Vector3(-5f, 1f, 5f),
            new Vector3(5f, 1f, 5f)
        });
        dynComp.Step(0.02f);

        Physics.SyncTransforms();

        // 3. Trigger full perception scan
        uavPerception.PerformScan();
        Assert.IsTrue(uavPerception.HasObstacles, "Perception sensor must detect the dynamic obstacle!");
        Assert.IsTrue(uavPerception.NearestObstacle.IsDynamic, "Detected obstacle must be recognized as dynamic!");

        // 4. Trigger ThreatAssessment -> fires event -> ReplanningController executes Stage 1
        threatAssessment.EvaluateThreats();

        // 5. Verify physical/runtime state changes without direct counter or method invocation
        Assert.IsTrue(pathFollower.IsSpeedOverrideActive, "PathFollower must have active tactical speed override!");
        Assert.AreEqual(1, replanningController.SpeedPacingCount, "Speed pacing count must increment!");
        Assert.AreEqual(0, replanningController.VerticalEvasionCount, "Vertical evasion must not be used for Stage 1!");
        Assert.AreEqual(0, replanningController.SpatialReplanCount, "Spatial A* must not be used for Stage 1!");
        Assert.AreEqual(NavigationState.Rerouting, replanningController.State, "NavigationState must transition to Rerouting!");
    }

    [Test]
    public void Scenario3D_Stage2_VerticalStepClimb_FullPipelineE2E()
    {
        // 1. Setup UAV on flight path towards Z = 20m
        uavObj.transform.position = new Vector3(0f, 1f, 0f);
        uavObj.transform.forward = Vector3.forward;

        List<Node> flightPath = new List<Node>
        {
            new Node(true, new Vector3(0f, 1f, 20f), 0, 0)
        };
        pathFollower.StartFollowing(flightPath);

        // 2. Spawn actual ground-aligned static obstacle of height 2.0m on Obstacle layer
        obstacleParentObj = GameObject.CreatePrimitive(PrimitiveType.Cube);
        obstacleParentObj.name = "LowStaticObstacle_E2E";
        obstacleParentObj.layer = LayerMask.NameToLayer(ProceduralObstacleGenerator.ObstacleLayerName);
        obstacleParentObj.transform.position = new Vector3(0f, 1.0f, 5f);
        obstacleParentObj.transform.localScale = new Vector3(2f, 2.0f, 2f);

        BoxCollider col = obstacleParentObj.GetComponent<BoxCollider>();
        Physics.SyncTransforms();

        float obstacleTopY = col.bounds.max.y;
        float expectedTargetAltitude = obstacleTopY + threatAssessment.VerticalSafetyMargin;

        // 3. Trigger full perception scan
        uavPerception.PerformScan();
        Assert.IsTrue(uavPerception.HasObstacles, "Perception sensor must detect the static obstacle!");
        Assert.IsFalse(uavPerception.NearestObstacle.IsDynamic, "Detected obstacle must be static!");

        // 4. Trigger ThreatAssessment -> fires event -> ReplanningController executes Stage 2
        threatAssessment.EvaluateThreats();

        // 5. Verify physical/runtime state changes without direct counter or method invocation
        Assert.AreEqual(expectedTargetAltitude, pathFollower.TargetAltitude, 0.01f, "Target altitude must equal obstacle top + safety margin!");
        Assert.AreEqual(1, replanningController.VerticalEvasionCount, "Vertical evasion count must increment!");
        Assert.AreEqual(0, replanningController.SpeedPacingCount, "Speed pacing must not be used for static obstacle!");
        Assert.AreEqual(0, replanningController.SpatialReplanCount, "Spatial A* must not be used when vertical climb is feasible!");
        Assert.AreEqual(NavigationState.Rerouting, replanningController.State, "NavigationState must transition to Rerouting!");
        Assert.AreEqual(1, pathFollower.RemainingPath.Count, "Horizontal path geometry must remain unchanged!");
    }

    [Test]
    public void Scenario3D_Stage3_ToweringObstacle_SpatialAStar_FullPipelineE2E()
    {
        // 1. Setup UAV on flight path towards Z = 20m
        uavObj.transform.position = new Vector3(0f, 1f, 0f);
        uavObj.transform.forward = Vector3.forward;

        List<Node> flightPath = new List<Node>
        {
            new Node(true, new Vector3(0f, 1f, 20f), 0, 0)
        };
        pathFollower.StartFollowing(flightPath);

        // 2. Spawn actual towering static obstacle of height 7.5m (> flight ceiling 6.0m) directly blocking path
        obstacleParentObj = GameObject.CreatePrimitive(PrimitiveType.Cube);
        obstacleParentObj.name = "ToweringObstacle_E2E";
        obstacleParentObj.layer = LayerMask.NameToLayer(ProceduralObstacleGenerator.ObstacleLayerName);
        obstacleParentObj.transform.position = new Vector3(0f, 3.75f, 8f);
        obstacleParentObj.transform.localScale = new Vector3(3f, 7.5f, 3f);

        Physics.SyncTransforms();

        // 3. Trigger full perception scan
        uavPerception.PerformScan();
        Assert.IsTrue(uavPerception.HasObstacles, "Perception sensor must detect the towering obstacle!");

        // 4. Trigger ThreatAssessment -> fires event -> ReplanningController escalates to Stage 3
        threatAssessment.EvaluateThreats();

        // 5. Verify physical/runtime state changes without direct counter or method invocation
        Assert.AreEqual(1, replanningController.SpatialReplanCount, "Spatial A* replan count must increment!");
        Assert.AreEqual(0, replanningController.VerticalEvasionCount, "Vertical evasion must not be used for obstacle exceeding ceiling!");
        Assert.AreEqual(0, replanningController.SpeedPacingCount, "Speed pacing must not be used for static obstacle!");
        Assert.AreEqual(NavigationState.Rerouting, replanningController.State, "NavigationState must transition to Rerouting!");
        Assert.Greater(pathFollower.RemainingPath.Count, 0, "A* detour path must be calculated and assigned to PathFollower!");
    }
}
