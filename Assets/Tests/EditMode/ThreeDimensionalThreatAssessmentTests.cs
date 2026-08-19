using System.Collections.Generic;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;

[TestFixture]
public class ThreeDimensionalThreatAssessmentTests
{
    private GameObject uavObj;
    private GameObject obstacleObj;
    private UAVPerception perception;
    private PathFollower pathFollower;
    private ThreatAssessment threatAssessment;
    private GridManager gridManager;

    [SetUp]
    public void SetUp()
    {
        uavObj = new GameObject("TestUAV");
        gridManager = uavObj.AddComponent<GridManager>();
        gridManager.gridWorldSize = new Vector2(40f, 40f);
        gridManager.nodeRadius = 0.5f;
        gridManager.CreateGrid();

        perception = uavObj.AddComponent<UAVPerception>();
        pathFollower = uavObj.AddComponent<PathFollower>();
        threatAssessment = uavObj.AddComponent<ThreatAssessment>();

        typeof(UAVPerception).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(perception, null);
        typeof(PathFollower).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(pathFollower, null);
        typeof(ThreatAssessment).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(threatAssessment, null);

        pathFollower.MoveSpeed = 2.0f;
    }

    [TearDown]
    public void TearDown()
    {
        if (obstacleObj != null)
        {
            Object.DestroyImmediate(obstacleObj);
        }
        if (uavObj != null)
        {
            Object.DestroyImmediate(uavObj);
        }
    }

    private DetectedObstacle CreateStaticObstacle(Vector3 center, Vector3 size)
    {
        obstacleObj = GameObject.CreatePrimitive(PrimitiveType.Cube);
        obstacleObj.name = "TestStaticObstacle";
        obstacleObj.transform.position = center;
        obstacleObj.transform.localScale = size;

        BoxCollider col = obstacleObj.GetComponent<BoxCollider>();

        return new DetectedObstacle(
            obstacleObj,
            col,
            center,
            center - uavObj.transform.position,
            Vector3.forward,
            Vector3.Distance(uavObj.transform.position, center),
            0f,
            Vector3.back,
            Vector3.zero,
            isDynamic: false);
    }

    private DetectedObstacle CreateDynamicObstacle(Vector3 center, Vector3 size, Vector3 velocity)
    {
        obstacleObj = GameObject.CreatePrimitive(PrimitiveType.Cube);
        obstacleObj.name = "TestDynamicObstacle";
        obstacleObj.transform.position = center;
        obstacleObj.transform.localScale = size;

        BoxCollider col = obstacleObj.GetComponent<BoxCollider>();

        return new DetectedObstacle(
            obstacleObj,
            col,
            center,
            center - uavObj.transform.position,
            Vector3.forward,
            Vector3.Distance(uavObj.transform.position, center),
            0f,
            Vector3.back,
            velocity,
            isDynamic: true);
    }

    [Test]
    public void PredictCollision_SufficientVerticalOverflight_ReturnsNoCollision()
    {
        // Obstacle at (0, 1.0, 10) with height 2.0m -> top ceiling is Y = 2.0m
        DetectedObstacle obs = CreateStaticObstacle(new Vector3(0f, 1.0f, 10f), new Vector3(2f, 2f, 2f));

        // UAV at Y = 3.5m (1.5m above obstacle top, well above verticalSafetyMargin = 0.5m)
        uavObj.transform.position = new Vector3(0f, 3.5f, 0f);
        pathFollower.SetTargetAltitude(3.5f);

        List<Node> waypoints = new List<Node>
        {
            new Node(true, new Vector3(0f, 3.5f, 20f), 0, 0)
        };

        CollisionPredictionResult result = CollisionPrediction.PredictPathCollision(
            uavObj.transform.position,
            new Vector3(0f, 0f, 2.0f),
            2.0f,
            waypoints,
            new Vector3(0f, 3.5f, 20f),
            obs,
            safetyRadius: 1.0f,
            lookaheadTime: 5.0f,
            verticalSafetyMargin: 0.5f);

        Assert.IsFalse(result.WillCollide, "UAV flying 1.5m above obstacle top must not report collision!");
        Assert.GreaterOrEqual(result.VerticalSeparation, 0.5f, "Vertical separation must be >= 0.5m safety margin!");
    }

    [Test]
    public void PredictCollision_InsufficientVerticalClearance_ReportsCriticalThreat()
    {
        // Obstacle at (0, 1.0, 10) with height 2.0m -> top ceiling is Y = 2.0m
        DetectedObstacle obs = CreateStaticObstacle(new Vector3(0f, 1.0f, 10f), new Vector3(2f, 2f, 2f));

        // UAV at Y = 1.5m (inside vertical envelope of obstacle)
        uavObj.transform.position = new Vector3(0f, 1.5f, 0f);
        pathFollower.SetTargetAltitude(1.5f);

        List<Node> waypoints = new List<Node>
        {
            new Node(true, new Vector3(0f, 1.5f, 20f), 0, 0)
        };

        CollisionPredictionResult result = CollisionPrediction.PredictPathCollision(
            uavObj.transform.position,
            new Vector3(0f, 0f, 2.0f),
            2.0f,
            waypoints,
            new Vector3(0f, 1.5f, 20f),
            obs,
            safetyRadius: 1.0f,
            lookaheadTime: 6.0f,
            verticalSafetyMargin: 0.5f);

        Assert.IsTrue(result.WillCollide, "UAV on direct collision path without vertical clearance must report collision!");
        Assert.Less(result.VerticalSeparation, 0.5f, "Vertical separation must be below safety margin!");
        Assert.AreEqual(0, result.ObstructedWaypointIndex);
    }

    [Test]
    public void PredictCollision_3DClimbingTrajectory_EvaluatesClearanceAtCPATime()
    {
        // Obstacle at Z = 10m, height 2.0m (top is Y = 2.0m)
        DetectedObstacle obs = CreateStaticObstacle(new Vector3(0f, 1.0f, 10f), new Vector3(2f, 2f, 2f));

        // UAV starts at (0, 1.0, 0) and climbs along ramp to (0, 4.0, 20)
        // At Z = 10m (midpoint), UAV altitude will be 2.5m (0.5m clearance above 2.0m top)
        uavObj.transform.position = new Vector3(0f, 1.0f, 0f);

        List<Node> waypoints = new List<Node>
        {
            new Node(true, new Vector3(0f, 4.0f, 20f), 0, 0)
        };

        CollisionPredictionResult result = CollisionPrediction.PredictPathCollision(
            uavObj.transform.position,
            new Vector3(0f, 0.3f, 2.0f),
            2.0f,
            waypoints,
            new Vector3(0f, 4.0f, 20f),
            obs,
            safetyRadius: 1.0f,
            lookaheadTime: 8.0f,
            verticalSafetyMargin: 0.5f);

        Assert.IsFalse(result.WillCollide, "Climbing UAV achieving safe altitude at CPA must clear obstacle!");
        Assert.GreaterOrEqual(result.VerticalSeparation, 0.49f);
    }

    [Test]
    public void DynamicThreat_VerticalSeparation_ClearsVelocityObstacle()
    {
        // Dynamic obstacle crossing at Z = 10m, height 1.5m (top at Y = 1.75m), moving +X at 1.2 m/s
        DetectedObstacle dynObs = CreateDynamicObstacle(
            new Vector3(-2f, 1.0f, 10f),
            new Vector3(1.5f, 1.5f, 1.5f),
            new Vector3(1.2f, 0f, 0f));

        // UAV cruising at Y = 3.5m (1.75m above dynamic threat top)
        uavObj.transform.position = new Vector3(0f, 3.5f, 0f);
        pathFollower.SetTargetAltitude(3.5f);

        List<Node> waypoints = new List<Node>
        {
            new Node(true, new Vector3(0f, 3.5f, 20f), 0, 0)
        };

        CollisionPredictionResult result = CollisionPrediction.PredictPathCollision(
            uavObj.transform.position,
            new Vector3(0f, 0f, 2.0f),
            2.0f,
            waypoints,
            new Vector3(0f, 3.5f, 20f),
            dynObs,
            safetyRadius: 1.0f,
            lookaheadTime: 6.0f,
            verticalSafetyMargin: 0.5f);

        Assert.IsFalse(result.WillCollide, "Dynamic obstacle passing underneath high altitude UAV must not collide!");
        Assert.GreaterOrEqual(result.VerticalSeparation, 0.5f);
    }

    [Test]
    public void PlanarEquivalence_LegacyFlatScenarios_ProduceIdenticalThreatClassification()
    {
        // Legacy flat configuration: UAV at Y = 1.0m, Obstacle at Y = 0.5m with size 2.0m (top at Y = 1.5m)
        DetectedObstacle obs = CreateStaticObstacle(new Vector3(0f, 0.5f, 8f), new Vector3(2f, 2f, 2f));

        uavObj.transform.position = new Vector3(0f, 1.0f, 0f);
        pathFollower.SetTargetAltitude(1.0f);

        List<Node> waypoints = new List<Node>
        {
            new Node(true, new Vector3(0f, 0f, 20f), 0, 0) // Y = 0 from legacy grid generator
        };

        CollisionPredictionResult result = CollisionPrediction.PredictPathCollision(
            uavObj.transform.position,
            new Vector3(0f, 0f, 2.0f),
            2.0f,
            waypoints,
            new Vector3(0f, 1.0f, 20f),
            obs,
            safetyRadius: 1.0f,
            lookaheadTime: 5.0f,
            verticalSafetyMargin: 0.5f);

        Assert.IsTrue(result.WillCollide, "Legacy flat path must detect collision identically!");
        Assert.AreEqual(0, result.ObstructedWaypointIndex);
        Assert.LessOrEqual(result.TimeToCollision, 5.0f);
    }

    [Test]
    public void ThreatAssessment_WarningAndAdvisoryThresholds_RespectVerticalSeparation()
    {
        // Obstacle placed laterally at X = 1.5m, Z = 8m (inside warning radius 2.2m)
        DetectedObstacle obs = CreateStaticObstacle(new Vector3(1.5f, 1.0f, 8f), new Vector3(2f, 2f, 2f));

        // Case A: UAV at Y = 1.0m (no vertical separation)
        uavObj.transform.position = new Vector3(0f, 1.0f, 0f);
        pathFollower.SetTargetAltitude(1.0f);

        CollisionPredictionResult resultA = CollisionPrediction.PredictPathCollision(
            uavObj.transform.position,
            new Vector3(0f, 0f, 2.0f),
            2.0f,
            new List<Node> { new Node(true, new Vector3(0f, 1.0f, 20f), 0, 0) },
            new Vector3(0f, 1.0f, 20f),
            obs,
            safetyRadius: 1.0f,
            lookaheadTime: 5.0f,
            verticalSafetyMargin: 0.5f);

        Assert.IsFalse(resultA.WillCollide);
        Assert.LessOrEqual(resultA.CrossTrackDistance, threatAssessment.WarningRadius);
        Assert.Less(resultA.VerticalSeparation, 0.5f);

        // Case B: UAV at Y = 4.0m (clear vertical overflight)
        uavObj.transform.position = new Vector3(0f, 4.0f, 0f);
        pathFollower.SetTargetAltitude(4.0f);

        CollisionPredictionResult resultB = CollisionPrediction.PredictPathCollision(
            uavObj.transform.position,
            new Vector3(0f, 0f, 2.0f),
            2.0f,
            new List<Node> { new Node(true, new Vector3(0f, 4.0f, 20f), 0, 0) },
            new Vector3(0f, 4.0f, 20f),
            obs,
            safetyRadius: 1.0f,
            lookaheadTime: 5.0f,
            verticalSafetyMargin: 0.5f);

        Assert.IsFalse(resultB.WillCollide);
        Assert.GreaterOrEqual(resultB.VerticalSeparation, 0.5f, "Overflight must report vertical clearance!");
    }
}
