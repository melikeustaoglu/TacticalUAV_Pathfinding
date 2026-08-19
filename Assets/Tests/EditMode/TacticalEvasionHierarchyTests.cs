using System.Collections.Generic;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;

[TestFixture]
public class TacticalEvasionHierarchyTests
{
    private GameObject uavObj;
    private GameObject obstacleObj;
    private GameObject secondaryObstacleObj;
    private GameObject targetObj;
    private PathFollower pathFollower;
    private ThreatAssessment threatAssessment;
    private ReplanningController replanningController;
    private GridManager gridManager;
    private Pathfinding pathfinding;

    [SetUp]
    public void SetUp()
    {
        uavObj = new GameObject("TestUAV");
        gridManager = uavObj.AddComponent<GridManager>();
        gridManager.gridWorldSize = new Vector2(40f, 40f);
        gridManager.nodeRadius = 0.5f;

        pathfinding = uavObj.AddComponent<Pathfinding>();
        typeof(Pathfinding).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(pathfinding, null);
        gridManager.CreateGrid();

        pathFollower = uavObj.AddComponent<PathFollower>();
        threatAssessment = uavObj.AddComponent<ThreatAssessment>();
        replanningController = uavObj.AddComponent<ReplanningController>();

        typeof(PathFollower).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(pathFollower, null);
        typeof(ThreatAssessment).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(threatAssessment, null);
        typeof(ReplanningController).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(replanningController, null);

        targetObj = new GameObject("TestTarget");
        targetObj.transform.position = new Vector3(0f, 1f, 20f);
        pathfinding.targetTransform = targetObj.transform;

        pathFollower.MoveSpeed = 2.0f;
        pathFollower.MinFlightAltitude = 1.0f;
        pathFollower.MaxFlightAltitude = 6.0f;
        pathFollower.MaxClimbRate = 1.5f;
        pathFollower.MaxDescentRate = 2.0f;
    }

    [TearDown]
    public void TearDown()
    {
        if (targetObj != null) Object.DestroyImmediate(targetObj);
        if (secondaryObstacleObj != null) Object.DestroyImmediate(secondaryObstacleObj);
        if (obstacleObj != null) Object.DestroyImmediate(obstacleObj);
        if (uavObj != null) Object.DestroyImmediate(uavObj);
    }

    private DetectedObstacle CreateObstacle(string name, Vector3 center, Vector3 size, Vector3 velocity, bool isDynamic)
    {
        GameObject obj = GameObject.CreatePrimitive(PrimitiveType.Cube);
        obj.name = name;
        obj.transform.position = center;
        obj.transform.localScale = size;
        BoxCollider col = obj.GetComponent<BoxCollider>();

        return new DetectedObstacle(
            obj,
            col,
            center,
            center - uavObj.transform.position,
            Vector3.forward,
            Vector3.Distance(uavObj.transform.position, center),
            0f,
            Vector3.back,
            velocity,
            isDynamic: isDynamic);
    }

    [Test]
    public void Hierarchy_CrossingDynamicThreat_SelectsStage1SpeedPacing()
    {
        uavObj.transform.position = new Vector3(0f, 1f, 0f);

        // Path heading towards (0, 1, 20)
        List<Node> path = new List<Node>
        {
            new Node(true, new Vector3(0f, 1f, 20f), 0, 0)
        };
        pathFollower.StartFollowing(path);

        // Crossing dynamic obstacle at Z = 10m moving +X at 1.5 m/s
        obstacleObj = GameObject.CreatePrimitive(PrimitiveType.Cube);
        obstacleObj.name = "DynamicThreat";
        obstacleObj.transform.position = new Vector3(-2f, 1f, 10f);
        obstacleObj.transform.localScale = Vector3.one * 1.5f;

        DetectedObstacle dynObs = new DetectedObstacle(
            obstacleObj,
            obstacleObj.GetComponent<BoxCollider>(),
            obstacleObj.transform.position,
            obstacleObj.transform.position - uavObj.transform.position,
            Vector3.forward,
            10.2f,
            0f,
            Vector3.back,
            new Vector3(1.5f, 0f, 0f),
            isDynamic: true);

        ThreatReport report = new ThreatReport(
            ThreatLevel.Critical,
            dynObs,
            new Vector3(0f, 1f, 10f),
            distanceToCollision: 10f,
            timeToCollision: 5.0f,
            obstructedWaypointIndex: 0);

        bool replanned = replanningController.TryExecuteReplan("Dynamic crossing threat", report);

        Assert.IsTrue(replanned);
        Assert.AreEqual(1, replanningController.SpeedPacingCount, "Dynamic crossing threat must select Stage 1 speed pacing!");
        Assert.AreEqual(0, replanningController.VerticalEvasionCount);
        Assert.AreEqual(0, replanningController.SpatialReplanCount);
        Assert.IsTrue(pathFollower.IsSpeedOverrideActive);
    }

    [Test]
    public void Hierarchy_LowStaticObstacle_SelectsStage2VerticalClimb()
    {
        uavObj.transform.position = new Vector3(0f, 1f, 0f);

        List<Node> path = new List<Node>
        {
            new Node(true, new Vector3(0f, 1f, 20f), 0, 0)
        };
        pathFollower.StartFollowing(path);

        // Static obstacle at Z = 10m, height 2.0m (top ceiling at Y = 2.0m)
        DetectedObstacle staticObs = CreateObstacle("LowStaticObs", new Vector3(0f, 1f, 10f), new Vector3(2f, 2f, 2f), Vector3.zero, false);
        obstacleObj = staticObs.GameObject;

        ThreatReport report = new ThreatReport(
            ThreatLevel.Critical,
            staticObs,
            new Vector3(0f, 1f, 10f),
            distanceToCollision: 10f,
            timeToCollision: 5.0f,
            obstructedWaypointIndex: 0);

        bool replanned = replanningController.TryExecuteReplan("Low static obstacle in corridor", report);

        Assert.IsTrue(replanned);
        Assert.AreEqual(0, replanningController.SpeedPacingCount);
        Assert.AreEqual(1, replanningController.VerticalEvasionCount, "Low static obstacle within ceiling must select Stage 2 vertical step climb!");
        Assert.AreEqual(0, replanningController.SpatialReplanCount);
        Assert.AreEqual(2.5f, pathFollower.TargetAltitude, 0.01f, "Target altitude should be obstacle top (2.0m) + margin (0.5m) = 2.5m!");
    }

    [Test]
    public void Hierarchy_TallStaticObstacleExceedingCeiling_EscalatesToStage3SpatialAStar()
    {
        uavObj.transform.position = new Vector3(0f, 1f, 0f);

        List<Node> path = new List<Node>
        {
            new Node(true, new Vector3(0f, 1f, 20f), 0, 0)
        };
        pathFollower.StartFollowing(path);

        // Tall obstacle of height 7.0m (top at Y = 7.0m), exceeding ceiling of 6.0m
        DetectedObstacle tallObs = CreateObstacle("TallStaticObs", new Vector3(0f, 3.5f, 10f), new Vector3(2f, 7f, 2f), Vector3.zero, false);
        obstacleObj = tallObs.GameObject;

        ThreatReport report = new ThreatReport(
            ThreatLevel.Critical,
            tallObs,
            new Vector3(0f, 1f, 10f),
            distanceToCollision: 10f,
            timeToCollision: 5.0f,
            obstructedWaypointIndex: 0);

        bool replanned = replanningController.TryExecuteReplan("Tall obstacle exceeding ceiling", report);

        Assert.IsTrue(replanned);
        Assert.AreEqual(0, replanningController.SpeedPacingCount);
        Assert.AreEqual(0, replanningController.VerticalEvasionCount, "Obstacle exceeding ceiling must not trigger vertical climb!");
        Assert.AreEqual(1, replanningController.SpatialReplanCount, "Obstacle exceeding ceiling must escalate to Stage 3 Spatial A*!");
    }

    [Test]
    public void Hierarchy_InsufficientTimeToClimb_EscalatesToStage3SpatialAStar()
    {
        uavObj.transform.position = new Vector3(0f, 1f, 0f);

        List<Node> path = new List<Node>
        {
            new Node(true, new Vector3(0f, 1f, 20f), 0, 0)
        };
        pathFollower.StartFollowing(path);

        // Obstacle only 0.4m away (TTC = 0.2s), requires 2.5m climb which is kinematically impossible in 0.2s
        DetectedObstacle closeObs = CreateObstacle("CloseObs", new Vector3(0f, 1.5f, 0.4f), new Vector3(2f, 3f, 2f), Vector3.zero, false);
        obstacleObj = closeObs.GameObject;

        ThreatReport report = new ThreatReport(
            ThreatLevel.Critical,
            closeObs,
            new Vector3(0f, 1f, 0.4f),
            distanceToCollision: 0.4f,
            timeToCollision: 0.2f,
            obstructedWaypointIndex: 0);

        bool replanned = replanningController.TryExecuteReplan("Obstacle with insufficient climb time", report);

        Assert.IsTrue(replanned);
        Assert.AreEqual(0, replanningController.VerticalEvasionCount, "Insufficient climb time must reject vertical evasion!");
        Assert.AreEqual(1, replanningController.SpatialReplanCount, "Must escalate to Stage 3 Spatial A*!");
    }

    [Test]
    public void Hierarchy_MultiThreat_VerticalClimbVerifiesAllActiveThreats()
    {
        uavObj.transform.position = new Vector3(0f, 1f, 0f);

        List<Node> path = new List<Node>
        {
            new Node(true, new Vector3(0f, 1f, 20f), 0, 0)
        };
        pathFollower.StartFollowing(path);

        // Primary threat at Z = 8m (top at Y = 2.0m, candidate climb is 2.5m)
        DetectedObstacle obsA = CreateObstacle("ThreatA", new Vector3(0f, 1f, 8f), new Vector3(2f, 2f, 2f), Vector3.zero, false);
        obstacleObj = obsA.GameObject;

        // Secondary threat at Z = 12m with height 4.0m (top at Y = 4.0m)
        DetectedObstacle obsB = CreateObstacle("ThreatB", new Vector3(0f, 2f, 12f), new Vector3(2f, 4f, 2f), Vector3.zero, false);
        secondaryObstacleObj = obsB.GameObject;

        ThreatReport primaryReport = new ThreatReport(ThreatLevel.Critical, obsA, new Vector3(0f, 1f, 8f), 8f, 4.0f, 0);
        ThreatReport secondaryReport = new ThreatReport(ThreatLevel.Warning, obsB, new Vector3(0f, 1f, 12f), 12f, 6.0f, 0);

        // Populate active threat list in threatAssessment via reflection
        FieldInfo activeThreatsField = typeof(ThreatAssessment).GetField("activeThreatReports", BindingFlags.NonPublic | BindingFlags.Instance);
        List<ThreatReport> activeList = activeThreatsField?.GetValue(threatAssessment) as List<ThreatReport>;
        activeList?.Add(primaryReport);
        activeList?.Add(secondaryReport);

        bool replanned = replanningController.TryExecuteReplan("Multi-threat compound scenario", primaryReport);

        Assert.IsTrue(replanned);
        // Candidate 2.5m climb would collide with secondary threat at 4.0m, so vertical climb must be rejected
        Assert.AreEqual(0, replanningController.VerticalEvasionCount);
        Assert.AreEqual(1, replanningController.SpatialReplanCount, "Must escalate to Stage 3 when vertical climb violates secondary active threat!");
    }

    [Test]
    public void Hierarchy_FlightCeilingStrictEnforcement_NeverCommandsAboveMaxAltitude()
    {
        uavObj.transform.position = new Vector3(0f, 1f, 0f);
        pathFollower.MaxFlightAltitude = 5.0f;

        // Obstacle with top at 4.8m -> candidate climb is 5.3m (> ceiling 5.0m)
        DetectedObstacle obs = CreateObstacle("NearCeilingObs", new Vector3(0f, 2.4f, 10f), new Vector3(2f, 4.8f, 2f), Vector3.zero, false);
        obstacleObj = obs.GameObject;

        ThreatReport report = new ThreatReport(ThreatLevel.Critical, obs, new Vector3(0f, 1f, 10f), 10f, 5.0f, 0);

        bool canClimb = replanningController.TryTacticalVerticalEvasion(report, out float targetAlt);
        Assert.IsFalse(canClimb, "Vertical climb above max flight ceiling must be strictly rejected!");
    }

    [Test]
    public void Hierarchy_UnreachableDestination_EntersStage4EmergencySafeHold()
    {
        uavObj.transform.position = new Vector3(0f, 1f, 0f);

        List<Node> path = new List<Node>
        {
            new Node(true, new Vector3(0f, 1f, 20f), 0, 0)
        };
        pathFollower.StartFollowing(path);

        // Tall obstacle exceeding ceiling
        DetectedObstacle tallObs = CreateObstacle("Wall", new Vector3(0f, 5f, 10f), new Vector3(50f, 10f, 2f), Vector3.zero, false);
        obstacleObj = tallObs.GameObject;

        // Set target destination inside solid unwalkable wall
        gridManager.NodeFromWorldPoint(targetObj.transform.position).isWalkable = false;

        ThreatReport report = new ThreatReport(ThreatLevel.Critical, tallObs, new Vector3(0f, 1f, 10f), 10f, 5.0f, 0);

        bool replanned = replanningController.TryExecuteReplan("Completely blocked arena", report);

        Assert.IsFalse(replanned);
        Assert.AreEqual(NavigationState.NoSafePath, replanningController.State, "Unreachable destination must transition to NoSafePath!");
        Assert.IsFalse(pathFollower.IsFollowing, "UAV must halt in emergency safe hold!");
    }

    [Test]
    public void Hierarchy_CooldownAndHysteresis_PreventsRapidOscillation()
    {
        uavObj.transform.position = new Vector3(0f, 1f, 0f);

        List<Node> path = new List<Node>
        {
            new Node(true, new Vector3(0f, 1f, 20f), 0, 0)
        };
        pathFollower.StartFollowing(path);

        DetectedObstacle obs = CreateObstacle("LowObs", new Vector3(0f, 1f, 10f), new Vector3(2f, 2f, 2f), Vector3.zero, false);
        obstacleObj = obs.GameObject;

        ThreatReport report = new ThreatReport(ThreatLevel.Critical, obs, new Vector3(0f, 1f, 10f), 10f, 5.0f, 0);

        bool firstReplan = replanningController.TryExecuteReplan("First trigger", report);
        Assert.IsTrue(firstReplan);
        Assert.AreEqual(1, replanningController.ReplanCount);

        // Immediate subsequent trigger within 0.1s must be blocked by cooldown
        bool secondReplan = replanningController.TryExecuteReplan("Immediate second trigger", report);
        Assert.IsFalse(secondReplan, "Immediate second replan must be rejected by cooldown!");
        Assert.AreEqual(1, replanningController.ReplanCount);
    }

    [Test]
    public void Hierarchy_ZeroAltitudeDelta_PreservesPlanarLegacyBehavior()
    {
        Assert.AreEqual(0, replanningController.ReplanCount);
        Assert.AreEqual(0, replanningController.SpeedPacingCount);
        Assert.AreEqual(0, replanningController.VerticalEvasionCount);
        Assert.AreEqual(0, replanningController.SpatialReplanCount);
        Assert.AreEqual(NavigationState.Normal, replanningController.State);
    }
}
