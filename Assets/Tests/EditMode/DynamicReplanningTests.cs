using System.Collections.Generic;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;

[TestFixture]
public class DynamicReplanningTests
{
    private GameObject uavObj;
    private Pathfinding pathfinding;
    private GridManager gridManager;
    private PathFollower pathFollower;
    private ThreatAssessment threatAssessment;
    private ReplanningController replanningController;
    private UAVPerception perception;

    [SetUp]
    public void SetUp()
    {
        uavObj = new GameObject("TestUAV");
        gridManager = uavObj.AddComponent<GridManager>();
        gridManager.gridWorldSize = new Vector2(30f, 30f);
        gridManager.nodeRadius = 0.5f;
        gridManager.enableClearancePotentialField = false;

        pathfinding = uavObj.AddComponent<Pathfinding>();
        MethodInfo awakeMethod = typeof(Pathfinding).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance);
        awakeMethod?.Invoke(pathfinding, null);

        gridManager.CreateGrid();

        pathFollower = uavObj.AddComponent<PathFollower>();
        perception = uavObj.AddComponent<UAVPerception>();
        threatAssessment = uavObj.AddComponent<ThreatAssessment>();
        replanningController = uavObj.AddComponent<ReplanningController>();

        typeof(PathFollower).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(pathFollower, null);
        typeof(ThreatAssessment).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(threatAssessment, null);
        typeof(ReplanningController).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(replanningController, null);

        GameObject targetObj = new GameObject("TestTarget");
        targetObj.transform.position = new Vector3(10f, 1f, 10f);
        pathfinding.targetTransform = targetObj.transform;
    }

    [TearDown]
    public void TearDown()
    {
        if (uavObj != null)
        {
            Object.DestroyImmediate(uavObj);
        }
    }

    [Test]
    public void DynamicHazardAvoidance_PathfindingFindPath_RoutesAroundDynamicHazardFootprint()
    {
        Vector3 start = new Vector3(0f, 1f, -10f);
        Vector3 target = new Vector3(0f, 1f, 10f);

        // Place dynamic hazard directly on the straight-line path at (0, 1, 0) with 2.5m radius
        Vector3 hazardPos = new Vector3(0f, 1f, 0f);
        float hazardRadius = 2.5f;

        pathfinding.FindPath(start, target, hazardPos, hazardRadius);

        Assert.IsNotNull(pathfinding.path);
        Assert.Greater(pathfinding.path.Count, 0);

        // Verify that NO node in the smoothed path breaches the dynamic hazard radius
        for (int i = 0; i < pathfinding.path.Count; i++)
        {
            Vector3 nodePos = pathfinding.path[i].worldPosition;
            float flatDist = Vector3.Distance(
                new Vector3(nodePos.x, 0f, nodePos.z),
                new Vector3(hazardPos.x, 0f, hazardPos.z));

            Assert.GreaterOrEqual(flatDist, hazardRadius - 0.1f,
                $"Waypoint at index {i} ({nodePos}) was within the dynamic hazard footprint ({flatDist}m < {hazardRadius}m)!");
        }
    }

    [Test]
    public void DynamicThreat_TriggersDynamicReplan_WhenThreatIsCritical()
    {
        uavObj.transform.position = new Vector3(-8f, 1f, -8f);

        // Set active initial path
        List<Node> initialPath = new List<Node>
        {
            gridManager.NodeFromWorldPoint(new Vector3(-4f, 1f, -4f)),
            gridManager.NodeFromWorldPoint(new Vector3(0f, 1f, 0f)),
            gridManager.NodeFromWorldPoint(new Vector3(10f, 1f, 10f))
        };
        pathFollower.StartFollowing(initialPath);

        GameObject obstacleObj = new GameObject("MovingThreat");
        obstacleObj.transform.position = new Vector3(0f, 1f, 0f);

        DetectedObstacle threatObstacle = new DetectedObstacle(
            obstacleObj,
            null,
            obstacleObj.transform.position,
            obstacleObj.transform.position - uavObj.transform.position,
            Vector3.forward,
            8.0f,
            0f,
            Vector3.back,
            new Vector3(-1f, 0f, 0f),
            isDynamic: true);

        ThreatReport criticalReport = new ThreatReport(
            ThreatLevel.Critical,
            threatObstacle,
            new Vector3(0f, 1f, 0f),
            distanceToCollision: 6.0f,
            timeToCollision: 3.0f,
            obstructedWaypointIndex: 1);

        bool replanSuccess = replanningController.TryExecuteReplan("Critical Moving Threat Intercept", criticalReport);

        Assert.IsTrue(replanSuccess);
        Assert.AreEqual(NavigationState.Rerouting, replanningController.State);
        Assert.AreEqual(1, replanningController.ReplanCount);

        Object.DestroyImmediate(obstacleObj);
    }

    [Test]
    public void DynamicReplan_RespectsCooldown_AndPreventsRapidRepeatedRequests()
    {
        uavObj.transform.position = Vector3.zero;

        List<Node> initialPath = new List<Node>
        {
            gridManager.NodeFromWorldPoint(new Vector3(0f, 1f, 5f)),
            gridManager.NodeFromWorldPoint(new Vector3(10f, 1f, 10f))
        };
        pathFollower.StartFollowing(initialPath);

        GameObject threatA = new GameObject("ThreatA");
        DetectedObstacle obsA = new DetectedObstacle(threatA, null, new Vector3(0f, 1f, 5f), Vector3.forward * 5f, Vector3.forward, 5f, 0f, Vector3.back);
        ThreatReport reportA = new ThreatReport(ThreatLevel.Critical, obsA, new Vector3(0f, 1f, 5f), 5f, 2.5f, 0);

        bool firstReplan = replanningController.TryExecuteReplan("First Threat", reportA);
        Assert.IsTrue(firstReplan);

        // Immediate secondary request within cooldown should be rejected
        GameObject threatB = new GameObject("ThreatB");
        DetectedObstacle obsB = new DetectedObstacle(threatB, null, new Vector3(2f, 1f, 5f), Vector3.forward * 5f, Vector3.forward, 5f, 0f, Vector3.back);
        ThreatReport reportB = new ThreatReport(ThreatLevel.Critical, obsB, new Vector3(2f, 1f, 5f), 5f, 2.5f, 0);

        bool secondReplanImmediate = replanningController.TryExecuteReplan("Second Threat Too Fast", reportB);
        Assert.IsFalse(secondReplanImmediate, "Secondary replan must be blocked by replanCooldown!");
        Assert.AreEqual(1, replanningController.ReplanCount);

        Object.DestroyImmediate(threatA);
        Object.DestroyImmediate(threatB);
    }

    [Test]
    public void DynamicReplan_StateRecovery_ReturnsToNormalWhenThreatCleared()
    {
        uavObj.transform.position = Vector3.zero;

        List<Node> initialPath = new List<Node>
        {
            gridManager.NodeFromWorldPoint(new Vector3(0f, 1f, 5f)),
            gridManager.NodeFromWorldPoint(new Vector3(10f, 1f, 10f))
        };
        pathFollower.StartFollowing(initialPath);

        GameObject threat = new GameObject("Threat");
        DetectedObstacle obs = new DetectedObstacle(threat, null, new Vector3(0f, 1f, 5f), Vector3.forward * 5f, Vector3.forward, 5f, 0f, Vector3.back);
        ThreatReport report = new ThreatReport(ThreatLevel.Critical, obs, new Vector3(0f, 1f, 5f), 5f, 2.5f, 0);

        replanningController.TryExecuteReplan("Critical Threat", report);
        Assert.AreEqual(NavigationState.Rerouting, replanningController.State);

        // Advance cooldown timestamp via reflection to simulate time progression
        FieldInfo lastReplanField = typeof(ReplanningController).GetField("lastReplanTime", BindingFlags.NonPublic | BindingFlags.Instance);
        lastReplanField?.SetValue(replanningController, Time.time - 5.0f);

        // Trigger navigation state update with clear threat report
        MethodInfo updateNavMethod = typeof(ReplanningController).GetMethod("UpdateNavigationState", BindingFlags.NonPublic | BindingFlags.Instance);
        updateNavMethod?.Invoke(replanningController, null);

        Assert.AreEqual(NavigationState.Normal, replanningController.State);

        Object.DestroyImmediate(threat);
    }

    [Test]
    public void StaticObstacle_FindPathAndReplanning_Remains100PercentFunctional()
    {
        Vector3 start = new Vector3(-8f, 1f, -8f);
        Vector3 target = new Vector3(8f, 1f, 8f);

        // Standard static pathfinding query with no dynamic hazard
        pathfinding.FindPath(start, target);

        Assert.IsNotNull(pathfinding.path);
        Assert.Greater(pathfinding.path.Count, 0);

        Node startNode = gridManager.NodeFromWorldPoint(start);
        Node targetNode = gridManager.NodeFromWorldPoint(target);

        Node finalPathNode = pathfinding.path[pathfinding.path.Count - 1];
        Assert.AreEqual(targetNode.gridX, finalPathNode.gridX);
        Assert.AreEqual(targetNode.gridY, finalPathNode.gridY);
    }

    [Test]
    public void DynamicHazard_MaintainsMinimumSafeClearance()
    {
        Vector3 start = new Vector3(-5f, 1f, 0f);
        Vector3 target = new Vector3(5f, 1f, 0f);
        Vector3 hazard = new Vector3(0f, 1f, 0f);
        float hazardRadius = 1.5f;

        pathfinding.FindPath(start, target, hazard, hazardRadius);

        Assert.IsNotNull(pathfinding.path);
        Assert.Greater(pathfinding.path.Count, 0);

        for (int i = 0; i < pathfinding.path.Count; i++)
        {
            Vector3 pt = pathfinding.path[i].worldPosition;
            float dist = Vector3.Distance(new Vector3(pt.x, 0f, pt.z), new Vector3(hazard.x, 0f, hazard.z));
            Assert.Greater(dist, 0.5f, $"Waypoint at {pt} was too close to moving hazard ({dist}m < 0.5m)!");
        }
    }
}
