using System.Collections.Generic;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;

[TestFixture]
public class PredictiveHazardProjectionTests
{
    private GameObject uavObj;
    private GridManager gridManager;
    private Pathfinding pathfinding;
    private PathFollower pathFollower;
    private ThreatAssessment threatAssessment;
    private ReplanningController replanningController;

    [SetUp]
    public void SetUp()
    {
        uavObj = new GameObject("TestUAV_PredictiveHazard");
        uavObj.transform.position = new Vector3(0f, 1f, 0f);

        gridManager = uavObj.AddComponent<GridManager>();
        gridManager.gridWorldSize = new Vector2(50f, 50f);
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

        pathFollower.MoveSpeed = 2.0f;

        GameObject targetObj = new GameObject("TestTarget");
        targetObj.transform.position = new Vector3(0f, 1f, 20f);
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
    public void PredictiveHazard_StaticObstacle_MaintainsPointDistanceFootprint()
    {
        Vector3 hazardPos = new Vector3(0f, 1f, 10f);
        DynamicHazard staticHazard = new DynamicHazard(hazardPos, 2.0f, Vector3.zero, isDynamic: false, projectedHorizonTime: 0f);

        Vector3 testPoint = new Vector3(3f, 1f, 10f);
        float dist = staticHazard.DistanceToHazard2D(testPoint);

        Assert.AreEqual(3.0f, dist, 0.01f, "Static hazard distance must equal 2D point Euclidean distance!");
    }

    [Test]
    public void PredictiveHazard_MovingObstacle_ExpandsDistanceAlongVelocityVector()
    {
        // Moving +X at 2 m/s for 2 seconds (sweeps from (0,1,10) to (4,1,10))
        Vector3 hazardPos = new Vector3(0f, 1f, 10f);
        Vector3 velocity = new Vector3(2f, 0f, 0f);
        DynamicHazard movingHazard = new DynamicHazard(hazardPos, 1.5f, velocity, isDynamic: true, projectedHorizonTime: 2.0f);

        // Test point ahead of the hazard along its forward motion corridor at (3, 1, 10)
        Vector3 pointAlongCorridor = new Vector3(3f, 1f, 10f);
        float distToCorridor = movingHazard.DistanceToHazard2D(pointAlongCorridor);

        // Point is directly on the forward motion segment [0, 4] -> distance should be ~0m
        Assert.AreEqual(0.0f, distToCorridor, 0.01f, "Point directly on moving threat's forward path must have ~0 distance!");

        // Test point laterally offset from the middle of the corridor at (2, 1, 12)
        Vector3 lateralPoint = new Vector3(2f, 1f, 12f);
        float lateralDist = movingHazard.DistanceToHazard2D(lateralPoint);
        Assert.AreEqual(2.0f, lateralDist, 0.01f, "Lateral distance to motion corridor must measure orthogonal segment distance!");
    }

    [Test]
    public void PredictiveHazard_ZeroVelocity_CollapsesToPointDistance()
    {
        Vector3 hazardPos = new Vector3(0f, 1f, 10f);
        DynamicHazard hazard = new DynamicHazard(hazardPos, 2.0f, Vector3.zero, isDynamic: true, projectedHorizonTime: 3.0f);

        Vector3 testPoint = new Vector3(0f, 1f, 15f);
        float dist = hazard.DistanceToHazard2D(testPoint);

        Assert.AreEqual(5.0f, dist, 0.01f);
    }

    [Test]
    public void PredictiveHazard_ZeroProjectedHorizon_CollapsesToPointDistance()
    {
        Vector3 hazardPos = new Vector3(0f, 1f, 10f);
        Vector3 velocity = new Vector3(2f, 0f, 0f);
        DynamicHazard hazard = new DynamicHazard(hazardPos, 2.0f, velocity, isDynamic: true, projectedHorizonTime: 0f);

        Vector3 testPoint = new Vector3(3f, 1f, 10f);
        float dist = hazard.DistanceToHazard2D(testPoint);

        // Horizon is 0s, so distance to (0, 10) from (3, 10) is 3m
        Assert.AreEqual(3.0f, dist, 0.01f);
    }

    [Test]
    public void PredictiveAStar_AvoidsProjectedForwardCorridorOfMovingThreat()
    {
        Vector3 start = new Vector3(0f, 1f, 0f);
        Vector3 target = new Vector3(0f, 1f, 20f);

        // Obstacle starts at (-3, 1, 10) moving +X at 2 m/s across the flight path to (+3, 1, 10) with 2.5s horizon
        List<DynamicHazard> hazards = new List<DynamicHazard>
        {
            new DynamicHazard(new Vector3(-3f, 1f, 10f), 1.8f, new Vector3(2f, 0f, 0f), isDynamic: true, projectedHorizonTime: 2.5f)
        };

        pathfinding.FindPath(start, target, hazards);

        Assert.IsNotNull(pathfinding.path);
        Assert.Greater(pathfinding.path.Count, 0, "A valid detour must be found!");

        // Verify that no intermediate path node penetrates the projected corridor
        for (int i = 1; i < pathfinding.path.Count - 1; i++)
        {
            float dist = hazards[0].DistanceToHazard2D(pathfinding.path[i].worldPosition);
            Assert.GreaterOrEqual(dist, hazards[0].Radius - 0.2f,
                $"Path node {i} at {pathfinding.path[i].worldPosition} breached the projected threat corridor!");
        }
    }

    [Test]
    public void PredictiveSmoothing_DoesNotCutAcrossProjectedTrajectory()
    {
        Vector3 p1 = new Vector3(0f, 1f, 0f);
        Vector3 p2 = new Vector3(0f, 1f, 20f);

        // Obstacle starts at (-2, 1, 10) moving +X at 2 m/s crossing center (X=0)
        List<DynamicHazard> hazards = new List<DynamicHazard>
        {
            new DynamicHazard(new Vector3(-2f, 1f, 10f), 1.8f, new Vector3(2f, 0f, 0f), isDynamic: true, projectedHorizonTime: 2.0f)
        };

        bool isClear = pathfinding.IsCorridorClear(p1, p2, hazards);
        Assert.IsFalse(isClear, "Corridor crossing the projected forward path of a moving threat must be rejected by path smoothing!");
    }

    [Test]
    public void MultiThreatTelemetry_SpeedPacingAndSpatialReplans_AreTrackedCorrectly()
    {
        List<Node> path = new List<Node> { gridManager.NodeFromWorldPoint(new Vector3(0f, 1f, 20f)) };
        pathFollower.StartFollowing(path);

        Assert.AreEqual(0, replanningController.SpeedPacingCount);
        Assert.AreEqual(0, replanningController.SpatialReplanCount);

        // 1. Trigger VO speed pacing with crossing threat
        GameObject crossingObs = new GameObject("CrossingObs");
        crossingObs.transform.position = new Vector3(-2f, 1f, 10f);
        DetectedObstacle detCross = new DetectedObstacle(crossingObs, null, crossingObs.transform.position, crossingObs.transform.position, Vector3.forward, 10.2f, 0f, Vector3.back, new Vector3(1.5f, 0f, 0f), true);
        ThreatReport repCross = new ThreatReport(ThreatLevel.Critical, detCross, new Vector3(0f, 1f, 10f), 10f, 5f, 0);

        replanningController.TryExecuteReplan("Pacing Replan", repCross);
        Assert.AreEqual(1, replanningController.SpeedPacingCount);
        Assert.AreEqual(0, replanningController.SpatialReplanCount);

        // Advance cooldown
        FieldInfo lastReplanTimeField = typeof(ReplanningController).GetField("lastReplanTime", BindingFlags.NonPublic | BindingFlags.Instance);
        lastReplanTimeField?.SetValue(replanningController, Time.time - 5.0f);

        // 2. Trigger spatial replan with head-on threat with tall collider exceeding ceiling
        GameObject headOnObs = new GameObject("HeadOnObs");
        headOnObs.transform.position = new Vector3(0f, 1f, 10f);
        BoxCollider col = headOnObs.AddComponent<BoxCollider>();
        col.size = new Vector3(2f, 12f, 2f);
        col.center = new Vector3(0f, 5f, 0f);
        DetectedObstacle detHeadOn = new DetectedObstacle(headOnObs, col, headOnObs.transform.position, headOnObs.transform.position, Vector3.forward, 10f, 0f, Vector3.back, new Vector3(0f, 0f, -2f), true);
        ThreatReport repHeadOn = new ThreatReport(ThreatLevel.Critical, detHeadOn, new Vector3(0f, 1f, 5f), 5f, 2.5f, 0);

        replanningController.TryExecuteReplan("Spatial Replan", repHeadOn);
        Assert.AreEqual(1, replanningController.SpeedPacingCount);
        Assert.AreEqual(1, replanningController.SpatialReplanCount);

        Object.DestroyImmediate(crossingObs);
        Object.DestroyImmediate(headOnObs);
    }

    [Test]
    public void MultiThreatTelemetry_PeakSimultaneousThreats_IsUpdatedCorrectly()
    {
        List<Node> path = new List<Node> { gridManager.NodeFromWorldPoint(new Vector3(0f, 1f, 20f)) };
        pathFollower.StartFollowing(path);

        List<ThreatReport> tripleThreats = new List<ThreatReport>
        {
            new ThreatReport(ThreatLevel.Critical, default(DetectedObstacle), new Vector3(0f, 1f, 6f), 6f, 3f, 0),
            new ThreatReport(ThreatLevel.Warning, default(DetectedObstacle), new Vector3(2f, 1f, 10f), 10f, 5f, 0),
            new ThreatReport(ThreatLevel.Warning, default(DetectedObstacle), new Vector3(-2f, 1f, 14f), 14f, 7f, 0)
        };

        FieldInfo activeThreatsField = typeof(ThreatAssessment).GetField("activeThreatReports", BindingFlags.NonPublic | BindingFlags.Instance);
        activeThreatsField?.SetValue(threatAssessment, tripleThreats);

        GameObject obs = new GameObject("TestObs");
        DetectedObstacle det = new DetectedObstacle(obs, null, new Vector3(0f, 1f, 6f), Vector3.forward * 6f, Vector3.forward, 6f, 0f, Vector3.back, Vector3.zero, false);
        ThreatReport rep = new ThreatReport(ThreatLevel.Critical, det, new Vector3(0f, 1f, 6f), 6f, 3f, 0);

        replanningController.TryExecuteReplan("Triple Threat Replan", rep);

        Assert.AreEqual(3, replanningController.PeakSimultaneousThreats);

        Object.DestroyImmediate(obs);
    }
}
