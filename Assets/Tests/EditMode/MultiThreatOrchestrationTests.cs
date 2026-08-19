using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;

[TestFixture]
public class MultiThreatOrchestrationTests
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
        uavObj = new GameObject("TestUAV_Orchestration");
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
    public void MultiThreat_ReplanTracksAllActiveThreats()
    {
        List<Node> path = new List<Node> { gridManager.NodeFromWorldPoint(new Vector3(0f, 1f, 20f)) };
        pathFollower.StartFollowing(path);

        GameObject obsA = new GameObject("ObsA");
        GameObject obsB = new GameObject("ObsB");
        obsA.transform.position = new Vector3(0f, 1f, 8f);
        obsB.transform.position = new Vector3(2f, 1f, 12f);

        DetectedObstacle detA = new DetectedObstacle(obsA, null, obsA.transform.position, obsA.transform.position, Vector3.forward, 8f, 0f, Vector3.back, new Vector3(0f, 0f, -2f), true);
        DetectedObstacle detB = new DetectedObstacle(obsB, null, obsB.transform.position, obsB.transform.position, Vector3.forward, 12.1f, 0f, Vector3.back, new Vector3(0f, 0f, -1f), true);

        ThreatReport repA = new ThreatReport(ThreatLevel.Critical, detA, new Vector3(0f, 1f, 4f), 4f, 2f, 0);
        ThreatReport repB = new ThreatReport(ThreatLevel.Warning, detB, new Vector3(2f, 1f, 8f), 8f, 4f, 0);

        FieldInfo activeThreatsField = typeof(ThreatAssessment).GetField("activeThreatReports", BindingFlags.NonPublic | BindingFlags.Instance);
        activeThreatsField?.SetValue(threatAssessment, new List<ThreatReport> { repA, repB });

        replanningController.TryExecuteReplan("Multi-Threat Trigger", repA);

        Assert.AreEqual(NavigationState.Rerouting, replanningController.State);
        Assert.IsTrue(replanningController.CurrentlyAvoidingObstacles.Contains(obsA));
        Assert.IsTrue(replanningController.CurrentlyAvoidingObstacles.Contains(obsB));

        Object.DestroyImmediate(obsA);
        Object.DestroyImmediate(obsB);
    }

    [Test]
    public void ThreatA_ClearsThreatBRemains_AvoidanceContinues()
    {
        List<Node> path = new List<Node> { gridManager.NodeFromWorldPoint(new Vector3(0f, 1f, 20f)) };
        pathFollower.StartFollowing(path);

        GameObject obsA = new GameObject("ObsA");
        GameObject obsB = new GameObject("ObsB");
        obsA.transform.position = new Vector3(0f, 1f, 8f);
        obsB.transform.position = new Vector3(2f, 1f, 12f);

        DetectedObstacle detA = new DetectedObstacle(obsA, null, obsA.transform.position, obsA.transform.position, Vector3.forward, 8f, 0f, Vector3.back, new Vector3(0f, 0f, -2f), true);
        DetectedObstacle detB = new DetectedObstacle(obsB, null, obsB.transform.position, obsB.transform.position, Vector3.forward, 12.1f, 0f, Vector3.back, new Vector3(0f, 0f, -1f), true);

        ThreatReport repA = new ThreatReport(ThreatLevel.Critical, detA, new Vector3(0f, 1f, 4f), 4f, 2f, 0);
        ThreatReport repB = new ThreatReport(ThreatLevel.Warning, detB, new Vector3(2f, 1f, 8f), 8f, 4f, 0);

        FieldInfo activeThreatsField = typeof(ThreatAssessment).GetField("activeThreatReports", BindingFlags.NonPublic | BindingFlags.Instance);
        activeThreatsField?.SetValue(threatAssessment, new List<ThreatReport> { repA, repB });

        replanningController.TryExecuteReplan("Initial Avoidance", repA);
        Assert.AreEqual(NavigationState.Rerouting, replanningController.State);

        // Advance time past cooldown
        FieldInfo lastReplanTimeField = typeof(ReplanningController).GetField("lastReplanTime", BindingFlags.NonPublic | BindingFlags.Instance);
        lastReplanTimeField?.SetValue(replanningController, Time.time - 5.0f);

        // Threat A clears, but Threat B remains Warning in ActiveThreatReports
        activeThreatsField?.SetValue(threatAssessment, new List<ThreatReport> { repB });
        FieldInfo currentReportField = typeof(ThreatAssessment).GetField("currentReport", BindingFlags.NonPublic | BindingFlags.Instance);
        currentReportField?.SetValue(threatAssessment, ThreatReport.Clear);

        typeof(ReplanningController).GetMethod("UpdateNavigationState", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(replanningController, null);

        // Must remain in Rerouting because Threat B is still an active Warning threat!
        Assert.AreEqual(NavigationState.Rerouting, replanningController.State, "UAV must remain in Rerouting while Threat B is still active!");

        Object.DestroyImmediate(obsA);
        Object.DestroyImmediate(obsB);
    }

    [Test]
    public void ThreatB_ClearsAfterThreatA_AllThreatsResolved_ReturnsToNormal()
    {
        List<Node> path = new List<Node> { gridManager.NodeFromWorldPoint(new Vector3(0f, 1f, 20f)) };
        pathFollower.StartFollowing(path);

        GameObject obs = new GameObject("Obs");
        obs.transform.position = new Vector3(0f, 1f, 8f);
        DetectedObstacle det = new DetectedObstacle(obs, null, obs.transform.position, obs.transform.position, Vector3.forward, 8f, 0f, Vector3.back, new Vector3(0f, 0f, -2f), true);
        ThreatReport rep = new ThreatReport(ThreatLevel.Critical, det, new Vector3(0f, 1f, 4f), 4f, 2f, 0);

        replanningController.TryExecuteReplan("Avoid", rep);
        Assert.AreEqual(NavigationState.Rerouting, replanningController.State);

        // Expire cooldown
        FieldInfo lastReplanTimeField = typeof(ReplanningController).GetField("lastReplanTime", BindingFlags.NonPublic | BindingFlags.Instance);
        lastReplanTimeField?.SetValue(replanningController, Time.time - 5.0f);

        // Clear all active threats
        FieldInfo activeThreatsField = typeof(ThreatAssessment).GetField("activeThreatReports", BindingFlags.NonPublic | BindingFlags.Instance);
        activeThreatsField?.SetValue(threatAssessment, new List<ThreatReport>());
        FieldInfo currentReportField = typeof(ThreatAssessment).GetField("currentReport", BindingFlags.NonPublic | BindingFlags.Instance);
        currentReportField?.SetValue(threatAssessment, ThreatReport.Clear);

        typeof(ReplanningController).GetMethod("UpdateNavigationState", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(replanningController, null);

        Assert.AreEqual(NavigationState.Normal, replanningController.State, "State must transition to Normal once all threats have cleared!");
        Assert.IsNull(replanningController.CurrentlyAvoidingObstacle);
        Assert.AreEqual(0, replanningController.CurrentlyAvoidingObstacles.Count);

        Object.DestroyImmediate(obs);
    }

    [Test]
    public void MultiThreatCooldown_PreventsRapidRepeatedReplans()
    {
        List<Node> path = new List<Node> { gridManager.NodeFromWorldPoint(new Vector3(0f, 1f, 20f)) };
        pathFollower.StartFollowing(path);

        GameObject obsA = new GameObject("ObsA");
        GameObject obsB = new GameObject("ObsB");
        obsA.transform.position = new Vector3(0f, 1f, 8f);
        obsB.transform.position = new Vector3(0f, 1f, 10f);
        DetectedObstacle detA = new DetectedObstacle(obsA, null, obsA.transform.position, obsA.transform.position, Vector3.forward, 8f, 0f, Vector3.back, new Vector3(0f, 0f, -2f), true);
        DetectedObstacle detB = new DetectedObstacle(obsB, null, obsB.transform.position, obsB.transform.position, Vector3.forward, 10f, 0f, Vector3.back, new Vector3(0f, 0f, -2f), true);

        ThreatReport repA = new ThreatReport(ThreatLevel.Critical, detA, new Vector3(0f, 1f, 4f), 4f, 2f, 0);
        ThreatReport repB = new ThreatReport(ThreatLevel.Critical, detB, new Vector3(0f, 1f, 5f), 5f, 2.5f, 0);

        bool firstReplan = replanningController.TryExecuteReplan("First Threat", repA);
        Assert.IsTrue(firstReplan);

        // Immediate subsequent replan within cooldown must be rejected
        bool immediateReplan = replanningController.TryExecuteReplan("Second Threat within Cooldown", repB);
        Assert.IsFalse(immediateReplan, "Immediate replan within cooldown window must be rejected!");

        Object.DestroyImmediate(obsA);
        Object.DestroyImmediate(obsB);
    }

    [Test]
    public void MultiThreatState_DoesNotOscillate()
    {
        List<Node> path = new List<Node> { gridManager.NodeFromWorldPoint(new Vector3(0f, 1f, 20f)) };
        pathFollower.StartFollowing(path);

        GameObject obs = new GameObject("Obs");
        obs.transform.position = new Vector3(0f, 1f, 8f);
        DetectedObstacle det = new DetectedObstacle(obs, null, obs.transform.position, obs.transform.position, Vector3.forward, 8f, 0f, Vector3.back, new Vector3(0f, 0f, -2f), true);
        ThreatReport rep = new ThreatReport(ThreatLevel.Critical, det, new Vector3(0f, 1f, 4f), 4f, 2f, 0);

        replanningController.TryExecuteReplan("Avoid", rep);
        Assert.AreEqual(NavigationState.Rerouting, replanningController.State);

        // Multiple rapid UpdateNavigationState calls while threat is active
        for (int i = 0; i < 10; i++)
        {
            typeof(ReplanningController).GetMethod("UpdateNavigationState", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(replanningController, null);
            Assert.AreEqual(NavigationState.Rerouting, replanningController.State);
        }

        Object.DestroyImmediate(obs);
    }

    [Test]
    public void MultiThreat_VOFailure_TriggersCompoundSpatialReplan()
    {
        List<Node> path = new List<Node> { gridManager.NodeFromWorldPoint(new Vector3(0f, 1f, 20f)) };
        pathFollower.StartFollowing(path);

        GameObject headOnObs = new GameObject("HeadOnObs");
        headOnObs.transform.position = new Vector3(0f, 1f, 10f);
        BoxCollider col = headOnObs.AddComponent<BoxCollider>();
        col.size = new Vector3(2f, 12f, 2f);
        col.center = new Vector3(0f, 5f, 0f);
        DetectedObstacle det = new DetectedObstacle(headOnObs, col, headOnObs.transform.position, headOnObs.transform.position, Vector3.forward, 10f, 0f, Vector3.back, new Vector3(0f, 0f, -2f), true);
        ThreatReport rep = new ThreatReport(ThreatLevel.Critical, det, new Vector3(0f, 1f, 5f), 5f, 2.5f, 0);

        bool result = replanningController.TryExecuteReplan("VO Failure Spatial Replan", rep);

        Assert.IsTrue(result);
        Assert.IsFalse(pathFollower.IsSpeedOverrideActive, "Head-on threat must fall through to spatial A* replan!");
        Assert.AreEqual(NavigationState.Rerouting, replanningController.State);
        Assert.Greater(pathfinding.path.Count, 0);

        Object.DestroyImmediate(headOnObs);
    }

    [Test]
    public void MultiThreat_MaximumFiveThreatsIsRespected()
    {
        List<Node> path = new List<Node> { gridManager.NodeFromWorldPoint(new Vector3(0f, 1f, 20f)) };
        pathFollower.StartFollowing(path);

        List<ThreatReport> manyThreats = new List<ThreatReport>();
        List<GameObject> objs = new List<GameObject>();

        for (int i = 0; i < 10; i++)
        {
            GameObject obj = new GameObject($"Threat_{i}");
            objs.Add(obj);
            obj.transform.position = new Vector3(i - 5, 1f, 8f + i);
            DetectedObstacle det = new DetectedObstacle(obj, null, obj.transform.position, obj.transform.position, Vector3.forward, 8f + i, 0f, Vector3.back, new Vector3(0f, 0f, -1f), true);
            manyThreats.Add(new ThreatReport(ThreatLevel.Warning, det, new Vector3(0f, 1f, 8f + i), 8f + i, 4f + i, 0));
        }

        FieldInfo activeThreatsField = typeof(ThreatAssessment).GetField("activeThreatReports", BindingFlags.NonPublic | BindingFlags.Instance);
        activeThreatsField?.SetValue(threatAssessment, manyThreats);

        Assert.DoesNotThrow(() =>
        {
            replanningController.TryExecuteReplan("10 Threats bounded", manyThreats[0]);
        });

        for (int i = 0; i < objs.Count; i++)
        {
            Object.DestroyImmediate(objs[i]);
        }
    }

    [Test]
    public void MultiThreat_HighestSeverityRemainsPrimaryThreat()
    {
        GameObject obsA = new GameObject("ObsA");
        GameObject obsB = new GameObject("ObsB");
        DetectedObstacle detA = new DetectedObstacle(obsA, null, new Vector3(0f, 1f, 6f), Vector3.forward * 6f, Vector3.forward, 6f, 0f, Vector3.back);
        DetectedObstacle detB = new DetectedObstacle(obsB, null, new Vector3(2f, 1f, 10f), Vector3.forward * 10.2f, Vector3.forward, 10.2f, 0f, Vector3.back);

        ThreatReport repA = new ThreatReport(ThreatLevel.Critical, detA, new Vector3(0f, 1f, 6f), 6f, 3f, 0);
        ThreatReport repB = new ThreatReport(ThreatLevel.Warning, detB, new Vector3(2f, 1f, 10f), 10f, 5f, 0);

        FieldInfo activeThreatsField = typeof(ThreatAssessment).GetField("activeThreatReports", BindingFlags.NonPublic | BindingFlags.Instance);
        activeThreatsField?.SetValue(threatAssessment, new List<ThreatReport> { repA, repB });

        FieldInfo currentReportField = typeof(ThreatAssessment).GetField("currentReport", BindingFlags.NonPublic | BindingFlags.Instance);
        currentReportField?.SetValue(threatAssessment, repA);

        Assert.AreEqual(ThreatLevel.Critical, threatAssessment.CurrentThreatLevel);
        Assert.AreEqual(obsA, threatAssessment.CurrentThreatReport.ThreateningObstacle.GameObject);

        Object.DestroyImmediate(obsA);
        Object.DestroyImmediate(obsB);
    }

    [Test]
    public void SingleThreatBehavior_RemainsBackwardCompatible()
    {
        List<Node> path = new List<Node> { gridManager.NodeFromWorldPoint(new Vector3(0f, 1f, 20f)) };
        pathFollower.StartFollowing(path);

        GameObject singleObs = new GameObject("SingleObs");
        singleObs.transform.position = new Vector3(-2f, 1f, 10f);
        DetectedObstacle det = new DetectedObstacle(singleObs, null, singleObs.transform.position, singleObs.transform.position, Vector3.forward, 10.2f, 0f, Vector3.back, new Vector3(1.5f, 0f, 0f), true);
        ThreatReport rep = new ThreatReport(ThreatLevel.Critical, det, new Vector3(0f, 1f, 10f), 10f, 5f, 0);

        bool replanResult = replanningController.TryExecuteReplan("Single Threat Replan", rep);

        Assert.IsTrue(replanResult);
        Assert.AreEqual(NavigationState.Rerouting, replanningController.State);

        Object.DestroyImmediate(singleObs);
    }

    [Test]
    public void StaticObstacleBehavior_RemainsUnchanged()
    {
        List<Node> path = new List<Node> { gridManager.NodeFromWorldPoint(new Vector3(0f, 1f, 20f)) };
        pathFollower.StartFollowing(path);

        GameObject staticObs = new GameObject("StaticObs");
        staticObs.transform.position = new Vector3(0f, 1f, 10f);
        DetectedObstacle det = new DetectedObstacle(staticObs, null, staticObs.transform.position, staticObs.transform.position, Vector3.forward, 10f, 0f, Vector3.back, Vector3.zero, false);
        ThreatReport rep = new ThreatReport(ThreatLevel.Critical, det, new Vector3(0f, 1f, 10f), 10f, 5f, 0);

        bool canModulate = replanningController.TryTacticalSpeedModulation(rep, out _);

        Assert.IsFalse(canModulate, "Static obstacles must not trigger speed modulation and must proceed directly to spatial replanning!");

        Object.DestroyImmediate(staticObs);
    }
}
