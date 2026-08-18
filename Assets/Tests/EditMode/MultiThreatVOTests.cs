using System.Collections.Generic;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;

[TestFixture]
public class MultiThreatVOTests
{
    private GameObject uavObj;
    private ThreatAssessment threatAssessment;
    private UAVPerception perception;
    private PathFollower pathFollower;
    private ReplanningController replanningController;
    private GridManager gridManager;
    private Pathfinding pathfinding;

    [SetUp]
    public void SetUp()
    {
        uavObj = new GameObject("TestUAV_MultiVO");
        uavObj.transform.position = new Vector3(0f, 1f, 0f);

        gridManager = uavObj.AddComponent<GridManager>();
        gridManager.gridWorldSize = new Vector2(30f, 30f);
        gridManager.nodeRadius = 0.5f;

        pathfinding = uavObj.AddComponent<Pathfinding>();
        typeof(Pathfinding).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(pathfinding, null);
        gridManager.CreateGrid();

        pathFollower = uavObj.AddComponent<PathFollower>();
        perception = uavObj.AddComponent<UAVPerception>();
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
    public void MultiVO_SingleThreat_PreservesExistingBehavior()
    {
        List<Node> path = new List<Node>
        {
            gridManager.NodeFromWorldPoint(new Vector3(0f, 1f, 20f))
        };
        pathFollower.StartFollowing(path);

        GameObject obsA = new GameObject("ObsA");
        obsA.transform.position = new Vector3(-2f, 1f, 10f);

        DetectedObstacle detA = new DetectedObstacle(
            obsA, null, obsA.transform.position, obsA.transform.position - uavObj.transform.position,
            Vector3.forward, 10.2f, 0f, Vector3.back, new Vector3(1.5f, 0f, 0f), isDynamic: true);

        ThreatReport reportA = new ThreatReport(
            ThreatLevel.Critical, detA, new Vector3(0f, 1f, 10f), 10.0f, 5.0f, 0);

        bool canModulate = replanningController.TryTacticalSpeedModulation(reportA, out float speedRatio);

        Assert.IsTrue(canModulate);
        Assert.GreaterOrEqual(speedRatio, 0.49f);

        Object.DestroyImmediate(obsA);
    }

    [Test]
    public void MultiVO_MultipleThreats_CandidateVelocitySafeAgainstAll_ReturnsTrue()
    {
        List<Node> path = new List<Node>
        {
            gridManager.NodeFromWorldPoint(new Vector3(0f, 1f, 20f))
        };
        pathFollower.StartFollowing(path);

        // Two crossing threats ahead moving parallel in same direction (+X)
        // Threat 1 at Z=8m moving +X at 1.5 m/s
        GameObject obs1 = new GameObject("Obs1");
        obs1.transform.position = new Vector3(-2f, 1f, 8f);
        DetectedObstacle det1 = new DetectedObstacle(
            obs1, null, obs1.transform.position, obs1.transform.position - uavObj.transform.position,
            Vector3.forward, 8.2f, 0f, Vector3.back, new Vector3(1.5f, 0f, 0f), isDynamic: true);
        ThreatReport rep1 = new ThreatReport(ThreatLevel.Critical, det1, new Vector3(0f, 1f, 8f), 8f, 4f, 0);

        // Threat 2 at Z=14m moving +X at 1.5 m/s
        GameObject obs2 = new GameObject("Obs2");
        obs2.transform.position = new Vector3(-2f, 1f, 14f);
        DetectedObstacle det2 = new DetectedObstacle(
            obs2, null, obs2.transform.position, obs2.transform.position - uavObj.transform.position,
            Vector3.forward, 14.1f, 0f, Vector3.back, new Vector3(1.5f, 0f, 0f), isDynamic: true);
        ThreatReport rep2 = new ThreatReport(ThreatLevel.Warning, det2, new Vector3(0f, 1f, 14f), 14f, 7f, 0);

        // Set ActiveThreatReports on ThreatAssessment via reflection
        FieldInfo activeThreatsField = typeof(ThreatAssessment).GetField("activeThreatReports", BindingFlags.NonPublic | BindingFlags.Instance);
        activeThreatsField?.SetValue(threatAssessment, new List<ThreatReport> { rep1, rep2 });

        bool canModulate = replanningController.TryTacticalSpeedModulation(rep1, out float speedRatio);

        Assert.IsTrue(canModulate, "Speed modulation should succeed when a single speed ratio safely clears both VO cones!");
        Assert.GreaterOrEqual(speedRatio, 0.49f);

        Object.DestroyImmediate(obs1);
        Object.DestroyImmediate(obs2);
    }

    [Test]
    public void MultiVO_MultipleThreats_CandidateVelocityUnsafeAgainstOne_ReturnsFalse()
    {
        List<Node> path = new List<Node>
        {
            gridManager.NodeFromWorldPoint(new Vector3(0f, 1f, 20f))
        };
        pathFollower.StartFollowing(path);

        // Threat 1: Crossing threat that COULD be resolved by slowing down
        GameObject obs1 = new GameObject("CrossingObs");
        obs1.transform.position = new Vector3(-2f, 1f, 8f);
        DetectedObstacle det1 = new DetectedObstacle(
            obs1, null, obs1.transform.position, obs1.transform.position - uavObj.transform.position,
            Vector3.forward, 8.2f, 0f, Vector3.back, new Vector3(1.5f, 0f, 0f), isDynamic: true);
        ThreatReport rep1 = new ThreatReport(ThreatLevel.Critical, det1, new Vector3(0f, 1f, 8f), 8f, 4f, 0);

        // Threat 2: Head-on threat coming straight down -Z (cannot be resolved by speed reduction)
        GameObject obs2 = new GameObject("HeadOnObs");
        obs2.transform.position = new Vector3(0f, 1f, 12f);
        DetectedObstacle det2 = new DetectedObstacle(
            obs2, null, obs2.transform.position, obs2.transform.position - uavObj.transform.position,
            Vector3.forward, 12f, 0f, Vector3.back, new Vector3(0f, 0f, -2.0f), isDynamic: true);
        ThreatReport rep2 = new ThreatReport(ThreatLevel.Critical, det2, new Vector3(0f, 1f, 6f), 6f, 3f, 0);

        FieldInfo activeThreatsField = typeof(ThreatAssessment).GetField("activeThreatReports", BindingFlags.NonPublic | BindingFlags.Instance);
        activeThreatsField?.SetValue(threatAssessment, new List<ThreatReport> { rep1, rep2 });

        bool canModulate = replanningController.TryTacticalSpeedModulation(rep1, out _);

        Assert.IsFalse(canModulate, "Speed modulation must be rejected if even ONE active threat remains in collision!");

        Object.DestroyImmediate(obs1);
        Object.DestroyImmediate(obs2);
    }

    [Test]
    public void MultiVO_ConflictingThreats_RejectSpeedModulation()
    {
        List<Node> path = new List<Node>
        {
            gridManager.NodeFromWorldPoint(new Vector3(0f, 1f, 20f))
        };
        pathFollower.StartFollowing(path);

        // Threat 1: Moving left-to-right at Z=6m
        GameObject obs1 = new GameObject("ThreatLtoR");
        obs1.transform.position = new Vector3(-1f, 1f, 6f);
        DetectedObstacle det1 = new DetectedObstacle(
            obs1, null, obs1.transform.position, obs1.transform.position - uavObj.transform.position,
            Vector3.forward, 6.1f, 0f, Vector3.back, new Vector3(2.0f, 0f, 0f), isDynamic: true);
        ThreatReport rep1 = new ThreatReport(ThreatLevel.Critical, det1, new Vector3(0f, 1f, 6f), 6f, 3f, 0);

        // Threat 2: Head-on at Z=10m
        GameObject obs2 = new GameObject("ThreatHeadOn");
        obs2.transform.position = new Vector3(0f, 1f, 10f);
        DetectedObstacle det2 = new DetectedObstacle(
            obs2, null, obs2.transform.position, obs2.transform.position - uavObj.transform.position,
            Vector3.forward, 10f, 0f, Vector3.back, new Vector3(0f, 0f, -1.5f), isDynamic: true);
        ThreatReport rep2 = new ThreatReport(ThreatLevel.Critical, det2, new Vector3(0f, 1f, 10f), 10f, 5f, 0);

        FieldInfo activeThreatsField = typeof(ThreatAssessment).GetField("activeThreatReports", BindingFlags.NonPublic | BindingFlags.Instance);
        activeThreatsField?.SetValue(threatAssessment, new List<ThreatReport> { rep1, rep2 });

        bool canModulate = replanningController.TryTacticalSpeedModulation(rep1, out _);

        Assert.IsFalse(canModulate, "Conflicting VO cones must reject speed modulation and fall back to spatial A* replanning!");

        Object.DestroyImmediate(obs1);
        Object.DestroyImmediate(obs2);
    }

    [Test]
    public void TryTacticalSpeedModulation_MultipleSafeThreats_AppliesSpeedOverride()
    {
        List<Node> path = new List<Node>
        {
            gridManager.NodeFromWorldPoint(new Vector3(0f, 1f, 20f))
        };
        pathFollower.StartFollowing(path);

        GameObject obs = new GameObject("SafeCrossingObs");
        obs.transform.position = new Vector3(-2f, 1f, 10f);
        DetectedObstacle det = new DetectedObstacle(
            obs, null, obs.transform.position, obs.transform.position - uavObj.transform.position,
            Vector3.forward, 10.2f, 0f, Vector3.back, new Vector3(1.5f, 0f, 0f), isDynamic: true);
        ThreatReport rep = new ThreatReport(ThreatLevel.Critical, det, new Vector3(0f, 1f, 10f), 10f, 5f, 0);

        bool replanResult = replanningController.TryExecuteReplan("Multi-VO Safe Pacing", rep);

        Assert.IsTrue(replanResult);
        Assert.IsTrue(pathFollower.IsSpeedOverrideActive);
        Assert.AreEqual(NavigationState.Rerouting, replanningController.State);

        Object.DestroyImmediate(obs);
    }

    [Test]
    public void TryTacticalSpeedModulation_OneBlockingThreat_FallsBackToSpatialReplan()
    {
        List<Node> path = new List<Node>
        {
            gridManager.NodeFromWorldPoint(new Vector3(0f, 1f, 20f))
        };
        pathFollower.StartFollowing(path);

        // Head-on blocking threat
        GameObject obs = new GameObject("BlockingHeadOnObs");
        obs.transform.position = new Vector3(0f, 1f, 10f);
        DetectedObstacle det = new DetectedObstacle(
            obs, null, obs.transform.position, obs.transform.position - uavObj.transform.position,
            Vector3.forward, 10f, 0f, Vector3.back, new Vector3(0f, 0f, -2.0f), isDynamic: true);
        ThreatReport rep = new ThreatReport(ThreatLevel.Critical, det, new Vector3(0f, 1f, 5f), 5f, 2.5f, 0);

        bool replanResult = replanningController.TryExecuteReplan("Head-On Threat Fallback", rep);

        Assert.IsTrue(replanResult);
        // Should execute spatial A* replan, NOT speed override
        Assert.IsFalse(pathFollower.IsSpeedOverrideActive, "Head-on collision must fall back to spatial A* replanning!");
        Assert.AreEqual(NavigationState.Rerouting, replanningController.State);

        Object.DestroyImmediate(obs);
    }

    [Test]
    public void MultiVO_StaticObstacle_DoesNotTriggerSpeedModulation()
    {
        List<Node> path = new List<Node>
        {
            gridManager.NodeFromWorldPoint(new Vector3(0f, 1f, 20f))
        };
        pathFollower.StartFollowing(path);

        GameObject staticObs = new GameObject("StaticObs");
        staticObs.transform.position = new Vector3(0f, 1f, 10f);
        DetectedObstacle det = new DetectedObstacle(
            staticObs, null, staticObs.transform.position, staticObs.transform.position - uavObj.transform.position,
            Vector3.forward, 10f, 0f, Vector3.back, Vector3.zero, isDynamic: false);
        ThreatReport rep = new ThreatReport(ThreatLevel.Critical, det, new Vector3(0f, 1f, 10f), 10f, 5f, 0);

        bool canModulate = replanningController.TryTacticalSpeedModulation(rep, out _);

        Assert.IsFalse(canModulate, "Static obstacle must never trigger speed modulation!");

        Object.DestroyImmediate(staticObs);
    }

    [Test]
    public void MultiVO_MaximumThreatLimit_IsRespected()
    {
        List<Node> path = new List<Node>
        {
            gridManager.NodeFromWorldPoint(new Vector3(0f, 1f, 20f))
        };
        pathFollower.StartFollowing(path);

        List<ThreatReport> manyThreats = new List<ThreatReport>();
        List<GameObject> objs = new List<GameObject>();

        for (int i = 0; i < 10; i++)
        {
            GameObject obj = new GameObject($"Threat_{i}");
            objs.Add(obj);
            obj.transform.position = new Vector3(-2f, 1f, 5f + i * 2f);
            DetectedObstacle det = new DetectedObstacle(
                obj, null, obj.transform.position, obj.transform.position - uavObj.transform.position,
                Vector3.forward, 5f + i * 2f, 0f, Vector3.back, new Vector3(1.5f, 0f, 0f), isDynamic: true);
            manyThreats.Add(new ThreatReport(ThreatLevel.Warning, det, new Vector3(0f, 1f, 5f + i * 2f), 5f + i * 2f, 3f + i, 0));
        }

        FieldInfo activeThreatsField = typeof(ThreatAssessment).GetField("activeThreatReports", BindingFlags.NonPublic | BindingFlags.Instance);
        activeThreatsField?.SetValue(threatAssessment, manyThreats);

        // Ensure no out of bounds / exception occurs with 10 threats (capped at top 5)
        Assert.DoesNotThrow(() =>
        {
            replanningController.TryTacticalSpeedModulation(manyThreats[0], out _);
        });

        for (int i = 0; i < objs.Count; i++)
        {
            Object.DestroyImmediate(objs[i]);
        }
    }
}
