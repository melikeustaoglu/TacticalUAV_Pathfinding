using System.Collections.Generic;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;

[TestFixture]
public class TacticalSpeedModulationTests
{
    private GameObject uavObj;
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
        gridManager.gridWorldSize = new Vector2(30f, 30f);
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

        pathFollower.MoveSpeed = 2.0f; // Nominal cruise speed 2 m/s
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
    public void TacticalSpeedOverride_AppliesConfiguredSpeedRatioCorrectly()
    {
        pathFollower.ApplyTacticalSpeedOverride(0.6f, 3.0f);

        Assert.IsTrue(pathFollower.IsSpeedOverrideActive);
        Assert.AreEqual(0.6f, pathFollower.CurrentSpeedOverrideRatio, 0.01f);
    }

    [Test]
    public void TacticalSpeedOverride_EnforcesMinimumSpeedClamp_NeverBelow0Point5Mps()
    {
        pathFollower.MoveSpeed = 1.0f;

        // Attempting to override to 0.1 ratio (0.1 m/s)
        pathFollower.ApplyTacticalSpeedOverride(0.1f, 3.0f);

        // Effective speed must be clamped to minimum 0.5 m/s (0.5 ratio of 1.0 m/s)
        Assert.GreaterOrEqual(pathFollower.MoveSpeed * pathFollower.CurrentSpeedOverrideRatio, 0.499f);
    }

    [Test]
    public void TacticalSpeedOverride_AutomaticallyExpiresAfterDuration()
    {
        pathFollower.ApplyTacticalSpeedOverride(0.5f, 2.0f);
        Assert.IsTrue(pathFollower.IsSpeedOverrideActive);

        // Fast-forward expiration timestamp via reflection
        FieldInfo endTimeField = typeof(PathFollower).GetField("speedOverrideEndTime", BindingFlags.NonPublic | BindingFlags.Instance);
        endTimeField?.SetValue(pathFollower, Time.time - 1.0f);

        Assert.IsFalse(pathFollower.IsSpeedOverrideActive, "Speed override must be reported inactive after duration expires!");
        Assert.AreEqual(1.0f, pathFollower.CurrentSpeedOverrideRatio, 0.01f);
    }

    [Test]
    public void TacticalSpeedOverride_ClearOverride_RestoresNominalCruiseSpeedImmediately()
    {
        pathFollower.ApplyTacticalSpeedOverride(0.5f, 5.0f);
        Assert.IsTrue(pathFollower.IsSpeedOverrideActive);

        pathFollower.ClearSpeedOverride();

        Assert.IsFalse(pathFollower.IsSpeedOverrideActive);
        Assert.AreEqual(1.0f, pathFollower.CurrentSpeedOverrideRatio, 0.01f);
    }

    [Test]
    public void TacticalSpeedOverride_RepeatedRequests_AreBoundedWithoutOscillation()
    {
        pathFollower.ApplyTacticalSpeedOverride(0.7f, 2.0f);
        pathFollower.ApplyTacticalSpeedOverride(0.6f, 3.0f);
        pathFollower.ApplyTacticalSpeedOverride(0.5f, 4.0f);

        Assert.IsTrue(pathFollower.IsSpeedOverrideActive);
        Assert.AreEqual(0.5f, pathFollower.CurrentSpeedOverrideRatio, 0.01f);
    }

    [Test]
    public void TryTacticalSpeedModulation_CrossingDynamicObstacle_AcceptsVOSafeSpeed()
    {
        uavObj.transform.position = Vector3.zero;
        pathFollower.MoveSpeed = 2.0f;

        // Set path heading +Z towards (0, 0, 20)
        List<Node> path = new List<Node>
        {
            gridManager.NodeFromWorldPoint(new Vector3(0f, 1f, 20f))
        };
        pathFollower.StartFollowing(path);

        // Dynamic obstacle crossing from left to right at Z=10m, moving +X at 1.5 m/s
        GameObject obsObj = new GameObject("CrossingDynamicObs");
        obsObj.transform.position = new Vector3(-2f, 1f, 10f);

        DetectedObstacle crossingObs = new DetectedObstacle(
            obsObj,
            null,
            obsObj.transform.position,
            obsObj.transform.position - uavObj.transform.position,
            Vector3.forward,
            10.2f,
            0f,
            Vector3.back,
            new Vector3(1.5f, 0f, 0f),
            isDynamic: true);

        ThreatReport report = new ThreatReport(
            ThreatLevel.Critical,
            crossingObs,
            new Vector3(0f, 1f, 10f),
            distanceToCollision: 10.0f,
            timeToCollision: 5.0f,
            obstructedWaypointIndex: 0);

        bool canModulate = replanningController.TryTacticalSpeedModulation(report, out float recommendedRatio);

        // Speed modulation should successfully find a safe reduced speed ratio
        Assert.IsTrue(canModulate, "Crossing dynamic threat should be resolvable by tactical VO speed pacing!");
        Assert.GreaterOrEqual(recommendedRatio, 0.49f);
        Assert.LessOrEqual(recommendedRatio, 0.85f);

        Object.DestroyImmediate(obsObj);
    }

    [Test]
    public void TryTacticalSpeedModulation_HeadOnDirectCollision_CannotEscapeVO_ReturnsFalse()
    {
        uavObj.transform.position = Vector3.zero;
        pathFollower.MoveSpeed = 2.0f;

        List<Node> path = new List<Node>
        {
            gridManager.NodeFromWorldPoint(new Vector3(0f, 1f, 20f))
        };
        pathFollower.StartFollowing(path);

        // Head-on moving threat coming straight down -Z directly along flight line
        GameObject obsObj = new GameObject("HeadOnDynamicObs");
        obsObj.transform.position = new Vector3(0f, 1f, 10f);

        DetectedObstacle headOnObs = new DetectedObstacle(
            obsObj,
            null,
            obsObj.transform.position,
            obsObj.transform.position - uavObj.transform.position,
            Vector3.forward,
            10.0f,
            0f,
            Vector3.back,
            new Vector3(0f, 0f, -2.0f), // Head on at 2 m/s
            isDynamic: true);

        ThreatReport report = new ThreatReport(
            ThreatLevel.Critical,
            headOnObs,
            new Vector3(0f, 1f, 5f),
            distanceToCollision: 5.0f,
            timeToCollision: 2.5f,
            obstructedWaypointIndex: 0);

        bool canModulate = replanningController.TryTacticalSpeedModulation(report, out _);

        // In a direct head-on collision, simply slowing down does not move the velocity vector outside the VO cone
        Assert.IsFalse(canModulate, "Direct head-on collision cannot escape VO by speed reduction alone and must trigger spatial A* replan!");

        Object.DestroyImmediate(obsObj);
    }

    [Test]
    public void TryTacticalSpeedModulation_StaticObstacle_NeverTriggersSpeedModulation()
    {
        uavObj.transform.position = Vector3.zero;
        pathFollower.MoveSpeed = 2.0f;

        List<Node> path = new List<Node>
        {
            gridManager.NodeFromWorldPoint(new Vector3(0f, 1f, 20f))
        };
        pathFollower.StartFollowing(path);

        GameObject staticObj = new GameObject("StaticObs");
        staticObj.transform.position = new Vector3(0f, 1f, 10f);

        DetectedObstacle staticObs = new DetectedObstacle(
            staticObj,
            null,
            staticObj.transform.position,
            staticObj.transform.position - uavObj.transform.position,
            Vector3.forward,
            10.0f,
            0f,
            Vector3.back,
            Vector3.zero,
            isDynamic: false);

        ThreatReport report = new ThreatReport(
            ThreatLevel.Critical,
            staticObs,
            new Vector3(0f, 1f, 10f),
            distanceToCollision: 10.0f,
            timeToCollision: 5.0f,
            obstructedWaypointIndex: 0);

        bool canModulate = replanningController.TryTacticalSpeedModulation(report, out _);

        Assert.IsFalse(canModulate, "Static obstacles must never trigger speed modulation and must proceed directly to spatial A* replan!");

        Object.DestroyImmediate(staticObj);
    }
}
