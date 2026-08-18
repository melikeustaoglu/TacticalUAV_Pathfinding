using System.Collections.Generic;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;

[TestFixture]
public class MultiThreatTelemetryTests
{
    private GameObject uavObj;
    private ThreatAssessment threatAssessment;
    private UAVPerception perception;
    private PathFollower pathFollower;
    private GridManager gridManager;
    private Pathfinding pathfinding;

    [SetUp]
    public void SetUp()
    {
        uavObj = new GameObject("TestUAV_MultiThreat");
        gridManager = uavObj.AddComponent<GridManager>();
        gridManager.gridWorldSize = new Vector2(30f, 30f);
        gridManager.nodeRadius = 0.5f;

        pathfinding = uavObj.AddComponent<Pathfinding>();
        typeof(Pathfinding).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(pathfinding, null);
        gridManager.CreateGrid();

        pathFollower = uavObj.AddComponent<PathFollower>();
        perception = uavObj.AddComponent<UAVPerception>();
        threatAssessment = uavObj.AddComponent<ThreatAssessment>();

        typeof(PathFollower).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(pathFollower, null);
        typeof(ThreatAssessment).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(threatAssessment, null);

        pathFollower.MoveSpeed = 2.0f;
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
    public void ThreatAssessment_MultipleObstaclesInFOV_PopulatesAllEvaluatedReportsList()
    {
        uavObj.transform.position = new Vector3(0f, 1f, 0f);
        uavObj.transform.rotation = Quaternion.identity;

        List<Node> path = new List<Node>
        {
            gridManager.NodeFromWorldPoint(new Vector3(0f, 1f, 20f))
        };
        pathFollower.StartFollowing(path);

        // Populate perception detectedObstacles via reflection
        FieldInfo obstaclesField = typeof(UAVPerception).GetField("detectedObstacles", BindingFlags.NonPublic | BindingFlags.Instance);
        List<DetectedObstacle> mockObstacles = new List<DetectedObstacle>
        {
            // Threat A: 10m ahead directly on path
            new DetectedObstacle(null, null, new Vector3(0f, 1f, 10f), new Vector3(0f, 1f, 10f), Vector3.forward, 10f, 0f, Vector3.back),
            // Threat B: 15m ahead slightly offset (Warning/Advisory)
            new DetectedObstacle(null, null, new Vector3(1.5f, 1f, 15f), new Vector3(1.5f, 1f, 15f), Vector3.forward, 15.07f, 5.7f, Vector3.back),
            // Threat C: 20m ahead far lateral (Advisory/None)
            new DetectedObstacle(null, null, new Vector3(6f, 1f, 20f), new Vector3(6f, 1f, 20f), Vector3.forward, 20.88f, 16.7f, Vector3.back)
        };
        obstaclesField?.SetValue(perception, mockObstacles);

        threatAssessment.EvaluateThreats();

        Assert.AreEqual(3, threatAssessment.AllEvaluatedReports.Count, "All 3 perceived obstacles must be evaluated and recorded!");
    }

    [Test]
    public void ThreatAssessment_MultipleThreatsAboveWarning_PopulatesActiveThreatReports()
    {
        uavObj.transform.position = new Vector3(0f, 1f, 0f);

        List<Node> path = new List<Node>
        {
            gridManager.NodeFromWorldPoint(new Vector3(0f, 1f, 20f))
        };
        pathFollower.StartFollowing(path);

        FieldInfo obstaclesField = typeof(UAVPerception).GetField("detectedObstacles", BindingFlags.NonPublic | BindingFlags.Instance);
        List<DetectedObstacle> mockObstacles = new List<DetectedObstacle>
        {
            // Direct collision threat (Critical)
            new DetectedObstacle(null, null, new Vector3(0f, 1f, 6f), new Vector3(0f, 1f, 6f), Vector3.forward, 6f, 0f, Vector3.back),
            // Close lateral proximity (Warning)
            new DetectedObstacle(null, null, new Vector3(1.8f, 1f, 8f), new Vector3(1.8f, 1f, 8f), Vector3.forward, 8.2f, 12.6f, Vector3.back),
            // Far lateral (None/Advisory)
            new DetectedObstacle(null, null, new Vector3(8.0f, 1f, 8f), new Vector3(8.0f, 1f, 8f), Vector3.forward, 11.3f, 45f, Vector3.back)
        };
        obstaclesField?.SetValue(perception, mockObstacles);

        threatAssessment.EvaluateThreats();

        // Expect at least 2 active threats (Critical + Warning)
        Assert.GreaterOrEqual(threatAssessment.ActiveThreatReports.Count, 2);
        for (int i = 0; i < threatAssessment.ActiveThreatReports.Count; i++)
        {
            Assert.GreaterOrEqual(threatAssessment.ActiveThreatReports[i].ThreatLevel, ThreatLevel.Warning);
        }
    }

    [Test]
    public void ThreatAssessment_HighestThreatSelection_PreservesCurrentThreatReportBehavior()
    {
        uavObj.transform.position = new Vector3(0f, 1f, 0f);

        List<Node> path = new List<Node>
        {
            gridManager.NodeFromWorldPoint(new Vector3(0f, 1f, 20f))
        };
        pathFollower.StartFollowing(path);

        FieldInfo obstaclesField = typeof(UAVPerception).GetField("detectedObstacles", BindingFlags.NonPublic | BindingFlags.Instance);
        List<DetectedObstacle> mockObstacles = new List<DetectedObstacle>
        {
            new DetectedObstacle(null, null, new Vector3(1.8f, 1f, 8f), new Vector3(1.8f, 1f, 8f), Vector3.forward, 8.2f, 12.6f, Vector3.back),
            new DetectedObstacle(null, null, new Vector3(0f, 1f, 6f), new Vector3(0f, 1f, 6f), Vector3.forward, 6f, 0f, Vector3.back)
        };
        obstaclesField?.SetValue(perception, mockObstacles);

        threatAssessment.EvaluateThreats();

        // Highest threat must be Critical
        Assert.AreEqual(ThreatLevel.Critical, threatAssessment.CurrentThreatLevel);
        Assert.AreEqual(ThreatLevel.Critical, threatAssessment.CurrentThreatReport.ThreatLevel);
        Assert.AreEqual(6.0f, threatAssessment.CurrentThreatReport.DistanceToCollision, 0.1f);
    }

    [Test]
    public void ThreatAssessment_ActiveThreatReports_AreSortedBySeverityDescending()
    {
        uavObj.transform.position = new Vector3(0f, 1f, 0f);

        List<Node> path = new List<Node>
        {
            gridManager.NodeFromWorldPoint(new Vector3(0f, 1f, 20f))
        };
        pathFollower.StartFollowing(path);

        FieldInfo obstaclesField = typeof(UAVPerception).GetField("detectedObstacles", BindingFlags.NonPublic | BindingFlags.Instance);
        List<DetectedObstacle> mockObstacles = new List<DetectedObstacle>
        {
            // Warning placed first in perception list
            new DetectedObstacle(null, null, new Vector3(1.8f, 1f, 8f), new Vector3(1.8f, 1f, 8f), Vector3.forward, 8.2f, 12.6f, Vector3.back),
            // Critical placed second
            new DetectedObstacle(null, null, new Vector3(0f, 1f, 6f), new Vector3(0f, 1f, 6f), Vector3.forward, 6f, 0f, Vector3.back)
        };
        obstaclesField?.SetValue(perception, mockObstacles);

        threatAssessment.EvaluateThreats();

        Assert.GreaterOrEqual(threatAssessment.ActiveThreatReports.Count, 2);
        // First element in ActiveThreatReports must be Critical
        Assert.AreEqual(ThreatLevel.Critical, threatAssessment.ActiveThreatReports[0].ThreatLevel);
    }

    [Test]
    public void ThreatAssessment_NoPerceivedObstacles_ClearsActiveThreatReports()
    {
        uavObj.transform.position = new Vector3(0f, 1f, 0f);

        FieldInfo obstaclesField = typeof(UAVPerception).GetField("detectedObstacles", BindingFlags.NonPublic | BindingFlags.Instance);
        obstaclesField?.SetValue(perception, new List<DetectedObstacle>());

        threatAssessment.EvaluateThreats();

        Assert.AreEqual(0, threatAssessment.ActiveThreatReports.Count);
        Assert.AreEqual(0, threatAssessment.AllEvaluatedReports.Count);
        Assert.AreEqual(ThreatLevel.None, threatAssessment.CurrentThreatLevel);
    }
}
