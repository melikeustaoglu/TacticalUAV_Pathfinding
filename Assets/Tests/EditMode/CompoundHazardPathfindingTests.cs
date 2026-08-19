using System.Collections.Generic;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;

[TestFixture]
public class CompoundHazardPathfindingTests
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
        uavObj = new GameObject("TestUAV_CompoundPathfinding");
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
    public void CompoundHazardPathfinding_TwoHazards_AvoidsBoth()
    {
        Vector3 start = new Vector3(0f, 1f, 0f);
        Vector3 target = new Vector3(0f, 1f, 20f);

        // Place two dynamic hazards directly along the Z axis
        List<DynamicHazard> hazards = new List<DynamicHazard>
        {
            new DynamicHazard(new Vector3(0f, 1f, 6f), 2.0f),
            new DynamicHazard(new Vector3(0f, 1f, 14f), 2.0f)
        };

        pathfinding.FindPath(start, target, hazards);

        Assert.IsNotNull(pathfinding.path);
        Assert.Greater(pathfinding.path.Count, 0, "A valid detour path must be found around both hazards!");

        // Verify that every node in the path respects both hazard footprints
        for (int i = 0; i < pathfinding.path.Count; i++)
        {
            Vector3 nodePos = pathfinding.path[i].worldPosition;
            Vector3 nodeFlat = new Vector3(nodePos.x, 0f, nodePos.z);

            for (int h = 0; h < hazards.Count; h++)
            {
                Vector3 hazardFlat = new Vector3(hazards[h].Position.x, 0f, hazards[h].Position.z);
                float dist = Vector3.Distance(nodeFlat, hazardFlat);

                // Allow start/end nodes if coincident, but intermediate nodes must maintain clearance
                if (i > 0 && i < pathfinding.path.Count - 1)
                {
                    Assert.GreaterOrEqual(dist, hazards[h].Radius - 0.2f,
                        $"Node {i} at {nodePos} is inside Hazard {h} at {hazards[h].Position} (dist={dist:F2}, radius={hazards[h].Radius:F2})");
                }
            }
        }
    }

    [Test]
    public void CompoundHazardPathfinding_ThreeHazards_AvoidsAll()
    {
        Vector3 start = new Vector3(0f, 1f, 0f);
        Vector3 target = new Vector3(0f, 1f, 20f);

        // Staggered hazard pattern
        List<DynamicHazard> hazards = new List<DynamicHazard>
        {
            new DynamicHazard(new Vector3(0f, 1f, 5f), 1.8f),
            new DynamicHazard(new Vector3(2f, 1f, 10f), 1.8f),
            new DynamicHazard(new Vector3(-2f, 1f, 15f), 1.8f)
        };

        pathfinding.FindPath(start, target, hazards);

        Assert.IsNotNull(pathfinding.path);
        Assert.Greater(pathfinding.path.Count, 0);

        for (int i = 1; i < pathfinding.path.Count - 1; i++)
        {
            Vector3 nodePos = pathfinding.path[i].worldPosition;
            Vector3 nodeFlat = new Vector3(nodePos.x, 0f, nodePos.z);

            for (int h = 0; h < hazards.Count; h++)
            {
                Vector3 hazardFlat = new Vector3(hazards[h].Position.x, 0f, hazards[h].Position.z);
                float dist = Vector3.Distance(nodeFlat, hazardFlat);
                Assert.GreaterOrEqual(dist, hazards[h].Radius - 0.2f);
            }
        }
    }

    [Test]
    public void CompoundHazardPathfinding_PathSmoothing_DoesNotCutAcrossHazard()
    {
        Vector3 p1 = new Vector3(0f, 1f, 0f);
        Vector3 p2 = new Vector3(0f, 1f, 20f);

        // Place a hazard directly between p1 and p2
        List<DynamicHazard> hazards = new List<DynamicHazard>
        {
            new DynamicHazard(new Vector3(0f, 1f, 10f), 2.5f)
        };

        // Corridor check between p1 and p2 must report FALSE because hazard is directly on line
        bool isClear = pathfinding.IsCorridorClear(p1, p2, hazards);
        Assert.IsFalse(isClear, "Path smoothing must not allow direct straight line through dynamic hazard!");
    }

    [Test]
    public void CompoundHazardPathfinding_SingleHazard_PreservesExistingBehavior()
    {
        Vector3 start = new Vector3(0f, 1f, 0f);
        Vector3 target = new Vector3(0f, 1f, 20f);
        Vector3 hazardPos = new Vector3(0f, 1f, 10f);
        float hazardRadius = 2.0f;

        // Legacy single-hazard overload
        pathfinding.FindPath(start, target, hazardPos, hazardRadius);

        Assert.IsNotNull(pathfinding.path);
        Assert.Greater(pathfinding.path.Count, 0);

        for (int i = 1; i < pathfinding.path.Count - 1; i++)
        {
            Vector3 nodeFlat = new Vector3(pathfinding.path[i].worldPosition.x, 0f, pathfinding.path[i].worldPosition.z);
            Vector3 hazardFlat = new Vector3(hazardPos.x, 0f, hazardPos.z);
            Assert.GreaterOrEqual(Vector3.Distance(nodeFlat, hazardFlat), hazardRadius - 0.2f);
        }
    }

    [Test]
    public void CompoundHazardPathfinding_StaticObstaclesRemainUnchanged()
    {
        Vector3 start = new Vector3(0f, 1f, 0f);
        Vector3 target = new Vector3(0f, 1f, 20f);

        // Path without dynamic hazards should find direct or near-direct path
        pathfinding.FindPath(start, target, (IReadOnlyList<DynamicHazard>)null);

        Assert.IsNotNull(pathfinding.path);
        Assert.Greater(pathfinding.path.Count, 0);
    }

    [Test]
    public void CompoundHazardPathfinding_ConflictingHazards_FindsSafeAlternativeWhenAvailable()
    {
        Vector3 start = new Vector3(0f, 1f, 0f);
        Vector3 target = new Vector3(0f, 1f, 20f);

        // Hazards blocking center (X=0) and left (X=-3), leaving right side (X=+4) open
        List<DynamicHazard> hazards = new List<DynamicHazard>
        {
            new DynamicHazard(new Vector3(0f, 1f, 10f), 2.5f),
            new DynamicHazard(new Vector3(-3f, 1f, 10f), 2.5f)
        };

        pathfinding.FindPath(start, target, hazards);

        Assert.IsNotNull(pathfinding.path);
        Assert.Greater(pathfinding.path.Count, 0);

        // Detour must route through positive X
        bool routedThroughRightSide = false;
        for (int i = 0; i < pathfinding.path.Count; i++)
        {
            if (pathfinding.path[i].worldPosition.x > 1.5f)
            {
                routedThroughRightSide = true;
                break;
            }
        }

        Assert.IsTrue(routedThroughRightSide, "Path should route around the left/center block through the open right flank!");
    }

    [Test]
    public void CompoundHazardPathfinding_NoSafeRoute_ReturnsFailureGracefully()
    {
        Vector3 start = new Vector3(0f, 1f, 0f);
        Vector3 target = new Vector3(0f, 1f, 20f);

        // Impassable wall of hazards across the entire grid width at Z=10m
        List<DynamicHazard> wallHazards = new List<DynamicHazard>();
        for (float x = -25f; x <= 25f; x += 2f)
        {
            wallHazards.Add(new DynamicHazard(new Vector3(x, 1f, 10f), 2.5f));
        }

        pathfinding.FindPath(start, target, wallHazards);

        // No path should be found, but it must complete gracefully with 0 elements
        Assert.AreEqual(0, pathfinding.path.Count, "Path count should be 0 when entire corridor is blocked!");
    }

    [Test]
    public void ReplanningController_MultiThreatSpatialFallback_PassesAllActiveHazards()
    {
        List<Node> path = new List<Node>
        {
            gridManager.NodeFromWorldPoint(new Vector3(0f, 1f, 20f))
        };
        pathFollower.StartFollowing(path);

        // Threat 1: Head-on critical threat with tall collider exceeding ceiling (forcing Stage 3 spatial fallback)
        GameObject obs1 = new GameObject("HeadOnThreat");
        obs1.transform.position = new Vector3(0f, 1f, 8f);
        BoxCollider col1 = obs1.AddComponent<BoxCollider>();
        col1.size = new Vector3(2f, 12f, 2f);
        col1.center = new Vector3(0f, 5f, 0f);
        DetectedObstacle det1 = new DetectedObstacle(
            obs1, col1, obs1.transform.position, obs1.transform.position - uavObj.transform.position,
            Vector3.forward, 8f, 0f, Vector3.back, new Vector3(0f, 0f, -2.0f), isDynamic: true);
        ThreatReport rep1 = new ThreatReport(ThreatLevel.Critical, det1, new Vector3(0f, 1f, 4f), 4f, 2f, 0);

        // Threat 2: Active threat at (2, 1, 12)
        GameObject obs2 = new GameObject("SecondThreat");
        obs2.transform.position = new Vector3(2f, 1f, 12f);
        DetectedObstacle det2 = new DetectedObstacle(
            obs2, null, obs2.transform.position, obs2.transform.position - uavObj.transform.position,
            Vector3.forward, 12.1f, 0f, Vector3.back, new Vector3(0f, 0f, -1.0f), isDynamic: true);
        ThreatReport rep2 = new ThreatReport(ThreatLevel.Warning, det2, new Vector3(2f, 1f, 8f), 8f, 4f, 0);

        FieldInfo activeThreatsField = typeof(ThreatAssessment).GetField("activeThreatReports", BindingFlags.NonPublic | BindingFlags.Instance);
        activeThreatsField?.SetValue(threatAssessment, new List<ThreatReport> { rep1, rep2 });

        bool replanResult = replanningController.TryExecuteReplan("Multi-Threat Fallback Replan", rep1);

        Assert.IsTrue(replanResult);
        Assert.AreEqual(NavigationState.Rerouting, replanningController.State);
        Assert.Greater(pathfinding.path.Count, 0);

        Object.DestroyImmediate(obs1);
        Object.DestroyImmediate(obs2);
    }
}
