using System.Collections.Generic;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;

[TestFixture]
public class PathfindingTests
{
    private GameObject testObj;
    private GridManager gridManager;
    private Pathfinding pathfinding;

    [SetUp]
    public void SetUp()
    {
        testObj = new GameObject("TestPathfinding");
        gridManager = testObj.AddComponent<GridManager>();
        gridManager.gridWorldSize = new Vector2(20, 20);
        gridManager.nodeRadius = 0.5f;
        gridManager.enableClearancePotentialField = false; // Pure geometric A* for baseline test

        pathfinding = testObj.AddComponent<Pathfinding>();

        // Explicitly invoke Awake in EditMode to initialize internal gridManager reference
        MethodInfo awakeMethod = typeof(Pathfinding).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance);
        awakeMethod?.Invoke(pathfinding, null);

        gridManager.CreateGrid();
    }

    [TearDown]
    public void TearDown()
    {
        if (testObj != null)
        {
            Object.DestroyImmediate(testObj);
        }
    }

    [Test]
    public void Pathfinding_UnobstructedStraightLine_FindsPathReachingTarget()
    {
        Vector3 start = new Vector3(-8f, 1f, -8f);
        Vector3 target = new Vector3(8f, 1f, 8f);

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
    public void Pathfinding_SolidObstacleBarrier_FindsSafeDetourAroundBarrier()
    {
        // Place a solid barrier of blocked nodes across X from x = 5 to 15 at y = 10
        for (int x = 5; x <= 15; x++)
        {
            gridManager.grid[x, 10].isWalkable = false;
        }

        Vector3 start = gridManager.grid[10, 5].worldPosition;
        Vector3 target = gridManager.grid[10, 15].worldPosition;

        pathfinding.FindPath(start, target);

        Assert.IsNotNull(pathfinding.path);
        Assert.Greater(pathfinding.path.Count, 0);

        // Verify that no node in the returned path is on an unwalkable cell
        for (int i = 0; i < pathfinding.path.Count; i++)
        {
            Assert.IsTrue(pathfinding.path[i].isWalkable, $"Path node at index {i} was unwalkable!");
        }
    }

    [Test]
    public void Pathfinding_CompletelyEnclosedTarget_ReturnsEmptyPathGracefully()
    {
        Node targetNode = gridManager.grid[10, 10];
        // Enclose targetNode with impassable wall on all 8 neighbors
        List<Node> neighbors = gridManager.GetNeighbors(targetNode);
        for (int i = 0; i < neighbors.Count; i++)
        {
            neighbors[i].isWalkable = false;
        }
        targetNode.isWalkable = false;

        Vector3 start = gridManager.grid[2, 2].worldPosition;
        Vector3 target = targetNode.worldPosition;

        pathfinding.FindPath(start, target);

        // Path should be empty when target is unreachable
        Assert.AreEqual(0, pathfinding.path.Count);
    }

    [Test]
    public void Pathfinding_DeterministicExecution_YieldsIdenticalPaths()
    {
        Vector3 start = new Vector3(-6f, 1f, -4f);
        Vector3 target = new Vector3(6f, 1f, 4f);

        pathfinding.FindPath(start, target);
        List<Node> firstPath = new List<Node>(pathfinding.path);

        pathfinding.FindPath(start, target);
        List<Node> secondPath = new List<Node>(pathfinding.path);

        Assert.AreEqual(firstPath.Count, secondPath.Count);
        for (int i = 0; i < firstPath.Count; i++)
        {
            Assert.AreEqual(firstPath[i].gridX, secondPath[i].gridX);
            Assert.AreEqual(firstPath[i].gridY, secondPath[i].gridY);
        }
    }
}
