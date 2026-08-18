using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;

[TestFixture]
public class GridManagerTests
{
    private GameObject gridObj;
    private GridManager gridManager;

    [SetUp]
    public void SetUp()
    {
        gridObj = new GameObject("TestGridManager");
        gridManager = gridObj.AddComponent<GridManager>();
        gridManager.gridWorldSize = new Vector2(20, 20);
        gridManager.nodeRadius = 0.5f;
        gridManager.enableClearancePotentialField = true;
        gridManager.clearanceSafetyThreshold = 3.0f;
        gridManager.maxClearancePenalty = 20;
    }

    [TearDown]
    public void TearDown()
    {
        if (gridObj != null)
        {
            Object.DestroyImmediate(gridObj);
        }
    }

    [Test]
    public void GridManager_CreateGrid_GeneratesCorrectDimensions()
    {
        gridManager.CreateGrid();

        Assert.IsNotNull(gridManager.grid);
        Assert.AreEqual(20, gridManager.grid.GetLength(0)); // 20m / (0.5 * 2) = 20
        Assert.AreEqual(20, gridManager.grid.GetLength(1));
        Assert.AreEqual(400, gridManager.MaxSize);
    }

    [Test]
    public void GridManager_NodeFromWorldPoint_ClampsSafelyAtExtremeCoordinates()
    {
        gridManager.CreateGrid();

        Node centerNode = gridManager.NodeFromWorldPoint(Vector3.zero);
        Assert.IsNotNull(centerNode);
        Assert.AreEqual(10, centerNode.gridX);
        Assert.AreEqual(10, centerNode.gridY);

        // Extreme off-grid coordinates must clamp without IndexOutOfRangeException
        Node farNegative = gridManager.NodeFromWorldPoint(new Vector3(-500f, 0f, -500f));
        Assert.AreEqual(0, farNegative.gridX);
        Assert.AreEqual(0, farNegative.gridY);

        Node farPositive = gridManager.NodeFromWorldPoint(new Vector3(500f, 0f, 500f));
        Assert.AreEqual(19, farPositive.gridX);
        Assert.AreEqual(19, farPositive.gridY);
    }

    [Test]
    public void GridManager_GetNeighbors_Returns8NeighborsInCenterAnd3InCorner()
    {
        gridManager.CreateGrid();

        Node centerNode = gridManager.grid[10, 10];
        List<Node> centerNeighbors = gridManager.GetNeighbors(centerNode);
        Assert.AreEqual(8, centerNeighbors.Count);

        Node cornerNode = gridManager.grid[0, 0];
        List<Node> cornerNeighbors = gridManager.GetNeighbors(cornerNode);
        Assert.AreEqual(3, cornerNeighbors.Count);
    }

    [Test]
    public void GridManager_ClearancePotentialField_CalculatesMonotonicDistanceDecay()
    {
        gridManager.CreateGrid();

        // Artificially mark node [10, 10] as blocked obstacle
        gridManager.grid[10, 10].isWalkable = false;

        gridManager.CalculateClearancePotentialField();

        Node obstacleNode = gridManager.grid[10, 10];
        Node immediateNeighbor = gridManager.grid[10, 11]; // 1m away
        Node distanceNode = gridManager.grid[10, 15];      // 5m away

        Assert.AreEqual(0f, obstacleNode.clearanceDistance, 0.01f);
        Assert.AreEqual(1.0f, immediateNeighbor.clearanceDistance, 0.1f);
        Assert.Greater(immediateNeighbor.clearancePenalty, 0);

        // Far away node (> threshold 3.0m) should have 0 penalty
        Assert.Greater(distanceNode.clearanceDistance, 3.0f);
        Assert.AreEqual(0, distanceNode.clearancePenalty);
    }
}
