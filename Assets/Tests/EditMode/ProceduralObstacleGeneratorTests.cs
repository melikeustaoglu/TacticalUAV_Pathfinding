using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;

[TestFixture]
public class ProceduralObstacleGeneratorTests
{
    private GameObject gridOriginObj;

    [SetUp]
    public void SetUp()
    {
        gridOriginObj = new GameObject("TestGridOrigin");
    }

    [TearDown]
    public void TearDown()
    {
        if (gridOriginObj != null)
        {
            Object.DestroyImmediate(gridOriginObj);
        }
    }

    [Test]
    public void ProceduralGenerator_SameSeed_GeneratesIdenticalPositions()
    {
        Vector3 start = new Vector3(-10f, 1f, -10f);
        Vector3 target = new Vector3(10f, 1f, 10f);
        Vector2 size = new Vector2(50f, 50f);

        Transform obstaclesA = ProceduralObstacleGenerator.Generate(
            gridOriginObj.transform, size, start, target, obstacleCount: 10, seed: 42);

        List<Vector3> positionsA = new List<Vector3>();
        foreach (Transform child in obstaclesA)
        {
            positionsA.Add(child.position);
        }

        Transform obstaclesB = ProceduralObstacleGenerator.Generate(
            gridOriginObj.transform, size, start, target, obstacleCount: 10, seed: 42);

        List<Vector3> positionsB = new List<Vector3>();
        foreach (Transform child in obstaclesB)
        {
            positionsB.Add(child.position);
        }

        Assert.AreEqual(positionsA.Count, positionsB.Count);
        for (int i = 0; i < positionsA.Count; i++)
        {
            Assert.AreEqual(positionsA[i].x, positionsB[i].x, 0.001f);
            Assert.AreEqual(positionsA[i].z, positionsB[i].z, 0.001f);
        }

        Object.DestroyImmediate(obstaclesA.gameObject);
        Object.DestroyImmediate(obstaclesB.gameObject);
    }

    [Test]
    public void ProceduralGenerator_DifferentSeeds_GeneratesDivergentLayouts()
    {
        Vector3 start = new Vector3(-10f, 1f, -10f);
        Vector3 target = new Vector3(10f, 1f, 10f);
        Vector2 size = new Vector2(50f, 50f);

        Transform obstacles42 = ProceduralObstacleGenerator.Generate(
            gridOriginObj.transform, size, start, target, obstacleCount: 10, seed: 42);

        Transform obstacles100 = ProceduralObstacleGenerator.Generate(
            gridOriginObj.transform, size, start, target, obstacleCount: 10, seed: 100);

        bool foundDifference = false;
        int count = Mathf.Min(obstacles42.childCount, obstacles100.childCount);
        for (int i = 3; i < count; i++) // Skip first 3 deterministic corridor blockers
        {
            if (Vector3.Distance(obstacles42.GetChild(i).position, obstacles100.GetChild(i).position) > 0.5f)
            {
                foundDifference = true;
                break;
            }
        }

        Assert.IsTrue(foundDifference, "Different seeds should generate different scatter positions!");

        Object.DestroyImmediate(obstacles42.gameObject);
        Object.DestroyImmediate(obstacles100.gameObject);
    }

    [Test]
    public void ProceduralGenerator_SpawnAndTargetExclusion_RespectsClearanceRadius()
    {
        Vector3 start = new Vector3(-10f, 1f, -10f);
        Vector3 target = new Vector3(10f, 1f, 10f);
        Vector2 size = new Vector2(50f, 50f);
        float clearanceRadius = 4.0f;

        Transform obstacles = ProceduralObstacleGenerator.Generate(
            gridOriginObj.transform, size, start, target, obstacleCount: 15, seed: 77);

        foreach (Transform child in obstacles)
        {
            Vector3 flatPos = new Vector3(child.position.x, 0f, child.position.z);
            Vector3 flatStart = new Vector3(start.x, 0f, start.z);
            Vector3 flatTarget = new Vector3(target.x, 0f, target.z);

            Assert.GreaterOrEqual(Vector3.Distance(flatPos, flatStart), clearanceRadius - 0.1f);
            Assert.GreaterOrEqual(Vector3.Distance(flatPos, flatTarget), clearanceRadius - 0.1f);
        }

        Object.DestroyImmediate(obstacles.gameObject);
    }
}
