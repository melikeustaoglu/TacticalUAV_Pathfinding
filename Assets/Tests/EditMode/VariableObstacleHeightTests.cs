using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;

[TestFixture]
public class VariableObstacleHeightTests
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
    public void LegacyMode_DisabledVariableHeights_PreservesIsotropicObstacleBehavior()
    {
        Vector3 start = new Vector3(-10f, 1f, -10f);
        Vector3 target = new Vector3(10f, 1f, 10f);
        Vector2 size = new Vector2(50f, 50f);

        Transform obstacles = ProceduralObstacleGenerator.Generate(
            gridOriginObj.transform,
            size,
            start,
            target,
            obstacleCount: 10,
            seed: 42,
            distributionMode: ObstacleDistributionMode.Uniform,
            enableVariableObstacleHeights: false);

        Assert.Greater(obstacles.childCount, 0);

        foreach (Transform child in obstacles)
        {
            Vector3 scale = child.localScale;
            // In legacy mode, X, Y, and Z scale must be exactly equal (isotropic cube)
            Assert.AreEqual(scale.x, scale.y, 0.0001f, $"Obstacle {child.name} Y scale ({scale.y}) does not match X scale ({scale.x}) in legacy mode!");
            Assert.AreEqual(scale.x, scale.z, 0.0001f, $"Obstacle {child.name} Z scale ({scale.z}) does not match X scale ({scale.x}) in legacy mode!");

            // In legacy mode, center position Y must be exactly 0.5f
            Assert.AreEqual(0.5f, child.position.y, 0.0001f, $"Obstacle {child.name} Y position ({child.position.y}) is not 0.5f in legacy mode!");
        }

        Object.DestroyImmediate(obstacles.gameObject);
    }

    [Test]
    public void VariableHeights_Enabled_ProducesHeightsWithinConfiguredMinMax()
    {
        Vector3 start = new Vector3(-10f, 1f, -10f);
        Vector3 target = new Vector3(10f, 1f, 10f);
        Vector2 size = new Vector2(50f, 50f);

        float minH = 1.5f;
        float maxH = 4.5f;
        float defaultH = 2.0f;

        Transform obstacles = ProceduralObstacleGenerator.Generate(
            gridOriginObj.transform,
            size,
            start,
            target,
            obstacleCount: 15,
            seed: 123,
            distributionMode: ObstacleDistributionMode.Uniform,
            enableVariableObstacleHeights: true,
            minObstacleHeight: minH,
            maxObstacleHeight: maxH,
            defaultObstacleHeight: defaultH);

        Assert.Greater(obstacles.childCount, 0);

        foreach (Transform child in obstacles)
        {
            float height = child.localScale.y;
            Assert.GreaterOrEqual(height, minH - 0.001f, $"Obstacle {child.name} height {height:F2} is below minObstacleHeight {minH}!");
            Assert.LessOrEqual(height, maxH + 0.001f, $"Obstacle {child.name} height {height:F2} exceeds maxObstacleHeight {maxH}!");
        }

        Object.DestroyImmediate(obstacles.gameObject);
    }

    [Test]
    public void VariableHeights_GroundAlignment_BottomPlaneIsStrictlyAtGroundZero()
    {
        Vector3 start = new Vector3(-10f, 1f, -10f);
        Vector3 target = new Vector3(10f, 1f, 10f);
        Vector2 size = new Vector2(50f, 50f);

        Transform obstacles = ProceduralObstacleGenerator.Generate(
            gridOriginObj.transform,
            size,
            start,
            target,
            obstacleCount: 12,
            seed: 888,
            distributionMode: ObstacleDistributionMode.CorridorFocused,
            corridorFocusWeight: 0.8f,
            corridorWidth: 8.0f,
            enableVariableObstacleHeights: true,
            minObstacleHeight: 1.0f,
            maxObstacleHeight: 6.0f,
            defaultObstacleHeight: 2.5f);

        foreach (Transform child in obstacles)
        {
            float height = child.localScale.y;
            float centerY = child.position.y;
            float bottomY = centerY - (height * 0.5f);

            // In variable-height mode, bottom plane must sit precisely flush at ground level (Y = 0)
            Assert.AreEqual(0f, bottomY, 0.001f, $"Obstacle {child.name} bottom Y ({bottomY:F3}) is not flush with ground plane Y=0!");
            Assert.AreEqual(height * 0.5f, centerY, 0.001f, $"Obstacle {child.name} center Y ({centerY:F3}) does not equal height/2 ({height * 0.5f:F3})!");
        }

        Object.DestroyImmediate(obstacles.gameObject);
    }

    [Test]
    public void VariableHeights_SameSeed_ProducesIdentical3DObstacleGeometry()
    {
        Vector3 start = new Vector3(-10f, 1f, -10f);
        Vector3 target = new Vector3(10f, 1f, 10f);
        Vector2 size = new Vector2(50f, 50f);

        Transform obstaclesA = ProceduralObstacleGenerator.Generate(
            gridOriginObj.transform, size, start, target, obstacleCount: 10, seed: 999,
            distributionMode: ObstacleDistributionMode.CorridorFocused, corridorFocusWeight: 0.7f, corridorWidth: 10.0f,
            enableDynamicObstacles: false, dynamicObstacleCount: 0, dynamicObstacleSpeed: 1.0f,
            dynamicMovementMode: ObstacleMovementMode.Patrol, dynamicLoopMode: PatrolLoopMode.PingPong,
            enableVariableObstacleHeights: true, minObstacleHeight: 1.2f, maxObstacleHeight: 5.0f, defaultObstacleHeight: 2.5f);

        Transform obstaclesB = ProceduralObstacleGenerator.Generate(
            gridOriginObj.transform, size, start, target, obstacleCount: 10, seed: 999,
            distributionMode: ObstacleDistributionMode.CorridorFocused, corridorFocusWeight: 0.7f, corridorWidth: 10.0f,
            enableDynamicObstacles: false, dynamicObstacleCount: 0, dynamicObstacleSpeed: 1.0f,
            dynamicMovementMode: ObstacleMovementMode.Patrol, dynamicLoopMode: PatrolLoopMode.PingPong,
            enableVariableObstacleHeights: true, minObstacleHeight: 1.2f, maxObstacleHeight: 5.0f, defaultObstacleHeight: 2.5f);

        Assert.AreEqual(obstaclesA.childCount, obstaclesB.childCount);

        for (int i = 0; i < obstaclesA.childCount; i++)
        {
            Transform childA = obstaclesA.GetChild(i);
            Transform childB = obstaclesB.GetChild(i);

            // Positions (X, Y, Z) must match identically
            Assert.AreEqual(childA.position.x, childB.position.x, 0.001f);
            Assert.AreEqual(childA.position.y, childB.position.y, 0.001f);
            Assert.AreEqual(childA.position.z, childB.position.z, 0.001f);

            // Dimensions (X, Y, Z scales) must match identically
            Assert.AreEqual(childA.localScale.x, childB.localScale.x, 0.001f);
            Assert.AreEqual(childA.localScale.y, childB.localScale.y, 0.001f);
            Assert.AreEqual(childA.localScale.z, childB.localScale.z, 0.001f);
        }

        Object.DestroyImmediate(obstaclesA.gameObject);
        Object.DestroyImmediate(obstaclesB.gameObject);
    }

    [Test]
    public void ScenarioConfig_DefaultValues_PreserveLegacyModeAndValidAltitudeBounds()
    {
        UAVScenarioConfig config = ScriptableObject.CreateInstance<UAVScenarioConfig>();

        // Default mode must be false to guarantee 100% backward compatibility
        Assert.IsFalse(config.enableVariableObstacleHeights, "Default enableVariableObstacleHeights must be false!");
        Assert.AreEqual(1.0f, config.minObstacleHeight, 0.01f);
        Assert.AreEqual(4.0f, config.maxObstacleHeight, 0.01f);
        Assert.AreEqual(2.0f, config.defaultObstacleHeight, 0.01f);

        // Flight altitude parameters
        Assert.AreEqual(1.0f, config.minFlightAltitude, 0.01f);
        Assert.AreEqual(6.0f, config.maxFlightAltitude, 0.01f);
        Assert.AreEqual(1.0f, config.nominalFlightAltitude, 0.01f);
        Assert.GreaterOrEqual(config.maxFlightAltitude, config.minFlightAltitude);

        Object.DestroyImmediate(config);
    }
}
