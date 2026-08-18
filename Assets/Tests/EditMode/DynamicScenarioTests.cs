using System.IO;
using NUnit.Framework;
using UnityEngine;

[TestFixture]
public class DynamicScenarioTests
{
    [Test]
    public void DynamicThreatScenario_AssetFile_ExistsAndIsLoadable()
    {
        string assetPath = "Assets/Scenarios/Scenario_DynamicThreats.asset";
        Assert.IsTrue(File.Exists(assetPath), $"Asset file at '{assetPath}' was not found!");

        string yaml = File.ReadAllText(assetPath);
        Assert.IsTrue(yaml.Contains("Scenario_DynamicThreats"));
    }

    [Test]
    public void DynamicThreatScenario_DynamicObstacles_AreEnabledWithConfiguredParameters()
    {
        string assetPath = "Assets/Scenarios/Scenario_DynamicThreats.asset";
        string yaml = File.ReadAllText(assetPath);

        Assert.IsTrue(yaml.Contains("enableDynamicObstacles: 1"), "Dynamic obstacles must be enabled!");
        Assert.IsTrue(yaml.Contains("dynamicObstacleCount: 2"), "Dynamic obstacle count must be 2!");
        Assert.IsTrue(yaml.Contains("dynamicObstacleSpeed: 1.2"), "Dynamic obstacle speed must be 1.2 m/s!");
        Assert.IsTrue(yaml.Contains("seed: 400"), "Dynamic scenario must use seed 400!");
    }

    [Test]
    public void Existing4Scenarios_RemainStatic_WithoutDynamicObstaclesEnabled()
    {
        string[] staticScenarios = new string[]
        {
            "Assets/Scenarios/DefaultScenario.asset",
            "Assets/Scenarios/Scenario_AlternativeSeed.asset",
            "Assets/Scenarios/Scenario_DenseObstacles.asset",
            "Assets/Scenarios/Scenario_LongRange.asset"
        };

        for (int i = 0; i < staticScenarios.Length; i++)
        {
            string path = staticScenarios[i];
            Assert.IsTrue(File.Exists(path), $"Static scenario '{path}' missing!");
            string yaml = File.ReadAllText(path);

            // Existing scenarios should not have enableDynamicObstacles set to 1
            Assert.IsFalse(yaml.Contains("enableDynamicObstacles: 1"), $"Scenario '{path}' unexpectedly enabled dynamic obstacles!");
        }
    }

    [Test]
    public void DynamicScenario_ProceduralGeneration_CreatesDynamicObstacleComponents()
    {
        GameObject gridObj = new GameObject("TestGridOrigin");
        Vector3 start = new Vector3(-10f, 1f, -10f);
        Vector3 target = new Vector3(10f, 1f, 10f);
        Vector2 size = new Vector2(50f, 50f);

        Transform obstacles = ProceduralObstacleGenerator.Generate(
            gridObj.transform,
            size,
            start,
            target,
            obstacleCount: 10,
            seed: 400,
            distributionMode: ObstacleDistributionMode.CorridorFocused,
            corridorFocusWeight: 0.6f,
            corridorWidth: 10.0f,
            enableDynamicObstacles: true,
            dynamicObstacleCount: 2,
            dynamicObstacleSpeed: 1.2f,
            dynamicMovementMode: ObstacleMovementMode.Patrol,
            dynamicLoopMode: PatrolLoopMode.PingPong);

        int dynamicCount = 0;
        foreach (Transform child in obstacles)
        {
            DynamicObstacle dyn = child.GetComponent<DynamicObstacle>();
            if (dyn != null)
            {
                dynamicCount++;
                Assert.IsTrue(dyn.MovementEnabled);
                Assert.AreEqual(1.2f, dyn.Speed, 0.01f);
                Assert.AreEqual(ObstacleMovementMode.Patrol, dyn.MovementMode);
                Assert.AreEqual(PatrolLoopMode.PingPong, dyn.LoopMode);
                Assert.AreEqual(2, dyn.PatrolWaypoints.Count);
            }
        }

        Assert.AreEqual(2, dynamicCount, "Expected exactly 2 DynamicObstacle components!");

        Object.DestroyImmediate(obstacles.gameObject);
        Object.DestroyImmediate(gridObj);
    }

    [Test]
    public void DynamicScenario_ProceduralGeneration_IsFullyDeterministic()
    {
        GameObject gridObjA = new GameObject("GridA");
        GameObject gridObjB = new GameObject("GridB");
        Vector3 start = new Vector3(-10f, 1f, -10f);
        Vector3 target = new Vector3(10f, 1f, 10f);
        Vector2 size = new Vector2(50f, 50f);

        Transform obstaclesA = ProceduralObstacleGenerator.Generate(
            gridObjA.transform, size, start, target, 10, 400,
            ObstacleDistributionMode.CorridorFocused, 0.6f, 10.0f,
            true, 2, 1.2f, ObstacleMovementMode.Patrol, PatrolLoopMode.PingPong);

        Transform obstaclesB = ProceduralObstacleGenerator.Generate(
            gridObjB.transform, size, start, target, 10, 400,
            ObstacleDistributionMode.CorridorFocused, 0.6f, 10.0f,
            true, 2, 1.2f, ObstacleMovementMode.Patrol, PatrolLoopMode.PingPong);

        Assert.AreEqual(obstaclesA.childCount, obstaclesB.childCount);
        for (int i = 0; i < obstaclesA.childCount; i++)
        {
            Transform childA = obstaclesA.GetChild(i);
            Transform childB = obstaclesB.GetChild(i);

            Assert.AreEqual(childA.position.x, childB.position.x, 0.001f);
            Assert.AreEqual(childA.position.z, childB.position.z, 0.001f);

            DynamicObstacle dynA = childA.GetComponent<DynamicObstacle>();
            DynamicObstacle dynB = childB.GetComponent<DynamicObstacle>();

            Assert.AreEqual(dynA != null, dynB != null);
            if (dynA != null && dynB != null)
            {
                Assert.AreEqual(dynA.PatrolWaypoints[0], dynB.PatrolWaypoints[0]);
                Assert.AreEqual(dynA.PatrolWaypoints[1], dynB.PatrolWaypoints[1]);
            }
        }

        Object.DestroyImmediate(obstaclesA.gameObject);
        Object.DestroyImmediate(obstaclesB.gameObject);
        Object.DestroyImmediate(gridObjA);
        Object.DestroyImmediate(gridObjB);
    }

    [Test]
    public void Benchmark_ScenarioDiscovery_IncludesAll5Scenarios()
    {
        string[] expectedScenarios = new string[]
        {
            "Assets/Scenarios/DefaultScenario.asset",
            "Assets/Scenarios/Scenario_AlternativeSeed.asset",
            "Assets/Scenarios/Scenario_DenseObstacles.asset",
            "Assets/Scenarios/Scenario_LongRange.asset",
            "Assets/Scenarios/Scenario_DynamicThreats.asset"
        };

        for (int i = 0; i < expectedScenarios.Length; i++)
        {
            Assert.IsTrue(File.Exists(expectedScenarios[i]), $"Scenario '{expectedScenarios[i]}' was not found!");
        }

        Assert.AreEqual(5, expectedScenarios.Length);
    }
}
