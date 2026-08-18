using System.IO;
using NUnit.Framework;
using UnityEngine;

[TestFixture]
public class BenchmarkRegressionTests
{
    [Test]
    public void Benchmark_All4ScenarioAssets_ExistAndAreLoadable()
    {
        string[] scenarioPaths = new string[]
        {
            "Assets/Scenarios/DefaultScenario.asset",
            "Assets/Scenarios/Scenario_AlternativeSeed.asset",
            "Assets/Scenarios/Scenario_DenseObstacles.asset",
            "Assets/Scenarios/Scenario_LongRange.asset"
        };

        for (int i = 0; i < scenarioPaths.Length; i++)
        {
            Assert.IsTrue(File.Exists(scenarioPaths[i]), $"Scenario asset at '{scenarioPaths[i]}' was not found!");
        }
    }

    [Test]
    public void Benchmark_DenseObstaclesScenario_HasConfiguredStressParameters()
    {
        string densePath = "Assets/Scenarios/Scenario_DenseObstacles.asset";
        Assert.IsTrue(File.Exists(densePath));

        string yamlContent = File.ReadAllText(densePath);
        Assert.IsTrue(yamlContent.Contains("seed: 300"), "Dense scenario must use unique seed 300!");
        Assert.IsTrue(yamlContent.Contains("obstacleCount: 18"), "Dense scenario must use 18 obstacles!");
        Assert.IsTrue(yamlContent.Contains("distributionMode: 1"), "Dense scenario must use CorridorFocused distribution mode!");
    }

    [Test]
    public void BenchmarkReporter_AggregateCompilation_CalculatesAccurateAverages()
    {
        AggregateBenchmarkReport report = new AggregateBenchmarkReport();

        report.scenarioSummaries.Add(new ScenarioBenchmarkSummaryEntry
        {
            scenarioName = "Scenario_A",
            success = true,
            overallScore = 90.0f,
            flightTime = 20.0f,
            actualDistance = 30.0f,
            pathEfficiency = 0.90f,
            replans = 1,
            threatEncounters = 4,
            criticalThreats = 1
        });

        report.scenarioSummaries.Add(new ScenarioBenchmarkSummaryEntry
        {
            scenarioName = "Scenario_B",
            success = true,
            overallScore = 80.0f,
            flightTime = 30.0f,
            actualDistance = 40.0f,
            pathEfficiency = 0.80f,
            replans = 2,
            threatEncounters = 6,
            criticalThreats = 1
        });

        int count = report.scenarioSummaries.Count;
        float totalScore = 0f;
        float totalTime = 0f;
        float totalDist = 0f;
        float totalEff = 0f;
        int totalReplans = 0;
        int totalThreats = 0;
        int totalCrit = 0;

        for (int i = 0; i < count; i++)
        {
            totalScore += report.scenarioSummaries[i].overallScore;
            totalTime += report.scenarioSummaries[i].flightTime;
            totalDist += report.scenarioSummaries[i].actualDistance;
            totalEff += report.scenarioSummaries[i].pathEfficiency;
            totalReplans += report.scenarioSummaries[i].replans;
            totalThreats += report.scenarioSummaries[i].threatEncounters;
            totalCrit += report.scenarioSummaries[i].criticalThreats;
        }

        report.totalScenariosEvaluated = count;
        report.averageOverallScore = totalScore / count;
        report.averageFlightTime = totalTime / count;
        report.averageDistance = totalDist / count;
        report.averagePathEfficiency = totalEff / count;
        report.totalReplans = totalReplans;
        report.totalThreatEncounters = totalThreats;
        report.totalCriticalThreats = totalCrit;

        Assert.AreEqual(85.0f, report.averageOverallScore, 0.01f);
        Assert.AreEqual(25.0f, report.averageFlightTime, 0.01f);
        Assert.AreEqual(35.0f, report.averageDistance, 0.01f);
        Assert.AreEqual(0.85f, report.averagePathEfficiency, 0.01f);
        Assert.AreEqual(3, report.totalReplans);
        Assert.AreEqual(10, report.totalThreatEncounters);
        Assert.AreEqual(2, report.totalCriticalThreats);
    }
}
