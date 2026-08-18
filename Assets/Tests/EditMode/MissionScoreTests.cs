using NUnit.Framework;
using UnityEngine;

[TestFixture]
public class MissionScoreTests
{
    [Test]
    public void MissionScore_FlawlessMission_ProducesPerfect100Score()
    {
        // 30m planned, 30m actual (100% efficiency), 20s flight time at 1.5 m/s, 1 replan, 1 threat, clearance 3.0m
        MissionResult perfectResult = new MissionResult(
            isSuccess: true,
            finalState: MissionState.Completed,
            totalFlightTime: 20.0f,
            totalDistanceTraveled: 30.0f,
            plannedPathDistance: 30.0f,
            totalReplans: 1,
            totalThreatEncounters: 1,
            criticalThreatCount: 1,
            minimumClearanceObserved: 3.0f,
            pathEfficiency: 1.0f);

        MissionScore score = MissionScore.Evaluate(perfectResult, nominalSpeed: 1.5f);

        Assert.AreEqual(100.0f, score.SafetyScore, 0.01f);
        Assert.AreEqual(100.0f, score.NavigationScore, 0.01f);
        Assert.AreEqual(100.0f, score.EfficiencyScore, 0.01f);
        Assert.AreEqual(100.0f, score.ThreatManagementScore, 0.01f);
        Assert.AreEqual(100.0f, score.TimeScore, 0.01f);
        Assert.AreEqual(100.0f, score.OverallScore, 0.01f);
    }

    [Test]
    public void MissionScore_FailedMission_StrictlyCapsOverallScoreToMax20Percent()
    {
        MissionResult failedResult = new MissionResult(
            isSuccess: false,
            finalState: MissionState.Failed,
            totalFlightTime: 10.0f,
            totalDistanceTraveled: 10.0f,
            plannedPathDistance: 30.0f,
            totalReplans: 3,
            totalThreatEncounters: 4,
            criticalThreatCount: 2,
            minimumClearanceObserved: 0.5f,
            pathEfficiency: 0.33f);

        MissionScore score = MissionScore.Evaluate(failedResult, nominalSpeed: 1.5f);

        Assert.LessOrEqual(score.OverallScore, 20.0f);
    }

    [Test]
    public void MissionScore_DivisionByZeroSafety_ClampsScoresGracefully()
    {
        MissionResult zeroResult = new MissionResult(
            isSuccess: true,
            finalState: MissionState.Completed,
            totalFlightTime: 0.0f,
            totalDistanceTraveled: 0.0f,
            plannedPathDistance: 0.0f,
            totalReplans: 0,
            totalThreatEncounters: 0,
            criticalThreatCount: 0,
            minimumClearanceObserved: float.PositiveInfinity,
            pathEfficiency: 0.0f);

        MissionScore score = MissionScore.Evaluate(zeroResult, nominalSpeed: 1.5f);

        Assert.IsFalse(float.IsNaN(score.OverallScore));
        Assert.IsFalse(float.IsInfinity(score.OverallScore));
        Assert.GreaterOrEqual(score.OverallScore, 0f);
        Assert.LessOrEqual(score.OverallScore, 100f);
    }

    [Test]
    public void MissionScore_ExactFormulaWeighting_MatchesSpecification()
    {
        // Safety = 50, Nav = 100, Eff = 50, Threat = 100, Time = 50
        // Expected Overall = (0.25 * 50) + (0.30 * 100) + (0.20 * 50) + (0.15 * 100) + (0.10 * 50)
        //                  = 12.5 + 30.0 + 10.0 + 15.0 + 5.0 = 72.5
        MissionResult customResult = new MissionResult(
            isSuccess: true,
            finalState: MissionState.Completed,
            totalFlightTime: 23.5294f, // timeRatio = (30 / 1.5) / 23.5294 = 20 / 23.5294 = 0.85 -> TimeScore = 50
            totalDistanceTraveled: 37.5f, // pathEfficiency = 30 / 37.5 = 0.80 -> EfficiencyScore = 50
            plannedPathDistance: 30.0f,
            totalReplans: 1, // NavigationScore = 100
            totalThreatEncounters: 1, // ThreatScore = 100
            criticalThreatCount: 1,
            minimumClearanceObserved: 1.0f, // SafetyScore = 50
            pathEfficiency: 0.80f);

        MissionScore score = MissionScore.Evaluate(customResult, nominalSpeed: 1.5f);

        Assert.AreEqual(50.0f, score.SafetyScore, 0.1f);
        Assert.AreEqual(100.0f, score.NavigationScore, 0.1f);
        Assert.AreEqual(50.0f, score.EfficiencyScore, 0.1f);
        Assert.AreEqual(100.0f, score.ThreatManagementScore, 0.1f);
        Assert.AreEqual(50.0f, score.TimeScore, 0.1f);
        Assert.AreEqual(72.5f, score.OverallScore, 0.2f);
    }
}
