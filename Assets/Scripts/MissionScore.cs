using System;
using UnityEngine;

/// <summary>
/// Deterministic and transparent mission performance evaluation score.
/// Computes normalized sub-scores (0-100) across Safety, Path Efficiency,
/// Navigation & Replanning, Threat Management, and Flight Duration.
/// </summary>
[Serializable]
public struct MissionScore
{
    // Normalized Sub-Scores (0 - 100)
    public float OverallScore;
    public float SafetyScore;
    public float EfficiencyScore;
    public float NavigationScore;
    public float ThreatManagementScore;
    public float TimeScore;

    public MissionScore(
        float overall,
        float safety,
        float efficiency,
        float navigation,
        float threatManagement,
        float time)
    {
        OverallScore = Mathf.Clamp(overall, 0f, 100f);
        SafetyScore = Mathf.Clamp(safety, 0f, 100f);
        EfficiencyScore = Mathf.Clamp(efficiency, 0f, 100f);
        NavigationScore = Mathf.Clamp(navigation, 0f, 100f);
        ThreatManagementScore = Mathf.Clamp(threatManagement, 0f, 100f);
        TimeScore = Mathf.Clamp(time, 0f, 100f);
    }

    /// <summary>
    /// Evaluates a completed MissionResult and computes the multi-criteria MissionScore.
    /// </summary>
    /// <param name="result">The telemetry result snapshot from MissionManager.</param>
    /// <param name="nominalSpeed">The UAV cruise speed configured for the mission (m/s).</param>
    /// <returns>Normalized MissionScore struct.</returns>
    public static MissionScore Evaluate(MissionResult result, float nominalSpeed = 1.5f)
    {
        // -------------------------------------------------------------
        // 1. SAFETY SCORE (0 - 100)
        // Baseline safety envelope is 1.0m. Clearance >= 2.0m yields 100.
        // Clearance in [1.0m, 2.0m) scales from 50 to 100.
        // Clearance < 1.0m scales from 0 to 50.
        // -------------------------------------------------------------
        float safetyScore;
        if (float.IsPositiveInfinity(result.MinimumClearanceObserved) || result.MinimumClearanceObserved < 0f)
        {
            safetyScore = 100f; // No obstacles encountered
        }
        else if (result.MinimumClearanceObserved >= 2.0f)
        {
            safetyScore = 100f;
        }
        else if (result.MinimumClearanceObserved >= 1.0f)
        {
            safetyScore = 50f + 50f * ((result.MinimumClearanceObserved - 1.0f) / 1.0f);
        }
        else
        {
            safetyScore = 50f * Mathf.Max(0f, result.MinimumClearanceObserved / 1.0f);
        }

        // Penalize excessive critical threat entries if any (> 2)
        if (result.CriticalThreatCount > 2)
        {
            safetyScore -= (result.CriticalThreatCount - 2) * 10f;
        }
        safetyScore = Mathf.Clamp(safetyScore, 0f, 100f);

        // -------------------------------------------------------------
        // 2. EFFICIENCY SCORE (0 - 100)
        // Measures Path Efficiency = PlannedPathDistance / TotalDistanceTraveled.
        // Ideal straight trajectory gives 1.0 (100 pts).
        // Efficiency in [0.80, 1.00] maps to [50, 100] pts.
        // Efficiency < 0.80 maps to [0, 50] pts.
        // -------------------------------------------------------------
        float efficiencyScore;
        float eff = result.PathEfficiency;
        if (float.IsNaN(eff) || float.IsInfinity(eff) || eff <= 0f)
        {
            efficiencyScore = 0f;
        }
        else if (eff >= 1.0f)
        {
            efficiencyScore = 100f;
        }
        else if (eff >= 0.80f)
        {
            efficiencyScore = 50f + 50f * ((eff - 0.80f) / 0.20f);
        }
        else
        {
            efficiencyScore = 50f * Mathf.Max(0f, eff / 0.80f);
        }
        efficiencyScore = Mathf.Clamp(efficiencyScore, 0f, 100f);

        // -------------------------------------------------------------
        // 3. NAVIGATION SCORE (0 - 100)
        // Mission completion grants base 70 pts.
        // Optimal dynamic replanning (1-2 replans) adds up to 30 pts.
        // -------------------------------------------------------------
        float navigationScore = 0f;
        if (result.IsSuccess && result.FinalState == MissionState.Completed)
        {
            navigationScore = 70f;
            if (result.TotalReplans <= 1)
            {
                navigationScore += 30f; // Perfect clean route or single decisive replan
            }
            else if (result.TotalReplans == 2)
            {
                navigationScore += 25f; // Two replans (minor secondary adjustment)
            }
            else
            {
                navigationScore += Mathf.Max(0f, 30f - (result.TotalReplans - 1) * 10f);
            }
        }
        navigationScore = Mathf.Clamp(navigationScore, 0f, 100f);

        // -------------------------------------------------------------
        // 4. THREAT MANAGEMENT SCORE (0 - 100)
        // Measures safe resolution of all detected threat encounters.
        // -------------------------------------------------------------
        float threatScore = 0f;
        if (result.IsSuccess)
        {
            threatScore = 100f;
            // Minor deduction for multiple distinct threat encounter episodes
            if (result.TotalThreatEncounters > 1)
            {
                threatScore -= (result.TotalThreatEncounters - 1) * 5f;
            }
            if (result.CriticalThreatCount > 1)
            {
                threatScore -= (result.CriticalThreatCount - 1) * 5f;
            }
        }
        threatScore = Mathf.Clamp(threatScore, 0f, 100f);

        // -------------------------------------------------------------
        // 5. TIME SCORE (0 - 100)
        // Compares TotalFlightTime against nominal expected time:
        // NominalTime = PlannedPathDistance / nominalSpeed.
        // TimeRatio = NominalTime / TotalFlightTime.
        // TimeRatio in [0.85, 1.00] maps to [50, 100] pts.
        // -------------------------------------------------------------
        float timeScore = 0f;
        if (result.TotalFlightTime > 0.001f && result.PlannedPathDistance > 0.001f && nominalSpeed > 0.01f)
        {
            float nominalTime = result.PlannedPathDistance / nominalSpeed;
            float timeRatio = nominalTime / result.TotalFlightTime;

            if (timeRatio >= 1.0f)
            {
                timeScore = 100f;
            }
            else if (timeRatio >= 0.85f)
            {
                timeScore = 50f + 50f * ((timeRatio - 0.85f) / 0.15f);
            }
            else
            {
                timeScore = 50f * Mathf.Max(0f, timeRatio / 0.85f);
            }
        }
        timeScore = Mathf.Clamp(timeScore, 0f, 100f);

        // -------------------------------------------------------------
        // 6. OVERALL COMPOSITE SCORE (0 - 100)
        // Weighted multi-criteria index:
        // 25% Safety + 30% Navigation + 20% Efficiency + 15% Threat + 10% Time
        // Failed mission overall score is strictly capped at 20%.
        // -------------------------------------------------------------
        float overallScore = (0.25f * safetyScore) +
                             (0.30f * navigationScore) +
                             (0.20f * efficiencyScore) +
                             (0.15f * threatScore) +
                             (0.10f * timeScore);

        if (!result.IsSuccess)
        {
            overallScore *= 0.20f;
        }

        overallScore = Mathf.Clamp(overallScore, 0f, 100f);

        return new MissionScore(
            overallScore,
            safetyScore,
            efficiencyScore,
            navigationScore,
            threatScore,
            timeScore);
    }

    public override string ToString()
    {
        return $"Overall: {OverallScore:F1} | Safety: {SafetyScore:F1} | Eff: {EfficiencyScore:F1} | " +
               $"Nav: {NavigationScore:F1} | Threat: {ThreatManagementScore:F1} | Time: {TimeScore:F1}";
    }
}
