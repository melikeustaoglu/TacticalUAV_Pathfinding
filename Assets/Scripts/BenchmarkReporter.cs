using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

/// <summary>
/// Serializable data container for structured mission benchmark exports.
/// Completely compatible with UnityEngine.JsonUtility.
/// </summary>
[Serializable]
public class MissionBenchmarkReport
{
    [Header("Scenario Profile")]
    public string scenarioName;
    public string exportTimestamp;
    public Vector3 startPosition;
    public Vector3 targetPosition;
    public int obstacleCount;
    public int seed;
    public float cruiseSpeed;
    public float sensorRange;

    [Header("Mission Outcome")]
    public bool success;
    public string finalState;

    [Header("Flight Telemetry")]
    public float flightTime;
    public float actualDistance;
    public float plannedDistance;
    public float pathEfficiency;

    [Header("Tactical Counters")]
    public int replans;
    public int speedPacingResolutions;
    public int verticalEvasions;
    public int spatialDetours;
    public int peakSimultaneousThreats;
    public int threatEncounters;
    public int criticalThreats;
    public float minimumClearance;
    public float peakAltitudeReached;
    public float maxFlightAltitude;
    public float nominalFlightAltitude;

    [Header("Tactical Decision Summary")]
    public string dominantTacticalDecision;
    public int voPacingDecisions;
    public int verticalStepClimbs;
    public int verticalCeilingRejections;
    public int verticalClimbTimeRejections;
    public int verticalMultiThreatRejections;
    public int safeHoldDecisions;

    [Header("Evaluation Scores")]
    public float overallScore;
    public float safetyScore;
    public float navigationScore;
    public float efficiencyScore;
    public float threatManagementScore;
    public float timeScore;

    [Header("Event Timeline")]
    public List<MissionEventRecord> timeline = new List<MissionEventRecord>();
}

/// <summary>
/// Passive Benchmark Reporter and Mission JSON Exporter.
/// Aggregates final telemetry from MissionManager, performance scores from MissionScore,
/// and chronological events from MissionEventLogger upon mission completion, exporting
/// a structured JSON report to Application.persistentDataPath.
/// </summary>
public class BenchmarkReporter : MonoBehaviour
{
    [Header("Export Configuration")]
    [SerializeField] private bool autoExportOnComplete = true;
    [SerializeField] private bool logSummaryToConsole = true;

    private MissionManager missionManager;
    private MissionEventLogger eventLogger;
    private PathFollower pathFollower;
    private UAVPerception perception;
    private ReplanningController replanningController;

    private float peakAltitudeReached = 0f;

    public MissionBenchmarkReport LastReport { get; private set; }
    public string LastReportJson { get; private set; }
    public string LastExportPath { get; private set; }

    private void Awake()
    {
        missionManager = GetComponent<MissionManager>() ?? FindFirstObjectByType<MissionManager>();
        eventLogger = GetComponent<MissionEventLogger>() ?? FindFirstObjectByType<MissionEventLogger>();
        pathFollower = GetComponent<PathFollower>() ?? FindFirstObjectByType<PathFollower>();
        perception = GetComponent<UAVPerception>() ?? FindFirstObjectByType<UAVPerception>();
        replanningController = GetComponent<ReplanningController>() ?? FindFirstObjectByType<ReplanningController>();
    }

    private void Update()
    {
        float currentY = pathFollower != null ? pathFollower.transform.position.y : transform.position.y;
        if (currentY > peakAltitudeReached)
        {
            peakAltitudeReached = currentY;
        }
    }

    private void OnEnable()
    {
        if (missionManager == null)
        {
            missionManager = GetComponent<MissionManager>() ?? FindFirstObjectByType<MissionManager>();
        }

        if (missionManager != null)
        {
            missionManager.OnMissionCompleted += HandleMissionCompleted;
        }
    }

    private void OnDisable()
    {
        if (missionManager != null)
        {
            missionManager.OnMissionCompleted -= HandleMissionCompleted;
        }
    }

    private bool hasExported = false;

    private void HandleMissionCompleted(MissionResult result)
    {
        if (!autoExportOnComplete || hasExported)
            return;

        hasExported = true;
        GenerateAndExportReport(result);
    }

    /// <summary>
    /// Constructs the structured benchmark report and serializes it to JSON.
    /// </summary>
    /// <param name="result">Authoritative final mission result from MissionManager.</param>
    /// <returns>The generated MissionBenchmarkReport instance.</returns>
    public MissionBenchmarkReport GenerateAndExportReport(MissionResult result)
    {
        PathfindingRuntimeSetup setup = FindFirstObjectByType<PathfindingRuntimeSetup>();
        UAVScenarioConfig cfg = setup != null ? setup.ScenarioConfig : null;
        if (replanningController == null)
        {
            replanningController = GetComponent<ReplanningController>() ?? FindFirstObjectByType<ReplanningController>();
        }

        MissionBenchmarkReport report = new MissionBenchmarkReport();

        // 1. Scenario Identification
        report.scenarioName = cfg != null ? cfg.name : "DefaultScenario";
        report.exportTimestamp = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss");
        report.startPosition = cfg != null ? cfg.startPosition : GameManagerBootstrapper.DefaultStartPosition;
        report.targetPosition = cfg != null ? cfg.targetPosition : GameManagerBootstrapper.DefaultTargetPosition;
        report.obstacleCount = cfg != null ? cfg.obstacleCount : ProceduralObstacleGenerator.DefaultObstacleCount;
        report.seed = cfg != null ? cfg.seed : ProceduralObstacleGenerator.DefaultSeed;
        report.cruiseSpeed = cfg != null ? cfg.uavMoveSpeed : (pathFollower != null ? pathFollower.MoveSpeed : 1.5f);
        report.sensorRange = cfg != null ? cfg.sensorDetectionRange : (perception != null ? perception.DetectionRange : 10f);

        // 2. Mission Outcome & Authoritative Telemetry
        report.success = result.IsSuccess;
        report.finalState = result.FinalState.ToString();
        report.flightTime = result.TotalFlightTime;
        report.actualDistance = result.TotalDistanceTraveled;
        report.plannedDistance = result.PlannedPathDistance;
        report.pathEfficiency = result.PathEfficiency;
        report.replans = result.TotalReplans;
        report.speedPacingResolutions = replanningController != null ? replanningController.SpeedPacingCount : 0;
        report.verticalEvasions = replanningController != null ? replanningController.VerticalEvasionCount : 0;
        report.spatialDetours = replanningController != null ? replanningController.SpatialReplanCount : result.TotalReplans;
        report.peakSimultaneousThreats = replanningController != null ? replanningController.PeakSimultaneousThreats : 0;
        report.threatEncounters = result.TotalThreatEncounters;
        report.criticalThreats = result.CriticalThreatCount;
        report.minimumClearance = result.MinimumClearanceObserved;
        report.peakAltitudeReached = Mathf.Max(peakAltitudeReached, pathFollower != null ? pathFollower.transform.position.y : transform.position.y);
        report.maxFlightAltitude = pathFollower != null ? pathFollower.MaxFlightAltitude : (cfg != null ? cfg.maxFlightAltitude : 6.0f);
        report.nominalFlightAltitude = cfg != null ? cfg.nominalFlightAltitude : (replanningController != null ? replanningController.NominalAltitude : 1.0f);

        // 3. Tactical Decision Summary
        report.dominantTacticalDecision = replanningController != null ? replanningController.LatestDecisionReason.ToString() : "None";
        report.voPacingDecisions = replanningController != null ? replanningController.VoPacingDecisions : 0;
        report.verticalStepClimbs = replanningController != null ? replanningController.VerticalStepClimbs : 0;
        report.verticalCeilingRejections = replanningController != null ? replanningController.VerticalCeilingRejections : 0;
        report.verticalClimbTimeRejections = replanningController != null ? replanningController.VerticalClimbTimeRejections : 0;
        report.verticalMultiThreatRejections = replanningController != null ? replanningController.VerticalMultiThreatRejections : 0;
        report.spatialDetours = replanningController != null ? replanningController.SpatialReplanCount : result.TotalReplans;
        report.safeHoldDecisions = replanningController != null ? replanningController.SafeHoldDecisions : 0;

        // 4. Evaluation Scores
        float nominalSpeed = report.cruiseSpeed;
        MissionScore score = missionManager != null && missionManager.Score.HasValue
            ? missionManager.Score.Value
            : MissionScore.Evaluate(result, nominalSpeed);

        report.overallScore = score.OverallScore;
        report.safetyScore = score.SafetyScore;
        report.navigationScore = score.NavigationScore;
        report.efficiencyScore = score.EfficiencyScore;
        report.threatManagementScore = score.ThreatManagementScore;
        report.timeScore = score.TimeScore;

        // 4. Chronological Event Timeline
        if (eventLogger == null)
        {
            eventLogger = GetComponent<MissionEventLogger>() ?? FindFirstObjectByType<MissionEventLogger>();
        }

        if (eventLogger != null && eventLogger.Events != null)
        {
            report.timeline = new List<MissionEventRecord>(eventLogger.Events);
        }

        // 5. JSON Serialization
        string json = JsonUtility.ToJson(report, true);
        LastReport = report;
        LastReportJson = json;

        // 6. Safe Persistent File Export
        try
        {
            string exportDir = Path.Combine(Application.persistentDataPath, "MissionReports");
            if (!Directory.Exists(exportDir))
            {
                Directory.CreateDirectory(exportDir);
            }

            string safeName = string.IsNullOrEmpty(report.scenarioName) ? "Scenario" : report.scenarioName;
            string fileName = $"mission_report_{safeName}_{DateTime.Now:yyyyMMdd_HHmmss}.json";
            string filePath = Path.Combine(exportDir, fileName);

            File.WriteAllText(filePath, json);
            LastExportPath = filePath;

            if (logSummaryToConsole)
            {
                string clearanceStr = float.IsPositiveInfinity(report.minimumClearance)
                    ? "N/A (Clear)"
                    : $"{report.minimumClearance:F2}m";

                Debug.Log(
                    $"[BenchmarkReporter] Mission Report Exported\n" +
                    $"Scenario={report.scenarioName}\n" +
                    $"Success={report.success}\n" +
                    $"State={report.finalState}\n" +
                    $"Score={report.overallScore:F1}\n" +
                    $"FlightTime={report.flightTime:F2}s\n" +
                    $"Distance={report.actualDistance:F2}m\n" +
                    $"Efficiency={report.pathEfficiency * 100f:F1}%\n" +
                    $"Replans={report.replans}\n" +
                    $"Threats={report.threatEncounters}\n" +
                    $"MinClearance={clearanceStr}\n" +
                    $"Events={report.timeline.Count}\n" +
                    $"File={filePath}");
            }

            // Automatically compile and update the multi-scenario aggregate benchmark summary
            GenerateAndExportAggregateSummary(exportDir);
        }
        catch (Exception ex)
        {
            Debug.LogWarning($"[BenchmarkReporter] File export warning: {ex.Message}");
        }

        return report;
    }

    /// <summary>
    /// Scans all exported individual mission reports in the designated directory, compiles
    /// a consolidated multi-scenario comparison matrix, and exports both benchmark_summary.json
    /// and benchmark_summary.md.
    /// </summary>
    /// <param name="reportsDirectory">Target directory containing individual JSON reports. Defaults to MissionReports.</param>
    /// <returns>Compiled AggregateBenchmarkReport instance.</returns>
    public static AggregateBenchmarkReport GenerateAndExportAggregateSummary(string reportsDirectory = null)
    {
        if (string.IsNullOrEmpty(reportsDirectory))
        {
            reportsDirectory = Path.Combine(Application.persistentDataPath, "MissionReports");
        }

        AggregateBenchmarkReport aggReport = new AggregateBenchmarkReport();
        aggReport.exportTimestamp = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss");

        try
        {
            if (!Directory.Exists(reportsDirectory))
            {
                Directory.CreateDirectory(reportsDirectory);
                return aggReport;
            }

            string[] reportFiles = Directory.GetFiles(reportsDirectory, "mission_report_*.json");
            if (reportFiles == null || reportFiles.Length == 0)
            {
                return aggReport;
            }

            // Sort files by LastWriteTime descending to select the latest valid report per scenario
            Array.Sort(reportFiles, (a, b) => File.GetLastWriteTime(b).CompareTo(File.GetLastWriteTime(a)));

            Dictionary<string, ScenarioBenchmarkSummaryEntry> scenarioMap = new Dictionary<string, ScenarioBenchmarkSummaryEntry>(StringComparer.OrdinalIgnoreCase);

            for (int i = 0; i < reportFiles.Length; i++)
            {
                string filePath = reportFiles[i];
                try
                {
                    string jsonContent = File.ReadAllText(filePath);
                    if (string.IsNullOrEmpty(jsonContent))
                        continue;

                    MissionBenchmarkReport singleReport = JsonUtility.FromJson<MissionBenchmarkReport>(jsonContent);
                    if (singleReport == null || string.IsNullOrEmpty(singleReport.scenarioName))
                        continue;

                    // Keep the latest report per distinct scenario name
                    if (!scenarioMap.ContainsKey(singleReport.scenarioName))
                    {
                        ScenarioBenchmarkSummaryEntry entry = new ScenarioBenchmarkSummaryEntry
                        {
                            scenarioName = singleReport.scenarioName,
                            success = singleReport.success,
                            finalState = singleReport.finalState,
                            obstacleCount = singleReport.obstacleCount,
                            seed = singleReport.seed,
                            cruiseSpeed = singleReport.cruiseSpeed,
                            sensorRange = singleReport.sensorRange,
                            overallScore = singleReport.overallScore,
                            safetyScore = singleReport.safetyScore,
                            navigationScore = singleReport.navigationScore,
                            efficiencyScore = singleReport.efficiencyScore,
                            threatManagementScore = singleReport.threatManagementScore,
                            timeScore = singleReport.timeScore,
                            flightTime = singleReport.flightTime,
                            actualDistance = singleReport.actualDistance,
                            plannedDistance = singleReport.plannedDistance,
                            pathEfficiency = singleReport.pathEfficiency,
                            replans = singleReport.replans,
                            threatEncounters = singleReport.threatEncounters,
                            criticalThreats = singleReport.criticalThreats,
                            minimumClearance = singleReport.minimumClearance,
                            eventCount = singleReport.timeline != null ? singleReport.timeline.Count : 0,
                            sourceReportFile = Path.GetFileName(filePath)
                        };

                        scenarioMap[singleReport.scenarioName] = entry;
                    }
                }
                catch (Exception readEx)
                {
                    // Non-fatal: skip corrupted or partially written single report
                    Debug.LogWarning($"[BenchmarkReporter] Aggregate parse skipped file '{Path.GetFileName(filePath)}': {readEx.Message}");
                }
            }

            foreach (var kvp in scenarioMap)
            {
                aggReport.scenarioSummaries.Add(kvp.Value);
            }

            // Calculate aggregate statistics
            int count = aggReport.scenarioSummaries.Count;
            aggReport.totalScenariosEvaluated = count;

            if (count > 0)
            {
                float totalScore = 0f;
                float totalTime = 0f;
                float totalDist = 0f;
                float totalEff = 0f;
                int totalSucc = 0;
                int totalRep = 0;
                int totalThreats = 0;
                int totalCrit = 0;

                for (int i = 0; i < count; i++)
                {
                    var s = aggReport.scenarioSummaries[i];
                    if (s.success) totalSucc++;
                    totalScore += s.overallScore;
                    totalTime += s.flightTime;
                    totalDist += s.actualDistance;
                    totalEff += s.pathEfficiency;
                    totalRep += s.replans;
                    totalThreats += s.threatEncounters;
                    totalCrit += s.criticalThreats;
                }

                aggReport.successfulMissions = totalSucc;
                aggReport.averageOverallScore = totalScore / count;
                aggReport.averageFlightTime = totalTime / count;
                aggReport.averageDistance = totalDist / count;
                aggReport.averagePathEfficiency = totalEff / count;
                aggReport.totalReplans = totalRep;
                aggReport.totalThreatEncounters = totalThreats;
                aggReport.totalCriticalThreats = totalCrit;
            }

            // 1. Export aggregate JSON
            string aggregateJson = JsonUtility.ToJson(aggReport, true);
            string aggregateJsonPath = Path.Combine(reportsDirectory, "benchmark_summary.json");
            File.WriteAllText(aggregateJsonPath, aggregateJson);

            // 2. Export human-readable Markdown summary table
            string markdown = GenerateMarkdownSummary(aggReport);
            string markdownPath = Path.Combine(reportsDirectory, "benchmark_summary.md");
            File.WriteAllText(markdownPath, markdown);
        }
        catch (Exception ex)
        {
            Debug.LogWarning($"[BenchmarkReporter] Aggregate summary export warning: {ex.Message}");
        }

        return aggReport;
    }

    private static string GenerateMarkdownSummary(AggregateBenchmarkReport report)
    {
        var sb = new System.Text.StringBuilder(2048);
        sb.AppendLine("# Tactical UAV Pathfinding — Multi-Scenario Benchmark Summary");
        sb.AppendLine($"**Generated**: {report.exportTimestamp} | **Evaluation Engine**: Unity 2022.3.62f3 LTS");
        sb.AppendLine();
        sb.AppendLine("### Executive Comparison Matrix");
        sb.AppendLine();
        sb.AppendLine("| Scenario | Status | State | Score | Time (s) | Distance (m) | Planned (m) | Efficiency | Replans | Clearance (m) | Threats | Crit | Safety | Nav | Eff | Threat | Time |");
        sb.AppendLine("|:---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|");

        for (int i = 0; i < report.scenarioSummaries.Count; i++)
        {
            var s = report.scenarioSummaries[i];
            string status = s.success ? "**PASS**" : "**FAIL**";
            string clearanceStr = float.IsPositiveInfinity(s.minimumClearance) || s.minimumClearance < 0f
                ? "N/A"
                : $"{s.minimumClearance:F2}m";

            sb.AppendLine($"| `{s.scenarioName}` | {status} | {s.finalState} | {s.overallScore:F1} | {s.flightTime:F2}s | {s.actualDistance:F2}m | {s.plannedDistance:F2}m | {s.pathEfficiency * 100f:F1}% | {s.replans} | {clearanceStr} | {s.threatEncounters} | {s.criticalThreats} | {s.safetyScore:F1} | {s.navigationScore:F1} | {s.efficiencyScore:F1} | {s.threatManagementScore:F1} | {s.timeScore:F1} |");
        }

        sb.AppendLine();
        sb.AppendLine("### Summary Metrics");
        float successRate = report.totalScenariosEvaluated > 0 ? ((float)report.successfulMissions / report.totalScenariosEvaluated * 100f) : 0f;
        sb.AppendLine($"- **Total Scenarios Evaluated**: {report.totalScenariosEvaluated}");
        sb.AppendLine($"- **Mission Success Rate**: {report.successfulMissions} / {report.totalScenariosEvaluated} ({successRate:F1}%)");
        sb.AppendLine($"- **Average Overall Score**: {report.averageOverallScore:F1} / 100.0");
        sb.AppendLine($"- **Average Flight Duration**: {report.averageFlightTime:F2}s");
        sb.AppendLine($"- **Average Distance Traveled**: {report.averageDistance:F2}m");
        sb.AppendLine($"- **Average Path Efficiency**: {report.averagePathEfficiency * 100f:F1}%");
        sb.AppendLine($"- **Total Dynamic Replans**: {report.totalReplans}");
        sb.AppendLine($"- **Total Threat Encounters**: {report.totalThreatEncounters} ({report.totalCriticalThreats} Critical)");
        sb.AppendLine();

        return sb.ToString();
    }
}

/// <summary>
/// Serializable summary entry for a single scenario within the aggregate benchmark.
/// </summary>
[Serializable]
public class ScenarioBenchmarkSummaryEntry
{
    public string scenarioName;
    public bool success;
    public string finalState;
    public int obstacleCount;
    public int seed;
    public float cruiseSpeed;
    public float sensorRange;
    public float overallScore;
    public float safetyScore;
    public float navigationScore;
    public float efficiencyScore;
    public float threatManagementScore;
    public float timeScore;
    public float flightTime;
    public float actualDistance;
    public float plannedDistance;
    public float pathEfficiency;
    public int replans;
    public int threatEncounters;
    public int criticalThreats;
    public float minimumClearance;
    public int eventCount;
    public string sourceReportFile;
}

/// <summary>
/// Serializable container for the multi-scenario aggregate benchmark report.
/// Compatible with UnityEngine.JsonUtility.
/// </summary>
[Serializable]
public class AggregateBenchmarkReport
{
    public string exportTimestamp;
    public int totalScenariosEvaluated;
    public int successfulMissions;
    public float averageOverallScore;
    public float averageFlightTime;
    public float averageDistance;
    public float averagePathEfficiency;
    public int totalReplans;
    public int totalThreatEncounters;
    public int totalCriticalThreats;
    public List<ScenarioBenchmarkSummaryEntry> scenarioSummaries = new List<ScenarioBenchmarkSummaryEntry>();
}
