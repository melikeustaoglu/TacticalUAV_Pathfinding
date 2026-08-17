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
    public int threatEncounters;
    public int criticalThreats;
    public float minimumClearance;

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

    public MissionBenchmarkReport LastReport { get; private set; }
    public string LastReportJson { get; private set; }
    public string LastExportPath { get; private set; }

    private void Awake()
    {
        missionManager = GetComponent<MissionManager>() ?? FindFirstObjectByType<MissionManager>();
        eventLogger = GetComponent<MissionEventLogger>() ?? FindFirstObjectByType<MissionEventLogger>();
        pathFollower = GetComponent<PathFollower>() ?? FindFirstObjectByType<PathFollower>();
        perception = GetComponent<UAVPerception>() ?? FindFirstObjectByType<UAVPerception>();
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
        report.threatEncounters = result.TotalThreatEncounters;
        report.criticalThreats = result.CriticalThreatCount;
        report.minimumClearance = result.MinimumClearanceObserved;

        // 3. Evaluation Scores
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
        }
        catch (Exception ex)
        {
            Debug.LogWarning($"[BenchmarkReporter] File export warning: {ex.Message}");
        }

        return report;
    }
}
