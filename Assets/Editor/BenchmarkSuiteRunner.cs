using System;
using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

/// <summary>
/// Automated Multi-Scenario Benchmark Suite Runner (Editor-Only).
/// Sequentially executes all 4 standardized tactical scenarios in isolated Play Mode sessions,
/// collects authoritative telemetry and scoring, and compiles the final aggregate benchmark summary.
/// </summary>
[InitializeOnLoad]
public static class BenchmarkSuiteRunner
{
    private const string MainScenePath = "Assets/Scenes/Main.unity";
    private const float ScenarioTimeoutSeconds = 45.0f;

    private static readonly string[] ScenarioAssetPaths = new string[]
    {
        "Assets/Scenarios/DefaultScenario.asset",
        "Assets/Scenarios/Scenario_AlternativeSeed.asset",
        "Assets/Scenarios/Scenario_DenseObstacles.asset",
        "Assets/Scenarios/Scenario_LongRange.asset",
        "Assets/Scenarios/Scenario_DynamicThreats.asset"
    };

    // SessionState Keys (Survive Play Mode domain reload)
    private const string KeyIsRunning = "BenchmarkRunner_IsRunning";
    private const string KeyCurrentIndex = "BenchmarkRunner_CurrentIndex";
    private const string KeyPlayStartTime = "BenchmarkRunner_PlayStartTime";
    private const string KeyOriginalScenario = "BenchmarkRunner_OriginalScenario";

    static BenchmarkSuiteRunner()
    {
        EditorApplication.playModeStateChanged += OnPlayModeStateChanged;
    }

    [MenuItem("Tactical UAV/Run All Benchmark Scenarios", priority = 10)]
    public static void StartBenchmarkSuite()
    {
        if (EditorApplication.isPlaying || EditorApplication.isPlayingOrWillChangePlaymode)
        {
            EditorUtility.DisplayDialog("Benchmark Runner", "Please exit Play Mode before launching the benchmark suite.", "OK");
            return;
        }

        // 1. Record original scenario from Main.unity for restoration upon completion/abort
        string originalScenarioPath = GetCurrentSceneScenarioPath();
        SessionState.SetString(KeyOriginalScenario, originalScenarioPath);

        // 2. Initialize suite state
        SessionState.SetBool(KeyIsRunning, true);
        SessionState.SetInt(KeyCurrentIndex, 0);

        Debug.Log("[BenchmarkSuiteRunner] Initiating 4-scenario automated benchmark suite...");
        EditorApplication.delayCall += StepNextScenario;
    }

    [MenuItem("Tactical UAV/Abort Benchmark Suite", priority = 11)]
    public static void AbortBenchmarkSuite()
    {
        bool wasRunning = SessionState.GetBool(KeyIsRunning, false);

        SessionState.SetBool(KeyIsRunning, false);
        SessionState.EraseInt(KeyCurrentIndex);
        SessionState.EraseString(KeyPlayStartTime);

        RestoreOriginalSceneScenario();

        if (EditorApplication.isPlaying)
        {
            EditorApplication.isPlaying = false;
        }

        if (wasRunning)
        {
            Debug.LogWarning("[BenchmarkSuiteRunner] Benchmark suite was successfully aborted. Scene restored to clean state.");
        }
        else
        {
            Debug.Log("[BenchmarkSuiteRunner] Benchmark runner is idle. Clean state verified.");
        }
    }

    private static void StepNextScenario()
    {
        if (!SessionState.GetBool(KeyIsRunning, false))
            return;

        int currentIndex = SessionState.GetInt(KeyCurrentIndex, 0);

        if (currentIndex < 0 || currentIndex >= ScenarioAssetPaths.Length)
        {
            FinishBenchmarkSuite();
            return;
        }

        string targetScenarioPath = ScenarioAssetPaths[currentIndex];
        string scenarioName = Path.GetFileNameWithoutExtension(targetScenarioPath);

        if (!File.Exists(targetScenarioPath))
        {
            Debug.LogError($"[BenchmarkSuiteRunner] Could not locate scenario asset at '{targetScenarioPath}'. Skipping.");
            SessionState.SetInt(KeyCurrentIndex, currentIndex + 1);
            EditorApplication.delayCall += StepNextScenario;
            return;
        }

        // Configure Main.unity with the target scenario
        bool configured = ConfigureSceneScenario(targetScenarioPath);
        if (!configured)
        {
            Debug.LogError($"[BenchmarkSuiteRunner] Failed to configure '{targetScenarioPath}' in Main.unity. Skipping.");
            SessionState.SetInt(KeyCurrentIndex, currentIndex + 1);
            EditorApplication.delayCall += StepNextScenario;
            return;
        }

        Debug.Log($"[BenchmarkSuiteRunner] [{currentIndex + 1}/{ScenarioAssetPaths.Length}] Launching scenario: {scenarioName}");

        // Set start time and enter Play Mode
        SessionState.SetString(KeyPlayStartTime, EditorApplication.timeSinceStartup.ToString());
        EditorApplication.EnterPlaymode();
    }

    private static void OnPlayModeStateChanged(PlayModeStateChange state)
    {
        if (!SessionState.GetBool(KeyIsRunning, false))
            return;

        if (state == PlayModeStateChange.EnteredPlayMode)
        {
            EditorApplication.update += MonitorActiveMission;
        }
        else if (state == PlayModeStateChange.ExitingPlayMode)
        {
            EditorApplication.update -= MonitorActiveMission;
        }
        else if (state == PlayModeStateChange.EnteredEditMode)
        {
            // Advance to the next scenario after returning to Edit Mode
            int currentIndex = SessionState.GetInt(KeyCurrentIndex, 0);
            SessionState.SetInt(KeyCurrentIndex, currentIndex + 1);

            EditorApplication.delayCall += StepNextScenario;
        }
    }

    private static void MonitorActiveMission()
    {
        if (!EditorApplication.isPlaying)
            return;

        int currentIndex = SessionState.GetInt(KeyCurrentIndex, 0);
        string scenarioName = currentIndex >= 0 && currentIndex < ScenarioAssetPaths.Length
            ? Path.GetFileNameWithoutExtension(ScenarioAssetPaths[currentIndex])
            : "Scenario";

        // 1. Check Safety Timeout (45s)
        if (double.TryParse(SessionState.GetString(KeyPlayStartTime, "0"), out double startTime))
        {
            double elapsed = EditorApplication.timeSinceStartup - startTime;
            if (elapsed > ScenarioTimeoutSeconds)
            {
                Debug.LogWarning($"[BenchmarkSuiteRunner] Scenario '{scenarioName}' exceeded safety timeout ({ScenarioTimeoutSeconds:F0}s). Forcing completion.");
                EditorApplication.isPlaying = false;
                return;
            }
        }

        // 2. Check Terminal Mission State (Completed / Failed)
        MissionManager missionManager = UnityEngine.Object.FindFirstObjectByType<MissionManager>();
        if (missionManager != null)
        {
            if (missionManager.State == MissionState.Completed || missionManager.State == MissionState.Failed)
            {
                // Terminal state reached; exit Play Mode so runner advances to next scenario
                EditorApplication.isPlaying = false;
            }
        }
    }

    private static void FinishBenchmarkSuite()
    {
        SessionState.SetBool(KeyIsRunning, false);
        SessionState.EraseInt(KeyCurrentIndex);
        SessionState.EraseString(KeyPlayStartTime);

        // Restore original scene state
        RestoreOriginalSceneScenario();

        // Generate final aggregate benchmark summary (JSON + Markdown)
        AggregateBenchmarkReport report = BenchmarkReporter.GenerateAndExportAggregateSummary();

        Debug.Log(
            $"[BenchmarkSuiteRunner] =========================================\n" +
            $"[BenchmarkSuiteRunner] BENCHMARK SUITE EXECUTION COMPLETED!\n" +
            $"[BenchmarkSuiteRunner] Total Scenarios Evaluated: {report.totalScenariosEvaluated}\n" +
            $"[BenchmarkSuiteRunner] Successful Missions: {report.successfulMissions} / {report.totalScenariosEvaluated}\n" +
            $"[BenchmarkSuiteRunner] Average Overall Score: {report.averageOverallScore:F1} / 100.0\n" +
            $"[BenchmarkSuiteRunner] Average Flight Time: {report.averageFlightTime:F2}s\n" +
            $"[BenchmarkSuiteRunner] Average Path Efficiency: {report.averagePathEfficiency * 100f:F1}%\n" +
            $"[BenchmarkSuiteRunner] Total Dynamic Replans: {report.totalReplans}\n" +
            $"[BenchmarkSuiteRunner] Reports Exported to Application.persistentDataPath/MissionReports/\n" +
            $"[BenchmarkSuiteRunner] =========================================");
    }

    private static bool ConfigureSceneScenario(string scenarioPath)
    {
        try
        {
            Scene scene = EditorSceneManager.OpenScene(MainScenePath, OpenSceneMode.Single);
            PathfindingRuntimeSetup setup = UnityEngine.Object.FindFirstObjectByType<PathfindingRuntimeSetup>();

            if (setup == null)
            {
                Debug.LogError("[BenchmarkSuiteRunner] PathfindingRuntimeSetup not found in Main.unity!");
                return false;
            }

            UAVScenarioConfig config = AssetDatabase.LoadAssetAtPath<UAVScenarioConfig>(scenarioPath);
            setup.ScenarioConfig = config;

            EditorSceneManager.SaveScene(scene);
            return true;
        }
        catch (Exception ex)
        {
            Debug.LogError($"[BenchmarkSuiteRunner] Error configuring scene: {ex.Message}");
            return false;
        }
    }

    private static string GetCurrentSceneScenarioPath()
    {
        try
        {
            if (File.Exists(MainScenePath))
            {
                Scene scene = EditorSceneManager.OpenScene(MainScenePath, OpenSceneMode.Single);
                PathfindingRuntimeSetup setup = UnityEngine.Object.FindFirstObjectByType<PathfindingRuntimeSetup>();
                if (setup != null && setup.ScenarioConfig != null)
                {
                    return AssetDatabase.GetAssetPath(setup.ScenarioConfig);
                }
            }
        }
        catch { }

        return "Assets/Scenarios/Scenario_AlternativeSeed.asset";
    }

    private static void RestoreOriginalSceneScenario()
    {
        string originalPath = SessionState.GetString(KeyOriginalScenario, "Assets/Scenarios/Scenario_AlternativeSeed.asset");
        if (string.IsNullOrEmpty(originalPath) || !File.Exists(originalPath))
        {
            originalPath = "Assets/Scenarios/Scenario_AlternativeSeed.asset";
        }

        ConfigureSceneScenario(originalPath);
        SessionState.EraseString(KeyOriginalScenario);
    }
}
