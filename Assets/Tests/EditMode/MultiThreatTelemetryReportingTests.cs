using System.Collections.Generic;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.UI;

[TestFixture]
public class MultiThreatTelemetryReportingTests
{
    private GameObject uavObj;
    private MissionManager missionManager;
    private PathFollower pathFollower;
    private ThreatAssessment threatAssessment;
    private ReplanningController replanningController;
    private TacticalHUD tacticalHud;
    private BenchmarkReporter benchmarkReporter;

    [SetUp]
    public void SetUp()
    {
        uavObj = new GameObject("TestUAV_TelemetryReporting");
        uavObj.transform.position = new Vector3(0f, 1f, 0f);

        pathFollower = uavObj.AddComponent<PathFollower>();
        replanningController = uavObj.AddComponent<ReplanningController>();
        threatAssessment = uavObj.AddComponent<ThreatAssessment>();
        missionManager = uavObj.AddComponent<MissionManager>();
        tacticalHud = uavObj.AddComponent<TacticalHUD>();
        benchmarkReporter = uavObj.AddComponent<BenchmarkReporter>();

        typeof(PathFollower).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(pathFollower, null);
        typeof(ThreatAssessment).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(threatAssessment, null);
        typeof(ReplanningController).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(replanningController, null);
        typeof(MissionManager).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(missionManager, null);
        typeof(TacticalHUD).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(tacticalHud, null);
        typeof(BenchmarkReporter).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(benchmarkReporter, null);

        pathFollower.MoveSpeed = 2.0f;

        // Initialize Tactical HUD UI hierarchy
        typeof(TacticalHUD).GetMethod("CreateUGUIHierarchy", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(tacticalHud, null);
    }

    [TearDown]
    public void TearDown()
    {
        Canvas[] canvases = Object.FindObjectsByType<Canvas>(FindObjectsSortMode.None);
        for (int i = 0; i < canvases.Length; i++)
        {
            if (canvases[i].gameObject.name.Contains("TacticalHUD"))
            {
                Object.DestroyImmediate(canvases[i].gameObject);
            }
        }

        if (uavObj != null)
        {
            Object.DestroyImmediate(uavObj);
        }
    }

    [Test]
    public void ReplanningController_ExposesMultiThreatTelemetryCorrectly()
    {
        Assert.AreEqual(0, replanningController.SpeedPacingCount);
        Assert.AreEqual(0, replanningController.SpatialReplanCount);
        Assert.AreEqual(0, replanningController.PeakSimultaneousThreats);
    }

    [Test]
    public void TacticalHUD_TelemetryText_IncludesActiveThreatCount()
    {
        // Populate 2 active threats in ThreatAssessment
        List<ThreatReport> threats = new List<ThreatReport>
        {
            new ThreatReport(ThreatLevel.Critical, default(DetectedObstacle), new Vector3(0f, 1f, 4f), 4f, 2f, 0),
            new ThreatReport(ThreatLevel.Warning, default(DetectedObstacle), new Vector3(2f, 1f, 8f), 8f, 4f, 0)
        };
        FieldInfo activeThreatsField = typeof(ThreatAssessment).GetField("activeThreatReports", BindingFlags.NonPublic | BindingFlags.Instance);
        activeThreatsField?.SetValue(threatAssessment, threats);

        tacticalHud.RefreshDisplay(true);

        FieldInfo telemetryTextField = typeof(TacticalHUD).GetField("telemetryText", BindingFlags.NonPublic | BindingFlags.Instance);
        Text textComponent = telemetryTextField?.GetValue(tacticalHud) as Text;

        Assert.IsNotNull(textComponent);
        Assert.IsTrue(textComponent.text.Contains("Active Threats:"), "HUD must contain 'Active Threats:' label!");
        Assert.IsTrue(textComponent.text.Contains("2"), "HUD must display active threat count 2!");
    }

    [Test]
    public void TacticalHUD_TelemetryText_DisplaysSpeedPacingOverrideWhenActive()
    {
        pathFollower.ApplyTacticalSpeedOverride(0.65f, 3.0f);

        tacticalHud.RefreshDisplay(true);

        FieldInfo telemetryTextField = typeof(TacticalHUD).GetField("telemetryText", BindingFlags.NonPublic | BindingFlags.Instance);
        Text textComponent = telemetryTextField?.GetValue(tacticalHud) as Text;

        Assert.IsNotNull(textComponent);
        Assert.IsTrue(textComponent.text.Contains("VO Pacing"), "HUD flight speed line must display '[VO Pacing: 65%]' when speed override is active!");
    }

    [Test]
    public void TacticalHUD_TelemetryText_DisplaysResolutionBreakdown()
    {
        FieldInfo speedPacingField = typeof(ReplanningController).GetField("speedPacingCount", BindingFlags.NonPublic | BindingFlags.Instance);
        FieldInfo spatialReplanField = typeof(ReplanningController).GetField("spatialReplanCount", BindingFlags.NonPublic | BindingFlags.Instance);
        FieldInfo replanCountField = typeof(ReplanningController).GetField("replanCount", BindingFlags.NonPublic | BindingFlags.Instance);

        speedPacingField?.SetValue(replanningController, 2);
        spatialReplanField?.SetValue(replanningController, 3);
        replanCountField?.SetValue(replanningController, 5);

        tacticalHud.RefreshDisplay(true);

        FieldInfo telemetryTextField = typeof(TacticalHUD).GetField("telemetryText", BindingFlags.NonPublic | BindingFlags.Instance);
        Text textComponent = telemetryTextField?.GetValue(tacticalHud) as Text;

        Assert.IsNotNull(textComponent);
        Assert.IsTrue(textComponent.text.Contains("Pacing: 2") && textComponent.text.Contains("Spatial: 3"), "HUD must display pacing vs spatial replan breakdown!");
    }

    [Test]
    public void BenchmarkReporter_ExportsMultiThreatTelemetryFields()
    {
        FieldInfo speedPacingField = typeof(ReplanningController).GetField("speedPacingCount", BindingFlags.NonPublic | BindingFlags.Instance);
        FieldInfo spatialReplanField = typeof(ReplanningController).GetField("spatialReplanCount", BindingFlags.NonPublic | BindingFlags.Instance);
        FieldInfo peakThreatsField = typeof(ReplanningController).GetField("peakSimultaneousThreats", BindingFlags.NonPublic | BindingFlags.Instance);

        speedPacingField?.SetValue(replanningController, 4);
        spatialReplanField?.SetValue(replanningController, 2);
        peakThreatsField?.SetValue(replanningController, 3);

        MissionResult mockResult = new MissionResult(
            true,
            MissionState.Completed,
            12.5f,
            24.0f,
            20.0f,
            6,
            5,
            2,
            1.8f,
            0.83f);

        benchmarkReporter.GenerateAndExportReport(mockResult);

        MissionBenchmarkReport report = benchmarkReporter.LastReport;
        Assert.IsNotNull(report);
        Assert.AreEqual(6, report.replans);
        Assert.AreEqual(4, report.speedPacingResolutions);
        Assert.AreEqual(2, report.spatialDetours);
        Assert.AreEqual(3, report.peakSimultaneousThreats);
    }

    [Test]
    public void BenchmarkReporter_SerializesMultiThreatMetricsToJson()
    {
        FieldInfo speedPacingField = typeof(ReplanningController).GetField("speedPacingCount", BindingFlags.NonPublic | BindingFlags.Instance);
        FieldInfo spatialReplanField = typeof(ReplanningController).GetField("spatialReplanCount", BindingFlags.NonPublic | BindingFlags.Instance);
        FieldInfo peakThreatsField = typeof(ReplanningController).GetField("peakSimultaneousThreats", BindingFlags.NonPublic | BindingFlags.Instance);

        speedPacingField?.SetValue(replanningController, 3);
        spatialReplanField?.SetValue(replanningController, 1);
        peakThreatsField?.SetValue(replanningController, 2);

        MissionResult mockResult = new MissionResult(true, MissionState.Completed, 10f, 20f, 20f, 4, 3, 1, 2f, 1f);
        benchmarkReporter.GenerateAndExportReport(mockResult);

        string json = benchmarkReporter.LastReportJson;
        Assert.IsNotNull(json);
        Assert.IsTrue(json.Contains("\"speedPacingResolutions\": 3"));
        Assert.IsTrue(json.Contains("\"spatialDetours\": 1"));
        Assert.IsTrue(json.Contains("\"peakSimultaneousThreats\": 2"));
    }

    [Test]
    public void BenchmarkReporter_ZeroThreats_DefaultsGracefully()
    {
        MissionResult mockResult = new MissionResult(true, MissionState.Completed, 5f, 10f, 10f, 0, 0, 0, float.PositiveInfinity, 1f);
        benchmarkReporter.GenerateAndExportReport(mockResult);

        MissionBenchmarkReport report = benchmarkReporter.LastReport;
        Assert.IsNotNull(report);
        Assert.AreEqual(0, report.replans);
        Assert.AreEqual(0, report.speedPacingResolutions);
        Assert.AreEqual(0, report.spatialDetours);
        Assert.AreEqual(0, report.peakSimultaneousThreats);
    }
}
