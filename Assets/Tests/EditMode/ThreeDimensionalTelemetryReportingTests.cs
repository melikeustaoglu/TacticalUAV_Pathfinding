using System.Collections.Generic;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.UI;

[TestFixture]
public class ThreeDimensionalTelemetryReportingTests
{
    private GameObject uavObj;
    private GameObject obstacleObj;
    private GameObject targetObj;
    private PathFollower pathFollower;
    private UAVPerception uavPerception;
    private ThreatAssessment threatAssessment;
    private ReplanningController replanningController;
    private GridManager gridManager;
    private Pathfinding pathfinding;

    [SetUp]
    public void SetUp()
    {
        uavObj = new GameObject("TelemetryTestUAV");
        gridManager = uavObj.AddComponent<GridManager>();
        gridManager.gridWorldSize = new Vector2(40f, 40f);
        gridManager.nodeRadius = 0.5f;
        gridManager.obstacleMask = ProceduralObstacleGenerator.GetObstacleMask();

        pathfinding = uavObj.AddComponent<Pathfinding>();
        typeof(Pathfinding).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(pathfinding, null);
        gridManager.CreateGrid();

        pathFollower = uavObj.AddComponent<PathFollower>();
        uavPerception = uavObj.AddComponent<UAVPerception>();
        threatAssessment = uavObj.AddComponent<ThreatAssessment>();
        replanningController = uavObj.AddComponent<ReplanningController>();

        uavPerception.ObstacleMask = gridManager.obstacleMask;

        typeof(PathFollower).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(pathFollower, null);
        typeof(UAVPerception).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(uavPerception, null);
        typeof(ThreatAssessment).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(threatAssessment, null);
        typeof(ReplanningController).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(replanningController, null);

        typeof(ReplanningController).GetMethod("Start", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(replanningController, null);

        targetObj = new GameObject("TelemetryTestTarget");
        targetObj.transform.position = new Vector3(0f, 1f, 20f);
        pathfinding.targetTransform = targetObj.transform;

        pathFollower.MoveSpeed = 1.5f;
        pathFollower.MinFlightAltitude = 1.0f;
        pathFollower.MaxFlightAltitude = 6.0f;
        pathFollower.MaxClimbRate = 1.5f;
        pathFollower.MaxDescentRate = 2.0f;
        replanningController.NominalAltitude = 1.0f;
    }

    [TearDown]
    public void TearDown()
    {
        if (targetObj != null) Object.DestroyImmediate(targetObj);
        if (obstacleObj != null) Object.DestroyImmediate(obstacleObj);
        if (uavObj != null) Object.DestroyImmediate(uavObj);
    }

    [Test]
    public void Stage1SpeedPacing_FiresTelemetryEvent()
    {
        uavObj.transform.position = new Vector3(0f, 1f, 0f);
        uavObj.transform.forward = Vector3.forward;

        List<Node> flightPath = new List<Node>
        {
            new Node(true, new Vector3(0f, 1f, 20f), 0, 0)
        };
        pathFollower.StartFollowing(flightPath);

        obstacleObj = GameObject.CreatePrimitive(PrimitiveType.Cube);
        obstacleObj.name = "DynamicThreat_Telemetry";
        obstacleObj.layer = LayerMask.NameToLayer(ProceduralObstacleGenerator.ObstacleLayerName);
        obstacleObj.transform.position = new Vector3(-5f, 1f, 5f);
        obstacleObj.transform.localScale = Vector3.one * 1.5f;

        DynamicObstacle dynComp = obstacleObj.AddComponent<DynamicObstacle>();
        dynComp.Speed = 1.5f;
        dynComp.MovementMode = ObstacleMovementMode.Patrol;
        dynComp.MovementEnabled = true;
        dynComp.SetPatrolWaypoints(new List<Vector3>
        {
            new Vector3(-5f, 1f, 5f),
            new Vector3(5f, 1f, 5f)
        });
        dynComp.Step(0.02f);

        Physics.SyncTransforms();

        int eventFiredCount = 0;
        float firedRatio = 0f;
        float firedDuration = 0f;

        replanningController.OnSpeedPacingApplied += (ratio, duration) =>
        {
            eventFiredCount++;
            firedRatio = ratio;
            firedDuration = duration;
        };

        uavPerception.PerformScan();
        threatAssessment.EvaluateThreats();

        Assert.AreEqual(1, eventFiredCount, "OnSpeedPacingApplied must fire exactly once on Stage 1 activation!");
        Assert.Greater(firedRatio, 0f, "Fired speed override ratio must be positive!");
        Assert.LessOrEqual(firedRatio, 1.0f, "Fired speed override ratio must be <= 1.0!");
        Assert.Greater(firedDuration, 0f, "Fired override duration must be positive!");
        Assert.AreEqual(1, replanningController.SpeedPacingCount);
    }

    [Test]
    public void Stage2VerticalEvasion_FiresTelemetryEvent()
    {
        uavObj.transform.position = new Vector3(0f, 1f, 0f);
        uavObj.transform.forward = Vector3.forward;

        List<Node> flightPath = new List<Node>
        {
            new Node(true, new Vector3(0f, 1f, 20f), 0, 0)
        };
        pathFollower.StartFollowing(flightPath);

        obstacleObj = GameObject.CreatePrimitive(PrimitiveType.Cube);
        obstacleObj.name = "StaticObs_Telemetry";
        obstacleObj.layer = LayerMask.NameToLayer(ProceduralObstacleGenerator.ObstacleLayerName);
        obstacleObj.transform.position = new Vector3(0f, 1.0f, 5f);
        obstacleObj.transform.localScale = new Vector3(2f, 2.0f, 2f);

        BoxCollider col = obstacleObj.GetComponent<BoxCollider>();
        Physics.SyncTransforms();

        int eventFiredCount = 0;
        float firedTargetAltitude = 0f;

        replanningController.OnVerticalEvasionExecuted += (targetAlt) =>
        {
            eventFiredCount++;
            firedTargetAltitude = targetAlt;
        };

        uavPerception.PerformScan();
        threatAssessment.EvaluateThreats();

        float expectedAltitude = col.bounds.max.y + threatAssessment.VerticalSafetyMargin;
        Assert.AreEqual(1, eventFiredCount, "OnVerticalEvasionExecuted must fire exactly once on Stage 2 activation!");
        Assert.AreEqual(expectedAltitude, firedTargetAltitude, 0.01f, "Fired target altitude must match obstacle top + safety margin!");
        Assert.AreEqual(1, replanningController.VerticalEvasionCount);
    }

    [Test]
    public void MissionEventLogger_Records3DTacticalEvents()
    {
        uavObj.transform.position = new Vector3(0f, 1f, 0f);
        uavObj.transform.forward = Vector3.forward;

        MissionEventLogger logger = uavObj.AddComponent<MissionEventLogger>();
        typeof(MissionEventLogger).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(logger, null);
        typeof(MissionEventLogger).GetMethod("OnEnable", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(logger, null);

        List<Node> flightPath = new List<Node>
        {
            new Node(true, new Vector3(0f, 1f, 20f), 0, 0)
        };
        pathFollower.StartFollowing(flightPath);

        // 1. Trigger Stage 2 Vertical Evasion
        obstacleObj = GameObject.CreatePrimitive(PrimitiveType.Cube);
        obstacleObj.name = "StaticObs_EventLogger";
        obstacleObj.layer = LayerMask.NameToLayer(ProceduralObstacleGenerator.ObstacleLayerName);
        obstacleObj.transform.position = new Vector3(0f, 1.0f, 5f);
        obstacleObj.transform.localScale = new Vector3(2f, 2.0f, 2f);

        Physics.SyncTransforms();
        uavPerception.PerformScan();
        threatAssessment.EvaluateThreats();

        bool hasVerticalEvasionEvent = false;
        for (int i = 0; i < logger.Events.Count; i++)
        {
            if (logger.Events[i].EventType == "VERTICAL_EVASION_EXECUTED")
            {
                hasVerticalEvasionEvent = true;
                break;
            }
        }

        Assert.IsTrue(hasVerticalEvasionEvent, "MissionEventLogger must record VERTICAL_EVASION_EXECUTED event!");
    }

    [Test]
    public void BenchmarkReport_Contains3DAltitudeMetrics()
    {
        uavObj.transform.position = new Vector3(0f, 3.8f, 10f);

        BenchmarkReporter reporter = uavObj.AddComponent<BenchmarkReporter>();
        typeof(BenchmarkReporter).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(reporter, null);
        typeof(BenchmarkReporter).GetMethod("Update", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(reporter, null);

        MissionResult result = new MissionResult(true, MissionState.Completed, 12f, 20f, 20f, 1, 1, 0, 2.5f, 1.0f);
        MissionBenchmarkReport report = reporter.GenerateAndExportReport(result);

        Assert.IsNotNull(report);
        Assert.AreEqual(6.0f, report.maxFlightAltitude, 0.01f);
        Assert.AreEqual(1.0f, report.nominalFlightAltitude, 0.01f);
        Assert.GreaterOrEqual(report.peakAltitudeReached, 3.8f, "Peak altitude reached must record UAV position altitude!");
        Assert.IsTrue(reporter.LastReportJson.Contains("\"peakAltitudeReached\":"), "JSON export must contain peakAltitudeReached field!");
        Assert.IsTrue(reporter.LastReportJson.Contains("\"verticalEvasions\":"), "JSON export must contain verticalEvasions field!");
    }

    [Test]
    public void TacticalHUD_Displays3DCoordinatesAndVerticalCounter()
    {
        uavObj.transform.position = new Vector3(3.5f, 4.2f, 7.8f);

        MissionManager missionManager = uavObj.AddComponent<MissionManager>();
        TacticalHUD hud = uavObj.AddComponent<TacticalHUD>();
        typeof(TacticalHUD).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(hud, null);
        typeof(TacticalHUD).GetMethod("Start", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(hud, null);

        hud.RefreshDisplay(true);

        FieldInfo telemetryTextField = typeof(TacticalHUD).GetField("telemetryText", BindingFlags.NonPublic | BindingFlags.Instance);
        Text textComp = telemetryTextField?.GetValue(hud) as Text;

        Assert.IsNotNull(textComp, "TacticalHUD telemetry text component must exist!");
        string content = textComp.text;

        Assert.IsTrue(content.Contains("Y:"), "HUD coordinates must display Y altitude!");
        Assert.IsTrue(content.Contains("Vertical:"), "HUD dynamic replans line must display Vertical counter!");
    }

    [Test]
    public void AltitudeRecovery_ReturnsTowardNominalAfterThreatClear()
    {
        // 1. UAV is cruising at nominal altitude 1.0m
        pathFollower.SetTargetAltitude(1.0f);
        replanningController.NominalAltitude = 1.0f;

        // 2. UAV performs climb to 2.8m to evade an obstacle
        pathFollower.SetTargetAltitude(2.8f);
        Assert.AreEqual(2.8f, pathFollower.TargetAltitude, 0.01f);

        // 3. Threat clears completely (no active obstacles in perception)
        uavPerception.PerformScan();
        threatAssessment.EvaluateThreats();

        // 4. Trigger recovery
        replanningController.RecoverNominalAltitude();

        // 5. Verify target altitude recovered to nominal (1.0m) and does not breach min flight altitude
        Assert.AreEqual(1.0f, pathFollower.TargetAltitude, 0.01f, "Target altitude must recover to nominal altitude!");
        Assert.GreaterOrEqual(pathFollower.TargetAltitude, pathFollower.MinFlightAltitude, "Target altitude must not breach min flight altitude!");
    }

    [Test]
    public void TacticalExplainability_ExposesRejectionReasonsAndCounters()
    {
        // 1. Setup obstacle taller than MaxFlightAltitude (6.0m)
        obstacleObj = GameObject.CreatePrimitive(PrimitiveType.Cube);
        obstacleObj.name = "CeilingBreachObstacle";
        obstacleObj.layer = LayerMask.NameToLayer(ProceduralObstacleGenerator.ObstacleLayerName);
        obstacleObj.transform.position = new Vector3(0f, 3.5f, 6f);
        obstacleObj.transform.localScale = new Vector3(2f, 7.0f, 2f); // Top is at 7.0m > 6.0m max altitude
        Physics.SyncTransforms();

        uavObj.transform.position = new Vector3(0f, 1f, 0f);
        pathFollower.StartFollowing(new List<Node> { new Node(true, new Vector3(0f, 1f, 20f), 0, 0) });

        uavPerception.PerformScan();
        threatAssessment.EvaluateThreats();

        ThreatReport report = threatAssessment.CurrentThreatReport;
        Assert.AreNotEqual(ThreatLevel.None, report.ThreatLevel);

        TacticalDecisionReason lastReason = TacticalDecisionReason.None;
        replanningController.OnTacticalDecisionMade += (reason, desc) => lastReason = reason;

        // Try vertical evasion
        bool feasible = replanningController.TryTacticalVerticalEvasion(report, out float targetAlt, out TacticalDecisionReason failureReason);

        Assert.IsFalse(feasible, "Vertical evasion must be infeasible for 7.0m obstacle with 6.0m ceiling!");
        Assert.AreEqual(TacticalDecisionReason.VerticalRejectedCeilingExceeded, failureReason);
    }

    [Test]
    public void TacticalHUD_DisplaysTacticalDecisionReasonBadge()
    {
        uavObj.transform.position = new Vector3(0f, 1f, 0f);
        MissionManager missionManager = uavObj.AddComponent<MissionManager>();
        TacticalHUD hud = uavObj.AddComponent<TacticalHUD>();
        typeof(TacticalHUD).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(hud, null);
        typeof(TacticalHUD).GetMethod("Start", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(hud, null);

        hud.RefreshDisplay(true);

        FieldInfo telemetryTextField = typeof(TacticalHUD).GetField("telemetryText", BindingFlags.NonPublic | BindingFlags.Instance);
        Text textComp = telemetryTextField?.GetValue(hud) as Text;

        Assert.IsNotNull(textComp);
        Assert.IsTrue(textComp.text.Contains("Tactical Decision:"), "HUD must contain Tactical Decision line!");
    }
}
