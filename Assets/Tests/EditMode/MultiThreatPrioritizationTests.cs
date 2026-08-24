using System;
using System.Collections.Generic;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;

/// <summary>
/// Phase C Multi-Threat Prioritization Test Suite (C1–C6).
/// Deterministically validates:
/// - C1: 3 simultaneous threats prioritized by composite TTC & CPA kinematics.
/// - C2: Distant high-speed threat prioritized over close slow threat due to compressed reaction time.
/// - C3: Strict preservation of discrete ThreatLevel dominance across severity bands.
/// - C4: Anti-oscillation hysteresis margin (Delta = 0.05) preventing rank flipping on similar risk.
/// - C5: Emergency cooldown preemption for imminent critical threats (P >= 0.85, TTC <= 1.5s).
/// - C6: Disappearing/reappearing threat fallback and promotion without state corruption.
/// </summary>
[TestFixture]
public class MultiThreatPrioritizationTests
{
    private GameObject uavObj;
    private ThreatAssessment threatAssessment;
    private PathFollower pathFollower;
    private ReplanningController replanningController;
    private GridManager gridManager;
    private Pathfinding pathfinding;
    private GroundTruthStateProvider stateProvider;

    [SetUp]
    public void SetUp()
    {
        uavObj = new GameObject("TestUAV_PhaseC");
        uavObj.transform.position = new Vector3(0f, 1f, 0f);

        gridManager = uavObj.AddComponent<GridManager>();
        gridManager.gridWorldSize = new Vector2(40f, 40f);
        gridManager.nodeRadius = 0.5f;

        pathfinding = uavObj.AddComponent<Pathfinding>();
        typeof(Pathfinding).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(pathfinding, null);
        gridManager.CreateGrid();

        pathFollower = uavObj.AddComponent<PathFollower>();
        stateProvider = uavObj.AddComponent<GroundTruthStateProvider>();
        threatAssessment = uavObj.AddComponent<ThreatAssessment>();
        replanningController = uavObj.AddComponent<ReplanningController>();

        threatAssessment.SetStateProvider(stateProvider);

        typeof(PathFollower).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(pathFollower, null);
        typeof(ThreatAssessment).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(threatAssessment, null);
        typeof(ReplanningController).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(replanningController, null);

        // Configure nominal UAV forward cruise velocity (0, 0, 2 m/s) at (0, 1, 0)
        typeof(GroundTruthStateProvider).GetField("currentState", BindingFlags.NonPublic | BindingFlags.Instance)?.SetValue(
            stateProvider,
            new EstimatedState(
                new Vector3(0f, 1f, 0f),
                new Vector3(0f, 0f, 2f),
                0f,
                0f,
                Vector3.zero,
                0f,
                Vector3.zero,
                Vector3.zero,
                0f,
                Time.time,
                EstimatorStatus.Nominal,
                GpsFixState.Fix3D));

        pathFollower.MoveSpeed = 2.0f;
        pathFollower.StartFollowing(new List<Node> { new Node(true, new Vector3(0f, 1f, 20f), 0, 0) });
    }

    [TearDown]
    public void TearDown()
    {
        if (uavObj != null)
        {
            UnityEngine.Object.DestroyImmediate(uavObj);
        }
    }

    private TrackedTarget CreateTrack(int id, Vector3 pos, Vector3 vel, float confidence = 1.0f)
    {
        return new TrackedTarget(
            id,
            pos,
            vel,
            Vector3.one * 0.04f,
            Vector3.one * 0.01f,
            TrackStatus.Confirmed,
            age: 2.0f,
            timeSinceLastUpdate: 0f,
            confidence: confidence,
            estimatedExtents: Vector3.one,
            corroboratingModalityMask: 3); // Dual LiDAR + Radar
    }

    [Test]
    public void C1_ThreeSimultaneousThreats_PrioritizesMostCriticalByTTCAndCPA()
    {
        // Ownship at (0, 1, 0) with velocity (0, 0, 2)
        // Threat 1: Distant Head-On at (0, 1, 8), V = (0, 0, 0), TTC = 4.0s, dCPA = 0m (Critical)
        TrackedTarget t1 = CreateTrack(1, new Vector3(0f, 1f, 8f), Vector3.zero);

        // Threat 2: Imminent Crossing at (-4, 1, 4), V = (2, 0, 0), TTC = 2.0s, dCPA = 0m (Critical)
        TrackedTarget t2 = CreateTrack(2, new Vector3(-4f, 1f, 4f), new Vector3(2f, 0f, 0f));

        // Threat 3: Glancing Near-Miss at (1.8, 1, 6), V = (0, 0, 0), TTC = 3.0s, dCPA = 1.8m (Warning)
        TrackedTarget t3 = CreateTrack(3, new Vector3(1.8f, 1f, 6f), Vector3.zero);

        TrackedTarget[] targets = new TrackedTarget[] { t1, t2, t3 };
        threatAssessment.EvaluateTrackedTargets(targets, 3);

        // [DERIVED]: Threat 2 has shortest TTC (2.0s) on direct collision -> Highest priority score
        Assert.AreEqual(2, threatAssessment.CurrentThreatReport.ThreateningTrack.TrackId, "Threat 2 must be selected as primary.");
        Assert.AreEqual(3, threatAssessment.ActiveThreatReports.Count);

        // Verify sorted order: Threat 2 (Critical, TTC=2.0s) > Threat 1 (Critical, TTC=4.0s) > Threat 3 (Warning, TTC=3.0s)
        Assert.AreEqual(2, threatAssessment.ActiveThreatReports[0].ThreateningTrack.TrackId);
        Assert.AreEqual(1, threatAssessment.ActiveThreatReports[1].ThreateningTrack.TrackId);
        Assert.AreEqual(3, threatAssessment.ActiveThreatReports[2].ThreateningTrack.TrackId);

        Assert.Greater(threatAssessment.ActiveThreatReports[0].PriorityScore, threatAssessment.ActiveThreatReports[1].PriorityScore);
        Assert.Greater(threatAssessment.ActiveThreatReports[1].PriorityScore, threatAssessment.ActiveThreatReports[2].PriorityScore);
    }

    [Test]
    public void C2_DistantHighSpeedThreat_Vs_CloseLowSpeedThreat()
    {
        // Ownship at (0, 1, 0) moving +Z at 2 m/s
        // Threat 1: High-Speed Head-On at (0, 1, 15), V = (0, 0, -8), Closing = 10 m/s, TTC = 1.5s, Dist = 15m
        TrackedTarget fastDistant = CreateTrack(1, new Vector3(0f, 1f, 15f), new Vector3(0f, 0f, -8f));

        // Threat 2: Slow Close Obstacle at (0, 1, 4), V = (0, 0, 0.8), Closing = 1.2 m/s, TTC = 3.33s, Dist = 4m
        TrackedTarget slowClose = CreateTrack(2, new Vector3(0f, 1f, 4f), new Vector3(0f, 0f, 0.8f));

        TrackedTarget[] targets = new TrackedTarget[] { fastDistant, slowClose };
        threatAssessment.EvaluateTrackedTargets(targets, 2);

        // [DERIVED]: Fast distant threat will impact in 1.5s vs 3.33s -> Must be prioritized despite larger initial distance
        Assert.AreEqual(1, threatAssessment.CurrentThreatReport.ThreateningTrack.TrackId, "Fast incoming threat must be prioritized over slow close threat.");
        Assert.AreEqual(2, threatAssessment.ActiveThreatReports.Count);
        Assert.Greater(threatAssessment.ActiveThreatReports[0].PriorityScore, threatAssessment.ActiveThreatReports[1].PriorityScore);
    }

    [Test]
    public void C3_DiscreteThreatLevelSemantics_AlwaysPreserved()
    {
        // Verify strict tier bounding:
        // Critical: [0.70, 1.00]
        // Warning:  [0.40, 0.65]
        // Advisory: [0.15, 0.35]
        // None:     0.00

        float minCritical = ThreatAssessment.ComputeThreatPriority(ThreatLevel.Critical, 10f, 10f, 50f, 0f, 0.40f, 1.0f, 2.2f);
        float maxWarning = ThreatAssessment.ComputeThreatPriority(ThreatLevel.Warning, 0f, 0f, 0f, 20f, 1.00f, 1.0f, 2.2f);
        float minWarning = ThreatAssessment.ComputeThreatPriority(ThreatLevel.Warning, 10f, 10f, 50f, 0f, 0.40f, 1.0f, 2.2f);
        float maxAdvisory = ThreatAssessment.ComputeThreatPriority(ThreatLevel.Advisory, 0f, 0f, 0f, 20f, 1.00f, 1.0f, 2.2f);
        float noneScore = ThreatAssessment.ComputeThreatPriority(ThreatLevel.None, 0f, 0f, 0f, 20f, 1.00f, 1.0f, 2.2f);

        Assert.GreaterOrEqual(minCritical, 0.70f);
        Assert.LessOrEqual(maxWarning, 0.65f);
        Assert.Greater(minCritical, maxWarning, "Minimum Critical priority must strictly exceed Maximum Warning priority.");

        Assert.GreaterOrEqual(minWarning, 0.40f);
        Assert.LessOrEqual(maxAdvisory, 0.35f);
        Assert.Greater(minWarning, maxAdvisory, "Minimum Warning priority must strictly exceed Maximum Advisory priority.");

        Assert.AreEqual(0f, noneScore, "ThreatLevel.None must always evaluate to 0.00.");
    }

    [Test]
    public void C4_PriorityHysteresis_PreventsOscillationOnSimilarRisk()
    {
        // Candidate 1: Imminent collision P ~ 0.88
        ThreatReport rep1 = new ThreatReport(
            ThreatLevel.Critical,
            CreateTrack(1, new Vector3(0f, 1f, 5f), Vector3.zero),
            new Vector3(0f, 1f, 5f),
            5.0f,
            2.5f,
            0,
            priorityScore: 0.88f,
            closingVelocity: 2.0f,
            distanceAtCpa: 0f);

        // Candidate 2: Slightly higher risk P ~ 0.90 (Delta P = 0.02 < hysteresis threshold 0.05)
        ThreatReport rep2 = new ThreatReport(
            ThreatLevel.Critical,
            CreateTrack(2, new Vector3(0f, 1f, 4.8f), Vector3.zero),
            new Vector3(0f, 1f, 4.8f),
            4.8f,
            2.4f,
            0,
            priorityScore: 0.90f,
            closingVelocity: 2.0f,
            distanceAtCpa: 0f);

        // Candidate 3: Substantially higher risk P ~ 0.96 (Delta P = 0.08 > hysteresis threshold 0.05)
        ThreatReport rep3 = new ThreatReport(
            ThreatLevel.Critical,
            CreateTrack(3, new Vector3(0f, 1f, 2.0f), Vector3.zero),
            new Vector3(0f, 1f, 2.0f),
            2.0f,
            1.0f,
            0,
            priorityScore: 0.96f,
            closingVelocity: 2.0f,
            distanceAtCpa: 0f);

        MethodInfo isMoreSevere = typeof(ThreatAssessment).GetMethod("IsMoreSevereThreat", BindingFlags.NonPublic | BindingFlags.Static);

        // rep2 compared against rep1 (current): Delta P = 0.02 <= 0.05 -> Should NOT trigger switch
        bool switchOnSmallDelta = (bool)isMoreSevere.Invoke(null, new object[] { rep2, rep1 });
        Assert.IsFalse(switchOnSmallDelta, "Hysteresis must reject rank switch when Delta P <= 0.05.");

        // rep3 compared against rep1 (current): Delta P = 0.08 > 0.05 -> MUST trigger switch
        bool switchOnLargeDelta = (bool)isMoreSevere.Invoke(null, new object[] { rep3, rep1 });
        Assert.IsTrue(switchOnLargeDelta, "Rank switch must succeed when Delta P > 0.05.");
    }

    [Test]
    public void C5_EmergencyPreemption_BypassesCooldownForImminentCriticalThreat()
    {
        // 1. Trigger an initial non-emergency warning replan at t = 0
        GameObject initialObs = GameObject.CreatePrimitive(PrimitiveType.Cube);
        initialObs.transform.position = new Vector3(0f, 1f, 10f);
        DetectedObstacle detInit = new DetectedObstacle(
            initialObs, initialObs.GetComponent<BoxCollider>(), initialObs.transform.position,
            initialObs.transform.position - uavObj.transform.position,
            Vector3.forward, 10f, 0f, Vector3.back, Vector3.zero, isDynamic: false);
        ThreatReport repInit = new ThreatReport(
            ThreatLevel.Warning, detInit, new Vector3(0f, 1f, 10f), 10f, 5.0f, 0, priorityScore: 0.55f);

        bool firstReplan = replanningController.TryExecuteReplan("Initial Warning", repInit);
        Assert.IsTrue(firstReplan);
        Assert.AreEqual(1, replanningController.ReplanCount);

        // 2. Normal secondary trigger within cooldown is rejected
        bool nonEmergencyReplan = replanningController.TryExecuteReplan("Secondary non-critical", repInit);
        Assert.IsFalse(nonEmergencyReplan, "Normal trigger within cooldown must be blocked.");

        // 3. Imminent Critical Threat (P >= 0.85, TTC <= 1.5s) arrives within cooldown
        GameObject emergencyObs = GameObject.CreatePrimitive(PrimitiveType.Cube);
        emergencyObs.transform.position = new Vector3(0f, 1f, 2.5f);
        DetectedObstacle detEmerg = new DetectedObstacle(
            emergencyObs, emergencyObs.GetComponent<BoxCollider>(), emergencyObs.transform.position,
            emergencyObs.transform.position - uavObj.transform.position,
            Vector3.forward, 2.5f, 0f, Vector3.back, Vector3.zero, isDynamic: false);
        ThreatReport repEmerg = new ThreatReport(
            ThreatLevel.Critical, detEmerg, new Vector3(0f, 1f, 2.5f), 2.5f, 1.25f, 0, priorityScore: 0.92f);

        bool emergencyPreempted = replanningController.TryExecuteReplan("Imminent Critical Threat", repEmerg);
        Assert.IsTrue(emergencyPreempted, "Imminent critical threat must bypass cooldown via emergency preemption!");
        Assert.AreEqual(2, replanningController.ReplanCount);

        UnityEngine.Object.DestroyImmediate(initialObs);
        UnityEngine.Object.DestroyImmediate(emergencyObs);
    }

    [Test]
    public void C6_DisappearingReappearingThreat_GracefullyReevaluatesPriority()
    {
        TrackedTarget t1 = CreateTrack(1, new Vector3(0f, 1f, 4f), Vector3.zero); // Higher priority (closer, TTC 2.0s)
        TrackedTarget t2 = CreateTrack(2, new Vector3(0f, 1f, 8f), Vector3.zero); // Lower priority (further, TTC 4.0s)

        // 1. Both active
        threatAssessment.EvaluateTrackedTargets(new TrackedTarget[] { t1, t2 }, 2);
        Assert.AreEqual(1, threatAssessment.CurrentThreatReport.ThreateningTrack.TrackId);
        Assert.AreEqual(2, threatAssessment.ActiveThreatReports.Count);

        // 2. Target 1 disappears (dropped/pruned) -> Target 2 promoted
        threatAssessment.EvaluateTrackedTargets(new TrackedTarget[] { t2 }, 1);
        Assert.AreEqual(2, threatAssessment.CurrentThreatReport.ThreateningTrack.TrackId, "Target 2 must be promoted when Target 1 disappears.");
        Assert.AreEqual(1, threatAssessment.ActiveThreatReports.Count);

        // 3. All targets clear -> Current threat is Clear
        threatAssessment.EvaluateTrackedTargets(new TrackedTarget[0], 0);
        Assert.AreEqual(ThreatLevel.None, threatAssessment.CurrentThreatLevel);
        Assert.AreEqual(0, threatAssessment.ActiveThreatReports.Count);
    }
}
