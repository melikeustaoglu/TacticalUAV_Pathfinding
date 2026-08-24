using System.Collections.Generic;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;

/// <summary>
/// Phase B.5 Uncertainty-Aware Autonomy Adaptation Test Suite (B5.1–B5.5).
/// Deterministically validates:
/// - B5.1: GPS denial throttles cruise speed monotonically and smoothly down to bounded floor.
/// - B5.2: GPS reacquisition restores nominal cruise speed without overshoot or deadlocks.
/// - B5.3: High uncertainty expands emergency TTC lookahead threshold to trigger earlier evasion.
/// - B5.4: Spatial A* detour inflates dynamic hazard clearance buffer under position drift.
/// - B5.5: Estimator failure commands immediate safe hold and stops UAV guidance.
/// </summary>
[TestFixture]
public class UncertaintyAwareAutonomyTests
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
        uavObj = new GameObject("TestUAV_PhaseB5");
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
        pathFollower.SetStateProvider(stateProvider);
        replanningController.SetStateProvider(stateProvider);

        typeof(PathFollower).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(pathFollower, null);
        typeof(ThreatAssessment).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(threatAssessment, null);
        typeof(ReplanningController).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(replanningController, null);

        pathFollower.MoveSpeed = 2.0f;
        SetEstimatorState(new Vector3(0f, 1f, 0f), new Vector3(0f, 0f, 2f), posVariance: 0.01f, status: EstimatorStatus.Nominal);
        pathFollower.StartFollowing(new List<Node> { new Node(true, new Vector3(0f, 1f, 20f), 0, 0) });
    }

    [TearDown]
    public void TearDown()
    {
        if (uavObj != null)
        {
            Object.DestroyImmediate(uavObj);
        }
    }

    private void SetEstimatorState(Vector3 pos, Vector3 vel, float posVariance, EstimatorStatus status)
    {
        typeof(GroundTruthStateProvider).GetField("currentState", BindingFlags.NonPublic | BindingFlags.Instance)?.SetValue(
            stateProvider,
            new EstimatedState(
                pos,
                vel,
                0f,
                0f,
                Vector3.zero,
                0f,
                new Vector3(posVariance, 0f, posVariance),
                Vector3.zero,
                0f,
                Time.time,
                status,
                status == EstimatorStatus.Nominal ? GpsFixState.Fix3D : GpsFixState.NoFix));
    }

    [Test]
    public void B5_1_GPSDenial_ThrottlesCruiseSpeed()
    {
        // 1. Nominal GPS (sigma_horiz = sqrt(0.01) = 0.10m <= 0.15m) -> 100% cruise speed
        SetEstimatorState(Vector3.zero, new Vector3(0f, 0f, 2f), posVariance: 0.01f, status: EstimatorStatus.Nominal);
        Assert.AreEqual(1.0f, pathFollower.UncertaintySpeedScale, 1e-4f);
        Assert.AreEqual(2.0f, pathFollower.EffectiveCruiseSpeed, 1e-4f);

        // 2. Moderate GPS Outage (sigma_horiz = sqrt(0.64) = 0.80m -> excess = 0.65m)
        // S_speed = 1.0 - 0.60 * 0.65 = 0.610 -> Effective speed = 2.0 * 0.610 = 1.22 m/s
        SetEstimatorState(Vector3.zero, new Vector3(0f, 0f, 2f), posVariance: 0.64f, status: EstimatorStatus.Degraded);
        Assert.AreEqual(0.61f, pathFollower.UncertaintySpeedScale, 1e-3f);
        Assert.AreEqual(1.22f, pathFollower.EffectiveCruiseSpeed, 1e-2f);

        // 3. Severe GPS Outage (sigma_horiz = sqrt(4.0) = 2.00m) -> Clamped to 60% floor (1.20 m/s)
        SetEstimatorState(Vector3.zero, new Vector3(0f, 0f, 2f), posVariance: 4.00f, status: EstimatorStatus.Degraded);
        Assert.AreEqual(0.60f, pathFollower.UncertaintySpeedScale, 1e-4f);
        Assert.AreEqual(1.20f, pathFollower.EffectiveCruiseSpeed, 1e-4f);
    }

    [Test]
    public void B5_2_GPSReacquisition_RestoresNominalCruiseSpeed()
    {
        // 1. Enter GPS outage -> Throttled speed
        SetEstimatorState(Vector3.zero, new Vector3(0f, 0f, 2f), posVariance: 1.44f, status: EstimatorStatus.Degraded); // sigma = 1.2m
        Assert.AreEqual(0.60f, pathFollower.UncertaintySpeedScale, 1e-4f);

        // 2. Reacquire GPS -> Covariance contracts back to nominal (sigma = 0.10m)
        SetEstimatorState(Vector3.zero, new Vector3(0f, 0f, 2f), posVariance: 0.01f, status: EstimatorStatus.Nominal);
        Assert.AreEqual(1.0f, pathFollower.UncertaintySpeedScale, 1e-4f);
        Assert.AreEqual(2.0f, pathFollower.EffectiveCruiseSpeed, 1e-4f);
    }

    [Test]
    public void B5_3_HighUncertainty_ExpandsEmergencyTTC()
    {
        // 1. Nominal GPS -> Emergency TTC = 4.00s
        SetEstimatorState(Vector3.zero, new Vector3(0f, 0f, 2f), posVariance: 0.01f, status: EstimatorStatus.Nominal);
        Assert.AreEqual(4.0f, replanningController.EffectiveEmergencyTtcThreshold, 1e-4f);

        // 2. Moderate GPS Outage (sigma = sqrt(0.5625) = 0.75m -> excess = 0.60m)
        // TTC_eff = 4.0 + 2.5 * 0.60 = 5.50s
        SetEstimatorState(Vector3.zero, new Vector3(0f, 0f, 2f), posVariance: 0.5625f, status: EstimatorStatus.Degraded);
        Assert.AreEqual(5.50f, replanningController.EffectiveEmergencyTtcThreshold, 1e-3f);

        // 3. Severe Outage (sigma = 2.0m) -> Clamped to max 6.00s
        SetEstimatorState(Vector3.zero, new Vector3(0f, 0f, 2f), posVariance: 4.00f, status: EstimatorStatus.Degraded);
        Assert.AreEqual(6.0f, replanningController.EffectiveEmergencyTtcThreshold, 1e-4f);
    }

    [Test]
    public void B5_4_SpatialReplan_MaintainsExpandedClearanceUnderDrift()
    {
        // 1. Under high uncertainty (sigma = 1.0m), safety radius inflates to 2.5m (clamped max)
        SetEstimatorState(new Vector3(0f, 1f, 0f), new Vector3(0f, 0f, 2f), posVariance: 1.0f, status: EstimatorStatus.Degraded);
        Assert.AreEqual(2.5f, threatAssessment.EffectiveSafetyRadius, 1e-3f);

        // Lock flight altitude bounds to 1.0m so Stage 2 vertical climb is rejected by ceiling limit
        pathFollower.MinFlightAltitude = 1.0f;
        pathFollower.MaxFlightAltitude = 1.0f;

        GameObject obs = GameObject.CreatePrimitive(PrimitiveType.Cube);
        obs.transform.position = new Vector3(0f, 1f, 10f);
        Physics.SyncTransforms();

        DetectedObstacle det = new DetectedObstacle(
            obs, obs.GetComponent<BoxCollider>(), obs.transform.position,
            obs.transform.position - uavObj.transform.position,
            Vector3.forward, 10f, 0f, Vector3.back, Vector3.zero, isDynamic: false);
        ThreatReport rep = new ThreatReport(
            ThreatLevel.Critical, det, new Vector3(0f, 1f, 10f), 10f, 3.0f, 0, priorityScore: 0.90f);

        // Set replanning target transform
        GameObject targetObj = new GameObject("TestTarget_B5");
        targetObj.transform.position = new Vector3(0f, 1f, 25f);
        pathfinding.targetTransform = targetObj.transform;

        bool replanned = replanningController.TryExecuteReplan("Spatial A* under high uncertainty", rep);
        Assert.IsTrue(replanned, "Spatial replanning should generate an inflated detour.");
        Assert.AreEqual(TacticalDecisionReason.SpatialDetourExecuted, replanningController.LatestDecisionReason);
        Assert.IsNotNull(pathfinding.path);
        Assert.Greater(pathfinding.path.Count, 0);

        // 3. Verify all detour waypoints maintain expanded distance from obstacle center
        for (int i = 0; i < pathfinding.path.Count; i++)
        {
            Vector3 wp = pathfinding.path[i].worldPosition;
            if (Mathf.Abs(wp.z - obs.transform.position.z) < 1.0f)
            {
                float lateralOffset = Mathf.Abs(wp.x - obs.transform.position.x);
                Assert.GreaterOrEqual(lateralOffset, 2.0f, "Detour waypoints must clear the inflated hazard boundary.");
            }
        }

        Object.DestroyImmediate(obs);
        Object.DestroyImmediate(targetObj);
    }

    [Test]
    public void B5_5_EstimatorFailure_CommandsImmediateSafeHold()
    {
        // 1. UAV cruising nominally
        SetEstimatorState(Vector3.zero, new Vector3(0f, 0f, 2f), posVariance: 0.01f, status: EstimatorStatus.Nominal);
        Assert.IsTrue(pathFollower.IsFollowing);

        // 2. Estimator diverges / enters Failed status
        SetEstimatorState(Vector3.zero, Vector3.zero, posVariance: 999f, status: EstimatorStatus.Failed);

        // 3. Trigger replanning attempt under failed estimator
        ThreatReport dummyReport = new ThreatReport(
            ThreatLevel.Warning, default(DetectedObstacle), Vector3.forward * 5f, 5f, 2.5f, 0, priorityScore: 0.60f);

        bool replanResult = replanningController.TryExecuteReplan("Estimator failure test", dummyReport);

        // Assert fail-safe response
        Assert.IsFalse(replanResult, "Replanning must be refused when estimator has failed.");
        Assert.AreEqual(NavigationState.NoSafePath, replanningController.State, "UAV must transition to NoSafePath hold.");
        Assert.AreEqual(TacticalDecisionReason.NoSafePathHold, replanningController.LatestDecisionReason);
        Assert.IsFalse(pathFollower.IsFollowing, "Path follower must stop immediately on estimator failure.");
    }
}
