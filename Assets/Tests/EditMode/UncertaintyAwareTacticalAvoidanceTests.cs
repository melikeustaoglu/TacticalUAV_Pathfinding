using System.Collections.Generic;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;

/// <summary>
/// Phase 11.3 Uncertainty-Aware Tactical Avoidance Tests.
/// Validates dynamic uncertainty envelopes, clamps, and EstimatorStatus policies in EditMode.
/// </summary>
[TestFixture]
public class UncertaintyAwareTacticalAvoidanceTests
{
    private GameObject uavObj;
    private ThreatAssessment threatAssessment;
    private ReplanningController replanningController;
    private PathFollower pathFollower;
    private UAVPerception perception;
    private MockEstimatedStateProvider mockStateProvider;

    private class MockEstimatedStateProvider : MonoBehaviour, IEstimatedStateProvider
    {
        public EstimatedState State = EstimatedState.Uninitialized;
        public EstimatedState CurrentState => State;
        public bool IsEstimatorReady => State.IsValid;
        public event System.Action<EstimatedState> OnStateEstimated;

        public void SetMockState(EstimatedState state)
        {
            State = state;
            OnStateEstimated?.Invoke(state);
        }
    }

    [SetUp]
    public void SetUp()
    {
        uavObj = new GameObject("UncertaintyUAV");
        mockStateProvider = uavObj.AddComponent<MockEstimatedStateProvider>();
        pathFollower = uavObj.AddComponent<PathFollower>();
        perception = uavObj.AddComponent<UAVPerception>();
        threatAssessment = uavObj.AddComponent<ThreatAssessment>();
        replanningController = uavObj.AddComponent<ReplanningController>();

        typeof(PathFollower).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(pathFollower, null);
        typeof(UAVPerception).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(perception, null);
        typeof(ThreatAssessment).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(threatAssessment, null);
        typeof(ReplanningController).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(replanningController, null);
    }

    [TearDown]
    public void TearDown()
    {
        if (uavObj != null) Object.DestroyImmediate(uavObj);
    }

    [Test]
    public void Uncertainty_EffectiveSafetyRadius_ExpandsWithPositionCovariance()
    {
        // sigma_h = sqrt(0.25) = 0.5m -> R_eff = 1.0 + 2.0 * 0.5 = 2.0m
        EstimatedState state = new EstimatedState(
            Vector3.zero,
            Vector3.zero,
            0f,
            0f,
            Vector3.zero,
            0f,
            new Vector3(0.25f, 0.01f, 0.25f),
            Vector3.zero,
            0f,
            0f,
            EstimatorStatus.Nominal,
            GpsFixState.Fix3D);

        mockStateProvider.SetMockState(state);

        Assert.AreEqual(2.0f, threatAssessment.EffectiveSafetyRadius, 1e-4f);
    }

    [Test]
    public void Uncertainty_EffectiveSafetyRadius_RespectsMaximumClamp()
    {
        // sigma_h = sqrt(4.0) = 2.0m -> raw = 1.0 + 2.0 * 2.0 = 5.0m -> clamped to 2.5m
        EstimatedState state = new EstimatedState(
            Vector3.zero,
            Vector3.zero,
            0f,
            0f,
            Vector3.zero,
            0f,
            new Vector3(4.0f, 0.01f, 4.0f),
            Vector3.zero,
            0f,
            0f,
            EstimatorStatus.Nominal,
            GpsFixState.Fix3D);

        mockStateProvider.SetMockState(state);

        Assert.AreEqual(2.5f, threatAssessment.EffectiveSafetyRadius, 1e-4f);
    }

    [Test]
    public void Uncertainty_EffectiveVerticalSafetyMargin_ExpandsWithVerticalCovariance()
    {
        // sigma_v = sqrt(0.16) = 0.4m -> M_eff = 0.5 + 2.0 * 0.4 = 1.3m
        EstimatedState state = new EstimatedState(
            Vector3.zero,
            Vector3.zero,
            0f,
            0f,
            Vector3.zero,
            0f,
            new Vector3(0.01f, 0.16f, 0.01f),
            Vector3.zero,
            0f,
            0f,
            EstimatorStatus.Nominal,
            GpsFixState.Fix3D);

        mockStateProvider.SetMockState(state);

        Assert.AreEqual(1.3f, threatAssessment.EffectiveVerticalSafetyMargin, 1e-4f);
    }

    [Test]
    public void Uncertainty_EffectiveWarningRadius_ExpandsAndClampsAtMax()
    {
        // sigma_h = sqrt(1.0) = 1.0m -> raw = 2.2 + 2.0 * 1.0 = 4.2m -> clamped to 4.0m
        EstimatedState state = new EstimatedState(
            Vector3.zero,
            Vector3.zero,
            0f,
            0f,
            Vector3.zero,
            0f,
            new Vector3(1.0f, 0.01f, 1.0f),
            Vector3.zero,
            0f,
            0f,
            EstimatorStatus.Nominal,
            GpsFixState.Fix3D);

        mockStateProvider.SetMockState(state);

        Assert.AreEqual(4.0f, threatAssessment.EffectiveWarningRadius, 1e-4f);
    }

    [Test]
    public void Uncertainty_NominalEstimatorState_PreservesNormalTacticalBehavior()
    {
        // Minimal variance (sigma_h = 0.05m -> R_eff = 1.0 + 2.0 * 0.05 = 1.1m)
        EstimatedState state = new EstimatedState(
            Vector3.zero,
            Vector3.forward * 2f,
            0f,
            0f,
            Vector3.zero,
            0f,
            new Vector3(0.0025f, 0.0025f, 0.0025f),
            Vector3.zero,
            0f,
            0f,
            EstimatorStatus.Nominal,
            GpsFixState.Fix3D);

        mockStateProvider.SetMockState(state);

        Assert.AreEqual(1.1f, threatAssessment.EffectiveSafetyRadius, 1e-3f);
        Assert.AreEqual(0.6f, threatAssessment.EffectiveVerticalSafetyMargin, 1e-3f);
    }

    [Test]
    public void Uncertainty_DegradedEstimatorState_UsesExpandedUncertaintyEnvelope()
    {
        // Degraded mode (e.g. GPS denial dead reckoning, sigma_h = 0.4m -> R_eff = 1.8m)
        EstimatedState state = new EstimatedState(
            Vector3.zero,
            Vector3.forward * 2f,
            0f,
            0f,
            Vector3.zero,
            0f,
            new Vector3(0.16f, 0.04f, 0.16f),
            Vector3.zero,
            0f,
            0f,
            EstimatorStatus.Degraded,
            GpsFixState.Degraded);

        mockStateProvider.SetMockState(state);

        Assert.IsTrue(mockStateProvider.IsEstimatorReady);
        Assert.AreEqual(1.8f, threatAssessment.EffectiveSafetyRadius, 1e-4f);
        Assert.AreEqual(0.9f, threatAssessment.EffectiveVerticalSafetyMargin, 1e-4f);
    }

    [Test]
    public void Uncertainty_FailedEstimatorState_EntersSafeHoldBehavior()
    {
        EstimatedState state = new EstimatedState(
            Vector3.zero,
            Vector3.zero,
            0f,
            0f,
            Vector3.zero,
            0f,
            Vector3.one * 10f,
            Vector3.zero,
            0f,
            0f,
            EstimatorStatus.Failed,
            GpsFixState.NoFix);

        mockStateProvider.SetMockState(state);

        pathFollower.StartFollowing(new List<Node> { new Node(true, new Vector3(0f, 1f, 10f), 0, 0) });
        ThreatReport threat = new ThreatReport(ThreatLevel.Critical, DetectedObstacle.Empty, Vector3.forward * 3f, 3f, 1.5f, 0);

        bool result = replanningController.TryExecuteReplan("Critical Threat with Failed Estimator", threat);

        Assert.IsFalse(result, "Replanning must return false when estimator is failed!");
        Assert.AreEqual(NavigationState.NoSafePath, replanningController.State, "UAV must transition to NoSafePath safe hold!");
        Assert.AreEqual(TacticalDecisionReason.NoSafePathHold, replanningController.LatestDecisionReason);
    }

    [Test]
    public void Uncertainty_GPSRecovery_ContractsEnvelopesAsCovarianceDecreases()
    {
        // 1. High uncertainty during outage
        EstimatedState outageState = new EstimatedState(
            Vector3.zero, Vector3.zero, 0f, 0f, Vector3.zero, 0f,
            new Vector3(0.36f, 0.09f, 0.36f), Vector3.zero, 0f, 0f,
            EstimatorStatus.Degraded, GpsFixState.Degraded);
        mockStateProvider.SetMockState(outageState);

        float outageRadius = threatAssessment.EffectiveSafetyRadius;
        Assert.AreEqual(2.2f, outageRadius, 1e-4f);

        // 2. Low uncertainty after recovery
        EstimatedState recoveredState = new EstimatedState(
            Vector3.zero, Vector3.zero, 0f, 0f, Vector3.zero, 0f,
            new Vector3(0.01f, 0.01f, 0.01f), Vector3.zero, 0f, 0f,
            EstimatorStatus.Nominal, GpsFixState.Fix3D);
        mockStateProvider.SetMockState(recoveredState);

        float recoveredRadius = threatAssessment.EffectiveSafetyRadius;
        Assert.AreEqual(1.2f, recoveredRadius, 1e-4f);
        Assert.Less(recoveredRadius, outageRadius, "Safety envelope must contract upon covariance reduction!");
    }

    [Test]
    public void Uncertainty_ZeroCovariance_PreservesOriginalNominalParameters()
    {
        EstimatedState zeroVarState = new EstimatedState(
            Vector3.zero, Vector3.zero, 0f, 0f, Vector3.zero, 0f,
            Vector3.zero, Vector3.zero, 0f, 0f,
            EstimatorStatus.Nominal, GpsFixState.Fix3D);
        mockStateProvider.SetMockState(zeroVarState);

        Assert.AreEqual(1.0f, threatAssessment.EffectiveSafetyRadius, 1e-4f);
        Assert.AreEqual(0.5f, threatAssessment.EffectiveVerticalSafetyMargin, 1e-4f);
        Assert.AreEqual(2.2f, threatAssessment.EffectiveWarningRadius, 1e-4f);
    }

    [Test]
    public void Uncertainty_ReplanCooldown_RemainsOneSecond()
    {
        FieldInfo cooldownField = typeof(ReplanningController).GetField("replanCooldown", BindingFlags.NonPublic | BindingFlags.Instance);
        float cooldown = (float)(cooldownField?.GetValue(replanningController) ?? 0f);

        Assert.AreEqual(1.0f, cooldown, 1e-4f, "Replan cooldown must remain exactly 1.0 second!");
    }
}
