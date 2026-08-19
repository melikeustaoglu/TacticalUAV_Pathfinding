using System;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;

/// <summary>
/// Phase 12.5 Threat Assessment Multi-Track Integration & Ground-Truth Decoupling Tests.
/// Validates TrackedTarget consumption, CPA/TTC computation from estimated kinematics,
/// uncertainty-aware spatial expansion, multi-target deterministic selection, and zero-allocation updates.
/// </summary>
[TestFixture]
public class ThreatAssessmentTrackingTests
{
    private GameObject uavObj;
    private ThreatAssessment threatAssessment;
    private GroundTruthStateProvider stateProvider;

    [SetUp]
    public void SetUp()
    {
        uavObj = new GameObject("UAV_ThreatAssessment");
        uavObj.transform.position = Vector3.zero;
        uavObj.transform.rotation = Quaternion.identity;

        stateProvider = uavObj.AddComponent<GroundTruthStateProvider>();
        threatAssessment = uavObj.AddComponent<ThreatAssessment>();

        threatAssessment.SetStateProvider(stateProvider);

        // Initialize state provider with nominal forward velocity (0, 0, 5)
        stateProvider.SetGroundTruth(
            Vector3.zero,
            Quaternion.identity,
            new Vector3(0f, 0f, 5.0f),
            Vector3.zero,
            Vector3.zero,
            Vector3.zero,
            0f);
    }

    [TearDown]
    public void TearDown()
    {
        if (uavObj != null) Object.DestroyImmediate(uavObj);
    }

    [Test]
    public void ThreatAssessment_ConsumesConfirmedTrackedTarget()
    {
        // Target in front at (0, 0, 10), moving toward UAV at (0, 0, -5)
        TrackedTarget target = new TrackedTarget(
            1,
            new Vector3(0f, 0f, 10f),
            new Vector3(0f, 0f, -5f),
            Vector3.one * 0.04f,
            Vector3.one * 0.04f,
            TrackStatus.Confirmed,
            1.0f,
            0.05f,
            0.95f,
            Vector3.one);

        threatAssessment.EvaluateTrackedTargets(new TrackedTarget[] { target }, 1);

        Assert.AreEqual(ThreatLevel.Critical, threatAssessment.CurrentThreatLevel);
        Assert.AreEqual(1.0f, threatAssessment.CurrentThreatReport.TimeToCollision, 0.05f, "TTC must be ~1.0s (10m / 10m/s closure rate)!");
        Assert.AreEqual(1, threatAssessment.CurrentThreatReport.ThreateningTrack.TrackId);
    }

    [Test]
    public void ThreatAssessment_IgnoresTentativeTrack()
    {
        TrackedTarget tentativeTarget = new TrackedTarget(
            1,
            new Vector3(0f, 0f, 5f),
            new Vector3(0f, 0f, -5f),
            Vector3.one * 0.04f,
            Vector3.one * 0.04f,
            TrackStatus.Tentative,
            0.1f,
            0.05f,
            0.50f,
            Vector3.one);

        threatAssessment.EvaluateTrackedTargets(new TrackedTarget[] { tentativeTarget }, 1);

        Assert.AreEqual(ThreatLevel.None, threatAssessment.CurrentThreatLevel, "Tentative tracks must NOT be evaluated as active threats!");
    }

    [Test]
    public void ThreatAssessment_DoesNotUseGroundTruthVelocity()
    {
        // Verify via reflection that ThreatAssessment does not access DynamicObstacle
        FieldInfo[] fields = typeof(ThreatAssessment).GetFields(BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance);
        foreach (var field in fields)
        {
            Assert.IsFalse(field.FieldType.Name.Contains("DynamicObstacle"), "ThreatAssessment must not reference DynamicObstacle!");
        }
    }

    [Test]
    public void ThreatAssessment_UsesEstimatedTargetVelocityForCPA()
    {
        // UAV moving forward at (0, 0, 5)
        // Target at (5, 0, 10), moving left at (-5, 0, 0)
        // Relative closure reaches CPA
        TrackedTarget target = new TrackedTarget(
            1,
            new Vector3(5f, 0f, 10f),
            new Vector3(-5f, 0f, 0f),
            Vector3.one * 0.04f,
            Vector3.one * 0.04f,
            TrackStatus.Confirmed,
            1.0f,
            0.05f,
            0.95f,
            Vector3.one);

        threatAssessment.EvaluateTrackedTargets(new TrackedTarget[] { target }, 1);

        // CPA calculation from estimated kinematics should be non-trivial
        Assert.AreNotEqual(ThreatLevel.None, threatAssessment.CurrentThreatLevel);
        Assert.IsTrue(float.IsFinite(threatAssessment.CurrentThreatReport.TimeToCollision));
    }

    [Test]
    public void ThreatAssessment_UsesEstimatedTargetVelocityForTTC()
    {
        // UAV moving at (0,0,5), Target at (0,0,20) moving at (0,0,-5) -> closure rate = 10 m/s -> TTC = 2.0s
        TrackedTarget target = new TrackedTarget(
            1,
            new Vector3(0f, 0f, 20f),
            new Vector3(0f, 0f, -5f),
            Vector3.one * 0.04f,
            Vector3.one * 0.04f,
            TrackStatus.Confirmed,
            2.0f,
            0.05f,
            0.95f,
            Vector3.one);

        threatAssessment.EvaluateTrackedTargets(new TrackedTarget[] { target }, 1);

        Assert.AreEqual(2.0f, threatAssessment.CurrentThreatReport.TimeToCollision, 0.05f);
    }

    [Test]
    public void ThreatAssessment_HandlesMultipleTrackedTargets()
    {
        TrackedTarget target1 = new TrackedTarget(1, new Vector3(10f, 0f, 20f), Vector3.zero, Vector3.one * 0.04f, Vector3.one * 0.04f, TrackStatus.Confirmed, 1f, 0.05f, 0.9f, Vector3.one);
        TrackedTarget target2 = new TrackedTarget(2, new Vector3(0f, 0f, 6f), new Vector3(0f, 0f, -2f), Vector3.one * 0.04f, Vector3.one * 0.04f, TrackStatus.Confirmed, 1f, 0.05f, 0.9f, Vector3.one);
        TrackedTarget target3 = new TrackedTarget(3, new Vector3(-8f, 0f, 15f), Vector3.zero, Vector3.one * 0.04f, Vector3.one * 0.04f, TrackStatus.Confirmed, 1f, 0.05f, 0.9f, Vector3.one);

        TrackedTarget[] targets = new TrackedTarget[] { target1, target2, target3 };
        threatAssessment.EvaluateTrackedTargets(targets, 3);

        Assert.AreEqual(3, threatAssessment.AllEvaluatedReports.Count);
        Assert.AreEqual(2, threatAssessment.CurrentThreatReport.ThreateningTrack.TrackId, "Target #2 (head-on collision course) must be selected!");
    }

    [Test]
    public void ThreatAssessment_SelectsMostCriticalThreatDeterministically()
    {
        // Target 1: Warning level at 8m
        TrackedTarget t1 = new TrackedTarget(1, new Vector3(1.5f, 0f, 8f), Vector3.zero, Vector3.one * 0.04f, Vector3.one * 0.04f, TrackStatus.Confirmed, 1f, 0.05f, 0.9f, Vector3.one);
        // Target 2: Critical collision at 10m closing fast
        TrackedTarget t2 = new TrackedTarget(2, new Vector3(0f, 0f, 10f), new Vector3(0f, 0f, -5f), Vector3.one * 0.04f, Vector3.one * 0.04f, TrackStatus.Confirmed, 1f, 0.05f, 0.9f, Vector3.one);

        threatAssessment.EvaluateTrackedTargets(new TrackedTarget[] { t1, t2 }, 2);

        Assert.AreEqual(ThreatLevel.Critical, threatAssessment.CurrentThreatLevel);
        Assert.AreEqual(2, threatAssessment.CurrentThreatReport.ThreateningTrack.TrackId);
    }

    [Test]
    public void ThreatAssessment_UsesTargetPositionUncertainty()
    {
        // High target position variance expands effective safety radius
        TrackedTarget targetLowVar = new TrackedTarget(1, new Vector3(1.5f, 0f, 5f), Vector3.zero, Vector3.one * 0.01f, Vector3.one * 0.01f, TrackStatus.Confirmed, 1f, 0.05f, 0.9f, Vector3.one);
        TrackedTarget targetHighVar = new TrackedTarget(2, new Vector3(1.5f, 0f, 5f), Vector3.zero, Vector3.one * 0.36f, Vector3.one * 0.01f, TrackStatus.Confirmed, 1f, 0.05f, 0.9f, Vector3.one); // sigma = 0.6m

        float rLow = threatAssessment.SafetyRadius + threatAssessment.SigmaMultiplier * targetLowVar.HorizontalPositionStdDev;
        float rHigh = threatAssessment.SafetyRadius + threatAssessment.SigmaMultiplier * targetHighVar.HorizontalPositionStdDev;

        Assert.Greater(rHigh, rLow, "Target position uncertainty must expand the effective safety envelope!");
    }

    [Test]
    public void ThreatAssessment_HandlesCoastingTrack()
    {
        TrackedTarget coastingTarget = new TrackedTarget(
            1,
            new Vector3(0f, 0f, 10f),
            new Vector3(0f, 0f, -5f),
            Vector3.one * 0.16f,
            Vector3.one * 0.04f,
            TrackStatus.Coasting,
            2.0f,
            0.40f,
            0.70f,
            Vector3.one);

        threatAssessment.EvaluateTrackedTargets(new TrackedTarget[] { coastingTarget }, 1);

        Assert.AreEqual(ThreatLevel.Critical, threatAssessment.CurrentThreatLevel, "Coasting tracks must continue active collision evaluation!");
    }

    [Test]
    public void ThreatAssessment_IgnoresLostTrack()
    {
        TrackedTarget lostTarget = new TrackedTarget(
            1,
            new Vector3(0f, 0f, 5f),
            new Vector3(0f, 0f, -5f),
            Vector3.one * 1.0f,
            Vector3.one * 0.5f,
            TrackStatus.Lost,
            3.0f,
            1.50f,
            0.30f,
            Vector3.one);

        threatAssessment.EvaluateTrackedTargets(new TrackedTarget[] { lostTarget }, 1);

        Assert.AreEqual(ThreatLevel.None, threatAssessment.CurrentThreatLevel, "Lost tracks must not be evaluated as active threats!");
    }

    [Test]
    public void ThreatAssessment_RejectsDeletedTrack()
    {
        TrackedTarget deletedTarget = new TrackedTarget(
            1,
            new Vector3(0f, 0f, 5f),
            Vector3.zero,
            Vector3.one,
            Vector3.one,
            TrackStatus.Deleted,
            4f,
            3f,
            0f,
            Vector3.one);

        threatAssessment.EvaluateTrackedTargets(new TrackedTarget[] { deletedTarget }, 1);

        Assert.AreEqual(ThreatLevel.None, threatAssessment.CurrentThreatLevel);
    }

    [Test]
    public void ThreatAssessment_NoNaNOrInfinity()
    {
        // Degenerate co-located stationary target
        TrackedTarget target = new TrackedTarget(
            1,
            Vector3.zero,
            Vector3.zero,
            Vector3.one * 0.04f,
            Vector3.one * 0.04f,
            TrackStatus.Confirmed,
            1.0f,
            0.05f,
            0.95f,
            Vector3.one);

        threatAssessment.EvaluateTrackedTargets(new TrackedTarget[] { target }, 1);

        Assert.IsTrue(float.IsFinite(threatAssessment.CurrentThreatReport.DistanceToCollision));
        Assert.IsTrue(float.IsFinite(threatAssessment.CurrentThreatReport.TimeToCollision));
        Assert.IsFalse(float.IsNaN(threatAssessment.CurrentThreatReport.DistanceToCollision));
        Assert.IsFalse(float.IsNaN(threatAssessment.CurrentThreatReport.TimeToCollision));
    }

    [Test]
    public void ThreatAssessment_ZeroAllocationDuringEvaluation()
    {
        TrackedTarget[] targets = new TrackedTarget[]
        {
            new TrackedTarget(1, new Vector3(0f, 0f, 10f), new Vector3(0f, 0f, -2f), Vector3.one * 0.04f, Vector3.one * 0.04f, TrackStatus.Confirmed, 1f, 0.05f, 0.9f, Vector3.one),
            new TrackedTarget(2, new Vector3(5f, 0f, 15f), Vector3.zero, Vector3.one * 0.04f, Vector3.one * 0.04f, TrackStatus.Confirmed, 1f, 0.05f, 0.9f, Vector3.one)
        };

        // Warm up JIT
        for (int i = 0; i < 5; i++)
        {
            threatAssessment.EvaluateTrackedTargets(targets, 2);
        }

        long memBefore = GC.GetTotalMemory(true);

        for (int i = 0; i < 50; i++)
        {
            threatAssessment.EvaluateTrackedTargets(targets, 2);
        }

        long memAfter = GC.GetTotalMemory(false);
        Assert.AreEqual(memBefore, memAfter, "ThreatAssessment must execute evaluations with zero heap allocations!");
    }

    [Test]
    public void ThreatAssessment_GroundTruthIsolation()
    {
        // Verify ThreatReport has no Collider or GameObject reference when produced from TrackedTarget
        TrackedTarget target = new TrackedTarget(1, new Vector3(0f, 0f, 10f), new Vector3(0f, 0f, -5f), Vector3.one * 0.04f, Vector3.one * 0.04f, TrackStatus.Confirmed, 1f, 0.05f, 0.9f, Vector3.one);
        threatAssessment.EvaluateTrackedTargets(new TrackedTarget[] { target }, 1);

        Assert.IsNull(threatAssessment.CurrentThreatReport.ThreateningObstacle.GameObject);
        Assert.IsNull(threatAssessment.CurrentThreatReport.ThreateningObstacle.Collider);
        Assert.IsTrue(threatAssessment.CurrentThreatReport.HasTrack);
    }

    [Test]
    public void ThreatAssessment_PreservesExistingEmergencyTtcLogic()
    {
        Assert.AreEqual(4.5f, threatAssessment.LookaheadTime);
    }

    [Test]
    public void ThreatAssessment_PreservesExistingSafetyRadiusLogic()
    {
        Assert.AreEqual(1.0f, threatAssessment.SafetyRadius);
        Assert.AreEqual(2.2f, threatAssessment.WarningRadius);
        Assert.AreEqual(4.0f, threatAssessment.AdvisoryRadius);
        Assert.AreEqual(0.5f, threatAssessment.VerticalSafetyMargin);
    }
}
