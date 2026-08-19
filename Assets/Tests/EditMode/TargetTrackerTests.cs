using System;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;

/// <summary>
/// Phase 12.3 Target Tracker 6-State Kalman Filter Unit Tests.
/// Validates kinematic prediction, LiDAR position-only update, Radar position+velocity update,
/// Joseph-form covariance stability, and ground-truth boundary isolation.
/// </summary>
[TestFixture]
public class TargetTrackerTests
{
    private TargetTracker tracker;

    [SetUp]
    public void SetUp()
    {
        tracker = new TargetTracker();
    }

    [Test]
    public void TargetTracker_InitializesFromPositionOnlyDetection()
    {
        Vector3 pos = new Vector3(5f, 1f, 10f);
        Vector3 posVar = Vector3.one * 0.04f;
        TargetDetection det = new TargetDetection(TargetSensorModality.LiDAR, 1.0f, pos, posVar, 0.95f, 1);

        bool result = tracker.Initialize(det);

        Assert.IsTrue(result);
        Assert.IsTrue(tracker.IsInitialized);
        Assert.AreEqual(pos, tracker.EstimatedPosition);
        Assert.AreEqual(Vector3.zero, tracker.EstimatedVelocity);
        Assert.AreEqual(posVar.x, tracker.PositionVariance.x, 0.001f);
        Assert.AreEqual(4.0f, tracker.VelocityVariance.x, 0.001f, "Default initial velocity variance must be 4.0 m^2/s^2!");
    }

    [Test]
    public void TargetTracker_InitializesVelocityFromRadarDetection()
    {
        Vector3 pos = new Vector3(5f, 1f, 10f);
        Vector3 vel = new Vector3(0f, 0f, 2.5f);
        Vector3 posVar = Vector3.one * 0.09f;
        Vector3 velVar = Vector3.one * 0.04f;

        TargetDetection det = new TargetDetection(
            TargetSensorModality.Radar, 1.0f, pos, posVar, 0.90f, 1, vel, velVar, true);

        bool result = tracker.Initialize(det);

        Assert.IsTrue(result);
        Assert.AreEqual(pos, tracker.EstimatedPosition);
        Assert.AreEqual(vel, tracker.EstimatedVelocity);
        Assert.AreEqual(velVar.z, tracker.VelocityVariance.z, 0.001f);
    }

    [Test]
    public void TargetTracker_PredictsConstantVelocityMotion()
    {
        Vector3 initialPos = new Vector3(0f, 1f, 0f);
        Vector3 initialVel = new Vector3(1f, 0f, 2f);

        TargetDetection det = new TargetDetection(
            TargetSensorModality.Radar, 0.0f, initialPos, Vector3.one * 0.01f, 0.95f, 1,
            initialVel, Vector3.one * 0.01f, true);

        tracker.Initialize(det);
        tracker.Predict(2.0f); // dt = 2.0s

        Vector3 expectedPos = initialPos + initialVel * 2.0f; // (2, 1, 4)
        Assert.AreEqual(expectedPos.x, tracker.EstimatedPosition.x, 0.001f);
        Assert.AreEqual(expectedPos.y, tracker.EstimatedPosition.y, 0.001f);
        Assert.AreEqual(expectedPos.z, tracker.EstimatedPosition.z, 0.001f);
        Assert.AreEqual(initialVel, tracker.EstimatedVelocity);
    }

    [Test]
    public void TargetTracker_PositionUpdateReducesPositionUncertainty()
    {
        Vector3 pos = new Vector3(0f, 0f, 5f);
        TargetDetection initDet = new TargetDetection(TargetSensorModality.LiDAR, 0.0f, pos, Vector3.one * 1.0f, 0.95f, 1);
        tracker.Initialize(initDet);

        float initVar = tracker.PositionVariance.x;

        // Perform 5 consecutive position updates
        for (int i = 1; i <= 5; i++)
        {
            float t = i * 0.05f;
            TargetDetection det = new TargetDetection(TargetSensorModality.LiDAR, t, pos, Vector3.one * 0.04f, 0.95f, i + 1);
            tracker.Update(det);
        }

        float updatedVar = tracker.PositionVariance.x;
        Assert.Less(updatedVar, initVar, "Position variance must decrease as measurements are fused!");
    }

    [Test]
    public void TargetTracker_VelocityUpdateConvergesVelocity()
    {
        // Target moves at true velocity (0, 0, 2.0 m/s)
        Vector3 trueVel = new Vector3(0f, 0f, 2.0f);
        Vector3 startPos = new Vector3(0f, 0f, 0f);

        TargetDetection initDet = new TargetDetection(TargetSensorModality.LiDAR, 0.0f, startPos, Vector3.one * 0.04f, 0.95f, 1);
        tracker.Initialize(initDet);

        // Feed 15 position-only LiDAR measurements along trajectory
        for (int i = 1; i <= 15; i++)
        {
            float t = i * 0.1f;
            Vector3 pos = startPos + trueVel * t;
            TargetDetection det = new TargetDetection(TargetSensorModality.LiDAR, t, pos, Vector3.one * 0.04f, 0.95f, i + 1);
            tracker.Update(det);
        }

        Assert.AreEqual(2.0f, tracker.EstimatedVelocity.z, 0.25f, "Velocity must converge from position-only observations!");
    }

    [Test]
    public void TargetTracker_MultiplePredictionStepsRemainDeterministic()
    {
        TargetTracker t1 = new TargetTracker();
        TargetTracker t2 = new TargetTracker();

        TargetDetection det = new TargetDetection(TargetSensorModality.LiDAR, 0f, new Vector3(1f, 2f, 3f), Vector3.one * 0.05f, 0.9f, 1);
        t1.Initialize(det);
        t2.Initialize(det);

        for (int i = 1; i <= 20; i++)
        {
            t1.Predict(i * 0.05f);
            t2.Predict(i * 0.05f);
        }

        Assert.AreEqual(t1.EstimatedPosition, t2.EstimatedPosition);
        Assert.AreEqual(t1.EstimatedVelocity, t2.EstimatedVelocity);
        Assert.AreEqual(t1.PositionVariance, t2.PositionVariance);
    }

    [Test]
    public void TargetTracker_CovarianceRemainsSymmetric()
    {
        TargetDetection det = new TargetDetection(TargetSensorModality.LiDAR, 0f, new Vector3(0f, 0f, 5f), Vector3.one * 0.1f, 0.95f, 1);
        tracker.Initialize(det);

        for (int i = 1; i <= 10; i++)
        {
            float t = i * 0.05f;
            TargetDetection meas = new TargetDetection(TargetSensorModality.LiDAR, t, new Vector3(0f, 0f, 5f + i * 0.1f), Vector3.one * 0.05f, 0.95f, i + 1);
            tracker.Update(meas);
        }

        // Reflection to inspect internal covariance matrix symmetry
        FieldInfo covField = typeof(TargetTracker).GetField("covariance", BindingFlags.NonPublic | BindingFlags.Instance);
        Matrix6x6 P = (Matrix6x6)covField.GetValue(tracker);

        for (int r = 0; r < 6; r++)
        {
            for (int c = 0; c < 6; c++)
            {
                Assert.AreEqual(P[r, c], P[c, r], 0.0001f, $"Covariance element ({r},{c}) must equal ({c},{r})!");
            }
        }
    }

    [Test]
    public void TargetTracker_CovarianceRemainsPositiveFinite()
    {
        TargetDetection det = new TargetDetection(TargetSensorModality.LiDAR, 0f, Vector3.zero, Vector3.one * 0.1f, 0.95f, 1);
        tracker.Initialize(det);

        for (int i = 1; i <= 10; i++)
        {
            tracker.Predict(i * 0.1f);
        }

        Assert.IsTrue(float.IsFinite(tracker.PositionVariance.x));
        Assert.IsTrue(float.IsFinite(tracker.PositionVariance.y));
        Assert.IsTrue(float.IsFinite(tracker.PositionVariance.z));
        Assert.Greater(tracker.PositionVariance.x, 0f);
        Assert.Greater(tracker.PositionVariance.y, 0f);
        Assert.Greater(tracker.PositionVariance.z, 0f);
    }

    [Test]
    public void TargetTracker_HandlesPositionOnlyAfterRadarInitialization()
    {
        TargetDetection radarDet = new TargetDetection(
            TargetSensorModality.Radar, 0f, new Vector3(0f, 0f, 5f), Vector3.one * 0.05f, 0.9f, 1,
            new Vector3(0f, 0f, 1f), Vector3.one * 0.02f, true);

        tracker.Initialize(radarDet);

        // Subsequent LiDAR update (position only)
        TargetDetection lidarDet = new TargetDetection(
            TargetSensorModality.LiDAR, 0.1f, new Vector3(0f, 0f, 5.1f), Vector3.one * 0.02f, 0.95f, 2);

        bool success = tracker.Update(lidarDet);
        Assert.IsTrue(success);
        Assert.AreEqual(5.1f, tracker.EstimatedPosition.z, 0.1f);
    }

    [Test]
    public void TargetTracker_HandlesRadarUpdateAfterPositionOnlyInitialization()
    {
        TargetDetection lidarDet = new TargetDetection(
            TargetSensorModality.LiDAR, 0f, new Vector3(0f, 0f, 5f), Vector3.one * 0.05f, 0.95f, 1);

        tracker.Initialize(lidarDet);
        Assert.AreEqual(4.0f, tracker.VelocityVariance.z, 0.01f);

        // Subsequent Radar update with direct velocity observation
        TargetDetection radarDet = new TargetDetection(
            TargetSensorModality.Radar, 0.1f, new Vector3(0f, 0f, 5.2f), Vector3.one * 0.05f, 0.9f, 2,
            new Vector3(0f, 0f, 2.0f), Vector3.one * 0.04f, true);

        bool success = tracker.Update(radarDet);
        Assert.IsTrue(success);
        Assert.Less(tracker.VelocityVariance.z, 1.0f, "Radar update must contract velocity uncertainty rapidly!");
    }

    [Test]
    public void TargetTracker_RejectsInvalidTimestamp()
    {
        TargetDetection det = new TargetDetection(TargetSensorModality.LiDAR, 1.0f, Vector3.zero, Vector3.one * 0.1f, 0.95f, 1);
        tracker.Initialize(det);

        // Attempt backwards prediction (timestamp < 1.0f)
        bool result = tracker.Predict(0.5f);
        Assert.IsFalse(result, "Tracker must reject backwards-in-time prediction steps!");
    }

    [Test]
    public void TargetTracker_DoesNotAllocateDuringRepeatedUpdates()
    {
        TargetDetection initDet = new TargetDetection(TargetSensorModality.LiDAR, 0f, Vector3.zero, Vector3.one * 0.04f, 0.95f, 1);
        tracker.Initialize(initDet);

        // Warm up JIT
        for (int i = 1; i <= 5; i++)
        {
            TargetDetection det = new TargetDetection(TargetSensorModality.LiDAR, i * 0.05f, new Vector3(0f, 0f, i * 0.1f), Vector3.one * 0.04f, 0.95f, i + 1);
            tracker.Update(det);
        }

        long memBefore = GC.GetTotalMemory(true);

        for (int i = 6; i <= 50; i++)
        {
            TargetDetection det = new TargetDetection(TargetSensorModality.LiDAR, i * 0.05f, new Vector3(0f, 0f, i * 0.1f), Vector3.one * 0.04f, 0.95f, i + 1);
            tracker.Update(det);
        }

        long memAfter = GC.GetTotalMemory(false);
        Assert.AreEqual(memBefore, memAfter, "TargetTracker must execute with zero heap allocations!");
    }

    [Test]
    public void TargetTracker_DoesNotExposeGroundTruthReferences()
    {
        FieldInfo[] fields = typeof(TargetTracker).GetFields(BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance);
        foreach (var field in fields)
        {
            Assert.IsFalse(field.FieldType.Name.Contains("DynamicObstacle"), "Tracker must not contain DynamicObstacle references!");
            Assert.IsFalse(field.FieldType.Name.Contains("Collider"), "Tracker must not contain Collider references!");
            Assert.IsFalse(field.FieldType.Name.Contains("Transform"), "Tracker must not contain Transform references!");
        }
    }

    [Test]
    public void TargetTracker_SameInputSequenceProducesIdenticalState()
    {
        TargetTracker t1 = new TargetTracker();
        TargetTracker t2 = new TargetTracker();

        TargetDetection d1 = new TargetDetection(TargetSensorModality.LiDAR, 0f, new Vector3(2f, 1f, 5f), Vector3.one * 0.04f, 0.95f, 1);
        TargetDetection d2 = new TargetDetection(TargetSensorModality.Radar, 0.1f, new Vector3(2.1f, 1f, 5.2f), Vector3.one * 0.04f, 0.9f, 2, new Vector3(1f, 0f, 2f), Vector3.one * 0.04f, true);
        TargetDetection d3 = new TargetDetection(TargetSensorModality.LiDAR, 0.2f, new Vector3(2.2f, 1f, 5.4f), Vector3.one * 0.04f, 0.95f, 3);

        t1.Initialize(d1); t1.Update(d2); t1.Update(d3);
        t2.Initialize(d1); t2.Update(d2); t2.Update(d3);

        Assert.AreEqual(t1.EstimatedPosition.x, t2.EstimatedPosition.x, 0.00001f);
        Assert.AreEqual(t1.EstimatedPosition.y, t2.EstimatedPosition.y, 0.00001f);
        Assert.AreEqual(t1.EstimatedPosition.z, t2.EstimatedPosition.z, 0.00001f);
        Assert.AreEqual(t1.EstimatedVelocity.x, t2.EstimatedVelocity.x, 0.00001f);
        Assert.AreEqual(t1.EstimatedVelocity.y, t2.EstimatedVelocity.y, 0.00001f);
        Assert.AreEqual(t1.EstimatedVelocity.z, t2.EstimatedVelocity.z, 0.00001f);
    }
}
