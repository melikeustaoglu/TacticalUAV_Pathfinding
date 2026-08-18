using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;

[TestFixture]
public class DynamicPerceptionPredictionTests
{
    [Test]
    public void StaticObstacle_ReportsZeroVelocity_InDetectedObstacle()
    {
        DetectedObstacle staticObs = new DetectedObstacle(
            null, null, new Vector3(5f, 0f, 10f), new Vector3(5f, 0f, 10f), Vector3.forward, 11.18f, 26.5f, Vector3.back);

        Assert.AreEqual(Vector3.zero, staticObs.Velocity);
        Assert.IsFalse(staticObs.IsDynamic);
    }

    [Test]
    public void UAVPerception_DynamicObstacle_CapturesConfiguredVelocity()
    {
        GameObject obsObj = new GameObject("TestDynamicPerceptionObstacle");
        DynamicObstacle dynComp = obsObj.AddComponent<DynamicObstacle>();
        dynComp.MovementMode = ObstacleMovementMode.Linear;
        dynComp.LinearDirection = Vector3.right; // +X
        dynComp.Speed = 2.5f;
        dynComp.MovementEnabled = true;

        // Step obstacle to set current velocity
        dynComp.Step(0.1f);

        DetectedObstacle detected = new DetectedObstacle(
            obsObj,
            null,
            obsObj.transform.position,
            obsObj.transform.position,
            Vector3.forward,
            10f,
            0f,
            Vector3.back,
            dynComp.CurrentVelocity,
            isDynamic: true);

        Assert.IsTrue(detected.IsDynamic);
        Assert.AreEqual(2.5f, detected.Velocity.x, 0.01f);
        Assert.AreEqual(0f, detected.Velocity.z, 0.01f);

        Object.DestroyImmediate(obsObj);
    }

    [Test]
    public void PredictPathCollision_MovingObstacleVelocity_PassedToCollisionPrediction()
    {
        Vector3 uavPos = Vector3.zero;
        Vector3 uavVel = new Vector3(0f, 0f, 2.0f);
        Vector3 targetWaypoint = new Vector3(0f, 0f, 30f);

        Vector3 obstaclePos = new Vector3(0f, 0f, 12f);
        Vector3 obstacleVel = new Vector3(0f, 0f, -2.0f); // Head-on 2 m/s

        DetectedObstacle movingObs = new DetectedObstacle(
            null, null, obstaclePos, obstaclePos, Vector3.forward, 12f, 0f, Vector3.back, obstacleVel, isDynamic: true);

        CollisionPredictionResult result = CollisionPrediction.PredictPathCollision(
            uavPos, uavVel, 2.0f, null, targetWaypoint, movingObs, safetyRadius: 1.5f, lookaheadTime: 8.0f);

        Assert.IsTrue(result.WillCollide);
        // Relative speed = 2 + 2 = 4 m/s. Distance = 12m. Expected TTC = 3.0s, collision dist = 6m
        Assert.AreEqual(3.0f, result.TimeToCollision, 0.1f);
        Assert.AreEqual(6.0f, result.DistanceToCollision, 0.1f);
    }

    [Test]
    public void PredictPathCollision_HeadOnMovingTarget_CalculatesAccurateCPAAndTTC()
    {
        Vector3 uavPos = Vector3.zero;
        Vector3 uavVel = new Vector3(0f, 0f, 2.0f);
        Vector3 targetWp = new Vector3(0f, 0f, 20f);

        Vector3 obsPos = new Vector3(0f, 0f, 10f);
        Vector3 obsVel = new Vector3(0f, 0f, -2.0f); // Head-on

        DetectedObstacle obstacle = new DetectedObstacle(
            null, null, obsPos, obsPos, Vector3.forward, 10f, 0f, Vector3.back, obsVel, isDynamic: true);

        CollisionPredictionResult result = CollisionPrediction.PredictPathCollision(
            uavPos, uavVel, 2.0f, null, targetWp, obstacle, safetyRadius: 1.0f, lookaheadTime: 5.0f);

        Assert.IsTrue(result.WillCollide);
        Assert.AreEqual(2.5f, result.TimeToCollision, 0.1f); // 10m / 4m/s = 2.5s
        Assert.AreEqual(5.0f, result.DistanceToCollision, 0.1f); // 2 m/s * 2.5s = 5m
        Assert.Less(result.CrossTrackDistance, 0.1f);
    }

    [Test]
    public void PredictPathCollision_CrossingTrajectory_PredictsCollisionCorrectly()
    {
        Vector3 uavPos = Vector3.zero;
        Vector3 uavVel = new Vector3(0f, 0f, 2.0f); // Forward (+Z) at 2 m/s
        Vector3 targetWp = new Vector3(0f, 0f, 20f);

        // Obstacle starts at (5, 0, 10) and moves Left (-X) at 1.0 m/s
        // At t = 5.0s, obstacle reaches (0, 0, 10) exactly as UAV reaches (0, 0, 10)
        Vector3 obsPos = new Vector3(5f, 0f, 10f);
        Vector3 obsVel = new Vector3(-1.0f, 0f, 0f);

        DetectedObstacle crossingObs = new DetectedObstacle(
            null, null, obsPos, obsPos, (obsPos - uavPos).normalized, 11.18f, 26.5f, Vector3.left, obsVel, isDynamic: true);

        CollisionPredictionResult result = CollisionPrediction.PredictPathCollision(
            uavPos, uavVel, 2.0f, null, targetWp, crossingObs, safetyRadius: 1.5f, lookaheadTime: 8.0f);

        Assert.IsTrue(result.WillCollide);
        Assert.AreEqual(5.0f, result.TimeToCollision, 0.2f);
        Assert.AreEqual(10.0f, result.DistanceToCollision, 0.2f);
        Assert.Less(result.CrossTrackDistance, 0.2f);
    }

    [Test]
    public void PredictPathCollision_DivergingMovingObstacle_DoesNotTriggerFalseCollision()
    {
        Vector3 uavPos = Vector3.zero;
        Vector3 uavVel = new Vector3(0f, 0f, 2.0f);
        Vector3 targetWp = new Vector3(0f, 0f, 20f);

        // Obstacle starts at (5, 0, 10) and moves Right (+X) at 2.0 m/s (diverging away)
        Vector3 obsPos = new Vector3(5f, 0f, 10f);
        Vector3 obsVel = new Vector3(2.0f, 0f, 0f);

        DetectedObstacle divergingObs = new DetectedObstacle(
            null, null, obsPos, obsPos, (obsPos - uavPos).normalized, 11.18f, 26.5f, Vector3.left, obsVel, isDynamic: true);

        CollisionPredictionResult result = CollisionPrediction.PredictPathCollision(
            uavPos, uavVel, 2.0f, null, targetWp, divergingObs, safetyRadius: 1.5f, lookaheadTime: 8.0f);

        Assert.IsFalse(result.WillCollide);
        Assert.Greater(result.CrossTrackDistance, 1.5f);
    }

    [Test]
    public void PredictPathCollision_StaticObstacle_PreservesLegacyExactProjection()
    {
        Vector3 uavPos = new Vector3(0f, 1f, 0f);
        Vector3 velocity = new Vector3(0f, 0f, 2.0f);
        Vector3 targetWaypoint = new Vector3(0f, 1f, 20f);

        Vector3 obstaclePos = new Vector3(0f, 1f, 10f);
        DetectedObstacle staticObs = new DetectedObstacle(
            null, null, obstaclePos, obstaclePos - uavPos, (obstaclePos - uavPos).normalized, 10f, 0f, Vector3.back, Vector3.zero, isDynamic: false);

        CollisionPredictionResult result = CollisionPrediction.PredictPathCollision(
            uavPos, velocity, 2.0f, null, targetWaypoint, staticObs, safetyRadius: 1.5f, lookaheadTime: 8.0f);

        Assert.IsTrue(result.WillCollide);
        Assert.AreEqual(10f, result.DistanceToCollision, 0.1f);
        Assert.AreEqual(5.0f, result.TimeToCollision, 0.1f);
        Assert.Less(result.CrossTrackDistance, 0.1f);
    }
}
