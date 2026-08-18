using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;

[TestFixture]
public class CollisionPredictionTests
{
    [Test]
    public void PredictPathCollision_DirectHeadOnObstacle_ReturnsWillCollideTrueWithAccurateTTC()
    {
        Vector3 uavPos = new Vector3(0f, 1f, 0f);
        Vector3 velocity = new Vector3(0f, 0f, 2.0f); // 2 m/s forward (+Z)
        Vector3 targetWaypoint = new Vector3(0f, 1f, 20f);

        Vector3 obstaclePos = new Vector3(0f, 1f, 10f); // 10m ahead directly on flight path
        DetectedObstacle obstacle = new DetectedObstacle(
            null, null, obstaclePos, obstaclePos - uavPos, (obstaclePos - uavPos).normalized, 10f, 0f, Vector3.back);

        CollisionPredictionResult result = CollisionPrediction.PredictPathCollision(
            uavPos, velocity, 2.0f, null, targetWaypoint, obstacle, safetyRadius: 1.5f, lookaheadTime: 8.0f);

        Assert.IsTrue(result.WillCollide);
        Assert.AreEqual(10f, result.DistanceToCollision, 0.1f);
        Assert.AreEqual(5.0f, result.TimeToCollision, 0.1f); // 10m / 2m/s = 5s
        Assert.Less(result.CrossTrackDistance, 0.1f);
    }

    [Test]
    public void PredictPathCollision_LateralObstacleOutsideSafetyRadius_ReturnsWillCollideFalse()
    {
        Vector3 uavPos = new Vector3(0f, 1f, 0f);
        Vector3 velocity = new Vector3(0f, 0f, 2.0f);
        Vector3 targetWaypoint = new Vector3(0f, 1f, 20f);

        Vector3 obstaclePos = new Vector3(5.0f, 1f, 10f); // 5m to the right (+X)
        DetectedObstacle obstacle = new DetectedObstacle(
            null, null, obstaclePos, obstaclePos - uavPos, (obstaclePos - uavPos).normalized, 11.18f, 26.5f, Vector3.left);

        CollisionPredictionResult result = CollisionPrediction.PredictPathCollision(
            uavPos, velocity, 2.0f, null, targetWaypoint, obstacle, safetyRadius: 1.5f, lookaheadTime: 8.0f);

        Assert.IsFalse(result.WillCollide);
        Assert.AreEqual(5.0f, result.CrossTrackDistance, 0.1f);
        Assert.IsTrue(float.IsPositiveInfinity(result.TimeToCollision));
    }

    [Test]
    public void PredictPathCollision_ObstacleBeyondLookaheadWindow_Ignored()
    {
        Vector3 uavPos = new Vector3(0f, 1f, 0f);
        Vector3 velocity = new Vector3(0f, 0f, 2.0f);
        Vector3 targetWaypoint = new Vector3(0f, 1f, 50f);

        Vector3 obstaclePos = new Vector3(0f, 1f, 30f); // 30m ahead (requires 15s to reach)
        DetectedObstacle obstacle = new DetectedObstacle(
            null, null, obstaclePos, obstaclePos - uavPos, (obstaclePos - uavPos).normalized, 30f, 0f, Vector3.back);

        // Lookahead is 5 seconds (10m max distance)
        CollisionPredictionResult result = CollisionPrediction.PredictPathCollision(
            uavPos, velocity, 2.0f, null, targetWaypoint, obstacle, safetyRadius: 1.5f, lookaheadTime: 5.0f);

        Assert.IsFalse(result.WillCollide);
    }
}
