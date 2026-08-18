using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;

[TestFixture]
public class DynamicObstacleTests
{
    private GameObject obstacleObj;
    private DynamicObstacle dynamicObstacle;

    [SetUp]
    public void SetUp()
    {
        obstacleObj = new GameObject("TestDynamicObstacle");
        dynamicObstacle = obstacleObj.AddComponent<DynamicObstacle>();
    }

    [TearDown]
    public void TearDown()
    {
        if (obstacleObj != null)
        {
            Object.DestroyImmediate(obstacleObj);
        }
    }

    [Test]
    public void DynamicObstacle_InitialVelocity_IsZeroWhenMovementDisabled()
    {
        dynamicObstacle.MovementEnabled = false;
        dynamicObstacle.Speed = 2.0f;
        dynamicObstacle.Step(1.0f);

        Assert.AreEqual(Vector3.zero, dynamicObstacle.CurrentVelocity);
        Assert.IsFalse(dynamicObstacle.IsMoving);
    }

    [Test]
    public void DynamicObstacle_LinearMovement_ProducesExpectedDisplacement()
    {
        obstacleObj.transform.position = Vector3.zero;
        dynamicObstacle.MovementMode = ObstacleMovementMode.Linear;
        dynamicObstacle.LinearDirection = Vector3.forward; // +Z
        dynamicObstacle.Speed = 2.5f;
        dynamicObstacle.MovementEnabled = true;

        // Step 2 seconds: should move 5.0m in +Z
        dynamicObstacle.Step(2.0f);

        Assert.AreEqual(0f, obstacleObj.transform.position.x, 0.001f);
        Assert.AreEqual(0f, obstacleObj.transform.position.y, 0.001f);
        Assert.AreEqual(5.0f, obstacleObj.transform.position.z, 0.001f);
    }

    [Test]
    public void DynamicObstacle_LinearMovement_ReportsExpectedVelocity()
    {
        obstacleObj.transform.position = Vector3.zero;
        dynamicObstacle.MovementMode = ObstacleMovementMode.Linear;
        dynamicObstacle.LinearDirection = new Vector3(1f, 0f, 0f); // +X
        dynamicObstacle.Speed = 3.0f;
        dynamicObstacle.MovementEnabled = true;

        dynamicObstacle.Step(0.5f);

        Vector3 expectedVelocity = new Vector3(3.0f, 0f, 0f);
        Assert.AreEqual(expectedVelocity.x, dynamicObstacle.CurrentVelocity.x, 0.001f);
        Assert.AreEqual(expectedVelocity.y, dynamicObstacle.CurrentVelocity.y, 0.001f);
        Assert.AreEqual(expectedVelocity.z, dynamicObstacle.CurrentVelocity.z, 0.001f);
        Assert.IsTrue(dynamicObstacle.IsMoving);
    }

    [Test]
    public void DynamicObstacle_ZeroSpeedMovement_DoesNotProduceNaNOrInfinity()
    {
        obstacleObj.transform.position = new Vector3(5f, 0f, 5f);
        dynamicObstacle.Speed = 0f;
        dynamicObstacle.Step(1.0f);

        Assert.IsFalse(float.IsNaN(dynamicObstacle.CurrentVelocity.x));
        Assert.IsFalse(float.IsInfinity(dynamicObstacle.CurrentVelocity.x));
        Assert.AreEqual(Vector3.zero, dynamicObstacle.CurrentVelocity);

        // Also test zero deltaTime
        dynamicObstacle.Speed = 5.0f;
        dynamicObstacle.Step(0f);
        Assert.IsFalse(float.IsNaN(dynamicObstacle.CurrentVelocity.x));
        Assert.AreEqual(Vector3.zero, dynamicObstacle.CurrentVelocity);
    }

    [Test]
    public void DynamicObstacle_PatrolMovement_FollowsConfiguredWaypointOrder()
    {
        obstacleObj.transform.position = Vector3.zero;
        dynamicObstacle.MovementMode = ObstacleMovementMode.Patrol;
        dynamicObstacle.Speed = 2.0f;
        dynamicObstacle.MovementEnabled = true;

        Vector3 wp1 = new Vector3(0f, 0f, 10f);
        Vector3 wp2 = new Vector3(10f, 0f, 10f);
        dynamicObstacle.SetPatrolWaypoints(wp1, wp2);

        // Step 2.5 seconds (5m movement towards wp1)
        dynamicObstacle.Step(2.5f);
        Assert.AreEqual(5.0f, obstacleObj.transform.position.z, 0.01f);
        Assert.AreEqual(0, dynamicObstacle.CurrentWaypointIndex);

        // Step another 2.5 seconds (reaches wp1 at 10m)
        dynamicObstacle.Step(2.5f);
        Assert.AreEqual(10.0f, obstacleObj.transform.position.z, 0.01f);
        Assert.AreEqual(1, dynamicObstacle.CurrentWaypointIndex);
    }

    [Test]
    public void DynamicObstacle_PingPongPatrol_ReversesDirectionCorrectly()
    {
        obstacleObj.transform.position = Vector3.zero;
        dynamicObstacle.MovementMode = ObstacleMovementMode.Patrol;
        dynamicObstacle.LoopMode = PatrolLoopMode.PingPong;
        dynamicObstacle.Speed = 5.0f;
        dynamicObstacle.MovementEnabled = true;

        Vector3 wpA = new Vector3(0f, 0f, 0f);
        Vector3 wpB = new Vector3(10f, 0f, 0f);
        dynamicObstacle.SetPatrolWaypoints(wpA, wpB);

        // 1. Move to wpB (10m in 2s)
        dynamicObstacle.Step(2.0f);
        Assert.AreEqual(10.0f, obstacleObj.transform.position.x, 0.01f);
        Assert.AreEqual(-1, dynamicObstacle.PatrolDirectionSign);

        // 2. Move back towards wpA for 1s (5m back)
        dynamicObstacle.Step(1.0f);
        Assert.AreEqual(5.0f, obstacleObj.transform.position.x, 0.01f);
        Assert.Less(dynamicObstacle.CurrentVelocity.x, 0f); // Moving in -X
    }

    [Test]
    public void DynamicObstacle_LoopingPatrol_ReturnsToExpectedTrajectory()
    {
        obstacleObj.transform.position = Vector3.zero;
        dynamicObstacle.MovementMode = ObstacleMovementMode.Patrol;
        dynamicObstacle.LoopMode = PatrolLoopMode.Loop;
        dynamicObstacle.Speed = 10.0f;
        dynamicObstacle.MovementEnabled = true;

        Vector3 wp1 = new Vector3(0f, 0f, 10f);
        Vector3 wp2 = new Vector3(10f, 0f, 10f);
        Vector3 wp3 = new Vector3(10f, 0f, 0f);
        dynamicObstacle.SetPatrolWaypoints(wp1, wp2, wp3);

        // Advance past wp1 (10m), wp2 (10m), and wp3 (10m) -> total 30m in 3s
        dynamicObstacle.Step(3.0f);
        Assert.AreEqual(wp3.x, obstacleObj.transform.position.x, 0.01f);
        Assert.AreEqual(wp3.z, obstacleObj.transform.position.z, 0.01f);

        // Looping patrol wraps target back to wp1 (0, 0, 10)
        Assert.AreEqual(0, dynamicObstacle.CurrentWaypointIndex);

        // Step 0.5s towards wp1
        dynamicObstacle.Step(0.5f);
        Assert.Less(obstacleObj.transform.position.x, wp3.x);
        Assert.Greater(obstacleObj.transform.position.z, wp3.z);
    }

    [Test]
    public void DynamicObstacle_DeterministicConfiguration_ProducesDeterministicMovement()
    {
        GameObject objA = new GameObject("ObstacleA");
        DynamicObstacle obsA = objA.AddComponent<DynamicObstacle>();
        obsA.MovementMode = ObstacleMovementMode.Patrol;
        obsA.LoopMode = PatrolLoopMode.PingPong;
        obsA.Speed = 1.75f;
        obsA.SetPatrolWaypoints(new Vector3(2f, 0f, 3f), new Vector3(-4f, 0f, 8f), new Vector3(6f, 0f, -2f));

        GameObject objB = new GameObject("ObstacleB");
        DynamicObstacle obsB = objB.AddComponent<DynamicObstacle>();
        obsB.MovementMode = ObstacleMovementMode.Patrol;
        obsB.LoopMode = PatrolLoopMode.PingPong;
        obsB.Speed = 1.75f;
        obsB.SetPatrolWaypoints(new Vector3(2f, 0f, 3f), new Vector3(-4f, 0f, 8f), new Vector3(6f, 0f, -2f));

        for (int i = 0; i < 50; i++)
        {
            float dt = 0.05f;
            obsA.Step(dt);
            obsB.Step(dt);

            Assert.AreEqual(objA.transform.position.x, objB.transform.position.x, 0.0001f);
            Assert.AreEqual(objA.transform.position.z, objB.transform.position.z, 0.0001f);
            Assert.AreEqual(obsA.CurrentVelocity.x, obsB.CurrentVelocity.x, 0.0001f);
            Assert.AreEqual(obsA.CurrentVelocity.z, obsB.CurrentVelocity.z, 0.0001f);
        }

        Object.DestroyImmediate(objA);
        Object.DestroyImmediate(objB);
    }
}
