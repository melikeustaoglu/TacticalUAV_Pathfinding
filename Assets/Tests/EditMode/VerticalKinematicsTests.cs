using System;
using System.Collections.Generic;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;

[TestFixture]
public class VerticalKinematicsTests
{
    private GameObject uavObj;
    private PathFollower pathFollower;
    private GridManager gridManager;
    private MethodInfo moveAlongPathMethod;

    [SetUp]
    public void SetUp()
    {
        uavObj = new GameObject("TestUAV");
        gridManager = uavObj.AddComponent<GridManager>();
        gridManager.gridWorldSize = new Vector2(30f, 30f);
        gridManager.nodeRadius = 0.5f;
        gridManager.CreateGrid();

        pathFollower = uavObj.AddComponent<PathFollower>();
        typeof(PathFollower).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(pathFollower, null);

        pathFollower.MoveSpeed = 1.5f;
        pathFollower.MaxClimbRate = 1.5f;
        pathFollower.MaxDescentRate = 2.0f;
        pathFollower.VerticalAcceleration = 2.0f;
        pathFollower.VerticalDeceleration = 2.5f;
        pathFollower.AltitudeReachThreshold = 0.1f;
        pathFollower.MinFlightAltitude = 1.0f;
        pathFollower.MaxFlightAltitude = 6.0f;

        moveAlongPathMethod = typeof(PathFollower).GetMethod("MoveAlongPath", BindingFlags.NonPublic | BindingFlags.Instance);
    }

    [TearDown]
    public void TearDown()
    {
        if (uavObj != null)
        {
            UnityEngine.Object.DestroyImmediate(uavObj);
        }
    }

    private void StepMovement(float deltaTime)
    {
        moveAlongPathMethod?.Invoke(
            pathFollower,
            new object[]
            {
                uavObj.transform.position,
                deltaTime,
                (Action<Vector3>)(pos => uavObj.transform.position = pos),
                (Action<Quaternion>)(rot => uavObj.transform.rotation = rot)
            });
    }

    [Test]
    public void VerticalKinematics_ClimbRate_ClampedToConfiguredMaximum()
    {
        uavObj.transform.position = new Vector3(0f, 1.0f, 0f);
        pathFollower.SetTargetAltitude(5.0f);

        List<Node> path = new List<Node>
        {
            gridManager.NodeFromWorldPoint(new Vector3(0f, 0f, 20f))
        };
        pathFollower.StartFollowing(path);

        bool reachedHighClimbSpeed = false;
        for (int i = 0; i < 30; i++)
        {
            StepMovement(0.1f);

            Assert.LessOrEqual(
                pathFollower.CurrentVerticalSpeed,
                pathFollower.MaxClimbRate + 0.001f,
                $"Step {i}: Vertical speed {pathFollower.CurrentVerticalSpeed:F3} exceeded MaxClimbRate {pathFollower.MaxClimbRate}!");

            Assert.LessOrEqual(
                pathFollower.CurrentVelocity.y,
                pathFollower.MaxClimbRate + 0.001f,
                $"Step {i}: 3D velocity Y {pathFollower.CurrentVelocity.y:F3} exceeded MaxClimbRate {pathFollower.MaxClimbRate}!");

            if (pathFollower.CurrentVerticalSpeed > 0.5f)
            {
                reachedHighClimbSpeed = true;
            }
        }

        Assert.IsTrue(reachedHighClimbSpeed, "UAV should accelerate vertically during climb!");
    }

    [Test]
    public void VerticalKinematics_DescentRate_ClampedToConfiguredMaximum()
    {
        uavObj.transform.position = new Vector3(0f, 5.0f, 0f);
        pathFollower.SetTargetAltitude(1.0f);

        List<Node> path = new List<Node>
        {
            gridManager.NodeFromWorldPoint(new Vector3(0f, 0f, 20f))
        };
        pathFollower.StartFollowing(path);

        bool reachedHighDescentSpeed = false;
        for (int i = 0; i < 30; i++)
        {
            StepMovement(0.1f);

            Assert.GreaterOrEqual(
                pathFollower.CurrentVerticalSpeed,
                -pathFollower.MaxDescentRate - 0.001f,
                $"Step {i}: Descent speed {pathFollower.CurrentVerticalSpeed:F3} exceeded MaxDescentRate -{pathFollower.MaxDescentRate}!");

            Assert.LessOrEqual(
                Mathf.Abs(pathFollower.CurrentVelocity.y),
                pathFollower.MaxDescentRate + 0.001f,
                $"Step {i}: 3D velocity descent magnitude {Mathf.Abs(pathFollower.CurrentVelocity.y):F3} exceeded MaxDescentRate!");

            if (pathFollower.CurrentVerticalSpeed < -0.5f)
            {
                reachedHighDescentSpeed = true;
            }
        }

        Assert.IsTrue(reachedHighDescentSpeed, "UAV should accelerate vertically during descent!");
    }

    [Test]
    public void VerticalKinematics_SmoothAccelerationAndDeceleration_NoDiscontinuousVelocitySteps()
    {
        uavObj.transform.position = new Vector3(0f, 1.0f, 0f);
        pathFollower.SetTargetAltitude(4.0f);

        List<Node> path = new List<Node>
        {
            gridManager.NodeFromWorldPoint(new Vector3(0f, 0f, 20f))
        };
        pathFollower.StartFollowing(path);

        float maxAccelPerStep = Mathf.Max(pathFollower.VerticalAcceleration, pathFollower.VerticalDeceleration) * 0.05f + 0.001f;
        float prevVerticalSpeed = pathFollower.CurrentVerticalSpeed;

        for (int i = 0; i < 40; i++)
        {
            StepMovement(0.05f);
            float currentVy = pathFollower.CurrentVerticalSpeed;
            float stepAccel = Mathf.Abs(currentVy - prevVerticalSpeed);

            Assert.LessOrEqual(
                stepAccel,
                maxAccelPerStep,
                $"Step {i}: Velocity step delta {stepAccel:F4} exceeded maximum allowable acceleration per step {maxAccelPerStep:F4}!");

            prevVerticalSpeed = currentVy;
        }

        // Verify altitude safely approaches target without violent overshoot
        Assert.LessOrEqual(uavObj.transform.position.y, 4.05f, "Altitude should not overshoot target altitude + 4.0m!");
    }

    [Test]
    public void VerticalKinematics_AltitudeClamping_RespectsMinMaxFlightCeilings()
    {
        pathFollower.MinFlightAltitude = 1.0f;
        pathFollower.MaxFlightAltitude = 6.0f;

        // Command altitude above ceiling
        pathFollower.SetTargetAltitude(10.0f);
        Assert.AreEqual(6.0f, pathFollower.TargetAltitude, 0.001f, "Target altitude should be clamped to MaxFlightAltitude!");

        // Command altitude below floor
        pathFollower.SetTargetAltitude(-5.0f);
        Assert.AreEqual(1.0f, pathFollower.TargetAltitude, 0.001f, "Target altitude should be clamped to MinFlightAltitude!");

        // Command nominal altitude within bounds
        pathFollower.SetTargetAltitude(3.5f);
        Assert.AreEqual(3.5f, pathFollower.TargetAltitude, 0.001f, "Target altitude should accept valid bounded altitude!");
    }

    [Test]
    public void VerticalKinematics_Simultaneous3DMovement_PreservesHorizontalSpeedAndYawAlignment()
    {
        uavObj.transform.position = new Vector3(0f, 1.0f, 0f);
        pathFollower.SetTargetAltitude(4.0f);

        // Path directed along +X
        List<Node> path = new List<Node>
        {
            gridManager.NodeFromWorldPoint(new Vector3(20f, 0f, 0f))
        };
        pathFollower.StartFollowing(path);

        for (int i = 0; i < 15; i++)
        {
            StepMovement(0.1f);
        }

        // Both horizontal displacement and vertical displacement must occur simultaneously
        Assert.Greater(uavObj.transform.position.x, 0.1f, "UAV should advance horizontally along X!");
        Assert.Greater(uavObj.transform.position.y, 1.05f, "UAV should climb vertically along Y!");

        // Current 3D velocity vector must contain both horizontal and vertical components
        Assert.Greater(pathFollower.CurrentVelocity.x, 0.01f, "3D velocity should have positive horizontal X component!");
        Assert.Greater(pathFollower.CurrentVelocity.y, 0.01f, "3D velocity should have positive vertical Y component!");

        // Visual pitch attitude must be applied (clamped within [-30, +30] deg)
        float pitch = uavObj.transform.eulerAngles.x;
        if (pitch > 180f) pitch -= 360f;
        Assert.LessOrEqual(Mathf.Abs(pitch), 30.1f, "Pitch attitude must be clamped within +/- 30 degrees!");
    }

    [Test]
    public void VerticalKinematics_ZeroAltitudeDelta_PreservesExactLegacyPlanarFlight()
    {
        uavObj.transform.position = new Vector3(0f, 1.0f, 0f);
        pathFollower.SetTargetAltitude(1.0f);

        List<Node> path = new List<Node>
        {
            gridManager.NodeFromWorldPoint(new Vector3(0f, 0f, 20f))
        };
        pathFollower.StartFollowing(path);

        for (int i = 0; i < 10; i++)
        {
            StepMovement(0.1f);

            Assert.AreEqual(1.0f, uavObj.transform.position.y, 0.0001f, "Altitude should remain strictly at 1.0m when delta is zero!");
            Assert.AreEqual(0f, pathFollower.CurrentVerticalSpeed, 0.0001f, "Vertical speed should remain 0 m/s!");
            Assert.AreEqual(0f, pathFollower.CurrentVelocity.y, 0.0001f, "3D velocity Y should remain 0 m/s!");
        }
    }
}
