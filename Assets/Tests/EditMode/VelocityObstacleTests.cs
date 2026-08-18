using NUnit.Framework;
using UnityEngine;

[TestFixture]
public class VelocityObstacleTests
{
    [Test]
    public void VelocityObstacle_CandidateVelocityDirectlyInside_ReturnsTrue()
    {
        Vector3 uavPos = Vector3.zero;
        Vector3 uavVel = new Vector3(0f, 0f, 2.0f); // Moving forward (+Z) at 2 m/s

        Vector3 obsPos = new Vector3(0f, 0f, 10.0f); // 10m ahead
        Vector3 obsVel = new Vector3(0f, 0f, -1.0f); // Approaching (-Z) at 1 m/s

        float combinedRadius = 2.0f; // 2m combined radius

        VelocityObstacle vo = CollisionPrediction.CalculateVelocityObstacle(uavPos, obsPos, obsVel, combinedRadius);

        Assert.IsTrue(vo.IsValid);
        Assert.AreEqual(10.0f, vo.Distance, 0.01f);
        Assert.AreEqual(obsVel, vo.Apex);
        Assert.Greater(vo.HalfAngleDeg, 10.0f);

        // UAV velocity is directly along the collision line
        bool inside = vo.ContainsVelocity(uavVel);
        Assert.IsTrue(inside, "Direct head-on relative velocity must be inside VO cone!");
    }

    [Test]
    public void VelocityObstacle_CandidateVelocityLateralOutside_ReturnsFalse()
    {
        Vector3 uavPos = Vector3.zero;
        Vector3 uavVel = new Vector3(3.0f, 0f, 0f); // Moving laterally right (+X) at 3 m/s

        Vector3 obsPos = new Vector3(0f, 0f, 10.0f); // 10m ahead
        Vector3 obsVel = Vector3.zero; // Stationary obstacle

        float combinedRadius = 1.5f;

        VelocityObstacle vo = CollisionPrediction.CalculateVelocityObstacle(uavPos, obsPos, obsVel, combinedRadius);

        bool inside = vo.ContainsVelocity(uavVel);
        Assert.IsFalse(inside, "Orthogonal lateral velocity outside cone angle must be false!");
    }

    [Test]
    public void VelocityObstacle_TangentialBoundaryVelocity_EvaluatesDeterministically()
    {
        Vector3 uavPos = Vector3.zero;
        Vector3 obsPos = new Vector3(0f, 0f, 10.0f);
        Vector3 obsVel = Vector3.zero;
        float combinedRadius = 2.0f;

        // sin(halfAngle) = 2/10 = 0.2
        // A velocity vector exactly at halfAngle:
        float halfAngleRad = Mathf.Asin(combinedRadius / 10.0f);
        Vector3 tangentVel = new Vector3(Mathf.Sin(halfAngleRad) * 2.0f, 0f, Mathf.Cos(halfAngleRad) * 2.0f);

        VelocityObstacle vo = CollisionPrediction.CalculateVelocityObstacle(uavPos, obsPos, obsVel, combinedRadius);

        // Tangential velocity is on the boundary and within tolerance
        bool inside = vo.ContainsVelocity(tangentVel);
        Assert.IsTrue(inside, "Tangential boundary velocity must evaluate deterministically as inside/tangent!");
    }

    [Test]
    public void VelocityObstacle_ObstacleApproachingUAV_CorrectlyClassified()
    {
        Vector3 uavPos = Vector3.zero;
        Vector3 uavVel = Vector3.zero; // Stationary UAV

        Vector3 obsPos = new Vector3(0f, 0f, 12.0f);
        Vector3 obsVel = new Vector3(0f, 0f, -2.0f); // Approaching UAV at 2 m/s

        float combinedRadius = 1.5f;

        // UAV velocity (0,0,0) relative to obs velocity (0,0,-2) gives v_rel = (0,0,2) closing toward obstacle
        bool isThreat = CollisionPrediction.IsVelocityInsideObstacle(uavPos, uavVel, obsPos, obsVel, combinedRadius);
        Assert.IsTrue(isThreat, "Approaching obstacle on direct collision course must be classified inside VO!");
    }

    [Test]
    public void VelocityObstacle_ObstacleMovingAway_CorrectlyClassifiedAsOutsideVO()
    {
        Vector3 uavPos = Vector3.zero;
        Vector3 uavVel = new Vector3(0f, 0f, 1.0f); // UAV cruising forward at 1 m/s

        Vector3 obsPos = new Vector3(0f, 0f, 10.0f);
        Vector3 obsVel = new Vector3(0f, 0f, 3.0f); // Obstacle moving away faster at 3 m/s

        float combinedRadius = 2.0f;

        // Relative velocity v_rel = 1 - 3 = -2 m/s (diverging)
        bool inside = CollisionPrediction.IsVelocityInsideObstacle(uavPos, uavVel, obsPos, obsVel, combinedRadius);
        Assert.IsFalse(inside, "Diverging obstacle pulling away must be outside VO cone!");
    }

    [Test]
    public void VelocityObstacle_ZeroRelativeVelocity_SafelyHandledWithoutCollision()
    {
        Vector3 uavPos = Vector3.zero;
        Vector3 uavVel = new Vector3(0f, 0f, 2.0f);

        Vector3 obsPos = new Vector3(5.0f, 0f, 10.0f);
        Vector3 obsVel = new Vector3(0f, 0f, 2.0f); // Same velocity (parallel formation)

        float combinedRadius = 1.5f;

        // v_rel = 0, distance = 11.18m > 1.5m
        bool inside = CollisionPrediction.IsVelocityInsideObstacle(uavPos, uavVel, obsPos, obsVel, combinedRadius);
        Assert.IsFalse(inside, "Zero relative velocity with safe separation must not report collision!");
    }

    [Test]
    public void VelocityObstacle_ZeroDistanceOverlap_SafelyHandledWithoutNaNOrInfinity()
    {
        Vector3 uavPos = new Vector3(5f, 1f, 5f);
        Vector3 uavVel = new Vector3(1f, 0f, 1f);

        Vector3 obsPos = new Vector3(5f, 1f, 5f); // Identical position (zero distance)
        Vector3 obsVel = Vector3.zero;

        float combinedRadius = 2.0f;

        VelocityObstacle vo = CollisionPrediction.CalculateVelocityObstacle(uavPos, obsPos, obsVel, combinedRadius);

        Assert.IsTrue(vo.IsValid);
        Assert.IsFalse(float.IsNaN(vo.HalfAngleDeg));
        Assert.IsFalse(float.IsInfinity(vo.HalfAngleDeg));

        bool inside = vo.ContainsVelocity(uavVel);
        Assert.IsTrue(inside, "Overlapping position must evaluate as collision without NaN/exceptions!");
    }

    [Test]
    public void VelocityObstacle_LargeSeparationBeyondLookaheadHorizon_ReturnsFalse()
    {
        Vector3 uavPos = Vector3.zero;
        Vector3 uavVel = new Vector3(0f, 0f, 2.0f);

        Vector3 obsPos = new Vector3(0f, 0f, 500.0f); // 500m ahead
        Vector3 obsVel = Vector3.zero;

        float combinedRadius = 2.0f;
        float lookaheadTime = 10.0f; // 10s horizon (reaches only 20m)

        // Will reach in 250s, but lookahead is only 10s
        bool inside = CollisionPrediction.IsVelocityInsideObstacle(
            uavPos, uavVel, obsPos, obsVel, combinedRadius, maxLookaheadTime: lookaheadTime);

        Assert.IsFalse(inside, "Threat beyond lookahead horizon must be excluded by truncated VO!");
    }
}
