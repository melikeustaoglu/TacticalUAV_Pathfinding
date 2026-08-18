using NUnit.Framework;
using UnityEngine;

[TestFixture]
public class ThreatAssessmentTests
{
    [Test]
    public void ThreatReport_Clear_InitializesWithDefaultNoneValues()
    {
        ThreatReport clearReport = ThreatReport.Clear;

        Assert.AreEqual(ThreatLevel.None, clearReport.ThreatLevel);
        Assert.IsTrue(float.IsPositiveInfinity(clearReport.DistanceToCollision));
        Assert.IsTrue(float.IsPositiveInfinity(clearReport.TimeToCollision));
        Assert.AreEqual(-1, clearReport.ObstructedWaypointIndex);
    }

    [Test]
    public void ThreatReport_Constructor_CorrectlyStoresAllTelemetry()
    {
        Vector3 collisionPoint = new Vector3(5f, 1f, 10f);
        ThreatReport report = new ThreatReport(
            threatLevel: ThreatLevel.Critical,
            threateningObstacle: default,
            estimatedCollisionPoint: collisionPoint,
            distanceToCollision: 8.5f,
            timeToCollision: 4.25f,
            obstructedWaypointIndex: 2);

        Assert.AreEqual(ThreatLevel.Critical, report.ThreatLevel);
        Assert.AreEqual(collisionPoint, report.EstimatedCollisionPoint);
        Assert.AreEqual(8.5f, report.DistanceToCollision, 0.01f);
        Assert.AreEqual(4.25f, report.TimeToCollision, 0.01f);
        Assert.AreEqual(2, report.ObstructedWaypointIndex);
    }
}
