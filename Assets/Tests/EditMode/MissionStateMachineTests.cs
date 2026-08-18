using NUnit.Framework;
using UnityEngine;

[TestFixture]
public class MissionStateMachineTests
{
    private GameObject managerObj;
    private MissionManager missionManager;

    [SetUp]
    public void SetUp()
    {
        managerObj = new GameObject("TestMissionManager");
        missionManager = managerObj.AddComponent<MissionManager>();
    }

    [TearDown]
    public void TearDown()
    {
        if (managerObj != null)
        {
            Object.DestroyImmediate(managerObj);
        }
    }

    [Test]
    public void MissionManager_InitialState_IsPending()
    {
        Assert.AreEqual(MissionState.Pending, missionManager.State);
        Assert.IsFalse(missionManager.IsActive);
        Assert.IsNull(missionManager.Result);
        Assert.IsNull(missionManager.Score);
    }

    [Test]
    public void MissionResult_Constructor_PreservesCompleteTelemetryState()
    {
        MissionResult result = new MissionResult(
            isSuccess: true,
            finalState: MissionState.Completed,
            totalFlightTime: 22.5f,
            totalDistanceTraveled: 32.0f,
            plannedPathDistance: 27.5f,
            totalReplans: 1,
            totalThreatEncounters: 5,
            criticalThreatCount: 1,
            minimumClearanceObserved: 2.85f,
            pathEfficiency: 0.859f);

        Assert.IsTrue(result.IsSuccess);
        Assert.AreEqual(MissionState.Completed, result.FinalState);
        Assert.AreEqual(22.5f, result.TotalFlightTime, 0.01f);
        Assert.AreEqual(32.0f, result.TotalDistanceTraveled, 0.01f);
        Assert.AreEqual(27.5f, result.PlannedPathDistance, 0.01f);
        Assert.AreEqual(1, result.TotalReplans);
        Assert.AreEqual(5, result.TotalThreatEncounters);
        Assert.AreEqual(1, result.CriticalThreatCount);
        Assert.AreEqual(2.85f, result.MinimumClearanceObserved, 0.01f);
        Assert.AreEqual(0.859f, result.PathEfficiency, 0.001f);
    }

    [Test]
    public void MissionState_EnumValues_ContainExpectedLifecycleStates()
    {
        Assert.AreEqual(0, (int)MissionState.Pending);
        Assert.AreEqual(1, (int)MissionState.Navigating);
        Assert.AreEqual(2, (int)MissionState.Rerouting);
        Assert.AreEqual(3, (int)MissionState.Completed);
        Assert.AreEqual(4, (int)MissionState.Failed);
    }
}
