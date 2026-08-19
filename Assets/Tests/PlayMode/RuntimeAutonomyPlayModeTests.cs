using System.Collections;
using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

/// <summary>
/// Phase 4 Step 4.5: PlayMode End-to-End Mission Verification.
/// Executes live autonomy stack across Unity frame updates to validate end-to-end mission lifecycle,
/// state transitions (Pending -> Navigating -> Completed), physical waypoint traversal, and telemetry.
/// </summary>
public class RuntimeAutonomyPlayModeTests
{
    private GameObject uavObj;

    [TearDown]
    public void TearDown()
    {
        if (uavObj != null)
        {
            Object.DestroyImmediate(uavObj);
            uavObj = null;
        }
    }

    [UnityTest]
    public IEnumerator Mission_FullLifecycle_TransitionsPendingToNavigatingToCompleted()
    {
        // 1. Instantiate test-controlled UAV with complete autonomy stack
        Vector3 spawnPos = new Vector3(0f, 1f, 0f);
        uavObj = GameManagerBootstrapper.CreateUav(spawnPos);

        PathFollower pathFollower = uavObj.GetComponent<PathFollower>();
        MissionManager missionManager = uavObj.GetComponent<MissionManager>();

        Assert.IsNotNull(pathFollower, "UAV must be equipped with PathFollower!");
        Assert.IsNotNull(missionManager, "UAV must be equipped with MissionManager!");

        // 2. Initial state verification
        Assert.AreEqual(MissionState.Pending, missionManager.State, "Initial mission state must be Pending prior to path engagement!");
        Assert.IsFalse(missionManager.Result.HasValue, "MissionResult must be null while mission is pending!");

        // 3. Configure short deterministic 2-waypoint mission (2.0 meters along Z-axis)
        Node wp1 = new Node(true, new Vector3(0f, 1f, 1.0f), 0, 1);
        Node wp2 = new Node(true, new Vector3(0f, 1f, 2.0f), 0, 2);
        List<Node> missionPath = new List<Node> { wp1, wp2 };

        // 4. Start mission flight
        pathFollower.StartFollowing(missionPath);

        // 5. Verify transition to Navigating on first frame
        yield return null;
        Assert.AreEqual(MissionState.Navigating, missionManager.State, "MissionManager must transition to Navigating upon path follower engagement!");
        Assert.IsTrue(pathFollower.IsFollowing, "PathFollower must be actively following the route!");

        // 6. Bounded frame execution with hard loop timeout safety (max 300 frames ~ 5-6 seconds at 50-60 FPS)
        const int maxFrames = 300;
        int frameCount = 0;

        while (missionManager.State != MissionState.Completed && frameCount < maxFrames)
        {
            frameCount++;
            yield return null;
        }

        // 7. Verify mission completed within timeout bound
        Assert.Less(frameCount, maxFrames, "Mission must complete within the maximum allowed frame count!");
        Assert.AreEqual(MissionState.Completed, missionManager.State, "Mission must achieve Completed state!");

        // 8. Verify structured MissionResult telemetry
        Assert.IsTrue(missionManager.Result.HasValue, "MissionResult must be populated upon completion!");
        MissionResult result = missionManager.Result.Value;
        Assert.IsTrue(result.IsSuccess, "MissionResult.IsSuccess must be true upon reaching destination!");
        Assert.AreEqual(MissionState.Completed, result.FinalState, "Final state in result must be Completed!");
        Assert.Greater(result.TotalDistanceTraveled, 1.5f, "TotalDistanceTraveled must record physical displacement >= 1.5m!");
        Assert.Greater(result.TotalFlightTime, 0.05f, "TotalFlightTime must be positive non-zero!");

        // 9. Verify physical arrival at target waypoint
        Assert.AreEqual(0f, uavObj.transform.position.x, 0.2f);
        Assert.AreEqual(1f, uavObj.transform.position.y, 0.2f);
        Assert.AreEqual(2.0f, uavObj.transform.position.z, 0.2f);
    }
}
