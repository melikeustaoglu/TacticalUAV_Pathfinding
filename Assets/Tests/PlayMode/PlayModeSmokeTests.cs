using System.Collections;
using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

/// <summary>
/// Minimal deterministic PlayMode smoke test to validate test-harness isolation,
/// clean scene initialization, test-controlled UAV instantiation, and frame stepping.
/// </summary>
public class PlayModeSmokeTests
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
    public IEnumerator PlayMode_SmokeTest_InitializesIsolatedSceneAndStepsFrames()
    {
        // 1. Verify no unwanted background PathfindingSystem was spawned
        GameObject backgroundSystem = GameObject.Find("PathfindingSystem");
        Assert.IsNull(backgroundSystem, "Background PathfindingSystem must NOT be auto-spawned in automated test scenes!");

        // 2. Instantiate test UAV
        Vector3 spawnPos = new Vector3(0f, 1f, 0f);
        uavObj = GameManagerBootstrapper.CreateUav(spawnPos);
        Assert.IsNotNull(uavObj, "GameManagerBootstrapper.CreateUav must successfully create UAV instance!");

        // 3. Verify exactly 1 UAV exists in the test scene via its unique MissionManager component
        MissionManager[] activeUavs = Object.FindObjectsByType<MissionManager>(FindObjectsSortMode.None);
        Assert.AreEqual(1, activeUavs.Length, "Exactly ONE UAV instance must exist in the isolated test scene!");
        Assert.AreEqual(uavObj, activeUavs[0].gameObject, "Active UAV component must belong to the test-created UAV instance!");

        // 4. Yield across real Unity frame
        yield return null;

        // 5. Verify UAV remains valid and maintains expected transform
        Assert.IsNotNull(uavObj, "UAV GameObject must remain valid after frame yield!");
        Assert.AreEqual(spawnPos.x, uavObj.transform.position.x, 0.01f);
        Assert.AreEqual(spawnPos.y, uavObj.transform.position.y, 0.01f);
        Assert.AreEqual(spawnPos.z, uavObj.transform.position.z, 0.01f);
    }
}
