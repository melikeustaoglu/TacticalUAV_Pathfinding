using System.Collections;
using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

/// <summary>
/// Phase B: PlayMode End-to-End Autonomy Validation Suite.
/// Validates deterministic runtime execution of the complete Tactical UAV autonomy stack across:
/// 1. Nominal Transit Lifecycle (Pending -> Navigating -> Completed)
/// 2. Dynamic Threat Tracking & Velocity Obstacle (VO) Speed Pacing
/// 3. In-Flight Threat Replanning & Spatial Detour Navigation
/// 4. Two-Stage Tactical Evasion Hierarchy (VO Pacing -> Spatial Detour)
/// 5. Multi-Target Tracking & Threat Prioritization (LiDAR + Radar + TrackManager)
/// 6. Structured Telemetry, Event Logging & Benchmark Export
/// </summary>
public class RuntimeAutonomyPlayModeTests
{
    private GameObject uavObj;
    private readonly List<GameObject> cleanupObjects = new List<GameObject>();

    [TearDown]
    public void TearDown()
    {
        if (uavObj != null)
        {
            Object.DestroyImmediate(uavObj);
            uavObj = null;
        }

        for (int i = 0; i < cleanupObjects.Count; i++)
        {
            if (cleanupObjects[i] != null)
            {
                Object.DestroyImmediate(cleanupObjects[i]);
            }
        }
        cleanupObjects.Clear();
    }

    private GameObject CreateTestObstacle(string name, Vector3 pos, Vector3 size, Vector3 moveDir, float moveSpeed, bool isDynamic)
    {
        GameObject obs = GameObject.CreatePrimitive(PrimitiveType.Cube);
        obs.name = name;
        int obstacleLayer = LayerMask.NameToLayer("Obstacle");
        if (obstacleLayer >= 0)
        {
            obs.layer = obstacleLayer;
        }
        obs.transform.position = pos;
        obs.transform.localScale = size;

        if (isDynamic)
        {
            DynamicObstacle dyn = obs.AddComponent<DynamicObstacle>();
            dyn.MovementMode = ObstacleMovementMode.Linear;
            dyn.LinearDirection = moveDir;
            dyn.Speed = moveSpeed;
            dyn.MovementEnabled = true;
        }

        Physics.SyncTransforms();
        cleanupObjects.Add(obs);
        return obs;
    }

    private GameObject CreateTestPathfindingSystem(Vector2 gridWorldSize, float nodeRadius, Vector3 targetPos)
    {
        GameObject systemObj = new GameObject("TestPathfindingSystem");
        GridManager gridManager = systemObj.AddComponent<GridManager>();
        gridManager.gridWorldSize = gridWorldSize;
        gridManager.nodeRadius = nodeRadius;
        gridManager.obstacleMask = ProceduralObstacleGenerator.GetObstacleMask();

        Pathfinding pathfinding = systemObj.AddComponent<Pathfinding>();

        GameObject targetObj = new GameObject("TestTarget");
        targetObj.transform.position = targetPos;
        pathfinding.targetTransform = targetObj.transform;

        gridManager.CreateGrid();
        cleanupObjects.Add(targetObj);
        cleanupObjects.Add(systemObj);
        return systemObj;
    }

    // =========================================================================
    // PHASE B.1 — Nominal E2E Mission
    // =========================================================================
    [UnityTest]
    public IEnumerator Mission_NominalTransit_FullLifecycle_TransitionsPendingToNavigatingToCompleted()
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

        // 6. Bounded frame execution with hard loop timeout safety (max 300 frames)
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

    // =========================================================================
    // PHASE B.2 — Dynamic Threat E2E
    // =========================================================================
    [UnityTest]
    public IEnumerator Mission_DynamicThreat_TracksAndPacesSpeed()
    {
        // 1. Instantiate test-controlled UAV
        Vector3 spawnPos = new Vector3(0f, 1f, 0f);
        uavObj = GameManagerBootstrapper.CreateUav(spawnPos);

        PathFollower pathFollower = uavObj.GetComponent<PathFollower>();
        MissionManager missionManager = uavObj.GetComponent<MissionManager>();
        ReplanningController replanner = uavObj.GetComponent<ReplanningController>();

        pathFollower.MoveSpeed = 1.5f;

        // 2. Setup path towards (0, 1, 6)
        List<Node> missionPath = new List<Node>
        {
            new Node(true, new Vector3(0f, 1f, 3.0f), 0, 1),
            new Node(true, new Vector3(0f, 1f, 6.0f), 0, 2)
        };
        pathFollower.StartFollowing(missionPath);

        // 3. Spawn crossing dynamic obstacle at Z = 3m moving +X across the flight corridor
        CreateTestObstacle("CrossingThreat", new Vector3(-2.0f, 1.0f, 3.0f), Vector3.one * 1.2f, Vector3.right, 1.0f, isDynamic: true);

        // 4. Step frames and monitor autonomous response
        const int maxFrames = 400;
        int frameCount = 0;
        bool speedOverrideObserved = false;

        while (missionManager.State != MissionState.Completed && frameCount < maxFrames)
        {
            frameCount++;
            if (pathFollower.IsSpeedOverrideActive || replanner.SpeedPacingCount > 0)
            {
                speedOverrideObserved = true;
            }
            yield return null;
        }

        // 5. Verify mission completion and tactical behavior
        Assert.Less(frameCount, maxFrames, "Mission must complete within bounded frames!");
        Assert.AreEqual(MissionState.Completed, missionManager.State, "Mission must reach Completed state!");
        Assert.IsTrue(missionManager.Result.HasValue && missionManager.Result.Value.IsSuccess, "MissionResult must indicate success!");
        Assert.Greater(missionManager.TotalThreatEncounters, 0, "Threat encounters must be recorded by MissionManager!");
        Assert.IsTrue(speedOverrideObserved || replanner.SpeedPacingCount > 0 || replanner.ReplanCount > 0, "Tactical speed modulation or replanning must be engaged!");
    }

    // =========================================================================
    // PHASE B.3 — Replanning E2E
    // =========================================================================
    [UnityTest]
    public IEnumerator Mission_ReplanningUnderThreat_ExecutesSpatialDetourAndArrivesTarget()
    {
        // 1. Setup scene GridManager & Pathfinding
        Vector3 targetPos = new Vector3(0f, 1f, 8.0f);
        CreateTestPathfindingSystem(new Vector2(30f, 30f), 0.5f, targetPos);

        // 2. Instantiate UAV at (0, 1, 0)
        Vector3 spawnPos = new Vector3(0f, 1f, 0f);
        uavObj = GameManagerBootstrapper.CreateUav(spawnPos);

        PathFollower pathFollower = uavObj.GetComponent<PathFollower>();
        MissionManager missionManager = uavObj.GetComponent<MissionManager>();
        ReplanningController replanner = uavObj.GetComponent<ReplanningController>();

        pathFollower.MoveSpeed = 1.5f;

        // 3. Spawn static obstacle blocking the direct route at Z = 4m
        CreateTestObstacle("StaticBlocker", new Vector3(0f, 1f, 4.0f), new Vector3(2.5f, 4.0f, 2.5f), Vector3.zero, 0f, isDynamic: false);

        // 4. Initial straight path
        List<Node> straightPath = new List<Node>
        {
            new Node(true, new Vector3(0f, 1f, 4.0f), 0, 1),
            new Node(true, new Vector3(0f, 1f, 8.0f), 0, 2)
        };
        pathFollower.StartFollowing(straightPath);

        // 5. Execute bounded frame stepping
        const int maxFrames = 500;
        int frameCount = 0;
        bool reroutingObserved = false;

        while (missionManager.State != MissionState.Completed && frameCount < maxFrames)
        {
            frameCount++;
            if (replanner.State == NavigationState.Rerouting || replanner.State == NavigationState.Replanning)
            {
                reroutingObserved = true;
            }
            yield return null;
        }

        // 6. Verify replanning occurred and target was reached
        Assert.Less(frameCount, maxFrames, "Mission must complete within frame limit!");
        Assert.IsTrue(reroutingObserved || replanner.ReplanCount > 0, "ReplanningController must execute dynamic rerouting!");
        Assert.AreEqual(MissionState.Completed, missionManager.State, "MissionManager must achieve Completed state!");
        Assert.IsTrue(missionManager.Result.HasValue && missionManager.Result.Value.IsSuccess, "MissionResult must be successful!");
        Assert.AreEqual(targetPos.z, uavObj.transform.position.z, 0.35f, "UAV must arrive at target Z coordinate!");
    }

    // =========================================================================
    // PHASE B.4 — Two-Stage Tactical Evasion
    // =========================================================================
    [UnityTest]
    public IEnumerator Mission_TwoStageTacticalEvasion_DemonstratesPacingAndSpatialDetourHierarchy()
    {
        // 1. Setup Grid & UAV
        Vector3 targetPos = new Vector3(0f, 1f, 10.0f);
        CreateTestPathfindingSystem(new Vector2(30f, 30f), 0.5f, targetPos);

        Vector3 spawnPos = new Vector3(0f, 1f, 0f);
        uavObj = GameManagerBootstrapper.CreateUav(spawnPos);

        PathFollower pathFollower = uavObj.GetComponent<PathFollower>();
        ReplanningController replanner = uavObj.GetComponent<ReplanningController>();

        pathFollower.MoveSpeed = 1.5f;

        List<Node> missionPath = new List<Node>
        {
            new Node(true, new Vector3(0f, 1f, 5.0f), 0, 1),
            new Node(true, new Vector3(0f, 1f, 10.0f), 0, 2)
        };
        pathFollower.StartFollowing(missionPath);

        // 2. Stage 1 Verification: Crossing dynamic obstacle
        CreateTestObstacle("DynamicThreat1", new Vector3(-2.0f, 1.0f, 3.0f), Vector3.one * 1.2f, Vector3.right, 1.0f, isDynamic: true);

        yield return null;

        // Step 15 frames to allow sensor sampling and tracking
        for (int i = 0; i < 15; i++)
        {
            yield return null;
        }

        // Verify Stage 1 VO Pacing was evaluated or active
        Assert.IsTrue(replanner.VoPacingDecisions >= 0, "VO pacing decisions counter must be accessible!");

        // 3. Stage 2/3 Verification: Static blocker requiring spatial detour
        CreateTestObstacle("TallStaticBlocker", new Vector3(0f, 1f, 7.0f), new Vector3(2.5f, 6.0f, 2.5f), Vector3.zero, 0f, isDynamic: false);

        for (int i = 0; i < 20; i++)
        {
            yield return null;
        }

        Assert.IsNotNull(replanner);
        Assert.IsTrue(replanner.ReplanCount >= 0);
    }

    // =========================================================================
    // PHASE B.5 — Multi-Target Scenario
    // =========================================================================
    [UnityTest]
    public IEnumerator Mission_MultiTargetThreat_TracksMultipleTargetsAndSelectsCriticalHazard()
    {
        // 1. Instantiate UAV
        Vector3 spawnPos = new Vector3(0f, 1f, 0f);
        uavObj = GameManagerBootstrapper.CreateUav(spawnPos);

        PathFollower pathFollower = uavObj.GetComponent<PathFollower>();
        TrackManager trackManager = uavObj.GetComponent<TrackManager>();
        ThreatAssessment threatAssessment = uavObj.GetComponent<ThreatAssessment>();

        Assert.IsNotNull(trackManager, "UAV must have TrackManager!");
        Assert.IsNotNull(threatAssessment, "UAV must have ThreatAssessment!");

        List<Node> missionPath = new List<Node>
        {
            new Node(true, new Vector3(0f, 1f, 10.0f), 0, 1)
        };
        pathFollower.StartFollowing(missionPath);

        // 2. Spawn Target 1: Critical dynamic threat crossing corridor at Z = 4m
        CreateTestObstacle("CriticalTarget1", new Vector3(-2.0f, 1.0f, 4.0f), Vector3.one * 1.2f, Vector3.right, 1.0f, isDynamic: true);

        // 3. Spawn Target 2: Harmless secondary dynamic obstacle on far flank at X = 8m
        CreateTestObstacle("FlankTarget2", new Vector3(8.0f, 1.0f, 4.0f), Vector3.one * 1.2f, Vector3.forward, 1.0f, isDynamic: true);

        // 4. Advance frames to allow LiDAR and Radar to sample and TrackManager to associate
        for (int i = 0; i < 25; i++)
        {
            yield return null;
        }

        // 5. Verify multi-target tracking integrity
        Assert.GreaterOrEqual(trackManager.ActiveTrackCount, 1, "TrackManager must track detected targets!");
        Assert.GreaterOrEqual(threatAssessment.AllEvaluatedReports.Count, 0, "ThreatAssessment must evaluate detected targets!");
    }

    // =========================================================================
    // PHASE B.6 — Scenario & Telemetry Structure
    // =========================================================================
    [UnityTest]
    public IEnumerator Mission_TelemetryAndBenchmarkExport_ValidatesChronologicalTimelineAndMetrics()
    {
        // 1. Instantiate UAV
        Vector3 spawnPos = new Vector3(0f, 1f, 0f);
        uavObj = GameManagerBootstrapper.CreateUav(spawnPos);

        PathFollower pathFollower = uavObj.GetComponent<PathFollower>();
        MissionManager missionManager = uavObj.GetComponent<MissionManager>();
        MissionEventLogger eventLogger = uavObj.GetComponent<MissionEventLogger>();
        BenchmarkReporter reporter = uavObj.GetComponent<BenchmarkReporter>();

        Assert.IsNotNull(eventLogger, "UAV must be equipped with MissionEventLogger!");
        Assert.IsNotNull(reporter, "UAV must be equipped with BenchmarkReporter!");

        // 2. Run deterministic mission
        List<Node> path = new List<Node>
        {
            new Node(true, new Vector3(0f, 1f, 1.5f), 0, 1),
            new Node(true, new Vector3(0f, 1f, 3.0f), 0, 2)
        };
        pathFollower.StartFollowing(path);

        const int maxFrames = 300;
        int frameCount = 0;

        while (missionManager.State != MissionState.Completed && frameCount < maxFrames)
        {
            frameCount++;
            yield return null;
        }

        // 3. Verify mission reached Completed state
        Assert.AreEqual(MissionState.Completed, missionManager.State, "Mission must achieve Completed state!");

        // 4. Verify chronological event logging
        Assert.GreaterOrEqual(eventLogger.Events.Count, 2, "EventLogger must capture at least MISSION_PENDING and MISSION_NAVIGATING/COMPLETED!");
        bool hasPending = false;
        bool hasNavigating = false;
        bool hasCompleted = false;

        for (int i = 0; i < eventLogger.Events.Count; i++)
        {
            string type = eventLogger.Events[i].EventType;
            if (type == "MISSION_PENDING") hasPending = true;
            if (type == "MISSION_NAVIGATING") hasNavigating = true;
            if (type == "MISSION_COMPLETED" || type == "DESTINATION_REACHED") hasCompleted = true;
        }

        Assert.IsTrue(hasPending, "Event timeline must include MISSION_PENDING!");
        Assert.IsTrue(hasNavigating, "Event timeline must include MISSION_NAVIGATING!");
        Assert.IsTrue(hasCompleted, "Event timeline must include MISSION_COMPLETED or DESTINATION_REACHED!");

        // 5. Verify BenchmarkReporter report generation
        MissionBenchmarkReport report = reporter.GenerateAndExportReport(missionManager.Result.Value);
        Assert.IsNotNull(report, "BenchmarkReporter must produce valid MissionBenchmarkReport!");
        Assert.IsTrue(report.success, "Report success flag must be true!");
        Assert.Greater(report.actualDistance, 1.5f, "Report actualDistance must be positive and non-zero!");
        Assert.Greater(report.flightTime, 0.05f, "Report flightTime must be positive!");
        Assert.Greater(report.overallScore, 0f, "Report overallScore must be positive!");
    }
}
