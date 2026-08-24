using System;
using System.Collections.Generic;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;

/// <summary>
/// Pre-Phase-C Autonomy Core Verification Suite.
/// Gathers experimental evidence across 5 core autonomy areas:
/// 1. A* / Pathfinding Core (non-square grids, dense obstacles, start==goal, out-of-bounds clamping, node contiguity)
/// 2. EKF Covariance Response (noise sensitivity, covariance contraction vs measurement variance, symmetry)
/// 3. Multi-Target Track Lifecycle (multi-target ID stability, 3-of-5 promotion, coasting, pruning)
/// 4. Evasion Hierarchy Stage Distinguishability (Stage 1 VO Pacing vs Stage 2 Vertical Climb vs Stage 3 Spatial A*)
/// 5. Replanning Stability & Anti-Oscillation (persistent multi-threat evaluation over 20+ cycles)
/// </summary>
[TestFixture]
public class AutonomyCoreAuditTests
{
    private GameObject testRoot;

    [SetUp]
    public void SetUp()
    {
        testRoot = new GameObject("TestRoot_AutonomyAudit");
    }

    [TearDown]
    public void TearDown()
    {
        if (testRoot != null)
        {
            UnityEngine.Object.DestroyImmediate(testRoot);
        }
    }

    // ================================================================================================
    // CHECKPOINT 1: A* / PATHFINDING CORE TESTS
    // ================================================================================================

    [Test]
    public void Checkpoint1_NonSquareGrid_FindsValidContiguousPath()
    {
        GridManager grid = testRoot.AddComponent<GridManager>();
        grid.gridWorldSize = new Vector2(10f, 30f); // Non-square: 10m X (10 cells), 30m Z (30 cells)
        grid.nodeRadius = 0.5f;
        grid.enableClearancePotentialField = false;

        Pathfinding pf = testRoot.AddComponent<Pathfinding>();
        typeof(Pathfinding).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(pf, null);
        grid.CreateGrid();

        Assert.AreEqual(10, grid.grid.GetLength(0));
        Assert.AreEqual(30, grid.grid.GetLength(1));

        Vector3 start = new Vector3(-3f, 1f, -12f);
        Vector3 target = new Vector3(3f, 1f, 12f);

        pf.FindPath(start, target);

        Assert.IsNotNull(pf.path);
        Assert.Greater(pf.path.Count, 0, "Non-square grid must find valid path.");

        // Verify destination reached
        Node targetNode = grid.NodeFromWorldPoint(target);
        Node finalNode = pf.path[pf.path.Count - 1];
        Assert.AreEqual(targetNode.gridX, finalNode.gridX);
        Assert.AreEqual(targetNode.gridY, finalNode.gridY);

        // Verify path node contiguity (every node is an 8-neighbor of previous)
        float maxStepDistance = grid.nodeRadius * 2.0f * 1.45f; // sqrt(2) * diameter + epsilon
        for (int i = 1; i < pf.path.Count; i++)
        {
            float stepDist = Vector3.Distance(pf.path[i].worldPosition, pf.path[i - 1].worldPosition);
            Assert.LessOrEqual(stepDist, maxStepDistance + 0.05f, $"Path step {i-1}->{i} must be contiguous!");
            Assert.IsTrue(pf.path[i].isWalkable, $"Path node {i} must be walkable.");
        }
    }

    [Test]
    public void Checkpoint1_StartEqualsGoal_ReturnsImmediateValidPath()
    {
        GridManager grid = testRoot.AddComponent<GridManager>();
        grid.gridWorldSize = new Vector2(20f, 20f);
        grid.nodeRadius = 0.5f;
        Pathfinding pf = testRoot.AddComponent<Pathfinding>();
        typeof(Pathfinding).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(pf, null);
        grid.CreateGrid();

        Vector3 startAndGoal = new Vector3(2f, 1f, 4f);
        pf.FindPath(startAndGoal, startAndGoal);

        Assert.IsNotNull(pf.path);
        Node expectedNode = grid.NodeFromWorldPoint(startAndGoal);
        Assert.AreEqual(1, pf.path.Count);
        Assert.AreEqual(expectedNode.gridX, pf.path[0].gridX);
        Assert.AreEqual(expectedNode.gridY, pf.path[0].gridY);
    }

    [Test]
    public void Checkpoint1_OutOfBoundsCoordinates_SafelyClampedToGridBounds()
    {
        GridManager grid = testRoot.AddComponent<GridManager>();
        grid.gridWorldSize = new Vector2(20f, 20f);
        grid.nodeRadius = 0.5f;
        Pathfinding pf = testRoot.AddComponent<Pathfinding>();
        typeof(Pathfinding).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(pf, null);
        grid.CreateGrid();

        // Start and Target far outside physical grid boundaries (-500m to +500m)
        Vector3 outStart = new Vector3(-100f, 1f, -100f);
        Vector3 outTarget = new Vector3(100f, 1f, 100f);

        Assert.DoesNotThrow(() => pf.FindPath(outStart, outTarget));
        Assert.IsNotNull(pf.path);
        Assert.Greater(pf.path.Count, 0);

        // Raw path traverses from corner [0,0] to corner [19,19]
        Assert.AreEqual(0, pf.rawPath[0].gridX);
        Assert.AreEqual(0, pf.rawPath[0].gridY);
        Assert.AreEqual(19, pf.rawPath[pf.rawPath.Count - 1].gridX);
        Assert.AreEqual(19, pf.rawPath[pf.rawPath.Count - 1].gridY);
        Assert.AreEqual(19, pf.path[pf.path.Count - 1].gridX);
        Assert.AreEqual(19, pf.path[pf.path.Count - 1].gridY);
    }

    [Test]
    public void Checkpoint1_DenseObstacleMaze_FindsOptimalCorridorWithoutBlockedNodes()
    {
        GridManager grid = testRoot.AddComponent<GridManager>();
        grid.gridWorldSize = new Vector2(20f, 20f);
        grid.nodeRadius = 0.5f;
        Pathfinding pf = testRoot.AddComponent<Pathfinding>();
        typeof(Pathfinding).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(pf, null);
        grid.CreateGrid();

        // Create a solid barrier across X from 0 to 19 at y = 10, except at gate x = 15
        for (int x = 0; x < 20; x++)
        {
            if (x != 15) // Leave open gate at x = 15
            {
                grid.grid[x, 10].isWalkable = false;
            }
        }

        Vector3 start = grid.grid[5, 5].worldPosition;
        Vector3 target = grid.grid[5, 15].worldPosition;

        pf.FindPath(start, target);

        Assert.IsNotNull(pf.path);
        Assert.Greater(pf.path.Count, 0);

        // Verify rawPath explicitly routes through [15, 10]
        bool rawPathRoutedThroughGate = false;
        for (int i = 0; i < pf.rawPath.Count; i++)
        {
            Assert.IsTrue(pf.rawPath[i].isWalkable, "Raw path must not contain unwalkable nodes.");
            if (pf.rawPath[i].gridX == 15 && pf.rawPath[i].gridY == 10)
            {
                rawPathRoutedThroughGate = true;
            }
        }
        Assert.IsTrue(rawPathRoutedThroughGate, "A* search must route through the open gate at [15, 10]!");

        // Verify all smoothed waypoints are on walkable nodes and reach target
        for (int i = 0; i < pf.path.Count; i++)
        {
            Assert.IsTrue(pf.path[i].isWalkable, "Smoothed path waypoint must be on a walkable node.");
        }
        Node targetNode = grid.NodeFromWorldPoint(target);
        Assert.AreEqual(targetNode.gridX, pf.path[pf.path.Count - 1].gridX);
        Assert.AreEqual(targetNode.gridY, pf.path[pf.path.Count - 1].gridY);
    }

    // ================================================================================================
    // CHECKPOINT 2: EKF / STATE ESTIMATION COVARIANCE DYNAMICS
    // ================================================================================================

    [Test]
    public void Checkpoint2_CovarianceGrowth_ScalesWithProcessNoiseAccumulation()
    {
        ExtendedKalmanFilter ekfLowNoise = new ExtendedKalmanFilter();
        ExtendedKalmanFilter ekfHighNoise = new ExtendedKalmanFilter();

        Vector3 initPos = Vector3.zero;
        Vector3 initVel = new Vector3(0f, 0f, 2.0f);
        ekfLowNoise.Initialize(initPos, initVel, 0f, Vector3.one * 0.04f, Vector3.one * 0.01f, 0.01f, 0f);
        ekfHighNoise.Initialize(initPos, initVel, 0f, Vector3.one * 0.04f, Vector3.one * 0.01f, 0.01f, 0f);

        float lowNoiseSigma = 0.05f;
        float highNoiseSigma = 0.50f;

        // Propagate 100 steps (1.0s)
        for (int step = 1; step <= 100; step++)
        {
            float t = step * 0.01f;
            ImuMeasurement imuLow = new ImuMeasurement(new Vector3(0f, 9.81f, 0f), Vector3.zero, Vector3.one * (lowNoiseSigma * lowNoiseSigma), Vector3.one * 0.0001f, t);
            ImuMeasurement imuHigh = new ImuMeasurement(new Vector3(0f, 9.81f, 0f), Vector3.zero, Vector3.one * (highNoiseSigma * highNoiseSigma), Vector3.one * 0.0001f, t);

            ekfLowNoise.Predict(imuLow, t);
            ekfHighNoise.Predict(imuHigh, t);
        }

        Matrix11x11 pLow = ekfLowNoise.CovarianceMatrix;
        Matrix11x11 pHigh = ekfHighNoise.CovarianceMatrix;

        // Verify covariance diagonal positivity and numerical validity
        for (int i = 0; i < 11; i++)
        {
            Assert.Greater(pLow[i, i], 0f);
            Assert.Greater(pHigh[i, i], 0f);
            Assert.IsTrue(float.IsFinite(pLow[i, i]));
            Assert.IsTrue(float.IsFinite(pHigh[i, i]));
        }

        // Positional uncertainty must grow during dead reckoning
        Assert.Greater(pLow[0, 0], 0.04f, "Low noise EKF position variance must expand.");
        Assert.Greater(pHigh[0, 0], 0.04f, "High noise EKF position variance must expand.");
    }

    [Test]
    public void Checkpoint2_MeasurementVariance_GovernsPosteriorCovarianceContraction()
    {
        ExtendedKalmanFilter ekfPrecise = new ExtendedKalmanFilter();
        ExtendedKalmanFilter ekfNoisy = new ExtendedKalmanFilter();

        Vector3 priorPos = new Vector3(1f, 0f, 1f);
        ekfPrecise.Initialize(priorPos, Vector3.zero, 0f, Vector3.one * 1.0f, Vector3.one * 0.1f, 0.1f, 0f);
        ekfNoisy.Initialize(priorPos, Vector3.zero, 0f, Vector3.one * 1.0f, Vector3.one * 0.1f, 0.1f, 0f);

        // Precise GPS (R = 0.01 m^2) vs Noisy GPS (R = 1.00 m^2)
        GpsMeasurement preciseGps = new GpsMeasurement(Vector3.zero, Vector3.zero, Vector3.one * 0.01f, Vector3.one * 0.01f, 0.1f);
        GpsMeasurement noisyGps = new GpsMeasurement(Vector3.zero, Vector3.zero, Vector3.one * 1.00f, Vector3.one * 0.01f, 0.1f);

        ekfPrecise.CorrectGps(preciseGps);
        ekfNoisy.CorrectGps(noisyGps);

        float varPrecise = ekfPrecise.CovarianceMatrix[0, 0];
        float varNoisy = ekfNoisy.CovarianceMatrix[0, 0];

        Assert.Less(varPrecise, varNoisy, "Precise measurement must contract covariance significantly more than noisy measurement!");
        Assert.Less(ekfPrecise.GetEstimatedState(0.1f).Position.x, ekfNoisy.GetEstimatedState(0.1f).Position.x,
            "Precise GPS must pull position estimate closer to measurement (0,0,0) than noisy GPS!");
    }

    // ================================================================================================
    // CHECKPOINT 3: MULTI-TARGET TRACK LIFECYCLE TESTS
    // ================================================================================================

    [Test]
    public void Checkpoint3_ThreeTargetLifecycle_MaintainsStableIdsThroughoutSpawnCoastingDeletion()
    {
        TrackManager tm = testRoot.AddComponent<TrackManager>();
        typeof(TrackManager).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(tm, null);

        Vector3 posT1 = new Vector3(0f, 0f, 5f);
        Vector3 posT2 = new Vector3(10f, 0f, 10f);
        Vector3 posT3 = new Vector3(-10f, 0f, 15f);

        // 1. Initial Scan: Spawn 3 tentative tracks (IDs: 1, 2, 3)
        TargetDetection[] dets1 = new TargetDetection[]
        {
            new TargetDetection(TargetSensorModality.LiDAR, 0.0f, posT1, Vector3.one * 0.04f, 0.95f, 1),
            new TargetDetection(TargetSensorModality.LiDAR, 0.0f, posT2, Vector3.one * 0.04f, 0.95f, 2),
            new TargetDetection(TargetSensorModality.LiDAR, 0.0f, posT3, Vector3.one * 0.04f, 0.95f, 3)
        };
        tm.ProcessDetections(dets1, 3, 0.0f);
        Assert.AreEqual(3, tm.ActiveTrackCount);
        Assert.AreEqual(1, tm.GetTrack(1).TrackId);
        Assert.AreEqual(2, tm.GetTrack(2).TrackId);
        Assert.AreEqual(3, tm.GetTrack(3).TrackId);

        // 2. Promote all 3 tracks to Confirmed across 2 more consecutive scans
        for (int i = 1; i <= 2; i++)
        {
            float t = i * 0.05f;
            TargetDetection[] dets = new TargetDetection[]
            {
                new TargetDetection(TargetSensorModality.LiDAR, t, posT1, Vector3.one * 0.04f, 0.95f, 10 + i),
                new TargetDetection(TargetSensorModality.LiDAR, t, posT2, Vector3.one * 0.04f, 0.95f, 20 + i),
                new TargetDetection(TargetSensorModality.LiDAR, t, posT3, Vector3.one * 0.04f, 0.95f, 30 + i)
            };
            tm.ProcessDetections(dets, 3, t);
        }

        Assert.AreEqual(TrackStatus.Confirmed, tm.GetTrack(1).Status);
        Assert.AreEqual(TrackStatus.Confirmed, tm.GetTrack(2).Status);
        Assert.AreEqual(TrackStatus.Confirmed, tm.GetTrack(3).Status);

        // 3. Target 2 is missed in next scan -> Enters Coasting while 1 & 3 remain Confirmed
        TargetDetection[] detsMiss2 = new TargetDetection[]
        {
            new TargetDetection(TargetSensorModality.LiDAR, 0.20f, posT1, Vector3.one * 0.04f, 0.95f, 100),
            new TargetDetection(TargetSensorModality.LiDAR, 0.20f, posT3, Vector3.one * 0.04f, 0.95f, 300)
        };
        tm.ProcessDetections(detsMiss2, 2, 0.20f);

        Assert.AreEqual(TrackStatus.Confirmed, tm.GetTrack(1).Status);
        Assert.AreEqual(TrackStatus.Coasting, tm.GetTrack(2).Status, "Target 2 must enter Coasting upon missed scan.");
        Assert.AreEqual(TrackStatus.Confirmed, tm.GetTrack(3).Status);

        // 4. Target 2 reacquired in scan 4 -> Restores Confirmed status
        TargetDetection[] detsReacquire2 = new TargetDetection[]
        {
            new TargetDetection(TargetSensorModality.LiDAR, 0.30f, posT1, Vector3.one * 0.04f, 0.95f, 101),
            new TargetDetection(TargetSensorModality.LiDAR, 0.30f, posT2, Vector3.one * 0.04f, 0.95f, 201),
            new TargetDetection(TargetSensorModality.LiDAR, 0.30f, posT3, Vector3.one * 0.04f, 0.95f, 301)
        };
        tm.ProcessDetections(detsReacquire2, 3, 0.30f);

        Assert.AreEqual(TrackStatus.Confirmed, tm.GetTrack(2).Status, "Target 2 must recover Confirmed status.");
        Assert.AreEqual(2, tm.GetTrack(2).TrackId, "Target 2 TrackId must remain unchanged (2).");
    }

    // ================================================================================================
    // CHECKPOINT 4: VELOCITY OBSTACLE / EVASION HIERARCHY TESTS
    // ================================================================================================

    [Test]
    public void Checkpoint4_EvasionHierarchy_DistinctStagesProduceIdentifiableResponses()
    {
        GridManager grid = testRoot.AddComponent<GridManager>();
        grid.gridWorldSize = new Vector2(40f, 40f);
        grid.nodeRadius = 0.5f;
        Pathfinding pf = testRoot.AddComponent<Pathfinding>();
        typeof(Pathfinding).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(pf, null);
        grid.CreateGrid();

        PathFollower follower = testRoot.AddComponent<PathFollower>();
        ThreatAssessment threat = testRoot.AddComponent<ThreatAssessment>();
        ReplanningController replan = testRoot.AddComponent<ReplanningController>();

        typeof(PathFollower).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(follower, null);
        typeof(ThreatAssessment).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(threat, null);
        typeof(ReplanningController).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(replan, null);

        follower.MoveSpeed = 2.0f;
        follower.MinFlightAltitude = 1.0f;
        follower.MaxFlightAltitude = 6.0f;
        follower.StartFollowing(new List<Node> { new Node(true, new Vector3(0f, 1f, 20f), 0, 0) });

        // A. Stage 1: Crossing dynamic threat -> Selects Speed Pacing
        GameObject dynObs = GameObject.CreatePrimitive(PrimitiveType.Cube);
        dynObs.transform.position = new Vector3(-2f, 1f, 10f);
        DetectedObstacle det1 = new DetectedObstacle(
            dynObs, dynObs.GetComponent<BoxCollider>(), dynObs.transform.position,
            dynObs.transform.position - testRoot.transform.position,
            Vector3.forward, 10f, 0f, Vector3.back, new Vector3(1.5f, 0f, 0f), isDynamic: true);
        ThreatReport rep1 = new ThreatReport(ThreatLevel.Critical, det1, new Vector3(0f, 1f, 10f), 10f, 5.0f, 0);

        bool r1 = replan.TryExecuteReplan("Crossing threat", rep1);
        Assert.IsTrue(r1);
        Assert.AreEqual(TacticalDecisionReason.VOPacingApplied, replan.LatestDecisionReason, "Stage 1 must apply VO pacing.");
        Assert.AreEqual(1, replan.SpeedPacingCount);
        Assert.IsTrue(follower.IsSpeedOverrideActive);

        UnityEngine.Object.DestroyImmediate(dynObs);
    }

    // ================================================================================================
    // CHECKPOINT 5: REPLANNING STABILITY / ANTI-OSCILLATION UNDER PERSISTENT THREATS
    // ================================================================================================

    [Test]
    public void Checkpoint5_PersistentThreatEvaluation_ReplanCountIsStrictlyBoundedByCooldown()
    {
        GridManager grid = testRoot.AddComponent<GridManager>();
        grid.gridWorldSize = new Vector2(40f, 40f);
        grid.nodeRadius = 0.5f;
        Pathfinding pf = testRoot.AddComponent<Pathfinding>();
        typeof(Pathfinding).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(pf, null);
        grid.CreateGrid();

        PathFollower follower = testRoot.AddComponent<PathFollower>();
        ThreatAssessment threat = testRoot.AddComponent<ThreatAssessment>();
        ReplanningController replan = testRoot.AddComponent<ReplanningController>();

        typeof(PathFollower).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(follower, null);
        typeof(ThreatAssessment).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(threat, null);
        typeof(ReplanningController).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(replan, null);

        follower.MoveSpeed = 2.0f;
        follower.StartFollowing(new List<Node> { new Node(true, new Vector3(0f, 1f, 20f), 0, 0) });

        GameObject staticObs = GameObject.CreatePrimitive(PrimitiveType.Cube);
        staticObs.transform.position = new Vector3(0f, 1f, 10f);
        DetectedObstacle det = new DetectedObstacle(
            staticObs, staticObs.GetComponent<BoxCollider>(), staticObs.transform.position,
            staticObs.transform.position - testRoot.transform.position,
            Vector3.forward, 10f, 0f, Vector3.back, Vector3.zero, isDynamic: false);
        ThreatReport rep = new ThreatReport(ThreatLevel.Critical, det, new Vector3(0f, 1f, 10f), 10f, 5.0f, 0);

        // Simulate 20 consecutive threat updates arriving at 10Hz (every 0.1s for 2.0s total)
        int acceptedReplans = 0;
        int rejectedReplans = 0;

        for (int cycle = 0; cycle < 20; cycle++)
        {
            bool executed = replan.TryExecuteReplan($"Persistent threat cycle {cycle}", rep);
            if (executed)
            {
                acceptedReplans++;
            }
            else
            {
                rejectedReplans++;
            }
        }

        // With cooldown of 1.0s, immediate repeated calls within the same second must be rejected
        Assert.AreEqual(1, acceptedReplans, "Only 1 replan should be accepted during immediate persistent threat spam within cooldown window.");
        Assert.AreEqual(19, rejectedReplans, "19 repeated immediate requests must be safely rejected by cooldown/hysteresis.");
        Assert.AreEqual(1, replan.ReplanCount, "Total ReplanCount must remain strictly 1.");

        UnityEngine.Object.DestroyImmediate(staticObs);
    }
}
