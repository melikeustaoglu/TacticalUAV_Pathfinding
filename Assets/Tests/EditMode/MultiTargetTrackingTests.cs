using System;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;

/// <summary>
/// Phase 12.4 Multi-Target Tracking & Data Association Unit Tests.
/// Validates GNN Mahalanobis assignment, 3-of-5 track promotion, crossing-target ID stability,
/// coasting/lost/deleted lifecycle states, multi-sensor modality switching, and zero-allocation updates.
/// </summary>
[TestFixture]
public class MultiTargetTrackingTests
{
    private GameObject trackerObj;
    private TrackManager trackManager;

    [SetUp]
    public void SetUp()
    {
        trackerObj = new GameObject("TestTrackManager");
        trackManager = trackerObj.AddComponent<TrackManager>();
        typeof(TrackManager).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(trackManager, null);
    }

    [TearDown]
    public void TearDown()
    {
        if (trackerObj != null) UnityEngine.Object.DestroyImmediate(trackerObj);
    }

    [Test]
    public void MTT_SingleTarget_InitializesTentativeTrack()
    {
        TargetDetection[] dets = new TargetDetection[]
        {
            new TargetDetection(TargetSensorModality.LiDAR, 0.0f, new Vector3(0f, 0f, 5f), Vector3.one * 0.04f, 0.95f, 1)
        };

        trackManager.ProcessDetections(dets, 1, 0.0f);

        Assert.AreEqual(1, trackManager.ActiveTrackCount);
        TrackManager.TrackRecord track = trackManager.GetTrack(1);
        Assert.IsNotNull(track);
        Assert.AreEqual(TrackStatus.Tentative, track.Status);
    }

    [Test]
    public void MTT_SingleTarget_PromotesToConfirmedOnConsecutiveDetections()
    {
        Vector3 pos = new Vector3(0f, 0f, 5f);

        // 3 consecutive scans
        for (int i = 0; i < 3; i++)
        {
            float t = i * 0.05f;
            TargetDetection[] dets = new TargetDetection[]
            {
                new TargetDetection(TargetSensorModality.LiDAR, t, pos, Vector3.one * 0.04f, 0.95f, i + 1)
            };
            trackManager.ProcessDetections(dets, 1, t);
        }

        TrackManager.TrackRecord track = trackManager.GetTrack(1);
        Assert.IsNotNull(track);
        Assert.AreEqual(TrackStatus.Confirmed, track.Status, "Track must be promoted to Confirmed after 3 valid hits!");
    }

    [Test]
    public void MTT_KalmanFilter_AccuratelyEstimatesTargetVelocity()
    {
        Vector3 trueVel = new Vector3(0f, 0f, 3.0f);
        Vector3 startPos = new Vector3(0f, 0f, 2f);

        // Feed 15 position-only scans
        for (int i = 0; i < 15; i++)
        {
            float t = i * 0.1f;
            Vector3 pos = startPos + trueVel * t;
            TargetDetection[] dets = new TargetDetection[]
            {
                new TargetDetection(TargetSensorModality.LiDAR, t, pos, Vector3.one * 0.04f, 0.95f, i + 1)
            };
            trackManager.ProcessDetections(dets, 1, t);
        }

        TrackManager.TrackRecord track = trackManager.GetTrack(1);
        Assert.AreEqual(3.0f, track.Tracker.EstimatedVelocity.z, 0.35f);
    }

    [Test]
    public void MTT_DataAssociation_CorrectlyResolvesMultipleDetections()
    {
        Vector3 posA = new Vector3(0f, 0f, 5f);
        Vector3 posB = new Vector3(8f, 0f, 10f);

        // Scan 1: Initialize 2 tracks
        TargetDetection[] dets1 = new TargetDetection[]
        {
            new TargetDetection(TargetSensorModality.LiDAR, 0.0f, posA, Vector3.one * 0.04f, 0.95f, 1),
            new TargetDetection(TargetSensorModality.LiDAR, 0.0f, posB, Vector3.one * 0.04f, 0.95f, 2)
        };
        trackManager.ProcessDetections(dets1, 2, 0.0f);
        Assert.AreEqual(2, trackManager.ActiveTrackCount);

        // Scan 2: Displaced measurements
        TargetDetection[] dets2 = new TargetDetection[]
        {
            new TargetDetection(TargetSensorModality.LiDAR, 0.1f, posA + Vector3.forward * 0.1f, Vector3.one * 0.04f, 0.95f, 3),
            new TargetDetection(TargetSensorModality.LiDAR, 0.1f, posB + Vector3.forward * 0.1f, Vector3.one * 0.04f, 0.95f, 4)
        };
        trackManager.ProcessDetections(dets2, 2, 0.1f);

        TrackManager.TrackRecord trkA = trackManager.GetTrack(1);
        TrackManager.TrackRecord trkB = trackManager.GetTrack(2);

        Assert.AreEqual(posA.x, trkA.Tracker.EstimatedPosition.x, 0.2f);
        Assert.AreEqual(posB.x, trkB.Tracker.EstimatedPosition.x, 0.2f);
    }

    [Test]
    public void MTT_CrossingTargets_MaintainsStableTrackIDs()
    {
        // Target 1: moving left-to-right (-2 to +2 along X at z=5)
        // Target 2: moving right-to-left (+2 to -2 along X at z=5)
        float vx1 = 2.0f;
        float vx2 = -2.0f;

        // 1. Initial scans to establish velocity momentum and Confirmed status
        for (int i = 0; i < 4; i++)
        {
            float t = i * 0.1f;
            Vector3 p1 = new Vector3(-2f + vx1 * t, 0f, 5f);
            Vector3 p2 = new Vector3(2f + vx2 * t, 0f, 5f);

            TargetDetection[] dets = new TargetDetection[]
            {
                new TargetDetection(TargetSensorModality.LiDAR, t, p1, Vector3.one * 0.04f, 0.95f, i * 2 + 1),
                new TargetDetection(TargetSensorModality.LiDAR, t, p2, Vector3.one * 0.04f, 0.95f, i * 2 + 2)
            };
            trackManager.ProcessDetections(dets, 2, t);
        }

        // 2. Crossing intersection around t = 1.0s (both at x ~ 0)
        for (int i = 4; i < 15; i++)
        {
            float t = i * 0.1f;
            Vector3 p1 = new Vector3(-2f + vx1 * t, 0f, 5f);
            Vector3 p2 = new Vector3(2f + vx2 * t, 0f, 5f);

            TargetDetection[] dets = new TargetDetection[]
            {
                new TargetDetection(TargetSensorModality.LiDAR, t, p1, Vector3.one * 0.04f, 0.95f, i * 2 + 1),
                new TargetDetection(TargetSensorModality.LiDAR, t, p2, Vector3.one * 0.04f, 0.95f, i * 2 + 2)
            };
            trackManager.ProcessDetections(dets, 2, t);
        }

        TrackManager.TrackRecord trk1 = trackManager.GetTrack(1);
        TrackManager.TrackRecord trk2 = trackManager.GetTrack(2);

        Assert.IsNotNull(trk1);
        Assert.IsNotNull(trk2);

        // Track 1 must still be moving right (positive vx) and Track 2 moving left (negative vx)
        Assert.Greater(trk1.Tracker.EstimatedVelocity.x, 1.0f, "Track 1 must maintain positive vx through crossing!");
        Assert.Less(trk2.Tracker.EstimatedVelocity.x, -1.0f, "Track 2 must maintain negative vx through crossing!");
    }

    [Test]
    public void MTT_MissedDetection_EntersCoastingAndExpandsCovariance()
    {
        Vector3 pos = new Vector3(0f, 0f, 5f);

        // Confirm track
        for (int i = 0; i < 3; i++)
        {
            float t = i * 0.05f;
            TargetDetection[] dets = new TargetDetection[]
            {
                new TargetDetection(TargetSensorModality.LiDAR, t, pos, Vector3.one * 0.04f, 0.95f, i + 1)
            };
            trackManager.ProcessDetections(dets, 1, t);
        }

        TrackManager.TrackRecord track = trackManager.GetTrack(1);
        Assert.AreEqual(TrackStatus.Confirmed, track.Status);
        float confirmedVar = track.Tracker.PositionVariance.x;

        // Missed scan
        trackManager.ProcessDetections(new TargetDetection[0], 0, 0.20f);

        Assert.AreEqual(TrackStatus.Coasting, track.Status, "Missed scan must transition Confirmed -> Coasting!");
        Assert.Greater(track.Tracker.PositionVariance.x, confirmedVar, "Covariance must expand during coasting!");
    }

    [Test]
    public void MTT_Reacquisition_RestoresConfirmedStatusAndContractsCovariance()
    {
        Vector3 pos = new Vector3(0f, 0f, 5f);

        // Confirm track
        for (int i = 0; i < 3; i++)
        {
            trackManager.ProcessDetections(new TargetDetection[] { new TargetDetection(TargetSensorModality.LiDAR, i * 0.05f, pos, Vector3.one * 0.04f, 0.95f, i + 1) }, 1, i * 0.05f);
        }

        // Missed scan (Coasting)
        trackManager.ProcessDetections(new TargetDetection[0], 0, 0.20f);
        TrackManager.TrackRecord track = trackManager.GetTrack(1);
        Assert.AreEqual(TrackStatus.Coasting, track.Status);
        float coastVar = track.Tracker.PositionVariance.x;

        // Reacquisition
        trackManager.ProcessDetections(new TargetDetection[] { new TargetDetection(TargetSensorModality.LiDAR, 0.30f, pos, Vector3.one * 0.04f, 0.95f, 10) }, 1, 0.30f);

        Assert.AreEqual(TrackStatus.Confirmed, track.Status, "Reacquired detection must restore Confirmed status!");
        Assert.Less(track.Tracker.PositionVariance.x, coastVar, "Covariance must contract upon reacquisition!");
    }

    [Test]
    public void MTT_ProlongedOutage_PrunesAndDeletesTrack()
    {
        Vector3 pos = new Vector3(0f, 0f, 5f);

        // Confirm track
        for (int i = 0; i < 3; i++)
        {
            trackManager.ProcessDetections(new TargetDetection[] { new TargetDetection(TargetSensorModality.LiDAR, i * 0.05f, pos, Vector3.one * 0.04f, 0.95f, i + 1) }, 1, i * 0.05f);
        }

        // Advance time past coasting timeout (1.0s) -> Lost
        trackManager.ProcessDetections(new TargetDetection[0], 0, 1.20f);
        TrackManager.TrackRecord track = trackManager.GetTrack(1);
        Assert.AreEqual(TrackStatus.Lost, track.Status);

        // Advance time past lost timeout (2.0s) -> Deleted and pruned
        trackManager.ProcessDetections(new TargetDetection[0], 0, 2.50f);
        Assert.AreEqual(0, trackManager.ActiveTrackCount, "Track must be pruned from active list after prolonged outage!");
    }

    [Test]
    public void MTT_MahalanobisGating_RejectsSpuriousOutlierDetections()
    {
        Vector3 pos = new Vector3(0f, 0f, 5f);
        // 3 consecutive scans confirm Track 1
        trackManager.ProcessDetections(new TargetDetection[] { new TargetDetection(TargetSensorModality.LiDAR, 0.00f, pos, Vector3.one * 0.04f, 0.95f, 1) }, 1, 0.00f);
        trackManager.ProcessDetections(new TargetDetection[] { new TargetDetection(TargetSensorModality.LiDAR, 0.05f, pos, Vector3.one * 0.04f, 0.95f, 1) }, 1, 0.05f);
        trackManager.ProcessDetections(new TargetDetection[] { new TargetDetection(TargetSensorModality.LiDAR, 0.10f, pos, Vector3.one * 0.04f, 0.95f, 1) }, 1, 0.10f);

        // Distant spurious outlier at (50, 50, 50)
        TargetDetection outlier = new TargetDetection(TargetSensorModality.LiDAR, 0.15f, new Vector3(50f, 50f, 50f), Vector3.one * 0.04f, 0.95f, 2);
        trackManager.ProcessDetections(new TargetDetection[] { outlier }, 1, 0.15f);

        // Track 1 should be unmatched (Coasting) and Outlier spawns new tentative Track 2
        TrackManager.TrackRecord track1 = trackManager.GetTrack(1);
        TrackManager.TrackRecord track2 = trackManager.GetTrack(2);

        Assert.AreEqual(TrackStatus.Coasting, track1.Status, "Original track must not associate with distant outlier!");
        Assert.IsNotNull(track2);
    }

    [Test]
    public void MTT_RadarDoppler_AcceleratesVelocityConvergence()
    {
        Vector3 trueVel = new Vector3(0f, 0f, 2.5f);
        Vector3 pos = new Vector3(0f, 0f, 5f);

        TargetDetection radarDet = new TargetDetection(
            TargetSensorModality.Radar, 0.0f, pos, Vector3.one * 0.09f, 0.90f, 1,
            trueVel, Vector3.one * 0.04f, true);

        trackManager.ProcessDetections(new TargetDetection[] { radarDet }, 1, 0.0f);

        TrackManager.TrackRecord track = trackManager.GetTrack(1);
        Assert.AreEqual(trueVel.z, track.Tracker.EstimatedVelocity.z, 0.05f, "Radar measurement must initialize velocity instantaneously!");
    }

    [Test]
    public void MTT_LidarPositionOnlyUpdatePreservesVelocityEstimate()
    {
        Vector3 trueVel = new Vector3(0f, 0f, 2.0f);
        Vector3 pos = new Vector3(0f, 0f, 5f);

        TargetDetection radarDet = new TargetDetection(
            TargetSensorModality.Radar, 0.0f, pos, Vector3.one * 0.09f, 0.90f, 1,
            trueVel, Vector3.one * 0.04f, true);

        trackManager.ProcessDetections(new TargetDetection[] { radarDet }, 1, 0.0f);

        // Subsequent LiDAR update (position only)
        Vector3 pos2 = pos + trueVel * 0.1f;
        TargetDetection lidarDet = new TargetDetection(TargetSensorModality.LiDAR, 0.1f, pos2, Vector3.one * 0.04f, 0.95f, 2);
        trackManager.ProcessDetections(new TargetDetection[] { lidarDet }, 1, 0.1f);

        TrackManager.TrackRecord track = trackManager.GetTrack(1);
        Assert.AreEqual(2.0f, track.Tracker.EstimatedVelocity.z, 0.2f);
    }

    [Test]
    public void MTT_TrackLifecycle_RespectsTentativeThreeOfFiveRule()
    {
        // 1 detection, followed by 2 consecutive misses -> Deleted
        trackManager.ProcessDetections(new TargetDetection[] { new TargetDetection(TargetSensorModality.LiDAR, 0.0f, new Vector3(0f, 0f, 5f), Vector3.one * 0.04f, 0.95f, 1) }, 1, 0.0f);
        Assert.AreEqual(1, trackManager.ActiveTrackCount);

        trackManager.ProcessDetections(new TargetDetection[0], 0, 0.05f); // Miss 1
        Assert.AreEqual(1, trackManager.ActiveTrackCount);

        trackManager.ProcessDetections(new TargetDetection[0], 0, 0.10f); // Miss 2 -> Deleted
        Assert.AreEqual(0, trackManager.ActiveTrackCount, "Tentative track must be deleted after 2 consecutive misses!");
    }

    [Test]
    public void MTT_StaleDetection_IsRejected()
    {
        trackManager.ProcessDetections(new TargetDetection[] { new TargetDetection(TargetSensorModality.LiDAR, 1.0f, new Vector3(0f, 0f, 5f), Vector3.one * 0.04f, 0.95f, 1) }, 1, 1.0f);

        // Stale detection with timestamp earlier than current scan time
        TargetDetection staleDet = new TargetDetection(TargetSensorModality.LiDAR, 0.5f, new Vector3(0f, 0f, 5f), Vector3.one * 0.04f, 0.95f, 2);
        trackManager.ProcessDetections(new TargetDetection[] { staleDet }, 1, 0.5f);

        TrackManager.TrackRecord track = trackManager.GetTrack(1);
        Assert.AreEqual(1.0f, track.LastUpdateTime, "Track must reject stale detections and preserve current timestamp!");
    }

    [Test]
    public void MTT_TrackIds_NeverReuseAfterDeletion()
    {
        // Target 1
        trackManager.ProcessDetections(new TargetDetection[] { new TargetDetection(TargetSensorModality.LiDAR, 0.0f, new Vector3(0f, 0f, 5f), Vector3.one * 0.04f, 0.95f, 1) }, 1, 0.0f);
        Assert.AreEqual(1, trackManager.GetTrack(1).TrackId);

        // Force delete
        trackManager.ProcessDetections(new TargetDetection[0], 0, 0.05f);
        trackManager.ProcessDetections(new TargetDetection[0], 0, 0.10f);
        Assert.AreEqual(0, trackManager.ActiveTrackCount);

        // New target spawns
        trackManager.ProcessDetections(new TargetDetection[] { new TargetDetection(TargetSensorModality.LiDAR, 0.15f, new Vector3(2f, 0f, 8f), Vector3.one * 0.04f, 0.95f, 2) }, 1, 0.15f);

        Assert.AreEqual(1, trackManager.ActiveTrackCount);
        Assert.AreEqual(2, trackManager.GetTrack(2).TrackId, "Track ID must never be reused after deletion!");
    }

    [Test]
    public void MTT_ZeroAllocation_ExecutesUpdateWithoutPerFrameGC()
    {
        TargetDetection[] dets = new TargetDetection[4];
        for (int i = 0; i < 4; i++)
        {
            dets[i] = new TargetDetection(TargetSensorModality.LiDAR, 0.0f, new Vector3(i * 3f, 0f, 5f), Vector3.one * 0.04f, 0.95f, i + 1);
        }

        // Warm up JIT
        for (int i = 0; i < 5; i++)
        {
            trackManager.ProcessDetections(dets, 4, i * 0.05f);
        }

        long memBefore = GC.GetTotalMemory(true);

        for (int i = 6; i <= 50; i++)
        {
            for (int k = 0; k < 4; k++)
            {
                dets[k] = new TargetDetection(TargetSensorModality.LiDAR, i * 0.05f, new Vector3(k * 3f, 0f, 5f + i * 0.05f), Vector3.one * 0.04f, 0.95f, i * 4 + k);
            }
            trackManager.ProcessDetections(dets, 4, i * 0.05f);
        }

        long memAfter = GC.GetTotalMemory(false);
        Assert.AreEqual(memBefore, memAfter, "TrackManager must execute steady-state updates with zero heap allocations!");
    }

    [Test]
    public void MTT_TrackManager_HandlesMaximumSimultaneousTracks()
    {
        TargetDetection[] dets = new TargetDetection[64];
        for (int i = 0; i < 64; i++)
        {
            dets[i] = new TargetDetection(TargetSensorModality.LiDAR, 0.0f, new Vector3(i * 2f, 0f, 10f), Vector3.one * 0.04f, 0.95f, i + 1);
        }

        trackManager.ProcessDetections(dets, 64, 0.0f);
        Assert.AreEqual(64, trackManager.ActiveTrackCount, "TrackManager must support 64 simultaneous active tracks!");

        TrackedTarget[] targetBuffer = new TrackedTarget[64];
        int count = trackManager.GetConfirmedTargets(targetBuffer, 0, 64);
        Assert.AreEqual(0, count); // Still tentative

        // Promote all 64
        trackManager.ProcessDetections(dets, 64, 0.05f);
        trackManager.ProcessDetections(dets, 64, 0.10f);

        count = trackManager.GetConfirmedTargets(targetBuffer, 0, 64);
        Assert.AreEqual(64, count, "All 64 tracks must be promoted and accessible via preallocated buffer!");
    }
}
