using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;

/// <summary>
/// Phase A Acceptance Test Suite: Sensor Fusion & Perception.
/// Validates deterministic multi-sensor fusion (LiDAR + Radar), multi-modality association,
/// composite track confidence scoring, sensor dropout continuity, and recovery.
/// </summary>
[TestFixture]
public class SensorFusionTests
{
    private GameObject uavObj;
    private TrackManager trackManager;
    private readonly List<GameObject> cleanupObjects = new List<GameObject>();

    [SetUp]
    public void SetUp()
    {
        uavObj = new GameObject("TestUAV_SensorFusion");
        trackManager = uavObj.AddComponent<TrackManager>();
        trackManager.InitializeManager();
    }

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

    // =========================================================================
    // A1: Correct Multi-Sensor Track Fusion
    // =========================================================================
    [Test]
    public void SensorFusion_DualSensor_ConvergesToSingleTrack()
    {
        Vector3 targetPos = new Vector3(0f, 0f, 5.0f);
        Vector3 targetVel = new Vector3(0f, 0f, 1.5f);

        TargetDetection lidarDet = new TargetDetection(
            TargetSensorModality.LiDAR, 0.0f, targetPos, Vector3.one * 0.01f, 0.95f, 1);

        TargetDetection radarDet = new TargetDetection(
            TargetSensorModality.Radar, 0.0f, targetPos, Vector3.one * 0.09f, 0.90f, 2,
            targetVel, Vector3.one * 0.04f, true);

        TargetDetection[] batch = new TargetDetection[] { lidarDet, radarDet };
        trackManager.ProcessDetections(batch, 2, 0.0f);

        Assert.AreEqual(1, trackManager.ActiveTrackCount, "Dual sensor observations of the same target must fuse into exactly 1 logical track!");

        TrackManager.TrackRecord track = trackManager.GetTrack(1);
        Assert.IsNotNull(track, "Track with ID 1 must exist!");

        int expectedMask = (1 << (int)TargetSensorModality.LiDAR) | (1 << (int)TargetSensorModality.Radar);
        Assert.AreEqual(expectedMask, track.CorroboratingModalityMask, "Track must reflect dual-sensor corroboration from both LiDAR and Radar!");

        Assert.AreEqual(targetPos.z, track.Tracker.EstimatedPosition.z, 0.1f, "Position estimate must align with target!");
        Assert.AreEqual(targetVel.z, track.Tracker.EstimatedVelocity.z, 0.15f, "Radar velocity measurement must be incorporated into track!");

        Assert.GreaterOrEqual(track.Confidence, 0.70f, "Dual-sensor corroborated track must have high initial confidence!");
    }

    // =========================================================================
    // A2: Measurable Track Confidence
    // =========================================================================
    [Test]
    public void SensorFusion_DualSensor_HasMeasurablyHigherConfidenceThanSingleSensor()
    {
        Vector3 pos1 = new Vector3(-3f, 0f, 6.0f);
        TargetDetection lidar1 = new TargetDetection(TargetSensorModality.LiDAR, 0.0f, pos1, Vector3.one * 0.01f, 0.95f, 1);
        TargetDetection radar1 = new TargetDetection(TargetSensorModality.Radar, 0.0f, pos1, Vector3.one * 0.09f, 0.90f, 2, Vector3.zero, Vector3.one * 0.04f, true);

        Vector3 pos2 = new Vector3(3f, 0f, 6.0f);
        TargetDetection radar2 = new TargetDetection(TargetSensorModality.Radar, 0.0f, pos2, Vector3.one * 0.09f, 0.90f, 3, Vector3.zero, Vector3.one * 0.04f, true);

        for (int i = 0; i < 3; i++)
        {
            float t = i * 0.10f;
            TargetDetection l1 = new TargetDetection(TargetSensorModality.LiDAR, t, pos1, Vector3.one * 0.01f, 0.95f, 10 + i);
            TargetDetection r1 = new TargetDetection(TargetSensorModality.Radar, t, pos1, Vector3.one * 0.09f, 0.90f, 20 + i, Vector3.zero, Vector3.one * 0.04f, true);
            TargetDetection r2 = new TargetDetection(TargetSensorModality.Radar, t, pos2, Vector3.one * 0.09f, 0.90f, 30 + i, Vector3.zero, Vector3.one * 0.04f, true);

            trackManager.ProcessDetections(new TargetDetection[] { l1, r1, r2 }, 3, t);
        }

        Assert.AreEqual(2, trackManager.ActiveTrackCount, "Two distinct spatial targets must create 2 tracks!");

        TrackManager.TrackRecord dualTrack = trackManager.GetTrack(1);
        TrackManager.TrackRecord singleTrack = trackManager.GetTrack(2);

        Assert.IsNotNull(dualTrack);
        Assert.IsNotNull(singleTrack);

        Assert.AreEqual(TrackStatus.Confirmed, dualTrack.Status);
        Assert.AreEqual(TrackStatus.Confirmed, singleTrack.Status);

        float confidenceMargin = dualTrack.Confidence - singleTrack.Confidence;
        Assert.Greater(confidenceMargin, 0.10f, $"Dual-sensor track confidence ({dualTrack.Confidence:F3}) must exceed single-sensor track ({singleTrack.Confidence:F3}) by at least 0.10 margin!");

        Assert.Less(dualTrack.Tracker.HorizontalPositionStdDev, singleTrack.Tracker.HorizontalPositionStdDev,
            "Dual-sensor track position uncertainty must be lower than single-sensor track!");
    }

    // =========================================================================
    // A3: Sensor Dropout Continuity & Recovery
    // =========================================================================
    [Test]
    public void SensorFusion_SensorDropout_PreservesTrackAndDecaysConfidence()
    {
        Vector3 pos = new Vector3(0f, 0f, 5.0f);

        for (int i = 0; i < 5; i++)
        {
            float t = i * 0.10f;
            TargetDetection l = new TargetDetection(TargetSensorModality.LiDAR, t, pos, Vector3.one * 0.01f, 0.95f, i + 1);
            TargetDetection r = new TargetDetection(TargetSensorModality.Radar, t, pos, Vector3.one * 0.09f, 0.90f, i + 10, Vector3.zero, Vector3.one * 0.04f, true);
            trackManager.ProcessDetections(new TargetDetection[] { l, r }, 2, t);
        }

        TrackManager.TrackRecord track = trackManager.GetTrack(1);
        Assert.IsNotNull(track);
        Assert.AreEqual(TrackStatus.Confirmed, track.Status);

        float dualConfidence = track.Confidence;
        Assert.Greater(dualConfidence, 0.85f, "Established dual-sensor track must achieve high confidence!");

        trackManager.ProcessDetections(new TargetDetection[] {
            new TargetDetection(TargetSensorModality.Radar, 0.60f, pos, Vector3.one * 0.09f, 0.90f, 20, Vector3.zero, Vector3.one * 0.04f, true)
        }, 1, 0.60f);

        trackManager.ProcessDetections(new TargetDetection[] {
            new TargetDetection(TargetSensorModality.Radar, 1.20f, pos, Vector3.one * 0.09f, 0.90f, 21, Vector3.zero, Vector3.one * 0.04f, true)
        }, 1, 1.20f);

        Assert.AreEqual(1, trackManager.ActiveTrackCount, "Track must not duplicate or be lost during single-sensor dropout!");
        Assert.AreEqual(1, track.TrackId, "Track ID must be preserved during single-sensor dropout!");
        Assert.AreEqual(TrackStatus.Confirmed, track.Status, "Track must remain Confirmed while Radar continues updating!");

        float singleConfidence = track.Confidence;
        Assert.Less(singleConfidence, dualConfidence, "Track confidence must decay when LiDAR corroboration drops out!");
        Assert.Greater(dualConfidence - singleConfidence, 0.08f, "Confidence decay must exhibit a measurable margin!");
    }

    [Test]
    public void SensorFusion_SensorRecovery_RestoresCorroborationConfidence()
    {
        Vector3 pos = new Vector3(0f, 0f, 5.0f);

        for (int i = 0; i < 3; i++)
        {
            float t = i * 0.10f;
            TargetDetection l = new TargetDetection(TargetSensorModality.LiDAR, t, pos, Vector3.one * 0.01f, 0.95f, i + 1);
            TargetDetection r = new TargetDetection(TargetSensorModality.Radar, t, pos, Vector3.one * 0.09f, 0.90f, i + 10, Vector3.zero, Vector3.one * 0.04f, true);
            trackManager.ProcessDetections(new TargetDetection[] { l, r }, 2, t);
        }

        for (int i = 3; i < 8; i++)
        {
            float t = i * 0.10f;
            TargetDetection r = new TargetDetection(TargetSensorModality.Radar, t, pos, Vector3.one * 0.09f, 0.90f, i + 10, Vector3.zero, Vector3.one * 0.04f, true);
            trackManager.ProcessDetections(new TargetDetection[] { r }, 1, t);
        }

        TrackManager.TrackRecord track = trackManager.GetTrack(1);
        float droppedConfidence = track.Confidence;

        float tRecovery = 0.90f;
        TargetDetection lRec = new TargetDetection(TargetSensorModality.LiDAR, tRecovery, pos, Vector3.one * 0.01f, 0.95f, 50);
        TargetDetection rRec = new TargetDetection(TargetSensorModality.Radar, tRecovery, pos, Vector3.one * 0.09f, 0.90f, 51, Vector3.zero, Vector3.one * 0.04f, true);
        trackManager.ProcessDetections(new TargetDetection[] { lRec, rRec }, 2, tRecovery);

        float restoredConfidence = track.Confidence;

        Assert.Greater(restoredConfidence, droppedConfidence, "Confidence must recover upon LiDAR sensor return!");
        Assert.IsTrue((track.CorroboratingModalityMask & (1 << (int)TargetSensorModality.LiDAR)) != 0, "LiDAR corroboration bit must be restored!");
    }

    // =========================================================================
    // A4: Conflicting Detection Arbitration
    // =========================================================================
    [Test]
    public void SensorFusion_ConflictingMeasurements_PrioritizesReliableTrack()
    {
        Vector3 truePos = new Vector3(0f, 0f, 5.0f);
        Vector3 ghostPos = new Vector3(2.5f, 0f, 5.0f);

        for (int i = 0; i < 4; i++)
        {
            float t = i * 0.10f;
            TargetDetection lTrue = new TargetDetection(TargetSensorModality.LiDAR, t, truePos, Vector3.one * 0.01f, 0.95f, 100 + i);
            TargetDetection rTrue = new TargetDetection(TargetSensorModality.Radar, t, truePos, Vector3.one * 0.09f, 0.90f, 200 + i, Vector3.zero, Vector3.one * 0.04f, true);

            TargetDetection rGhost = new TargetDetection(TargetSensorModality.Radar, t, ghostPos, Vector3.one * 0.25f, 0.60f, 300 + i, Vector3.zero, Vector3.one * 0.10f, true);

            trackManager.ProcessDetections(new TargetDetection[] { lTrue, rTrue, rGhost }, 3, t);
        }

        Assert.AreEqual(2, trackManager.ActiveTrackCount);

        TrackManager.TrackRecord trueTrack = trackManager.GetTrack(1);
        TrackManager.TrackRecord ghostTrack = trackManager.GetTrack(2);

        Assert.IsNotNull(trueTrack, "True target track must exist!");
        Assert.IsNotNull(ghostTrack, "Ghost target track must exist!");

        Assert.AreEqual(TrackStatus.Confirmed, trueTrack.Status);

        float confidenceDelta = trueTrack.Confidence - ghostTrack.Confidence;
        Assert.Greater(confidenceDelta, 0.20f, $"True corroborated track confidence ({trueTrack.Confidence:F3}) must substantially exceed conflicting ghost confidence ({ghostTrack.Confidence:F3}) by at least 0.20 margin!");

        Assert.Greater(trueTrack.Confidence, 0.85f, "Corroborated real track must have high confidence!");
        Assert.Less(ghostTrack.Confidence, 0.75f, "Uncorroborated ghost track must have lower confidence!");
    }
}
