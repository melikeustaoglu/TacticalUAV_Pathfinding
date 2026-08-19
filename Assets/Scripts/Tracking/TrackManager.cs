using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Autonomous Multi-Target Track Manager and Lifecycle Coordinator.
/// Predicts, associates (GNN / Mahalanobis), and updates active 6-state target tracks.
/// Manages the full track lifecycle (Tentative 3/5 -> Confirmed -> Coasting -> Lost -> Deleted)
/// and exposes immutable TrackedTarget snapshots to downstream autonomy and threat assessment.
/// Zero steady-state heap allocations.
/// </summary>
public class TrackManager : MonoBehaviour
{
    public const int MaxTracks = 64;

    [Header("Lifecycle Thresholds")]
    [Tooltip("Number of required measurement hits in the last 5 scans to promote Tentative -> Confirmed.")]
    [SerializeField] private int promotionHitsRequired = 3;

    [Tooltip("Maximum allowable time in seconds to reacquire a Coasting track before marking Lost.")]
    [SerializeField] private float coastingTimeoutSeconds = 1.0f;

    [Tooltip("Maximum allowable time in seconds before permanently deleting a Lost track.")]
    [SerializeField] private float lostTimeoutSeconds = 2.0f;

    // Internal Track Container Record
    public class TrackRecord
    {
        public int TrackId;
        public TrackStatus Status;
        public TargetTracker Tracker;
        public float FirstDetectionTime;
        public float LastUpdateTime;
        public int ConsecutiveMisses;
        public int ScanHistory; // 5-bit sliding window mask
        public float Confidence;
        public Vector3 EstimatedExtents;

        public TrackRecord()
        {
            Tracker = new TargetTracker();
            Reset(-1);
        }

        public void Reset(int id)
        {
            TrackId = id;
            Status = TrackStatus.Deleted;
            Tracker.Reset();
            FirstDetectionTime = 0f;
            LastUpdateTime = 0f;
            ConsecutiveMisses = 0;
            ScanHistory = 0;
            Confidence = 0f;
            EstimatedExtents = Vector3.one;
        }

        public int CountHistoryHits()
        {
            int hits = 0;
            int mask = ScanHistory & 0x1F; // Last 5 bits
            while (mask > 0)
            {
                if ((mask & 1) == 1) hits++;
                mask >>= 1;
            }
            return hits;
        }
    }

    private readonly TrackRecord[] trackPool = new TrackRecord[MaxTracks];
    private readonly TargetTracker[] trackerPointers = new TargetTracker[MaxTracks];
    private readonly int[] trackMatches = new int[MaxTracks];
    private readonly int[] detectionMatches = new int[MaxTracks];
    private readonly TrackedTarget[] publishedTargets = new TrackedTarget[MaxTracks];

    private readonly DataAssociation dataAssociation = new DataAssociation();

    private int activeTrackCount = 0;
    private int nextTrackId = 1;
    private float lastProcessTime = -1f;

    public int ActiveTrackCount => activeTrackCount;
    public int NextTrackId => nextTrackId;

    public event Action<TrackedTarget[], int> OnTracksUpdated;

    private void Awake()
    {
        InitializeManager();
    }

    public void InitializeManager()
    {
        for (int i = 0; i < MaxTracks; i++)
        {
            if (trackPool[i] == null)
            {
                trackPool[i] = new TrackRecord();
                trackerPointers[i] = trackPool[i].Tracker;
            }
            else
            {
                trackPool[i].Reset(-1);
            }
        }
        activeTrackCount = 0;
        nextTrackId = 1;
        lastProcessTime = -1f;
    }

    public void Reset()
    {
        InitializeManager();
    }

    /// <summary>
    /// Processes a batch of TargetDetections from onboard sensors (LiDAR, Radar) at the specified timestamp.
    /// Runs Prediction -> GNN Association -> Kalman Updates -> Lifecycle Transitions -> Target Publishing.
    /// </summary>
    public void ProcessDetections(TargetDetection[] detections, int detectionCount, float currentTime)
    {
        if (currentTime < lastProcessTime - 0.0001f)
        {
            return; // Reject backward-in-time scans
        }

        lastProcessTime = currentTime;
        int m = Mathf.Min(detectionCount, MaxTracks);

        // 1. Predict all active tracks to current timestamp
        for (int i = 0; i < activeTrackCount; i++)
        {
            trackPool[i].Tracker.Predict(currentTime);
        }

        // 2. Sort active tracks by TrackId deterministically before association
        SortActiveTracksDeterministic();

        // 3. Perform Global Mahalanobis Data Association
        dataAssociation.Associate(
            trackerPointers,
            activeTrackCount,
            detections,
            m,
            trackMatches,
            detectionMatches);

        // 4. Update Matched Tracks & Coast Unmatched Tracks
        for (int i = 0; i < activeTrackCount; i++)
        {
            TrackRecord track = trackPool[i];
            int matchDetIdx = trackMatches[i];

            if (matchDetIdx >= 0 && matchDetIdx < m)
            {
                TargetDetection matchedDet = detections[matchDetIdx];

                // Check for stale measurement
                if (matchedDet.Timestamp >= track.LastUpdateTime - 0.0001f)
                {
                    track.Tracker.Update(matchedDet);
                    track.LastUpdateTime = matchedDet.Timestamp;
                    track.ConsecutiveMisses = 0;
                    track.ScanHistory = ((track.ScanHistory << 1) | 1) & 0x1F;
                    track.Confidence = matchedDet.Confidence;

                    // Lifecycle promotion rules
                    if (track.Status == TrackStatus.Tentative)
                    {
                        if (track.CountHistoryHits() >= promotionHitsRequired)
                        {
                            track.Status = TrackStatus.Confirmed;
                        }
                    }
                    else if (track.Status == TrackStatus.Coasting || track.Status == TrackStatus.Lost)
                    {
                        // Reacquisition within coasting window restores Confirmed
                        track.Status = TrackStatus.Confirmed;
                    }
                }
            }
            else
            {
                // Unmatched track -> Missed scan
                track.ConsecutiveMisses++;
                track.ScanHistory = (track.ScanHistory << 1) & 0x1F;

                float timeSinceUpdate = currentTime - track.LastUpdateTime;

                if (track.Status == TrackStatus.Tentative)
                {
                    // Tentative tracks deleted after 2 consecutive missed scans
                    if (track.ConsecutiveMisses >= 2)
                    {
                        track.Status = TrackStatus.Deleted;
                    }
                }
                else if (track.Status == TrackStatus.Confirmed)
                {
                    track.Status = TrackStatus.Coasting;
                }
                else if (track.Status == TrackStatus.Coasting)
                {
                    if (timeSinceUpdate > coastingTimeoutSeconds)
                    {
                        track.Status = TrackStatus.Lost;
                    }
                }
                else if (track.Status == TrackStatus.Lost)
                {
                    if (timeSinceUpdate > lostTimeoutSeconds)
                    {
                        track.Status = TrackStatus.Deleted;
                    }
                }
            }
        }

        // 5. Initialize new Tentative Tracks from Unmatched Detections
        for (int j = 0; j < m; j++)
        {
            if (detectionMatches[j] == -1 && activeTrackCount < MaxTracks)
            {
                TargetDetection unassignedDet = detections[j];
                if (unassignedDet.IsValid)
                {
                    int newTrackId = nextTrackId++;
                    TrackRecord newTrack = trackPool[activeTrackCount];
                    newTrack.Reset(newTrackId);
                    newTrack.Tracker.Initialize(unassignedDet);
                    newTrack.Status = TrackStatus.Tentative;
                    newTrack.FirstDetectionTime = currentTime;
                    newTrack.LastUpdateTime = unassignedDet.Timestamp;
                    newTrack.ConsecutiveMisses = 0;
                    newTrack.ScanHistory = 1;
                    newTrack.Confidence = unassignedDet.Confidence;

                    activeTrackCount++;
                }
            }
        }

        // 6. Prune Deleted Tracks & Compact Active Array in-place
        PruneDeletedTracks();

        // 7. Publish Confirmed & Active Tracks to output buffer
        PublishTrackedTargets(currentTime);
    }

    private void PruneDeletedTracks()
    {
        int writeIdx = 0;
        for (int readIdx = 0; readIdx < activeTrackCount; readIdx++)
        {
            if (trackPool[readIdx].Status != TrackStatus.Deleted)
            {
                if (writeIdx != readIdx)
                {
                    SwapTrackRecords(writeIdx, readIdx);
                }
                writeIdx++;
            }
        }
        activeTrackCount = writeIdx;
    }

    private void SwapTrackRecords(int a, int b)
    {
        TrackRecord tempRec = trackPool[a];
        trackPool[a] = trackPool[b];
        trackPool[b] = tempRec;

        trackerPointers[a] = trackPool[a].Tracker;
        trackerPointers[b] = trackPool[b].Tracker;
    }

    private void SortActiveTracksDeterministic()
    {
        for (int i = 0; i < activeTrackCount - 1; i++)
        {
            for (int j = i + 1; j < activeTrackCount; j++)
            {
                if (trackPool[i].TrackId > trackPool[j].TrackId)
                {
                    SwapTrackRecords(i, j);
                }
            }
        }
    }

    private void PublishTrackedTargets(float currentTime)
    {
        for (int i = 0; i < activeTrackCount; i++)
        {
            TrackRecord trk = trackPool[i];
            TargetTracker filter = trk.Tracker;

            float age = currentTime - trk.FirstDetectionTime;
            float timeSinceUpdate = currentTime - trk.LastUpdateTime;

            publishedTargets[i] = new TrackedTarget(
                trk.TrackId,
                filter.EstimatedPosition,
                filter.EstimatedVelocity,
                filter.PositionVariance,
                filter.VelocityVariance,
                trk.Status,
                age,
                timeSinceUpdate,
                trk.Confidence,
                trk.EstimatedExtents);
        }

        OnTracksUpdated?.Invoke(publishedTargets, activeTrackCount);
    }

    /// <summary>
    /// Copies confirmed targets into the caller-provided preallocated array.
    /// Returns the number of confirmed targets copied.
    /// </summary>
    public int GetConfirmedTargets(TrackedTarget[] outputBuffer, int offset, int maxCount)
    {
        if (outputBuffer == null || offset < 0 || maxCount <= 0) return 0;

        int written = 0;
        for (int i = 0; i < activeTrackCount && written < maxCount && (offset + written) < outputBuffer.Length; i++)
        {
            if (publishedTargets[i].Status == TrackStatus.Confirmed || publishedTargets[i].Status == TrackStatus.Coasting)
            {
                outputBuffer[offset + written] = publishedTargets[i];
                written++;
            }
        }
        return written;
    }

    /// <summary>
    /// Returns a specific track record by TrackId, or null if not found.
    /// </summary>
    public TrackRecord GetTrack(int trackId)
    {
        for (int i = 0; i < activeTrackCount; i++)
        {
            if (trackPool[i].TrackId == trackId) return trackPool[i];
        }
        return null;
    }
}
