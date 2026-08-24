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
    public const int MaxSensors = 16;
    public const int MaxBufferedDetections = 128;

    [Header("Lifecycle Thresholds")]
    [Tooltip("Number of required measurement hits in the last 5 scans to promote Tentative -> Confirmed.")]
    [SerializeField] private int promotionHitsRequired = 3;

    [Tooltip("Maximum allowable time in seconds to reacquire a Coasting track before marking Lost.")]
    [SerializeField] private float coastingTimeoutSeconds = 1.0f;

    [Tooltip("Maximum allowable time in seconds before permanently deleting a Lost track.")]
    [SerializeField] private float lostTimeoutSeconds = 2.0f;

    [Header("Runtime Sensor Polling")]
    [Tooltip("Automatically discover and poll attached ITargetSensor components during runtime Update.")]
    [SerializeField] private bool autoPollSensors = true;

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

        // Phase A Corroboration & Metrics
        public int CorroboratingModalityMask;
        public float LastLidarDetectionTime;
        public float LastRadarDetectionTime;
        public float LatestDetectionConfidence;
        public bool WasUpdatedInCurrentCycle;

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
            CorroboratingModalityMask = 0;
            LastLidarDetectionTime = -10f;
            LastRadarDetectionTime = -10f;
            LatestDetectionConfidence = 0.90f;
            WasUpdatedInCurrentCycle = false;
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
    private readonly TargetDetection[] modalityDetections = new TargetDetection[MaxBufferedDetections];

    private readonly DataAssociation dataAssociation = new DataAssociation();

    // Sensor Polling Buffers & State
    private readonly ITargetSensor[] cachedSensors = new ITargetSensor[MaxSensors];
    private int cachedSensorCount = 0;

    private readonly TargetDetection[] sensorDetectionBuffer = new TargetDetection[MaxBufferedDetections];
    private int pendingDetectionCount = 0;
    private bool hasPendingScan = false;
    private Action<TargetDetection[], int> onSensorDetectionsUpdatedHandler;

    private int activeTrackCount = 0;
    private int nextTrackId = 1;
    private float lastProcessTime = -1f;

    public int ActiveTrackCount => activeTrackCount;
    public int NextTrackId => nextTrackId;
    public int SensorCount => cachedSensorCount;
    public bool AutoPollSensors { get => autoPollSensors; set => autoPollSensors = value; }

    public event Action<TrackedTarget[], int> OnTracksUpdated;

    private void Awake()
    {
        InitializeManager();
        InitializeSensors();
    }

    private void OnEnable()
    {
        SubscribeAllSensors();
    }

    private void OnDisable()
    {
        UnsubscribeAllSensors();
    }

    private void Start()
    {
        if (cachedSensorCount == 0)
        {
            DiscoverSensors();
        }
    }

    private void Update()
    {
        if (autoPollSensors)
        {
            PollSensors(Time.time);
        }
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
        pendingDetectionCount = 0;
        hasPendingScan = false;
    }

    public void InitializeSensors()
    {
        if (onSensorDetectionsUpdatedHandler == null)
        {
            onSensorDetectionsUpdatedHandler = HandleSensorDetectionsUpdated;
        }
        if (cachedSensorCount == 0)
        {
            DiscoverSensors();
        }
    }

    public void Reset()
    {
        InitializeManager();
    }

    /// <summary>
    /// Discovers and caches all ITargetSensor components attached to this GameObject and its children.
    /// </summary>
    public void DiscoverSensors()
    {
        UnsubscribeAllSensors();
        cachedSensorCount = 0;

        // 1. Search on this GameObject
        ITargetSensor[] onSelf = GetComponents<ITargetSensor>();
        if (onSelf != null)
        {
            for (int i = 0; i < onSelf.Length && cachedSensorCount < MaxSensors; i++)
            {
                if (onSelf[i] != null && !ContainsSensor(onSelf[i]))
                {
                    cachedSensors[cachedSensorCount++] = onSelf[i];
                }
            }
        }

        // 2. Search on child GameObjects
        ITargetSensor[] onChildren = GetComponentsInChildren<ITargetSensor>();
        if (onChildren != null)
        {
            for (int i = 0; i < onChildren.Length && cachedSensorCount < MaxSensors; i++)
            {
                if (onChildren[i] != null && !ContainsSensor(onChildren[i]))
                {
                    cachedSensors[cachedSensorCount++] = onChildren[i];
                }
            }
        }

        SortSensorsDeterministic();
        SubscribeAllSensors();
    }

    /// <summary>
    /// Manually registers an ITargetSensor with the manager.
    /// </summary>
    public bool RegisterSensor(ITargetSensor sensor)
    {
        if (sensor == null || cachedSensorCount >= MaxSensors || ContainsSensor(sensor))
        {
            return false;
        }

        if (onSensorDetectionsUpdatedHandler == null)
        {
            onSensorDetectionsUpdatedHandler = HandleSensorDetectionsUpdated;
        }

        cachedSensors[cachedSensorCount++] = sensor;
        SortSensorsDeterministic();
        sensor.OnDetectionsUpdated += onSensorDetectionsUpdatedHandler;
        return true;
    }

    /// <summary>
    /// Unregisters a previously registered ITargetSensor.
    /// </summary>
    public bool UnregisterSensor(ITargetSensor sensor)
    {
        if (sensor == null || cachedSensorCount == 0) return false;

        int foundIdx = -1;
        for (int i = 0; i < cachedSensorCount; i++)
        {
            if (cachedSensors[i] == sensor)
            {
                foundIdx = i;
                break;
            }
        }

        if (foundIdx < 0) return false;

        sensor.OnDetectionsUpdated -= onSensorDetectionsUpdatedHandler;

        for (int i = foundIdx; i < cachedSensorCount - 1; i++)
        {
            cachedSensors[i] = cachedSensors[i + 1];
        }
        cachedSensors[cachedSensorCount - 1] = null;
        cachedSensorCount--;
        return true;
    }

    /// <summary>
    /// Clears all cached sensors and unsubscribes from events.
    /// </summary>
    public void ClearSensors()
    {
        UnsubscribeAllSensors();
        for (int i = 0; i < cachedSensorCount; i++)
        {
            cachedSensors[i] = null;
        }
        cachedSensorCount = 0;
        pendingDetectionCount = 0;
        hasPendingScan = false;
    }

    public ITargetSensor GetSensor(int index)
    {
        if (index >= 0 && index < cachedSensorCount)
        {
            return cachedSensors[index];
        }
        return null;
    }

    private bool ContainsSensor(ITargetSensor sensor)
    {
        for (int i = 0; i < cachedSensorCount; i++)
        {
            if (cachedSensors[i] == sensor) return true;
        }
        return false;
    }

    private void SubscribeAllSensors()
    {
        if (onSensorDetectionsUpdatedHandler == null)
        {
            onSensorDetectionsUpdatedHandler = HandleSensorDetectionsUpdated;
        }

        for (int i = 0; i < cachedSensorCount; i++)
        {
            ITargetSensor sensor = cachedSensors[i];
            if (sensor != null)
            {
                sensor.OnDetectionsUpdated -= onSensorDetectionsUpdatedHandler;
                sensor.OnDetectionsUpdated += onSensorDetectionsUpdatedHandler;
            }
        }
    }

    private void UnsubscribeAllSensors()
    {
        if (onSensorDetectionsUpdatedHandler == null) return;

        for (int i = 0; i < cachedSensorCount; i++)
        {
            ITargetSensor sensor = cachedSensors[i];
            if (sensor != null)
            {
                sensor.OnDetectionsUpdated -= onSensorDetectionsUpdatedHandler;
            }
        }
    }

    private void SortSensorsDeterministic()
    {
        for (int i = 0; i < cachedSensorCount - 1; i++)
        {
            for (int j = i + 1; j < cachedSensorCount; j++)
            {
                ITargetSensor a = cachedSensors[i];
                ITargetSensor b = cachedSensors[j];
                if (a == null || b == null) continue;

                int comp = a.Modality.CompareTo(b.Modality);
                if (comp == 0)
                {
                    int idA = (a is Component compA) ? compA.GetInstanceID() : 0;
                    int idB = (b is Component compB) ? compB.GetInstanceID() : 0;
                    comp = idA.CompareTo(idB);
                }

                if (comp > 0)
                {
                    cachedSensors[i] = b;
                    cachedSensors[j] = a;
                }
            }
        }
    }

    private void HandleSensorDetectionsUpdated(TargetDetection[] detections, int count)
    {
        hasPendingScan = true;
        for (int i = 0; i < count && pendingDetectionCount < MaxBufferedDetections; i++)
        {
            sensorDetectionBuffer[pendingDetectionCount++] = detections[i];
        }
    }

    /// <summary>
    /// Polls all configured sensors at the specified timestamp, aggregates detections into
    /// a preallocated combined buffer, and invokes ProcessDetections if any sensor performed a scan.
    /// Zero steady-state heap allocations.
    /// </summary>
    public void PollSensors(float currentTime)
    {
        if (cachedSensorCount == 0) return;

        // 1. Evaluate any sensor that has not yet evaluated at currentTime
        for (int i = 0; i < cachedSensorCount; i++)
        {
            ITargetSensor sensor = cachedSensors[i];
            if (sensor == null || !sensor.IsActive || sensor.Health == SensorHealth.Failed || sensor.Health == SensorHealth.Timeout)
            {
                continue;
            }

            sensor.Evaluate(currentTime);
        }

        // 2. If any sensor performed a scan (captured via event or pending flag), process the batch
        if (hasPendingScan)
        {
            ProcessDetections(sensorDetectionBuffer, pendingDetectionCount, currentTime);
            pendingDetectionCount = 0;
            hasPendingScan = false;
        }
    }

    public void PollSensors()
    {
        PollSensors(Time.time);
    }

    /// <summary>
    /// Processes a batch of TargetDetections from onboard sensors (LiDAR, Radar) at the specified timestamp.
    public const float CorroborationWindowSeconds = 0.50f;

    /// <summary>
    /// Computes the composite track confidence score C_track in [0.0, 1.0] from physical observables:
    /// 1. Sensor Modality Corroboration (dual-sensor LiDAR+Radar = 1.0, single-sensor = 0.65, coasting = 0.35)
    /// 2. Sensor Detection Quality (instantaneous detection confidence)
    /// 3. Measurement Hit Consistency (hits / opportunities in 5-scan sliding window)
    /// 4. Spatial Uncertainty / Covariance Convergence (1 / (1 + sigma_pos))
    /// Weighted sum: w_corrob=0.40, w_quality=0.25, w_hist=0.20, w_cov=0.15.
    /// </summary>
    public static float ComputeCompositeConfidence(TrackRecord track, float currentTime)
    {
        if (track == null || track.Status == TrackStatus.Deleted)
        {
            return 0f;
        }

        // 1. Modality Corroboration Factor (fCorrob)
        bool lidarActive = (currentTime - track.LastLidarDetectionTime) <= CorroborationWindowSeconds;
        bool radarActive = (currentTime - track.LastRadarDetectionTime) <= CorroborationWindowSeconds;

        float fCorrob;
        if (lidarActive && radarActive)
        {
            fCorrob = 1.0f; // Dual-sensor verified
        }
        else if (lidarActive || radarActive)
        {
            fCorrob = 0.65f; // Single-sensor active
        }
        else
        {
            fCorrob = 0.35f; // Coasting / no recent sensor hits
        }

        // 2. Sensor Detection Quality Factor (fQuality)
        float fQuality = Mathf.Clamp01(track.LatestDetectionConfidence);

        // 3. Hit Consistency Factor (fHist, 5-scan sliding window)
        int hits = track.CountHistoryHits();
        float fHist = Mathf.Clamp01(hits / 5.0f);
        if (track.Status == TrackStatus.Tentative && hits < 3)
        {
            fHist = Mathf.Max(0.20f, hits / 5.0f);
        }

        // 4. Uncertainty / Covariance Factor (fCov)
        float posStdDev = track.Tracker != null ? track.Tracker.HorizontalPositionStdDev : 1.0f;
        float fCov = 1.0f / (1.0f + Mathf.Max(0f, posStdDev));

        // 5. Composite Weighted Sum: w_corrob=0.40, w_quality=0.25, w_hist=0.20, w_cov=0.15
        float composite = 0.40f * fCorrob + 0.25f * fQuality + 0.20f * fHist + 0.15f * fCov;

        if (track.Status == TrackStatus.Coasting)
        {
            composite *= 0.85f;
        }
        else if (track.Status == TrackStatus.Lost)
        {
            composite *= 0.50f;
        }

        return Mathf.Clamp01(composite);
    }

    /// <summary>
    /// Processes a batch of TargetDetections from onboard sensors (LiDAR, Radar) at the specified timestamp.
    /// Runs Multi-Modality Sequential Association -> Kalman Updates -> Lifecycle Transitions -> Target Publishing.
    /// Eliminates ghost tracks when multiple sensors observe the same target simultaneously.
    /// </summary>
    public void ProcessDetections(TargetDetection[] detections, int detectionCount, float currentTime)
    {
        if (currentTime < lastProcessTime - 0.0001f)
        {
            return; // Reject backward-in-time scans
        }

        lastProcessTime = currentTime;
        int m = Mathf.Min(detectionCount, MaxBufferedDetections);

        // Reset per-cycle update flags on all active tracks
        for (int i = 0; i < activeTrackCount; i++)
        {
            trackPool[i].WasUpdatedInCurrentCycle = false;
        }

        // 1. Predict all active tracks to current timestamp
        for (int i = 0; i < activeTrackCount; i++)
        {
            trackPool[i].Tracker.Predict(currentTime);
        }

        // 2. Process detections partitioned by sensor modality sequentially
        for (int mod = 0; mod <= (int)TargetSensorModality.ElectroOptical; mod++)
        {
            TargetSensorModality currentModality = (TargetSensorModality)mod;

            // Collect detections belonging to currentModality
            int modalityCount = 0;
            for (int d = 0; d < m; d++)
            {
                if (detections[d].IsValid && detections[d].Modality == currentModality)
                {
                    modalityDetections[modalityCount++] = detections[d];
                }
            }

            if (modalityCount == 0)
            {
                continue; // No detections for this modality
            }

            // Predict active tracks to current timestamp (ensures newly spawned tracks from earlier passes are at currentTime)
            for (int i = 0; i < activeTrackCount; i++)
            {
                trackPool[i].Tracker.Predict(currentTime);
            }

            // Sort active tracks deterministically before association
            SortActiveTracksDeterministic();

            // Perform Hungarian Data Association for this modality
            dataAssociation.Associate(
                trackerPointers,
                activeTrackCount,
                modalityDetections,
                modalityCount,
                trackMatches,
                detectionMatches);

            // Update matched tracks with this modality's measurements
            for (int i = 0; i < activeTrackCount; i++)
            {
                TrackRecord track = trackPool[i];
                int matchDetIdx = trackMatches[i];

                if (matchDetIdx >= 0 && matchDetIdx < modalityCount)
                {
                    TargetDetection matchedDet = modalityDetections[matchDetIdx];

                    if (matchedDet.Timestamp >= track.LastUpdateTime - 0.0001f)
                    {
                        track.Tracker.Update(matchedDet);
                        track.LastUpdateTime = matchedDet.Timestamp;
                        track.ConsecutiveMisses = 0;
                        track.WasUpdatedInCurrentCycle = true;
                        track.LatestDetectionConfidence = matchedDet.Confidence;

                        if (matchedDet.Modality == TargetSensorModality.LiDAR)
                        {
                            track.LastLidarDetectionTime = matchedDet.Timestamp;
                        }
                        else if (matchedDet.Modality == TargetSensorModality.Radar)
                        {
                            track.LastRadarDetectionTime = matchedDet.Timestamp;
                        }
                        track.CorroboratingModalityMask |= (1 << (int)matchedDet.Modality);
                    }
                }
            }

            // Initialize new tentative tracks from unassigned detections of this modality
            for (int j = 0; j < modalityCount; j++)
            {
                if (detectionMatches[j] == -1 && activeTrackCount < MaxTracks)
                {
                    TargetDetection unassignedDet = modalityDetections[j];
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
                        newTrack.WasUpdatedInCurrentCycle = true;
                        newTrack.LatestDetectionConfidence = unassignedDet.Confidence;

                        if (unassignedDet.Modality == TargetSensorModality.LiDAR)
                        {
                            newTrack.LastLidarDetectionTime = unassignedDet.Timestamp;
                        }
                        else if (unassignedDet.Modality == TargetSensorModality.Radar)
                        {
                            newTrack.LastRadarDetectionTime = unassignedDet.Timestamp;
                        }
                        newTrack.CorroboratingModalityMask = (1 << (int)unassignedDet.Modality);

                        activeTrackCount++;
                    }
                }
            }
        }

        // 3. Lifecycle Transitions & Composite Confidence Calculation
        for (int i = 0; i < activeTrackCount; i++)
        {
            TrackRecord track = trackPool[i];

            if (track.WasUpdatedInCurrentCycle)
            {
                track.ScanHistory = ((track.ScanHistory << 1) | 1) & 0x1F;

                if (track.Status == TrackStatus.Tentative)
                {
                    if (track.CountHistoryHits() >= promotionHitsRequired)
                    {
                        track.Status = TrackStatus.Confirmed;
                    }
                }
                else if (track.Status == TrackStatus.Coasting || track.Status == TrackStatus.Lost)
                {
                    track.Status = TrackStatus.Confirmed;
                }
            }
            else
            {
                // Unmatched in this entire evaluation cycle
                track.ConsecutiveMisses++;
                track.ScanHistory = (track.ScanHistory << 1) & 0x1F;

                float timeSinceUpdate = currentTime - track.LastUpdateTime;

                if (track.Status == TrackStatus.Tentative)
                {
                    if (track.ConsecutiveMisses >= 2)
                    {
                        track.Status = TrackStatus.Deleted;
                    }
                }
                else if (track.Status == TrackStatus.Confirmed)
                {
                    if (timeSinceUpdate > coastingTimeoutSeconds)
                    {
                        track.Status = TrackStatus.Lost;
                    }
                    else
                    {
                        track.Status = TrackStatus.Coasting;
                    }
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

            // Update active corroboration mask based on active corroboration window
            int activeMask = 0;
            if ((currentTime - track.LastLidarDetectionTime) <= CorroborationWindowSeconds)
            {
                activeMask |= (1 << (int)TargetSensorModality.LiDAR);
            }
            if ((currentTime - track.LastRadarDetectionTime) <= CorroborationWindowSeconds)
            {
                activeMask |= (1 << (int)TargetSensorModality.Radar);
            }
            track.CorroboratingModalityMask = activeMask;

            // Compute composite track confidence
            track.Confidence = ComputeCompositeConfidence(track, currentTime);
        }

        // 4. Prune Deleted Tracks & Compact Active Array in-place
        PruneDeletedTracks();

        // 5. Publish Confirmed & Active Tracks to output buffer
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
                trk.EstimatedExtents,
                trk.CorroboratingModalityMask);
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
    /// Copies all active targets (Tentative, Confirmed, Coasting, Lost) into the caller-provided array.
    /// Returns the number of targets copied.
    /// </summary>
    public int GetAllTargets(TrackedTarget[] outputBuffer, int offset, int maxCount)
    {
        if (outputBuffer == null || offset < 0 || maxCount <= 0) return 0;

        int written = 0;
        for (int i = 0; i < activeTrackCount && written < maxCount && (offset + written) < outputBuffer.Length; i++)
        {
            outputBuffer[offset + written] = publishedTargets[i];
            written++;
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
