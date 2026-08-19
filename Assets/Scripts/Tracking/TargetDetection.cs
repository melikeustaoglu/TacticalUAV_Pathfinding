using System;
using UnityEngine;

/// <summary>
/// Physical sensing modality producing target observations.
/// </summary>
public enum TargetSensorModality
{
    LiDAR,
    Radar,
    ElectroOptical
}

/// <summary>
/// Immutable detection contract representing an instantaneous, noisy target observation.
/// Decoupled from ground-truth simulation objects (Transform, Collider, DynamicObstacle).
/// </summary>
public readonly struct TargetDetection : IEquatable<TargetDetection>
{
    public TargetSensorModality Modality { get; }
    public float Timestamp { get; }
    public Vector3 MeasuredPosition { get; }
    public Vector3 MeasuredVelocity { get; }
    public bool HasVelocity { get; }
    public Vector3 PositionVariance { get; }
    public Vector3 VelocityVariance { get; }
    public float Confidence { get; }
    public int DetectionId { get; }

    public static readonly TargetDetection Invalid = new TargetDetection(
        TargetSensorModality.LiDAR,
        -1f,
        Vector3.zero,
        Vector3.zero,
        0f,
        -1,
        Vector3.zero,
        Vector3.zero,
        false);

    public bool IsValid => Confidence > 0.0001f && DetectionId >= 0 && float.IsFinite(MeasuredPosition.x);

    public TargetDetection(
        TargetSensorModality modality,
        float timestamp,
        Vector3 measuredPosition,
        Vector3 positionVariance,
        float confidence,
        int detectionId,
        Vector3 measuredVelocity = default,
        Vector3 velocityVariance = default,
        bool hasVelocity = false)
    {
        Modality = modality;
        Timestamp = timestamp;
        MeasuredPosition = measuredPosition;
        PositionVariance = positionVariance;
        Confidence = Mathf.Clamp01(confidence);
        DetectionId = detectionId;
        MeasuredVelocity = measuredVelocity;
        VelocityVariance = velocityVariance;
        HasVelocity = hasVelocity;
    }

    public bool Equals(TargetDetection other)
    {
        return DetectionId == other.DetectionId &&
               Modality == other.Modality &&
               Mathf.Approximately(Timestamp, other.Timestamp) &&
               MeasuredPosition == other.MeasuredPosition &&
               MeasuredVelocity == other.MeasuredVelocity &&
               HasVelocity == other.HasVelocity &&
               PositionVariance == other.PositionVariance &&
               VelocityVariance == other.VelocityVariance &&
               Mathf.Approximately(Confidence, other.Confidence);
    }

    public override bool Equals(object obj) => obj is TargetDetection other && Equals(other);

    public override int GetHashCode()
    {
        unchecked
        {
            int hash = 17;
            hash = hash * 31 + DetectionId.GetHashCode();
            hash = hash * 31 + Modality.GetHashCode();
            hash = hash * 31 + Timestamp.GetHashCode();
            hash = hash * 31 + MeasuredPosition.GetHashCode();
            return hash;
        }
    }

    public static bool operator ==(TargetDetection left, TargetDetection right) => left.Equals(right);
    public static bool operator !=(TargetDetection left, TargetDetection right) => !left.Equals(right);

    public override string ToString()
    {
        return $"[TargetDetection #{DetectionId} {Modality} @ {Timestamp:F2}s Pos={MeasuredPosition:F2} " +
               $"HasVel={HasVelocity} Vel={MeasuredVelocity:F2} Conf={Confidence:F2}]";
    }
}

/// <summary>
/// Allocation-conscious interface for onboard target detection sensors (LiDAR, Radar).
/// Exposes detection batches via preallocated buffers without per-frame allocations.
/// </summary>
public interface ITargetSensor
{
    TargetSensorModality Modality { get; }
    SensorHealth Health { get; }
    bool IsActive { get; set; }
    float DetectionRange { get; }
    float FieldOfViewAngle { get; }
    float UpdateRateHz { get; }
    int LastDetectionCount { get; }

    void InitializeSensor();
    void SetHealth(SensorHealth newHealth);
    void ResetSensor();

    /// <summary>
    /// Evaluates target sensing at the specified simulation timestamp.
    /// Returns true if a new measurement scan was performed.
    /// </summary>
    bool Evaluate(float simulationTime);

    /// <summary>
    /// Copies active detections from the internal sensor buffer into the caller-provided buffer.
    /// Returns the number of valid detections written.
    /// </summary>
    int TryGetDetections(TargetDetection[] outputBuffer, int offset, int maxCount, float currentTime);

    /// <summary>
    /// Event fired when a new batch of detections is generated.
    /// Passes preallocated array and count without heap allocations.
    /// </summary>
    event Action<TargetDetection[], int> OnDetectionsUpdated;
}
