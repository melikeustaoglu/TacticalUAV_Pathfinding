using System;
using UnityEngine;

/// <summary>
/// Operational tracking state of an individual target in the multi-target tracker lifecycle.
/// </summary>
public enum TrackStatus
{
    /// <summary>New unconfirmed track candidate under initial 3-of-5 confirmation evaluation.</summary>
    Tentative,

    /// <summary>Validated, actively tracked target published to downstream autonomy and threat assessment.</summary>
    Confirmed,

    /// <summary>Temporary measurement dropout; track state is propagated purely by kinematic prediction.</summary>
    Coasting,

    /// <summary>Prolonged measurement loss (>1.0s); maintained for potential late reacquisition.</summary>
    Lost,

    /// <summary>Dead track marked for pruning; track ID is permanently retired.</summary>
    Deleted
}

/// <summary>
/// Immutable, allocation-conscious representation of a tracked dynamic or static target.
/// Exposes estimated 3D position, 3D velocity, and spatial uncertainty to the autonomy layer
/// with strict ground-truth boundary isolation (zero Unity GameObject/Transform/Collider references).
/// </summary>
public readonly struct TrackedTarget : IEquatable<TrackedTarget>
{
    public int TrackId { get; }
    public Vector3 EstimatedPosition { get; }
    public Vector3 EstimatedVelocity { get; }
    public Vector3 PositionVariance { get; }
    public Vector3 VelocityVariance { get; }
    public TrackStatus Status { get; }
    public float Age { get; }
    public float TimeSinceLastUpdate { get; }
    public float Confidence { get; }
    public Vector3 EstimatedExtents { get; }
    public int CorroboratingModalityMask { get; }

    public bool IsDualSensorCorroborated =>
        (CorroboratingModalityMask & (1 << (int)TargetSensorModality.LiDAR)) != 0 &&
        (CorroboratingModalityMask & (1 << (int)TargetSensorModality.Radar)) != 0;

    public float HorizontalPositionStdDev => Mathf.Sqrt(Mathf.Max(PositionVariance.x, PositionVariance.z));
    public float VerticalPositionStdDev => Mathf.Sqrt(Mathf.Max(0f, PositionVariance.y));
    public float HorizontalVelocityStdDev => Mathf.Sqrt(Mathf.Max(VelocityVariance.x, VelocityVariance.z));
    public float Speed => EstimatedVelocity.magnitude;
    public bool IsValid => TrackId >= 0 && float.IsFinite(EstimatedPosition.x) && Status != TrackStatus.Deleted;

    public static readonly TrackedTarget Empty = new TrackedTarget(
        -1, Vector3.zero, Vector3.zero, Vector3.one * 9999f, Vector3.one * 9999f,
        TrackStatus.Deleted, 0f, 0f, 0f, Vector3.one, 0);

    public TrackedTarget(
        int trackId,
        Vector3 estimatedPosition,
        Vector3 estimatedVelocity,
        Vector3 positionVariance,
        Vector3 velocityVariance,
        TrackStatus status,
        float age,
        float timeSinceLastUpdate,
        float confidence,
        Vector3 estimatedExtents,
        int corroboratingModalityMask = 0)
    {
        TrackId = trackId;
        EstimatedPosition = estimatedPosition;
        EstimatedVelocity = estimatedVelocity;
        PositionVariance = positionVariance;
        VelocityVariance = velocityVariance;
        Status = status;
        Age = Mathf.Max(0f, age);
        TimeSinceLastUpdate = Mathf.Max(0f, timeSinceLastUpdate);
        Confidence = Mathf.Clamp01(confidence);
        EstimatedExtents = estimatedExtents;
        CorroboratingModalityMask = corroboratingModalityMask;
    }

    public bool Equals(TrackedTarget other)
    {
        return TrackId == other.TrackId &&
               Status == other.Status &&
               EstimatedPosition == other.EstimatedPosition &&
               EstimatedVelocity == other.EstimatedVelocity &&
               PositionVariance == other.PositionVariance &&
               VelocityVariance == other.VelocityVariance &&
               Mathf.Approximately(Age, other.Age) &&
               Mathf.Approximately(TimeSinceLastUpdate, other.TimeSinceLastUpdate) &&
               Mathf.Approximately(Confidence, other.Confidence);
    }

    public override bool Equals(object obj) => obj is TrackedTarget other && Equals(other);

    public override int GetHashCode()
    {
        unchecked
        {
            int hash = 17;
            hash = hash * 31 + TrackId.GetHashCode();
            hash = hash * 31 + Status.GetHashCode();
            hash = hash * 31 + EstimatedPosition.GetHashCode();
            return hash;
        }
    }

    public static bool operator ==(TrackedTarget left, TrackedTarget right) => left.Equals(right);
    public static bool operator !=(TrackedTarget left, TrackedTarget right) => !left.Equals(right);

    public override string ToString()
    {
        return $"[Track #{TrackId} {Status} Pos={EstimatedPosition:F2} Vel={EstimatedVelocity:F2} " +
               $"sigma_pos={HorizontalPositionStdDev:F2}m Conf={Confidence:F2} Age={Age:F2}s]";
    }
}
