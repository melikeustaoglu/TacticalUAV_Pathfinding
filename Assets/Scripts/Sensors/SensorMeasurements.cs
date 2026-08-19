using System;
using UnityEngine;

/// <summary>
/// Immutable, allocation-free satellite navigation measurement snapshot.
/// Formatted for direct mapping to ROS 2 sensor_msgs/NavSatFix and PX4 SensorGps.
/// </summary>
[Serializable]
public struct GpsMeasurement
{
    /// <summary>Estimated 3D position vector in local navigation frame (meters).</summary>
    public Vector3 Position { get; }

    /// <summary>Estimated 3D velocity vector in local navigation frame (m/s).</summary>
    public Vector3 Velocity { get; }

    /// <summary>Diagonal position variance [Var(X), Var(Y), Var(Z)] in m^2.</summary>
    public Vector3 PositionVariance { get; }

    /// <summary>Diagonal velocity variance [Var(Vx), Var(Vy), Var(Vz)] in (m/s)^2.</summary>
    public Vector3 VelocityVariance { get; }

    /// <summary>Measurement publication timestamp (seconds).</summary>
    public float Timestamp { get; }

    /// <summary>Satellite constellation fix status.</summary>
    public GpsFixState FixQuality { get; }

    /// <summary>Number of tracked satellites.</summary>
    public int SatellitesVisible { get; }

    /// <summary>Position Dilution of Precision (PDOP).</summary>
    public float DilutionOfPrecision { get; }

    /// <summary>Horizontal position standard deviation (1-sigma, meters).</summary>
    public float HorizontalAccuracy => Mathf.Sqrt(Mathf.Max(0f, Mathf.Max(PositionVariance.x, PositionVariance.z)));

    /// <summary>Vertical altitude standard deviation (1-sigma, meters).</summary>
    public float VerticalAccuracy => Mathf.Sqrt(Mathf.Max(0f, PositionVariance.y));

    /// <summary>Whether this measurement contains a valid navigation fix.</summary>
    public bool IsValid => FixQuality != GpsFixState.NoFix &&
                           float.IsFinite(Position.x) && float.IsFinite(Position.y) && float.IsFinite(Position.z) &&
                           float.IsFinite(Velocity.x) && float.IsFinite(Velocity.y) && float.IsFinite(Velocity.z);

    public GpsMeasurement(
        Vector3 position,
        Vector3 velocity,
        Vector3 positionVariance,
        Vector3 velocityVariance,
        float timestamp,
        GpsFixState fixQuality = GpsFixState.Fix3D,
        int satellitesVisible = 12,
        float dilutionOfPrecision = 1.2f)
    {
        Position = position;
        Velocity = velocity;
        PositionVariance = positionVariance;
        VelocityVariance = velocityVariance;
        Timestamp = timestamp;
        FixQuality = fixQuality;
        SatellitesVisible = Mathf.Max(0, satellitesVisible);
        DilutionOfPrecision = Mathf.Max(0.5f, dilutionOfPrecision);
    }

    /// <summary>Represents an invalid or unacquired GPS measurement.</summary>
    public static GpsMeasurement Invalid => new GpsMeasurement(
        Vector3.zero,
        Vector3.zero,
        Vector3.one * 9999f,
        Vector3.one * 9999f,
        0f,
        GpsFixState.NoFix,
        0,
        99.0f);
}

/// <summary>
/// Immutable, allocation-free Inertial Measurement Unit (IMU) observation snapshot.
/// Formatted for direct mapping to ROS 2 sensor_msgs/Imu and PX4 SensorCombined.
/// </summary>
[Serializable]
public struct ImuMeasurement
{
    /// <summary>Specific force / linear acceleration in body frame (m/s^2).</summary>
    public Vector3 LinearAcceleration { get; }

    /// <summary>Angular velocity / body rates [p, q, r] in body frame (rad/s).</summary>
    public Vector3 AngularVelocity { get; }

    /// <summary>Diagonal acceleration variance [Var(Ax), Var(Ay), Var(Az)] in (m/s^2)^2.</summary>
    public Vector3 AccelerationVariance { get; }

    /// <summary>Diagonal angular velocity variance [Var(Wx), Var(Wy), Var(Wz)] in (rad/s)^2.</summary>
    public Vector3 AngularVelocityVariance { get; }

    /// <summary>Measurement timestamp (seconds).</summary>
    public float Timestamp { get; }

    /// <summary>Whether this measurement contains finite, valid IMU readings.</summary>
    public bool IsValid => float.IsFinite(LinearAcceleration.x) && float.IsFinite(LinearAcceleration.y) && float.IsFinite(LinearAcceleration.z) &&
                           float.IsFinite(AngularVelocity.x) && float.IsFinite(AngularVelocity.y) && float.IsFinite(AngularVelocity.z);

    public ImuMeasurement(
        Vector3 linearAcceleration,
        Vector3 angularVelocity,
        Vector3 accelerationVariance,
        Vector3 angularVelocityVariance,
        float timestamp)
    {
        LinearAcceleration = linearAcceleration;
        AngularVelocity = angularVelocity;
        AccelerationVariance = accelerationVariance;
        AngularVelocityVariance = angularVelocityVariance;
        Timestamp = timestamp;
    }

    /// <summary>Represents an invalid or uncalibrated IMU observation.</summary>
    public static ImuMeasurement Invalid => new ImuMeasurement(
        Vector3.zero,
        Vector3.zero,
        Vector3.one * 9999f,
        Vector3.one * 9999f,
        0f);
}

/// <summary>
/// Immutable, allocation-free Barometric Altimeter measurement snapshot.
/// Formatted for direct mapping to ROS 2 sensor_msgs/Range and PX4 SensorBaro.
/// </summary>
[Serializable]
public struct AltimeterMeasurement
{
    /// <summary>Measured altitude above ground / reference datum (meters).</summary>
    public float Altitude { get; }

    /// <summary>Measured vertical climb/descent velocity (m/s).</summary>
    public float VerticalVelocity { get; }

    /// <summary>Altitude variance in m^2.</summary>
    public float AltitudeVariance { get; }

    /// <summary>Measurement timestamp (seconds).</summary>
    public float Timestamp { get; }

    /// <summary>Standard deviation of altitude measurement (1-sigma, meters).</summary>
    public float Accuracy => Mathf.Sqrt(Mathf.Max(0f, AltitudeVariance));

    /// <summary>Whether this altitude observation is finite and physically valid.</summary>
    public bool IsValid => float.IsFinite(Altitude) && float.IsFinite(VerticalVelocity) && AltitudeVariance >= 0f;

    public AltimeterMeasurement(
        float altitude,
        float verticalVelocity,
        float altitudeVariance,
        float timestamp)
    {
        Altitude = altitude;
        VerticalVelocity = verticalVelocity;
        AltitudeVariance = Mathf.Max(0f, altitudeVariance);
        Timestamp = timestamp;
    }

    /// <summary>Represents an invalid altimeter measurement.</summary>
    public static AltimeterMeasurement Invalid => new AltimeterMeasurement(
        float.NaN,
        float.NaN,
        9999f,
        0f);
}
