using System;

/// <summary>
/// Generic hardware sensor contract for onboard UAV perception and state estimation sensors.
/// Decoupled from Unity Engine objects for high-performance testability and ROS 2 / PX4 bridging.
/// </summary>
/// <typeparam name="TMeasurement">Immutable measurement struct type produced by this sensor.</typeparam>
public interface ISensor<TMeasurement>
{
    /// <summary>Type identifier for this sensor modality.</summary>
    SensorType Type { get; }

    /// <summary>Current operational health status.</summary>
    SensorHealth Health { get; }

    /// <summary>Configured sampling / publication frequency in Hertz.</summary>
    float UpdateRateHz { get; }

    /// <summary>Timestamp of the most recent measurement published (seconds).</summary>
    float LastMeasurementTime { get; }

    /// <summary>Whether this sensor is powered on and publishing observations.</summary>
    bool IsActive { get; set; }

    /// <summary>Gets the latest measurement snapshot produced by this sensor.</summary>
    TMeasurement CurrentMeasurement { get; }

    /// <summary>Reactive event dispatched when a new sensor observation is generated.</summary>
    event Action<TMeasurement> OnMeasurementUpdated;

    /// <summary>Resets internal sensor state, counters, biases, and buffers.</summary>
    void ResetSensor();
}

/// <summary>Specialized contract for Satellite Navigation (GNSS/GPS) sensors.</summary>
public interface IGpsSensor : ISensor<GpsMeasurement>
{
    /// <summary>Current satellite constellation fix status.</summary>
    GpsFixState FixQuality { get; }
}

/// <summary>Specialized contract for 6-DOF Inertial Measurement Units (IMU).</summary>
public interface IImuSensor : ISensor<ImuMeasurement>
{
    /// <summary>Current estimated accelerometer bias vector (m/s^2).</summary>
    UnityEngine.Vector3 AccelerometerBias { get; }

    /// <summary>Current estimated gyroscope yaw rate bias (rad/s).</summary>
    float GyroYawBias { get; }
}

/// <summary>Specialized contract for Barometric Altimeters / Pressure Altitude sensors.</summary>
public interface IAltimeterSensor : ISensor<AltimeterMeasurement>
{
    /// <summary>Reference ground datum altitude (meters).</summary>
    float ReferenceAltitude { get; set; }
}
