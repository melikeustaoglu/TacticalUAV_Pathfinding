using System;

/// <summary>
/// Operational health status of an onboard sensor hardware unit.
/// Compatible with ROS 2 diagnostic_msgs/DiagnosticStatus and PX4 sensor health flags.
/// </summary>
public enum SensorHealth
{
    /// <summary>Sensor is uninitialized or unpowered.</summary>
    Uninitialized,

    /// <summary>Sensor is performing startup self-test or bias calibration.</summary>
    Calibrating,

    /// <summary>Sensor is operating nominally within expected noise and sample rate parameters.</summary>
    Healthy,

    /// <summary>Sensor is degraded (e.g. high dilution of precision, high noise, intermittent packet drops).</summary>
    Degraded,

    /// <summary>Sensor data timed out (no measurements received within expected deadline).</summary>
    Timeout,

    /// <summary>Sensor hardware failure or irrecoverable communication error.</summary>
    Failed
}

/// <summary>
/// Classification of physical sensor modalities on the autonomous UAV.
/// </summary>
public enum SensorType
{
    GPS,
    IMU,
    Barometer,
    Magnetometer,
    Rangefinder,
    Lidar
}
