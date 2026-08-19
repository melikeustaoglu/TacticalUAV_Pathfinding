using System;
using UnityEngine;

/// <summary>
/// Configuration parameters for Satellite Navigation (GNSS/GPS) sensor simulation.
/// Default values model a commercial tactical GNSS receiver (e.g. u-blox ZED-F9P class).
/// </summary>
[Serializable]
public class GpsSensorConfig
{
    [Tooltip("Sensor publication frequency in Hertz.")]
    [Range(1f, 20f)]
    public float updateRateHz = 10.0f;

    [Tooltip("Standard deviation of horizontal position measurement noise (1-sigma, meters).")]
    public float horizontalNoiseSigma = 0.8f;

    [Tooltip("Standard deviation of vertical altitude measurement noise (1-sigma, meters).")]
    public float verticalNoiseSigma = 1.5f;

    [Tooltip("Standard deviation of 3D velocity measurement noise (1-sigma, m/s).")]
    public float velocityNoiseSigma = 0.1f;

    [Tooltip("Simulated transport and filtering latency in seconds.")]
    [Range(0f, 0.5f)]
    public float latencySeconds = 0.05f;

    [Tooltip("Default number of visible satellites.")]
    [Range(4, 24)]
    public int defaultSatellites = 12;

    [Tooltip("Default Position Dilution of Precision (PDOP).")]
    [Range(0.5f, 10f)]
    public float defaultPdop = 1.2f;

    [Tooltip("Deterministic pseudo-random seed (0 = non-deterministic).")]
    public int seed = 42;
}

/// <summary>
/// Configuration parameters for 6-DOF Inertial Measurement Unit (IMU) sensor simulation.
/// Default values model an industrial MEMS IMU (e.g. InvenSense MPU-9250 / Bosch BMI088).
/// </summary>
[Serializable]
public class ImuSensorConfig
{
    [Tooltip("Sensor sampling frequency in Hertz.")]
    [Range(20f, 200f)]
    public float updateRateHz = 100.0f;

    [Tooltip("Standard deviation of accelerometer noise (1-sigma, m/s^2).")]
    public float accelNoiseSigma = 0.05f;

    [Tooltip("Standard deviation of gyroscope angular rate noise (1-sigma, rad/s).")]
    public float gyroNoiseSigma = 0.005f;

    [Tooltip("Initial accelerometer bias vector in body frame (m/s^2).")]
    public Vector3 initialAccelBias = Vector3.zero;

    [Tooltip("Initial gyroscope yaw rate bias in body frame (rad/s).")]
    public float initialGyroYawBias = 0.0f;

    [Tooltip("Deterministic pseudo-random seed (0 = non-deterministic).")]
    public int seed = 43;
}

/// <summary>
/// Configuration parameters for Barometric Altimeter sensor simulation.
/// Default values model a high-precision digital barometric pressure sensor (e.g. Bosch BMP388).
/// </summary>
[Serializable]
public class AltimeterSensorConfig
{
    [Tooltip("Sensor publication frequency in Hertz.")]
    [Range(5f, 50f)]
    public float updateRateHz = 20.0f;

    [Tooltip("Standard deviation of altitude measurement noise (1-sigma, meters).")]
    public float noiseSigma = 0.25f;

    [Tooltip("Standard deviation of vertical velocity measurement noise (1-sigma, m/s).")]
    public float verticalVelocityNoiseSigma = 0.08f;

    [Tooltip("Deterministic pseudo-random seed (0 = non-deterministic).")]
    public int seed = 44;
}
