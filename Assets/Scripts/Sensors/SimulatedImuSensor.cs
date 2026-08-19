using System;
using UnityEngine;

/// <summary>
/// Hardware-simulated 6-DOF Inertial Measurement Unit (IMU) sensor.
/// Simulates body-frame specific force (linear acceleration) and angular body rates with noise and bias.
/// </summary>
public class SimulatedImuSensor : MonoBehaviour, IImuSensor
{
    [SerializeField] private ImuSensorConfig config = new ImuSensorConfig();

    private GaussianNoiseGenerator noiseGen;
    private ImuMeasurement currentMeasurement = ImuMeasurement.Invalid;
    private SensorHealth health = SensorHealth.Healthy;
    private Vector3 currentAccelBias = Vector3.zero;
    private float currentGyroYawBias = 0.0f;
    private float lastSampleTime = -10f;
    private bool isActive = true;

    private PathFollower pathFollower;
    private Rigidbody rb;
    private Vector3 lastVelocity = Vector3.zero;
    private float lastYawDeg = 0f;
    private float lastKinematicsTime = -10f;

    public SensorType Type => SensorType.IMU;
    public SensorHealth Health => health;
    public float UpdateRateHz => config != null ? config.updateRateHz : 100f;
    public float LastMeasurementTime => lastSampleTime;
    public bool IsActive
    {
        get => isActive;
        set => isActive = value;
    }

    public Vector3 AccelerometerBias => currentAccelBias;
    public float GyroYawBias => currentGyroYawBias;
    public ImuMeasurement CurrentMeasurement => currentMeasurement;
    public ImuSensorConfig Config => config;

    public event Action<ImuMeasurement> OnMeasurementUpdated;

    private void Awake()
    {
        pathFollower = GetComponent<PathFollower>();
        rb = GetComponent<Rigidbody>();
        InitializeSensor();
    }

    private void FixedUpdate()
    {
        if (!isActive || health == SensorHealth.Failed) return;

        Vector3 currentVel = (pathFollower != null)
            ? pathFollower.CurrentVelocity
            : ((rb != null && !rb.isKinematic) ? rb.linearVelocity : Vector3.zero);

        float currentYaw = transform.eulerAngles.y;
        float dt = (lastKinematicsTime > 0f) ? (Time.time - lastKinematicsTime) : Time.fixedDeltaTime;
        lastKinematicsTime = Time.time;

        if (dt <= 0.0001f) dt = 0.02f;

        Vector3 worldAccel = (currentVel - lastVelocity) / dt;
        lastVelocity = currentVel;

        float yawDeltaDeg = Mathf.DeltaAngle(lastYawDeg, currentYaw);
        lastYawDeg = currentYaw;
        float yawRateRad = (yawDeltaDeg * Mathf.Deg2Rad) / dt;

        Vector3 bodyRates = new Vector3(0f, yawRateRad, 0f);

        UpdateFromKinematics(worldAccel, bodyRates, transform.rotation, Time.time);
    }

    public void InitializeSensor()
    {
        if (config == null) config = new ImuSensorConfig();
        noiseGen = new GaussianNoiseGenerator(config.seed);
        health = SensorHealth.Healthy;
        currentAccelBias = config.initialAccelBias;
        currentGyroYawBias = config.initialGyroYawBias;
        lastSampleTime = -10f;
    }

    /// <summary>
    /// Updates the IMU observation from true kinematics (world acceleration, world angular velocity, attitude).
    /// </summary>
    public bool UpdateFromKinematics(
        Vector3 trueWorldAcceleration,
        Vector3 trueWorldAngularVelocity,
        Quaternion trueOrientation,
        float currentTime)
    {
        if (!isActive || health == SensorHealth.Failed)
        {
            currentMeasurement = ImuMeasurement.Invalid;
            return false;
        }

        float sampleInterval = (config.updateRateHz > 0f) ? (1f / config.updateRateHz) : 0.01f;
        if (currentTime - lastSampleTime < sampleInterval - 0.00001f)
        {
            return false;
        }

        if (noiseGen == null)
        {
            noiseGen = new GaussianNoiseGenerator(config.seed);
        }

        lastSampleTime = currentTime;

        // Specific force: a_body = R_world_to_body * (a_world - gravity)
        Vector3 trueSpecificForceWorld = trueWorldAcceleration - Physics.gravity;
        Vector3 trueSpecificForceBody = Quaternion.Inverse(trueOrientation) * trueSpecificForceWorld;
        Vector3 trueBodyRates = Quaternion.Inverse(trueOrientation) * trueWorldAngularVelocity;

        // Apply noise and bias
        Vector3 accelNoise = noiseGen.SampleVector3(config.accelNoiseSigma, config.accelNoiseSigma, config.accelNoiseSigma);
        Vector3 gyroNoise = noiseGen.SampleVector3(config.gyroNoiseSigma, config.gyroNoiseSigma, config.gyroNoiseSigma);

        Vector3 noisyAccel = trueSpecificForceBody + currentAccelBias + accelNoise;
        Vector3 noisyRates = trueBodyRates + new Vector3(0f, currentGyroYawBias, 0f) + gyroNoise;

        Vector3 accelVariance = Vector3.one * (config.accelNoiseSigma * config.accelNoiseSigma);
        Vector3 gyroVariance = Vector3.one * (config.gyroNoiseSigma * config.gyroNoiseSigma);

        currentMeasurement = new ImuMeasurement(
            noisyAccel,
            noisyRates,
            accelVariance,
            gyroVariance,
            currentTime);

        OnMeasurementUpdated?.Invoke(currentMeasurement);
        return true;
    }

    public void SetBias(Vector3 accelBias, float gyroYawBias)
    {
        currentAccelBias = accelBias;
        currentGyroYawBias = gyroYawBias;
    }

    public void SetHealth(SensorHealth newHealth)
    {
        health = newHealth;
        if (health == SensorHealth.Failed)
        {
            currentMeasurement = ImuMeasurement.Invalid;
        }
    }

    public void ResetSensor()
    {
        InitializeSensor();
        currentMeasurement = ImuMeasurement.Invalid;
    }
}
