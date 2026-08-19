using System;
using UnityEngine;

/// <summary>
/// Hardware-simulated digital barometric pressure altimeter sensor.
/// Simulates noisy altitude and climb-rate observations relative to reference datum.
/// </summary>
public class SimulatedBaroAltimeter : MonoBehaviour, IAltimeterSensor
{
    [SerializeField] private AltimeterSensorConfig config = new AltimeterSensorConfig();

    private GaussianNoiseGenerator noiseGen;
    private AltimeterMeasurement currentMeasurement = AltimeterMeasurement.Invalid;
    private SensorHealth health = SensorHealth.Healthy;
    private float referenceAltitude = 0.0f;
    private float lastSampleTime = -10f;
    private bool isActive = true;

    private PathFollower pathFollower;
    private Rigidbody rb;

    public SensorType Type => SensorType.Barometer;
    public SensorHealth Health => health;
    public float UpdateRateHz => config != null ? config.updateRateHz : 20f;
    public float LastMeasurementTime => lastSampleTime;
    public bool IsActive
    {
        get => isActive;
        set => isActive = value;
    }

    public float ReferenceAltitude
    {
        get => referenceAltitude;
        set => referenceAltitude = value;
    }

    public AltimeterMeasurement CurrentMeasurement => currentMeasurement;
    public AltimeterSensorConfig Config => config;

    public event Action<AltimeterMeasurement> OnMeasurementUpdated;

    private void Awake()
    {
        pathFollower = GetComponent<PathFollower>();
        rb = GetComponent<Rigidbody>();
        InitializeSensor();
    }

    private void Update()
    {
        if (!isActive || health == SensorHealth.Failed) return;

        float trueAlt = transform.position.y;
        float vertSpeed = (pathFollower != null)
            ? pathFollower.CurrentVerticalSpeed
            : ((rb != null && !rb.isKinematic) ? rb.linearVelocity.y : 0f);

        UpdateFromSimulationState(trueAlt, vertSpeed, Time.time);
    }

    public void InitializeSensor()
    {
        if (config == null) config = new AltimeterSensorConfig();
        noiseGen = new GaussianNoiseGenerator(config.seed);
        health = SensorHealth.Healthy;
        lastSampleTime = -10f;
    }

    /// <summary>
    /// Updates the barometric observation from true world altitude and vertical velocity.
    /// </summary>
    public bool UpdateFromSimulationState(float trueAltitude, float trueVerticalVelocity, float currentTime)
    {
        if (!isActive || health == SensorHealth.Failed)
        {
            currentMeasurement = AltimeterMeasurement.Invalid;
            return false;
        }

        float sampleInterval = (config.updateRateHz > 0f) ? (1f / config.updateRateHz) : 0.05f;
        if (currentTime - lastSampleTime < sampleInterval - 0.0001f)
        {
            return false;
        }

        if (noiseGen == null)
        {
            noiseGen = new GaussianNoiseGenerator(config.seed);
        }

        lastSampleTime = currentTime;

        // Apply Gaussian noise
        float altNoise = noiseGen.Sample(0f, config.noiseSigma);
        float velNoise = noiseGen.Sample(0f, config.verticalVelocityNoiseSigma);

        float noisyAlt = (trueAltitude - referenceAltitude) + altNoise;
        float noisyVel = trueVerticalVelocity + velNoise;
        float altVariance = config.noiseSigma * config.noiseSigma;

        currentMeasurement = new AltimeterMeasurement(
            noisyAlt,
            noisyVel,
            altVariance,
            currentTime);

        OnMeasurementUpdated?.Invoke(currentMeasurement);
        return true;
    }

    public void SetHealth(SensorHealth newHealth)
    {
        health = newHealth;
        if (health == SensorHealth.Failed)
        {
            currentMeasurement = AltimeterMeasurement.Invalid;
        }
    }

    public void ResetSensor()
    {
        InitializeSensor();
        currentMeasurement = AltimeterMeasurement.Invalid;
    }
}
