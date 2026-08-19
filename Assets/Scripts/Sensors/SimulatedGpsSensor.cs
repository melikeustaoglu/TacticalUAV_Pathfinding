using System;
using UnityEngine;

/// <summary>
/// Hardware-simulated Satellite Navigation (GNSS/GPS) sensor receiver.
/// Transforms true physical kinematic state into noisy, rate-throttled GpsMeasurement observations.
/// </summary>
public class SimulatedGpsSensor : MonoBehaviour, IGpsSensor
{
    [SerializeField] private GpsSensorConfig config = new GpsSensorConfig();
    private GaussianNoiseGenerator noiseGen;
    private GpsMeasurement currentMeasurement = GpsMeasurement.Invalid;
    private SensorHealth health = SensorHealth.Healthy;
    private GpsFixState fixQuality = GpsFixState.Fix3D;
    private float lastSampleTime = -10f;
    private bool isActive = true;
    private PathFollower pathFollower;
    private Rigidbody rb;

    public SensorType Type => SensorType.GPS;
    public SensorHealth Health => health;
    public GpsFixState FixQuality => fixQuality;
    public float UpdateRateHz => config != null ? config.updateRateHz : 10f;
    public float LastMeasurementTime => lastSampleTime;
    public bool IsActive
    {
        get => isActive;
        set => isActive = value;
    }

    public GpsMeasurement CurrentMeasurement => currentMeasurement;
    public GpsSensorConfig Config => config;

    public event Action<GpsMeasurement> OnMeasurementUpdated;

    private void Awake()
    {
        pathFollower = GetComponent<PathFollower>();
        rb = GetComponent<Rigidbody>();
        InitializeSensor();
    }

    private void Update()
    {
        if (!isActive || health == SensorHealth.Failed) return;

        Vector3 truePos = transform.position;
        Vector3 trueVel = (pathFollower != null)
            ? pathFollower.CurrentVelocity
            : ((rb != null && !rb.isKinematic) ? rb.linearVelocity : Vector3.zero);

        UpdateFromSimulationState(truePos, trueVel, Time.time);
    }

    public void InitializeSensor()
    {
        if (config == null) config = new GpsSensorConfig();
        noiseGen = new GaussianNoiseGenerator(config.seed);
        health = SensorHealth.Healthy;
        fixQuality = GpsFixState.Fix3D;
        lastSampleTime = -10f;
    }

    /// <summary>
    /// Updates the GPS sensor reading from simulation ground-truth position and velocity.
    /// Throttled to the configured updateRateHz.
    /// </summary>
    public bool UpdateFromSimulationState(Vector3 truePosition, Vector3 trueVelocity, float currentTime)
    {
        if (!isActive || health == SensorHealth.Failed)
        {
            currentMeasurement = GpsMeasurement.Invalid;
            return false;
        }

        float sampleInterval = (config.updateRateHz > 0f) ? (1f / config.updateRateHz) : 0.1f;
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
        Vector3 posNoise = noiseGen.SampleVector3(config.horizontalNoiseSigma, config.verticalNoiseSigma, config.horizontalNoiseSigma);
        Vector3 velNoise = noiseGen.SampleVector3(config.velocityNoiseSigma, config.velocityNoiseSigma, config.velocityNoiseSigma);

        Vector3 noisyPos = truePosition + posNoise;
        Vector3 noisyVel = trueVelocity + velNoise;

        Vector3 posVariance = new Vector3(
            config.horizontalNoiseSigma * config.horizontalNoiseSigma,
            config.verticalNoiseSigma * config.verticalNoiseSigma,
            config.horizontalNoiseSigma * config.horizontalNoiseSigma);

        Vector3 velVariance = Vector3.one * (config.velocityNoiseSigma * config.velocityNoiseSigma);

        currentMeasurement = new GpsMeasurement(
            noisyPos,
            noisyVel,
            posVariance,
            velVariance,
            currentTime,
            fixQuality,
            config.defaultSatellites,
            config.defaultPdop);

        OnMeasurementUpdated?.Invoke(currentMeasurement);
        return true;
    }

    public void SetHealth(SensorHealth newHealth, GpsFixState newFix = GpsFixState.Fix3D)
    {
        health = newHealth;
        fixQuality = newFix;
        if (health == SensorHealth.Failed || fixQuality == GpsFixState.NoFix)
        {
            currentMeasurement = GpsMeasurement.Invalid;
        }
    }

    public void ResetSensor()
    {
        InitializeSensor();
        currentMeasurement = GpsMeasurement.Invalid;
    }
}
