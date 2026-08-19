using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Serializable schedule configuration for deterministic sensor failure injection.
/// </summary>
[Serializable]
public struct SensorFailureSchedule
{
    public SensorType targetSensor;
    public SensorHealth failureHealth;
    public float startTime;
    public float duration;
    public GpsFixState targetFix;

    public SensorFailureSchedule(
        SensorType targetSensor,
        SensorHealth failureHealth,
        float startTime,
        float duration,
        GpsFixState targetFix = GpsFixState.NoFix)
    {
        this.targetSensor = targetSensor;
        this.failureHealth = failureHealth;
        this.startTime = startTime;
        this.duration = duration;
        this.targetFix = targetFix;
    }
}

/// <summary>
/// Autonomous Sensor Failure Injection Component.
/// Provides deterministic and testable control over simulated sensor health (GPS, IMU, Barometer),
/// enabling controlled failure, degradation, timeout, and recovery testing.
/// </summary>
public class SensorFailureInjector : MonoBehaviour
{
    [Header("Scheduled Failures")]
    [SerializeField] private List<SensorFailureSchedule> schedules = new List<SensorFailureSchedule>();

    private SimulatedGpsSensor gpsSensor;
    private SimulatedImuSensor imuSensor;
    private SimulatedBaroAltimeter baroSensor;

    // Manual overrides tracked with expiry times (-1 for indefinite)
    private readonly Dictionary<SensorType, float> manualFailureExpiries = new Dictionary<SensorType, float>();
    private readonly Dictionary<SensorType, SensorHealth> manualFailureHealths = new Dictionary<SensorType, SensorHealth>();
    private readonly Dictionary<SensorType, GpsFixState> manualGpsFixStates = new Dictionary<SensorType, GpsFixState>();

    public IReadOnlyList<SensorFailureSchedule> Schedules => schedules;

    private void Awake()
    {
        InitializeSensors();
    }

    public void InitializeSensors()
    {
        if (gpsSensor == null) gpsSensor = GetComponent<SimulatedGpsSensor>();
        if (imuSensor == null) imuSensor = GetComponent<SimulatedImuSensor>();
        if (baroSensor == null) baroSensor = GetComponent<SimulatedBaroAltimeter>();
    }

    private void Update()
    {
        EvaluateSchedules(Time.time);
    }

    /// <summary>
    /// Evaluates scheduled and manual failure states at a specific simulation timestamp.
    /// Fully deterministic in EditMode and PlayMode.
    /// </summary>
    public void EvaluateSchedules(float currentTime)
    {
        InitializeSensors();

        // 1. Process Manual Overrides
        ProcessManualOverrides(currentTime);

        // 2. Process Timed Schedules
        for (int i = 0; i < schedules.Count; i++)
        {
            SensorFailureSchedule sched = schedules[i];
            bool isInFailureWindow = (currentTime >= sched.startTime) &&
                                     (sched.duration <= 0f || currentTime < sched.startTime + sched.duration);

            if (isInFailureWindow)
            {
                ApplyFailureToSensor(sched.targetSensor, sched.failureHealth, sched.targetFix);
            }
            else if (currentTime >= sched.startTime + sched.duration && sched.duration > 0f)
            {
                // Check if any other manual or scheduled failure is active on this sensor
                if (!IsSensorInManualFailure(sched.targetSensor, currentTime))
                {
                    ApplyFailureToSensor(sched.targetSensor, SensorHealth.Healthy, GpsFixState.Fix3D);
                }
            }
        }
    }

    private void ProcessManualOverrides(float currentTime)
    {
        List<SensorType> expiredKeys = null;

        foreach (var kvp in manualFailureExpiries)
        {
            SensorType sensor = kvp.Key;
            float expiryTime = kvp.Value;

            if (expiryTime > 0f && currentTime >= expiryTime)
            {
                if (expiredKeys == null) expiredKeys = new List<SensorType>();
                expiredKeys.Add(sensor);
            }
            else
            {
                SensorHealth health = manualFailureHealths.ContainsKey(sensor) ? manualFailureHealths[sensor] : SensorHealth.Failed;
                GpsFixState fix = manualGpsFixStates.ContainsKey(sensor) ? manualGpsFixStates[sensor] : GpsFixState.NoFix;
                ApplyFailureToSensor(sensor, health, fix);
            }
        }

        if (expiredKeys != null)
        {
            for (int i = 0; i < expiredKeys.Count; i++)
            {
                RecoverSensor(expiredKeys[i]);
            }
        }
    }

    private bool IsSensorInManualFailure(SensorType sensor, float currentTime)
    {
        if (!manualFailureExpiries.ContainsKey(sensor)) return false;
        float expiry = manualFailureExpiries[sensor];
        return expiry <= 0f || currentTime < expiry;
    }

    private void ApplyFailureToSensor(SensorType sensor, SensorHealth health, GpsFixState fix)
    {
        switch (sensor)
        {
            case SensorType.GPS:
                if (gpsSensor != null) gpsSensor.SetHealth(health, fix);
                break;
            case SensorType.IMU:
                if (imuSensor != null) imuSensor.SetHealth(health);
                break;
            case SensorType.Barometer:
                if (baroSensor != null) baroSensor.SetHealth(health);
                break;
        }
    }

    /// <summary>
    /// Injects a complete failure on the specified sensor modality.
    /// </summary>
    public void InjectFailure(SensorType sensor, float duration = -1f)
    {
        InitializeSensors();
        float expiry = duration > 0f ? (Time.time + duration) : -1f;
        manualFailureExpiries[sensor] = expiry;
        manualFailureHealths[sensor] = SensorHealth.Failed;
        manualGpsFixStates[sensor] = GpsFixState.NoFix;
        ApplyFailureToSensor(sensor, SensorHealth.Failed, GpsFixState.NoFix);
    }

    /// <summary>
    /// Injects a degraded state on the specified sensor modality.
    /// </summary>
    public void InjectDegraded(SensorType sensor, GpsFixState fix = GpsFixState.Degraded, float duration = -1f)
    {
        InitializeSensors();
        float expiry = duration > 0f ? (Time.time + duration) : -1f;
        manualFailureExpiries[sensor] = expiry;
        manualFailureHealths[sensor] = SensorHealth.Degraded;
        manualGpsFixStates[sensor] = fix;
        ApplyFailureToSensor(sensor, SensorHealth.Degraded, fix);
    }

    /// <summary>
    /// Restores the specified sensor to Healthy state.
    /// </summary>
    public void RecoverSensor(SensorType sensor)
    {
        InitializeSensors();
        manualFailureExpiries.Remove(sensor);
        manualFailureHealths.Remove(sensor);
        manualGpsFixStates.Remove(sensor);
        ApplyFailureToSensor(sensor, SensorHealth.Healthy, GpsFixState.Fix3D);
    }

    /// <summary>
    /// Clears all manual overrides and scheduled failures, restoring all sensors to Healthy.
    /// </summary>
    public void ClearAllFailures()
    {
        InitializeSensors();
        manualFailureExpiries.Clear();
        manualFailureHealths.Clear();
        manualGpsFixStates.Clear();
        schedules.Clear();
        ApplyFailureToSensor(SensorType.GPS, SensorHealth.Healthy, GpsFixState.Fix3D);
        ApplyFailureToSensor(SensorType.IMU, SensorHealth.Healthy, GpsFixState.Fix3D);
        ApplyFailureToSensor(SensorType.Barometer, SensorHealth.Healthy, GpsFixState.Fix3D);
    }

    /// <summary>
    /// Adds a timed failure schedule.
    /// </summary>
    public void AddSchedule(SensorFailureSchedule schedule)
    {
        schedules.Add(schedule);
    }
}
