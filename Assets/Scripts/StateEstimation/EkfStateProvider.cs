using System;
using UnityEngine;

/// <summary>
/// Onboard State Estimation Service Provider wrapping the Extended Kalman Filter (EKF) core engine.
/// Consumes hardware sensor interfaces (GPS, IMU, Baro) and publishes EstimatedState to the autonomy stack.
/// </summary>
public class EkfStateProvider : MonoBehaviour, IEstimatedStateProvider
{
    private ExtendedKalmanFilter ekf;
    private IGpsSensor gpsSensor;
    private IImuSensor imuSensor;
    private IAltimeterSensor baroSensor;
    private EstimatedState currentState = EstimatedState.Uninitialized;

    private float lastPredictionTime = -1f;
    private float lastGpsCorrectionTime = -1f;
    private float lastAcceptedGpsCorrectionTime = -1f;
    private float lastBaroCorrectionTime = -1f;
    private bool hasReceivedImu = false;
    private bool hasReceivedGps = false;
    private bool hasReceivedBaro = false;

    [Header("Watchdog Timeout Thresholds (Seconds)")]
    [Tooltip("Maximum allowable time without GPS updates before marking fix lost (default 0.50s = 5 missed frames @ 10Hz).")]
    [SerializeField] private float gpsTimeoutThreshold = 0.50f;

    [Tooltip("Maximum allowable time without IMU updates before marking estimator failed (default 0.10s = 10 missed frames @ 100Hz).")]
    [SerializeField] private float imuTimeoutThreshold = 0.10f;

    [Tooltip("Maximum allowable time without Barometer updates before marking degraded vertical sensing (default 0.30s = 6 missed frames @ 20Hz).")]
    [SerializeField] private float baroTimeoutThreshold = 0.30f;

    public ExtendedKalmanFilter EkfCore => ekf;
    public EstimatedState CurrentState => currentState;
    public bool IsEstimatorReady => ekf != null && ekf.IsInitialized;

    // Diagnostics Properties
    public EstimatorStatus Status => currentState.Status;
    public GpsFixState GpsState => currentState.GpsState;
    public int AcceptedMeasurements => ekf != null ? ekf.AcceptedMeasurementsCount : 0;
    public int RejectedMeasurements => ekf != null ? ekf.RejectedMeasurementsCount : 0;
    public float HorizontalPositionStdDev => currentState.HorizontalPositionStandardDeviation;
    public float VerticalPositionStdDev => currentState.VerticalPositionStandardDeviation;
    public float VelocityStdDev => currentState.HorizontalVelocityStandardDeviation;
    public float YawStdDev => Mathf.Sqrt(Mathf.Max(0f, currentState.YawVariance));
    public float NavigationConfidence => currentState.NavigationConfidence;
    public float DeadReckoningDuration => currentState.DeadReckoningDuration;
    public float LastPredictionTime => lastPredictionTime;
    public float LastGpsCorrectionTime => lastGpsCorrectionTime;
    public float LastAcceptedGpsCorrectionTime => lastAcceptedGpsCorrectionTime;
    public float LastBaroCorrectionTime => lastBaroCorrectionTime;

    public float GpsTimeoutThreshold
    {
        get => gpsTimeoutThreshold;
        set => gpsTimeoutThreshold = Mathf.Max(0.01f, value);
    }

    public float ImuTimeoutThreshold
    {
        get => imuTimeoutThreshold;
        set => imuTimeoutThreshold = Mathf.Max(0.01f, value);
    }

    public float BaroTimeoutThreshold
    {
        get => baroTimeoutThreshold;
        set => baroTimeoutThreshold = Mathf.Max(0.01f, value);
    }

    public event Action<EstimatedState> OnStateEstimated;

    private void Awake()
    {
        InitializeProvider();
    }

    public void InitializeProvider()
    {
        if (ekf == null) ekf = new ExtendedKalmanFilter();
        if (gpsSensor == null) gpsSensor = GetComponent<IGpsSensor>();
        if (imuSensor == null) imuSensor = GetComponent<IImuSensor>();
        if (baroSensor == null) baroSensor = GetComponent<IAltimeterSensor>();
        hasReceivedImu = false;
        hasReceivedGps = false;
        hasReceivedBaro = false;
        lastPredictionTime = -1f;
        lastGpsCorrectionTime = -1f;
        lastAcceptedGpsCorrectionTime = -1f;
        lastBaroCorrectionTime = -1f;
    }

    private void OnEnable()
    {
        InitializeProvider();
        if (gpsSensor != null) gpsSensor.OnMeasurementUpdated += HandleGpsUpdated;
        if (imuSensor != null) imuSensor.OnMeasurementUpdated += HandleImuUpdated;
        if (baroSensor != null) baroSensor.OnMeasurementUpdated += HandleBaroUpdated;
    }

    private void OnDisable()
    {
        if (gpsSensor != null) gpsSensor.OnMeasurementUpdated -= HandleGpsUpdated;
        if (imuSensor != null) imuSensor.OnMeasurementUpdated -= HandleImuUpdated;
        if (baroSensor != null) baroSensor.OnMeasurementUpdated -= HandleBaroUpdated;
    }

    private void Update()
    {
        CheckTimeouts(Time.time);
    }

    /// <summary>
    /// Checks sensor timeouts against simulation/game time and updates state health.
    /// Deterministic in both EditMode and PlayMode.
    /// </summary>
    public void CheckTimeouts(float currentTime)
    {
        if (ekf == null || !ekf.IsInitialized) return;

        PublishState(currentTime);
    }

    public void HandleImuUpdated(ImuMeasurement imu)
    {
        if (ekf == null) return;
        hasReceivedImu = true;
        lastPredictionTime = imu.Timestamp;
        ekf.Predict(imu, imu.Timestamp);
        PublishState(imu.Timestamp);
    }

    public void HandleGpsUpdated(GpsMeasurement gps)
    {
        if (ekf == null) return;
        hasReceivedGps = true;
        lastGpsCorrectionTime = gps.Timestamp;
        bool accepted = ekf.CorrectGps(gps);
        if (accepted)
        {
            lastAcceptedGpsCorrectionTime = gps.Timestamp;
        }
        PublishState(gps.Timestamp);
    }

    public void HandleBaroUpdated(AltimeterMeasurement baro)
    {
        if (ekf == null) return;
        hasReceivedBaro = true;
        lastBaroCorrectionTime = baro.Timestamp;
        ekf.CorrectBaro(baro);
        PublishState(baro.Timestamp);
    }

    /// <summary>
    /// Computes the composite navigation confidence score C_nav in [0.0, 1.0] from physical observables:
    /// 1. GPS Availability (Fix3D/Differential = 1.00, Fix2D/Degraded = 0.70, NoFix = 0.35)
    /// 2. Spatial Horizontal Uncertainty (1 / (1 + max(0, sigma_horiz - 0.15)))
    /// 3. Dead-Reckoning Temporal Duration (1 / (1 + 0.20 * T_dr))
    /// Weighted sum: w_gps=0.40, w_uncert=0.35, w_time=0.25, scaled by StatusModifier (Nominal=1.0, Degraded=0.85, Failed=0.0).
    /// </summary>
    public static float ComputeNavigationConfidence(
        EstimatorStatus status,
        GpsFixState gpsState,
        float horizStdDev,
        float deadReckoningDuration)
    {
        if (status == EstimatorStatus.Failed || status == EstimatorStatus.Uninitialized)
        {
            return 0f;
        }

        // 1. GPS Availability Factor (weight = 0.40)
        float fGps;
        switch (gpsState)
        {
            case GpsFixState.Differential:
            case GpsFixState.Fix3D:
                fGps = 1.00f;
                break;
            case GpsFixState.Fix2D:
            case GpsFixState.Degraded:
                fGps = 0.70f;
                break;
            case GpsFixState.NoFix:
            default:
                fGps = 0.35f;
                break;
        }

        // 2. Spatial Horizontal Uncertainty Factor (weight = 0.35)
        float excessSigma = Mathf.Max(0f, horizStdDev - 0.15f);
        float fUncert = 1.0f / (1.0f + excessSigma);

        // 3. Dead-Reckoning Temporal Factor (weight = 0.25)
        float fTime = 1.0f / (1.0f + 0.20f * Mathf.Max(0f, deadReckoningDuration));

        // Weighted composite sum
        float composite = 0.40f * fGps + 0.35f * fUncert + 0.25f * fTime;

        // Status modifier
        if (status == EstimatorStatus.Degraded)
        {
            composite *= 0.85f;
        }

        return Mathf.Clamp01(composite);
    }

    public void PublishState(float timestamp)
    {
        if (ekf == null) return;
        EstimatedState baseState = ekf.GetEstimatedState(timestamp);

        // Dynamic Watchdog Health & Timeout Evaluation
        EstimatorStatus effectiveStatus = baseState.Status;
        GpsFixState effectiveGps = baseState.GpsState;

        if (ekf.IsInitialized)
        {
            // 1. IMU Watchdog: If IMU packets cease for > imuTimeoutThreshold, strapdown integration is impossible
            if (hasReceivedImu && (timestamp - lastPredictionTime > imuTimeoutThreshold + 0.0001f))
            {
                effectiveStatus = EstimatorStatus.Failed;
            }
            // 2. GPS Watchdog: If GPS packets cease for > gpsTimeoutThreshold, GNSS lock is lost
            else if (hasReceivedGps && (timestamp - lastGpsCorrectionTime > gpsTimeoutThreshold + 0.0001f))
            {
                effectiveGps = GpsFixState.NoFix;
                effectiveStatus = EstimatorStatus.Degraded;
            }
            // 3. Healthy GPS status maintenance
            else if (hasReceivedGps && effectiveGps != GpsFixState.NoFix)
            {
                effectiveStatus = EstimatorStatus.Nominal;
            }
        }

        float drDuration = (lastAcceptedGpsCorrectionTime > 0f && timestamp >= lastAcceptedGpsCorrectionTime)
            ? (timestamp - lastAcceptedGpsCorrectionTime)
            : 0f;
        float horizStdDev = baseState.HorizontalPositionStandardDeviation;
        float navConfidence = ComputeNavigationConfidence(effectiveStatus, effectiveGps, horizStdDev, drDuration);

        currentState = new EstimatedState(
            baseState.Position,
            baseState.Velocity,
            baseState.YawDegrees,
            baseState.PitchDegrees,
            baseState.AccelerometerBias,
            baseState.GyroYawBias,
            baseState.PositionVariance,
            baseState.VelocityVariance,
            baseState.YawVariance,
            timestamp,
            effectiveStatus,
            effectiveGps,
            navConfidence,
            drDuration);

        OnStateEstimated?.Invoke(currentState);
    }
}
