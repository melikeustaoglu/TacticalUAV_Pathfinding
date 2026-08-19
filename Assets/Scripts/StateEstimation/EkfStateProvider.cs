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
    private float lastBaroCorrectionTime = -1f;

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
    public float LastPredictionTime => lastPredictionTime;
    public float LastGpsCorrectionTime => lastGpsCorrectionTime;
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
        lastPredictionTime = imu.Timestamp;
        ekf.Predict(imu, imu.Timestamp);
        PublishState(imu.Timestamp);
    }

    public void HandleGpsUpdated(GpsMeasurement gps)
    {
        if (ekf == null) return;
        lastGpsCorrectionTime = gps.Timestamp;
        ekf.CorrectGps(gps);
        PublishState(gps.Timestamp);
    }

    public void HandleBaroUpdated(AltimeterMeasurement baro)
    {
        if (ekf == null) return;
        lastBaroCorrectionTime = baro.Timestamp;
        ekf.CorrectBaro(baro);
        PublishState(baro.Timestamp);
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
            if (lastPredictionTime > 0f && (timestamp - lastPredictionTime > imuTimeoutThreshold + 0.0001f))
            {
                effectiveStatus = EstimatorStatus.Failed;
            }
            // 2. GPS Watchdog: If GPS packets cease for > gpsTimeoutThreshold, GNSS lock is lost
            else if (lastGpsCorrectionTime > 0f && (timestamp - lastGpsCorrectionTime > gpsTimeoutThreshold + 0.0001f))
            {
                effectiveGps = GpsFixState.NoFix;

                // If position uncertainty has grown past threshold, transition to Degraded
                if (baseState.HorizontalPositionStandardDeviation > 0.35f)
                {
                    effectiveStatus = EstimatorStatus.Degraded;
                }
            }
            // 3. Covariance-driven degradation
            else if (baseState.HorizontalPositionStandardDeviation > 0.40f)
            {
                effectiveStatus = EstimatorStatus.Degraded;
            }
        }

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
            effectiveGps);

        OnStateEstimated?.Invoke(currentState);
    }
}
