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

    public ExtendedKalmanFilter EkfCore => ekf;
    public EstimatedState CurrentState => currentState;
    public bool IsEstimatorReady => ekf != null && ekf.IsInitialized;

    // Diagnostics Properties
    public EstimatorStatus Status => ekf != null ? ekf.Status : EstimatorStatus.Uninitialized;
    public GpsFixState GpsState => ekf != null ? ekf.GpsState : GpsFixState.NoFix;
    public int AcceptedMeasurements => ekf != null ? ekf.AcceptedMeasurementsCount : 0;
    public int RejectedMeasurements => ekf != null ? ekf.RejectedMeasurementsCount : 0;
    public float HorizontalPositionStdDev => currentState.HorizontalPositionStandardDeviation;
    public float VerticalPositionStdDev => currentState.VerticalPositionStandardDeviation;
    public float VelocityStdDev => currentState.HorizontalVelocityStandardDeviation;
    public float YawStdDev => Mathf.Sqrt(Mathf.Max(0f, currentState.YawVariance));
    public float LastPredictionTime => lastPredictionTime;
    public float LastGpsCorrectionTime => lastGpsCorrectionTime;
    public float LastBaroCorrectionTime => lastBaroCorrectionTime;

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

    private void PublishState(float timestamp)
    {
        if (ekf == null) return;
        currentState = ekf.GetEstimatedState(timestamp);
        OnStateEstimated?.Invoke(currentState);
    }
}
