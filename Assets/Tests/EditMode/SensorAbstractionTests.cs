using System.Reflection;
using NUnit.Framework;
using UnityEngine;

/// <summary>
/// Phase 11.2.1 Sensor Abstraction Layer EditMode Tests.
/// Validates sensor measurement contracts, statistical noise generation, rate throttling,
/// health transitions, and ROS 2 / PX4 message mapping compatibility.
/// </summary>
[TestFixture]
public class SensorAbstractionTests
{
    private GameObject sensorHostObj;
    private SimulatedGpsSensor gpsSensor;
    private SimulatedImuSensor imuSensor;
    private SimulatedBaroAltimeter baroSensor;

    [SetUp]
    public void SetUp()
    {
        sensorHostObj = new GameObject("SensorHost");
        gpsSensor = sensorHostObj.AddComponent<SimulatedGpsSensor>();
        imuSensor = sensorHostObj.AddComponent<SimulatedImuSensor>();
        baroSensor = sensorHostObj.AddComponent<SimulatedBaroAltimeter>();

        typeof(SimulatedGpsSensor).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(gpsSensor, null);
        typeof(SimulatedImuSensor).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(imuSensor, null);
        typeof(SimulatedBaroAltimeter).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(baroSensor, null);
    }

    [TearDown]
    public void TearDown()
    {
        if (sensorHostObj != null) Object.DestroyImmediate(sensorHostObj);
    }

    [Test]
    public void GpsMeasurement_CreationAndVarianceMetrics_AreValid()
    {
        Vector3 pos = new Vector3(10f, 2f, 30f);
        Vector3 vel = new Vector3(1.5f, 0f, 2.0f);
        Vector3 posVar = new Vector3(0.64f, 2.25f, 0.64f); // 0.8m horizontal, 1.5m vertical sigma
        Vector3 velVar = Vector3.one * 0.01f; // 0.1 m/s sigma

        GpsMeasurement meas = new GpsMeasurement(pos, vel, posVar, velVar, 5.0f, GpsFixState.Fix3D, 14, 1.1f);

        Assert.IsTrue(meas.IsValid);
        Assert.AreEqual(pos, meas.Position);
        Assert.AreEqual(vel, meas.Velocity);
        Assert.AreEqual(0.8f, meas.HorizontalAccuracy, 0.001f);
        Assert.AreEqual(1.5f, meas.VerticalAccuracy, 0.001f);
        Assert.AreEqual(5.0f, meas.Timestamp, 0.001f);
        Assert.AreEqual(GpsFixState.Fix3D, meas.FixQuality);
        Assert.AreEqual(14, meas.SatellitesVisible);
        Assert.AreEqual(1.1f, meas.DilutionOfPrecision, 0.001f);
    }

    [Test]
    public void GpsMeasurement_InvalidState_ReportsCorrectly()
    {
        GpsMeasurement invalid = GpsMeasurement.Invalid;
        Assert.IsFalse(invalid.IsValid);
        Assert.AreEqual(GpsFixState.NoFix, invalid.FixQuality);
        Assert.AreEqual(0, invalid.SatellitesVisible);
    }

    [Test]
    public void ImuMeasurement_CreationAndSpecificForce_AreValid()
    {
        Vector3 accel = new Vector3(0.1f, 9.81f, -0.2f);
        Vector3 rates = new Vector3(0.01f, 0.05f, -0.02f);
        Vector3 accelVar = Vector3.one * 0.0025f;
        Vector3 gyroVar = Vector3.one * 0.000025f;

        ImuMeasurement meas = new ImuMeasurement(accel, rates, accelVar, gyroVar, 10.0f);

        Assert.IsTrue(meas.IsValid);
        Assert.AreEqual(accel, meas.LinearAcceleration);
        Assert.AreEqual(rates, meas.AngularVelocity);
        Assert.AreEqual(10.0f, meas.Timestamp, 0.001f);
    }

    [Test]
    public void AltimeterMeasurement_CreationAndAccuracy_AreValid()
    {
        AltimeterMeasurement meas = new AltimeterMeasurement(25.4f, 0.5f, 0.0625f, 8.2f);

        Assert.IsTrue(meas.IsValid);
        Assert.AreEqual(25.4f, meas.Altitude, 0.001f);
        Assert.AreEqual(0.5f, meas.VerticalVelocity, 0.001f);
        Assert.AreEqual(0.25f, meas.Accuracy, 0.001f);
        Assert.AreEqual(8.2f, meas.Timestamp, 0.001f);
    }

    [Test]
    public void GaussianNoiseGenerator_DeterministicSeed_ProducesRepeatableDistribution()
    {
        GaussianNoiseGenerator gen1 = new GaussianNoiseGenerator(12345);
        GaussianNoiseGenerator gen2 = new GaussianNoiseGenerator(12345);

        float sum = 0f;
        int count = 100;
        for (int i = 0; i < count; i++)
        {
            float s1 = gen1.Sample(0f, 1f);
            float s2 = gen2.Sample(0f, 1f);
            Assert.AreEqual(s1, s2, 0.00001f, "Deterministic generator must produce identical values for identical seed!");
            sum += s1;
        }

        float mean = sum / count;
        Assert.Less(Mathf.Abs(mean), 0.5f, "Sample mean of 100 standard normal samples should be close to 0!");
    }

    [Test]
    public void SimulatedGpsSensor_RateThrottlingAndNoise_BehavesDeterministically()
    {
        gpsSensor.Config.updateRateHz = 10.0f; // 0.1s interval
        gpsSensor.Config.seed = 999;
        gpsSensor.InitializeSensor();

        int eventFiredCount = 0;
        gpsSensor.OnMeasurementUpdated += m => eventFiredCount++;

        Vector3 truePos = new Vector3(10f, 2f, 10f);
        Vector3 trueVel = new Vector3(1f, 0f, 0f);

        // First sample at t = 0.0s -> Success
        bool sampled1 = gpsSensor.UpdateFromSimulationState(truePos, trueVel, 0.0f);
        Assert.IsTrue(sampled1);
        Assert.AreEqual(1, eventFiredCount);
        Assert.IsTrue(gpsSensor.CurrentMeasurement.IsValid);
        Assert.AreNotEqual(truePos, gpsSensor.CurrentMeasurement.Position, "Measurement must include noise!");

        // Immediate subsequent sample at t = 0.03s (< 0.1s interval) -> Throttled
        bool sampled2 = gpsSensor.UpdateFromSimulationState(truePos, trueVel, 0.03f);
        Assert.IsFalse(sampled2, "Sensor update must be throttled by updateRateHz!");
        Assert.AreEqual(1, eventFiredCount);

        // Subsequent sample at t = 0.11s (> 0.1s interval) -> Success
        bool sampled3 = gpsSensor.UpdateFromSimulationState(truePos, trueVel, 0.11f);
        Assert.IsTrue(sampled3);
        Assert.AreEqual(2, eventFiredCount);
    }

    [Test]
    public void SimulatedGpsSensor_HealthTransitions_InvalidatesMeasurementWhenFailed()
    {
        gpsSensor.UpdateFromSimulationState(new Vector3(5f, 1f, 5f), Vector3.zero, 0.0f);
        Assert.IsTrue(gpsSensor.CurrentMeasurement.IsValid);

        gpsSensor.SetHealth(SensorHealth.Failed);
        Assert.AreEqual(SensorHealth.Failed, gpsSensor.Health);
        Assert.IsFalse(gpsSensor.CurrentMeasurement.IsValid);

        // Subsequent update attempt while failed must return false
        bool updated = gpsSensor.UpdateFromSimulationState(new Vector3(5f, 1f, 5f), Vector3.zero, 1.0f);
        Assert.IsFalse(updated);
    }

    [Test]
    public void SimulatedImuSensor_KinematicsTransformation_ComputesSpecificForceAndRates()
    {
        imuSensor.Config.updateRateHz = 100.0f; // 10ms interval
        imuSensor.InitializeSensor();

        Vector3 trueAccelWorld = Vector3.zero; // Hover / level flight
        Vector3 trueRatesWorld = new Vector3(0f, 0.5f, 0f); // 0.5 rad/s yaw rate
        Quaternion levelRot = Quaternion.identity;

        bool updated = imuSensor.UpdateFromKinematics(trueAccelWorld, trueRatesWorld, levelRot, 0.0f);
        Assert.IsTrue(updated);

        ImuMeasurement meas = imuSensor.CurrentMeasurement;
        Assert.IsTrue(meas.IsValid);

        // At rest, specific force opposes gravity: a_meas_y ≈ +9.81 m/s^2 (Physics.gravity is -9.81)
        Assert.AreEqual(9.81f, meas.LinearAcceleration.y, 0.5f, "Specific force at rest must balance gravity!");
        Assert.AreEqual(0.5f, meas.AngularVelocity.y, 0.1f, "Yaw rate must match true angular velocity!");
    }

    [Test]
    public void SimulatedBaroAltimeter_ReferenceDatumAndNoise_CalculatesRelativeAltitude()
    {
        baroSensor.Config.updateRateHz = 20.0f;
        baroSensor.ReferenceAltitude = 5.0f; // Reference ground datum is at 5m
        baroSensor.InitializeSensor();

        bool updated = baroSensor.UpdateFromSimulationState(15.0f, 1.2f, 0.0f);
        Assert.IsTrue(updated);

        AltimeterMeasurement meas = baroSensor.CurrentMeasurement;
        Assert.IsTrue(meas.IsValid);

        // Measured relative altitude = (15.0 - 5.0) ± noise ≈ 10.0m
        Assert.AreEqual(10.0f, meas.Altitude, 1.0f);
        Assert.AreEqual(1.2f, meas.VerticalVelocity, 0.5f);
    }

    [Test]
    public void SensorContracts_ROS2AndPX4Mapping_PreservesAllRequiredFields()
    {
        // ROS 2 sensor_msgs/NavSatFix compatibility check
        IGpsSensor gps = gpsSensor;
        Assert.AreEqual(SensorType.GPS, gps.Type);
        Assert.IsNotNull(gps.CurrentMeasurement);

        // ROS 2 sensor_msgs/Imu compatibility check
        IImuSensor imu = imuSensor;
        Assert.AreEqual(SensorType.IMU, imu.Type);
        Assert.IsNotNull(imu.CurrentMeasurement);

        // ROS 2 sensor_msgs/Range / PX4 SensorBaro compatibility check
        IAltimeterSensor baro = baroSensor;
        Assert.AreEqual(SensorType.Barometer, baro.Type);
        Assert.IsNotNull(baro.CurrentMeasurement);
    }
}
