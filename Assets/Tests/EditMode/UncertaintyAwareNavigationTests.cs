using System.Collections.Generic;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;

/// <summary>
/// Phase 11.5 Uncertainty-Aware Navigation Decision Layer Tests.
/// Validates dynamic cruise speed scaling, emergency TTC expansion, altitude recovery safety gating,
/// and backward compatibility under nominal/zero covariance.
/// </summary>
[TestFixture]
public class UncertaintyAwareNavigationTests
{
    private GameObject uavObj;
    private SimulatedGpsSensor gpsSensor;
    private SimulatedImuSensor imuSensor;
    private SimulatedBaroAltimeter baroSensor;
    private EkfStateProvider ekfProvider;
    private SensorFailureInjector failureInjector;
    private ThreatAssessment threatAssessment;
    private ReplanningController replanningController;
    private PathFollower pathFollower;
    private UAVPerception perception;

    [SetUp]
    public void SetUp()
    {
        uavObj = new GameObject("UncertaintyNavUAV");
        gpsSensor = uavObj.AddComponent<SimulatedGpsSensor>();
        imuSensor = uavObj.AddComponent<SimulatedImuSensor>();
        baroSensor = uavObj.AddComponent<SimulatedBaroAltimeter>();
        ekfProvider = uavObj.AddComponent<EkfStateProvider>();
        failureInjector = uavObj.AddComponent<SensorFailureInjector>();
        pathFollower = uavObj.AddComponent<PathFollower>();
        perception = uavObj.AddComponent<UAVPerception>();
        threatAssessment = uavObj.AddComponent<ThreatAssessment>();
        replanningController = uavObj.AddComponent<ReplanningController>();

        typeof(SimulatedGpsSensor).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(gpsSensor, null);
        typeof(SimulatedImuSensor).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(imuSensor, null);
        typeof(SimulatedBaroAltimeter).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(baroSensor, null);
        typeof(EkfStateProvider).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(ekfProvider, null);
        typeof(EkfStateProvider).GetMethod("OnEnable", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(ekfProvider, null);
        typeof(SensorFailureInjector).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(failureInjector, null);
        typeof(PathFollower).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(pathFollower, null);
        typeof(UAVPerception).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(perception, null);
        typeof(ThreatAssessment).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(threatAssessment, null);
        typeof(ReplanningController).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(replanningController, null);
    }

    [TearDown]
    public void TearDown()
    {
        if (uavObj != null) Object.DestroyImmediate(uavObj);
    }

    [Test]
    public void UncertaintyNav_CruiseSpeedScalesDown_UnderPositionUncertainty()
    {
        // sigma_horiz = 0.65m (variance = 0.65^2 = 0.4225)
        // V_scale = clamp(1.0 - 0.60 * (0.65 - 0.15), 0.60, 1.0) = 1.0 - 0.60 * 0.50 = 0.70
        float targetSigma = 0.65f;
        float varValue = targetSigma * targetSigma;

        ekfProvider.EkfCore.Initialize(
            Vector3.zero, Vector3.zero, 0f,
            new Vector3(varValue, 0.01f, varValue),
            Vector3.one * 0.01f, 0.01f, 0f);

        ekfProvider.PublishState(0.01f);

        float expectedScale = 1.0f - 0.60f * (targetSigma - 0.15f);
        Assert.AreEqual(expectedScale, replanningController.EffectiveCruiseSpeedScale, 0.01f);
    }

    [Test]
    public void UncertaintyNav_CruiseSpeedRespectsMinimumClamp()
    {
        // Very large uncertainty: sigma_horiz = 2.0m
        float varValue = 4.0f;
        ekfProvider.EkfCore.Initialize(
            Vector3.zero, Vector3.zero, 0f,
            new Vector3(varValue, 0.01f, varValue),
            Vector3.one * 0.01f, 0.01f, 0f);

        ekfProvider.PublishState(0.01f);

        Assert.AreEqual(replanningController.MinCruiseSpeedScale, replanningController.EffectiveCruiseSpeedScale, 0.001f);
    }

    [Test]
    public void UncertaintyNav_EmergencyTtcExpands_UnderPositionUncertainty()
    {
        // sigma_horiz = 0.60m (variance = 0.36)
        // TTC_eff = clamp(4.0 + 2.5 * (0.60 - 0.15), 4.0, 6.0) = 4.0 + 2.5 * 0.45 = 5.125s
        float targetSigma = 0.60f;
        float varValue = targetSigma * targetSigma;

        ekfProvider.EkfCore.Initialize(
            Vector3.zero, Vector3.zero, 0f,
            new Vector3(varValue, 0.01f, varValue),
            Vector3.one * 0.01f, 0.01f, 0f);

        ekfProvider.PublishState(0.01f);

        float expectedTtc = 4.0f + 2.5f * (targetSigma - 0.15f);
        Assert.AreEqual(expectedTtc, replanningController.EffectiveEmergencyTtcThreshold, 0.01f);
    }

    [Test]
    public void UncertaintyNav_EmergencyTtcRespectsMaximumClamp()
    {
        // Large uncertainty: sigma_horiz = 2.5m
        float varValue = 6.25f;
        ekfProvider.EkfCore.Initialize(
            Vector3.zero, Vector3.zero, 0f,
            new Vector3(varValue, 0.01f, varValue),
            Vector3.one * 0.01f, 0.01f, 0f);

        ekfProvider.PublishState(0.01f);

        Assert.AreEqual(replanningController.MaxEmergencyTtcThreshold, replanningController.EffectiveEmergencyTtcThreshold, 0.001f);
    }

    [Test]
    public void UncertaintyNav_AltitudeRecovery_InhibitedDuringElevatedVerticalUncertainty()
    {
        pathFollower.SetTargetAltitude(3.0f);
        replanningController.NominalAltitude = 1.0f;

        // sigma_vert = 0.50m (> 0.35m threshold), EstimatorStatus.Nominal
        float vertVar = 0.50f * 0.50f;
        ekfProvider.EkfCore.Initialize(
            Vector3.zero, Vector3.zero, 0f,
            new Vector3(0.01f, vertVar, 0.01f),
            Vector3.one * 0.01f, 0.01f, 0f);

        ekfProvider.PublishState(0.01f);

        replanningController.RecoverNominalAltitude();

        Assert.AreEqual(3.0f, pathFollower.TargetAltitude, 0.01f, "Altitude recovery must be inhibited when vertical uncertainty is elevated!");
    }

    [Test]
    public void UncertaintyNav_AltitudeRecovery_InhibitedDuringDegradedStatus()
    {
        pathFollower.SetTargetAltitude(3.0f);
        replanningController.NominalAltitude = 1.0f;

        // 1. Initial healthy state
        gpsSensor.UpdateFromSimulationState(Vector3.zero, Vector3.zero, 0.0f);
        baroSensor.UpdateFromSimulationState(0f, 0f, 0.0f);
        imuSensor.UpdateFromKinematics(Vector3.zero, Vector3.zero, Quaternion.identity, 0.0f);

        // 2. Inject GPS failure and advance time past GPS timeout threshold (0.5s) with continuous IMU -> Degraded status
        failureInjector.InjectFailure(SensorType.GPS);
        imuSensor.UpdateFromKinematics(Vector3.zero, Vector3.zero, Quaternion.identity, 0.60f);
        ekfProvider.CheckTimeouts(0.60f);

        Assert.AreEqual(EstimatorStatus.Degraded, ekfProvider.Status);

        replanningController.RecoverNominalAltitude();

        Assert.AreEqual(3.0f, pathFollower.TargetAltitude, 0.01f, "Altitude recovery must be inhibited when EstimatorStatus is Degraded!");
    }

    [Test]
    public void UncertaintyNav_AltitudeRecovery_PermittedWhenNominalAndLowUncertainty()
    {
        pathFollower.SetTargetAltitude(3.0f);
        replanningController.NominalAltitude = 1.0f;

        // Low uncertainties (sigma_horiz = 0.10m, sigma_vert = 0.10m), EstimatorStatus.Nominal
        float varVal = 0.10f * 0.10f;
        ekfProvider.EkfCore.Initialize(
            Vector3.zero, Vector3.zero, 0f,
            new Vector3(varVal, varVal, varVal),
            Vector3.one * 0.01f, 0.01f, 0f);

        ekfProvider.PublishState(0.01f);
        Assert.AreEqual(EstimatorStatus.Nominal, ekfProvider.Status);

        replanningController.RecoverNominalAltitude();

        Assert.AreEqual(1.0f, pathFollower.TargetAltitude, 0.01f, "Altitude recovery must execute when EstimatorStatus is Nominal and vertical uncertainty is low!");
    }

    [Test]
    public void UncertaintyNav_ZeroUncertainty_PreservesExactNominalParameters()
    {
        // Zero / minimal variance
        ekfProvider.EkfCore.Initialize(
            Vector3.zero, Vector3.zero, 0f,
            new Vector3(0.01f, 0.01f, 0.01f),
            Vector3.one * 0.01f, 0.01f, 0f);

        ekfProvider.PublishState(0.01f);

        Assert.AreEqual(1.0f, replanningController.EffectiveCruiseSpeedScale, 0.001f);
        Assert.AreEqual(4.0f, replanningController.EffectiveEmergencyTtcThreshold, 0.001f);
    }

    [Test]
    public void UncertaintyNav_GpsRecovery_RestoresFullCruiseSpeedAndTtc()
    {
        // 1. Initial converged healthy state (25 updates)
        for (int i = 0; i < 25; i++)
        {
            float t = i * 0.05f;
            imuSensor.UpdateFromKinematics(Vector3.zero, Vector3.zero, Quaternion.identity, t);
            gpsSensor.UpdateFromSimulationState(Vector3.zero, Vector3.zero, t);
        }
        Assert.AreEqual(1.0f, replanningController.EffectiveCruiseSpeedScale, 0.05f);

        // 2. Outage causing uncertainty growth
        failureInjector.InjectFailure(SensorType.GPS);
        for (int i = 25; i <= 85; i++)
        {
            imuSensor.UpdateFromKinematics(Vector3.zero, Vector3.zero, Quaternion.identity, i * 0.02f);
        }

        Assert.Less(replanningController.EffectiveCruiseSpeedScale, 1.0f);
        Assert.Greater(replanningController.EffectiveEmergencyTtcThreshold, 4.0f);

        // 3. Recovery
        failureInjector.RecoverSensor(SensorType.GPS);
        for (int i = 86; i <= 115; i++)
        {
            float t = i * 0.05f;
            imuSensor.UpdateFromKinematics(Vector3.zero, Vector3.zero, Quaternion.identity, t);
            gpsSensor.UpdateFromSimulationState(Vector3.zero, Vector3.zero, t);
        }

        Assert.AreEqual(1.0f, replanningController.EffectiveCruiseSpeedScale, 0.05f);
        Assert.AreEqual(4.0f, replanningController.EffectiveEmergencyTtcThreshold, 0.05f);
    }

    [Test]
    public void UncertaintyNav_CombinedSpatialAndKinematicAvoidance_ExecutesConservatively()
    {
        // sigma_horiz = 0.50m (var = 0.25)
        ekfProvider.EkfCore.Initialize(
            Vector3.zero, Vector3.zero, 0f,
            new Vector3(0.25f, 0.01f, 0.25f),
            Vector3.one * 0.01f, 0.01f, 0f);

        ekfProvider.PublishState(0.01f);

        // Spatial envelope from Phase 11.3 expands: R_eff = clamp(1.0 + 2*0.50, 1.0, 2.5) = 2.0m
        Assert.AreEqual(2.0f, threatAssessment.EffectiveSafetyRadius, 0.05f);

        // Kinematic parameters from Phase 11.5 scale down: V_scale = clamp(1.0 - 0.6*(0.50-0.15), 0.6, 1.0) = 0.79
        Assert.AreEqual(0.79f, replanningController.EffectiveCruiseSpeedScale, 0.02f);

        // Temporal horizon expands: TTC_eff = clamp(4.0 + 2.5*(0.50-0.15), 4.0, 6.0) = 4.875s
        Assert.AreEqual(4.875f, replanningController.EffectiveEmergencyTtcThreshold, 0.02f);
    }
}
