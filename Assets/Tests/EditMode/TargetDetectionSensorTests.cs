using System.Reflection;
using NUnit.Framework;
using UnityEngine;

/// <summary>
/// Phase 12.2 Target Detection & Sensor Simulation Tests.
/// Validates immutable detection contracts, LiDAR/Radar sensing, noise models, and ground-truth isolation.
/// </summary>
[TestFixture]
public class TargetDetectionSensorTests
{
    private GameObject uavObj;
    private GameObject targetObj;
    private SimulatedLidarSensor lidarSensor;
    private SimulatedRadarSensor radarSensor;
    private DynamicObstacle dynamicObstacle;

    [SetUp]
    public void SetUp()
    {
        uavObj = new GameObject("SensorUAV");
        uavObj.transform.position = Vector3.zero;
        uavObj.transform.rotation = Quaternion.identity;

        lidarSensor = uavObj.AddComponent<SimulatedLidarSensor>();
        radarSensor = uavObj.AddComponent<SimulatedRadarSensor>();

        targetObj = GameObject.CreatePrimitive(PrimitiveType.Cube);
        targetObj.name = "TestTarget";
        targetObj.transform.position = new Vector3(0f, 0f, 5f);
        dynamicObstacle = targetObj.AddComponent<DynamicObstacle>();

        // Set up layer masks
        int defaultLayer = LayerMask.NameToLayer("Default");
        int mask = 1 << defaultLayer;
        lidarSensor.TargetMask = mask;
        radarSensor.TargetMask = mask;

        typeof(SimulatedLidarSensor).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(lidarSensor, null);
        typeof(SimulatedRadarSensor).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(radarSensor, null);
        typeof(DynamicObstacle).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(dynamicObstacle, null);
    }

    [TearDown]
    public void TearDown()
    {
        if (uavObj != null) Object.DestroyImmediate(uavObj);
        if (targetObj != null) Object.DestroyImmediate(targetObj);
    }

    [Test]
    public void TargetDetection_ConstructorClampsConfidence()
    {
        TargetDetection highConf = new TargetDetection(TargetSensorModality.LiDAR, 0f, Vector3.zero, Vector3.one, 1.5f, 1);
        TargetDetection lowConf = new TargetDetection(TargetSensorModality.LiDAR, 0f, Vector3.zero, Vector3.one, -0.5f, 2);

        Assert.AreEqual(1.0f, highConf.Confidence, 0.001f);
        Assert.AreEqual(0.0f, lowConf.Confidence, 0.001f);
    }

    [Test]
    public void TargetDetection_IsImmutable()
    {
        Vector3 pos = new Vector3(1f, 2f, 3f);
        TargetDetection detection = new TargetDetection(TargetSensorModality.LiDAR, 1.0f, pos, Vector3.one * 0.01f, 0.95f, 10);

        Assert.AreEqual(TargetSensorModality.LiDAR, detection.Modality);
        Assert.AreEqual(1.0f, detection.Timestamp);
        Assert.AreEqual(pos, detection.MeasuredPosition);
        Assert.AreEqual(10, detection.DetectionId);
        Assert.IsTrue(detection.IsValid);
    }

    [Test]
    public void Lidar_DetectsTargetWithinRangeAndFov()
    {
        targetObj.transform.position = new Vector3(0f, 0f, 5f);
        lidarSensor.Evaluate(0.05f);

        TargetDetection[] buffer = new TargetDetection[8];
        int count = lidarSensor.TryGetDetections(buffer, 0, 8, 0.05f);

        Assert.GreaterOrEqual(count, 1, "LiDAR must detect obstacle in front within range and FOV!");
        Assert.AreEqual(TargetSensorModality.LiDAR, buffer[0].Modality);
        Assert.AreEqual(5f, buffer[0].MeasuredPosition.z, 0.5f);
    }

    [Test]
    public void Lidar_RejectsTargetOutsideRange()
    {
        // Target placed at 25m, but LiDAR range is 15m
        targetObj.transform.position = new Vector3(0f, 0f, 25f);
        lidarSensor.Evaluate(0.05f);

        TargetDetection[] buffer = new TargetDetection[8];
        int count = lidarSensor.TryGetDetections(buffer, 0, 8, 0.05f);

        Assert.AreEqual(0, count, "LiDAR must reject targets beyond detection range!");
    }

    [Test]
    public void Lidar_RejectsTargetOutsideFov()
    {
        // Target placed at 90 degrees to right (x = 8, z = 0), outside 120-deg FOV (half-angle 60 deg)
        targetObj.transform.position = new Vector3(8f, 0f, 0f);
        lidarSensor.Evaluate(0.05f);

        TargetDetection[] buffer = new TargetDetection[8];
        int count = lidarSensor.TryGetDetections(buffer, 0, 8, 0.05f);

        Assert.AreEqual(0, count, "LiDAR must reject targets outside FOV cone!");
    }

    [Test]
    public void Lidar_GeneratesPositionVariance()
    {
        targetObj.transform.position = new Vector3(0f, 0f, 4f);
        lidarSensor.Evaluate(0.05f);

        TargetDetection[] buffer = new TargetDetection[8];
        int count = lidarSensor.TryGetDetections(buffer, 0, 8, 0.05f);

        Assert.AreEqual(1, count);
        float expectedVariance = lidarSensor.PositionNoiseSigma * lidarSensor.PositionNoiseSigma;
        Assert.AreEqual(expectedVariance, buffer[0].PositionVariance.x, 0.0001f);
        Assert.AreEqual(expectedVariance, buffer[0].PositionVariance.y, 0.0001f);
        Assert.AreEqual(expectedVariance, buffer[0].PositionVariance.z, 0.0001f);
    }

    [Test]
    public void Lidar_DoesNotProvideVelocity()
    {
        targetObj.transform.position = new Vector3(0f, 0f, 6f);
        lidarSensor.Evaluate(0.05f);

        TargetDetection[] buffer = new TargetDetection[8];
        lidarSensor.TryGetDetections(buffer, 0, 8, 0.05f);

        Assert.IsFalse(buffer[0].HasVelocity, "LiDAR must not provide velocity measurements!");
        Assert.AreEqual(Vector3.zero, buffer[0].MeasuredVelocity);
        Assert.AreEqual(Vector3.zero, buffer[0].VelocityVariance);
    }

    [Test]
    public void Radar_DetectsTargetWithinRangeAndFov()
    {
        targetObj.transform.position = new Vector3(0f, 0f, 10f);
        radarSensor.Evaluate(0.10f);

        TargetDetection[] buffer = new TargetDetection[8];
        int count = radarSensor.TryGetDetections(buffer, 0, 8, 0.10f);

        Assert.GreaterOrEqual(count, 1, "Radar must detect obstacle within range and FOV!");
        Assert.AreEqual(TargetSensorModality.Radar, buffer[0].Modality);
        Assert.AreEqual(10f, buffer[0].MeasuredPosition.z, 1.0f);
    }

    [Test]
    public void Radar_ProvidesVelocityMeasurement()
    {
        targetObj.transform.position = new Vector3(0f, 0f, 8f);

        // Configure dynamic obstacle with moving velocity
        typeof(DynamicObstacle).GetField("currentVelocity", BindingFlags.NonPublic | BindingFlags.Instance)?.SetValue(dynamicObstacle, new Vector3(0f, 0f, 2.0f));

        radarSensor.Evaluate(0.10f);

        TargetDetection[] buffer = new TargetDetection[8];
        int count = radarSensor.TryGetDetections(buffer, 0, 8, 0.10f);

        Assert.AreEqual(1, count);
        Assert.IsTrue(buffer[0].HasVelocity, "Radar must report HasVelocity = true!");
        Assert.AreEqual(2.0f, buffer[0].MeasuredVelocity.z, 0.6f);
    }

    [Test]
    public void Radar_VelocityMeasurementContainsConfiguredNoise()
    {
        targetObj.transform.position = new Vector3(0f, 0f, 8f);
        typeof(DynamicObstacle).GetField("currentVelocity", BindingFlags.NonPublic | BindingFlags.Instance)?.SetValue(dynamicObstacle, new Vector3(0f, 0f, 3.0f));

        radarSensor.Evaluate(0.10f);

        TargetDetection[] buffer = new TargetDetection[8];
        radarSensor.TryGetDetections(buffer, 0, 8, 0.10f);

        float expectedVelVar = radarSensor.VelocityNoiseSigma * radarSensor.VelocityNoiseSigma;
        Assert.AreEqual(expectedVelVar, buffer[0].VelocityVariance.z, 0.0001f);
    }

    [Test]
    public void Sensor_DetectionIdsIncreaseMonotonically()
    {
        targetObj.transform.position = new Vector3(0f, 0f, 5f);

        lidarSensor.Evaluate(0.05f);
        TargetDetection[] buf1 = new TargetDetection[4];
        lidarSensor.TryGetDetections(buf1, 0, 4, 0.05f);
        int id1 = buf1[0].DetectionId;

        lidarSensor.Evaluate(0.11f);
        TargetDetection[] buf2 = new TargetDetection[4];
        lidarSensor.TryGetDetections(buf2, 0, 4, 0.11f);
        int id2 = buf2[0].DetectionId;

        Assert.Greater(id2, id1, "Detection IDs must increase monotonically across scans!");
    }

    [Test]
    public void Sensor_EvaluationIsDeterministicForSameSeedAndTimestamp()
    {
        GameObject uav2 = new GameObject("SensorUAV2");
        SimulatedLidarSensor lidar2 = uav2.AddComponent<SimulatedLidarSensor>();
        lidar2.TargetMask = lidarSensor.TargetMask;
        lidar2.RandomSeed = lidarSensor.RandomSeed;
        typeof(SimulatedLidarSensor).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)?.Invoke(lidar2, null);

        targetObj.transform.position = new Vector3(0f, 0f, 5f);

        lidarSensor.Evaluate(0.05f);
        lidar2.Evaluate(0.05f);

        TargetDetection[] buf1 = new TargetDetection[4];
        TargetDetection[] buf2 = new TargetDetection[4];
        lidarSensor.TryGetDetections(buf1, 0, 4, 0.05f);
        lidar2.TryGetDetections(buf2, 0, 4, 0.05f);

        Assert.AreEqual(buf1[0].MeasuredPosition.x, buf2[0].MeasuredPosition.x, 0.0001f);
        Assert.AreEqual(buf1[0].MeasuredPosition.y, buf2[0].MeasuredPosition.y, 0.0001f);
        Assert.AreEqual(buf1[0].MeasuredPosition.z, buf2[0].MeasuredPosition.z, 0.0001f);

        Object.DestroyImmediate(uav2);
    }

    [Test]
    public void Sensor_DoesNotGenerateDuplicateDetectionsWithinSameUpdateInterval()
    {
        targetObj.transform.position = new Vector3(0f, 0f, 5f);

        bool firstEval = lidarSensor.Evaluate(0.05f);
        bool secondEval = lidarSensor.Evaluate(0.06f); // Only 0.01s later, below (1/20Hz = 0.05s) interval

        Assert.IsTrue(firstEval);
        Assert.IsFalse(secondEval, "Sensor must not evaluate or duplicate detections within same sample interval!");
    }

    [Test]
    public void Lidar_UsesConfiguredDetectionConfidence()
    {
        targetObj.transform.position = new Vector3(0f, 0f, 5f);
        lidarSensor.Evaluate(0.05f);

        TargetDetection[] buffer = new TargetDetection[4];
        lidarSensor.TryGetDetections(buffer, 0, 4, 0.05f);

        Assert.AreEqual(lidarSensor.DetectionConfidence, buffer[0].Confidence, 0.001f);
    }

    [Test]
    public void Radar_UsesConfiguredDetectionConfidence()
    {
        targetObj.transform.position = new Vector3(0f, 0f, 10f);
        radarSensor.Evaluate(0.10f);

        TargetDetection[] buffer = new TargetDetection[4];
        radarSensor.TryGetDetections(buffer, 0, 4, 0.10f);

        Assert.AreEqual(radarSensor.DetectionConfidence, buffer[0].Confidence, 0.001f);
    }

    [Test]
    public void Sensors_DoNotExposeGroundTruthReferences()
    {
        // Reflection audit: verify TargetDetection contains NO reference types
        FieldInfo[] fields = typeof(TargetDetection).GetFields(BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance);
        foreach (var field in fields)
        {
            Assert.IsTrue(field.FieldType.IsValueType, $"Field {field.Name} in TargetDetection must be a value type to guarantee decoupling!");
        }

        PropertyInfo[] properties = typeof(TargetDetection).GetProperties(BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance);
        foreach (var prop in properties)
        {
            Assert.IsTrue(prop.PropertyType.IsValueType, $"Property {prop.Name} in TargetDetection must be a value type to guarantee decoupling!");
        }
    }
}
