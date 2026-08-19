using System;
using UnityEngine;

/// <summary>
/// Simulated 3D Doppler Radar Sensor.
/// Detects targets within range/FOV and measures both 3D position and relative velocity (Doppler),
/// applying independent Gaussian measurement noise and enforcing strict ground-truth boundary isolation.
/// </summary>
public class SimulatedRadarSensor : MonoBehaviour, ITargetSensor
{
    [Header("Radar Physical Parameters")]
    [Tooltip("Maximum radar detection range in meters.")]
    [SerializeField] private float detectionRange = 25.0f;

    [Tooltip("Horizontal forward Field of View in degrees.")]
    [Range(10f, 360f)]
    [SerializeField] private float horizontalFovAngle = 90.0f;

    [Tooltip("Vertical Field of View in degrees.")]
    [Range(5f, 180f)]
    [SerializeField] private float verticalFovAngle = 20.0f;

    [Tooltip("Measurement update frequency in Hz.")]
    [SerializeField] private float updateRateHz = 10.0f;

    [Header("Noise & Accuracy")]
    [Tooltip("Standard deviation of Gaussian position measurement noise in meters.")]
    [SerializeField] private float positionNoiseSigma = 0.35f;

    [Tooltip("Standard deviation of Gaussian velocity measurement noise in m/s.")]
    [SerializeField] private float velocityNoiseSigma = 0.15f;

    [Tooltip("Base detection confidence score [0.0, 1.0].")]
    [Range(0f, 1f)]
    [SerializeField] private float detectionConfidence = 0.90f;

    [Tooltip("Layer mask for detectable target colliders.")]
    [SerializeField] private LayerMask targetMask;

    [Tooltip("Deterministic pseudo-random generator seed.")]
    [SerializeField] private int randomSeed = 101;

    // State & Buffers
    private SensorHealth health = SensorHealth.Healthy;
    private bool isActive = true;
    private float lastSampleTime = -10f;
    private int nextDetectionId = 1;
    private int lastDetectionCount = 0;
    private GaussianNoiseGenerator noiseGen;

    private readonly Collider[] overlapBuffer = new Collider[64];
    private readonly TargetDetection[] internalBuffer = new TargetDetection[64];

    public TargetSensorModality Modality => TargetSensorModality.Radar;
    public SensorHealth Health => health;
    public bool IsActive { get => isActive; set => isActive = value; }
    public float DetectionRange => detectionRange;
    public float FieldOfViewAngle => horizontalFovAngle;
    public float VerticalFovAngle => verticalFovAngle;
    public float UpdateRateHz => updateRateHz;
    public float PositionNoiseSigma => positionNoiseSigma;
    public float VelocityNoiseSigma => velocityNoiseSigma;
    public float DetectionConfidence => detectionConfidence;
    public int LastDetectionCount => lastDetectionCount;
    public LayerMask TargetMask { get => targetMask; set => targetMask = value; }
    public int RandomSeed { get => randomSeed; set { randomSeed = value; noiseGen = new GaussianNoiseGenerator(value); } }

    public event Action<TargetDetection[], int> OnDetectionsUpdated;

    private void Awake()
    {
        InitializeSensor();
    }

    public void InitializeSensor()
    {
        noiseGen = new GaussianNoiseGenerator(randomSeed);
        health = SensorHealth.Healthy;
        lastSampleTime = -10f;
        lastDetectionCount = 0;
    }

    public void SetHealth(SensorHealth newHealth)
    {
        health = newHealth;
        if (health == SensorHealth.Failed || health == SensorHealth.Timeout)
        {
            lastDetectionCount = 0;
        }
    }

    public void ResetSensor()
    {
        InitializeSensor();
        nextDetectionId = 1;
    }

    private void Update()
    {
        Evaluate(Time.time);
    }

    /// <summary>
    /// Evaluates Radar detection scan at the given simulation timestamp.
    /// </summary>
    public bool Evaluate(float simulationTime)
    {
        if (!isActive || health == SensorHealth.Failed || health == SensorHealth.Timeout)
        {
            lastDetectionCount = 0;
            return false;
        }

        float sampleInterval = (updateRateHz > 0f) ? (1f / updateRateHz) : 0.10f;
        if (simulationTime - lastSampleTime < sampleInterval - 0.0001f)
        {
            return false;
        }

        if (noiseGen == null)
        {
            noiseGen = new GaussianNoiseGenerator(randomSeed);
        }

        lastSampleTime = simulationTime;

        Vector3 sensorPos = transform.position;
        Vector3 sensorForward = transform.forward;
        Vector3 sensorUp = transform.up;

        int hitCount = Physics.OverlapSphereNonAlloc(sensorPos, detectionRange, overlapBuffer, targetMask);

        // Sort candidates deterministically by InstanceID to guarantee reproducible ordering
        SortCollidersDeterministic(overlapBuffer, hitCount);

        int detectionCount = 0;

        for (int i = 0; i < hitCount && detectionCount < internalBuffer.Length; i++)
        {
            Collider candidate = overlapBuffer[i];
            if (candidate == null) continue;

            // Ignore self
            if (candidate.transform.root == transform.root) continue;

            Vector3 closestPoint = candidate.ClosestPoint(sensorPos);
            Vector3 toTarget = closestPoint - sensorPos;
            float distance = toTarget.magnitude;

            if (distance > detectionRange || distance < 0.001f) continue;

            Vector3 direction = toTarget / distance;

            // 1. Horizontal FOV check
            Vector3 flatForward = Vector3.ProjectOnPlane(sensorForward, sensorUp).normalized;
            Vector3 flatDir = Vector3.ProjectOnPlane(direction, sensorUp).normalized;
            float horizAngle = Vector3.Angle(flatForward, flatDir);
            if (horizAngle > horizontalFovAngle * 0.5f) continue;

            // 2. Vertical FOV check
            float vertAngle = Mathf.Abs(Mathf.Asin(Mathf.Clamp(Vector3.Dot(direction, sensorUp), -1f, 1f)) * Mathf.Rad2Deg);
            if (vertAngle > verticalFovAngle * 0.5f) continue;

            // 3. Line-of-sight raycast verification
            if (Physics.Raycast(sensorPos, direction, out RaycastHit hit, detectionRange, targetMask))
            {
                if (hit.collider == candidate ||
                    hit.collider.transform.IsChildOf(candidate.transform) ||
                    candidate.transform.IsChildOf(hit.collider.transform))
                {
                    // Obtain true velocity within the sensor simulation layer
                    Vector3 trueVelocity = Vector3.zero;
                    DynamicObstacle dynObs = candidate.GetComponentInParent<DynamicObstacle>();
                    if (dynObs != null && dynObs.MovementEnabled)
                    {
                        trueVelocity = dynObs.CurrentVelocity;
                    }
                    else
                    {
                        Rigidbody rb = candidate.GetComponentInParent<Rigidbody>();
                        if (rb != null && !rb.isKinematic)
                        {
                            trueVelocity = rb.linearVelocity;
                        }
                    }

                    // Apply Gaussian position and velocity noise
                    Vector3 posNoise = noiseGen.SampleVector3(positionNoiseSigma, positionNoiseSigma, positionNoiseSigma);
                    Vector3 velNoise = noiseGen.SampleVector3(velocityNoiseSigma, velocityNoiseSigma, velocityNoiseSigma);

                    Vector3 measuredPos = hit.point + posNoise;
                    Vector3 measuredVel = trueVelocity + velNoise;

                    Vector3 posVariance = Vector3.one * (positionNoiseSigma * positionNoiseSigma);
                    Vector3 velVariance = Vector3.one * (velocityNoiseSigma * velocityNoiseSigma);

                    internalBuffer[detectionCount] = new TargetDetection(
                        TargetSensorModality.Radar,
                        simulationTime,
                        measuredPos,
                        posVariance,
                        detectionConfidence,
                        nextDetectionId++,
                        measuredVel,
                        velVariance,
                        true);

                    detectionCount++;
                }
            }
        }

        lastDetectionCount = detectionCount;
        OnDetectionsUpdated?.Invoke(internalBuffer, lastDetectionCount);
        return true;
    }

    public int TryGetDetections(TargetDetection[] outputBuffer, int offset, int maxCount, float currentTime)
    {
        if (outputBuffer == null || offset < 0 || maxCount <= 0) return 0;

        int countToCopy = Mathf.Min(lastDetectionCount, Mathf.Min(maxCount, outputBuffer.Length - offset));
        for (int i = 0; i < countToCopy; i++)
        {
            outputBuffer[offset + i] = internalBuffer[i];
        }
        return countToCopy;
    }

    private static void SortCollidersDeterministic(Collider[] buffer, int count)
    {
        for (int i = 0; i < count - 1; i++)
        {
            for (int j = i + 1; j < count; j++)
            {
                if (buffer[i] != null && buffer[j] != null)
                {
                    if (buffer[i].GetInstanceID() > buffer[j].GetInstanceID())
                    {
                        Collider temp = buffer[i];
                        buffer[i] = buffer[j];
                        buffer[j] = temp;
                    }
                }
            }
        }
    }
}
