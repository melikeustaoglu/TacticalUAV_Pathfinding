using System;
using UnityEngine;

/// <summary>
/// Authoritative Diagnostic and Evaluation Component comparing Onboard Estimated State against Simulation Ground Truth.
///
/// ====================================================================================================
/// ARCHITECTURAL ROLE:
/// ----------------------------------------------------------------------------------------------------
/// This component is strictly for benchmarking, telemetry, and validation.
/// It observes both Ground Truth (Transform / Rigidbody) and Estimated State (IEstimatedStateProvider)
/// to compute online estimation errors (RMSE, Max Error, Absolute Residuals).
///
/// Under no circumstances does this component feed Ground Truth back into the EKF or Autonomy layers.
/// ====================================================================================================
/// </summary>
public class StateEstimationDiagnostics : MonoBehaviour
{
    [Header("Evaluation Settings")]
    [Tooltip("Warmup time in seconds before accumulating RMSE metrics (allows filter convergence).")]
    [SerializeField] private float warmupDurationSeconds = 1.0f;

    private IEstimatedStateProvider stateProvider;
    private PathFollower pathFollower;
    private Rigidbody rb;

    // Instantaneous Residuals
    private float currentPositionError = 0f;
    private float currentVelocityError = 0f;
    private float currentYawErrorDeg = 0f;

    // Cumulative Statistics (Post-Warmup)
    private float sumSqPosError = 0f;
    private float sumSqVelError = 0f;
    private float sumSqYawError = 0f;
    private float maxPosError = 0f;
    private int sampleCount = 0;
    private float startTime = 0f;

    public float CurrentPositionError => currentPositionError;
    public float CurrentVelocityError => currentVelocityError;
    public float CurrentYawErrorDeg => currentYawErrorDeg;

    public float RmsePosition => sampleCount > 0 ? Mathf.Sqrt(sumSqPosError / sampleCount) : 0f;
    public float RmseVelocity => sampleCount > 0 ? Mathf.Sqrt(sumSqVelError / sampleCount) : 0f;
    public float RmseYawDeg => sampleCount > 0 ? Mathf.Sqrt(sumSqYawError / sampleCount) : 0f;
    public float MaxPositionError => maxPosError;
    public int SampleCount => sampleCount;

    private void Awake()
    {
        stateProvider = GetComponent<IEstimatedStateProvider>();
        pathFollower = GetComponent<PathFollower>();
        rb = GetComponent<Rigidbody>();
        startTime = Time.time;
    }

    private void Update()
    {
        SampleDiagnostics();
    }

    public void SampleDiagnostics()
    {
        if (stateProvider == null)
        {
            stateProvider = GetComponent<IEstimatedStateProvider>();
            if (stateProvider == null) return;
        }

        EstimatedState est = stateProvider.CurrentState;
        if (!est.IsValid) return;

        Vector3 truePos = transform.position;
        Vector3 trueVel = (pathFollower != null)
            ? pathFollower.CurrentVelocity
            : ((rb != null && !rb.isKinematic) ? rb.linearVelocity : Vector3.zero);

        float trueYaw = transform.eulerAngles.y;

        // Instantaneous Errors
        currentPositionError = Vector3.Distance(truePos, est.Position);
        currentVelocityError = Vector3.Distance(trueVel, est.Velocity);
        currentYawErrorDeg = Mathf.Abs(Mathf.DeltaAngle(trueYaw, est.YawDegrees));

        // Accumulate statistics after warmup period
        if (Time.time - startTime >= warmupDurationSeconds)
        {
            sumSqPosError += currentPositionError * currentPositionError;
            sumSqVelError += currentVelocityError * currentVelocityError;
            sumSqYawError += currentYawErrorDeg * currentYawErrorDeg;
            if (currentPositionError > maxPosError) maxPosError = currentPositionError;
            sampleCount++;
        }
    }

    public void ResetMetrics()
    {
        sumSqPosError = 0f;
        sumSqVelError = 0f;
        sumSqYawError = 0f;
        maxPosError = 0f;
        sampleCount = 0;
        startTime = Time.time;
    }
}
