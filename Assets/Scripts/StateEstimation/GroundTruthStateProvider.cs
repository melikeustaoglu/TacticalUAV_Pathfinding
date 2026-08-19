using System;
using UnityEngine;

/// <summary>
/// Transitional State Estimation Provider bridging Unity simulation ground truth into EstimatedState.
///
/// ====================================================================================================
/// TRANSITIONAL ARCHITECTURAL ROLE (Phase 11.1):
/// ----------------------------------------------------------------------------------------------------
/// This component provides a transitional implementation of IEstimatedStateProvider by mapping Unity's
/// Transform and Rigidbody states directly into EstimatedState with nominal covariance.
///
/// Autonomy components (ThreatAssessment, ReplanningController, PathFollower) MUST depend strictly
/// on the IEstimatedStateProvider interface, NOT on GroundTruthStateProvider or Transform.
///
/// In Phase 11.2, this transitional provider will be superseded by the hardware-simulated sensor suite
/// (GPSSensor, IMUSensor, BaroAltimeterSensor) and the Extended Kalman Filter (EKF) state estimator.
/// ====================================================================================================
/// </summary>
[DefaultExecutionOrder(-100)]
public class GroundTruthStateProvider : MonoBehaviour, IEstimatedStateProvider
{
    [Header("Transitional Simulation Settings")]
    [Tooltip("Injects synthetic baseline variance for uncertainty propagation testing.")]
    [SerializeField] private bool injectBaselineVariance = false;

    [Tooltip("Synthetic horizontal/vertical position variance (m^2) when variance injection is active.")]
    [SerializeField] private float syntheticPositionVariance = 0.04f; // ~0.2m 1-sigma standard deviation

    private Rigidbody rb;
    private PathFollower pathFollower;
    private EstimatedState currentState = EstimatedState.Uninitialized;

    public EstimatedState CurrentState => currentState;
    public bool IsEstimatorReady => currentState.IsValid;

    public event Action<EstimatedState> OnStateEstimated;

    private void Awake()
    {
        rb = GetComponent<Rigidbody>();
        pathFollower = GetComponent<PathFollower>();
        SampleState();
    }

    private void Update()
    {
        SampleState();
    }

    private void FixedUpdate()
    {
        SampleState();
    }

    /// <summary>
    /// Samples ground-truth simulation kinematics and dispatches an updated EstimatedState.
    /// </summary>
    public void SampleState()
    {
        Vector3 pos = (rb != null && !rb.isKinematic) ? rb.position : transform.position;
        Quaternion rot = (rb != null && !rb.isKinematic) ? rb.rotation : transform.rotation;
        Vector3 vel = (pathFollower != null) ? pathFollower.CurrentVelocity : ((rb != null) ? rb.linearVelocity : Vector3.zero);

        float yaw = rot.eulerAngles.y;
        float pitch = rot.eulerAngles.x;
        if (pitch > 180f) pitch -= 360f;

        Vector3 posVar = injectBaselineVariance ? Vector3.one * syntheticPositionVariance : Vector3.zero;
        Vector3 velVar = injectBaselineVariance ? Vector3.one * (syntheticPositionVariance * 0.25f) : Vector3.zero;

        currentState = new EstimatedState(
            pos,
            vel,
            yaw,
            pitch,
            Vector3.zero,
            0f,
            posVar,
            velVar,
            0f,
            Time.time,
            EstimatorStatus.Nominal,
            GpsFixState.Fix3D);

        OnStateEstimated?.Invoke(currentState);
    }
}
