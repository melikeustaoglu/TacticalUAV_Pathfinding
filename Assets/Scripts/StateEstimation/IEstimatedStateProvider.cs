using System;

/// <summary>
/// Architectural Provider Interface decoupling the Autonomy Layer from Unity ground truth.
/// Downstream subsystems (PathFollower, ThreatAssessment, ReplanningController) consume
/// this contract to query the current estimated state and uncertainty metrics.
/// </summary>
public interface IEstimatedStateProvider
{
    /// <summary>
    /// Gets the most recent estimated state snapshot produced by the onboard state estimator.
    /// </summary>
    EstimatedState CurrentState { get; }

    /// <summary>
    /// Indicates whether the estimator is initialized and publishing valid state estimates.
    /// </summary>
    bool IsEstimatorReady { get; }

    /// <summary>
    /// Reactive event dispatched whenever a new filtered state estimate is computed.
    /// </summary>
    event Action<EstimatedState> OnStateEstimated;
}
