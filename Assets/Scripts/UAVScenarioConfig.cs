using UnityEngine;

public enum ObstacleDistributionMode
{
    Uniform = 0,
    CorridorFocused = 1,
    Mixed = 2
}

/// <summary>
/// Lightweight, optional ScriptableObject profile providing centralized configuration
/// for tactical mission endpoints, scenario obstacles, seed, and UAV operational limits.
/// </summary>
[CreateAssetMenu(fileName = "NewUAVScenarioConfig", menuName = "Tactical UAV/Scenario Config", order = 1)]
public class UAVScenarioConfig : ScriptableObject
{
    [Header("Mission Waypoints")]
    [Tooltip("UAV mission start / spawn position in world coordinates.")]
    public Vector3 startPosition = new Vector3(-10f, 1f, -10f);

    [Tooltip("Mission destination target position in world coordinates.")]
    public Vector3 targetPosition = new Vector3(10f, 1f, 10f);

    [Header("Scenario Obstacles")]
    [Tooltip("Total number of procedural obstacles generated in the operational area.")]
    [Range(1, 50)]
    public int obstacleCount = 10;

    [Tooltip("Deterministic pseudo-random seed for obstacle placement.")]
    public int seed = 42;

    [Header("Obstacle Placement Strategy")]
    [Tooltip("Strategy used to distribute procedural obstacles across the operational theater.")]
    public ObstacleDistributionMode distributionMode = ObstacleDistributionMode.Uniform;

    [Tooltip("Probability [0, 1] of an obstacle spawning focused within the tactical flight corridor rather than uniform scatter.")]
    [Range(0f, 1f)]
    public float corridorFocusWeight = 0.0f;

    [Tooltip("Width of the tactical flight corridor in meters.")]
    [Range(2f, 30f)]
    public float corridorWidth = 10.0f;

    [Header("UAV Flight & Perception Parameters")]
    [Tooltip("Nominal UAV cruise flight speed in meters per second.")]
    [Range(0.5f, 10.0f)]
    public float uavMoveSpeed = 1.5f;

    [Tooltip("Forward-looking perception sensor detection range in meters.")]
    [Range(1.0f, 30.0f)]
    public float sensorDetectionRange = 10.0f;

    [Header("Airspace Clearance Potential Field")]
    [Tooltip("Enable safety-weighted A* pathfinding incorporating obstacle clearance gradient.")]
    public bool enableClearancePenalty = true;

    [Tooltip("Distance threshold in meters below which obstacle proximity penalty applies.")]
    [Range(0.5f, 10.0f)]
    public float clearanceSafetyThreshold = 3.0f;

    [Tooltip("Maximum additive cost penalty at obstacle boundary.")]
    [Range(0, 50)]
    public int maxClearancePenalty = 20;

    [Header("Dynamic Moving Threats")]
    [Tooltip("Enable dynamic moving obstacles in the operational theater.")]
    public bool enableDynamicObstacles = false;

    [Tooltip("Number of dynamic moving obstacles generated in the scenario.")]
    [Range(0, 10)]
    public int dynamicObstacleCount = 0;

    [Tooltip("Movement speed of dynamic obstacles in meters per second.")]
    [Range(0.2f, 5.0f)]
    public float dynamicObstacleSpeed = 1.0f;

    [Tooltip("Movement pattern for dynamic obstacles: Linear or Patrol.")]
    public ObstacleMovementMode dynamicMovementMode = ObstacleMovementMode.Patrol;

    [Tooltip("Patrol loop pattern: PingPong or Loop.")]
    public PatrolLoopMode dynamicLoopMode = PatrolLoopMode.PingPong;

    [Header("3D Airspace & Obstacle Height Configuration")]
    [Tooltip("Enable variable 3D vertical obstacle heights. If false, preserves legacy isotropic cube dimensions.")]
    public bool enableVariableObstacleHeights = false;

    [Tooltip("Minimum procedural obstacle height in meters.")]
    [Range(0.5f, 10.0f)]
    public float minObstacleHeight = 1.0f;

    [Tooltip("Maximum procedural obstacle height in meters.")]
    [Range(1.0f, 20.0f)]
    public float maxObstacleHeight = 4.0f;

    [Tooltip("Default fixed obstacle height when variable height generation is disabled.")]
    [Range(0.5f, 10.0f)]
    public float defaultObstacleHeight = 2.0f;

    [Tooltip("Minimum allowable UAV flight altitude in meters.")]
    [Range(0.5f, 10.0f)]
    public float minFlightAltitude = 1.0f;

    [Tooltip("Maximum allowable UAV flight ceiling altitude in meters.")]
    [Range(1.0f, 25.0f)]
    public float maxFlightAltitude = 6.0f;

    [Tooltip("Nominal cruise flight altitude in meters.")]
    [Range(0.5f, 10.0f)]
    public float nominalFlightAltitude = 1.0f;
}
