using UnityEngine;

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

    [Header("UAV Flight & Perception Parameters")]
    [Tooltip("Nominal UAV cruise flight speed in meters per second.")]
    [Range(0.5f, 10.0f)]
    public float uavMoveSpeed = 1.5f;

    [Tooltip("Forward-looking perception sensor detection range in meters.")]
    [Range(1.0f, 30.0f)]
    public float sensorDetectionRange = 10.0f;
}
