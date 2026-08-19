using UnityEngine;

[DefaultExecutionOrder(-100)]
public class PathfindingRuntimeSetup : MonoBehaviour
{
    [Header("Scenario Configuration (Optional)")]
    [SerializeField] private UAVScenarioConfig scenarioConfig;

    [Header("Fallback / Direct Parameters")]
    public Vector3 startPosition;
    public Vector3 targetPosition;
    public int obstacleCount = ProceduralObstacleGenerator.DefaultObstacleCount;
    public int seed = ProceduralObstacleGenerator.DefaultSeed;

    public UAVScenarioConfig ScenarioConfig
    {
        get => scenarioConfig;
        set => scenarioConfig = value;
    }

    private void Start()
    {
        GridManager gridManager = GetComponent<GridManager>();
        Pathfinding pathfinding = GetComponent<Pathfinding>();
        if (gridManager == null || pathfinding == null)
            return;

        // Apply scenario profile if assigned; otherwise preserve existing fallback parameters
        Vector3 effectiveStart = scenarioConfig != null ? scenarioConfig.startPosition : (startPosition != Vector3.zero ? startPosition : GameManagerBootstrapper.DefaultStartPosition);
        Vector3 effectiveTarget = scenarioConfig != null ? scenarioConfig.targetPosition : (targetPosition != Vector3.zero ? targetPosition : GameManagerBootstrapper.DefaultTargetPosition);
        int effectiveObstacleCount = scenarioConfig != null ? scenarioConfig.obstacleCount : obstacleCount;
        int effectiveSeed = scenarioConfig != null ? scenarioConfig.seed : seed;

        // Update pathfinding markers with effective mission endpoints
        if (pathfinding.startMarkerTransform != null)
            pathfinding.startMarkerTransform.position = effectiveStart;
        if (pathfinding.targetTransform != null)
            pathfinding.targetTransform.position = effectiveTarget;
        pathfinding.agentSpawnPosition = effectiveStart;

        ObstacleDistributionMode effectiveMode = scenarioConfig != null ? scenarioConfig.distributionMode : ObstacleDistributionMode.Uniform;
        float effectiveFocusWeight = scenarioConfig != null ? scenarioConfig.corridorFocusWeight : 0.0f;
        float effectiveCorridorWidth = scenarioConfig != null ? scenarioConfig.corridorWidth : 10.0f;
        bool effectiveEnableDynamic = scenarioConfig != null && scenarioConfig.enableDynamicObstacles;
        int effectiveDynamicCount = scenarioConfig != null ? scenarioConfig.dynamicObstacleCount : 0;
        float effectiveDynamicSpeed = scenarioConfig != null ? scenarioConfig.dynamicObstacleSpeed : 1.0f;
        ObstacleMovementMode effectiveDynamicMode = scenarioConfig != null ? scenarioConfig.dynamicMovementMode : ObstacleMovementMode.Patrol;
        PatrolLoopMode effectiveDynamicLoop = scenarioConfig != null ? scenarioConfig.dynamicLoopMode : PatrolLoopMode.PingPong;
        bool effectiveEnableVariableHeights = scenarioConfig != null && scenarioConfig.enableVariableObstacleHeights;
        float effectiveMinHeight = scenarioConfig != null ? scenarioConfig.minObstacleHeight : 1.0f;
        float effectiveMaxHeight = scenarioConfig != null ? scenarioConfig.maxObstacleHeight : 4.0f;
        float effectiveDefaultHeight = scenarioConfig != null ? scenarioConfig.defaultObstacleHeight : 2.0f;

        ProceduralObstacleGenerator.Generate(
            gridManager.transform,
            gridManager.gridWorldSize,
            effectiveStart,
            effectiveTarget,
            effectiveObstacleCount,
            effectiveSeed,
            effectiveMode,
            effectiveFocusWeight,
            effectiveCorridorWidth,
            effectiveEnableDynamic,
            effectiveDynamicCount,
            effectiveDynamicSpeed,
            effectiveDynamicMode,
            effectiveDynamicLoop,
            effectiveEnableVariableHeights,
            effectiveMinHeight,
            effectiveMaxHeight,
            effectiveDefaultHeight);

        if (scenarioConfig != null)
        {
            gridManager.enableClearancePotentialField = scenarioConfig.enableClearancePenalty;
            gridManager.clearanceSafetyThreshold = scenarioConfig.clearanceSafetyThreshold;
            gridManager.maxClearancePenalty = scenarioConfig.maxClearancePenalty;
        }

        gridManager.CreateGrid();
        SpawnAndRegisterUav(pathfinding);
        pathfinding.FindTestPath();
    }

    private void SpawnAndRegisterUav(Pathfinding pathfinding)
    {
        Vector3 spawnPosition = pathfinding.agentSpawnPosition;
        GameObject uavObject = GameManagerBootstrapper.CreateUav(spawnPosition);
        PathFollower pathFollower = uavObject.GetComponent<PathFollower>();
        UAVPerception uavPerception = uavObject.GetComponent<UAVPerception>();

        if (scenarioConfig != null)
        {
            if (pathFollower != null)
            {
                pathFollower.MoveSpeed = scenarioConfig.uavMoveSpeed;
                pathFollower.MinFlightAltitude = scenarioConfig.minFlightAltitude;
                pathFollower.MaxFlightAltitude = scenarioConfig.maxFlightAltitude;
                pathFollower.SetTargetAltitude(scenarioConfig.nominalFlightAltitude);
            }
            if (uavPerception != null)
            {
                uavPerception.DetectionRange = scenarioConfig.sensorDetectionRange;
            }
        }

        pathfinding.RegisterAgent(uavObject.transform, pathFollower);
    }
}
