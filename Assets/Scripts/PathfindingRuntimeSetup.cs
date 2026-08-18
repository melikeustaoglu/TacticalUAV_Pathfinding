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

        ProceduralObstacleGenerator.Generate(
            gridManager.transform,
            gridManager.gridWorldSize,
            effectiveStart,
            effectiveTarget,
            effectiveObstacleCount,
            effectiveSeed,
            effectiveMode,
            effectiveFocusWeight,
            effectiveCorridorWidth);

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
            }
            if (uavPerception != null)
            {
                uavPerception.DetectionRange = scenarioConfig.sensorDetectionRange;
            }
        }

        pathfinding.RegisterAgent(uavObject.transform, pathFollower);
    }
}
