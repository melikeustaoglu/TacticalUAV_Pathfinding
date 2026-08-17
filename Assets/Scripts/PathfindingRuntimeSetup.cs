using UnityEngine;

[DefaultExecutionOrder(-100)]
public class PathfindingRuntimeSetup : MonoBehaviour
{
    public Vector3 startPosition;
    public Vector3 targetPosition;
    public int obstacleCount = ProceduralObstacleGenerator.DefaultObstacleCount;

    private void Start()
    {
        GridManager gridManager = GetComponent<GridManager>();
        Pathfinding pathfinding = GetComponent<Pathfinding>();
        if (gridManager == null || pathfinding == null)
            return;

        ProceduralObstacleGenerator.Generate(
            gridManager.transform,
            gridManager.gridWorldSize,
            startPosition,
            targetPosition,
            obstacleCount);

        gridManager.CreateGrid();
        SpawnAndRegisterUav(pathfinding);
        pathfinding.FindTestPath();
    }

    private static void SpawnAndRegisterUav(Pathfinding pathfinding)
    {
        Vector3 spawnPosition = pathfinding.agentSpawnPosition;
        GameObject uavObject = GameManagerBootstrapper.CreateUav(spawnPosition);
        PathFollower pathFollower = uavObject.GetComponent<PathFollower>();
        pathfinding.RegisterAgent(uavObject.transform, pathFollower);
    }
}
