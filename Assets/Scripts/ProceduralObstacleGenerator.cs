using UnityEngine;

public static class ProceduralObstacleGenerator
{
    public const string ObstacleLayerName = "Obstacle";
    public const int DefaultObstacleCount = 10;

    private const float EdgePadding = 2f;
    private const float ClearanceRadius = 4f;

    public static LayerMask GetObstacleMask()
    {
        int obstacleLayer = LayerMask.NameToLayer(ObstacleLayerName);
        if (obstacleLayer < 0)
        {
            Debug.LogWarning($"Layer '{ObstacleLayerName}' was not found. Falling back to the Default layer mask.");
            return 1 << 0;
        }

        return 1 << obstacleLayer;
    }

    public static Transform Generate(
        Transform gridOrigin,
        Vector2 gridWorldSize,
        Vector3 startPosition,
        Vector3 targetPosition,
        int obstacleCount = DefaultObstacleCount)
    {
        int obstacleLayer = LayerMask.NameToLayer(ObstacleLayerName);
        if (obstacleLayer < 0)
        {
            obstacleLayer = 0;
        }

        GameObject obstaclesParent = new GameObject("Obstacles");
        Vector3 gridCenter = gridOrigin.position;
        float halfWidth = gridWorldSize.x * 0.5f - EdgePadding;
        float halfDepth = gridWorldSize.y * 0.5f - EdgePadding;

        int spawnedCount = 0;
        int attemptCount = 0;
        int maxAttempts = obstacleCount * 20;

        while (spawnedCount < obstacleCount && attemptCount < maxAttempts)
        {
            attemptCount++;

            float x = gridCenter.x + Random.Range(-halfWidth, halfWidth);
            float z = gridCenter.z + Random.Range(-halfDepth, halfDepth);
            Vector3 candidatePosition = new Vector3(x, 0.5f, z);

            if (IsTooCloseToPoint(candidatePosition, startPosition, ClearanceRadius))
                continue;

            if (IsTooCloseToPoint(candidatePosition, targetPosition, ClearanceRadius))
                continue;

            float size = Random.Range(1f, 2.5f);
            GameObject obstacle = GameObject.CreatePrimitive(PrimitiveType.Cube);
            obstacle.name = $"Obstacle_{spawnedCount + 1}";
            obstacle.transform.SetParent(obstaclesParent.transform);
            obstacle.transform.position = candidatePosition;
            obstacle.transform.localScale = Vector3.one * size;
            obstacle.layer = obstacleLayer;

            spawnedCount++;
        }

        return obstaclesParent.transform;
    }

    private static bool IsTooCloseToPoint(Vector3 candidatePosition, Vector3 protectedPosition, float clearanceRadius)
    {
        Vector3 flatCandidate = new Vector3(candidatePosition.x, 0f, candidatePosition.z);
        Vector3 flatProtected = new Vector3(protectedPosition.x, 0f, protectedPosition.z);
        return Vector3.Distance(flatCandidate, flatProtected) < clearanceRadius;
    }
}
