using UnityEngine;

public static class ProceduralObstacleGenerator
{
    public const string ObstacleLayerName = "Obstacle";
    public const int DefaultObstacleCount = 10;
    public const int DefaultSeed = 42;

    private const float EdgePadding = 2f;
    private const float ClearanceRadius = 4f;

    public static LayerMask GetObstacleMask()
    {
        int obstacleLayer = LayerMask.NameToLayer(ObstacleLayerName);
        if (obstacleLayer < 0)
        {
            Debug.LogError($"Layer '{ObstacleLayerName}' was not found. Please add the '{ObstacleLayerName}' layer in Project Settings > Tags and Layers.");
            return 0;
        }

        return 1 << obstacleLayer;
    }

    public static Transform Generate(
        Transform gridOrigin,
        Vector2 gridWorldSize,
        Vector3 startPosition,
        Vector3 targetPosition,
        int obstacleCount = DefaultObstacleCount,
        int seed = DefaultSeed)
    {
        int obstacleLayer = LayerMask.NameToLayer(ObstacleLayerName);
        if (obstacleLayer < 0)
        {
            Debug.LogError($"Layer '{ObstacleLayerName}' was not found. Obstacles cannot be assigned correctly without this layer.");
            return new GameObject("Obstacles").transform;
        }

        GameObject obstaclesParent = new GameObject("Obstacles");
        Vector3 gridCenter = gridOrigin.position;
        float halfWidth = gridWorldSize.x * 0.5f - EdgePadding;
        float halfDepth = gridWorldSize.y * 0.5f - EdgePadding;

        System.Random rng = new System.Random(seed);
        int spawnedCount = 0;

        // 1. Guaranteed corridor-blocking cluster perpendicular to the direct start-to-target route
        Vector3 directLine = targetPosition - startPosition;
        Vector3 flatDir = new Vector3(directLine.x, 0f, directLine.z);
        float flatDist = flatDir.magnitude;

        if (flatDist > ClearanceRadius * 2f)
        {
            Vector3 forward = flatDir.normalized;
            Vector3 perp = new Vector3(-forward.z, 0f, forward.x);
            Vector3 midPoint = new Vector3(
                (startPosition.x + targetPosition.x) * 0.5f,
                0.5f,
                (startPosition.z + targetPosition.z) * 0.5f
            );

            // Center blocker directly on the straight-line path
            if (spawnedCount < obstacleCount && !IsTooCloseToPoint(midPoint, startPosition, ClearanceRadius) && !IsTooCloseToPoint(midPoint, targetPosition, ClearanceRadius))
            {
                SpawnObstacle(obstaclesParent.transform, midPoint, 2.5f, ++spawnedCount, obstacleLayer);
            }

            // Left flank blocker to widen the detour
            Vector3 leftFlank = midPoint + perp * 2.2f;
            if (spawnedCount < obstacleCount && !IsTooCloseToPoint(leftFlank, startPosition, ClearanceRadius) && !IsTooCloseToPoint(leftFlank, targetPosition, ClearanceRadius))
            {
                SpawnObstacle(obstaclesParent.transform, leftFlank, 2.0f, ++spawnedCount, obstacleLayer);
            }

            // Right flank blocker
            Vector3 rightFlank = midPoint - perp * 2.2f;
            if (spawnedCount < obstacleCount && !IsTooCloseToPoint(rightFlank, startPosition, ClearanceRadius) && !IsTooCloseToPoint(rightFlank, targetPosition, ClearanceRadius))
            {
                SpawnObstacle(obstaclesParent.transform, rightFlank, 2.0f, ++spawnedCount, obstacleLayer);
            }
        }

        // 2. Deterministic procedural scatter for remaining obstacles across the field
        int attemptCount = 0;
        int maxAttempts = obstacleCount * 30;

        while (spawnedCount < obstacleCount && attemptCount < maxAttempts)
        {
            attemptCount++;

            float x = gridCenter.x + (float)(rng.NextDouble() * (halfWidth * 2f) - halfWidth);
            float z = gridCenter.z + (float)(rng.NextDouble() * (halfDepth * 2f) - halfDepth);
            Vector3 candidatePosition = new Vector3(x, 0.5f, z);

            if (IsTooCloseToPoint(candidatePosition, startPosition, ClearanceRadius))
                continue;

            if (IsTooCloseToPoint(candidatePosition, targetPosition, ClearanceRadius))
                continue;

            float size = (float)(1.2 + rng.NextDouble() * 1.3);
            SpawnObstacle(obstaclesParent.transform, candidatePosition, size, ++spawnedCount, obstacleLayer);
        }

        return obstaclesParent.transform;
    }

    private static GameObject SpawnObstacle(Transform parent, Vector3 position, float size, int index, int layer)
    {
        GameObject obstacle = GameObject.CreatePrimitive(PrimitiveType.Cube);
        obstacle.name = $"Obstacle_{index}";
        obstacle.transform.SetParent(parent);
        obstacle.transform.position = position;
        obstacle.transform.localScale = Vector3.one * size;
        obstacle.layer = layer;
        return obstacle;
    }

    private static bool IsTooCloseToPoint(Vector3 candidatePosition, Vector3 protectedPosition, float clearanceRadius)
    {
        Vector3 flatCandidate = new Vector3(candidatePosition.x, 0f, candidatePosition.z);
        Vector3 flatProtected = new Vector3(protectedPosition.x, 0f, protectedPosition.z);
        return Vector3.Distance(flatCandidate, flatProtected) < clearanceRadius;
    }
}
