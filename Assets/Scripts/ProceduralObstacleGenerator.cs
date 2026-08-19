using System.Collections.Generic;
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
        int seed = DefaultSeed,
        ObstacleDistributionMode distributionMode = ObstacleDistributionMode.Uniform,
        float corridorFocusWeight = 0.0f,
        float corridorWidth = 10.0f,
        bool enableDynamicObstacles = false,
        int dynamicObstacleCount = 0,
        float dynamicObstacleSpeed = 1.0f,
        ObstacleMovementMode dynamicMovementMode = ObstacleMovementMode.Patrol,
        PatrolLoopMode dynamicLoopMode = PatrolLoopMode.PingPong,
        bool enableVariableObstacleHeights = false,
        float minObstacleHeight = 1.0f,
        float maxObstacleHeight = 4.0f,
        float defaultObstacleHeight = 2.0f)
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
        List<Vector3> spawnedPositions = new List<Vector3>();

        float effectiveMinHeight = Mathf.Min(minObstacleHeight, maxObstacleHeight);
        float effectiveMaxHeight = Mathf.Max(minObstacleHeight, maxObstacleHeight);
        float effectiveDefaultHeight = Mathf.Clamp(defaultObstacleHeight, effectiveMinHeight, effectiveMaxHeight);

        // 1. Guaranteed corridor-blocking cluster perpendicular to the direct start-to-target route
        Vector3 directLine = targetPosition - startPosition;
        Vector3 flatDir = new Vector3(directLine.x, 0f, directLine.z);
        float flatDist = flatDir.magnitude;

        Vector3 forward = flatDist > 0.001f ? flatDir.normalized : Vector3.forward;
        Vector3 perp = new Vector3(-forward.z, 0f, forward.x);

        if (flatDist > ClearanceRadius * 2f)
        {
            float centerHeight = enableVariableObstacleHeights ? effectiveDefaultHeight : 2.5f;
            float flankHeight = enableVariableObstacleHeights ? effectiveDefaultHeight : 2.0f;

            float centerY = enableVariableObstacleHeights ? (centerHeight * 0.5f) : 0.5f;
            float flankY = enableVariableObstacleHeights ? (flankHeight * 0.5f) : 0.5f;

            Vector3 midPoint = new Vector3(
                (startPosition.x + targetPosition.x) * 0.5f,
                centerY,
                (startPosition.z + targetPosition.z) * 0.5f
            );

            // Center blocker directly on the straight-line path
            if (spawnedCount < obstacleCount && !IsTooCloseToPoint(midPoint, startPosition, ClearanceRadius) && !IsTooCloseToPoint(midPoint, targetPosition, ClearanceRadius))
            {
                Vector3 centerScale = enableVariableObstacleHeights
                    ? new Vector3(2.5f, centerHeight, 2.5f)
                    : Vector3.one * 2.5f;

                SpawnObstacle(obstaclesParent.transform, midPoint, centerScale, ++spawnedCount, obstacleLayer);
                spawnedPositions.Add(midPoint);
            }

            // Left flank blocker to widen the detour
            Vector3 leftFlank = new Vector3(midPoint.x + perp.x * 2.2f, flankY, midPoint.z + perp.z * 2.2f);
            if (spawnedCount < obstacleCount && !IsTooCloseToPoint(leftFlank, startPosition, ClearanceRadius) && !IsTooCloseToPoint(leftFlank, targetPosition, ClearanceRadius))
            {
                Vector3 flankScale = enableVariableObstacleHeights
                    ? new Vector3(2.0f, flankHeight, 2.0f)
                    : Vector3.one * 2.0f;

                SpawnObstacle(obstaclesParent.transform, leftFlank, flankScale, ++spawnedCount, obstacleLayer);
                spawnedPositions.Add(leftFlank);
            }

            // Right flank blocker
            Vector3 rightFlank = new Vector3(midPoint.x - perp.x * 2.2f, flankY, midPoint.z - perp.z * 2.2f);
            if (spawnedCount < obstacleCount && !IsTooCloseToPoint(rightFlank, startPosition, ClearanceRadius) && !IsTooCloseToPoint(rightFlank, targetPosition, ClearanceRadius))
            {
                Vector3 flankScale = enableVariableObstacleHeights
                    ? new Vector3(2.0f, flankHeight, 2.0f)
                    : Vector3.one * 2.0f;

                SpawnObstacle(obstaclesParent.transform, rightFlank, flankScale, ++spawnedCount, obstacleLayer);
                spawnedPositions.Add(rightFlank);
            }
        }

        // 2. Deterministic procedural scatter for remaining obstacles
        int attemptCount = 0;
        int maxAttempts = obstacleCount * 50;
        float minObstacleSpacing = 2.0f;

        while (spawnedCount < obstacleCount && attemptCount < maxAttempts)
        {
            attemptCount++;
            Vector3 candidatePosition;

            bool placeInCorridor = distributionMode == ObstacleDistributionMode.CorridorFocused ||
                                  (distributionMode == ObstacleDistributionMode.Mixed && rng.NextDouble() < corridorFocusWeight);

            if (placeInCorridor && flatDist > ClearanceRadius * 2f)
            {
                // Position along the flight corridor between 15% and 85% of total span
                float t = 0.15f + (float)(rng.NextDouble() * 0.70f);
                float lateralOffset = (float)((rng.NextDouble() * 2.0 - 1.0) * (corridorWidth * 0.5f));

                Vector3 corridorPoint = startPosition + forward * (t * flatDist) + perp * lateralOffset;
                candidatePosition = new Vector3(
                    Mathf.Clamp(corridorPoint.x, gridCenter.x - halfWidth, gridCenter.x + halfWidth),
                    0.5f,
                    Mathf.Clamp(corridorPoint.z, gridCenter.z - halfDepth, gridCenter.z + halfDepth)
                );
            }
            else
            {
                // Uniform random placement across the entire operational arena
                float x = gridCenter.x + (float)(rng.NextDouble() * (halfWidth * 2f) - halfWidth);
                float z = gridCenter.z + (float)(rng.NextDouble() * (halfDepth * 2f) - halfDepth);
                candidatePosition = new Vector3(x, 0.5f, z);
            }

            if (IsTooCloseToPoint(candidatePosition, startPosition, ClearanceRadius))
                continue;

            if (IsTooCloseToPoint(candidatePosition, targetPosition, ClearanceRadius))
                continue;

            // Spacing check to prevent merging obstacles into unpassable solid mass
            bool tooCloseToOther = false;
            for (int i = 0; i < spawnedPositions.Count; i++)
            {
                if (Vector3.Distance(new Vector3(candidatePosition.x, 0f, candidatePosition.z),
                                     new Vector3(spawnedPositions[i].x, 0f, spawnedPositions[i].z)) < minObstacleSpacing)
                {
                    tooCloseToOther = true;
                    break;
                }
            }

            if (tooCloseToOther)
                continue;

            float size = (float)(1.2 + rng.NextDouble() * 1.3);

            if (enableVariableObstacleHeights)
            {
                float height = effectiveMinHeight + (float)(rng.NextDouble() * (effectiveMaxHeight - effectiveMinHeight));
                candidatePosition.y = height * 0.5f; // Ground alignment: bottom plane at Y = 0
                Vector3 scale = new Vector3(size, height, size);
                SpawnObstacle(obstaclesParent.transform, candidatePosition, scale, ++spawnedCount, obstacleLayer);
            }
            else
            {
                candidatePosition.y = 0.5f;
                SpawnObstacle(obstaclesParent.transform, candidatePosition, size, ++spawnedCount, obstacleLayer);
            }

            spawnedPositions.Add(candidatePosition);
        }

        // 3. Attach and configure DynamicObstacle components if dynamic threats are enabled
        if (enableDynamicObstacles && dynamicObstacleCount > 0)
        {
            int dynamicAssigned = 0;
            for (int i = 0; i < obstaclesParent.transform.childCount && dynamicAssigned < dynamicObstacleCount; i++)
            {
                Transform child = obstaclesParent.transform.GetChild(i);
                DynamicObstacle dyn = child.gameObject.AddComponent<DynamicObstacle>();
                dyn.MovementMode = dynamicMovementMode;
                dyn.LoopMode = dynamicLoopMode;
                dyn.Speed = dynamicObstacleSpeed;
                dyn.MovementEnabled = true;

                Vector3 basePos = child.position;
                Vector3 wpA = basePos - perp * 3.5f;
                Vector3 wpB = basePos + perp * 3.5f;
                dyn.SetPatrolWaypoints(wpA, wpB);
                dynamicAssigned++;
            }
        }

        return obstaclesParent.transform;
    }

    private static GameObject SpawnObstacle(Transform parent, Vector3 position, float size, int index, int layer)
    {
        return SpawnObstacle(parent, position, Vector3.one * size, index, layer);
    }

    private static GameObject SpawnObstacle(Transform parent, Vector3 position, Vector3 scale, int index, int layer)
    {
        GameObject obstacle = GameObject.CreatePrimitive(PrimitiveType.Cube);
        obstacle.name = $"Obstacle_{index}";
        obstacle.transform.SetParent(parent);
        obstacle.transform.position = position;
        obstacle.transform.localScale = scale;
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
