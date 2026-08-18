using System.Collections.Generic;
using UnityEngine;

public class GridManager : MonoBehaviour
{
    [Header("Grid Settings")]
    public Vector2 gridWorldSize = new Vector2(50, 50);
    public float nodeRadius = 0.5f;
    public LayerMask obstacleMask;

    [Header("Airspace Clearance Potential Field")]
    public bool enableClearancePotentialField = true;
    public float clearanceSafetyThreshold = 3.0f; // in meters
    public int maxClearancePenalty = 20;          // max integer additive penalty

    [HideInInspector]
    public Node[,] grid;

    private float nodeDiameter;
    private int gridSizeX, gridSizeY;

    public int MaxSize => Mathf.Max(1, gridSizeX * gridSizeY);

    void Start()
    {
        if (grid == null)
        {
            CreateGrid();
        }
    }

    void Update()
    {
    }

    public void CreateGrid()
    {
        Physics.SyncTransforms();

        nodeDiameter = nodeRadius * 2f;
        gridSizeX = Mathf.RoundToInt(gridWorldSize.x / nodeDiameter);
        gridSizeY = Mathf.RoundToInt(gridWorldSize.y / nodeDiameter);

        grid = new Node[gridSizeX, gridSizeY];

        Vector3 worldBottomLeft = transform.position - Vector3.right * (gridWorldSize.x / 2f) - Vector3.forward * (gridWorldSize.y / 2f);

        for (int x = 0; x < gridSizeX; x++)
        {
            for (int y = 0; y < gridSizeY; y++)
            {
                Vector3 worldPoint = worldBottomLeft + Vector3.right * (x * nodeDiameter + nodeRadius) + Vector3.forward * (y * nodeDiameter + nodeRadius);
                // Vertical bias proportional to nodeRadius so overlap tests correctly intersect colliders
                Vector3 overlapPoint = worldPoint + Vector3.up * (nodeRadius * 0.5f);
                // CheckSphere with inflation margin ensures grid paths strictly maintain safety clearance envelope
                bool isWalkable = !Physics.CheckSphere(overlapPoint, nodeRadius + 0.6f, obstacleMask);
                grid[x, y] = new Node(isWalkable, worldPoint, x, y);
            }
        }

        if (enableClearancePotentialField)
        {
            CalculateClearancePotentialField(clearanceSafetyThreshold, maxClearancePenalty);
        }
    }

    /// <summary>
    /// Computes the Distance Transform from all unwalkable obstacle cells across the grid
    /// and assigns an airspace clearance penalty to each walkable node using a smooth quadratic falloff.
    /// </summary>
    public void CalculateClearancePotentialField(float safetyThreshold = 3.0f, int maxPenalty = 20)
    {
        if (grid == null || gridSizeX == 0 || gridSizeY == 0)
            return;

        int[,] gridDist = new int[gridSizeX, gridSizeY];
        Queue<Vector2Int> queue = new Queue<Vector2Int>(gridSizeX * gridSizeY);

        for (int x = 0; x < gridSizeX; x++)
        {
            for (int y = 0; y < gridSizeY; y++)
            {
                if (!grid[x, y].isWalkable)
                {
                    gridDist[x, y] = 0;
                    queue.Enqueue(new Vector2Int(x, y));
                }
                else
                {
                    gridDist[x, y] = int.MaxValue;
                }
            }
        }

        // Multi-Source 8-Directional Distance Transform
        while (queue.Count > 0)
        {
            Vector2Int cur = queue.Dequeue();
            int curDist = gridDist[cur.x, cur.y];

            for (int dx = -1; dx <= 1; dx++)
            {
                for (int dy = -1; dy <= 1; dy++)
                {
                    if (dx == 0 && dy == 0)
                        continue;

                    int nx = cur.x + dx;
                    int ny = cur.y + dy;

                    if (nx >= 0 && nx < gridSizeX && ny >= 0 && ny < gridSizeY)
                    {
                        int stepCost = (dx == 0 || dy == 0) ? 10 : 14; // Octile metric scale
                        if (curDist + stepCost < gridDist[nx, ny])
                        {
                            gridDist[nx, ny] = curDist + stepCost;
                            queue.Enqueue(new Vector2Int(nx, ny));
                        }
                    }
                }
            }
        }

        float threshold = Mathf.Max(0.1f, safetyThreshold);

        for (int x = 0; x < gridSizeX; x++)
        {
            for (int y = 0; y < gridSizeY; y++)
            {
                Node node = grid[x, y];
                if (!node.isWalkable)
                {
                    node.clearanceDistance = 0f;
                    node.clearancePenalty = maxPenalty;
                    continue;
                }

                // Convert octile metric distance into physical world meters
                float distMeters = (gridDist[x, y] / 10f) * nodeDiameter;
                node.clearanceDistance = distMeters;

                if (distMeters >= threshold)
                {
                    node.clearancePenalty = 0;
                }
                else
                {
                    // Smooth quadratic falloff: penalty = maxPenalty * (1 - d/threshold)^2
                    float norm = 1.0f - (distMeters / threshold);
                    node.clearancePenalty = Mathf.RoundToInt(maxPenalty * (norm * norm));
                }
            }
        }
    }

    public List<Node> GetNeighbors(Node node)
    {
        List<Node> neighbors = new List<Node>();

        for (int x = -1; x <= 1; x++)
        {
            for (int y = -1; y <= 1; y++)
            {
                if (x == 0 && y == 0)
                    continue;

                int checkX = node.gridX + x;
                int checkY = node.gridY + y;

                if (checkX >= 0 && checkX < gridSizeX && checkY >= 0 && checkY < gridSizeY)
                {
                    neighbors.Add(grid[checkX, checkY]);
                }
            }
        }

        return neighbors;
    }

    public Node NodeFromWorldPoint(Vector3 worldPosition)
    {
        float percentX = (worldPosition.x + gridWorldSize.x / 2f) / gridWorldSize.x;
        float percentY = (worldPosition.z + gridWorldSize.y / 2f) / gridWorldSize.y;

        percentX = Mathf.Clamp01(percentX);
        percentY = Mathf.Clamp01(percentY);

        int x = Mathf.RoundToInt((gridSizeX - 1) * percentX);
        int y = Mathf.RoundToInt((gridSizeY - 1) * percentY);

        return grid[x, y];
    }

    private void OnDrawGizmos()
    {
        nodeDiameter = nodeRadius * 2f;
        Vector3 worldBottomLeft = transform.position - Vector3.right * (gridWorldSize.x / 2f) - Vector3.forward * (gridWorldSize.y / 2f);

        Gizmos.DrawWireCube(transform.position, new Vector3(gridWorldSize.x, 1f, gridWorldSize.y));

        if (grid != null)
        {
            foreach (Node node in grid)
            {
                Gizmos.color = node.isWalkable ? Color.green : Color.red;
                Gizmos.DrawCube(node.worldPosition, Vector3.one * (nodeDiameter - 0.1f));
            }
        }
        else
        {
            for (int x = 0; x < gridSizeX; x++)
            {
                for (int y = 0; y < gridSizeY; y++)
                {
                    Vector3 worldPoint = worldBottomLeft + Vector3.right * (x * nodeDiameter + nodeRadius) + Vector3.forward * (y * nodeDiameter + nodeRadius);
                    Gizmos.color = Color.white;
                    Gizmos.DrawWireCube(worldPoint, Vector3.one * (nodeDiameter - 0.1f));
                }
            }
        }
    }
}
