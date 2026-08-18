using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Encapsulates a spatial hazard footprint for dynamic moving obstacles during pathfinding search and smoothing.
/// Supports time-projected forward motion corridors for moving threats.
/// </summary>
[Serializable]
public struct DynamicHazard
{
    public Vector3 Position { get; }
    public float Radius { get; }
    public Vector3 Velocity { get; }
    public bool IsDynamic { get; }
    public float ProjectedHorizonTime { get; }

    public DynamicHazard(Vector3 position, float radius, Vector3 velocity = default, bool isDynamic = true, float projectedHorizonTime = 0f)
    {
        Position = position;
        Radius = radius;
        Velocity = velocity;
        IsDynamic = isDynamic;
        ProjectedHorizonTime = Mathf.Max(0f, projectedHorizonTime);
    }

    /// <summary>
    /// Calculates the minimum 2D horizontal distance from a test point to this hazard's position or projected motion corridor.
    /// </summary>
    public float DistanceToHazard2D(Vector3 testPoint)
    {
        Vector3 pFlat = new Vector3(testPoint.x, 0f, testPoint.z);
        Vector3 startFlat = new Vector3(Position.x, 0f, Position.z);

        if (!IsDynamic || Velocity.sqrMagnitude < 0.01f || ProjectedHorizonTime < 0.01f)
        {
            return Vector3.Distance(pFlat, startFlat);
        }

        Vector3 endFlat = startFlat + new Vector3(Velocity.x, 0f, Velocity.z) * ProjectedHorizonTime;
        Vector3 seg = endFlat - startFlat;
        float segLenSq = seg.sqrMagnitude;

        if (segLenSq < 1e-4f)
        {
            return Vector3.Distance(pFlat, startFlat);
        }

        float t = Mathf.Clamp01(Vector3.Dot(pFlat - startFlat, seg) / segLenSq);
        Vector3 closestOnSeg = startFlat + seg * t;
        return Vector3.Distance(pFlat, closestOnSeg);
    }
}

[RequireComponent(typeof(GridManager))]
public class Pathfinding : MonoBehaviour
{
    private GridManager gridManager;
    private LineRenderer pathLineRenderer;

    public List<Node> path = new List<Node>();
    public List<Node> rawPath = new List<Node>();

    [Header("Path Smoothing")]
    [SerializeField] private bool enableSmoothing = true;
    [SerializeField] private float smoothingSafetyRadius = 1.2f;

    [Header("Path Visualization")]
    [SerializeField] private Color pathLineColor = Color.cyan;
    [SerializeField] private float pathLineWidth = 0.15f;
    [SerializeField] private float pathLineHeightOffset = 0.15f;

    [Header("Test Setup")]
    public Transform startMarkerTransform;
    public Transform startTransform;
    public Transform targetTransform;
    public Vector3 agentSpawnPosition;

    private PathFollower pathFollower;

    private void Awake()
    {
        gridManager = GetComponent<GridManager>();
        SetupPathLineRenderer();
    }

    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.P))
        {
            FindTestPath();
        }
    }

    public void RegisterAgent(Transform agentTransform, PathFollower follower)
    {
        startTransform = agentTransform;
        pathFollower = follower;
    }

    public void FindTestPath()
    {
        if (startTransform == null || targetTransform == null || gridManager == null || gridManager.grid == null)
            return;

        ResetAgentToSpawnPosition();
        FindInitialMissionPath(startTransform.position, targetTransform.position);
        StartMovementAlongPath();
    }

    /// <summary>
    /// Generates the initial baseline direct mission trajectory along the grid line
    /// prior to in-flight dynamic obstacle discovery.
    /// </summary>
    public void FindInitialMissionPath(Vector3 startPos, Vector3 targetPos)
    {
        Node startNode = gridManager.NodeFromWorldPoint(startPos);
        Node targetNode = gridManager.NodeFromWorldPoint(targetPos);

        List<Node> directPath = new List<Node>();
        int x0 = startNode.gridX;
        int y0 = startNode.gridY;
        int x1 = targetNode.gridX;
        int y1 = targetNode.gridY;

        int dx = Mathf.Abs(x1 - x0);
        int dy = Mathf.Abs(y1 - y0);
        int sx = x0 < x1 ? 1 : -1;
        int sy = y0 < y1 ? 1 : -1;
        int err = dx - dy;

        int cx = x0;
        int cy = y0;

        while (true)
        {
            if (cx >= 0 && cx < gridManager.grid.GetLength(0) && cy >= 0 && cy < gridManager.grid.GetLength(1))
            {
                if (cx != x0 || cy != y0)
                {
                    directPath.Add(gridManager.grid[cx, cy]);
                }
            }

            if (cx == x1 && cy == y1)
                break;

            int e2 = 2 * err;
            if (e2 > -dy)
            {
                err -= dy;
                cx += sx;
            }
            if (e2 < dx)
            {
                err += dx;
                cy += sy;
            }
        }

        path = directPath;
        UpdatePathLineRenderer(path, GetLineStartPosition(startPos));
    }

    private void ResetAgentToSpawnPosition()
    {
        if (startTransform == null)
            return;

        startTransform.position = agentSpawnPosition;
        pathFollower?.StopFollowing();
    }

    private void StartMovementAlongPath()
    {
        if (startTransform == null || path == null || path.Count == 0)
            return;

        if (pathFollower == null)
        {
            pathFollower = startTransform.GetComponent<PathFollower>();
        }

        if (pathFollower != null)
        {
            pathFollower.StartFollowing(path);
        }
    }

    public void RequestPath(Vector3 startPos, Vector3 targetPos)
    {
        FindPath(startPos, targetPos);
    }

    public void FindPath(Vector3 startPos, Vector3 targetPos, Vector3? dynamicHazardPosition = null, float dynamicHazardRadius = 0f)
    {
        if (dynamicHazardPosition.HasValue && dynamicHazardRadius > 0.001f)
        {
            DynamicHazard[] hazards = new DynamicHazard[] { new DynamicHazard(dynamicHazardPosition.Value, dynamicHazardRadius) };
            FindPath(startPos, targetPos, (IReadOnlyList<DynamicHazard>)hazards);
        }
        else
        {
            FindPath(startPos, targetPos, (IReadOnlyList<DynamicHazard>)null);
        }
    }

    public void FindPath(Vector3 startPos, Vector3 targetPos, IReadOnlyList<DynamicHazard> dynamicHazards)
    {
        Node startNode = gridManager.NodeFromWorldPoint(startPos);
        Node targetNode = gridManager.NodeFromWorldPoint(targetPos);

        Heap<Node> openSet = new Heap<Node>(gridManager.MaxSize);
        HashSet<Node> closedSet = new HashSet<Node>();

        openSet.Add(startNode);

        for (int x = 0; x < gridManager.grid.GetLength(0); x++)
        {
            for (int y = 0; y < gridManager.grid.GetLength(1); y++)
            {
                Node node = gridManager.grid[x, y];
                node.gCost = int.MaxValue;
                node.hCost = 0;
                node.parent = null;
                node.HeapIndex = -1;
            }
        }

        startNode.gCost = 0;
        startNode.hCost = GetDistance(startNode, targetNode);

        while (openSet.Count > 0)
        {
            Node currentNode = openSet.RemoveFirst();
            closedSet.Add(currentNode);

            if (currentNode == targetNode)
            {
                RetracePath(startNode, targetNode, dynamicHazards);
                return;
            }

            foreach (Node neighbor in gridManager.GetNeighbors(currentNode))
            {
                if (!neighbor.isWalkable || closedSet.Contains(neighbor))
                    continue;

                // Temporary compound dynamic hazard footprint exclusion (scoped to this replan only)
                if (dynamicHazards != null && dynamicHazards.Count > 0 && neighbor != startNode && neighbor != targetNode)
                {
                    bool isHazardBlocked = false;

                    for (int h = 0; h < dynamicHazards.Count; h++)
                    {
                        DynamicHazard hazard = dynamicHazards[h];
                        if (hazard.Radius > 0.001f)
                        {
                            if (hazard.DistanceToHazard2D(neighbor.worldPosition) < hazard.Radius)
                            {
                                isHazardBlocked = true;
                                break;
                            }
                        }
                    }

                    if (isHazardBlocked)
                        continue; // Skip nodes inside any dynamic threat footprint
                }

                int newCostToNeighbor = currentNode.gCost + GetDistance(currentNode, neighbor) + neighbor.clearancePenalty;
                if (newCostToNeighbor < neighbor.gCost || !openSet.Contains(neighbor))
                {
                    neighbor.gCost = newCostToNeighbor;
                    neighbor.hCost = GetDistance(neighbor, targetNode);
                    neighbor.parent = currentNode;

                    if (!openSet.Contains(neighbor))
                    {
                        openSet.Add(neighbor);
                    }
                    else
                    {
                        openSet.UpdateItem(neighbor);
                    }
                }
            }
        }

        path.Clear();
        ClearPathLineRenderer();
    }

    private void RetracePath(Node startNode, Node endNode, IReadOnlyList<DynamicHazard> dynamicHazards = null)
    {
        List<Node> newPath = new List<Node>();
        Node currentNode = endNode;

        while (currentNode != startNode)
        {
            newPath.Add(currentNode);
            currentNode = currentNode.parent;
        }

        newPath.Add(startNode);
        newPath.Reverse();

        rawPath = new List<Node>(newPath);
        List<Node> smoothed = enableSmoothing ? SmoothPath(newPath, dynamicHazards) : newPath;

        if (smoothed.Count > 1 && Vector3.Distance(startNode.worldPosition, smoothed[0].worldPosition) < 0.2f)
        {
            smoothed.RemoveAt(0);
        }

        path = smoothed;
        UpdatePathLineRenderer(path, GetLineStartPosition(startNode.worldPosition));
    }

    public List<Node> SmoothPath(List<Node> inputPath, Vector3? dynamicHazardPosition, float dynamicHazardRadius = 0f)
    {
        if (dynamicHazardPosition.HasValue && dynamicHazardRadius > 0.001f)
        {
            DynamicHazard[] hazards = new DynamicHazard[] { new DynamicHazard(dynamicHazardPosition.Value, dynamicHazardRadius) };
            return SmoothPath(inputPath, (IReadOnlyList<DynamicHazard>)hazards);
        }
        return SmoothPath(inputPath, (IReadOnlyList<DynamicHazard>)null);
    }

    /// <summary>
    /// Smooths a raw grid-based A* node sequence into a minimal set of direct flight waypoints
    /// by pruning redundant intermediate nodes using physical safety corridor capsule checks and multi-hazard clearance.
    /// </summary>
    public List<Node> SmoothPath(List<Node> inputPath, IReadOnlyList<DynamicHazard> dynamicHazards = null)
    {
        if (inputPath == null || inputPath.Count <= 2)
            return inputPath != null ? new List<Node>(inputPath) : new List<Node>();

        List<Node> smoothed = new List<Node>(inputPath.Count);
        smoothed.Add(inputPath[0]);

        int currentIndex = 0;

        while (currentIndex < inputPath.Count - 1)
        {
            int furthestIndex = currentIndex + 1;

            // Greedily find the furthest waypoint that has a 100% collision-free clearance corridor
            for (int candidateIndex = inputPath.Count - 1; candidateIndex > currentIndex; candidateIndex--)
            {
                if (IsCorridorClear(inputPath[currentIndex].worldPosition, inputPath[candidateIndex].worldPosition, dynamicHazards))
                {
                    furthestIndex = candidateIndex;
                    break;
                }
            }

            smoothed.Add(inputPath[furthestIndex]);
            currentIndex = furthestIndex;
        }

        return smoothed;
    }

    public bool IsCorridorClear(Vector3 start, Vector3 end, Vector3? dynamicHazardPosition, float dynamicHazardRadius = 0f)
    {
        if (dynamicHazardPosition.HasValue && dynamicHazardRadius > 0.001f)
        {
            DynamicHazard[] hazards = new DynamicHazard[] { new DynamicHazard(dynamicHazardPosition.Value, dynamicHazardRadius) };
            return IsCorridorClear(start, end, (IReadOnlyList<DynamicHazard>)hazards);
        }
        return IsCorridorClear(start, end, (IReadOnlyList<DynamicHazard>)null);
    }

    /// <summary>
    /// Verifies whether the direct cylindrical flight corridor between two points is 100% free of obstacles
    /// while strictly maintaining the configured smoothing safety radius and compound dynamic hazard buffers.
    /// </summary>
    public bool IsCorridorClear(Vector3 start, Vector3 end, IReadOnlyList<DynamicHazard> dynamicHazards = null)
    {
        if (gridManager == null)
            return false;

        Vector3 p1 = new Vector3(start.x, 0.5f, start.z);
        Vector3 p2 = new Vector3(end.x, 0.5f, end.z);

        // 1. Physics capsule check against static colliders
        bool hasObstacle = Physics.CheckCapsule(p1, p2, smoothingSafetyRadius, gridManager.obstacleMask);
        if (hasObstacle)
            return false;

        // 2. Compound dynamic hazard envelope check
        if (dynamicHazards != null && dynamicHazards.Count > 0)
        {
            Vector3 seg = p2 - p1;
            float segLen = seg.magnitude;
            if (segLen > 0.001f)
            {
                Vector3 segDir = seg / segLen;

                for (int h = 0; h < dynamicHazards.Count; h++)
                {
                    DynamicHazard hazard = dynamicHazards[h];
                    if (hazard.Radius > 0.001f)
                    {
                        Vector3 hazardFlat = new Vector3(hazard.Position.x, 0.5f, hazard.Position.z);
                        Vector3 toHazard = hazardFlat - p1;
                        float proj = Mathf.Clamp(Vector3.Dot(toHazard, segDir), 0f, segLen);
                        Vector3 closestPoint = p1 + segDir * proj;
                        float distToSegment = hazard.DistanceToHazard2D(closestPoint);

                        if (distToSegment < (hazard.Radius + smoothingSafetyRadius))
                        {
                            return false; // Corridor passes too close to dynamic hazard footprint
                        }
                    }
                }
            }
        }

        return true;
    }

    public void UpdatePathLineRenderer(List<Node> nodes, Vector3 lineStartPosition)
    {
        if (pathLineRenderer == null)
            return;

        if (nodes == null || nodes.Count == 0)
        {
            ClearPathLineRenderer();
            return;
        }

        pathLineRenderer.enabled = true;
        pathLineRenderer.positionCount = nodes.Count + 1;
        pathLineRenderer.SetPosition(0, ElevateLinePosition(lineStartPosition));

        for (int i = 0; i < nodes.Count; i++)
        {
            pathLineRenderer.SetPosition(i + 1, ElevateLinePosition(nodes[i].worldPosition));
        }
    }

    public void ClearPathLineRenderer()
    {
        if (pathLineRenderer == null)
            return;

        pathLineRenderer.enabled = false;
        pathLineRenderer.positionCount = 0;
    }

    private void SetupPathLineRenderer()
    {
        GameObject lineObject = new GameObject("PathLineRenderer");
        lineObject.transform.SetParent(transform, false);

        pathLineRenderer = lineObject.AddComponent<LineRenderer>();
        pathLineRenderer.useWorldSpace = true;
        pathLineRenderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        pathLineRenderer.receiveShadows = false;
        pathLineRenderer.material = new Material(Shader.Find("Sprites/Default"));
        pathLineRenderer.startColor = pathLineColor;
        pathLineRenderer.endColor = pathLineColor;
        pathLineRenderer.startWidth = pathLineWidth;
        pathLineRenderer.endWidth = pathLineWidth;
        pathLineRenderer.positionCount = 0;
        pathLineRenderer.enabled = false;
    }

    private Vector3 GetLineStartPosition(Vector3 fallbackPosition)
    {
        if (startTransform != null)
            return startTransform.position;

        return fallbackPosition;
    }

    private Vector3 ElevateLinePosition(Vector3 worldPosition)
    {
        return new Vector3(worldPosition.x, worldPosition.y + pathLineHeightOffset, worldPosition.z);
    }

    private int GetDistance(Node nodeA, Node nodeB)
    {
        int distX = Mathf.Abs(nodeA.gridX - nodeB.gridX);
        int distY = Mathf.Abs(nodeA.gridY - nodeB.gridY);

        if (distX > distY)
            return 14 * distY + 10 * (distX - distY);

        return 14 * distX + 10 * (distY - distX);
    }

    private void OnDrawGizmos()
    {
        DrawTestSetupGizmos();
        DrawPathGizmos();
    }

    private void DrawTestSetupGizmos()
    {
        if (startMarkerTransform != null)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawWireSphere(startMarkerTransform.position, 0.5f);
        }
        else if (startTransform != null)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawWireSphere(startTransform.position, 0.5f);
        }

        if (targetTransform != null)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawWireSphere(targetTransform.position, 0.5f);
        }
    }

    private void DrawPathGizmos()
    {
        // 1. Draw Raw A* Grid Path (faint gray wire cubes) if smoothing is active
        if (enableSmoothing && rawPath != null && rawPath.Count > 0)
        {
            Gizmos.color = new Color(0.6f, 0.6f, 0.6f, 0.35f);
            for (int i = 0; i < rawPath.Count; i++)
            {
                Gizmos.DrawWireCube(rawPath[i].worldPosition, Vector3.one * 0.3f);
                if (i > 0)
                {
                    Gizmos.DrawLine(rawPath[i - 1].worldPosition, rawPath[i].worldPosition);
                }
            }
        }

        // 2. Draw Active Smoothed Path (Cyan solid cubes + flight segments)
        if (path == null || path.Count == 0)
            return;

        Gizmos.color = Color.cyan;

        for (int i = 0; i < path.Count; i++)
        {
            Vector3 nodePosition = path[i].worldPosition;
            Gizmos.DrawCube(nodePosition, Vector3.one * (gridManager != null ? gridManager.nodeRadius * 1.5f : 0.5f));

            if (i > 0)
            {
                Vector3 previousPosition = path[i - 1].worldPosition;
                Gizmos.DrawLine(previousPosition, nodePosition);
            }
        }
    }
}
