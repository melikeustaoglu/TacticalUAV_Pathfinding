using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(GridManager))]
public class Pathfinding : MonoBehaviour
{
    private GridManager gridManager;
    private LineRenderer pathLineRenderer;

    public List<Node> path = new List<Node>();

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
        FindPath(startTransform.position, targetTransform.position);
        StartMovementAlongPath();
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

    public void FindPath(Vector3 startPos, Vector3 targetPos)
    {
        Node startNode = gridManager.NodeFromWorldPoint(startPos);
        Node targetNode = gridManager.NodeFromWorldPoint(targetPos);

        List<Node> openSet = new List<Node>();
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
            }
        }

        startNode.gCost = 0;
        startNode.hCost = GetDistance(startNode, targetNode);

        while (openSet.Count > 0)
        {
            Node currentNode = openSet[0];
            for (int i = 1; i < openSet.Count; i++)
            {
                if (openSet[i].fCost < currentNode.fCost ||
                    openSet[i].fCost == currentNode.fCost && openSet[i].hCost < currentNode.hCost)
                {
                    currentNode = openSet[i];
                }
            }

            openSet.Remove(currentNode);
            closedSet.Add(currentNode);

            if (currentNode == targetNode)
            {
                RetracePath(startNode, targetNode);
                return;
            }

            foreach (Node neighbor in gridManager.GetNeighbors(currentNode))
            {
                if (!neighbor.isWalkable || closedSet.Contains(neighbor))
                    continue;

                int newCostToNeighbor = currentNode.gCost + GetDistance(currentNode, neighbor);
                if (newCostToNeighbor < neighbor.gCost || !openSet.Contains(neighbor))
                {
                    neighbor.gCost = newCostToNeighbor;
                    neighbor.hCost = GetDistance(neighbor, targetNode);
                    neighbor.parent = currentNode;

                    if (!openSet.Contains(neighbor))
                    {
                        openSet.Add(neighbor);
                    }
                }
            }
        }

        path.Clear();
        ClearPathLineRenderer();
    }

    private void RetracePath(Node startNode, Node endNode)
    {
        List<Node> newPath = new List<Node>();
        Node currentNode = endNode;

        while (currentNode != startNode)
        {
            newPath.Add(currentNode);
            currentNode = currentNode.parent;
        }

        newPath.Reverse();
        path = newPath;
        UpdatePathLineRenderer(path, GetLineStartPosition(startNode.worldPosition));
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
