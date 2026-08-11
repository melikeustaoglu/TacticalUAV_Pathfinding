using System.Collections.Generic;
using UnityEngine;

public class PathFollower : MonoBehaviour
{
    [SerializeField] private float moveSpeed = 1.5f;
    [SerializeField] private float nodeReachThreshold = 0.1f;
    [SerializeField] private bool useRigidbody;

    private Rigidbody rb;
    private Pathfinding pathfinding;
    private List<Node> currentPath;
    private int pathIndex;
    private bool isFollowing;

    private void Awake()
    {
        rb = GetComponent<Rigidbody>();
        pathfinding = FindFirstObjectByType<Pathfinding>();
    }

    public void StartFollowing(List<Node> path)
    {
        if (path == null || path.Count == 0)
        {
            StopFollowing();
            return;
        }

        currentPath = path;
        pathIndex = 0;
        isFollowing = true;
        UpdateRemainingPathLine();
    }

    public void StopFollowing()
    {
        isFollowing = false;
        currentPath = null;
        pathIndex = 0;
    }

    private void Update()
    {
        if (!isFollowing || useRigidbody && rb != null)
            return;

        MoveAlongPath(transform.position, moveSpeed * Time.deltaTime, position => transform.position = position);
    }

    private void FixedUpdate()
    {
        if (!isFollowing || !useRigidbody || rb == null)
            return;

        MoveAlongPath(rb.position, moveSpeed * Time.fixedDeltaTime, rb.MovePosition);
    }

    private void MoveAlongPath(Vector3 currentPosition, float step, System.Action<Vector3> applyPosition)
    {
        if (currentPath == null || currentPath.Count == 0 || pathIndex >= currentPath.Count)
        {
            StopFollowing();
            return;
        }

        // Keep moving through each waypoint sequentially and only advance after reaching it.
        Vector3 target = GetTargetPosition(currentPath[pathIndex]);

        if (Vector3.Distance(currentPosition, target) <= nodeReachThreshold)
        {
            pathIndex++;
            if (pathIndex >= currentPath.Count)
            {
                StopFollowing();
                return;
            }

            target = GetTargetPosition(currentPath[pathIndex]);
        }

        Vector3 newPosition = Vector3.MoveTowards(currentPosition, target, step);
        applyPosition(newPosition);

        if (Vector3.Distance(newPosition, target) <= nodeReachThreshold)
        {
            pathIndex++;
        }

        if (pathIndex >= currentPath.Count)
        {
            StopFollowing();
            return;
        }

        UpdateRemainingPathLine();
    }

    private void UpdateRemainingPathLine()
    {
        if (pathfinding == null || currentPath == null || currentPath.Count == 0)
            return;

        int remainingNodeCount = currentPath.Count - pathIndex;
        if (remainingNodeCount <= 0)
        {
            pathfinding.ClearPathLineRenderer();
            return;
        }

        List<Node> remainingPath = currentPath.GetRange(pathIndex, remainingNodeCount);
        pathfinding.UpdatePathLineRenderer(remainingPath, transform.position);
    }

    private Vector3 GetTargetPosition(Node node)
    {
        return new Vector3(node.worldPosition.x, transform.position.y, node.worldPosition.z);
    }
}
