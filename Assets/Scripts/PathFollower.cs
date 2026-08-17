using System;
using System.Collections.Generic;
using UnityEngine;

public class PathFollower : MonoBehaviour
{
    [Header("Movement Settings")]
    [SerializeField] private float moveSpeed = 1.5f;
    [SerializeField] private float rotationSpeed = 8.0f;
    [SerializeField] private float nodeReachThreshold = 0.1f;
    [SerializeField] private bool useRigidbody;

    private Rigidbody rb;
    private Pathfinding pathfinding;
    private List<Node> currentPath;
    private int pathIndex;
    private bool isFollowing;
    private Vector3 currentVelocity;
    private Vector3 lastPosition;

    // Runtime Telemetry for Autonomous Subsystems
    public bool IsFollowing => isFollowing;
    public int CurrentWaypointIndex => pathIndex;
    public Vector3 CurrentVelocity => isFollowing ? currentVelocity : Vector3.zero;
    public Vector3 TargetWaypoint => (currentPath != null && pathIndex < currentPath.Count) ? GetTargetPosition(currentPath[pathIndex]) : transform.position;
    public IReadOnlyList<Node> RemainingPath => (currentPath != null && pathIndex < currentPath.Count)
        ? currentPath.GetRange(pathIndex, currentPath.Count - pathIndex)
        : (IReadOnlyList<Node>)Array.Empty<Node>();

    public float MoveSpeed
    {
        get => moveSpeed;
        set => moveSpeed = Mathf.Max(0f, value);
    }

    public float RotationSpeed
    {
        get => rotationSpeed;
        set => rotationSpeed = Mathf.Max(0f, value);
    }

    private void Awake()
    {
        rb = GetComponent<Rigidbody>();
        pathfinding = FindFirstObjectByType<Pathfinding>();
        lastPosition = transform.position;
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
        lastPosition = transform.position;
        currentVelocity = Vector3.zero;
        UpdateRemainingPathLine();
    }

    public void StopFollowing()
    {
        isFollowing = false;
        currentPath = null;
        pathIndex = 0;
        currentVelocity = Vector3.zero;
    }

    private void Update()
    {
        if (!isFollowing || (useRigidbody && rb != null))
        {
            currentVelocity = Vector3.zero;
            lastPosition = transform.position;
            return;
        }

        MoveAlongPath(
            transform.position,
            moveSpeed * Time.deltaTime,
            Time.deltaTime,
            position => transform.position = position,
            rotation => transform.rotation = rotation);
    }

    private void FixedUpdate()
    {
        if (!isFollowing || !useRigidbody || rb == null)
            return;

        MoveAlongPath(
            rb.position,
            moveSpeed * Time.fixedDeltaTime,
            Time.fixedDeltaTime,
            rb.MovePosition,
            rb.MoveRotation);
    }

    private void MoveAlongPath(
        Vector3 currentPosition,
        float step,
        float deltaTime,
        Action<Vector3> applyPosition,
        Action<Quaternion> applyRotation)
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

        // Smooth heading alignment toward the active waypoint target
        Vector3 moveDirection = target - newPosition;
        moveDirection.y = 0f;
        if (moveDirection.sqrMagnitude > 0.0001f)
        {
            Quaternion targetRotation = Quaternion.LookRotation(moveDirection.normalized, Vector3.up);
            Quaternion currentRot = useRigidbody && rb != null ? rb.rotation : transform.rotation;
            Quaternion newRotation = Quaternion.Slerp(currentRot, targetRotation, rotationSpeed * deltaTime);
            applyRotation(newRotation);
        }

        // Compute actual velocity based on physical displacement over time
        Vector3 displacement = newPosition - currentPosition;
        currentVelocity = deltaTime > 0.00001f ? (displacement / deltaTime) : Vector3.zero;
        lastPosition = newPosition;

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
