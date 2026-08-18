using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Operational movement mode for dynamic moving obstacles.
/// </summary>
public enum ObstacleMovementMode
{
    Linear,
    Patrol
}

/// <summary>
/// Traversal looping mode when patrolling along predefined waypoints.
/// </summary>
public enum PatrolLoopMode
{
    PingPong,
    Loop
}

/// <summary>
/// Dynamic Obstacle Component.
/// Provides deterministic kinematic movement along linear or patrol waypoint trajectories
/// and exposes instantaneous velocity vectors for predictive threat evaluation and sensor perception.
/// </summary>
public class DynamicObstacle : MonoBehaviour
{
    [Header("Movement Configuration")]
    [Tooltip("Movement behavior mode: Linear or Patrol between waypoints.")]
    [SerializeField] private ObstacleMovementMode movementMode = ObstacleMovementMode.Patrol;

    [Tooltip("Patrol loop behavior: PingPong (reverse at ends) or Loop (wrap to start).")]
    [SerializeField] private PatrolLoopMode loopMode = PatrolLoopMode.PingPong;

    [Tooltip("Movement speed in meters per second.")]
    [SerializeField] private float speed = 1.0f;

    [Tooltip("Whether obstacle movement is currently enabled.")]
    [SerializeField] private bool movementEnabled = true;

    [Header("Linear Movement")]
    [Tooltip("Normalized direction vector for Linear movement mode.")]
    [SerializeField] private Vector3 linearDirection = Vector3.forward;

    [Header("Patrol Waypoints")]
    [Tooltip("Ordered world-space waypoints for Patrol movement mode.")]
    [SerializeField] private List<Vector3> patrolWaypoints = new List<Vector3>();

    [Tooltip("Arrival distance threshold for advancing to the next waypoint.")]
    [SerializeField] private float waypointTolerance = 0.05f;

    // Runtime state
    private int currentWaypointIndex = 0;
    private int patrolDirection = 1; // +1 for forward, -1 for reverse in PingPong
    private Vector3 currentVelocity = Vector3.zero;

    // Public Properties
    public ObstacleMovementMode MovementMode
    {
        get => movementMode;
        set => movementMode = value;
    }

    public PatrolLoopMode LoopMode
    {
        get => loopMode;
        set => loopMode = value;
    }

    public float Speed
    {
        get => speed;
        set => speed = Mathf.Max(0f, value);
    }

    public bool MovementEnabled
    {
        get => movementEnabled;
        set => movementEnabled = value;
    }

    public Vector3 LinearDirection
    {
        get => linearDirection;
        set => linearDirection = value.sqrMagnitude > 0.0001f ? value.normalized : Vector3.zero;
    }

    public IReadOnlyList<Vector3> PatrolWaypoints => patrolWaypoints;
    public Vector3 CurrentVelocity => currentVelocity;
    public bool IsMoving => currentVelocity.sqrMagnitude > 0.0001f;
    public int CurrentWaypointIndex => currentWaypointIndex;
    public int PatrolDirectionSign => patrolDirection;

    private void Awake()
    {
        if (linearDirection.sqrMagnitude > 0.0001f)
        {
            linearDirection = linearDirection.normalized;
        }
    }

    private void Update()
    {
        Step(Time.deltaTime);
    }

    /// <summary>
    /// Configures the patrol waypoint list.
    /// </summary>
    public void SetPatrolWaypoints(IEnumerable<Vector3> waypoints)
    {
        patrolWaypoints.Clear();
        if (waypoints != null)
        {
            patrolWaypoints.AddRange(waypoints);
        }
        currentWaypointIndex = 0;
        patrolDirection = 1;
    }

    /// <summary>
    /// Configures the patrol waypoint list via params array.
    /// </summary>
    public void SetPatrolWaypoints(params Vector3[] waypoints)
    {
        SetPatrolWaypoints((IEnumerable<Vector3>)waypoints);
    }

    /// <summary>
    /// Advances obstacle movement by a deterministic discrete time step (deltaTime).
    /// Updates transform position and calculates the exact instantaneous velocity vector.
    /// </summary>
    /// <param name="deltaTime">Time step in seconds.</param>
    public void Step(float deltaTime)
    {
        if (!movementEnabled || speed <= 0f || deltaTime <= 0f)
        {
            currentVelocity = Vector3.zero;
            return;
        }

        Vector3 startPosition = transform.position;

        if (movementMode == ObstacleMovementMode.Linear)
        {
            ExecuteLinearStep(deltaTime, startPosition);
        }
        else if (movementMode == ObstacleMovementMode.Patrol)
        {
            ExecutePatrolStep(deltaTime, startPosition);
        }
        else
        {
            currentVelocity = Vector3.zero;
        }
    }

    private void ExecuteLinearStep(float deltaTime, Vector3 startPosition)
    {
        if (linearDirection.sqrMagnitude < 0.0001f)
        {
            currentVelocity = Vector3.zero;
            return;
        }

        Vector3 moveStep = linearDirection.normalized * (speed * deltaTime);
        transform.position = startPosition + moveStep;
        currentVelocity = moveStep / deltaTime;
    }

    private void ExecutePatrolStep(float deltaTime, Vector3 startPosition)
    {
        if (patrolWaypoints == null || patrolWaypoints.Count == 0)
        {
            currentVelocity = Vector3.zero;
            return;
        }

        if (patrolWaypoints.Count == 1)
        {
            Vector3 target = patrolWaypoints[0];
            Vector3 toTarget = target - startPosition;
            float dist = toTarget.magnitude;

            if (dist <= waypointTolerance)
            {
                transform.position = target;
                currentVelocity = Vector3.zero;
            }
            else
            {
                float stepDist = Mathf.Min(dist, speed * deltaTime);
                Vector3 moveStep = toTarget.normalized * stepDist;
                transform.position = startPosition + moveStep;
                currentVelocity = moveStep / deltaTime;
            }
            return;
        }

        float remainingBudget = speed * deltaTime;
        Vector3 currentPos = startPosition;

        while (remainingBudget > 0.0001f)
        {
            if (currentWaypointIndex < 0 || currentWaypointIndex >= patrolWaypoints.Count)
            {
                currentWaypointIndex = 0;
            }

            Vector3 targetWaypoint = patrolWaypoints[currentWaypointIndex];
            Vector3 toWaypoint = targetWaypoint - currentPos;
            float distToWaypoint = toWaypoint.magnitude;

            if (distToWaypoint <= remainingBudget)
            {
                // Reach waypoint and consume budget
                currentPos = targetWaypoint;
                remainingBudget -= distToWaypoint;
                AdvanceWaypoint();
            }
            else
            {
                // Move towards current waypoint with remaining budget
                Vector3 stepVector = toWaypoint.normalized * remainingBudget;
                currentPos += stepVector;
                remainingBudget = 0f;
            }
        }

        transform.position = currentPos;
        Vector3 totalDisplacement = currentPos - startPosition;
        currentVelocity = totalDisplacement / deltaTime;
    }

    private void AdvanceWaypoint()
    {
        if (patrolWaypoints.Count <= 1)
            return;

        if (loopMode == PatrolLoopMode.Loop)
        {
            currentWaypointIndex = (currentWaypointIndex + 1) % patrolWaypoints.Count;
        }
        else if (loopMode == PatrolLoopMode.PingPong)
        {
            int nextIndex = currentWaypointIndex + patrolDirection;
            if (nextIndex >= patrolWaypoints.Count)
            {
                patrolDirection = -1;
                currentWaypointIndex = patrolWaypoints.Count - 2;
            }
            else if (nextIndex < 0)
            {
                patrolDirection = 1;
                currentWaypointIndex = 1;
            }
            else
            {
                currentWaypointIndex = nextIndex;
            }
        }
    }

    private void OnDrawGizmosSelected()
    {
        if (patrolWaypoints == null || patrolWaypoints.Count == 0)
            return;

        Gizmos.color = Color.yellow;
        for (int i = 0; i < patrolWaypoints.Count; i++)
        {
            Gizmos.DrawWireSphere(patrolWaypoints[i], 0.3f);
            if (i < patrolWaypoints.Count - 1)
            {
                Gizmos.DrawLine(patrolWaypoints[i], patrolWaypoints[i + 1]);
            }
            else if (loopMode == PatrolLoopMode.Loop && patrolWaypoints.Count > 2)
            {
                Gizmos.DrawLine(patrolWaypoints[i], patrolWaypoints[0]);
            }
        }

        if (Application.isPlaying && currentVelocity.sqrMagnitude > 0.001f)
        {
            Gizmos.color = Color.magenta;
            Gizmos.DrawRay(transform.position, currentVelocity);
        }
    }
}
