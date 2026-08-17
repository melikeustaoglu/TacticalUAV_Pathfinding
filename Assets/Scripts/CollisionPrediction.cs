using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Encapsulates the output of a forward trajectory collision forecast.
/// </summary>
[Serializable]
public struct CollisionPredictionResult
{
    public bool WillCollide { get; }
    public float TimeToCollision { get; }
    public float DistanceToCollision { get; }
    public Vector3 EstimatedCollisionPoint { get; }
    public float CrossTrackDistance { get; }
    public int ObstructedWaypointIndex { get; }

    public CollisionPredictionResult(
        bool willCollide,
        float timeToCollision,
        float distanceToCollision,
        Vector3 estimatedCollisionPoint,
        float crossTrackDistance,
        int obstructedWaypointIndex)
    {
        WillCollide = willCollide;
        TimeToCollision = timeToCollision;
        DistanceToCollision = distanceToCollision;
        EstimatedCollisionPoint = estimatedCollisionPoint;
        CrossTrackDistance = crossTrackDistance;
        ObstructedWaypointIndex = obstructedWaypointIndex;
    }

    public static CollisionPredictionResult Clear => new CollisionPredictionResult(
        false,
        float.PositiveInfinity,
        float.PositiveInfinity,
        Vector3.zero,
        float.PositiveInfinity,
        -1);

    public static CollisionPredictionResult NoCollision(float crossTrackDistance) => new CollisionPredictionResult(
        false,
        float.PositiveInfinity,
        float.PositiveInfinity,
        Vector3.zero,
        crossTrackDistance,
        -1);
}

/// <summary>
/// Mathematical and geometric trajectory collision predictor.
/// Evaluates the UAV's forward motion envelope along active path segments against obstacle bounding volumes.
/// </summary>
public static class CollisionPrediction
{
    /// <summary>
    /// Evaluates whether a detected obstacle will collide with the UAV's active flight trajectory within a lookahead window.
    /// </summary>
    /// <param name="currentPosition">Current UAV position.</param>
    /// <param name="currentVelocity">Current velocity vector.</param>
    /// <param name="nominalSpeed">Nominal flight speed fallback when stationary.</param>
    /// <param name="remainingWaypoints">List of upcoming path nodes.</param>
    /// <param name="targetWaypoint">Active immediate waypoint position.</param>
    /// <param name="obstacle">Detected obstacle data.</param>
    /// <param name="safetyRadius">UAV safety clearance envelope radius.</param>
    /// <param name="lookaheadTime">Forward time window in seconds.</param>
    /// <returns>Prediction result with collision metrics.</returns>
    public static CollisionPredictionResult PredictPathCollision(
        Vector3 currentPosition,
        Vector3 currentVelocity,
        float nominalSpeed,
        IReadOnlyList<Node> remainingWaypoints,
        Vector3 targetWaypoint,
        DetectedObstacle obstacle,
        float safetyRadius,
        float lookaheadTime)
    {
        float speed = currentVelocity.magnitude > 0.05f ? currentVelocity.magnitude : Mathf.Max(0.5f, nominalSpeed);
        float maxLookaheadDistance = speed * lookaheadTime;

        Vector3 obstaclePos = obstacle.WorldPosition;
        Vector3 obstacleCenter = obstacle.Collider != null ? obstacle.Collider.bounds.center : obstaclePos;

        // Build composite trajectory segments from current position through upcoming waypoints
        List<Vector3> trajectoryPoints = new List<Vector3>(16);
        trajectoryPoints.Add(currentPosition);

        if (remainingWaypoints != null && remainingWaypoints.Count > 0)
        {
            for (int i = 0; i < remainingWaypoints.Count; i++)
            {
                Vector3 wp = new Vector3(
                    remainingWaypoints[i].worldPosition.x,
                    currentPosition.y,
                    remainingWaypoints[i].worldPosition.z);

                if (i == 0 && Vector3.Distance(currentPosition, wp) < 0.1f)
                    continue;

                trajectoryPoints.Add(wp);
            }
        }
        else
        {
            trajectoryPoints.Add(targetWaypoint);
        }

        float cumulativeDistance = 0f;
        float minCrossTrack = float.MaxValue;
        Vector3 bestCollisionPoint = Vector3.zero;
        float bestDistanceToCollision = float.MaxValue;
        int bestWaypointIndex = -1;
        bool collisionFound = false;

        for (int i = 0; i < trajectoryPoints.Count - 1; i++)
        {
            Vector3 segStart = trajectoryPoints[i];
            Vector3 segEnd = trajectoryPoints[i + 1];
            Vector3 segVector = segEnd - segStart;
            float segLength = segVector.magnitude;

            if (segLength < 0.001f)
                continue;

            // Check if segment is beyond max lookahead distance
            if (cumulativeDistance > maxLookaheadDistance)
                break;

            // Project obstacle position onto line segment
            Vector3 segDir = segVector / segLength;
            Vector3 toObstacle = obstaclePos - segStart;
            float projection = Vector3.Dot(toObstacle, segDir);
            float clampedProj = Mathf.Clamp(projection, 0f, segLength);

            Vector3 closestPointOnSegment = segStart + segDir * clampedProj;
            float crossTrackDistance = Vector3.Distance(obstaclePos, closestPointOnSegment);

            // If collider is available, also test collider's closest point to path segment
            if (obstacle.Collider != null)
            {
                Vector3 colliderClosest = obstacle.Collider.ClosestPoint(closestPointOnSegment);
                float colliderDistToPath = Vector3.Distance(colliderClosest, closestPointOnSegment);
                if (colliderDistToPath < crossTrackDistance)
                {
                    crossTrackDistance = colliderDistToPath;
                }
            }

            float alongPathDistance = cumulativeDistance + clampedProj;

            if (crossTrackDistance < minCrossTrack)
            {
                minCrossTrack = crossTrackDistance;
            }

            // Check if path segment physically breaches the safety envelope
            if (crossTrackDistance <= safetyRadius && alongPathDistance <= maxLookaheadDistance)
            {
                if (alongPathDistance < bestDistanceToCollision)
                {
                    bestDistanceToCollision = alongPathDistance;
                    bestCollisionPoint = closestPointOnSegment;
                    bestWaypointIndex = i;
                    collisionFound = true;
                }
            }

            cumulativeDistance += segLength;
        }

        if (collisionFound)
        {
            float ttc = bestDistanceToCollision / speed;
            return new CollisionPredictionResult(
                true,
                ttc,
                bestDistanceToCollision,
                bestCollisionPoint,
                minCrossTrack,
                bestWaypointIndex);
        }

        return CollisionPredictionResult.NoCollision(minCrossTrack);
    }
}
