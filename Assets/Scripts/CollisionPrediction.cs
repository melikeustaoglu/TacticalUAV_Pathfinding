using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Mathematical representation of a Velocity Obstacle (VO) cone in relative velocity space.
/// Derived from Fiorini & Shiller Velocity Obstacle theory for dynamic moving obstacle avoidance.
/// </summary>
[Serializable]
public struct VelocityObstacle
{
    public Vector3 Apex { get; }              // Obstacle velocity vector (origin of the translated VO cone)
    public Vector3 RelativePosition { get; }  // Position of obstacle relative to UAV: P_obs - P_uav
    public float Distance { get; }            // Magnitude of RelativePosition: ||P_rel||
    public float CombinedRadius { get; }      // Combined safety boundary radius: R_uav + R_obs
    public float HalfAngleDeg { get; }        // Angular half-opening of the collision cone in degrees
    public bool IsValid { get; }

    public VelocityObstacle(
        Vector3 apex,
        Vector3 relativePosition,
        float distance,
        float combinedRadius,
        float halfAngleDeg,
        bool isValid)
    {
        Apex = apex;
        RelativePosition = relativePosition;
        Distance = distance;
        CombinedRadius = combinedRadius;
        HalfAngleDeg = halfAngleDeg;
        IsValid = isValid;
    }

    public static VelocityObstacle Invalid => new VelocityObstacle(
        Vector3.zero, Vector3.zero, 0f, 0f, 0f, false);

    /// <summary>
    /// Evaluates whether a candidate UAV velocity vector lies inside this Velocity Obstacle cone,
    /// indicating an inevitable collision if both agents maintain their constant velocities.
    /// </summary>
    /// <param name="candidateVelocity">UAV velocity vector to test.</param>
    /// <param name="maxLookaheadTime">Optional maximum time horizon for truncated VO (default infinity).</param>
    /// <returns>True if candidate velocity is on an active collision course; false otherwise.</returns>
    public bool ContainsVelocity(Vector3 candidateVelocity, float maxLookaheadTime = float.PositiveInfinity)
    {
        if (!IsValid)
            return false;

        // 1. Degenerate case: UAV is already inside or touching the combined safety radius
        if (Distance <= CombinedRadius)
        {
            Vector3 vRelOverlap = candidateVelocity - Apex;
            float dotOverlap = Vector3.Dot(vRelOverlap, RelativePosition);
            return dotOverlap >= -0.001f;
        }

        // 2. Compute relative velocity: v_rel = v_uav - v_obs
        Vector3 vRel = candidateVelocity - Apex;
        float vRelSq = vRel.sqrMagnitude;

        // Zero relative velocity when separated by distance > combinedRadius cannot cause collision
        if (vRelSq < 1e-6f)
            return false;

        float vRelMag = Mathf.Sqrt(vRelSq);

        // 3. Directional check: Relative velocity must be closing (moving toward the obstacle)
        float dotRel = Vector3.Dot(vRel, RelativePosition);
        if (dotRel <= 0f)
            return false; // Diverging / moving away from obstacle

        // 4. Time horizon / lookahead truncation check
        if (float.IsFinite(maxLookaheadTime) && maxLookaheadTime > 0f)
        {
            float projectedTime = dotRel / vRelSq;
            if (projectedTime > maxLookaheadTime)
                return false; // Collision occurs beyond lookahead window
        }

        // 5. Angular cone check:
        // cos(alpha) = (v_rel · P_rel) / (||v_rel|| * ||P_rel||)
        // Inside VO if cos(alpha) >= cos(halfAngle) = sqrt(1 - (R/d)^2)
        float cosAlpha = dotRel / (vRelMag * Distance);
        float sinHalf = Mathf.Clamp01(CombinedRadius / Distance);
        float cosHalf = Mathf.Sqrt(Mathf.Max(0f, 1f - sinHalf * sinHalf));

        // Use slight epsilon tolerance for exact boundary / tangential velocities
        return cosAlpha >= (cosHalf - 1e-5f);
    }
}

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
    public float VerticalSeparation { get; }

    public CollisionPredictionResult(
        bool willCollide,
        float timeToCollision,
        float distanceToCollision,
        Vector3 estimatedCollisionPoint,
        float crossTrackDistance,
        int obstructedWaypointIndex,
        float verticalSeparation = 0f)
    {
        WillCollide = willCollide;
        TimeToCollision = timeToCollision;
        DistanceToCollision = distanceToCollision;
        EstimatedCollisionPoint = estimatedCollisionPoint;
        CrossTrackDistance = crossTrackDistance;
        ObstructedWaypointIndex = obstructedWaypointIndex;
        VerticalSeparation = verticalSeparation;
    }

    public static CollisionPredictionResult Clear => new CollisionPredictionResult(
        false,
        float.PositiveInfinity,
        float.PositiveInfinity,
        Vector3.zero,
        float.PositiveInfinity,
        -1,
        float.PositiveInfinity);

    public static CollisionPredictionResult NoCollision(float crossTrackDistance, float verticalSeparation = float.PositiveInfinity) => new CollisionPredictionResult(
        false,
        float.PositiveInfinity,
        float.PositiveInfinity,
        Vector3.zero,
        crossTrackDistance,
        -1,
        verticalSeparation);
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
    /// <param name="verticalSafetyMargin">Vertical clearance buffer in meters above obstacle top.</param>
    /// <returns>Prediction result with collision metrics.</returns>
    public static CollisionPredictionResult PredictPathCollision(
        Vector3 currentPosition,
        Vector3 currentVelocity,
        float nominalSpeed,
        IReadOnlyList<Node> remainingWaypoints,
        Vector3 targetWaypoint,
        DetectedObstacle obstacle,
        float safetyRadius,
        float lookaheadTime,
        float verticalSafetyMargin = 0.5f)
    {
        float speed = currentVelocity.magnitude > 0.05f ? currentVelocity.magnitude : Mathf.Max(0.5f, nominalSpeed);
        float maxLookaheadDistance = speed * lookaheadTime;

        Vector3 obstaclePos = obstacle.WorldPosition;
        float obstacleTopY = obstaclePos.y + 0.5f;

        if (obstacle.Collider != null)
        {
            obstacleTopY = obstacle.Collider.bounds.max.y;
        }

        // Build composite trajectory segments from current position through upcoming waypoints
        List<Vector3> trajectoryPoints = new List<Vector3>(16);
        trajectoryPoints.Add(currentPosition);

        if (remainingWaypoints != null && remainingWaypoints.Count > 0)
        {
            for (int i = 0; i < remainingWaypoints.Count; i++)
            {
                float wpY = remainingWaypoints[i].worldPosition.y > 0.001f
                    ? remainingWaypoints[i].worldPosition.y
                    : currentPosition.y;

                Vector3 wp = new Vector3(
                    remainingWaypoints[i].worldPosition.x,
                    wpY,
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
        float minVerticalSeparation = float.MaxValue;
        Vector3 bestCollisionPoint = Vector3.zero;
        float bestDistanceToCollision = float.MaxValue;
        float bestVerticalSeparation = 0f;
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

            Vector3 segDir = segVector / segLength;
            float crossTrackDistance;
            Vector3 closestPointOnSegment;
            float alongPathDistance;
            float verticalSeparation;

            if (obstacle.IsDynamic && obstacle.Velocity.sqrMagnitude > 0.0001f)
            {
                // Dynamic moving obstacle: Time-parameterized Closest Point of Approach (CPA)
                float t0 = cumulativeDistance / speed;
                float t1 = (cumulativeDistance + segLength) / speed;
                Vector3 vUav = segDir * speed;
                Vector3 vRel = vUav - obstacle.Velocity;
                Vector3 r0 = (segStart - vUav * t0) - obstaclePos;

                float vRelSqr = vRel.sqrMagnitude;
                float tCpa = vRelSqr > 0.0001f ? -Vector3.Dot(r0, vRel) / vRelSqr : t0;
                float tEval = Mathf.Clamp(tCpa, t0, t1);

                Vector3 relAtEval = r0 + vRel * tEval;
                crossTrackDistance = relAtEval.magnitude;

                float distOnSeg = speed * (tEval - t0);
                alongPathDistance = cumulativeDistance + distOnSeg;
                closestPointOnSegment = segStart + segDir * distOnSeg;

                Vector3 obsPosAtEval = obstaclePos + obstacle.Velocity * (tEval - t0);
                float dynTopY = obsPosAtEval.y + (obstacle.Collider != null ? obstacle.Collider.bounds.extents.y : 0.5f);
                verticalSeparation = closestPointOnSegment.y - dynTopY;
            }
            else
            {
                // Static obstacle: Geometric orthogonal projection onto line segment
                Vector3 toObstacle = obstaclePos - segStart;
                float projection = Vector3.Dot(toObstacle, segDir);
                float clampedProj = Mathf.Clamp(projection, 0f, segLength);

                closestPointOnSegment = segStart + segDir * clampedProj;
                crossTrackDistance = Vector3.Distance(obstaclePos, closestPointOnSegment);

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

                alongPathDistance = cumulativeDistance + clampedProj;
                verticalSeparation = closestPointOnSegment.y - obstacleTopY;
            }

            if (crossTrackDistance < minCrossTrack)
            {
                minCrossTrack = crossTrackDistance;
                minVerticalSeparation = verticalSeparation;
            }

            // Check if path segment physically breaches the safety envelope
            // A collision occurs ONLY IF horizontal cross-track is within safety radius AND vertical separation is below vertical safety margin
            bool isVerticallySafe = verticalSeparation >= verticalSafetyMargin;

            if (!isVerticallySafe && crossTrackDistance <= safetyRadius && alongPathDistance <= maxLookaheadDistance)
            {
                if (alongPathDistance < bestDistanceToCollision)
                {
                    bestDistanceToCollision = alongPathDistance;
                    bestCollisionPoint = closestPointOnSegment;
                    bestWaypointIndex = i;
                    bestVerticalSeparation = verticalSeparation;
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
                bestWaypointIndex,
                bestVerticalSeparation);
        }

        return CollisionPredictionResult.NoCollision(minCrossTrack, minVerticalSeparation);
    }

    /// <summary>
    /// Computes the geometric Velocity Obstacle (VO) cone generated by a moving obstacle relative to the UAV.
    /// </summary>
    public static VelocityObstacle CalculateVelocityObstacle(
        Vector3 uavPosition,
        Vector3 obstaclePosition,
        Vector3 obstacleVelocity,
        float combinedRadius)
    {
        if (!float.IsFinite(combinedRadius) || combinedRadius <= 0f)
            return VelocityObstacle.Invalid;

        Vector3 relPos = obstaclePosition - uavPosition;
        float dist = relPos.magnitude;

        if (dist < 1e-5f)
        {
            // Zero distance - already overlapping
            return new VelocityObstacle(
                obstacleVelocity,
                relPos,
                0f,
                combinedRadius,
                90f,
                true);
        }

        float sinHalf = Mathf.Clamp01(combinedRadius / dist);
        float halfAngleDeg = Mathf.Asin(sinHalf) * Mathf.Rad2Deg;

        return new VelocityObstacle(
            obstacleVelocity,
            relPos,
            dist,
            combinedRadius,
            halfAngleDeg,
            true);
    }

    /// <summary>
    /// Evaluates whether the UAV's velocity vector lies within the Velocity Obstacle generated by a moving obstacle.
    /// </summary>
    public static bool IsVelocityInsideObstacle(
        Vector3 uavPosition,
        Vector3 uavVelocity,
        Vector3 obstaclePosition,
        Vector3 obstacleVelocity,
        float combinedRadius,
        float maxLookaheadTime = float.PositiveInfinity)
    {
        VelocityObstacle vo = CalculateVelocityObstacle(uavPosition, obstaclePosition, obstacleVelocity, combinedRadius);
        return vo.ContainsVelocity(uavVelocity, maxLookaheadTime);
    }
}
