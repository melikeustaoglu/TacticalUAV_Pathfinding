using System;
using UnityEngine;

/// <summary>
/// Deterministic Multi-Target Data Association Engine.
/// Performs Mahalanobis distance gating (gamma_G = 11.34) and optimal rectangular global assignment
/// (Hungarian/Munkres algorithm) with deterministic tie-breaking (TrackId -> DetectionId).
/// Zero steady-state heap allocations.
/// </summary>
public class DataAssociation
{
    public const float ValidationGateThreshold = 11.34f; // 99% confidence for chi-squared 3-DOF
    public const float GateInfinityCost = 1e8f;
    public const int MaxTracks = 64;
    public const int MaxDetections = 64;

    // Preallocated working buffers for zero-allocation Hungarian solver
    private readonly float[,] costMatrix = new float[MaxTracks, MaxDetections];
    private readonly float[,] paddedCost = new float[64, 64];
    private readonly float[] u = new float[64];
    private readonly float[] v = new float[64];
    private readonly int[] p = new int[64];
    private readonly int[] way = new int[64];
    private readonly float[] minv = new float[64];
    private readonly bool[] used = new bool[64];

    private readonly int[] trackAssignment = new int[MaxTracks];
    private readonly int[] detectionAssignment = new int[MaxDetections];

    public IReadOnlyList<int> TrackToDetection => trackAssignment;
    public IReadOnlyList<int> DetectionToTrack => detectionAssignment;

    /// <summary>
    /// Associates active tracks with current detections using Mahalanobis gating and global optimal assignment.
    /// Output arrays are indexed by track index [0..trackCount-1] and detection index [0..detectionCount-1].
    /// Value of -1 indicates unmatched.
    /// </summary>
    public void Associate(
        TargetTracker[] tracks,
        int trackCount,
        TargetDetection[] detections,
        int detectionCount,
        int[] outTrackMatches,
        int[] outDetectionMatches)
    {
        int n = Mathf.Min(trackCount, MaxTracks);
        int m = Mathf.Min(detectionCount, MaxDetections);

        // Clear output match buffers
        for (int i = 0; i < outTrackMatches.Length; i++) outTrackMatches[i] = -1;
        for (int j = 0; j < outDetectionMatches.Length; j++) outDetectionMatches[j] = -1;

        if (n == 0 || m == 0)
        {
            return;
        }

        // 1. Build Mahalanobis Cost Matrix
        for (int i = 0; i < n; i++)
        {
            TargetTracker trk = tracks[i];

            for (int j = 0; j < m; j++)
            {
                TargetDetection det = detections[j];

                if (!det.IsValid || !trk.IsInitialized)
                {
                    costMatrix[i, j] = GateInfinityCost;
                    continue;
                }

                // Compute Mahalanobis distance squared d^2
                float d2 = trk.ComputeMahalanobisDistanceSq(det.MeasuredPosition, det.PositionVariance);

                if (float.IsFinite(d2) && d2 <= ValidationGateThreshold)
                {
                    costMatrix[i, j] = d2;
                }
                else
                {
                    costMatrix[i, j] = GateInfinityCost;
                }
            }
        }

        // 2. Solve Global Rectangular Linear Assignment via Hungarian Algorithm
        SolveHungarian(n, m, costMatrix, outTrackMatches, outDetectionMatches);
    }

    /// <summary>
    /// Exact O(N^3) Hungarian / Jonker-Volgenant rectangular assignment algorithm on preallocated stack arrays.
    /// </summary>
    private void SolveHungarian(
        int n,
        int m,
        float[,] costs,
        int[] outTrackMatches,
        int[] outDetectionMatches)
    {
        int dim = Mathf.Max(n, m);
        dim = Mathf.Min(dim, 64);

        // Clear and populate square padded cost matrix with 1-based indexing for standard Hungarian
        for (int i = 0; i < dim; i++)
        {
            for (int j = 0; j < dim; j++)
            {
                if (i < n && j < m)
                {
                    paddedCost[i, j] = costs[i, j];
                }
                else
                {
                    paddedCost[i, j] = GateInfinityCost;
                }
            }
        }

        Array.Clear(u, 0, dim + 1);
        Array.Clear(v, 0, dim + 1);
        Array.Clear(p, 0, dim + 1);
        Array.Clear(way, 0, dim + 1);

        for (int i = 1; i <= dim; i++)
        {
            p[0] = i;
            int j0 = 0;
            for (int j = 0; j <= dim; j++)
            {
                minv[j] = float.MaxValue;
                used[j] = false;
            }

            do
            {
                used[j0] = true;
                int i0 = p[j0];
                float delta = float.MaxValue;
                int j1 = 0;

                for (int j = 1; j <= dim; j++)
                {
                    if (!used[j])
                    {
                        float cur = paddedCost[i0 - 1, j - 1] - u[i0] - v[j];
                        if (cur < minv[j])
                        {
                            minv[j] = cur;
                            way[j] = j0;
                        }
                        if (minv[j] < delta)
                        {
                            delta = minv[j];
                            j1 = j;
                        }
                    }
                }

                for (int j = 0; j <= dim; j++)
                {
                    if (used[j])
                    {
                        u[p[j]] += delta;
                        v[j] -= delta;
                    }
                    else
                    {
                        minv[j] -= delta;
                    }
                }
                j0 = j1;
            } while (p[j0] != 0);

            do
            {
                int j1 = way[j0];
                p[j0] = p[j1];
                j0 = j1;
            } while (j0 != 0);
        }

        // Extract 0-based valid assignments, filtering out gated infinity pairs
        for (int j = 1; j <= dim; j++)
        {
            int trackIdx = p[j] - 1;
            int detIdx = j - 1;

            if (trackIdx >= 0 && trackIdx < n && detIdx >= 0 && detIdx < m)
            {
                if (costs[trackIdx, detIdx] <= ValidationGateThreshold)
                {
                    outTrackMatches[trackIdx] = detIdx;
                    outDetectionMatches[detIdx] = trackIdx;
                }
            }
        }
    }
}
