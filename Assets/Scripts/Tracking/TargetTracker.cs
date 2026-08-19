using System;
using UnityEngine;

/// <summary>
/// 6-State Linear Kalman Filter for Individual Target Tracking.
/// Estimates 3D position and 3D velocity x = [px, py, pz, vx, vy, vz]^T using a constant-velocity
/// kinematic model with continuous white-acceleration process noise (q = 0.50 m^2/s^3) and
/// numerically stabilized Joseph-form covariance updates.
/// Operates strictly on TargetDetection measurements with zero ground-truth dependencies.
/// </summary>
public class TargetTracker
{
    private Vector3 estimatedPosition;
    private Vector3 estimatedVelocity;
    private Matrix6x6 covariance;

    private float lastUpdateTime = -1f;
    private bool isInitialized = false;
    private int updateCount = 0;
    private int predictCount = 0;

    // Process noise spectral density q = 0.50 m^2/s^3
    private const float DefaultProcessNoiseQ = 0.50f;
    private float processNoiseQ = DefaultProcessNoiseQ;

    public bool IsInitialized => isInitialized;
    public Vector3 EstimatedPosition => estimatedPosition;
    public Vector3 EstimatedVelocity => estimatedVelocity;
    public float Timestamp => lastUpdateTime;
    public float LastUpdateTime => lastUpdateTime;
    public int UpdateCount => updateCount;
    public int PredictCount => predictCount;
    public float ProcessNoiseQ { get => processNoiseQ; set => processNoiseQ = Mathf.Max(0.001f, value); }

    public Vector3 PositionVariance => new Vector3(
        Mathf.Max(0.0001f, covariance[0, 0]),
        Mathf.Max(0.0001f, covariance[1, 1]),
        Mathf.Max(0.0001f, covariance[2, 2]));

    public Vector3 VelocityVariance => new Vector3(
        Mathf.Max(0.0001f, covariance[3, 3]),
        Mathf.Max(0.0001f, covariance[4, 4]),
        Mathf.Max(0.0001f, covariance[5, 5]));

    public float HorizontalPositionStdDev => Mathf.Sqrt(Mathf.Max(PositionVariance.x, PositionVariance.z));
    public float VerticalPositionStdDev => Mathf.Sqrt(PositionVariance.y);
    public float HorizontalVelocityStdDev => Mathf.Sqrt(Mathf.Max(VelocityVariance.x, VelocityVariance.z));
    public float Speed => estimatedVelocity.magnitude;

    public TargetTracker()
    {
        Reset();
    }

    public void Reset()
    {
        estimatedPosition = Vector3.zero;
        estimatedVelocity = Vector3.zero;
        covariance = Matrix6x6.Identity;
        for (int i = 0; i < 6; i++) covariance[i, i] = 9999f;
        lastUpdateTime = -1f;
        isInitialized = false;
        updateCount = 0;
        predictCount = 0;
    }

    /// <summary>
    /// Initializes the tracker state and covariance from an initial target detection.
    /// </summary>
    public bool Initialize(TargetDetection detection)
    {
        if (!detection.IsValid) return false;

        estimatedPosition = detection.MeasuredPosition;
        estimatedVelocity = detection.HasVelocity ? detection.MeasuredVelocity : Vector3.zero;

        covariance = Matrix6x6.Zero;
        covariance[0, 0] = Mathf.Max(0.001f, detection.PositionVariance.x);
        covariance[1, 1] = Mathf.Max(0.001f, detection.PositionVariance.y);
        covariance[2, 2] = Mathf.Max(0.001f, detection.PositionVariance.z);

        if (detection.HasVelocity)
        {
            covariance[3, 3] = Mathf.Max(0.001f, detection.VelocityVariance.x);
            covariance[4, 4] = Mathf.Max(0.001f, detection.VelocityVariance.y);
            covariance[5, 5] = Mathf.Max(0.001f, detection.VelocityVariance.z);
        }
        else
        {
            // Default 4.0 m^2/s^2 uncertainty for unmeasured initial velocity
            covariance[3, 3] = 4.0f;
            covariance[4, 4] = 4.0f;
            covariance[5, 5] = 4.0f;
        }

        lastUpdateTime = detection.Timestamp;
        isInitialized = true;
        updateCount = 1;
        predictCount = 0;
        return true;
    }

    /// <summary>
    /// Time-propagation step advancing state and covariance to target timestamp using constant-velocity model.
    /// </summary>
    public bool Predict(float targetTimestamp)
    {
        if (!isInitialized) return false;
        if (!float.IsFinite(targetTimestamp) || targetTimestamp < lastUpdateTime - 0.0001f)
        {
            return false;
        }

        float dt = targetTimestamp - lastUpdateTime;
        if (dt <= 0.00001f)
        {
            return true; // Zero delta-t, no propagation needed
        }

        // Clamp anomalous large time jumps for stability
        if (dt > 2.0f) dt = 2.0f;

        // 1. State vector propagation: p_k = p_{k-1} + v_{k-1} * dt, v_k = v_{k-1}
        estimatedPosition += estimatedVelocity * dt;

        // 2. State transition Jacobian F(dt)
        Matrix6x6 F = Matrix6x6.Identity;
        F[0, 3] = dt;
        F[1, 4] = dt;
        F[2, 5] = dt;

        // 3. Continuous white acceleration process noise Q(dt)
        float q = processNoiseQ;
        float dt2 = dt * dt;
        float dt3 = dt2 * dt;

        float qPos = (dt3 / 3.0f) * q;
        float qCross = (dt2 / 2.0f) * q;
        float qVel = dt * q;

        Matrix6x6 Q = Matrix6x6.Zero;
        Q[0, 0] = qPos; Q[1, 1] = qPos; Q[2, 2] = qPos;
        Q[3, 3] = qVel; Q[4, 4] = qVel; Q[5, 5] = qVel;
        Q[0, 3] = qCross; Q[1, 4] = qCross; Q[2, 5] = qCross;
        Q[3, 0] = qCross; Q[4, 1] = qCross; Q[5, 2] = qCross;

        // 4. Covariance propagation: P = F P F^T + Q
        covariance = Matrix6x6.Multiply(Matrix6x6.Multiply(F, covariance), Matrix6x6.Transpose(F)) + Q;

        SymmetrizeAndClampCovariance();
        lastUpdateTime = targetTimestamp;
        predictCount++;
        return true;
    }

    /// <summary>
    /// Measurement correction step incorporating a new TargetDetection.
    /// Handles both position-only (LiDAR) and position+velocity (Radar) modalities.
    /// </summary>
    public bool Update(TargetDetection detection)
    {
        if (!detection.IsValid) return false;

        if (!isInitialized)
        {
            return Initialize(detection);
        }

        // Advance filter time if measurement timestamp is newer
        if (detection.Timestamp > lastUpdateTime)
        {
            Predict(detection.Timestamp);
        }

        if (detection.HasVelocity)
        {
            UpdatePositionAndVelocity(detection);
        }
        else
        {
            UpdatePositionOnly(detection);
        }

        lastUpdateTime = detection.Timestamp;
        updateCount++;
        return true;
    }

    private void UpdatePositionOnly(TargetDetection detection)
    {
        // Measurement z (3x1)
        Vector3 z = detection.MeasuredPosition;
        Vector3 hx = estimatedPosition;
        Vector3 y = z - hx; // Innovation (3x1)

        // Measurement noise R (3x3)
        Matrix3x3 R = Matrix3x3.Zero;
        R[0, 0] = Mathf.Max(0.0001f, detection.PositionVariance.x);
        R[1, 1] = Mathf.Max(0.0001f, detection.PositionVariance.y);
        R[2, 2] = Mathf.Max(0.0001f, detection.PositionVariance.z);

        // H is [I_3x3, 0_3x3] (3x6)
        // Innovation covariance S = H P H^T + R = P_pp + R (3x3)
        Matrix3x3 P_pp = covariance.GetSubMatrix3x3(0, 0);
        Matrix3x3 S = P_pp + R;

        if (!S.TryInvert(out Matrix3x3 S_inv))
        {
            return; // Inversion failed, skip update
        }

        // Kalman gain K = P H^T S^-1 = [P_pp; P_vp] * S^-1 (6x3)
        Matrix3x3 P_vp = covariance.GetSubMatrix3x3(3, 0);
        Matrix3x3 K_p = Matrix3x3.Multiply(P_pp, S_inv);
        Matrix3x3 K_v = Matrix3x3.Multiply(P_vp, S_inv);

        // State update
        estimatedPosition += K_p.MultiplyVector(y);
        estimatedVelocity += K_v.MultiplyVector(y);

        // Joseph-form covariance update: P+ = (I - K H) P (I - K H)^T + K R K^T
        Matrix6x6 I_KH = Matrix6x6.Identity;
        for (int r = 0; r < 3; r++)
        {
            for (int c = 0; c < 3; c++)
            {
                I_KH[r, c] -= K_p[r, c];
                I_KH[r + 3, c] -= K_v[r, c];
            }
        }

        Matrix6x6 KRKt = Matrix6x6.Zero;
        Matrix3x3 KR = Matrix3x3.Multiply(K_p, R);
        Matrix3x3 Kp_R_KpT = Matrix3x3.Multiply(KR, Matrix3x3.Transpose(K_p));
        Matrix3x3 Kv_R_KpT = Matrix3x3.Multiply(Matrix3x3.Multiply(K_v, R), Matrix3x3.Transpose(K_p));
        Matrix3x3 Kv_R_KvT = Matrix3x3.Multiply(Matrix3x3.Multiply(K_v, R), Matrix3x3.Transpose(K_v));

        KRKt.SetSubMatrix3x3(0, 0, Kp_R_KpT);
        KRKt.SetSubMatrix3x3(0, 3, Matrix3x3.Transpose(Kv_R_KpT));
        KRKt.SetSubMatrix3x3(3, 0, Kv_R_KpT);
        KRKt.SetSubMatrix3x3(3, 3, Kv_R_KvT);

        covariance = Matrix6x6.Multiply(Matrix6x6.Multiply(I_KH, covariance), Matrix6x6.Transpose(I_KH)) + KRKt;
        SymmetrizeAndClampCovariance();
    }

    private void UpdatePositionAndVelocity(TargetDetection detection)
    {
        // 6D Innovation y = z - x
        Vector6 z = new Vector6(
            detection.MeasuredPosition.x, detection.MeasuredPosition.y, detection.MeasuredPosition.z,
            detection.MeasuredVelocity.x, detection.MeasuredVelocity.y, detection.MeasuredVelocity.z);

        Vector6 hx = new Vector6(
            estimatedPosition.x, estimatedPosition.y, estimatedPosition.z,
            estimatedVelocity.x, estimatedVelocity.y, estimatedVelocity.z);

        Vector6 y = z - hx;

        // 6x6 Measurement Noise R
        Matrix6x6 R = Matrix6x6.Zero;
        R[0, 0] = Mathf.Max(0.0001f, detection.PositionVariance.x);
        R[1, 1] = Mathf.Max(0.0001f, detection.PositionVariance.y);
        R[2, 2] = Mathf.Max(0.0001f, detection.PositionVariance.z);
        R[3, 3] = Mathf.Max(0.0001f, detection.VelocityVariance.x);
        R[4, 4] = Mathf.Max(0.0001f, detection.VelocityVariance.y);
        R[5, 5] = Mathf.Max(0.0001f, detection.VelocityVariance.z);

        // H = I_6x6, so S = P + R
        Matrix6x6 S = covariance + R;
        if (!S.TryInvert(out Matrix6x6 S_inv))
        {
            return;
        }

        // K = P S^-1 (6x6)
        Matrix6x6 K = Matrix6x6.Multiply(covariance, S_inv);

        // State update x = x + K y
        Vector6 stateUpdate = K.MultiplyVector(y);
        estimatedPosition += new Vector3(stateUpdate[0], stateUpdate[1], stateUpdate[2]);
        estimatedVelocity += new Vector3(stateUpdate[3], stateUpdate[4], stateUpdate[5]);

        // Joseph-form: P+ = (I - K) P (I - K)^T + K R K^T
        Matrix6x6 I_K = Matrix6x6.Identity - K;
        Matrix6x6 KRKt = Matrix6x6.Multiply(Matrix6x6.Multiply(K, R), Matrix6x6.Transpose(K));

        covariance = Matrix6x6.Multiply(Matrix6x6.Multiply(I_K, covariance), Matrix6x6.Transpose(I_K)) + KRKt;
        SymmetrizeAndClampCovariance();
    }

    private void SymmetrizeAndClampCovariance()
    {
        for (int r = 0; r < 6; r++)
        {
            for (int c = r + 1; c < 6; c++)
            {
                float avg = (covariance[r, c] + covariance[c, r]) * 0.5f;
                covariance[r, c] = avg;
                covariance[c, r] = avg;
            }
            covariance[r, r] = Mathf.Max(0.0001f, covariance[r, r]);
        }
    }

    /// <summary>
    /// Computes the normalized Mahalanobis distance squared between this track and a candidate position detection.
    /// Used for GNN data association gating.
    /// </summary>
    public float ComputeMahalanobisDistanceSq(Vector3 candidatePos, Vector3 posVariance)
    {
        if (!isInitialized) return float.MaxValue;

        Vector3 y = candidatePos - estimatedPosition;
        Matrix3x3 P_pp = covariance.GetSubMatrix3x3(0, 0);
        Matrix3x3 R = Matrix3x3.Zero;
        R[0, 0] = Mathf.Max(0.0001f, posVariance.x);
        R[1, 1] = Mathf.Max(0.0001f, posVariance.y);
        R[2, 2] = Mathf.Max(0.0001f, posVariance.z);

        Matrix3x3 S = P_pp + R;
        if (!S.TryInvert(out Matrix3x3 S_inv))
        {
            return float.MaxValue;
        }

        Vector3 Sinv_y = S_inv.MultiplyVector(y);
        return Vector3.Dot(y, Sinv_y);
    }
}

// ------------------------------------------------------------------------------------------------
// Lightweight Value-Type Math Helper Structs (Zero Heap Allocations)
// ------------------------------------------------------------------------------------------------

public struct Vector6
{
    public float m0, m1, m2, m3, m4, m5;

    public Vector6(float v0, float v1, float v2, float v3, float v4, float v5)
    {
        m0 = v0; m1 = v1; m2 = v2; m3 = v3; m4 = v4; m5 = v5;
    }

    public float this[int index]
    {
        get => index switch
        {
            0 => m0, 1 => m1, 2 => m2, 3 => m3, 4 => m4, 5 => m5,
            _ => throw new IndexOutOfRangeException()
        };
        set
        {
            switch (index)
            {
                case 0: m0 = value; break;
                case 1: m1 = value; break;
                case 2: m2 = value; break;
                case 3: m3 = value; break;
                case 4: m4 = value; break;
                case 5: m5 = value; break;
                default: throw new IndexOutOfRangeException();
            }
        }
    }

    public static Vector6 operator -(Vector6 a, Vector6 b)
    {
        return new Vector6(a.m0 - b.m0, a.m1 - b.m1, a.m2 - b.m2, a.m3 - b.m3, a.m4 - b.m4, a.m5 - b.m5);
    }
}

public struct Matrix3x3
{
    public float m00, m01, m02;
    public float m10, m11, m12;
    public float m20, m21, m22;

    public static Matrix3x3 Zero => default;
    public static Matrix3x3 Identity => new Matrix3x3 { m00 = 1f, m11 = 1f, m22 = 1f };

    public float this[int row, int col]
    {
        get => (row, col) switch
        {
            (0, 0) => m00, (0, 1) => m01, (0, 2) => m02,
            (1, 0) => m10, (1, 1) => m11, (1, 2) => m12,
            (2, 0) => m20, (2, 1) => m21, (2, 2) => m22,
            _ => throw new IndexOutOfRangeException()
        };
        set
        {
            switch (row, col)
            {
                case (0, 0): m00 = value; break;
                case (0, 1): m01 = value; break;
                case (0, 2): m02 = value; break;
                case (1, 0): m10 = value; break;
                case (1, 1): m11 = value; break;
                case (1, 2): m12 = value; break;
                case (2, 0): m20 = value; break;
                case (2, 1): m21 = value; break;
                case (2, 2): m22 = value; break;
                default: throw new IndexOutOfRangeException();
            }
        }
    }

    public static Matrix3x3 operator +(Matrix3x3 a, Matrix3x3 b)
    {
        return new Matrix3x3
        {
            m00 = a.m00 + b.m00, m01 = a.m01 + b.m01, m02 = a.m02 + b.m02,
            m10 = a.m10 + b.m10, m11 = a.m11 + b.m11, m12 = a.m12 + b.m12,
            m20 = a.m20 + b.m20, m21 = a.m21 + b.m21, m22 = a.m22 + b.m22
        };
    }

    public static Matrix3x3 Multiply(Matrix3x3 a, Matrix3x3 b)
    {
        Matrix3x3 res = default;
        for (int r = 0; r < 3; r++)
        {
            for (int c = 0; c < 3; c++)
            {
                res[r, c] = a[r, 0] * b[0, c] + a[r, 1] * b[1, c] + a[r, 2] * b[2, c];
            }
        }
        return res;
    }

    public static Matrix3x3 Transpose(Matrix3x3 a)
    {
        return new Matrix3x3
        {
            m00 = a.m00, m01 = a.m10, m02 = a.m20,
            m10 = a.m01, m11 = a.m11, m12 = a.m21,
            m20 = a.m02, m21 = a.m12, m22 = a.m22
        };
    }

    public Vector3 MultiplyVector(Vector3 v)
    {
        return new Vector3(
            m00 * v.x + m01 * v.y + m02 * v.z,
            m10 * v.x + m11 * v.y + m12 * v.z,
            m20 * v.x + m21 * v.y + m22 * v.z);
    }

    public bool TryInvert(out Matrix3x3 inv)
    {
        inv = default;
        float det = m00 * (m11 * m22 - m12 * m21) -
                    m01 * (m10 * m22 - m12 * m20) +
                    m02 * (m10 * m21 - m11 * m20);

        if (Mathf.Abs(det) < 1e-12f || !float.IsFinite(det))
        {
            return false;
        }

        float invDet = 1.0f / det;
        inv.m00 = (m11 * m22 - m12 * m21) * invDet;
        inv.m01 = (m02 * m21 - m01 * m22) * invDet;
        inv.m02 = (m01 * m12 - m02 * m11) * invDet;

        inv.m10 = (m12 * m20 - m10 * m22) * invDet;
        inv.m11 = (m00 * m22 - m02 * m20) * invDet;
        inv.m12 = (m02 * m10 - m00 * m12) * invDet;

        inv.m20 = (m10 * m21 - m11 * m20) * invDet;
        inv.m21 = (m01 * m20 - m00 * m21) * invDet;
        inv.m22 = (m00 * m11 - m01 * m10) * invDet;
        return true;
    }
}

public struct Matrix6x6
{
    private float m00, m01, m02, m03, m04, m05;
    private float m10, m11, m12, m13, m14, m15;
    private float m20, m21, m22, m23, m24, m25;
    private float m30, m31, m32, m33, m34, m35;
    private float m40, m41, m42, m43, m44, m45;
    private float m50, m51, m52, m53, m54, m55;

    public static Matrix6x6 Zero => default;

    public static Matrix6x6 Identity
    {
        get
        {
            Matrix6x6 m = default;
            m[0, 0] = 1f; m[1, 1] = 1f; m[2, 2] = 1f;
            m[3, 3] = 1f; m[4, 4] = 1f; m[5, 5] = 1f;
            return m;
        }
    }

    public float this[int r, int c]
    {
        get => r switch
        {
            0 => c switch { 0 => m00, 1 => m01, 2 => m02, 3 => m03, 4 => m04, 5 => m05, _ => 0f },
            1 => c switch { 0 => m10, 1 => m11, 2 => m12, 3 => m13, 4 => m14, 5 => m15, _ => 0f },
            2 => c switch { 0 => m20, 1 => m21, 2 => m22, 3 => m23, 4 => m24, 5 => m25, _ => 0f },
            3 => c switch { 0 => m30, 1 => m31, 2 => m32, 3 => m33, 4 => m34, 5 => m35, _ => 0f },
            4 => c switch { 0 => m40, 1 => m41, 2 => m42, 3 => m43, 4 => m44, 5 => m45, _ => 0f },
            5 => c switch { 0 => m50, 1 => m51, 2 => m52, 3 => m53, 4 => m54, 5 => m55, _ => 0f },
            _ => 0f
        };
        set
        {
            switch (r)
            {
                case 0: switch (c) { case 0: m00 = value; break; case 1: m01 = value; break; case 2: m02 = value; break; case 3: m03 = value; break; case 4: m04 = value; break; case 5: m05 = value; break; } break;
                case 1: switch (c) { case 0: m10 = value; break; case 1: m11 = value; break; case 2: m12 = value; break; case 3: m13 = value; break; case 4: m14 = value; break; case 5: m15 = value; break; } break;
                case 2: switch (c) { case 0: m20 = value; break; case 1: m21 = value; break; case 2: m22 = value; break; case 3: m23 = value; break; case 4: m24 = value; break; case 5: m25 = value; break; } break;
                case 3: switch (c) { case 0: m30 = value; break; case 1: m31 = value; break; case 2: m32 = value; break; case 3: m33 = value; break; case 4: m34 = value; break; case 5: m35 = value; break; } break;
                case 4: switch (c) { case 0: m40 = value; break; case 1: m41 = value; break; case 2: m42 = value; break; case 3: m43 = value; break; case 4: m44 = value; break; case 5: m45 = value; break; } break;
                case 5: switch (c) { case 0: m50 = value; break; case 1: m51 = value; break; case 2: m52 = value; break; case 3: m53 = value; break; case 4: m54 = value; break; case 5: m55 = value; break; } break;
            }
        }
    }

    public Matrix3x3 GetSubMatrix3x3(int startRow, int startCol)
    {
        Matrix3x3 sub = default;
        for (int r = 0; r < 3; r++)
        {
            for (int c = 0; c < 3; c++)
            {
                sub[r, c] = this[startRow + r, startCol + c];
            }
        }
        return sub;
    }

    public void SetSubMatrix3x3(int startRow, int startCol, Matrix3x3 sub)
    {
        for (int r = 0; r < 3; r++)
        {
            for (int c = 0; c < 3; c++)
            {
                this[startRow + r, startCol + c] = sub[r, c];
            }
        }
    }

    public static Matrix6x6 operator +(Matrix6x6 a, Matrix6x6 b)
    {
        Matrix6x6 res = default;
        for (int r = 0; r < 6; r++)
        {
            for (int c = 0; c < 6; c++)
            {
                res[r, c] = a[r, c] + b[r, c];
            }
        }
        return res;
    }

    public static Matrix6x6 operator -(Matrix6x6 a, Matrix6x6 b)
    {
        Matrix6x6 res = default;
        for (int r = 0; r < 6; r++)
        {
            for (int c = 0; c < 6; c++)
            {
                res[r, c] = a[r, c] - b[r, c];
            }
        }
        return res;
    }

    public static Matrix6x6 Multiply(Matrix6x6 a, Matrix6x6 b)
    {
        Matrix6x6 res = default;
        for (int r = 0; r < 6; r++)
        {
            for (int c = 0; c < 6; c++)
            {
                float sum = 0f;
                for (int k = 0; k < 6; k++)
                {
                    sum += a[r, k] * b[k, c];
                }
                res[r, c] = sum;
            }
        }
        return res;
    }

    public static Matrix6x6 Transpose(Matrix6x6 a)
    {
        Matrix6x6 res = default;
        for (int r = 0; r < 6; r++)
        {
            for (int c = 0; c < 6; c++)
            {
                res[r, c] = a[c, r];
            }
        }
        return res;
    }

    public Vector6 MultiplyVector(Vector6 v)
    {
        Vector6 res = default;
        for (int r = 0; r < 6; r++)
        {
            float sum = 0f;
            for (int c = 0; c < 6; c++)
            {
                sum += this[r, c] * v[c];
            }
            res[r] = sum;
        }
        return res;
    }

    /// <summary>
    /// Computes 6x6 matrix inversion using Gauss-Jordan elimination with partial pivoting in-place on the stack.
    /// </summary>
    public bool TryInvert(out Matrix6x6 result)
    {
        result = Identity;
        Matrix6x6 a = this;

        for (int col = 0; col < 6; col++)
        {
            // 1. Pivot selection
            int maxRow = col;
            float maxVal = Mathf.Abs(a[col, col]);
            for (int row = col + 1; row < 6; row++)
            {
                float val = Mathf.Abs(a[row, col]);
                if (val > maxVal)
                {
                    maxVal = val;
                    maxRow = row;
                }
            }

            if (maxVal < 1e-12f || !float.IsFinite(maxVal))
            {
                result = default;
                return false;
            }

            // Swap rows
            if (maxRow != col)
            {
                for (int k = 0; k < 6; k++)
                {
                    float tempA = a[col, k]; a[col, k] = a[maxRow, k]; a[maxRow, k] = tempA;
                    float tempR = result[col, k]; result[col, k] = result[maxRow, k]; result[maxRow, k] = tempR;
                }
            }

            // 2. Scale pivot row
            float pivot = a[col, col];
            float invPivot = 1.0f / pivot;
            for (int k = 0; k < 6; k++)
            {
                a[col, k] *= invPivot;
                result[col, k] *= invPivot;
            }

            // 3. Eliminate other rows
            for (int row = 0; row < 6; row++)
            {
                if (row != col)
                {
                    float factor = a[row, col];
                    for (int k = 0; k < 6; k++)
                    {
                        a[row, k] -= factor * a[col, k];
                        result[row, k] -= factor * result[col, k];
                    }
                }
            }
        }
        return true;
    }
}
