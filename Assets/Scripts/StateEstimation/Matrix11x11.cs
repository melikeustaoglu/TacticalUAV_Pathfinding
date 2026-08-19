using System;
using UnityEngine;

/// <summary>
/// High-performance, zero-allocation fixed-size 11x11 Matrix and 11-element Vector utilities
/// engineered specifically for the 11-state Discrete Extended Kalman Filter.
/// </summary>
public struct Vector11
{
    public const int Size = 11;
    public float v0, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10;

    public float this[int index]
    {
        get
        {
            switch (index)
            {
                case 0: return v0;
                case 1: return v1;
                case 2: return v2;
                case 3: return v3;
                case 4: return v4;
                case 5: return v5;
                case 6: return v6;
                case 7: return v7;
                case 8: return v8;
                case 9: return v9;
                case 10: return v10;
                default: throw new IndexOutOfRangeException($"Vector11 index {index} out of range [0, 10].");
            }
        }
        set
        {
            switch (index)
            {
                case 0: v0 = value; break;
                case 1: v1 = value; break;
                case 2: v2 = value; break;
                case 3: v3 = value; break;
                case 4: v4 = value; break;
                case 5: v5 = value; break;
                case 6: v6 = value; break;
                case 7: v7 = value; break;
                case 8: v8 = value; break;
                case 9: v9 = value; break;
                case 10: v10 = value; break;
                default: throw new IndexOutOfRangeException($"Vector11 index {index} out of range [0, 10].");
            }
        }
    }

    public static Vector11 Zero => default;

    public bool IsFinite()
    {
        for (int i = 0; i < Size; i++)
        {
            if (!float.IsFinite(this[i])) return false;
        }
        return true;
    }
}

/// <summary>
/// Fixed-size 11x11 covariance and transition matrix structure.
/// Stored in flat stack-allocated fields for zero GC pressure and cache locality.
/// </summary>
public struct Matrix11x11
{
    public const int Size = 11;

    // Flat 121 elements (Row-Major: m_row_col)
    public float m00, m01, m02, m03, m04, m05, m06, m07, m08, m09, m010;
    public float m10, m11, m12, m13, m14, m15, m16, m17, m18, m19, m110;
    public float m20, m21, m22, m23, m24, m25, m26, m27, m28, m29, m210;
    public float m30, m31, m32, m33, m34, m35, m36, m37, m38, m39, m310;
    public float m40, m41, m42, m43, m44, m45, m46, m47, m48, m49, m410;
    public float m50, m51, m52, m53, m54, m55, m56, m57, m58, m59, m510;
    public float m60, m61, m62, m63, m64, m65, m66, m67, m68, m69, m610;
    public float m70, m71, m72, m73, m74, m75, m76, m77, m78, m79, m710;
    public float m80, m81, m82, m83, m84, m85, m86, m87, m88, m89, m810;
    public float m90, m91, m92, m93, m94, m95, m96, m97, m98, m99, m910;
    public float m100, m101, m102, m103, m104, m105, m106, m107, m108, m109, m1010;

    public float this[int row, int col]
    {
        get
        {
            switch (row * Size + col)
            {
                case 0: return m00; case 1: return m01; case 2: return m02; case 3: return m03; case 4: return m04; case 5: return m05; case 6: return m06; case 7: return m07; case 8: return m08; case 9: return m09; case 10: return m010;
                case 11: return m10; case 12: return m11; case 13: return m12; case 14: return m13; case 15: return m14; case 16: return m15; case 17: return m16; case 18: return m17; case 19: return m18; case 20: return m19; case 21: return m110;
                case 22: return m20; case 23: return m21; case 24: return m22; case 25: return m23; case 26: return m24; case 27: return m25; case 28: return m26; case 29: return m27; case 30: return m28; case 31: return m29; case 32: return m210;
                case 33: return m30; case 34: return m31; case 35: return m32; case 36: return m33; case 37: return m34; case 38: return m35; case 39: return m36; case 40: return m37; case 41: return m38; case 42: return m39; case 43: return m310;
                case 44: return m40; case 45: return m41; case 46: return m42; case 47: return m43; case 48: return m44; case 49: return m45; case 50: return m46; case 51: return m47; case 52: return m48; case 53: return m49; case 54: return m410;
                case 55: return m50; case 56: return m51; case 57: return m52; case 58: return m53; case 59: return m54; case 60: return m55; case 61: return m56; case 62: return m57; case 63: return m58; case 64: return m59; case 65: return m510;
                case 66: return m60; case 67: return m61; case 68: return m62; case 69: return m63; case 70: return m64; case 71: return m65; case 72: return m66; case 73: return m67; case 74: return m68; case 75: return m69; case 76: return m610;
                case 77: return m70; case 78: return m71; case 79: return m72; case 80: return m73; case 81: return m74; case 82: return m75; case 83: return m76; case 84: return m77; case 85: return m78; case 86: return m79; case 87: return m710;
                case 88: return m80; case 89: return m81; case 90: return m82; case 91: return m83; case 92: return m84; case 93: return m85; case 94: return m86; case 95: return m87; case 96: return m88; case 97: return m89; case 98: return m810;
                case 99: return m90; case 100: return m91; case 101: return m92; case 102: return m93; case 103: return m94; case 104: return m95; case 105: return m96; case 106: return m97; case 107: return m98; case 108: return m99; case 109: return m910;
                case 110: return m100; case 111: return m101; case 112: return m102; case 113: return m103; case 114: return m104; case 115: return m105; case 116: return m106; case 117: return m107; case 118: return m108; case 119: return m109; case 120: return m1010;
                default: throw new IndexOutOfRangeException($"Matrix11x11 index [{row}, {col}] out of range.");
            }
        }
        set
        {
            switch (row * Size + col)
            {
                case 0: m00 = value; break; case 1: m01 = value; break; case 2: m02 = value; break; case 3: m03 = value; break; case 4: m04 = value; break; case 5: m05 = value; break; case 6: m06 = value; break; case 7: m07 = value; break; case 8: m08 = value; break; case 9: m09 = value; break; case 10: m010 = value; break;
                case 11: m10 = value; break; case 12: m11 = value; break; case 13: m12 = value; break; case 14: m13 = value; break; case 15: m14 = value; break; case 16: m15 = value; break; case 17: m16 = value; break; case 18: m17 = value; break; case 19: m18 = value; break; case 20: m19 = value; break; case 21: m110 = value; break;
                case 22: m20 = value; break; case 23: m21 = value; break; case 24: m22 = value; break; case 25: m23 = value; break; case 26: m24 = value; break; case 27: m25 = value; break; case 28: m26 = value; break; case 29: m27 = value; break; case 30: m28 = value; break; case 31: m29 = value; break; case 32: m210 = value; break;
                case 33: m30 = value; break; case 34: m31 = value; break; case 35: m32 = value; break; case 36: m33 = value; break; case 37: m34 = value; break; case 38: m35 = value; break; case 39: m36 = value; break; case 40: m37 = value; break; case 41: m38 = value; break; case 42: m39 = value; break; case 43: m310 = value; break;
                case 44: m40 = value; break; case 45: m41 = value; break; case 46: m42 = value; break; case 47: m43 = value; break; case 48: m44 = value; break; case 49: m45 = value; break; case 50: m46 = value; break; case 51: m47 = value; break; case 52: m48 = value; break; case 53: m49 = value; break; case 54: m410 = value; break;
                case 55: m50 = value; break; case 56: m51 = value; break; case 57: m52 = value; break; case 58: m53 = value; break; case 59: m54 = value; break; case 60: m55 = value; break; case 61: m56 = value; break; case 62: m57 = value; break; case 63: m58 = value; break; case 64: m59 = value; break; case 65: m510 = value; break;
                case 66: m60 = value; break; case 67: m61 = value; break; case 68: m62 = value; break; case 69: m63 = value; break; case 70: m64 = value; break; case 71: m65 = value; break; case 72: m66 = value; break; case 73: m67 = value; break; case 74: m68 = value; break; case 75: m69 = value; break; case 76: m610 = value; break;
                case 77: m70 = value; break; case 78: m71 = value; break; case 79: m72 = value; break; case 80: m73 = value; break; case 81: m74 = value; break; case 82: m75 = value; break; case 83: m76 = value; break; case 84: m77 = value; break; case 85: m78 = value; break; case 86: m79 = value; break; case 87: m710 = value; break;
                case 88: m80 = value; break; case 89: m81 = value; break; case 90: m82 = value; break; case 91: m83 = value; break; case 92: m84 = value; break; case 93: m85 = value; break; case 94: m86 = value; break; case 95: m87 = value; break; case 96: m88 = value; break; case 97: m89 = value; break; case 98: m810 = value; break;
                case 99: m90 = value; break; case 100: m91 = value; break; case 101: m92 = value; break; case 102: m93 = value; break; case 103: m94 = value; break; case 104: m95 = value; break; case 105: m96 = value; break; case 106: m97 = value; break; case 107: m98 = value; break; case 108: m99 = value; break; case 109: m910 = value; break;
                case 110: m100 = value; break; case 111: m101 = value; break; case 112: m102 = value; break; case 113: m103 = value; break; case 114: m104 = value; break; case 115: m105 = value; break; case 116: m106 = value; break; case 117: m107 = value; break; case 118: m108 = value; break; case 119: m109 = value; break; case 120: m1010 = value; break;
                default: throw new IndexOutOfRangeException($"Matrix11x11 index [{row}, {col}] out of range.");
            }
        }
    }

    public static Matrix11x11 Identity
    {
        get
        {
            Matrix11x11 I = default;
            for (int i = 0; i < Size; i++) I[i, i] = 1.0f;
            return I;
        }
    }

    public static Matrix11x11 Zero => default;

    /// <summary>
    /// Computes result = F * P * F^T + Q in-place without memory allocation.
    /// </summary>
    public static void PropagateCovariance(in Matrix11x11 F, in Matrix11x11 P, in Matrix11x11 Q, ref Matrix11x11 result)
    {
        // Temp = F * P
        Matrix11x11 temp = default;
        for (int i = 0; i < Size; i++)
        {
            for (int j = 0; j < Size; j++)
            {
                float sum = 0f;
                for (int k = 0; k < Size; k++)
                {
                    sum += F[i, k] * P[k, j];
                }
                temp[i, j] = sum;
            }
        }

        // result = temp * F^T + Q
        for (int i = 0; i < Size; i++)
        {
            for (int j = 0; j < Size; j++)
            {
                float sum = 0f;
                for (int k = 0; k < Size; k++)
                {
                    sum += temp[i, k] * F[j, k]; // F^T[k, j] == F[j, k]
                }
                result[i, j] = sum + Q[i, j];
            }
        }

        // Enforce symmetry
        result.EnforceSymmetry();
    }

    /// <summary>
    /// Executes a Joseph-form scalar measurement covariance update:
    /// P_new = (I - K*H) * P * (I - K*H)^T + K * r * K^T
    /// for a single state index observation (where H has 1.0 at stateIndex and 0 elsewhere).
    /// </summary>
    public static void UpdateJosephScalar(
        in Matrix11x11 P,
        in Vector11 K,
        int stateIndex,
        float measurementVariance,
        ref Matrix11x11 result)
    {
        // A = (I - K * H). Since H has only 1.0 at stateIndex: A[i, j] = (i == j ? 1 : 0) - K[i] * (j == stateIndex ? 1 : 0)
        Matrix11x11 A = Identity;
        for (int i = 0; i < Size; i++)
        {
            A[i, stateIndex] -= K[i];
        }

        // Temp = A * P
        Matrix11x11 temp = default;
        for (int i = 0; i < Size; i++)
        {
            for (int j = 0; j < Size; j++)
            {
                float sum = 0f;
                for (int k = 0; k < Size; k++)
                {
                    sum += A[i, k] * P[k, j];
                }
                temp[i, j] = sum;
            }
        }

        // result = temp * A^T + r * K * K^T
        for (int i = 0; i < Size; i++)
        {
            for (int j = 0; j < Size; j++)
            {
                float sum = 0f;
                for (int k = 0; k < Size; k++)
                {
                    sum += temp[i, k] * A[j, k];
                }
                result[i, j] = sum + measurementVariance * K[i] * K[j];
            }
        }

        result.EnforceSymmetry();
        result.EnforcePositiveDiagonal(1e-8f);
    }

    public void EnforceSymmetry()
    {
        for (int i = 0; i < Size; i++)
        {
            for (int j = i + 1; j < Size; j++)
            {
                float avg = (this[i, j] + this[j, i]) * 0.5f;
                this[i, j] = avg;
                this[j, i] = avg;
            }
        }
    }

    public void EnforcePositiveDiagonal(float minVariance = 1e-8f)
    {
        for (int i = 0; i < Size; i++)
        {
            if (this[i, i] < minVariance || float.IsNaN(this[i, i]))
            {
                this[i, i] = minVariance;
            }
        }
    }

    public bool IsFinite()
    {
        for (int i = 0; i < Size; i++)
        {
            for (int j = 0; j < Size; j++)
            {
                if (!float.IsFinite(this[i, j])) return false;
            }
        }
        return true;
    }
}
