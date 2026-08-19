using System;

/// <summary>
/// High-performance, allocation-free Box-Muller Gaussian pseudo-random number generator.
/// Supports deterministic seeding for repeatable simulation and unit testing.
/// </summary>
public class GaussianNoiseGenerator
{
    private readonly System.Random random;
    private bool hasSpare = false;
    private double spare;

    public GaussianNoiseGenerator(int seed = 0)
    {
        random = (seed != 0) ? new System.Random(seed) : new System.Random();
    }

    /// <summary>
    /// Generates a standard normally distributed random scalar with mean 0 and standard deviation 1.
    /// </summary>
    public float SampleStandardNormal()
    {
        if (hasSpare)
        {
            hasSpare = false;
            return (float)spare;
        }

        double u, v, s;
        do
        {
            u = random.NextDouble() * 2.0 - 1.0;
            v = random.NextDouble() * 2.0 - 1.0;
            s = u * u + v * v;
        } while (s >= 1.0 || s == 0.0);

        double mul = Math.Sqrt(-2.0 * Math.Log(s) / s);
        spare = v * mul;
        hasSpare = true;
        return (float)(u * mul);
    }

    /// <summary>
    /// Generates a normally distributed scalar with specified mean and standard deviation.
    /// </summary>
    public float Sample(float mean, float standardDeviation)
    {
        if (standardDeviation <= 0f) return mean;
        return mean + SampleStandardNormal() * standardDeviation;
    }

    /// <summary>
    /// Generates a 3D Gaussian noise vector with specified standard deviations per axis.
    /// </summary>
    public UnityEngine.Vector3 SampleVector3(float sigmaX, float sigmaY, float sigmaZ)
    {
        return new UnityEngine.Vector3(
            Sample(0f, sigmaX),
            Sample(0f, sigmaY),
            Sample(0f, sigmaZ));
    }
}
