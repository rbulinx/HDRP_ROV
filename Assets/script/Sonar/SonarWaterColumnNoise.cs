using System;
using System.Collections.Generic;
using UnityEngine;
using Robalink.OculusEmulator;

[Serializable]
public class SonarWaterColumnNoise
{
    [Header("Enable")]
    public bool enabled = true;

    [Header("Suspended matter")]
    [Tooltip("Pseudo particle density in hits per meter. 0 disables false water-column echoes.")]
    [Range(0f, 2f)] public float particleDensity = 0.16f;
    [Range(0f, 1f)] public float particleReflectivity = 0.8f;
    [Tooltip("Small random speckle added to each false echo.")]
    [Range(0f, 1f)] public float particleSpeckle = 0.35f;

    [Header("Range attenuation")]
    [Tooltip("Absorption-like loss for false echoes.")]
    [Range(0f, 0.5f)] public float falseEchoAbsorption = 0.06f;
    [Tooltip("Additional turbidity loss applied to existing solid returns.")]
    [Range(0f, 0.5f)] public float solidReturnLossPerMeter = 0.015f;
    [Range(0f, 3f)] public float spreadingPower = 0.6f;
    [Range(0.05f, 5f)] public float minRangeMeters = 0.4f;

    [Header("Threshold / dropouts")]
    [Range(0f, 1f)] public float falseEchoThreshold = 0.015f;
    [Tooltip("Probability per meter that a range cell is partly lost in turbid water.")]
    [Range(0f, 0.5f)] public float dropoutProbabilityPerMeter = 0.01f;
    [Range(0f, 1f)] public float dropoutDarken = 0.7f;

    [Header("Shape")]
    [Range(1, 12)] public int maxFalseEchoesPerRay = 3;
    [Range(0.01f, 1f)] public float falseEchoWidthMeters = 0.12f;
    [Range(0.01f, 1f)] public float falseEchoSigmaMeters = 0.04f;
    [Range(1, 8)] public int processBeamStride = 1;
    [Range(1, 8)] public int processRangeStride = 1;

    [Header("Random")]
    public int seed = 912367;
    public bool animateNoise = true;

    public struct Point
    {
        public Vector3 position;
        public float rangeMeters;
        public float intensity01;
        public float sizeMultiplier;
        public bool isFalseEcho;
    }

    public void ApplyToRangeAzimuthFrame(
        VirtualSonarPingFrame frame,
        int beamIndex,
        float maxRangeMeters,
        float nearestSolidDistanceMeters,
        int frameIndex)
    {
        if (!enabled || frame == null || frame.Intensities8 == null) return;
        if (processBeamStride > 1 && beamIndex % processBeamStride != 0) return;

        int rangeCount = frame.RangeCount;
        if (rangeCount <= 0) return;

        float rangeResolution = Mathf.Max((float)frame.RangeResolutionMeters, 1e-5f);
        float rayLength = Mathf.Clamp(
            float.IsPositiveInfinity(nearestSolidDistanceMeters) ? maxRangeMeters : nearestSolidDistanceMeters,
            0f,
            maxRangeMeters);

        ApplySolidReturnLoss(frame, beamIndex, rangeResolution, frameIndex);
        AddFalseEchoes(frame, beamIndex, rayLength, rangeResolution, frameIndex);
    }

    public void AppendFalsePoints(
        List<Point> points,
        Vector3 origin,
        Vector3 direction,
        float maxRangeMeters,
        float nearestSolidDistanceMeters,
        int rayIndex,
        int frameIndex)
    {
        if (!enabled || points == null) return;

        direction.Normalize();
        float rayLength = Mathf.Clamp(
            float.IsPositiveInfinity(nearestSolidDistanceMeters) ? maxRangeMeters : nearestSolidDistanceMeters,
            0f,
            maxRangeMeters);

        int events = SampleFalseEchoCount(rayLength, rayIndex, frameIndex);
        for (int i = 0; i < events; i++)
        {
            uint state = HashToState(rayIndex, frameIndex, i + 31);
            float r = Mathf.Lerp(minRangeMeters, rayLength, Next01(ref state));
            float intensity = ComputeFalseEchoIntensity(r, ref state);
            if (intensity < falseEchoThreshold) continue;

            points.Add(new Point
            {
                position = origin + direction * r,
                rangeMeters = r,
                intensity01 = intensity,
                sizeMultiplier = 1f,
                isFalseEcho = true,
            });
        }
    }

    void ApplySolidReturnLoss(VirtualSonarPingFrame frame, int beamIndex, float rangeResolution, int frameIndex)
    {
        int rangeStride = Mathf.Max(1, processRangeStride);
        int rangeCount = frame.RangeCount;

        for (int rangeIndex = 0; rangeIndex < rangeCount; rangeIndex += rangeStride)
        {
            int end = Mathf.Min(rangeIndex + rangeStride, rangeCount);
            for (int idx = rangeIndex; idx < end; idx++)
            {
                byte current = frame.Intensities8[idx, beamIndex];
                if (current == 0) continue;

                float r = (idx + 1) * rangeResolution;
                float transmission = Mathf.Exp(-Mathf.Max(0f, solidReturnLossPerMeter) * particleDensity * r);

                uint state = HashToState(beamIndex, frameIndex, idx);
                float dropoutP = Mathf.Clamp01(dropoutProbabilityPerMeter * particleDensity * r);
                if (Next01(ref state) < dropoutP)
                    transmission *= 1f - dropoutDarken;

                frame.Intensities8[idx, beamIndex] = (byte)Mathf.Clamp(Mathf.RoundToInt(current * transmission), 0, 255);
            }
        }
    }

    void AddFalseEchoes(VirtualSonarPingFrame frame, int beamIndex, float rayLength, float rangeResolution, int frameIndex)
    {
        int events = SampleFalseEchoCount(rayLength, beamIndex, frameIndex);
        if (events <= 0) return;

        int rangeCount = frame.RangeCount;
        float echoSigma = Mathf.Max(rangeResolution * 0.5f, falseEchoSigmaMeters);
        int halfBins = Mathf.Max(1, Mathf.RoundToInt(falseEchoWidthMeters / rangeResolution));

        for (int echo = 0; echo < events; echo++)
        {
            uint state = HashToState(beamIndex, frameIndex, echo + 101);
            float r = Mathf.Lerp(minRangeMeters, rayLength, Next01(ref state));
            float intensity = ComputeFalseEchoIntensity(r, ref state);
            if (intensity < falseEchoThreshold) continue;

            int centerBin = Mathf.Clamp(Mathf.RoundToInt(r / rangeResolution) - 1, 0, rangeCount - 1);
            for (int delta = -halfBins; delta <= halfBins; delta++)
            {
                int rangeIndex = centerBin + delta;
                if (rangeIndex < 0 || rangeIndex >= rangeCount) continue;

                float rangeOffset = delta * rangeResolution;
                float weight = Mathf.Exp(-0.5f * rangeOffset * rangeOffset / Mathf.Max(1e-6f, echoSigma * echoSigma));
                int add = Mathf.RoundToInt(255f * intensity * weight);
                int updated = frame.Intensities8[rangeIndex, beamIndex] + add;
                frame.Intensities8[rangeIndex, beamIndex] = (byte)Mathf.Clamp(updated, 0, 255);
            }
        }
    }

    int SampleFalseEchoCount(float rayLength, int rayIndex, int frameIndex)
    {
        if (rayLength <= minRangeMeters || particleDensity <= 0f || particleReflectivity <= 0f)
            return 0;

        float expected = particleDensity * Mathf.Max(0f, rayLength - minRangeMeters);
        expected = Mathf.Min(expected, maxFalseEchoesPerRay);

        uint state = HashToState(rayIndex, animateNoise ? frameIndex : 0, 17);
        int whole = Mathf.FloorToInt(expected);
        int count = whole;
        if (Next01(ref state) < expected - whole)
            count++;

        return Mathf.Clamp(count, 0, Mathf.Max(0, maxFalseEchoesPerRay));
    }

    float ComputeFalseEchoIntensity(float rangeMeters, ref uint state)
    {
        float r = Mathf.Max(rangeMeters, minRangeMeters);
        float spread = spreadingPower <= 0f ? 1f : 1f / Mathf.Pow(r, spreadingPower);
        float absorb = Mathf.Exp(-falseEchoAbsorption * r);
        float speckle = Mathf.Lerp(1f - particleSpeckle, 1f + particleSpeckle, Next01(ref state));
        return Mathf.Clamp01(particleReflectivity * spread * absorb * speckle);
    }

    uint HashToState(int a, int b, int c)
    {
        unchecked
        {
            uint h = (uint)seed;
            h ^= (uint)a + 0x9e3779b9u + (h << 6) + (h >> 2);
            h ^= (uint)b + 0x85ebca6bu + (h << 6) + (h >> 2);
            h ^= (uint)c + 0xc2b2ae35u + (h << 6) + (h >> 2);
            return h == 0 ? 1u : h;
        }
    }

    static float Next01(ref uint state)
    {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        return (state & 0x00ffffff) / 16777216f;
    }
}
