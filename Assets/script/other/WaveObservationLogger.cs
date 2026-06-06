using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;
using UnityEngine.SceneManagement;

public sealed class WaveObservationLogger : MonoBehaviour
{
    static bool createdThisScene;
    public static event Action<WaveSummary> ObservationCompleted;

    [Header("Observation Target")]
    public WaterSurface waterSurface;
    public Transform observationPoint;
    public float fallbackWaterLevelY = 0f;

    [Header("Observation Settings")]
    public bool autoStartOnEnable = true;
    public float observationDurationSeconds = 300f;
    public float sampleRateHz = 10f;

    [Header("Output")]
    public bool writeRawCsv = true;
    public bool writeSpectrumCsv = true;
    public string outputFolderName = "WaveObservations";

    readonly List<float> sampleTimes = new List<float>(4096);
    readonly List<float> surfaceHeights = new List<float>(4096);

    WaterSearchParameters waterSearchParameters;
    WaterSearchResult waterSearchResult;
    bool hasWaterSearchCandidate;

    bool isObserving;
    bool pendingAutoStart;
    float observationStartTime;
    float nextSampleTime;

    public bool IsObserving => isObserving;
    public WaveSummary LastSummary { get; private set; }

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    static void CreateIfNeeded()
    {
        if (!SceneSelector.IsWaveEvaluationSceneActive()) return;
        EnsureInstance();
    }

    public static void EnsureInstance()
    {
        createdThisScene = false;
        Scene activeScene = SceneManager.GetActiveScene();
        if (!activeScene.IsValid()) return;
        if (activeScene.name == SceneSelector.MenuSceneName) return;
        if (activeScene.name != SceneSelector.WaveEvaluationSceneName) return;
        if (FindFirstObjectByType<WaveObservationLogger>() != null) return;

        WaterSurface surface = FindFirstObjectByType<WaterSurface>();
        GameObject go = new GameObject("WaveObservationLogger");
        WaveObservationLogger logger = go.AddComponent<WaveObservationLogger>();
        logger.waterSurface = surface;
        createdThisScene = true;
    }

    void OnEnable()
    {
        TryAssignWaterSurface();

        if (SceneSelector.IsWaveEvaluationSceneActive() &&
            FindFirstObjectByType<WaveParameterSweepRunner>() != null)
        {
            autoStartOnEnable = false;
        }

        if (!autoStartOnEnable) return;

        if (waterSurface != null)
        {
            StartObservation();
        }
        else
        {
            pendingAutoStart = true;
            Debug.Log("[WaveObservation] Waiting for WaterSurface before starting observation.");
        }
    }

    void Update()
    {
        if (pendingAutoStart && waterSurface == null)
            TryAssignWaterSurface();

        if (pendingAutoStart && waterSurface != null)
        {
            pendingAutoStart = false;
            StartObservation();
        }

        if (!isObserving) return;

        float now = Time.realtimeSinceStartup;
        if (now >= nextSampleTime)
        {
            Sample(now);
            nextSampleTime += 1f / Mathf.Max(0.1f, sampleRateHz);
        }

        if (now - observationStartTime >= observationDurationSeconds)
        {
            StopObservationAndWrite();
        }
    }

    [ContextMenu("Start Observation")]
    public void StartObservation()
    {
        if (isObserving)
        {
            Debug.LogWarning("[WaveObservation] Observation is already running.");
            return;
        }

        TryAssignWaterSurface();

        if (waterSurface == null)
        {
            Debug.LogWarning("[WaveObservation] WaterSurface not found. Observation was not started.");
            return;
        }

        sampleTimes.Clear();
        surfaceHeights.Clear();
        hasWaterSearchCandidate = false;

        isObserving = true;
        observationStartTime = Time.realtimeSinceStartup;
        nextSampleTime = observationStartTime;

        Debug.Log($"[WaveObservation] Started. duration={observationDurationSeconds:F1}s sampleRate={sampleRateHz:F1}Hz");
    }

    [ContextMenu("Stop Observation And Write")]
    public void StopObservationAndWrite()
    {
        if (!isObserving && sampleTimes.Count == 0)
        {
            Debug.LogWarning("[WaveObservation] No samples to write.");
            return;
        }

        isObserving = false;

        if (sampleTimes.Count < 8)
        {
            Debug.LogWarning("[WaveObservation] Too few samples. Increase duration or sample rate.");
            return;
        }

        WaveSummary summary = Analyze();
        LastSummary = summary;
        string folderPath = Path.Combine(Application.persistentDataPath, outputFolderName);
        Directory.CreateDirectory(folderPath);

        string stamp = DateTime.Now.ToString("yyyyMMdd_HHmmss", CultureInfo.InvariantCulture);
        string rawPath = Path.Combine(folderPath, $"wave_observation_{stamp}.csv");
        string spectrumPath = Path.Combine(folderPath, $"wave_spectrum_{stamp}.csv");

        if (writeRawCsv)
            File.WriteAllText(rawPath, BuildRawCsv(summary), Encoding.UTF8);
        if (writeSpectrumCsv)
            File.WriteAllText(spectrumPath, BuildSpectrumCsv(summary), Encoding.UTF8);

        Debug.Log(
            $"[WaveObservation] Completed.\n" +
            $"Samples: {sampleTimes.Count}\n" +
            $"Observed duration: {summary.ObservedDurationSeconds:F2} s\n" +
            $"Mean water level: {summary.MeanSurfaceY:F4} m\n" +
            $"Std(eta): {summary.StandardDeviationEta:F4} m\n" +
            $"Hs (4σ): {summary.SignificantWaveHeightSigma:F4} m\n" +
            $"Hs (top 1/3 waves): {summary.SignificantWaveHeightTopThird:F4} m\n" +
            $"Dominant frequency: {summary.DominantFrequencyHz:F4} Hz\n" +
            $"Dominant period: {summary.DominantPeriodSeconds:F4} s\n" +
            $"Raw CSV: {rawPath}\n" +
            $"Spectrum CSV: {spectrumPath}");

        ObservationCompleted?.Invoke(summary);
    }

    void Awake()
    {
        if (createdThisScene)
        {
            Debug.Log("[WaveObservation] Auto-created for this scene.");
            createdThisScene = false;
        }

        string folderPath = Path.Combine(Application.persistentDataPath, outputFolderName);
        Debug.Log($"[WaveObservation] Output folder: {folderPath}");
    }

    void Sample(float now)
    {
        Vector3 worldPos = observationPoint != null ? observationPoint.position : transform.position;
        float surfaceY = GetWaterSurfaceYAt(worldPos);

        sampleTimes.Add(now - observationStartTime);
        surfaceHeights.Add(surfaceY);
    }

    void TryAssignWaterSurface()
    {
        if (waterSurface == null)
            waterSurface = FindFirstObjectByType<WaterSurface>();
    }

    float GetWaterSurfaceYAt(Vector3 worldPos)
    {
        if (waterSurface != null && TryGetHDRPWaterSurface(worldPos, out float y))
            return y;

        return fallbackWaterLevelY;
    }

    bool TryGetHDRPWaterSurface(Vector3 worldPos, out float surfaceY)
    {
        if (!hasWaterSearchCandidate)
        {
            waterSearchResult.candidateLocationWS = worldPos;
            hasWaterSearchCandidate = true;
        }

        waterSearchParameters.startPositionWS = waterSearchResult.candidateLocationWS;
        waterSearchParameters.targetPositionWS = worldPos;
        waterSearchParameters.error = 0.01f;
        waterSearchParameters.maxIterations = 8;
        waterSearchParameters.outputNormal = false;

        bool ok = waterSurface.ProjectPointOnWaterSurface(waterSearchParameters, out waterSearchResult);
        if (ok)
        {
            surfaceY = waterSearchResult.projectedPositionWS.y;
            return true;
        }

        hasWaterSearchCandidate = false;
        surfaceY = fallbackWaterLevelY;
        return false;
    }

    WaveSummary Analyze()
    {
        int sampleCount = sampleTimes.Count;
        double meanSurfaceY = 0.0;
        for (int i = 0; i < sampleCount; i++)
            meanSurfaceY += surfaceHeights[i];
        meanSurfaceY /= sampleCount;

        double[] eta = new double[sampleCount];
        double variance = 0.0;
        for (int i = 0; i < sampleCount; i++)
        {
            eta[i] = surfaceHeights[i] - meanSurfaceY;
            variance += eta[i] * eta[i];
        }
        variance /= sampleCount;
        double stdEta = Math.Sqrt(variance);

        List<double> waveHeights = ComputeZeroUpcrossingWaveHeights(meanSurfaceY);
        double hsTopThird = 0.0;
        if (waveHeights.Count > 0)
        {
            waveHeights.Sort((a, b) => b.CompareTo(a));
            int topCount = Math.Max(1, waveHeights.Count / 3);
            for (int i = 0; i < topCount; i++)
                hsTopThird += waveHeights[i];
            hsTopThird /= topCount;
        }

        double meanDt = 0.0;
        for (int i = 1; i < sampleCount; i++)
            meanDt += sampleTimes[i] - sampleTimes[i - 1];
        meanDt /= Math.Max(1, sampleCount - 1);

        SpectrumPoint[] spectrum = ComputeSpectrum(eta, meanDt);
        double dominantFrequency = 0.0;
        double dominantEnergy = -1.0;
        for (int i = 0; i < spectrum.Length; i++)
        {
            if (spectrum[i].FrequencyHz <= 0.0) continue;
            if (spectrum[i].Energy <= dominantEnergy) continue;
            dominantEnergy = spectrum[i].Energy;
            dominantFrequency = spectrum[i].FrequencyHz;
        }

        return new WaveSummary
        {
            MeanSurfaceY = meanSurfaceY,
            StandardDeviationEta = stdEta,
            SignificantWaveHeightSigma = 4.0 * stdEta,
            SignificantWaveHeightTopThird = hsTopThird,
            DominantFrequencyHz = dominantFrequency,
            DominantPeriodSeconds = dominantFrequency > 1e-6 ? 1.0 / dominantFrequency : 0.0,
            ObservedDurationSeconds = sampleTimes[sampleCount - 1] - sampleTimes[0],
            Eta = eta,
            Spectrum = spectrum
        };
    }

    List<double> ComputeZeroUpcrossingWaveHeights(double meanSurfaceY)
    {
        var heights = new List<double>();
        bool inWave = false;
        double localMin = 0.0;
        double localMax = 0.0;

        for (int i = 1; i < surfaceHeights.Count; i++)
        {
            double prev = surfaceHeights[i - 1] - meanSurfaceY;
            double curr = surfaceHeights[i] - meanSurfaceY;

            if (!inWave && prev <= 0.0 && curr > 0.0)
            {
                inWave = true;
                localMin = curr;
                localMax = curr;
                continue;
            }

            if (!inWave) continue;

            if (curr < localMin) localMin = curr;
            if (curr > localMax) localMax = curr;

            if (prev <= 0.0 && curr > 0.0)
            {
                heights.Add(localMax - localMin);
                localMin = curr;
                localMax = curr;
            }
        }

        return heights;
    }

    SpectrumPoint[] ComputeSpectrum(double[] eta, double dt)
    {
        int n = eta.Length;
        int half = n / 2;
        var spectrum = new SpectrumPoint[Math.Max(0, half - 1)];
        if (n < 4 || dt <= 0.0) return spectrum;

        for (int k = 1; k < half; k++)
        {
            double re = 0.0;
            double im = 0.0;
            for (int t = 0; t < n; t++)
            {
                double window = 0.5 * (1.0 - Math.Cos(2.0 * Math.PI * t / (n - 1)));
                double phase = -2.0 * Math.PI * k * t / n;
                double sample = eta[t] * window;
                re += sample * Math.Cos(phase);
                im += sample * Math.Sin(phase);
            }

            double power = (re * re + im * im) / n;
            double frequencyHz = k / (n * dt);
            spectrum[k - 1] = new SpectrumPoint
            {
                FrequencyHz = frequencyHz,
                Energy = power
            };
        }

        return spectrum;
    }

    string BuildRawCsv(WaveSummary summary)
    {
        var sb = new StringBuilder();
        sb.AppendLine("time_sec,surface_y_m,eta_m");
        for (int i = 0; i < sampleTimes.Count; i++)
        {
            sb.Append(sampleTimes[i].ToString("F6", CultureInfo.InvariantCulture)).Append(',');
            sb.Append(surfaceHeights[i].ToString("F6", CultureInfo.InvariantCulture)).Append(',');
            sb.AppendLine(summary.Eta[i].ToString("F6", CultureInfo.InvariantCulture));
        }
        return sb.ToString();
    }

    string BuildSpectrumCsv(WaveSummary summary)
    {
        var sb = new StringBuilder();
        sb.AppendLine("frequency_hz,energy");
        for (int i = 0; i < summary.Spectrum.Length; i++)
        {
            sb.Append(summary.Spectrum[i].FrequencyHz.ToString("F6", CultureInfo.InvariantCulture)).Append(',');
            sb.AppendLine(summary.Spectrum[i].Energy.ToString("F12", CultureInfo.InvariantCulture));
        }
        return sb.ToString();
    }

    public struct SpectrumPoint
    {
        public double FrequencyHz;
        public double Energy;
    }

    public struct WaveSummary
    {
        public double MeanSurfaceY;
        public double StandardDeviationEta;
        public double SignificantWaveHeightSigma;
        public double SignificantWaveHeightTopThird;
        public double DominantFrequencyHz;
        public double DominantPeriodSeconds;
        public double ObservedDurationSeconds;
        public double[] Eta;
        public SpectrumPoint[] Spectrum;
    }
}
