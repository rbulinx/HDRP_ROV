using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Reflection;
using System.Text;
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;

public sealed class WaveParameterSweepRunner : MonoBehaviour
{
    [Serializable]
    public class SweepParameter
    {
        public string fieldName;
        public float baselineScale = 1f;
        public float[] multipliers = { 0.8f, 1.0f, 1.2f };
    }

    struct SweepCase
    {
        public string FieldName;
        public float BaselineValue;
        public float Multiplier;
        public float AppliedValue;
    }

    [Header("Execution")]
    public bool autoStartOnEnable = true;
    public float settleTimeSeconds = 3f;
    public float observationDurationSeconds = 60f;
    public bool writePerCaseWaveFiles = false;
    public string outputFolderName = "WaveParameterSweep";

    [Header("Target")]
    public WaterSurface waterSurface;
    public WaveObservationLogger observationLogger;

    [Header("Sweep Parameters")]
    public SweepParameter[] parameters =
    {
        new SweepParameter { fieldName = "largeBand0Multiplier", multipliers = new[] { 0.7f, 1.0f, 1.3f } },
        new SweepParameter { fieldName = "largeBand1Multiplier", multipliers = new[] { 0.7f, 1.0f, 1.3f } },
        new SweepParameter { fieldName = "largeWindSpeed", multipliers = new[] { 0.8f, 1.0f, 1.2f } },
        new SweepParameter { fieldName = "largeChaos", multipliers = new[] { 0.7f, 1.0f, 1.3f } },
        new SweepParameter { fieldName = "ripplesChaos", multipliers = new[] { 0.7f, 1.0f, 1.3f } },
        new SweepParameter { fieldName = "timeMultiplier", multipliers = new[] { 0.85f, 1.0f, 1.15f } },
    };

    readonly Queue<SweepCase> cases = new Queue<SweepCase>();
    readonly Dictionary<string, object> originalValues = new Dictionary<string, object>();
    readonly List<string> resultLines = new List<string>();

    FieldInfo activeField;
    SweepCase activeCase;
    float settleEndTime;
    bool waitingForObservation;
    bool sweepRunning;

    string OutputFolderPath => Path.Combine(Application.persistentDataPath, outputFolderName);

    public static void EnsureInstance()
    {
        if (!SceneSelector.IsWaveEvaluationSceneActive()) return;
        if (FindFirstObjectByType<WaveParameterSweepRunner>() != null) return;

        GameObject go = new GameObject("WaveParameterSweepRunner");
        go.AddComponent<WaveParameterSweepRunner>();
    }

    void OnEnable()
    {
        WaveObservationLogger.ObservationCompleted += OnObservationCompleted;

        if (!autoStartOnEnable) return;
        StartSweep();
    }

    void OnDisable()
    {
        WaveObservationLogger.ObservationCompleted -= OnObservationCompleted;
    }

    void Update()
    {
        if (!sweepRunning || waitingForObservation) return;
        if (Time.realtimeSinceStartup < settleEndTime) return;

        if (observationLogger == null)
        {
            Debug.LogWarning("[WaveSweep] Observation logger not found.");
            sweepRunning = false;
            return;
        }

        observationLogger.StartObservation();
        waitingForObservation = true;
    }

    [ContextMenu("Start Sweep")]
    public void StartSweep()
    {
        if (sweepRunning)
        {
            Debug.LogWarning("[WaveSweep] Sweep is already running.");
            return;
        }

        if (!SceneSelector.IsWaveEvaluationSceneActive())
        {
            Debug.LogWarning("[WaveSweep] Run this only in WaveEvaluation_U_Boat.");
            return;
        }

        if (waterSurface == null)
            waterSurface = FindFirstObjectByType<WaterSurface>();
        if (observationLogger == null)
            observationLogger = FindFirstObjectByType<WaveObservationLogger>();

        if (waterSurface == null || observationLogger == null)
        {
            Debug.LogWarning("[WaveSweep] Missing WaterSurface or WaveObservationLogger.");
            return;
        }

        observationLogger.autoStartOnEnable = false;
        observationLogger.observationDurationSeconds = observationDurationSeconds;
        observationLogger.writeRawCsv = writePerCaseWaveFiles;
        observationLogger.writeSpectrumCsv = writePerCaseWaveFiles;

        Directory.CreateDirectory(OutputFolderPath);
        Debug.Log($"[WaveSweep] Output folder: {OutputFolderPath}");

        cases.Clear();
        resultLines.Clear();
        originalValues.Clear();

        resultLines.Add("field_name,baseline_value,multiplier,applied_value,Hs_4sigma_m,Hs_top_third_m,dominant_frequency_hz,dominant_period_s,std_eta_m,observed_duration_s");
        BuildCases();

        if (cases.Count == 0)
        {
            Debug.LogWarning("[WaveSweep] No valid sweep cases were created.");
            return;
        }

        sweepRunning = true;
        Debug.Log($"[WaveSweep] Starting sweep with {cases.Count} cases.");
        BeginNextCase();
    }

    void BuildCases()
    {
        foreach (SweepParameter parameter in parameters)
        {
            if (parameter == null || string.IsNullOrWhiteSpace(parameter.fieldName)) continue;

            FieldInfo field = FindWaterField(parameter.fieldName);
            if (field == null)
            {
                Debug.LogWarning($"[WaveSweep] Field not found: {parameter.fieldName}");
                continue;
            }

            float baseline = Convert.ToSingle(field.GetValue(waterSurface), CultureInfo.InvariantCulture) * parameter.baselineScale;
            originalValues[parameter.fieldName] = field.GetValue(waterSurface);

            foreach (float multiplier in parameter.multipliers)
            {
                cases.Enqueue(new SweepCase
                {
                    FieldName = parameter.fieldName,
                    BaselineValue = baseline,
                    Multiplier = multiplier,
                    AppliedValue = baseline * multiplier
                });
            }
        }
    }

    void BeginNextCase()
    {
        if (cases.Count == 0)
        {
            FinishSweep();
            return;
        }

        activeCase = cases.Dequeue();
        activeField = FindWaterField(activeCase.FieldName);
        if (activeField == null)
        {
            BeginNextCase();
            return;
        }

        // Always restore the baseline before applying the next single-parameter change.
        RestoreOriginalValues();
        ApplyValue(activeField, activeCase.AppliedValue);
        settleEndTime = Time.realtimeSinceStartup + Mathf.Max(0f, settleTimeSeconds);
        waitingForObservation = false;

        Debug.Log($"[WaveSweep] Case start: {activeCase.FieldName} = {activeCase.AppliedValue:F4} (x{activeCase.Multiplier:F2})");
    }

    void OnObservationCompleted(WaveObservationLogger.WaveSummary summary)
    {
        if (!sweepRunning || !waitingForObservation) return;

        waitingForObservation = false;
        resultLines.Add(string.Join(",",
            activeCase.FieldName,
            activeCase.BaselineValue.ToString("F6", CultureInfo.InvariantCulture),
            activeCase.Multiplier.ToString("F3", CultureInfo.InvariantCulture),
            activeCase.AppliedValue.ToString("F6", CultureInfo.InvariantCulture),
            summary.SignificantWaveHeightSigma.ToString("F6", CultureInfo.InvariantCulture),
            summary.SignificantWaveHeightTopThird.ToString("F6", CultureInfo.InvariantCulture),
            summary.DominantFrequencyHz.ToString("F6", CultureInfo.InvariantCulture),
            summary.DominantPeriodSeconds.ToString("F6", CultureInfo.InvariantCulture),
            summary.StandardDeviationEta.ToString("F6", CultureInfo.InvariantCulture),
            summary.ObservedDurationSeconds.ToString("F6", CultureInfo.InvariantCulture)
        ));

        BeginNextCase();
    }

    void FinishSweep()
    {
        RestoreOriginalValues();
        sweepRunning = false;

        string stamp = DateTime.Now.ToString("yyyyMMdd_HHmmss", CultureInfo.InvariantCulture);
        string csvPath = Path.Combine(OutputFolderPath, $"wave_parameter_sweep_{stamp}.csv");
        File.WriteAllLines(csvPath, resultLines, Encoding.UTF8);

        Debug.Log($"[WaveSweep] Completed. Cases={resultLines.Count - 1} CSV={csvPath}");
    }

    void RestoreOriginalValues()
    {
        foreach (KeyValuePair<string, object> pair in originalValues)
        {
            FieldInfo field = FindWaterField(pair.Key);
            if (field != null)
                field.SetValue(waterSurface, pair.Value);
        }
    }

    FieldInfo FindWaterField(string fieldName)
    {
        return typeof(WaterSurface).GetField(fieldName, BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
    }

    void ApplyValue(FieldInfo field, float value)
    {
        Type type = field.FieldType;
        if (type == typeof(float))
            field.SetValue(waterSurface, value);
        else if (type == typeof(int))
            field.SetValue(waterSurface, Mathf.RoundToInt(value));
        else if (type == typeof(bool))
            field.SetValue(waterSurface, value >= 0.5f);
        else
            throw new InvalidOperationException($"Unsupported field type for sweep: {type.Name}");
    }
}
