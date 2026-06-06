using UnityEngine;

#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

public class SeaTurbiditySelector : MonoBehaviour
{
    public enum TurbidityPreset
    {
        Clear = 0,
        Normal = 1,
        Murky = 2
    }

    [Header("Targets")]
    public ImagingSonarSim[] sonarTargets;
    public bool applyToSonar = false;

    [Header("Preset")]
    public TurbidityPreset currentPreset = TurbidityPreset.Normal;
    public bool applyOnStart = true;

    [Header("Hotkeys")]
    public bool enableHotkeys = true;

    [Header("Clear")]
    public float clearAbsorptionPerMeter = 0.005f;
    public float clearSpeckle = 0.03f;
    public float clearBackground = 0.02f;
    public float clearGain = 2.8f;

    [Header("Normal")]
    public float normalAbsorptionPerMeter = 0.03f;
    public float normalSpeckle = 0.10f;
    public float normalBackground = 0.04f;
    public float normalGain = 2.0f;

    [Header("Murky")]
    public float murkyAbsorptionPerMeter = 0.16f;
    public float murkySpeckle = 0.30f;
    public float murkyBackground = 0.14f;
    public float murkyGain = 1.2f;

    void Start()
    {
        if (sonarTargets == null || sonarTargets.Length == 0)
        {
            sonarTargets = FindObjectsByType<ImagingSonarSim>(FindObjectsInactive.Exclude, FindObjectsSortMode.None);
        }

        if (applyOnStart)
        {
            ApplyCurrentPreset();
        }
    }

    void Update()
    {
#if ENABLE_INPUT_SYSTEM
        if (!enableHotkeys) return;
        if (Keyboard.current == null) return;

        if (Keyboard.current.f5Key.wasPressedThisFrame)
        {
            SetPreset(TurbidityPreset.Clear);
        }
        else if (Keyboard.current.f6Key.wasPressedThisFrame)
        {
            SetPreset(TurbidityPreset.Normal);
        }
        else if (Keyboard.current.f7Key.wasPressedThisFrame)
        {
            SetPreset(TurbidityPreset.Murky);
        }
#endif
    }

    public void SetPreset(TurbidityPreset preset)
    {
        currentPreset = preset;
        Debug.Log($"[Turbidity] SetPreset: {currentPreset}");
        ApplyCurrentPreset();
    }

    [ContextMenu("Apply Current Preset")]
    public void ApplyCurrentPreset()
    {
        if (!applyToSonar)
        {
            Debug.Log("[Turbidity] applyToSonar is false. Skip apply.");
            return;
        }
        if (sonarTargets == null || sonarTargets.Length == 0)
        {
            Debug.LogWarning("[Turbidity] sonarTargets is empty. Skip apply.");
            return;
        }

        float absorption = normalAbsorptionPerMeter;
        float speckle = normalSpeckle;
        float background = normalBackground;
        float gain = normalGain;

        if (currentPreset == TurbidityPreset.Clear)
        {
            absorption = clearAbsorptionPerMeter;
            speckle = clearSpeckle;
            background = clearBackground;
            gain = clearGain;
        }
        else if (currentPreset == TurbidityPreset.Murky)
        {
            absorption = murkyAbsorptionPerMeter;
            speckle = murkySpeckle;
            background = murkyBackground;
            gain = murkyGain;
        }

        for (int i = 0; i < sonarTargets.Length; i++)
        {
            ImagingSonarSim sonar = sonarTargets[i];
            if (sonar == null) continue;
            sonar.absorptionPerMeter = absorption;
            sonar.speckle = speckle;
            sonar.background = background;
            sonar.gain = gain;
        }

        Debug.Log($"[Turbidity] Applied preset={currentPreset} absorption={absorption:F3} speckle={speckle:F2} background={background:F2} gain={gain:F2}");
    }
}
