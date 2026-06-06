using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;

#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

public class CameraWaterVisibilitySelector : MonoBehaviour
{
    public enum VisibilityPreset
    {
        Clear = 0,
        Normal = 1,
        Murky = 2
    }

    [Header("Volume")]
    public Volume targetVolume;
    public bool applyToCamera = true;
    public bool applyFogMeanFreePath = false;

    [Header("Ocean Underwater (WaterSurface)")]
    public WaterSurface targetWaterSurface;
    public bool applyOceanAbsorptionMultiplier = true;

    [Header("Optional Sonar Sync")]
    public bool applyToSonarAlso = false;
    public SeaTurbiditySelector sonarTurbiditySelector;

    [Header("Preset")]
    public VisibilityPreset currentPreset = VisibilityPreset.Normal;
    public bool applyOnStart = true;

    [Header("Hotkeys")]
    public bool enableHotkeys = true;

    [Header("Mean Free Path (m)")]
    public float clearMeanFreePath = 800f;
    public float normalMeanFreePath = 120f;
    public float murkyMeanFreePath = 8f;

    [Header("Absorption Distance Multiplier")]
    public float clearAbsorptionDistanceMultiplier = 12f;
    public float normalAbsorptionDistanceMultiplier = 4f;
    public float murkyAbsorptionDistanceMultiplier = 0.5f;

    void Start()
    {
        if (targetVolume == null)
            targetVolume = FindFirstObjectByType<Volume>();
        if (targetWaterSurface == null)
            targetWaterSurface = FindFirstObjectByType<WaterSurface>();

        if (applyOnStart)
            ApplyCurrentPreset();
    }

    void Update()
    {
#if ENABLE_INPUT_SYSTEM
        if (!enableHotkeys || Keyboard.current == null) return;

        if (Keyboard.current.f5Key.wasPressedThisFrame) SetPreset(VisibilityPreset.Clear);
        else if (Keyboard.current.f6Key.wasPressedThisFrame) SetPreset(VisibilityPreset.Normal);
        else if (Keyboard.current.f7Key.wasPressedThisFrame) SetPreset(VisibilityPreset.Murky);
#endif
    }

    public void SetPreset(VisibilityPreset preset)
    {
        currentPreset = preset;
        Debug.Log($"[CameraVisibility] SetPreset: {currentPreset}");
        ApplyCurrentPreset();
    }

    public void SetPresetCameraOnly(VisibilityPreset preset)
    {
        currentPreset = preset;
        Debug.Log($"[CameraVisibility] SetPresetCameraOnly: {currentPreset}");
        bool prevSync = applyToSonarAlso;
        applyToSonarAlso = false;
        ApplyCurrentPreset();
        applyToSonarAlso = prevSync;
    }

    [ContextMenu("Apply Current Preset")]
    public void ApplyCurrentPreset()
    {
        Debug.Log($"[CameraVisibility] ApplyCurrentPreset start: preset={currentPreset} applyToCamera={applyToCamera} applyOceanAbsorptionMultiplier={applyOceanAbsorptionMultiplier} applyFogMeanFreePath={applyFogMeanFreePath}");

        if (applyToCamera)
        {
            if (applyFogMeanFreePath && targetVolume != null)
            {
                VolumeProfile profile = targetVolume.profile;
                if (profile != null)
                {
                    if (!profile.TryGet(out Fog fog))
                        fog = profile.Add<Fog>(true);

                    float mfp = normalMeanFreePath;
                    if (currentPreset == VisibilityPreset.Clear) mfp = clearMeanFreePath;
                    else if (currentPreset == VisibilityPreset.Murky) mfp = murkyMeanFreePath;

                    fog.active = true;
                    fog.meanFreePath.Override(Mathf.Max(1f, mfp));
                    Debug.Log($"[CameraVisibility] Fog meanFreePath set to {Mathf.Max(1f, mfp):F3}");
                }
                else
                {
                    Debug.LogWarning("[CameraVisibility] targetVolume.profile is null. Fog was not updated.");
                }
            }
            else if (applyFogMeanFreePath && targetVolume == null)
            {
                Debug.LogWarning("[CameraVisibility] targetVolume is null. Fog was not updated.");
            }

            if (applyOceanAbsorptionMultiplier && targetWaterSurface != null)
            {
                float adm = normalAbsorptionDistanceMultiplier;
                if (currentPreset == VisibilityPreset.Clear) adm = clearAbsorptionDistanceMultiplier;
                else if (currentPreset == VisibilityPreset.Murky) adm = murkyAbsorptionDistanceMultiplier;
                targetWaterSurface.absorptionDistanceMultiplier = Mathf.Max(0.001f, adm);
                Debug.Log($"[CameraVisibility] WaterSurface '{targetWaterSurface.name}' absorptionDistanceMultiplier set to {targetWaterSurface.absorptionDistanceMultiplier:F3}");
            }
            else if (applyOceanAbsorptionMultiplier && targetWaterSurface == null)
            {
                Debug.LogWarning("[CameraVisibility] targetWaterSurface is null. Ocean absorption was not updated.");
            }
        }

        if (applyToSonarAlso && sonarTurbiditySelector != null)
        {
            sonarTurbiditySelector.applyToSonar = true;
            sonarTurbiditySelector.SetPreset(ConvertToSonarPreset(currentPreset));
            Debug.Log("[CameraVisibility] Synced preset to sonar selector.");
        }
        else if (applyToSonarAlso && sonarTurbiditySelector == null)
        {
            Debug.LogWarning("[CameraVisibility] applyToSonarAlso is ON but sonarTurbiditySelector is null.");
        }
    }

    static SeaTurbiditySelector.TurbidityPreset ConvertToSonarPreset(VisibilityPreset preset)
    {
        if (preset == VisibilityPreset.Clear) return SeaTurbiditySelector.TurbidityPreset.Clear;
        if (preset == VisibilityPreset.Murky) return SeaTurbiditySelector.TurbidityPreset.Murky;
        return SeaTurbiditySelector.TurbidityPreset.Normal;
    }
}
