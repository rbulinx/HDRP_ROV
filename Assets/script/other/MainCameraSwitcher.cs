using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.Controls;
using UnityEngine.UI;

public class MainCameraSwitcher : MonoBehaviour
{
    [Header("Cameras")]
    public List<Camera> cameras = new List<Camera>();
    public int startIndex = 0;
    public bool collectChildCamerasOnReset = true;
    public bool collectChildCamerasOnAwakeIfEmpty = true;
    public bool assignMainCameraTag = true;
    public bool manageAudioListeners = true;

    [Header("Underwater Particles")]
    public bool manageUnderwaterParticles = true;
    public bool underwaterParticlesEnabled = true;
    [Range(0, 10000)] public int underwaterParticleMaxCount = 1200;
    [Range(0f, 2000f)] public float underwaterParticleEmissionRate = 140f;
    public UnderwaterSuspendedParticleField.VolumeShape underwaterParticleVolumeShape = UnderwaterSuspendedParticleField.VolumeShape.Ellipsoid;
    public Vector3 underwaterParticleBoxSize = new Vector3(9f, 5f, 12f);
    public Vector3 underwaterParticleLocalOffset = new Vector3(0f, 0f, 5f);
    public Vector2 underwaterParticleLifetime = new Vector2(8f, 18f);
    public Vector2 underwaterParticleSize = new Vector2(0.012f, 0.045f);
    public Vector2 underwaterParticleSpeed = new Vector2(0.002f, 0.018f);
    [Range(0f, 1f)] public float underwaterParticleAlpha = 0.32f;
    public Vector3 underwaterParticleDriftVelocity = new Vector3(0.003f, 0.001f, -0.01f);
    public bool onlyShowParticlesBelowWaterSurface = true;
    public float waterSurfaceY = 0f;
    public float waterSurfaceMargin = 0.15f;

    [Header("Sonar Suspended Matter")]
    public bool syncSonarNoiseWithSuspendedParticles = true;
    public bool sonarSuspendedMatterEnabled = true;
    [Range(0f, 2f)] public float sonarParticleDensity = 0.35f;
    [Range(0f, 1f)] public float sonarParticleReflectivity = 1f;
    [Range(0f, 1f)] public float sonarFalseEchoThreshold = 0.005f;
    [Range(1, 12)] public int sonarMaxFalseEchoesPerRay = 5;

    [Header("Camera Image Delay")]
    public bool manageCameraImageDelay = true;
    public bool cameraImageDelayEnabled = false;
    [Range(0.03f, 2f)] public float cameraImageDelaySeconds = 0.3f;
    [Range(10, 60)] public int cameraImageDelayBufferFps = 30;
    [Range(0.25f, 1f)] public float cameraImageDelayRenderScale = 0.75f;
    public int cameraImageDelayOverlaySortingOrder = -1000;
    public Key toggleCameraImageDelayKey = Key.B;

    [Header("Keys")]
    public Key nextCameraKey = Key.C;
    public Key refreshCamerasKey = Key.V;
    public bool shiftForPrevious = true;

    [Header("On-screen Label")]
    public bool showLabel = true;
    public float labelSeconds = 2f;
    public int labelFontSize = 20;

    int currentIndex;
    Text labelText;
    float hideLabelAt;
    float nextSonarSyncTime;

    void Reset()
    {
        if (collectChildCamerasOnReset)
            CollectChildCameras();
    }

    void Awake()
    {
        EnvironmentControlMenu.EnsureInstance();

        if (collectChildCamerasOnAwakeIfEmpty && cameras.Count == 0)
            CollectChildCameras();

        RemoveNullCameras();

        currentIndex = Mathf.Clamp(startIndex, 0, Mathf.Max(0, cameras.Count - 1));
        CreateLabelIfNeeded();

        if (cameras.Count > 0)
            ApplyCamera(currentIndex, true);
    }

    void Update()
    {
        if (Time.unscaledTime >= nextSonarSyncTime)
        {
            nextSonarSyncTime = Time.unscaledTime + 0.5f;
            ApplySonarSuspendedMatterSettings();
        }

        Keyboard keyboard = Keyboard.current;
        if (keyboard == null)
        {
            UpdateLabelVisibility();
            return;
        }

        KeyControl delayKey = keyboard[toggleCameraImageDelayKey];
        if (delayKey != null && delayKey.wasPressedThisFrame)
            SetCameraImageDelayEnabled(!cameraImageDelayEnabled);

        if (cameras.Count <= 1)
        {
            UpdateLabelVisibility();
            return;
        }

        KeyControl key = keyboard[nextCameraKey];
        if (key != null && key.wasPressedThisFrame)
        {
            bool previous = shiftForPrevious &&
                            (keyboard.leftShiftKey.isPressed || keyboard.rightShiftKey.isPressed);
            SwitchBy(previous ? -1 : 1);
        }

        KeyControl refreshKey = keyboard[refreshCamerasKey];
        if (refreshKey != null && refreshKey.wasPressedThisFrame)
        {
            CollectChildCameras();
            if (cameras.Count > 0)
                ApplyCamera(Mathf.Clamp(currentIndex, 0, cameras.Count - 1), false);
        }

        UpdateLabelVisibility();
    }

    public void SwitchBy(int delta)
    {
        if (cameras.Count == 0) return;

        int next = currentIndex + delta;
        if (next < 0) next = cameras.Count - 1;
        if (next >= cameras.Count) next = 0;

        ApplyCamera(next, false);
    }

    public void SwitchTo(int index)
    {
        if (index < 0 || index >= cameras.Count) return;
        ApplyCamera(index, false);
    }

    [ContextMenu("Collect Enabled Display Cameras")]
    public void CollectEnabledDisplayCameras()
    {
        cameras.Clear();

        Camera[] found = FindObjectsByType<Camera>(FindObjectsInactive.Exclude, FindObjectsSortMode.None);
        for (int i = 0; i < found.Length; i++)
        {
            Camera cam = found[i];
            if (cam == null || cam.targetDisplay != 0) continue;
            cameras.Add(cam);
        }

        cameras.Sort((a, b) => string.Compare(a.name, b.name, System.StringComparison.OrdinalIgnoreCase));
        currentIndex = Mathf.Clamp(currentIndex, 0, Mathf.Max(0, cameras.Count - 1));
        if (cameras.Count > 0)
            ApplyCamera(currentIndex, true);
    }

    [ContextMenu("Collect Child Cameras")]
    public void CollectChildCameras()
    {
        cameras.Clear();

        Camera[] found = GetComponentsInChildren<Camera>(true);
        for (int i = 0; i < found.Length; i++)
        {
            Camera cam = found[i];
            if (cam == null || cam.targetDisplay != 0) continue;
            cameras.Add(cam);
        }

        currentIndex = Mathf.Clamp(currentIndex, 0, Mathf.Max(0, cameras.Count - 1));
    }

    void ApplyCamera(int index, bool initial)
    {
        currentIndex = index;

        for (int i = 0; i < cameras.Count; i++)
        {
            Camera cam = cameras[i];
            if (cam == null) continue;

            bool active = i == currentIndex;
            cam.enabled = active;

            if (assignMainCameraTag)
                cam.tag = active ? "MainCamera" : "Untagged";

            if (manageAudioListeners)
            {
                AudioListener listener = cam.GetComponent<AudioListener>();
                if (listener != null)
                    listener.enabled = active;
            }

            ApplyUnderwaterParticles(cam, active);
            ApplyCameraImageDelay(cam, active);
        }

        if (!initial)
            ShowLabel();

        ApplySonarSuspendedMatterSettings();
    }

    public void SetUnderwaterParticlesEnabled(bool value)
    {
        underwaterParticlesEnabled = value;
        if (cameras.Count > 0)
            ApplyCamera(currentIndex, true);
    }

    public void ToggleUnderwaterParticles()
    {
        SetUnderwaterParticlesEnabled(!underwaterParticlesEnabled);
    }

    public void SetSonarSuspendedMatterEnabled(bool value)
    {
        sonarSuspendedMatterEnabled = value;
        ApplySonarSuspendedMatterSettings();
    }

    public void ToggleSonarSuspendedMatter()
    {
        SetSonarSuspendedMatterEnabled(!sonarSuspendedMatterEnabled);
    }

    public void SetCameraImageDelayEnabled(bool value)
    {
        cameraImageDelayEnabled = value;
        for (int i = 0; i < cameras.Count; i++)
        {
            Camera cam = cameras[i];
            if (cam == null) continue;
            ApplyCameraImageDelay(cam, i == currentIndex);
        }

        ShowStatusLabel(string.Format("Camera image delay: {0}  ({1:0.00}s)",
            cameraImageDelayEnabled ? "ON" : "OFF",
            cameraImageDelaySeconds));
    }

    public void ToggleCameraImageDelay()
    {
        SetCameraImageDelayEnabled(!cameraImageDelayEnabled);
    }

    void ApplyUnderwaterParticles(Camera cam, bool active)
    {
        if (cam == null) return;

        UnderwaterSuspendedParticleField field = cam.GetComponent<UnderwaterSuspendedParticleField>();
        if (!manageUnderwaterParticles)
        {
            if (field != null) field.SetParticlesEnabled(false);
            return;
        }

        if (field == null)
            field = cam.gameObject.AddComponent<UnderwaterSuspendedParticleField>();

        field.maxParticles = underwaterParticleMaxCount;
        field.emissionRate = underwaterParticleEmissionRate;
        field.volumeShape = underwaterParticleVolumeShape;
        field.boxSize = underwaterParticleBoxSize;
        field.localOffset = underwaterParticleLocalOffset;
        field.lifetime = underwaterParticleLifetime;
        field.size = underwaterParticleSize;
        field.speed = underwaterParticleSpeed;
        field.alpha = underwaterParticleAlpha;
        field.driftVelocity = underwaterParticleDriftVelocity;
        field.onlyBelowWaterSurface = onlyShowParticlesBelowWaterSurface;
        field.waterSurfaceY = waterSurfaceY;
        field.waterSurfaceMargin = waterSurfaceMargin;
        field.SetParticlesEnabled(active && underwaterParticlesEnabled);
    }

    void ApplyCameraImageDelay(Camera cam, bool active)
    {
        if (cam == null) return;

        CameraImageDelay delay = cam.GetComponent<CameraImageDelay>();
        if (!manageCameraImageDelay)
        {
            if (delay != null)
                delay.Configure(false, cameraImageDelaySeconds, cameraImageDelayBufferFps, cameraImageDelayRenderScale);
            return;
        }

        if (delay == null)
            delay = cam.gameObject.AddComponent<CameraImageDelay>();

        delay.overlaySortingOrder = cameraImageDelayOverlaySortingOrder;
        delay.Configure(active && cameraImageDelayEnabled, cameraImageDelaySeconds, cameraImageDelayBufferFps, cameraImageDelayRenderScale);
    }

    void ApplySonarSuspendedMatterSettings()
    {
        if (!syncSonarNoiseWithSuspendedParticles) return;

        bool enabled = sonarSuspendedMatterEnabled;

        ImagingSonarSim[] imagingSonars = FindObjectsByType<ImagingSonarSim>(FindObjectsInactive.Exclude, FindObjectsSortMode.None);
        for (int i = 0; i < imagingSonars.Length; i++)
        {
            ImagingSonarSim sonar = imagingSonars[i];
            if (sonar == null) continue;

            sonar.enableWaterColumnNoise = enabled;
            if (sonar.waterColumnNoise == null)
                sonar.waterColumnNoise = new SonarWaterColumnNoise();

            sonar.waterColumnNoise.enabled = enabled;
            sonar.waterColumnNoise.particleDensity = sonarParticleDensity;
            sonar.waterColumnNoise.particleReflectivity = sonarParticleReflectivity;
            sonar.waterColumnNoise.falseEchoThreshold = sonarFalseEchoThreshold;
            sonar.waterColumnNoise.maxFalseEchoesPerRay = sonarMaxFalseEchoesPerRay;
        }

        Robalink.OculusEmulator.VirtualOculusM750dTerrainSonar[] oculusSonars =
            FindObjectsByType<Robalink.OculusEmulator.VirtualOculusM750dTerrainSonar>(FindObjectsInactive.Exclude, FindObjectsSortMode.None);
        for (int i = 0; i < oculusSonars.Length; i++)
        {
            Robalink.OculusEmulator.VirtualOculusM750dTerrainSonar sonar = oculusSonars[i];
            if (sonar == null) continue;

            sonar.enableWaterColumnNoise = enabled;
            if (sonar.waterColumnNoise == null)
                sonar.waterColumnNoise = new SonarWaterColumnNoise();

            sonar.waterColumnNoise.enabled = enabled;
            sonar.waterColumnNoise.particleDensity = sonarParticleDensity;
            sonar.waterColumnNoise.particleReflectivity = sonarParticleReflectivity;
            sonar.waterColumnNoise.falseEchoThreshold = sonarFalseEchoThreshold;
            sonar.waterColumnNoise.maxFalseEchoesPerRay = sonarMaxFalseEchoesPerRay;
        }
    }

    void RemoveNullCameras()
    {
        for (int i = cameras.Count - 1; i >= 0; i--)
        {
            if (cameras[i] == null)
                cameras.RemoveAt(i);
        }
    }

    void CreateLabelIfNeeded()
    {
        if (!showLabel) return;

        GameObject canvasObject = new GameObject("CameraSwitcherCanvas");
        Canvas canvas = canvasObject.AddComponent<Canvas>();
        canvas.renderMode = RenderMode.ScreenSpaceOverlay;
        canvas.sortingOrder = 2200;

        CanvasScaler scaler = canvasObject.AddComponent<CanvasScaler>();
        scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
        scaler.referenceResolution = new Vector2(1920f, 1080f);

        GameObject textObject = new GameObject("CameraSwitcherLabel");
        textObject.transform.SetParent(canvasObject.transform, false);

        labelText = textObject.AddComponent<Text>();
        labelText.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
        labelText.fontSize = labelFontSize;
        labelText.fontStyle = FontStyle.Bold;
        labelText.color = Color.white;
        labelText.alignment = TextAnchor.LowerLeft;
        labelText.raycastTarget = false;

        Shadow shadow = textObject.AddComponent<Shadow>();
        shadow.effectColor = new Color(0f, 0f, 0f, 0.8f);
        shadow.effectDistance = new Vector2(1.5f, -1.5f);

        RectTransform rect = labelText.rectTransform;
        rect.anchorMin = new Vector2(0f, 0f);
        rect.anchorMax = new Vector2(0f, 0f);
        rect.pivot = new Vector2(0f, 0f);
        rect.anchoredPosition = new Vector2(24f, 24f);
        rect.sizeDelta = new Vector2(900f, 60f);

        labelText.gameObject.SetActive(false);
    }

    void ShowLabel()
    {
        if (labelText == null) return;

        Camera cam = cameras[currentIndex];
        ShowStatusLabel(string.Format("Camera: {0}  ({1}/{2})", cam != null ? cam.name : "---", currentIndex + 1, cameras.Count));
    }

    void ShowStatusLabel(string message)
    {
        if (labelText == null) return;

        labelText.text = message;
        labelText.gameObject.SetActive(true);
        hideLabelAt = Time.unscaledTime + labelSeconds;
    }

    void UpdateLabelVisibility()
    {
        if (labelText == null || !labelText.gameObject.activeSelf) return;
        if (Time.unscaledTime >= hideLabelAt)
            labelText.gameObject.SetActive(false);
    }
}
