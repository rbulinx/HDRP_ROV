using UnityEngine;
using UnityEngine.UI;
using UnityEngine.EventSystems;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;
using UnityEngine.SceneManagement;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.Reflection;
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.UI;
using Robalink.OculusEmulator;

#if UNITY_EDITOR
using UnityEditor;
#endif

[DefaultExecutionOrder(-950)]
public class StartupEnvironmentPresetMenu : MonoBehaviour
{
    [Header("Enable")]
    public bool showOnBoot = true;
    public bool pauseWhileMenuOpen = false;
    public bool allowToggleDuringPlay = true;

    [Header("References")]
    public CameraWaterVisibilitySelector cameraSelector;
    public ImagingSonarSim[] imagingSonars;
    public VirtualOculusM750dTerrainSonar[] virtualOculusSonars;
    public ROVGamepadThrustController[] inputControllers;
    public Transform sonarRigTransform;

    [Header("Defaults")]
    public CameraWaterVisibilitySelector.VisibilityPreset cameraDefault = CameraWaterVisibilitySelector.VisibilityPreset.Normal;
    public SonarModePreset sonarModeDefault = SonarModePreset.ImagingSonar;
    public ROVGamepadThrustController.ExternalInputDevice inputDefault = ROVGamepadThrustController.ExternalInputDevice.Gamepad;
    public SonarRigXPreset sonarRigXDefault = SonarRigXPreset.Deg20;

    [Header("Audio")]
    public AudioClip startupBgmClip;
    public AudioClip bgmClip;
    [Range(0f, 1f)] public float bgmVolume = 0.35f;
    public AudioClip clickClip;
    [Range(0f, 1f)] public float clickVolume = 0.8f;

    public enum SonarModePreset
    {
        ImagingSonar,
        VirtualOculus
    }

    public enum SonarRigXPreset
    {
        Deg0,
        Deg20,
        Deg90
    }

    public enum EnvironmentTimePreset
    {
        Day,
        Night
    }

    public enum PerformancePreset
    {
        Quality,
        Lightweight
    }

    Canvas canvas;
    GameObject titlePanel;
    GameObject sceneSelectPanel;
    GameObject settingsPanel;
    GameObject environmentPanel;
    GameObject controlsHelpPanel;
    Text startButtonText;
    Button mineSceneButton;
    Button uBoatSceneButton;
    Button underwaterStructureSceneButton;
    Button camClearButton;
    Button camNormalButton;
    Button camMurkyButton;
    Button sonarImagingButton;
    Button sonarOculusButton;
    Button inputGamepadButton;
    Button inputJoystickButton;
    Button inputKeyboardButton;
    Button sonarRig0Button;
    Button sonarRig20Button;
    Button sonarRig90Button;
    Button missionOnButton;
    Button missionOffButton;
    Button dayButton;
    Button nightButton;
    Button performanceQualityButton;
    Button performanceLightButton;
    Button environmentMenuButton;
    Button envBackButton;
    Button envClearButton;
    Button envNormalButton;
    Button envMurkyButton;
    Button envSonarNoiseOnButton;
    Button envSonarNoiseOffButton;
    Button envVisualMatterOnButton;
    Button envVisualMatterOffButton;
    Button envWorksiteDebrisOnButton;
    Button envWorksiteDebrisOffButton;
    Button envCurrentNoneButton;
    Button envCurrentWeakButton;
    Button envCurrentStrongButton;
    InputField envCurrentXInput;
    InputField envCurrentYInput;
    InputField envCurrentZInput;
    Button envDayButton;
    Button envNightButton;
    Button envBgmOnButton;
    Button envBgmOffButton;
    Button controlsHelpButton;
    Button controlsHelpCloseButton;
    Button startActionButton;
    Button sceneSelectActionButton;
    Button endActionButton;

    bool titleOpened;
    bool sceneSelectOpened;
    bool settingsOpened;
    bool hasStarted;
    bool applyPendingLaunchAfterStart;

    AudioSource bgmSource;
    AudioSource sfxSource;
    readonly Dictionary<string, AudioClip> gameBgmCache = new Dictionary<string, AudioClip>();
    readonly List<Button> sceneSelectionButtons = new List<Button>();
    readonly List<string> sceneSelectionNames = new List<string>();

    CameraWaterVisibilitySelector.VisibilityPreset selectedCamera;
    SonarModePreset selectedSonarMode;
    ROVGamepadThrustController.ExternalInputDevice selectedInput;
    SonarRigXPreset selectedSonarRigX;
    bool selectedMissionEnabled;
    EnvironmentTimePreset selectedEnvironmentTime;
    PerformancePreset selectedPerformance = PerformancePreset.Quality;
    bool performanceSelectionTouched;
    Vector3 selectedCurrentVelocity = Vector3.zero;
    bool selectedSonarNoiseEnabled = true;
    bool selectedVisualMatterEnabled = true;
    bool selectedWorksiteDebrisEnabled = true;
    bool selectedBgmEnabled = true;

    static bool globalBgmEnabled = true;

    static readonly Color ButtonNormalColor = new Color(0.12f, 0.5f, 0.7f, 0.95f);
    static readonly Color ButtonSelectedColor = new Color(0.10f, 0.70f, 0.30f, 1f);

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    static void CreateIfNeeded()
    {
        EnsureInstance();
    }

    public static void EnsureInstance()
    {
        if (SceneSelector.IsWaveEvaluationSceneActive()) return;
        if (FindFirstObjectByType<StartupEnvironmentPresetMenu>() != null) return;
        GameObject go = new GameObject("StartupEnvironmentPresetMenu");
        go.AddComponent<StartupEnvironmentPresetMenu>();
    }

    void Awake()
    {
        bool applyPendingLaunchOnAwake = false;

        EnsureMenuCamera();
        EnsureEventSystem();
        EnsureSelectorsAndAutoCreate();
        EnsureInputControllers();
        EnsureSonarRigTransform();
        EnsureAudio();

        selectedCamera = (cameraSelector != null) ? cameraSelector.currentPreset : cameraDefault;
        selectedSonarMode = DetermineCurrentSonarMode();
        selectedInput = inputDefault;
        selectedSonarRigX = ReadSonarRigPresetOrDefault();
        selectedMissionEnabled = SceneSelector.SelectedMissionEnabled;
        selectedEnvironmentTime = SceneSelector.SelectedEnvironmentTimePreset;
        selectedCurrentVelocity = ReadCurrentVelocityOrDefault();
        selectedBgmEnabled = globalBgmEnabled;

        if (!SceneSelector.IsMenuSceneActive() && SceneSelector.TryConsumePendingLaunch(out SceneSelector.LaunchConfiguration launchConfiguration))
        {
            selectedCamera = launchConfiguration.cameraPreset;
            selectedSonarMode = launchConfiguration.sonarMode;
            selectedInput = launchConfiguration.inputDevice;
            selectedSonarRigX = launchConfiguration.sonarRigXPreset;
            selectedMissionEnabled = launchConfiguration.missionEnabled;
            selectedEnvironmentTime = launchConfiguration.environmentTimePreset;
            selectedCurrentVelocity = launchConfiguration.currentVelocity;
            applyPendingLaunchOnAwake = true;
        }

        BuildUi();

        if (applyPendingLaunchOnAwake)
        {
            hasStarted = true;
            CloseAllMenus();
            applyPendingLaunchAfterStart = true;
        }
        else if (showOnBoot)
        {
            hasStarted = false;
            if (SceneSelector.IsMenuSceneActive())
                OpenTitle();
            else
                OpenSettings();
        }
        else
        {
            hasStarted = true;
            CloseAllMenus();
        }
    }

    void Start()
    {
        if (!applyPendingLaunchAfterStart) return;

        applyPendingLaunchAfterStart = false;
        ApplyCurrentSelectionsToScene();
        PlayMenuBgm();
    }

    void Update()
    {
        if (TryApplyVisibilityHotkey())
            return;

        if (titleOpened && WasTitleProceedPressedThisFrame())
        {
            PlayClickSound();
            OpenSceneSelect();
            return;
        }

        if (settingsOpened && WasSettingsSubmitPressedThisFrame())
        {
            SubmitCurrentSelection();
            return;
        }

        if (allowToggleDuringPlay && hasStarted && WasTogglePressedThisFrame())
        {
            if (settingsOpened) CloseSettings();
            else OpenSettings();
        }
    }

    bool TryApplyVisibilityHotkey()
    {
        if (Keyboard.current == null) return false;

        if (Keyboard.current.f5Key.wasPressedThisFrame)
        {
            SetCameraPreset(CameraWaterVisibilitySelector.VisibilityPreset.Clear);
            return true;
        }

        if (Keyboard.current.f6Key.wasPressedThisFrame)
        {
            SetCameraPreset(CameraWaterVisibilitySelector.VisibilityPreset.Normal);
            return true;
        }

        if (Keyboard.current.f7Key.wasPressedThisFrame)
        {
            SetCameraPreset(CameraWaterVisibilitySelector.VisibilityPreset.Murky);
            return true;
        }

        return false;
    }

    void OnDestroy()
    {
        if ((titleOpened || sceneSelectOpened || settingsOpened) && pauseWhileMenuOpen) Time.timeScale = 1f;
    }

    void OpenTitle()
    {
        titleOpened = true;
        sceneSelectOpened = false;
        settingsOpened = false;
        titlePanel.SetActive(true);
        if (sceneSelectPanel != null) sceneSelectPanel.SetActive(false);
        settingsPanel.SetActive(false);
        if (environmentPanel != null) environmentPanel.SetActive(false);
        PlayMenuBgm();
        if (pauseWhileMenuOpen) Time.timeScale = 0f;
        SelectButton(null);
    }

    void OpenSceneSelect()
    {
        titleOpened = false;
        sceneSelectOpened = true;
        settingsOpened = false;
        titlePanel.SetActive(false);
        if (sceneSelectPanel != null) sceneSelectPanel.SetActive(true);
        settingsPanel.SetActive(false);
        if (environmentPanel != null) environmentPanel.SetActive(false);
        UpdateTexts();
        if (pauseWhileMenuOpen) Time.timeScale = 0f;
        SelectButton(null);
    }

    void OpenSceneSelectFromSettings()
    {
        titleOpened = false;
        sceneSelectOpened = true;
        settingsOpened = false;
        titlePanel.SetActive(false);
        if (sceneSelectPanel != null) sceneSelectPanel.SetActive(true);
        settingsPanel.SetActive(false);
        if (environmentPanel != null) environmentPanel.SetActive(false);
        UpdateTexts();
        if (pauseWhileMenuOpen) Time.timeScale = 0f;
        SelectButton(null);
    }

    void OpenSettings()
    {
        titleOpened = false;
        sceneSelectOpened = false;
        settingsOpened = true;
        titlePanel.SetActive(false);
        if (sceneSelectPanel != null) sceneSelectPanel.SetActive(false);
        settingsPanel.SetActive(true);
        if (environmentPanel != null) environmentPanel.SetActive(false);
        if (startButtonText != null) startButtonText.text = hasStarted ? "RESUME" : "START";
        UpdateTexts();
        PlayMenuBgm();
        if (pauseWhileMenuOpen) Time.timeScale = 0f;
        SelectButton(startActionButton);
    }

    void CloseSettings()
    {
        settingsOpened = false;
        CloseControlsHelp();
        settingsPanel.SetActive(false);
        if (environmentPanel != null) environmentPanel.SetActive(false);
        if (!hasStarted)  // ゲーム開始前のみBGMを停止
            StopMenuBgm();
        if (pauseWhileMenuOpen) Time.timeScale = 1f;
        SelectButton(null);
    }

    void CloseAllMenus()
    {
        titleOpened = false;
        sceneSelectOpened = false;
        settingsOpened = false;
        if (titlePanel != null) titlePanel.SetActive(false);
        if (sceneSelectPanel != null) sceneSelectPanel.SetActive(false);
        if (settingsPanel != null) settingsPanel.SetActive(false);
        if (environmentPanel != null) environmentPanel.SetActive(false);
        CloseControlsHelp();
        if (!hasStarted)  // ゲーム開始前のみBGMを停止
            StopMenuBgm();
        if (pauseWhileMenuOpen) Time.timeScale = 1f;
        SelectButton(null);
    }

    void ApplyAndStart()
    {
        SyncCurrentVelocityFromInputs();
        if (SceneSelector.IsMenuSceneActive())
        {
            if (SceneManager.GetActiveScene().name == SceneSelector.SelectedGameSceneName)
            {
                hasStarted = true;
                ApplyCurrentSelectionsToScene();
                PlayMenuBgm();
                CloseSettings();
                return;
            }

            SceneSelector.PrepareLaunch(selectedCamera, selectedSonarMode, selectedInput, selectedSonarRigX, selectedCurrentVelocity, selectedMissionEnabled, selectedEnvironmentTime);
            SceneManager.LoadScene(SceneSelector.SelectedGameSceneName);
            return;
        }

        if (SceneManager.GetActiveScene().name != SceneSelector.SelectedGameSceneName)
        {
            SceneSelector.PrepareLaunch(selectedCamera, selectedSonarMode, selectedInput, selectedSonarRigX, selectedCurrentVelocity, selectedMissionEnabled, selectedEnvironmentTime);
            SceneManager.LoadScene(SceneSelector.SelectedGameSceneName);
            return;
        }

        hasStarted = true;
        ApplyCurrentSelectionsToScene();
        PlayMenuBgm(); // Switch from startup BGM to game BGM
        CloseSettings();
    }

    void ApplyCurrentSelectionsToScene()
    {
        EnsureSelectorsAndAutoCreate();
        EnsureInputControllers();

        if (cameraSelector != null)
            cameraSelector.SetPresetCameraOnly(selectedCamera);
        else
            Debug.LogWarning("[EnvMenu] cameraSelector is null. Camera preset was not applied.");

        ApplySonarModeSelection();

        ApplyInputSelectionToControllers();
        ApplySonarRigXPreset(selectedSonarRigX);
        ApplyCurrentVelocityToScene();
        ApplyMissionSelectionToScene();
        ApplyEnvironmentTimeSelectionToScene();
        if (selectedPerformance == PerformancePreset.Lightweight || performanceSelectionTouched)
            ApplyPerformancePresetToScene();
    }

    void EndGame()
    {
        if (pauseWhileMenuOpen) Time.timeScale = 1f;
#if UNITY_EDITOR
        EditorApplication.isPlaying = false;
#else
        Application.Quit();
#endif
    }

    void SetCameraPreset(CameraWaterVisibilitySelector.VisibilityPreset preset)
    {
        selectedCamera = preset;
        EnsureSelectorsAndAutoCreate();
        if (cameraSelector != null) cameraSelector.SetPresetCameraOnly(selectedCamera);
        UpdateTexts();
    }

    void SetSonarMode(SonarModePreset preset)
    {
        selectedSonarMode = preset;
        EnsureSelectorsAndAutoCreate();
        ApplySonarModeSelection();
        UpdateTexts();
    }

    void SetInputDevice(ROVGamepadThrustController.ExternalInputDevice device)
    {
        selectedInput = device;
        EnsureInputControllers();
        ApplyInputSelectionToControllers();
        UpdateTexts();
        if (selectedInput == ROVGamepadThrustController.ExternalInputDevice.Keyboard)
            OpenControlsHelp();
        else
            CloseControlsHelp();
    }

    void SetSonarRigXPreset(SonarRigXPreset preset)
    {
        selectedSonarRigX = preset;
        ApplySonarRigXPreset(selectedSonarRigX);
        UpdateTexts();
    }

    void SetMissionEnabled(bool enabled)
    {
        selectedMissionEnabled = enabled;
        ApplyMissionSelectionToScene();
        UpdateTexts();
    }

    void SetEnvironmentTime(EnvironmentTimePreset preset)
    {
        selectedEnvironmentTime = preset;
        ApplyEnvironmentTimeSelectionToScene();
        UpdateTexts();
    }

    void SetPerformancePreset(PerformancePreset preset)
    {
        selectedPerformance = preset;
        performanceSelectionTouched = true;
        ApplyPerformancePresetToScene();
        UpdateTexts();
    }

    void SetCurrentVelocityComponent(int axis, string value)
    {
        if (!TryParseFloat(value, out float parsed)) return;

        if (axis == 0) selectedCurrentVelocity.x = parsed;
        else if (axis == 1) selectedCurrentVelocity.y = parsed;
        else if (axis == 2) selectedCurrentVelocity.z = parsed;

        ApplyCurrentVelocityToScene();
    }

    void UpdateTexts()
    {
        UpdateButtonHighlights();
        UpdateCurrentInputTexts();
    }

    void EnsureSelectorsAndAutoCreate()
    {
        bool isMenuScene = SceneSelector.IsMenuSceneActive();

        if (cameraSelector == null)
            cameraSelector = FindFirstObjectByType<CameraWaterVisibilitySelector>();
        if (imagingSonars == null || imagingSonars.Length == 0)
            imagingSonars = FindObjectsByType<ImagingSonarSim>(FindObjectsInactive.Exclude, FindObjectsSortMode.None);
        if (virtualOculusSonars == null || virtualOculusSonars.Length == 0)
            virtualOculusSonars = FindObjectsByType<VirtualOculusM750dTerrainSonar>(FindObjectsInactive.Exclude, FindObjectsSortMode.None);

        if (cameraSelector == null)
        {
            GameObject go = new GameObject("CameraWaterVisibilitySelector_Auto");
            cameraSelector = go.AddComponent<CameraWaterVisibilitySelector>();
            cameraSelector.applyOnStart = false;
            cameraSelector.enableHotkeys = false;
        }

        if (cameraSelector != null && cameraSelector.targetWaterSurface == null)
        {
            cameraSelector.targetWaterSurface = FindFirstObjectByType<WaterSurface>();
            if (cameraSelector.targetWaterSurface != null)
            {
                WaveObservationLogger.EnsureInstance();
            }
            else if (!isMenuScene)
            {
                Debug.LogWarning("[EnvMenu] WaterSurface not found. Camera ocean preset may not change.");
            }
        }
    }

    void EnsureInputControllers()
    {
        if (inputControllers != null && inputControllers.Length > 0) return;
        inputControllers = FindObjectsByType<ROVGamepadThrustController>(FindObjectsInactive.Exclude, FindObjectsSortMode.None);
    }

    void EnsureSonarRigTransform()
    {
        if (sonarRigTransform != null) return;

        ImagingSonarSim sonarSim = FindFirstObjectByType<ImagingSonarSim>();
        if (sonarSim != null)
        {
            sonarRigTransform = sonarSim.transform;
            return;
        }

        VirtualOculusM750dTerrainSonar oculus = FindFirstObjectByType<VirtualOculusM750dTerrainSonar>();
        if (oculus != null)
        {
            sonarRigTransform = oculus.transform;
            return;
        }

        SonarRigGamepadController sonarRig = FindFirstObjectByType<SonarRigGamepadController>();
        if (sonarRig != null) sonarRigTransform = sonarRig.transform;
    }

    SonarRigXPreset ReadSonarRigPresetOrDefault()
    {
        EnsureSonarRigTransform();
        if (sonarRigTransform == null) return sonarRigXDefault;

        float x = NormalizeAngle(sonarRigTransform.localEulerAngles.x);
        if (Mathf.Abs(x - 0f) <= 3f) return SonarRigXPreset.Deg0;
        if (Mathf.Abs(x - 20f) <= 3f) return SonarRigXPreset.Deg20;
        if (Mathf.Abs(x - 90f) <= 3f) return SonarRigXPreset.Deg90;
        return sonarRigXDefault;
    }

    Vector3 ReadCurrentVelocityOrDefault()
    {
        EnsureInputControllers();
        if (inputControllers != null)
        {
            for (int i = 0; i < inputControllers.Length; i++)
            {
                if (inputControllers[i] != null)
                    return inputControllers[i].currentVelocity;
            }
        }

        MonoBehaviour[] behaviours = FindObjectsByType<MonoBehaviour>(FindObjectsInactive.Exclude, FindObjectsSortMode.None);
        for (int i = 0; i < behaviours.Length; i++)
        {
            MonoBehaviour behaviour = behaviours[i];
            if (behaviour == null) continue;

            FieldInfo field = behaviour.GetType().GetField("currentVelocity", BindingFlags.Instance | BindingFlags.Public);
            if (field != null && field.FieldType == typeof(Vector3))
                return (Vector3)field.GetValue(behaviour);
        }

        return Vector3.zero;
    }

    void ApplySonarRigXPreset(SonarRigXPreset preset)
    {
        EnsureSonarRigTransform();
        if (sonarRigTransform == null) return;

        float x = SonarRigPresetToDegrees(preset);
        Vector3 e = sonarRigTransform.localEulerAngles;
        e.x = x;
        sonarRigTransform.localEulerAngles = e;
    }

    void ApplyCurrentVelocityToScene()
    {
        MonoBehaviour[] behaviours = FindObjectsByType<MonoBehaviour>(FindObjectsInactive.Exclude, FindObjectsSortMode.None);
        int applied = 0;

        for (int i = 0; i < behaviours.Length; i++)
        {
            MonoBehaviour behaviour = behaviours[i];
            if (behaviour == null) continue;

            FieldInfo field = behaviour.GetType().GetField("currentVelocity", BindingFlags.Instance | BindingFlags.Public);
            if (field == null || field.FieldType != typeof(Vector3)) continue;

            field.SetValue(behaviour, selectedCurrentVelocity);
            applied++;
        }

    }

    void ApplyMissionSelectionToScene()
    {
        SceneSelector.SetMissionEnabled(selectedMissionEnabled);

        if (SceneManager.GetActiveScene().name != SceneSelector.UBoatSceneName)
            return;

        if (selectedMissionEnabled)
            CableClearMission.EnsureInstance();
        else
            CableClearMission.RemoveInstances();
    }

    void ApplyEnvironmentTimeSelectionToScene()
    {
        SceneSelector.SetEnvironmentTimePreset(selectedEnvironmentTime);
        if (SceneSelector.IsMenuSceneActive()) return;

        bool night = selectedEnvironmentTime == EnvironmentTimePreset.Night;
        RenderSettings.ambientLight = night
            ? new Color(0.0015f, 0.003f, 0.008f, 1f)
            : new Color(0.212f, 0.227f, 0.259f, 1f);
        RenderSettings.fog = true;
        RenderSettings.fogColor = night
            ? new Color(0.001f, 0.003f, 0.01f, 1f)
            : new Color(0.5f, 0.5f, 0.5f, 1f);
        RenderSettings.fogDensity = night ? 0.22f : 0.119f;

        ApplyEnvironmentVolumePreset(night);

        Light[] lights = FindObjectsByType<Light>(FindObjectsInactive.Exclude, FindObjectsSortMode.None);
        for (int i = 0; i < lights.Length; i++)
        {
            Light lightComponent = lights[i];
            if (lightComponent == null || lightComponent.type != LightType.Directional) continue;

            lightComponent.color = night
                ? new Color(0.12f, 0.18f, 0.34f, 1f)
                : new Color(0.507f, 0.647f, 0.774f, 1f);
            lightComponent.intensity = night ? 0.1f : 91672f;
            lightComponent.transform.rotation = night
                ? Quaternion.Euler(12f, -35f, 0f)
                : Quaternion.Euler(55f, -35f, 0f);

            HDAdditionalLightData hdLight = lightComponent.GetComponent<HDAdditionalLightData>();
            if (hdLight != null)
            {
                hdLight.lightDimmer = night ? 0.002f : 1f;
                hdLight.volumetricDimmer = night ? 0.01f : 1f;
            }
        }

        EnsureInputControllers();
        if (inputControllers == null) return;
        for (int i = 0; i < inputControllers.Length; i++)
        {
            if (inputControllers[i] == null) continue;
            inputControllers[i].SetLightIntensity(night ? 14000f : 8000f);
        }

        if (!night && cameraSelector != null)
            cameraSelector.SetPresetCameraOnly(selectedCamera);
    }

    void ApplyPerformancePresetToScene()
    {
        bool light = selectedPerformance == PerformancePreset.Lightweight;

        ImagingSonarSim[] sonars = FindObjectsByType<ImagingSonarSim>(FindObjectsInactive.Include, FindObjectsSortMode.None);
        for (int i = 0; i < sonars.Length; i++)
            ApplyImagingSonarPerformance(sonars[i], light);

        VirtualOculusM750dTerrainSonar[] oculusSonars = FindObjectsByType<VirtualOculusM750dTerrainSonar>(FindObjectsInactive.Include, FindObjectsSortMode.None);
        for (int i = 0; i < oculusSonars.Length; i++)
            ApplyOculusSonarPerformance(oculusSonars[i], light);

        MainCameraSwitcher[] switchers = FindObjectsByType<MainCameraSwitcher>(FindObjectsInactive.Include, FindObjectsSortMode.None);
        for (int i = 0; i < switchers.Length; i++)
            ApplyCameraPerformance(switchers[i], light);

        UnderwaterSuspendedParticleField[] particleFields = FindObjectsByType<UnderwaterSuspendedParticleField>(FindObjectsInactive.Include, FindObjectsSortMode.None);
        for (int i = 0; i < particleFields.Length; i++)
            ApplyParticlePerformance(particleFields[i], light);

        UnderwaterWorksiteDebrisField[] debrisFields = FindObjectsByType<UnderwaterWorksiteDebrisField>(FindObjectsInactive.Include, FindObjectsSortMode.None);
        for (int i = 0; i < debrisFields.Length; i++)
            ApplyWorksiteDebrisPerformance(debrisFields[i], light);
    }

    static void ApplyImagingSonarPerformance(ImagingSonarSim sonar, bool light)
    {
        if (sonar == null) return;

        if (light)
        {
            sonar.scanHz = Mathf.Min(sonar.scanHz, 6f);
            sonar.beams = Mathf.Min(sonar.beams, 192);
            sonar.elevSamples = Mathf.Min(sonar.elevSamples, 4);
            sonar.mapResolution = 512;
            sonar.pointSize = 2;
            sonar.enableShadow = false;
            if (sonar.waterColumnNoise != null)
            {
                sonar.waterColumnNoise.processBeamStride = Mathf.Max(sonar.waterColumnNoise.processBeamStride, 2);
                sonar.waterColumnNoise.processRangeStride = Mathf.Max(sonar.waterColumnNoise.processRangeStride, 2);
                sonar.waterColumnNoise.maxFalseEchoesPerRay = Mathf.Min(sonar.waterColumnNoise.maxFalseEchoesPerRay, 2);
            }
        }
        else
        {
            sonar.scanHz = Mathf.Max(sonar.scanHz, 15f);
            sonar.beams = Mathf.Max(sonar.beams, 512);
            sonar.elevSamples = Mathf.Max(sonar.elevSamples, 10);
            sonar.mapResolution = 512;
            sonar.pointSize = 1;
            sonar.enableShadow = true;
            if (sonar.waterColumnNoise != null)
            {
                sonar.waterColumnNoise.processBeamStride = 1;
                sonar.waterColumnNoise.processRangeStride = 1;
                sonar.waterColumnNoise.maxFalseEchoesPerRay = Mathf.Max(sonar.waterColumnNoise.maxFalseEchoesPerRay, 5);
            }
        }

        sonar.SetMaxRangeMeters(sonar.maxRangeMeters);
    }

    static void ApplyOculusSonarPerformance(VirtualOculusM750dTerrainSonar sonar, bool light)
    {
        if (sonar == null) return;

        if (light)
        {
            sonar.beamCount = Mathf.Min(sonar.beamCount, 128);
            sonar.rangeCount = Mathf.Min(sonar.rangeCount, 512);
            sonar.verticalSamplesPerBeam = Mathf.Min(sonar.verticalSamplesPerBeam, 4);
            sonar.maxEchoesPerBeam = Mathf.Min(sonar.maxEchoesPerBeam, 2);
            sonar.useSphereCast = false;
            sonar.pingRate = PingRateType.Low;
            if (sonar.waterColumnNoise != null)
            {
                sonar.waterColumnNoise.processBeamStride = Mathf.Max(sonar.waterColumnNoise.processBeamStride, 2);
                sonar.waterColumnNoise.processRangeStride = Mathf.Max(sonar.waterColumnNoise.processRangeStride, 2);
                sonar.waterColumnNoise.maxFalseEchoesPerRay = Mathf.Min(sonar.waterColumnNoise.maxFalseEchoesPerRay, 2);
            }
        }
        else
        {
            sonar.beamCount = Mathf.Max(sonar.beamCount, 256);
            sonar.rangeCount = Mathf.Max(sonar.rangeCount, 1024);
            sonar.verticalSamplesPerBeam = Mathf.Max(sonar.verticalSamplesPerBeam, 12);
            sonar.maxEchoesPerBeam = Mathf.Max(sonar.maxEchoesPerBeam, 3);
            sonar.useSphereCast = true;
            sonar.pingRate = PingRateType.Normal;
            if (sonar.waterColumnNoise != null)
            {
                sonar.waterColumnNoise.processBeamStride = 1;
                sonar.waterColumnNoise.processRangeStride = 1;
                sonar.waterColumnNoise.maxFalseEchoesPerRay = Mathf.Max(sonar.waterColumnNoise.maxFalseEchoesPerRay, 5);
            }
        }
    }

    static void ApplyCameraPerformance(MainCameraSwitcher switcher, bool light)
    {
        if (switcher == null) return;

        if (light)
        {
            switcher.underwaterParticleMaxCount = Mathf.Min(switcher.underwaterParticleMaxCount, 350);
            switcher.underwaterParticleEmissionRate = Mathf.Min(switcher.underwaterParticleEmissionRate, 45f);
            switcher.sonarMaxFalseEchoesPerRay = Mathf.Min(switcher.sonarMaxFalseEchoesPerRay, 2);
            switcher.cameraImageDelayBufferFps = Mathf.Min(switcher.cameraImageDelayBufferFps, 15);
            switcher.cameraImageDelayRenderScale = Mathf.Min(switcher.cameraImageDelayRenderScale, 0.5f);
        }
        else
        {
            switcher.underwaterParticleMaxCount = Mathf.Max(switcher.underwaterParticleMaxCount, 1200);
            switcher.underwaterParticleEmissionRate = Mathf.Max(switcher.underwaterParticleEmissionRate, 140f);
            switcher.sonarMaxFalseEchoesPerRay = Mathf.Max(switcher.sonarMaxFalseEchoesPerRay, 5);
            switcher.cameraImageDelayBufferFps = Mathf.Max(switcher.cameraImageDelayBufferFps, 30);
            switcher.cameraImageDelayRenderScale = Mathf.Max(switcher.cameraImageDelayRenderScale, 0.75f);
        }
    }

    static void ApplyParticlePerformance(UnderwaterSuspendedParticleField field, bool light)
    {
        if (field == null) return;

        if (light)
        {
            field.maxParticles = Mathf.Min(field.maxParticles, 350);
            field.emissionRate = Mathf.Min(field.emissionRate, 45f);
        }
        else
        {
            field.maxParticles = Mathf.Max(field.maxParticles, 1200);
            field.emissionRate = Mathf.Max(field.emissionRate, 140f);
        }
    }

    static void ApplyWorksiteDebrisPerformance(UnderwaterWorksiteDebrisField field, bool light)
    {
        if (field == null) return;

        if (light)
        {
            field.sonarUsesIndividualDebris = false;
            field.sonarEchoDensityPerMeter = Mathf.Min(field.sonarEchoDensityPerMeter, 1.5f);
            field.maxSonarEchoesPerRay = Mathf.Min(field.maxSonarEchoesPerRay, 2);
            field.animateInPlayMode = false;
        }
        else
        {
            field.sonarUsesIndividualDebris = true;
            field.sonarEchoDensityPerMeter = Mathf.Max(field.sonarEchoDensityPerMeter, 5f);
            field.maxSonarEchoesPerRay = Mathf.Max(field.maxSonarEchoesPerRay, 8);
            field.animateInPlayMode = true;
        }
    }

    void ApplyEnvironmentVolumePreset(bool night)
    {
        Volume volume = FindFirstObjectByType<Volume>();
        if (volume == null)
        {
            GameObject go = new GameObject("EnvironmentTimeVolume");
            volume = go.AddComponent<Volume>();
            volume.isGlobal = true;
            volume.priority = 100f;
            volume.profile = ScriptableObject.CreateInstance<VolumeProfile>();
        }

        VolumeProfile profile = volume.profile;
        if (profile == null)
        {
            profile = ScriptableObject.CreateInstance<VolumeProfile>();
            volume.profile = profile;
        }

        if (!profile.TryGet(out ColorAdjustments colorAdjustments))
            colorAdjustments = profile.Add<ColorAdjustments>(true);

        colorAdjustments.active = night;
        colorAdjustments.postExposure.Override(night ? -3.0f : 0f);
        colorAdjustments.contrast.Override(night ? 4f : 0f);
        colorAdjustments.saturation.Override(night ? -25f : 0f);
        colorAdjustments.colorFilter.Override(night ? new Color(0.42f, 0.53f, 0.82f, 1f) : Color.white);

        if (profile.TryGet(out Exposure exposure))
            exposure.active = false;

        if (!profile.TryGet(out Fog fog))
            fog = profile.Add<Fog>(true);

        fog.active = night;
        fog.enabled.Override(night);
        fog.meanFreePath.Override(night ? 38f : 120f);
        fog.baseHeight.Override(-40f);
        fog.maximumHeight.Override(night ? 25f : 60f);

        WaterSurface waterSurface = FindFirstObjectByType<WaterSurface>();
        if (waterSurface != null)
        {
            if (night)
                waterSurface.absorptionDistanceMultiplier = Mathf.Max(waterSurface.absorptionDistanceMultiplier, 2.0f);
            else if (cameraSelector != null)
                cameraSelector.targetWaterSurface = waterSurface;
        }
    }

    static float SonarRigPresetToDegrees(SonarRigXPreset preset)
    {
        switch (preset)
        {
            case SonarRigXPreset.Deg0: return 0f;
            case SonarRigXPreset.Deg20: return 20f;
            case SonarRigXPreset.Deg90: return 90f;
            default: return 20f;
        }
    }

    static float NormalizeAngle(float deg)
    {
        while (deg > 180f) deg -= 360f;
        while (deg < -180f) deg += 360f;
        return deg;
    }

    SonarModePreset DetermineCurrentSonarMode()
    {
        EnsureSelectorsAndAutoCreate();

        if (virtualOculusSonars != null)
        {
            for (int i = 0; i < virtualOculusSonars.Length; i++)
            {
                if (virtualOculusSonars[i] != null && virtualOculusSonars[i].enabled)
                    return SonarModePreset.VirtualOculus;
            }
        }

        return sonarModeDefault;
    }

    void ApplySonarModeSelection()
    {
        EnsureSelectorsAndAutoCreate();

        bool useImaging = selectedSonarMode == SonarModePreset.ImagingSonar;
        bool useVirtualOculus = selectedSonarMode == SonarModePreset.VirtualOculus;

        if (imagingSonars != null)
        {
            for (int i = 0; i < imagingSonars.Length; i++)
            {
                ImagingSonarSim sonar = imagingSonars[i];
                if (sonar == null) continue;
                if (sonar.targetImage != null)
                {
                    Canvas parentCanvas = sonar.targetImage.GetComponentInParent<Canvas>(true);
                    if (parentCanvas != null)
                        parentCanvas.gameObject.SetActive(useImaging);
                    sonar.targetImage.gameObject.SetActive(useImaging);
                    sonar.targetImage.enabled = useImaging;
                }

                sonar.enabled = false;
                sonar.enabled = useImaging;
            }
        }

        if (virtualOculusSonars != null)
        {
            for (int i = 0; i < virtualOculusSonars.Length; i++)
            {
                VirtualOculusM750dTerrainSonar sonar = virtualOculusSonars[i];
                if (sonar == null) continue;
                sonar.enabled = useVirtualOculus;
            }
        }
    }

    void ApplyInputSelectionToControllers()
    {
        if (inputControllers == null || inputControllers.Length == 0) return;

        for (int i = 0; i < inputControllers.Length; i++)
        {
            ROVGamepadThrustController c = inputControllers[i];
            if (c == null) continue;
            c.ApplyMenuInputSelection(selectedInput);
        }
    }

    void BuildUi()
    {
        canvas = gameObject.AddComponent<Canvas>();
        canvas.renderMode = RenderMode.ScreenSpaceOverlay;
        canvas.sortingOrder = 2200;

        var scaler = gameObject.AddComponent<CanvasScaler>();
        scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
        gameObject.AddComponent<GraphicRaycaster>();

        BuildTitlePanel();
        BuildSceneSelectPanel();
        BuildSettingsPanel();
        BuildEnvironmentPanel();
        BuildControlsHelpPanel();
        CloseAllMenus();
    }

    void EnsureAudio()
    {
        if (startupBgmClip == null)
        {
            startupBgmClip = Resources.Load<AudioClip>("Audio/Menu/StartupBgm");
            if (startupBgmClip == null)
                Debug.LogWarning("[EnvMenu] Failed to load startupBGM from Audio/Menu/StartupBgm");
        }
        if (bgmClip == null)
        {
            bgmClip = Resources.Load<AudioClip>("Audio/Menu/water_tunnel");
            if (bgmClip == null)
                Debug.LogWarning("[EnvMenu] Failed to load water_tunnel from Audio/Menu/water_tunnel");
        }
        if (clickClip == null)
        {
            clickClip = Resources.Load<AudioClip>("Audio/Menu/click");
            if (clickClip == null)
                Debug.LogWarning("[EnvMenu] Failed to load click from Audio/Menu/click");
        }

        bgmSource = gameObject.AddComponent<AudioSource>();
        bgmSource.playOnAwake = false;
        bgmSource.loop = true;
        bgmSource.volume = bgmVolume;
        bgmSource.spatialBlend = 0f; // 2D音声に設定

        sfxSource = gameObject.AddComponent<AudioSource>();
        sfxSource.playOnAwake = false;
        sfxSource.loop = false;
        sfxSource.volume = clickVolume;
        sfxSource.spatialBlend = 0f; // 2D音声に設定

    }

    void PlayMenuBgm()
    {
        if (bgmSource == null) 
        {
            Debug.LogWarning("[EnvMenu] bgmSource is null");
            return;
        }

        if (!selectedBgmEnabled)
        {
            StopMenuBgm();
            return;
        }

        AudioClip clipToPlay = hasStarted ? ResolveGameBgmClip() : startupBgmClip;
        if (clipToPlay == null) 
        {
            Debug.LogWarning("[EnvMenu] No BGM clip to play. hasStarted=" + hasStarted);
            return;
        }

        bgmSource.volume = bgmVolume;
        bool clipChanged = bgmSource.clip != clipToPlay;
        if (clipChanged)
        {
            if (bgmSource.isPlaying)
                bgmSource.Stop();
            bgmSource.clip = clipToPlay;
        }
        if (!bgmSource.isPlaying)
        {
            bgmSource.Play();
        }
    }

    AudioClip ResolveGameBgmClip()
    {
        string sceneName = SceneManager.GetActiveScene().name;
        if (string.IsNullOrWhiteSpace(sceneName))
            return bgmClip;

        if (gameBgmCache.TryGetValue(sceneName, out AudioClip cachedClip))
            return cachedClip != null ? cachedClip : bgmClip;

        AudioClip clip =
            Resources.Load<AudioClip>("Audio/Menu/" + sceneName) ??
            Resources.Load<AudioClip>("Audio/Menu/Bgm_" + sceneName) ??
            Resources.Load<AudioClip>("Audio/Menu/Bgm_" + sceneName.ToLowerInvariant());

        gameBgmCache[sceneName] = clip;
        return clip != null ? clip : bgmClip;
    }

    void StopMenuBgm()
    {
        if (bgmSource != null && bgmSource.isPlaying)
            bgmSource.Stop();
    }

    void SetBgmEnabled(bool enabled)
    {
        selectedBgmEnabled = enabled;
        globalBgmEnabled = enabled;

        if (enabled)
            PlayMenuBgm();
        else
            StopMenuBgm();

        UpdateButtonHighlights();
    }

    void PlayClickSound()
    {
        if (sfxSource == null) 
        {
            Debug.LogWarning("[EnvMenu] sfxSource is null");
            return;
        }
        if (clickClip == null) 
        {
            Debug.LogWarning("[EnvMenu] clickClip is null");
            return;
        }
        
        sfxSource.volume = clickVolume;
        sfxSource.PlayOneShot(clickClip);
    }

    void AddClickSound(Button button)
    {
        if (button == null) return;
        button.onClick.AddListener(PlayClickSound);
    }

    void BuildTitlePanel()
    {
        titlePanel = CreateUiObject("TitlePanel", transform);
        StretchRect(titlePanel.GetComponent<RectTransform>());
        RawImage bg = CreateGradientBackground(titlePanel.transform);
        StretchRect(bg.rectTransform);

        var title = CreateText("Title", titlePanel.transform, 42, FontStyle.Bold);
        title.text = "ROV de GO";
        SetRect(title.rectTransform, new Vector2(0.5f, 0.62f), new Vector2(600f, 70f));

        var hint = CreateText("Hint", titlePanel.transform, 22, FontStyle.Normal);
        hint.text = "Click / Enter / A to select scene";
        SetRect(hint.rectTransform, new Vector2(0.5f, 0.50f), new Vector2(760f, 50f));
    }

    void BuildSceneSelectPanel()
    {
        sceneSelectPanel = CreateUiObject("SceneSelectPanel", transform);
        var panelImage = sceneSelectPanel.AddComponent<Image>();
        panelImage.color = new Color(0f, 0.05f, 0.12f, 0.92f);
        StretchRect(sceneSelectPanel.GetComponent<RectTransform>());

        GameObject content = CreateUiObject("ContentPanel", sceneSelectPanel.transform);
        var contentImage = content.AddComponent<Image>();
        contentImage.color = new Color(0.03f, 0.10f, 0.20f, 0.60f);

        RectTransform contentRect = content.GetComponent<RectTransform>();
        contentRect.anchorMin = new Vector2(0.5f, 0.5f);
        contentRect.anchorMax = new Vector2(0.5f, 0.5f);
        contentRect.pivot = new Vector2(0.5f, 0.5f);
        contentRect.anchoredPosition = new Vector2(0f, 0f);
        List<string> gameScenes = SceneSelector.GetAvailableGameSceneNames();
        int sceneCount = Mathf.Max(1, gameScenes.Count);
        contentRect.sizeDelta = new Vector2(620f, Mathf.Max(340f, 210f + sceneCount * 55f));

        var title = CreateText("Title", content.transform, 24, FontStyle.Bold);
        title.text = "Scene Select";
        SetRect(title.rectTransform, new Vector2(0.5f, 0.82f), new Vector2(360f, 40f));

        var hint = CreateText("Hint", content.transform, 16, FontStyle.Normal);
        hint.text = "Choose the game scene";
        SetRect(hint.rectTransform, new Vector2(0.5f, 0.68f), new Vector2(400f, 30f));

        sceneSelectionButtons.Clear();
        sceneSelectionNames.Clear();

        if (gameScenes.Count == 0)
        {
            var emptyText = CreateText("EmptyScenes", content.transform, 15, FontStyle.Normal);
            emptyText.text = "No game scenes found in Assets/Scenes";
            SetRect(emptyText.rectTransform, new Vector2(0.5f, 0.48f), new Vector2(460f, 36f));
            return;
        }

        float firstRow = 30f;
        for (int i = 0; i < gameScenes.Count; i++)
        {
            string sceneName = gameScenes[i];
            Button sceneButton = CreatePresetButton(content.transform, "SceneButton_" + SanitizeObjectName(sceneName), new Vector2(0f, firstRow - i * 55f), sceneName,
                () => SelectSceneAndOpenSettings(sceneName));
            sceneButton.GetComponent<RectTransform>().sizeDelta = new Vector2(Mathf.Clamp(120f + sceneName.Length * 8f, 180f, 360f), 44f);
            AddClickSound(sceneButton);
            sceneSelectionButtons.Add(sceneButton);
            sceneSelectionNames.Add(sceneName);
        }
    }

    void BuildSettingsPanel()
    {
        settingsPanel = CreateUiObject("SettingsPanel", transform);
        var panelImage = settingsPanel.AddComponent<Image>();
        panelImage.color = new Color(0f, 0.05f, 0.12f, 0.92f);
        StretchRect(settingsPanel.GetComponent<RectTransform>());

        GameObject content = CreateUiObject("ContentPanel", settingsPanel.transform);
        var contentImage = content.AddComponent<Image>();
        contentImage.color = new Color(0.03f, 0.10f, 0.20f, 0.60f);
        RectTransform contentRect = content.GetComponent<RectTransform>();
        contentRect.anchorMin = new Vector2(0.5f, 0.5f);
        contentRect.anchorMax = new Vector2(0.5f, 0.5f);
        contentRect.pivot = new Vector2(0.5f, 0.5f);
        contentRect.anchoredPosition = new Vector2(0f, 0f);
        contentRect.sizeDelta = new Vector2(600f, 480f);

        float colL = -155f;
        float colC = 0f;
        float colR = 155f;
        float row1 = 145f;
        float row2 = 102f;
        float row3 = 59f;
        float row4 = 16f;
        float row5 = -27f;
        float row6 = -70f;
        float row7 = -113f;
        float row8 = -156f;
        Vector2 presetSize = new Vector2(124f, 28f);

        var title = CreateText("Title", content.transform, 20, FontStyle.Bold);
        title.text = "Settings";
        SetRect(title.rectTransform, new Vector2(0.5f, 0.92f), new Vector2(320f, 34f));

        camClearButton = CreatePresetButton(content.transform, "CamClear", new Vector2(colL, row1), "Camera Clear",
            () => SetCameraPreset(CameraWaterVisibilitySelector.VisibilityPreset.Clear));
        camNormalButton = CreatePresetButton(content.transform, "CamNormal", new Vector2(colC, row1), "Camera Normal",
            () => SetCameraPreset(CameraWaterVisibilitySelector.VisibilityPreset.Normal));
        camMurkyButton = CreatePresetButton(content.transform, "CamMurky", new Vector2(colR, row1), "Camera Murky",
            () => SetCameraPreset(CameraWaterVisibilitySelector.VisibilityPreset.Murky));
        camClearButton.GetComponent<RectTransform>().sizeDelta = presetSize;
        camNormalButton.GetComponent<RectTransform>().sizeDelta = presetSize;
        camMurkyButton.GetComponent<RectTransform>().sizeDelta = presetSize;
        AddClickSound(camClearButton);
        AddClickSound(camNormalButton);
        AddClickSound(camMurkyButton);

        sonarImagingButton = CreatePresetButton(content.transform, "SonarImaging", new Vector2(-80f, row2), "Imaging sonar sim",
            () => SetSonarMode(SonarModePreset.ImagingSonar));
        sonarOculusButton = CreatePresetButton(content.transform, "SonarOculus", new Vector2(80f, row2), "Virtual Oculus",
            () => SetSonarMode(SonarModePreset.VirtualOculus));
        sonarImagingButton.GetComponent<RectTransform>().sizeDelta = new Vector2(130f, 28f);
        sonarOculusButton.GetComponent<RectTransform>().sizeDelta = new Vector2(130f, 28f);
        AddClickSound(sonarImagingButton);
        AddClickSound(sonarOculusButton);

        inputGamepadButton = CreatePresetButton(content.transform, "InputGamepad", new Vector2(colL, row3), "Input Gamepad",
            () => SetInputDevice(ROVGamepadThrustController.ExternalInputDevice.Gamepad));
        inputJoystickButton = CreatePresetButton(content.transform, "InputJoystick", new Vector2(colC, row3), "Input Joystick",
            () => SetInputDevice(ROVGamepadThrustController.ExternalInputDevice.Joystick));
        inputKeyboardButton = CreatePresetButton(content.transform, "InputKeyboard", new Vector2(colR, row3), "Input Keyboard",
            () => SetInputDevice(ROVGamepadThrustController.ExternalInputDevice.Keyboard));
        inputGamepadButton.GetComponent<RectTransform>().sizeDelta = presetSize;
        inputJoystickButton.GetComponent<RectTransform>().sizeDelta = presetSize;
        inputKeyboardButton.GetComponent<RectTransform>().sizeDelta = presetSize;
        AddClickSound(inputGamepadButton);
        AddClickSound(inputJoystickButton);
        AddClickSound(inputKeyboardButton);

        sonarRig0Button = CreatePresetButton(content.transform, "SonarRigX0", new Vector2(colL, row4), "Sonar X 0°",
            () => SetSonarRigXPreset(SonarRigXPreset.Deg0));
        sonarRig20Button = CreatePresetButton(content.transform, "SonarRigX20", new Vector2(colC, row4), "Sonar X 20°",
            () => SetSonarRigXPreset(SonarRigXPreset.Deg20));
        sonarRig90Button = CreatePresetButton(content.transform, "SonarRigX90", new Vector2(colR, row4), "Sonar X 90°",
            () => SetSonarRigXPreset(SonarRigXPreset.Deg90));
        sonarRig0Button.GetComponent<RectTransform>().sizeDelta = presetSize;
        sonarRig20Button.GetComponent<RectTransform>().sizeDelta = presetSize;
        sonarRig90Button.GetComponent<RectTransform>().sizeDelta = presetSize;
        AddClickSound(sonarRig0Button);
        AddClickSound(sonarRig20Button);
        AddClickSound(sonarRig90Button);

        missionOnButton = CreatePresetButton(content.transform, "MissionOn", new Vector2(-80f, row5), "Mission ON",
            () => SetMissionEnabled(true));
        missionOffButton = CreatePresetButton(content.transform, "MissionOff", new Vector2(80f, row5), "Mission OFF",
            () => SetMissionEnabled(false));
        missionOnButton.GetComponent<RectTransform>().sizeDelta = new Vector2(130f, 28f);
        missionOffButton.GetComponent<RectTransform>().sizeDelta = new Vector2(130f, 28f);
        AddClickSound(missionOnButton);
        AddClickSound(missionOffButton);

        dayButton = CreatePresetButton(content.transform, "DayButton", new Vector2(-80f, row6), "Day",
            () => SetEnvironmentTime(EnvironmentTimePreset.Day));
        nightButton = CreatePresetButton(content.transform, "NightButton", new Vector2(80f, row6), "Night",
            () => SetEnvironmentTime(EnvironmentTimePreset.Night));
        dayButton.GetComponent<RectTransform>().sizeDelta = new Vector2(130f, 28f);
        nightButton.GetComponent<RectTransform>().sizeDelta = new Vector2(130f, 28f);
        AddClickSound(dayButton);
        AddClickSound(nightButton);

        performanceQualityButton = CreatePresetButton(content.transform, "PerformanceQuality", new Vector2(-80f, row7), "Quality",
            () => SetPerformancePreset(PerformancePreset.Quality));
        performanceLightButton = CreatePresetButton(content.transform, "PerformanceLight", new Vector2(80f, row7), "Light",
            () => SetPerformancePreset(PerformancePreset.Lightweight));
        performanceQualityButton.GetComponent<RectTransform>().sizeDelta = new Vector2(130f, 28f);
        performanceLightButton.GetComponent<RectTransform>().sizeDelta = new Vector2(130f, 28f);
        AddClickSound(performanceQualityButton);
        AddClickSound(performanceLightButton);

        environmentMenuButton = CreatePresetButton(content.transform, "EnvironmentMenuButton", new Vector2(-80f, row8), "ENV MENU", OpenEnvironmentMenu);
        environmentMenuButton.GetComponent<RectTransform>().sizeDelta = new Vector2(130f, 28f);
        AddClickSound(environmentMenuButton);

        controlsHelpButton = CreatePresetButton(content.transform, "ControlsHelpButton", new Vector2(90f, row8), "Controls", OpenControlsHelp);
        controlsHelpButton.GetComponent<RectTransform>().sizeDelta = new Vector2(120f, 28f);
        AddClickSound(controlsHelpButton);

        startActionButton = CreatePresetButton(content.transform, "StartButton", new Vector2(-156f, -204f), "START", ApplyAndStart);
        startActionButton.GetComponent<RectTransform>().sizeDelta = new Vector2(120f, 30f);
        startButtonText = startActionButton.GetComponentInChildren<Text>();
        AddClickSound(startActionButton);

        sceneSelectActionButton = CreatePresetButton(content.transform, "SceneSelectButton", new Vector2(0f, -204f), "SCENE SELECT", OpenSceneSelectFromSettings);
        sceneSelectActionButton.GetComponent<RectTransform>().sizeDelta = new Vector2(140f, 30f);
        AddClickSound(sceneSelectActionButton);

        endActionButton = CreatePresetButton(content.transform, "EndButton", new Vector2(156f, -204f), "END", EndGame);
        endActionButton.GetComponent<RectTransform>().sizeDelta = new Vector2(120f, 30f);
        AddClickSound(endActionButton);

        UpdateButtonHighlights();
    }

    void BuildEnvironmentPanel()
    {
        environmentPanel = CreateUiObject("EnvironmentPanel", transform);
        var panelImage = environmentPanel.AddComponent<Image>();
        panelImage.color = new Color(0f, 0.06f, 0.09f, 0.94f);
        StretchRect(environmentPanel.GetComponent<RectTransform>());

        GameObject content = CreateUiObject("EnvironmentContent", environmentPanel.transform);
        var contentImage = content.AddComponent<Image>();
        contentImage.color = new Color(0.03f, 0.11f, 0.16f, 0.78f);
        RectTransform contentRect = content.GetComponent<RectTransform>();
        contentRect.anchorMin = new Vector2(0.5f, 0.5f);
        contentRect.anchorMax = new Vector2(0.5f, 0.5f);
        contentRect.pivot = new Vector2(0.5f, 0.5f);
        contentRect.anchoredPosition = Vector2.zero;
        contentRect.sizeDelta = new Vector2(660f, 500f);

        Text title = CreateText("EnvironmentTitle", content.transform, 22, FontStyle.Bold);
        title.text = "Environment Menu";
        SetRect(title.rectTransform, new Vector2(0.5f, 0.92f), new Vector2(360f, 34f));

        float y = 150f;
        CreateSectionLabel(content.transform, "Transparency", y);
        envClearButton = CreatePresetButton(content.transform, "EnvClear", new Vector2(-155f, y), "Clear", () => SetEnvironmentVisibility(CameraWaterVisibilitySelector.VisibilityPreset.Clear));
        envNormalButton = CreatePresetButton(content.transform, "EnvNormal", new Vector2(0f, y), "Normal", () => SetEnvironmentVisibility(CameraWaterVisibilitySelector.VisibilityPreset.Normal));
        envMurkyButton = CreatePresetButton(content.transform, "EnvMurky", new Vector2(155f, y), "Murky", () => SetEnvironmentVisibility(CameraWaterVisibilitySelector.VisibilityPreset.Murky));

        y = 86f;
        CreateSectionLabel(content.transform, "Debris", y);
        envSonarNoiseOnButton = CreatePresetButton(content.transform, "EnvSonarNoiseOn", new Vector2(-230f, y), "Sonar virtual ON", () => SetSonarNoiseEnabled(true));
        envSonarNoiseOffButton = CreatePresetButton(content.transform, "EnvSonarNoiseOff", new Vector2(-75f, y), "OFF", () => SetSonarNoiseEnabled(false));
        envVisualMatterOnButton = CreatePresetButton(content.transform, "EnvVisualMatterOn", new Vector2(90f, y), "Visual drift ON", () => SetVisualMatterEnabled(true));
        envVisualMatterOffButton = CreatePresetButton(content.transform, "EnvVisualMatterOff", new Vector2(245f, y), "OFF", () => SetVisualMatterEnabled(false));

        y = 42f;
        envWorksiteDebrisOnButton = CreatePresetButton(content.transform, "EnvWorksiteDebrisOn", new Vector2(-80f, y), "Worksite ON", () => SetWorksiteDebrisEnabled(true));
        envWorksiteDebrisOffButton = CreatePresetButton(content.transform, "EnvWorksiteDebrisOff", new Vector2(80f, y), "Worksite OFF", () => SetWorksiteDebrisEnabled(false));

        y = -32f;
        CreateSectionLabel(content.transform, "Current", y);
        envCurrentNoneButton = CreatePresetButton(content.transform, "EnvCurrentNone", new Vector2(-230f, y), "None", () => SetEnvironmentCurrentVelocity(Vector3.zero));
        envCurrentWeakButton = CreatePresetButton(content.transform, "EnvCurrentWeak", new Vector2(-115f, y), "Weak", () => SetEnvironmentCurrentVelocity(new Vector3(0.02f, 0f, -0.04f)));
        envCurrentStrongButton = CreatePresetButton(content.transform, "EnvCurrentStrong", new Vector2(0f, y), "Strong", () => SetEnvironmentCurrentVelocity(new Vector3(0.06f, 0f, -0.12f)));
        envCurrentXInput = CreateFloatInputField(content.transform, "EnvCurrentX", new Vector2(125f, y), "X", selectedCurrentVelocity.x, v => SetEnvironmentCurrentVelocityComponent(0, v));
        envCurrentYInput = CreateFloatInputField(content.transform, "EnvCurrentY", new Vector2(225f, y), "Y", selectedCurrentVelocity.y, v => SetEnvironmentCurrentVelocityComponent(1, v));
        envCurrentZInput = CreateFloatInputField(content.transform, "EnvCurrentZ", new Vector2(325f, y), "Z", selectedCurrentVelocity.z, v => SetEnvironmentCurrentVelocityComponent(2, v));

        y = -98f;
        CreateSectionLabel(content.transform, "Day / Night", y);
        envDayButton = CreatePresetButton(content.transform, "EnvDay", new Vector2(-80f, y), "Day", () => SetEnvironmentTime(EnvironmentTimePreset.Day));
        envNightButton = CreatePresetButton(content.transform, "EnvNight", new Vector2(80f, y), "Night", () => SetEnvironmentTime(EnvironmentTimePreset.Night));

        y = -148f;
        CreateSectionLabel(content.transform, "BGM", y);
        envBgmOnButton = CreatePresetButton(content.transform, "EnvBgmOn", new Vector2(-80f, y), "BGM ON", () => SetBgmEnabled(true));
        envBgmOffButton = CreatePresetButton(content.transform, "EnvBgmOff", new Vector2(80f, y), "BGM OFF", () => SetBgmEnabled(false));

        envBackButton = CreatePresetButton(content.transform, "EnvBack", new Vector2(0f, -200f), "Back", CloseEnvironmentPanel);
        envBackButton.GetComponent<RectTransform>().sizeDelta = new Vector2(140f, 32f);

        AddClickSound(envClearButton);
        AddClickSound(envNormalButton);
        AddClickSound(envMurkyButton);
        AddClickSound(envSonarNoiseOnButton);
        AddClickSound(envSonarNoiseOffButton);
        AddClickSound(envVisualMatterOnButton);
        AddClickSound(envVisualMatterOffButton);
        AddClickSound(envWorksiteDebrisOnButton);
        AddClickSound(envWorksiteDebrisOffButton);
        AddClickSound(envCurrentNoneButton);
        AddClickSound(envCurrentWeakButton);
        AddClickSound(envCurrentStrongButton);
        AddClickSound(envDayButton);
        AddClickSound(envNightButton);
        AddClickSound(envBgmOnButton);
        AddClickSound(envBgmOffButton);
        AddClickSound(envBackButton);
    }

    void CreateSectionLabel(Transform parent, string textValue, float y)
    {
        Text text = CreateText(textValue + "Label", parent, 12, FontStyle.Bold);
        text.text = textValue;
        text.alignment = TextAnchor.MiddleLeft;
        SetRect(text.rectTransform, new Vector2(0.5f, 0.5f), new Vector2(120f, 24f));
        text.rectTransform.anchoredPosition = new Vector2(-300f, y);
    }

    void BuildControlsHelpPanel()
    {
        controlsHelpPanel = CreateUiObject("ControlsHelpPanel", transform);
        StretchRect(controlsHelpPanel.GetComponent<RectTransform>());

        var blocker = controlsHelpPanel.AddComponent<Image>();
        blocker.color = new Color(0f, 0f, 0f, 0.48f);

        GameObject window = CreateUiObject("Window", controlsHelpPanel.transform);
        var windowImage = window.AddComponent<Image>();
        windowImage.color = new Color(0.03f, 0.10f, 0.18f, 0.97f);
        RectTransform windowRect = window.GetComponent<RectTransform>();
        windowRect.anchorMin = new Vector2(0.5f, 0.5f);
        windowRect.anchorMax = new Vector2(0.5f, 0.5f);
        windowRect.pivot = new Vector2(0.5f, 0.5f);
        windowRect.anchoredPosition = Vector2.zero;
        windowRect.sizeDelta = new Vector2(560f, 310f);

        Text title = CreateText("Title", window.transform, 20, FontStyle.Bold);
        title.text = "Keyboard Controls";
        SetRect(title.rectTransform, new Vector2(0.5f, 0.82f), new Vector2(460f, 34f));

        Text help = CreateText("Help", window.transform, 15, FontStyle.Normal);
        help.alignment = TextAnchor.MiddleLeft;
        help.text = GetInputHelpText(ROVGamepadThrustController.ExternalInputDevice.Keyboard);
        SetRect(help.rectTransform, new Vector2(0.5f, 0.52f), new Vector2(500f, 140f));

        controlsHelpCloseButton = CreatePresetButton(window.transform, "CloseButton", new Vector2(0f, -120f), "CLOSE", CloseControlsHelp);
        controlsHelpCloseButton.GetComponent<RectTransform>().sizeDelta = new Vector2(120f, 32f);
        AddClickSound(controlsHelpCloseButton);

        controlsHelpPanel.SetActive(false);
    }

    void OpenControlsHelp()
    {
        if (controlsHelpPanel == null) return;
        controlsHelpPanel.SetActive(true);
        SelectButton(controlsHelpCloseButton);
    }

    void CloseControlsHelp()
    {
        if (controlsHelpPanel == null) return;
        controlsHelpPanel.SetActive(false);
    }

    void OpenEnvironmentMenu()
    {
        if (environmentPanel == null) return;

        CloseControlsHelp();
        if (settingsPanel != null) settingsPanel.SetActive(false);
        environmentPanel.SetActive(true);
        UpdateEnvironmentButtonHighlights();
        SelectButton(envBackButton);
    }

    void CloseEnvironmentPanel()
    {
        if (environmentPanel != null) environmentPanel.SetActive(false);
        if (settingsPanel != null) settingsPanel.SetActive(true);
        UpdateTexts();
        SelectButton(environmentMenuButton);
    }

    void SetEnvironmentVisibility(CameraWaterVisibilitySelector.VisibilityPreset preset)
    {
        selectedCamera = preset;
        EnsureSelectorsAndAutoCreate();
        if (cameraSelector != null) cameraSelector.SetPresetCameraOnly(selectedCamera);

        SeaTurbiditySelector[] sonarSelectors = FindObjectsByType<SeaTurbiditySelector>(FindObjectsInactive.Exclude, FindObjectsSortMode.None);
        for (int i = 0; i < sonarSelectors.Length; i++)
        {
            sonarSelectors[i].applyToSonar = true;
            sonarSelectors[i].SetPreset(ConvertToSonarPreset(selectedCamera));
        }

        UpdateEnvironmentButtonHighlights();
        UpdateTexts();
    }

    void SetSonarNoiseEnabled(bool enabled)
    {
        selectedSonarNoiseEnabled = enabled;

        MainCameraSwitcher[] switchers = FindObjectsByType<MainCameraSwitcher>(FindObjectsInactive.Exclude, FindObjectsSortMode.None);
        for (int i = 0; i < switchers.Length; i++)
            switchers[i].SetSonarSuspendedMatterEnabled(enabled);

        ImagingSonarSim[] sonars = FindObjectsByType<ImagingSonarSim>(FindObjectsInactive.Exclude, FindObjectsSortMode.None);
        for (int i = 0; i < sonars.Length; i++)
            sonars[i].SetWaterColumnNoiseEnabled(enabled);

        VirtualOculusM750dTerrainSonar[] oculusSonars = FindObjectsByType<VirtualOculusM750dTerrainSonar>(FindObjectsInactive.Exclude, FindObjectsSortMode.None);
        for (int i = 0; i < oculusSonars.Length; i++)
            oculusSonars[i].SetWaterColumnNoiseEnabled(enabled);

        UpdateEnvironmentButtonHighlights();
    }

    void SetVisualMatterEnabled(bool enabled)
    {
        selectedVisualMatterEnabled = enabled;
        float surfaceY = ResolveWaterSurfaceY();

        MainCameraSwitcher[] switchers = FindObjectsByType<MainCameraSwitcher>(FindObjectsInactive.Exclude, FindObjectsSortMode.None);
        for (int i = 0; i < switchers.Length; i++)
        {
            switchers[i].waterSurfaceY = surfaceY;
            switchers[i].SetUnderwaterParticlesEnabled(enabled);
        }

        UnderwaterSuspendedParticleField[] fields = FindObjectsByType<UnderwaterSuspendedParticleField>(FindObjectsInactive.Include, FindObjectsSortMode.None);
        for (int i = 0; i < fields.Length; i++)
        {
            fields[i].waterSurfaceY = surfaceY;
            fields[i].SetParticlesEnabled(enabled);
        }

        Camera[] cameras = FindObjectsByType<Camera>(FindObjectsInactive.Exclude, FindObjectsSortMode.None);
        for (int i = 0; i < cameras.Length; i++)
        {
            Camera cam = cameras[i];
            if (cam == null || !cam.enabled || cam.targetDisplay != 0) continue;

            UnderwaterSuspendedParticleField field = cam.GetComponent<UnderwaterSuspendedParticleField>();
            if (field == null)
                field = cam.gameObject.AddComponent<UnderwaterSuspendedParticleField>();

            ConfigureVisualDriftField(field, surfaceY);
            field.SetParticlesEnabled(enabled);
        }

        UpdateEnvironmentButtonHighlights();
    }

    static void ConfigureVisualDriftField(UnderwaterSuspendedParticleField field, float surfaceY)
    {
        if (field == null) return;

        field.maxParticles = Mathf.Max(field.maxParticles, 1200);
        field.emissionRate = Mathf.Max(field.emissionRate, 140f);
        field.volumeShape = UnderwaterSuspendedParticleField.VolumeShape.Ellipsoid;
        field.boxSize = new Vector3(9f, 5f, 12f);
        field.localOffset = new Vector3(0f, 0f, 5f);
        field.lifetime = new Vector2(8f, 18f);
        field.size = new Vector2(0.012f, 0.045f);
        field.speed = new Vector2(0.002f, 0.018f);
        field.alpha = 0.32f;
        field.driftVelocity = new Vector3(0.003f, 0.001f, -0.01f);
        field.waterSurfaceY = surfaceY;
        field.onlyBelowWaterSurface = false;
    }

    void SetWorksiteDebrisEnabled(bool enabled)
    {
        selectedWorksiteDebrisEnabled = enabled;

        UnderwaterWorksiteDebrisField[] fields = FindObjectsByType<UnderwaterWorksiteDebrisField>(FindObjectsInactive.Include, FindObjectsSortMode.None);
        for (int i = 0; i < fields.Length; i++)
        {
            fields[i].debrisEnabled = enabled;
            fields[i].visibleToSonar = enabled;
        }

        UpdateEnvironmentButtonHighlights();
    }

    void SetEnvironmentCurrentVelocity(Vector3 velocity)
    {
        selectedCurrentVelocity = velocity;
        ApplyCurrentVelocityToScene();
        UpdateCurrentInputTexts();
        UpdateEnvironmentCurrentInputTexts();
        UpdateEnvironmentButtonHighlights();
    }

    void SetEnvironmentCurrentVelocityComponent(int axis, string value)
    {
        if (!TryParseFloat(value, out float parsed)) return;

        if (axis == 0) selectedCurrentVelocity.x = parsed;
        else if (axis == 1) selectedCurrentVelocity.y = parsed;
        else if (axis == 2) selectedCurrentVelocity.z = parsed;

        ApplyCurrentVelocityToScene();
        UpdateCurrentInputTexts();
        UpdateEnvironmentButtonHighlights();
    }

    static Button CreatePresetButton(Transform parent, string name, Vector2 pos, string label, UnityEngine.Events.UnityAction onClick)
    {
        GameObject buttonObject = CreateUiObject(name, parent);
        var image = buttonObject.AddComponent<Image>();
        image.color = new Color(0.12f, 0.5f, 0.7f, 0.95f);

        var button = buttonObject.AddComponent<Button>();
        var colors = button.colors;
        colors.normalColor = image.color;
        colors.highlightedColor = new Color(0.2f, 0.62f, 0.84f, 1f);
        colors.pressedColor = new Color(0.08f, 0.35f, 0.5f, 1f);
        colors.selectedColor = colors.highlightedColor;
        button.colors = colors;
        button.onClick.AddListener(onClick);

        RectTransform rect = buttonObject.GetComponent<RectTransform>();
        rect.anchorMin = new Vector2(0.5f, 0.5f);
        rect.anchorMax = new Vector2(0.5f, 0.5f);
        rect.anchoredPosition = pos;
        rect.sizeDelta = new Vector2(160f, 40f);

        Text text = CreateText("Label", buttonObject.transform, 9, FontStyle.Bold);
        text.text = label;
        RectTransform textRect = text.rectTransform;
        textRect.anchorMin = Vector2.zero;
        textRect.anchorMax = Vector2.one;
        textRect.offsetMin = Vector2.zero;
        textRect.offsetMax = Vector2.zero;

        return button;
    }

    InputField CreateFloatInputField(Transform parent, string name, Vector2 pos, string placeholder, float initialValue, UnityEngine.Events.UnityAction<string> onEndEdit)
    {
        GameObject fieldObject = CreateUiObject(name, parent);
        var image = fieldObject.AddComponent<Image>();
        image.color = new Color(0.06f, 0.14f, 0.22f, 0.95f);

        var input = fieldObject.AddComponent<InputField>();
        input.lineType = InputField.LineType.SingleLine;
        input.characterValidation = InputField.CharacterValidation.None;
        input.contentType = InputField.ContentType.Standard;
        input.onEndEdit.AddListener(onEndEdit);

        RectTransform rect = fieldObject.GetComponent<RectTransform>();
        rect.anchorMin = new Vector2(0.5f, 0.5f);
        rect.anchorMax = new Vector2(0.5f, 0.5f);
        rect.anchoredPosition = pos;
        rect.sizeDelta = new Vector2(100f, 30f);

        Text text = CreateText("Text", fieldObject.transform, 13, FontStyle.Normal);
        text.alignment = TextAnchor.MiddleLeft;
        text.color = Color.white;
        RectTransform textRect = text.rectTransform;
        textRect.anchorMin = Vector2.zero;
        textRect.anchorMax = Vector2.one;
        textRect.offsetMin = new Vector2(10f, 6f);
        textRect.offsetMax = new Vector2(-10f, -6f);

        Text placeholderText = CreateText("Placeholder", fieldObject.transform, 13, FontStyle.Italic);
        placeholderText.alignment = TextAnchor.MiddleLeft;
        placeholderText.color = new Color(1f, 1f, 1f, 0.4f);
        placeholderText.text = placeholder;
        RectTransform placeholderRect = placeholderText.rectTransform;
        placeholderRect.anchorMin = Vector2.zero;
        placeholderRect.anchorMax = Vector2.one;
        placeholderRect.offsetMin = new Vector2(10f, 6f);
        placeholderRect.offsetMax = new Vector2(-10f, -6f);

        input.textComponent = text;
        input.placeholder = placeholderText;
        input.text = FloatToInputString(initialValue);
        return input;
    }

    void UpdateButtonHighlights()
    {
        UpdateSceneSelectionButtons();

        SetButtonSelected(camClearButton, selectedCamera == CameraWaterVisibilitySelector.VisibilityPreset.Clear);
        SetButtonSelected(camNormalButton, selectedCamera == CameraWaterVisibilitySelector.VisibilityPreset.Normal);
        SetButtonSelected(camMurkyButton, selectedCamera == CameraWaterVisibilitySelector.VisibilityPreset.Murky);

        SetButtonSelected(sonarImagingButton, selectedSonarMode == SonarModePreset.ImagingSonar);
        SetButtonSelected(sonarOculusButton, selectedSonarMode == SonarModePreset.VirtualOculus);

        SetButtonSelected(inputGamepadButton, selectedInput == ROVGamepadThrustController.ExternalInputDevice.Gamepad);
        SetButtonSelected(inputJoystickButton, selectedInput == ROVGamepadThrustController.ExternalInputDevice.Joystick);
        SetButtonSelected(inputKeyboardButton, selectedInput == ROVGamepadThrustController.ExternalInputDevice.Keyboard);

        SetButtonSelected(sonarRig0Button, selectedSonarRigX == SonarRigXPreset.Deg0);
        SetButtonSelected(sonarRig20Button, selectedSonarRigX == SonarRigXPreset.Deg20);
        SetButtonSelected(sonarRig90Button, selectedSonarRigX == SonarRigXPreset.Deg90);

        SetButtonSelected(missionOnButton, selectedMissionEnabled);
        SetButtonSelected(missionOffButton, !selectedMissionEnabled);
        SetButtonSelected(dayButton, selectedEnvironmentTime == EnvironmentTimePreset.Day);
        SetButtonSelected(nightButton, selectedEnvironmentTime == EnvironmentTimePreset.Night);
        SetButtonSelected(performanceQualityButton, selectedPerformance == PerformancePreset.Quality);
        SetButtonSelected(performanceLightButton, selectedPerformance == PerformancePreset.Lightweight);
        UpdateEnvironmentButtonHighlights();

        if (controlsHelpButton != null)
            controlsHelpButton.gameObject.SetActive(selectedInput == ROVGamepadThrustController.ExternalInputDevice.Keyboard);
    }

    void UpdateCurrentInputTexts()
    {
        UpdateEnvironmentCurrentInputTexts();
    }

    void UpdateEnvironmentButtonHighlights()
    {
        SetButtonSelected(envClearButton, selectedCamera == CameraWaterVisibilitySelector.VisibilityPreset.Clear);
        SetButtonSelected(envNormalButton, selectedCamera == CameraWaterVisibilitySelector.VisibilityPreset.Normal);
        SetButtonSelected(envMurkyButton, selectedCamera == CameraWaterVisibilitySelector.VisibilityPreset.Murky);

        SetButtonSelected(envSonarNoiseOnButton, selectedSonarNoiseEnabled);
        SetButtonSelected(envSonarNoiseOffButton, !selectedSonarNoiseEnabled);
        SetButtonSelected(envVisualMatterOnButton, selectedVisualMatterEnabled);
        SetButtonSelected(envVisualMatterOffButton, !selectedVisualMatterEnabled);
        SetButtonSelected(envWorksiteDebrisOnButton, selectedWorksiteDebrisEnabled);
        SetButtonSelected(envWorksiteDebrisOffButton, !selectedWorksiteDebrisEnabled);

        SetButtonSelected(envCurrentNoneButton, selectedCurrentVelocity.sqrMagnitude < 0.0001f);
        SetButtonSelected(envCurrentWeakButton, Vector3.Distance(selectedCurrentVelocity, new Vector3(0.02f, 0f, -0.04f)) < 0.001f);
        SetButtonSelected(envCurrentStrongButton, Vector3.Distance(selectedCurrentVelocity, new Vector3(0.06f, 0f, -0.12f)) < 0.001f);

        SetButtonSelected(envDayButton, selectedEnvironmentTime == EnvironmentTimePreset.Day);
        SetButtonSelected(envNightButton, selectedEnvironmentTime == EnvironmentTimePreset.Night);
        SetButtonSelected(envBgmOnButton, selectedBgmEnabled);
        SetButtonSelected(envBgmOffButton, !selectedBgmEnabled);
    }

    void UpdateEnvironmentCurrentInputTexts()
    {
        if (envCurrentXInput != null) envCurrentXInput.SetTextWithoutNotify(FloatToInputString(selectedCurrentVelocity.x));
        if (envCurrentYInput != null) envCurrentYInput.SetTextWithoutNotify(FloatToInputString(selectedCurrentVelocity.y));
        if (envCurrentZInput != null) envCurrentZInput.SetTextWithoutNotify(FloatToInputString(selectedCurrentVelocity.z));
    }

    void SyncCurrentVelocityFromInputs()
    {
        if (envCurrentXInput != null && TryParseFloat(envCurrentXInput.text, out float x)) selectedCurrentVelocity.x = x;
        if (envCurrentYInput != null && TryParseFloat(envCurrentYInput.text, out float y)) selectedCurrentVelocity.y = y;
        if (envCurrentZInput != null && TryParseFloat(envCurrentZInput.text, out float z)) selectedCurrentVelocity.z = z;
    }

    static bool TryParseFloat(string text, out float value)
    {
        return float.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out value) ||
               float.TryParse(text, NumberStyles.Float, CultureInfo.CurrentCulture, out value);
    }

    static SeaTurbiditySelector.TurbidityPreset ConvertToSonarPreset(CameraWaterVisibilitySelector.VisibilityPreset preset)
    {
        if (preset == CameraWaterVisibilitySelector.VisibilityPreset.Clear) return SeaTurbiditySelector.TurbidityPreset.Clear;
        if (preset == CameraWaterVisibilitySelector.VisibilityPreset.Murky) return SeaTurbiditySelector.TurbidityPreset.Murky;
        return SeaTurbiditySelector.TurbidityPreset.Normal;
    }

    static float ResolveWaterSurfaceY()
    {
        WaterSurface waterSurface = FindFirstObjectByType<WaterSurface>();
        return waterSurface != null ? waterSurface.transform.position.y : 0f;
    }

    static string SanitizeObjectName(string value)
    {
        if (string.IsNullOrWhiteSpace(value)) return "Scene";

        char[] chars = value.ToCharArray();
        for (int i = 0; i < chars.Length; i++)
        {
            if (!char.IsLetterOrDigit(chars[i]) && chars[i] != '_' && chars[i] != '-')
                chars[i] = '_';
        }

        return new string(chars);
    }

    static string FloatToInputString(float value)
    {
        return value.ToString("0.###", CultureInfo.InvariantCulture);
    }

    static string GetInputHelpText(ROVGamepadThrustController.ExternalInputDevice device)
    {
        switch (device)
        {
            case ROVGamepadThrustController.ExternalInputDevice.Keyboard:
                return "Keyboard: W/S forward/back  A/D or Arrow left/right  Q/E yaw  R/Space up  F down\n"
                    + "Camera: Up/PageUp tilt up  Down/PageDown tilt down   Gripper: Z close  X open\n"
                    + "Heading Lock: H   Alt Hold: J   Auto Pay: K   Light: [ darker  ] brighter   Gain: - lower  = higher\n"
                    + "Menu: Enter/Space submit  Esc settings   Visibility: F5 Clear  F6 Normal  F7 Murky";

            case ROVGamepadThrustController.ExternalInputDevice.Joystick:
                return "Joystick: Stick move  Twist yaw  Throttle/slider heave\n"
                    + "Camera: Hat up/down, or button3/button2 fallback   Gripper: Trigger close, button2/button3 open\n"
                    + "Heading Lock: H key   Alt Hold: J key   Auto Pay: K key   Menu: Trigger submit   Visibility hotkeys remain F5/F6/F7";

            default:
                return "Gamepad: Left stick move  Right stick yaw/heave\n"
                    + "Camera: RB/LB tilt   Gripper: LT close, RT open   D-pad: light left/right, gain up/down\n"
                    + "Heading Lock: Y   Alt Hold: X   Auto Pay: B   Menu: A/Start submit, Start settings   Visibility hotkeys remain F5/F6/F7";
        }
    }

    void SelectSceneAndOpenSettings(string sceneName)
    {
        SceneSelector.SelectGameScene(sceneName);
        StartCoroutine(OpenSettingsNextFrame());
    }

    IEnumerator OpenSettingsNextFrame()
    {
        titleOpened = false;
        sceneSelectOpened = false;
        if (sceneSelectPanel != null) sceneSelectPanel.SetActive(false);

        do
        {
            yield return null;
        }
        while (IsPrimaryPointerStillPressed());

        yield return null;
        OpenSettings();
    }

    static bool IsPrimaryPointerStillPressed()
    {
        if (Mouse.current != null && Mouse.current.leftButton.isPressed) return true;
        return false;
    }

    void UpdateSceneSelectionButtons()
    {
        for (int i = 0; i < sceneSelectionButtons.Count && i < sceneSelectionNames.Count; i++)
            SetButtonSelected(sceneSelectionButtons[i], SceneSelector.SelectedGameSceneName == sceneSelectionNames[i]);
    }

    void SetButtonSelected(Button button, bool selected)
    {
        if (button == null) return;

        Image img = button.GetComponent<Image>();
        if (img != null) img.color = selected ? ButtonSelectedColor : ButtonNormalColor;

        ColorBlock colors = button.colors;
        Color baseColor = selected ? ButtonSelectedColor : ButtonNormalColor;
        colors.normalColor = baseColor;
        colors.highlightedColor = Color.Lerp(baseColor, Color.white, 0.18f);
        colors.pressedColor = Color.Lerp(baseColor, Color.black, 0.20f);
        colors.selectedColor = colors.highlightedColor;
        button.colors = colors;
    }

    static GameObject CreateUiObject(string name, Transform parent)
    {
        GameObject go = new GameObject(name);
        go.transform.SetParent(parent, false);
        go.AddComponent<RectTransform>();
        return go;
    }

    static Text CreateText(string name, Transform parent, int fontSize, FontStyle fontStyle)
    {
        GameObject go = CreateUiObject(name, parent);
        Text text = go.AddComponent<Text>();
        text.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
        text.fontSize = fontSize;
        text.fontStyle = fontStyle;
        text.color = Color.white;
        text.alignment = TextAnchor.MiddleCenter;
        return text;
    }

    static RawImage CreateGradientBackground(Transform parent)
    {
        GameObject go = CreateUiObject("GradientBackground", parent);
        RawImage image = go.AddComponent<RawImage>();
        image.texture = CreateVerticalGradientTexture(
            new Color(0.02f, 0.08f, 0.20f, 1f),
            new Color(0.22f, 0.48f, 0.78f, 1f));
        image.raycastTarget = false;
        return image;
    }

    static Texture2D CreateVerticalGradientTexture(Color topColor, Color bottomColor)
    {
        Texture2D texture = new Texture2D(1, 64, TextureFormat.RGBA32, false);
        texture.wrapMode = TextureWrapMode.Clamp;
        texture.filterMode = FilterMode.Bilinear;

        for (int y = 0; y < texture.height; y++)
        {
            float t = y / (float)(texture.height - 1);
            texture.SetPixel(0, y, Color.Lerp(bottomColor, topColor, t));
        }

        texture.Apply();
        return texture;
    }

    void EnsureMenuCamera()
    {
        if (!SceneSelector.IsMenuSceneActive()) return;

        Camera[] cameras = FindObjectsByType<Camera>(FindObjectsInactive.Include, FindObjectsSortMode.None);
        for (int i = 0; i < cameras.Length; i++)
        {
            Camera cam = cameras[i];
            if (cam == null) continue;
            if (!cam.enabled) continue;
            if (cam.targetDisplay != 0) continue;
            return;
        }

        GameObject cameraObject = new GameObject("MenuFallbackCamera");
        Camera fallbackCamera = cameraObject.AddComponent<Camera>();
        fallbackCamera.clearFlags = CameraClearFlags.SolidColor;
        fallbackCamera.backgroundColor = Color.black;
        fallbackCamera.cullingMask = ~0;
        fallbackCamera.nearClipPlane = 0.3f;
        fallbackCamera.farClipPlane = 1000f;
        fallbackCamera.depth = -100f;
        fallbackCamera.targetDisplay = 0;

        AudioListener existingListener = FindFirstObjectByType<AudioListener>(FindObjectsInactive.Include);
        if (existingListener == null)
            cameraObject.AddComponent<AudioListener>();
    }

    static void SetRect(RectTransform rect, Vector2 anchor, Vector2 size)
    {
        rect.anchorMin = anchor;
        rect.anchorMax = anchor;
        rect.pivot = new Vector2(0.5f, 0.5f);
        rect.anchoredPosition = Vector2.zero;
        rect.sizeDelta = size;
    }

    static void StretchRect(RectTransform rect)
    {
        rect.anchorMin = Vector2.zero;
        rect.anchorMax = Vector2.one;
        rect.offsetMin = Vector2.zero;
        rect.offsetMax = Vector2.zero;
    }

    static void EnsureEventSystem()
    {
        if (FindFirstObjectByType<EventSystem>() != null) return;

        GameObject es = new GameObject("EventSystem");
        EventSystem eventSystem = es.AddComponent<EventSystem>();
        eventSystem.sendNavigationEvents = true;

        InputSystemUIInputModule inputSystemModule = es.AddComponent<InputSystemUIInputModule>();
        inputSystemModule.AssignDefaultActions();
    }

    static bool WasTogglePressedThisFrame()
    {
        if (Keyboard.current != null && Keyboard.current.escapeKey.wasPressedThisFrame) return true;
        if (Gamepad.current != null && Gamepad.current.startButton.wasPressedThisFrame) return true;
        return false;
    }

    static bool WasTitleProceedPressedThisFrame()
    {
        if (Mouse.current != null && Mouse.current.leftButton.wasPressedThisFrame) return true;
        if (Keyboard.current != null &&
            (Keyboard.current.enterKey.wasPressedThisFrame || Keyboard.current.spaceKey.wasPressedThisFrame))
            return true;
        if (Gamepad.current != null &&
            (Gamepad.current.buttonSouth.wasPressedThisFrame || Gamepad.current.startButton.wasPressedThisFrame))
            return true;
        if (Joystick.current != null && Joystick.current.trigger.wasPressedThisFrame) return true;
        return false;
    }

    static bool WasSettingsSubmitPressedThisFrame()
    {
        if (Keyboard.current != null &&
            (Keyboard.current.enterKey.wasPressedThisFrame || Keyboard.current.spaceKey.wasPressedThisFrame))
            return true;
        if (Gamepad.current != null &&
            (Gamepad.current.buttonSouth.wasPressedThisFrame || Gamepad.current.startButton.wasPressedThisFrame))
            return true;
        if (Joystick.current != null && Joystick.current.trigger.wasPressedThisFrame) return true;
        return false;
    }

    static void SelectButton(Button button)
    {
        EventSystem current = EventSystem.current;
        if (current == null) return;

        current.SetSelectedGameObject(null);
        if (button != null && button.gameObject.activeInHierarchy)
            current.SetSelectedGameObject(button.gameObject);
    }

    void SubmitCurrentSelection()
    {
        EventSystem current = EventSystem.current;
        if (current == null) return;

        GameObject selected = current.currentSelectedGameObject;
        if (selected == null && startActionButton != null && startActionButton.gameObject.activeInHierarchy)
            selected = startActionButton.gameObject;

        if (selected == null) return;

        ExecuteEvents.Execute(selected, new BaseEventData(current), ExecuteEvents.submitHandler);
    }

}
