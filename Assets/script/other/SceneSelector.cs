using UnityEngine;
using UnityEngine.SceneManagement;
using System.Collections.Generic;
using System.IO;

#if UNITY_EDITOR
using UnityEditor;
#endif

public class SceneSelector : MonoBehaviour
{
    public const string MenuSceneName = "EmptyMenu";
    public const string AuvTestSceneName = "AUV_TEST";
    public const string DefaultGameSceneName = AuvTestSceneName;
    public const string UBoatSceneName = "U_Boat";
    public const string UnderwaterStructureSceneName = "UnderwaterStructure";
    public const string WaveEvaluationSceneName = "WaveEvaluation_U_Boat";
    public const string SampleSceneName = "SampleScene";

    public struct LaunchConfiguration
    {
        public string gameSceneName;
        public CameraWaterVisibilitySelector.VisibilityPreset cameraPreset;
        public StartupEnvironmentPresetMenu.SonarModePreset sonarMode;
        public ROVGamepadThrustController.ExternalInputDevice inputDevice;
        public StartupEnvironmentPresetMenu.SonarRigXPreset sonarRigXPreset;
        public Vector3 currentVelocity;
        public bool missionEnabled;
        public StartupEnvironmentPresetMenu.EnvironmentTimePreset environmentTimePreset;
    }

    static string selectedGameSceneName = DefaultGameSceneName;
    static bool selectedMissionEnabled = false;
    static StartupEnvironmentPresetMenu.EnvironmentTimePreset selectedEnvironmentTimePreset = StartupEnvironmentPresetMenu.EnvironmentTimePreset.Day;
    static bool hasPendingLaunchConfiguration;
    static LaunchConfiguration pendingLaunchConfiguration;

#if UNITY_EDITOR
    [InitializeOnLoadMethod]
    static void SynchronizeEditorBuildSettings()
    {
        var scenePaths = new List<string>
        {
            "Assets/Scenes/EmptyMenu.unity",
            "Assets/Scenes/AUV_TEST.unity"
        };

        var buildScenes = new EditorBuildSettingsScene[scenePaths.Count];
        for (int i = 0; i < scenePaths.Count; i++)
            buildScenes[i] = new EditorBuildSettingsScene(scenePaths[i], true);

        EditorBuildSettings.scenes = buildScenes;
    }
#endif

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
    static void InstallSceneLoadLogger()
    {
        SceneManager.sceneLoaded -= OnSceneLoadedLog;
        SceneManager.sceneLoaded += OnSceneLoadedLog;
    }

    public static string SelectedGameSceneName
    {
        get
        {
            if (string.IsNullOrWhiteSpace(selectedGameSceneName))
                selectedGameSceneName = DefaultGameSceneName;

            if (!IsSceneAvailable(selectedGameSceneName))
                selectedGameSceneName = GetDefaultAvailableGameSceneName();

            return selectedGameSceneName;
        }
    }

    public static bool SelectedMissionEnabled => selectedMissionEnabled;
    public static StartupEnvironmentPresetMenu.EnvironmentTimePreset SelectedEnvironmentTimePreset => selectedEnvironmentTimePreset;

    public static bool IsMenuSceneActive()
    {
        return SceneManager.GetActiveScene().name == MenuSceneName;
    }

    public static bool IsWaveEvaluationSceneActive()
    {
        return SceneManager.GetActiveScene().name == WaveEvaluationSceneName;
    }

    public static void SelectGameScene(string sceneName)
    {
        if (string.IsNullOrWhiteSpace(sceneName)) return;
        selectedGameSceneName = sceneName;
    }

    public static List<string> GetAvailableGameSceneNames()
    {
        var sceneNames = new List<string>();

#if UNITY_EDITOR
        string[] sceneGuids = AssetDatabase.FindAssets("t:Scene", new[] { "Assets/Scenes" });
        for (int i = 0; i < sceneGuids.Length; i++)
        {
            string path = AssetDatabase.GUIDToAssetPath(sceneGuids[i]);
            string sceneName = GetSceneNameFromPath(path);
            if (!ShouldHideFromGameSceneMenu(sceneName) && !sceneNames.Contains(sceneName))
                sceneNames.Add(sceneName);
        }
#else
        int count = SceneManager.sceneCountInBuildSettings;
        for (int i = 0; i < count; i++)
        {
            string sceneName = GetSceneNameFromPath(SceneUtility.GetScenePathByBuildIndex(i));
            if (!ShouldHideFromGameSceneMenu(sceneName) && !sceneNames.Contains(sceneName))
                sceneNames.Add(sceneName);
        }
#endif

        sceneNames.Sort((a, b) => string.Compare(a, b, System.StringComparison.OrdinalIgnoreCase));

        int defaultIndex = sceneNames.IndexOf(DefaultGameSceneName);
        if (defaultIndex > 0)
        {
            sceneNames.RemoveAt(defaultIndex);
            sceneNames.Insert(0, DefaultGameSceneName);
        }

        return sceneNames;
    }

    static bool IsSceneAvailable(string sceneName)
    {
        return GetAvailableGameSceneNames().Contains(sceneName);
    }

    static string GetDefaultAvailableGameSceneName()
    {
        List<string> scenes = GetAvailableGameSceneNames();
        if (scenes.Count > 0)
            return scenes[0];

        return DefaultGameSceneName;
    }

    static bool ShouldHideFromGameSceneMenu(string sceneName)
    {
        return string.IsNullOrWhiteSpace(sceneName) ||
               sceneName == MenuSceneName ||
               sceneName == WaveEvaluationSceneName ||
               sceneName == SampleSceneName;
    }

    static string GetSceneNameFromPath(string path)
    {
        if (string.IsNullOrWhiteSpace(path)) return string.Empty;
        return Path.GetFileNameWithoutExtension(path);
    }

    public static void SetMissionEnabled(bool enabled)
    {
        selectedMissionEnabled = enabled;
    }

    public static void SetEnvironmentTimePreset(StartupEnvironmentPresetMenu.EnvironmentTimePreset preset)
    {
        selectedEnvironmentTimePreset = preset;
    }

    public static void PrepareLaunch(
        CameraWaterVisibilitySelector.VisibilityPreset cameraPreset,
        StartupEnvironmentPresetMenu.SonarModePreset sonarMode,
        ROVGamepadThrustController.ExternalInputDevice inputDevice,
        StartupEnvironmentPresetMenu.SonarRigXPreset sonarRigXPreset,
        Vector3 currentVelocity,
        bool missionEnabled,
        StartupEnvironmentPresetMenu.EnvironmentTimePreset environmentTimePreset)
    {
        selectedMissionEnabled = missionEnabled;
        selectedEnvironmentTimePreset = environmentTimePreset;
        hasPendingLaunchConfiguration = true;
        pendingLaunchConfiguration = new LaunchConfiguration
        {
            gameSceneName = SelectedGameSceneName,
            cameraPreset = cameraPreset,
            sonarMode = sonarMode,
            inputDevice = inputDevice,
            sonarRigXPreset = sonarRigXPreset,
            currentVelocity = currentVelocity,
            missionEnabled = missionEnabled,
            environmentTimePreset = environmentTimePreset
        };
    }

    public static bool TryConsumePendingLaunch(out LaunchConfiguration configuration)
    {
        if (!hasPendingLaunchConfiguration)
        {
            configuration = default;
            return false;
        }

        hasPendingLaunchConfiguration = false;
        configuration = pendingLaunchConfiguration;
        selectedGameSceneName = configuration.gameSceneName;
        selectedMissionEnabled = configuration.missionEnabled;
        selectedEnvironmentTimePreset = configuration.environmentTimePreset;
        return true;
    }

    static void OnSceneLoadedLog(Scene scene, LoadSceneMode mode)
    {
        if (IsWaveEvaluationSceneActive())
        {
            WaveParameterSweepRunner.EnsureInstance();
            WaveObservationLogger.EnsureInstance();
        }

        if (scene.name == UBoatSceneName && SelectedMissionEnabled)
            CableClearMission.EnsureInstance();

        if (!IsWaveEvaluationSceneActive())
            StartupEnvironmentPresetMenu.EnsureInstance();

        if (!IsMenuSceneActive() && !IsWaveEvaluationSceneActive())
            EnvironmentControlMenu.EnsureInstance();
    }
}
