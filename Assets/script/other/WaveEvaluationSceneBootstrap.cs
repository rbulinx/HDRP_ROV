using UnityEngine;
using UnityEngine.Rendering.HighDefinition;
using UnityEngine.SceneManagement;

public static class WaveEvaluationSceneBootstrap
{
    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    static void Bootstrap()
    {
        if (!SceneSelector.IsWaveEvaluationSceneActive()) return;

        Scene scene = SceneManager.GetActiveScene();
        GameObject oceanRoot = FindOceanRoot(scene);

        foreach (GameObject root in scene.GetRootGameObjects())
        {
            if (root == null) continue;
            if (root == oceanRoot) continue;
            if (root.name == "WaveObservationLogger") continue;
            root.SetActive(false);
        }

        EnsureEvaluationCamera();
        EnsureEvaluationLight();
        WaveObservationLogger.EnsureInstance();
        WaveParameterSweepRunner.EnsureInstance();
    }

    static GameObject FindOceanRoot(Scene scene)
    {
        WaterSurface surface = Object.FindFirstObjectByType<WaterSurface>();
        if (surface != null) return surface.gameObject;

        foreach (GameObject root in scene.GetRootGameObjects())
        {
            if (root != null && root.name == "Ocean")
                return root;
        }

        return null;
    }

    static void EnsureEvaluationCamera()
    {
        if (Object.FindFirstObjectByType<Camera>() != null) return;

        GameObject cameraObject = new GameObject("WaveEvaluationCamera");
        Camera cam = cameraObject.AddComponent<Camera>();
        cam.transform.position = new Vector3(0f, 6f, -12f);
        cam.transform.rotation = Quaternion.Euler(18f, 0f, 0f);
        cam.clearFlags = CameraClearFlags.Skybox;
        cam.nearClipPlane = 0.1f;
        cam.farClipPlane = 2000f;

        if (Object.FindFirstObjectByType<AudioListener>() == null)
            cameraObject.AddComponent<AudioListener>();

        HDAdditionalCameraData hdData = cameraObject.AddComponent<HDAdditionalCameraData>();
        hdData.clearColorMode = HDAdditionalCameraData.ClearColorMode.Sky;
    }

    static void EnsureEvaluationLight()
    {
        if (Object.FindFirstObjectByType<Light>() != null) return;

        GameObject lightObject = new GameObject("WaveEvaluationLight");
        Light lightComponent = lightObject.AddComponent<Light>();
        lightComponent.type = LightType.Directional;
        lightComponent.intensity = 1.1f;
        lightComponent.transform.rotation = Quaternion.Euler(45f, -30f, 0f);
    }

}
