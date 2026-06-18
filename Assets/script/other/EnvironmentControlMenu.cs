using UnityEngine;

[DefaultExecutionOrder(-120)]
public class EnvironmentControlMenu : MonoBehaviour
{
    public static void EnsureInstance()
    {
        StartupEnvironmentPresetMenu.EnsureInstance();
    }

    void Awake()
    {
        if (Application.isPlaying)
            Destroy(gameObject);
        else
            DestroyImmediate(gameObject);
    }
}
