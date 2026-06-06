using UnityEngine;
using UnityEngine.InputSystem;

[AddComponentMenu("#NVJOB/Tools/Slow Motion")]
public class SlowMo : MonoBehaviour
{
    public AudioSource[] audios;

    void Awake()
    {
        Time.timeScale = 1.0f;
    }

    void LateUpdate()
    {
        if (Mouse.current != null && Mouse.current.leftButton.wasPressedThisFrame)
        {
            if (Time.timeScale != 0.33f)
            {
                Time.timeScale = 0.33f;
                for (int i = 0; i < audios.Length; i++) audios[i].pitch = 0.5f;
            }
            else if (Time.timeScale != 1.0f)
            {
                Time.timeScale = 1.0f;
                for (int i = 0; i < audios.Length; i++) audios[i].pitch = 1.0f;
            }
        }
    }

    void OnApplicationQuit()
    {
        Time.timeScale = 1.0f;
    }
}
