using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.Controls;
#if UNITY_STANDALONE_WIN
using System;
using System.Diagnostics;
using System.Runtime.InteropServices;
#endif

public sealed class ApplicationFocusKeeper : MonoBehaviour
{
    static bool created;

    CursorLockMode cursorLockBeforeBlur = CursorLockMode.None;
    bool cursorVisibleBeforeBlur = true;
    bool shouldRestoreCursorState;
    bool hasFocus = true;

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    static void Create()
    {
        if (created) return;
        created = true;

        var go = new GameObject(nameof(ApplicationFocusKeeper));
        DontDestroyOnLoad(go);
        go.hideFlags = HideFlags.HideAndDontSave;
        go.AddComponent<ApplicationFocusKeeper>();
    }

    void Awake()
    {
        Application.runInBackground = true;
    }

    void Update()
    {
        TryRestoreWindowFromBackground();

        if (!shouldRestoreCursorState) return;

        // Wait until the user intentionally returns to the Unity window.
        if (!WasAnyRestoreInputPressedThisFrame())
        {
            return;
        }

        Cursor.lockState = cursorLockBeforeBlur;
        Cursor.visible = cursorVisibleBeforeBlur;
        shouldRestoreCursorState = false;
    }

    void OnApplicationFocus(bool hasFocus)
    {
        HandleFocusChange(hasFocus);
    }

    void OnApplicationPause(bool isPaused)
    {
        HandleFocusChange(!isPaused);
    }

    void HandleFocusChange(bool hasFocus)
    {
        this.hasFocus = hasFocus;

        if (hasFocus)
        {
            shouldRestoreCursorState = true;
            return;
        }

        cursorLockBeforeBlur = Cursor.lockState;
        cursorVisibleBeforeBlur = Cursor.visible;

        // Make it easy to interact with the other app on the same PC.
        Cursor.lockState = CursorLockMode.None;
        Cursor.visible = true;
        shouldRestoreCursorState = false;
    }

    void TryRestoreWindowFromBackground()
    {
        if (hasFocus) return;

#if UNITY_STANDALONE_WIN
        if (!WasRestoreShortcutPressed()) return;
        RestoreUnityWindow();
#endif
    }

    static bool WasAnyRestoreInputPressedThisFrame()
    {
        if (Keyboard.current != null)
        {
            foreach (KeyControl key in Keyboard.current.allKeys)
            {
                if (key.wasPressedThisFrame) return true;
            }
        }

        if (Mouse.current != null)
        {
            if (Mouse.current.leftButton.wasPressedThisFrame) return true;
            if (Mouse.current.rightButton.wasPressedThisFrame) return true;
            if (Mouse.current.middleButton.wasPressedThisFrame) return true;
        }

        if (Gamepad.current != null)
        {
            if (Gamepad.current.buttonSouth.wasPressedThisFrame) return true;
            if (Gamepad.current.buttonNorth.wasPressedThisFrame) return true;
            if (Gamepad.current.buttonEast.wasPressedThisFrame) return true;
            if (Gamepad.current.buttonWest.wasPressedThisFrame) return true;
            if (Gamepad.current.startButton.wasPressedThisFrame) return true;
            if (Gamepad.current.selectButton.wasPressedThisFrame) return true;
            if (Gamepad.current.leftShoulder.wasPressedThisFrame) return true;
            if (Gamepad.current.rightShoulder.wasPressedThisFrame) return true;
        }

        if (Joystick.current != null && Joystick.current.trigger.wasPressedThisFrame) return true;

        return false;
    }

#if UNITY_STANDALONE_WIN
    static bool WasRestoreShortcutPressed()
    {
        if (Keyboard.current is { } keyboard &&
            keyboard.f8Key.wasPressedThisFrame)
        {
            return true;
        }

        if (Gamepad.current is not { } gamepad) return false;
        return gamepad.startButton.isPressed && gamepad.selectButton.wasPressedThisFrame;
    }

    static void RestoreUnityWindow()
    {
        IntPtr hwnd = Process.GetCurrentProcess().MainWindowHandle;
        if (hwnd == IntPtr.Zero) return;

        if (IsIconic(hwnd))
            ShowWindow(hwnd, SwRestore);
        else
            ShowWindow(hwnd, SwShow);

        // A brief topmost toggle makes the restore much more reliable on Windows.
        SetWindowPos(hwnd, HwndTopMost, 0, 0, 0, 0, SwpNoMove | SwpNoSize | SwpShowWindow);
        SetWindowPos(hwnd, HwndNotTopMost, 0, 0, 0, 0, SwpNoMove | SwpNoSize | SwpShowWindow);
        SetForegroundWindow(hwnd);
    }

    static readonly IntPtr HwndTopMost = new IntPtr(-1);
    static readonly IntPtr HwndNotTopMost = new IntPtr(-2);

    const int SwShow = 5;
    const int SwRestore = 9;
    const uint SwpNoMove = 0x0002;
    const uint SwpNoSize = 0x0001;
    const uint SwpShowWindow = 0x0040;

    [DllImport("user32.dll")]
    static extern bool ShowWindow(IntPtr hWnd, int nCmdShow);

    [DllImport("user32.dll")]
    static extern bool SetForegroundWindow(IntPtr hWnd);

    [DllImport("user32.dll")]
    static extern bool IsIconic(IntPtr hWnd);

    [DllImport("user32.dll")]
    static extern bool SetWindowPos(
        IntPtr hWnd,
        IntPtr hWndInsertAfter,
        int x,
        int y,
        int cx,
        int cy,
        uint uFlags);
#endif
}
