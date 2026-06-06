using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.Controls;

public class GripperSimpleGamepad : MonoBehaviour
{
    [Header("Assign finger transforms")]
    public Transform fingerL;
    public Transform fingerR;

    [Header("Finger movement along local X (+/-)")]
    public float maxGap = 0.08f;   // fully open distance between fingers (m)
    public float minGap = 0.00f;   // fully closed
    public float speed = 0.20f;    // gap change per second (m/s)

    [Header("Input Fallbacks")]
    public bool allowJoystickFallback = true;
    public bool allowKeyboardFallback = true;
    public Key keyboardCloseKey = Key.Z;
    public Key keyboardOpenKey = Key.X;

    float gap; // current gap
    float centerX;
    Vector3 fingerLBaseLocalPosition;
    Vector3 fingerRBaseLocalPosition;
    bool hasCachedFingerPositions;
    bool missingFingerWarningShown;
    bool missingInputWarningShown;
    bool childRigidbodyWarningShown;

    void Start()
    {
        EnsureKeyboardDefaults();
        gap = maxGap;
        NeutralizeFingerRigidbodies();
        CacheFingerPositions();
        ApplyGap();
    }

    void Update()
    {
        float close;
        float open;
        if (!TryReadGripInput(out close, out open))
        {
            return;
        }

        float delta = (open - close) * speed * Time.deltaTime;
        gap = Mathf.Clamp(gap + delta, minGap, maxGap);

        ApplyGap();
    }

    void ApplyGap()
    {
        if (!ValidateFingerAssignments()) return;

        if (!hasCachedFingerPositions)
        {
            CacheFingerPositions();
        }

        float half = gap * 0.5f;

        // Move each finger symmetrically around the initial center on local X.
        var lpL = fingerLBaseLocalPosition;
        var lpR = fingerRBaseLocalPosition;

        lpL.x = centerX - half;
        lpR.x = centerX + half;

        fingerL.localPosition = lpL;
        fingerR.localPosition = lpR;
    }

    void CacheFingerPositions()
    {
        if (!ValidateFingerAssignments()) return;

        fingerLBaseLocalPosition = fingerL.localPosition;
        fingerRBaseLocalPosition = fingerR.localPosition;
        centerX = (fingerLBaseLocalPosition.x + fingerRBaseLocalPosition.x) * 0.5f;
        hasCachedFingerPositions = true;
    }

    bool ValidateFingerAssignments()
    {
        if (fingerL != null && fingerR != null) return true;

        if (!missingFingerWarningShown)
        {
            Debug.LogWarning($"{nameof(GripperSimpleGamepad)} on '{name}' needs both fingerL and fingerR assigned.", this);
            missingFingerWarningShown = true;
        }

        return false;
    }

    void NeutralizeFingerRigidbodies()
    {
        NeutralizeFingerRigidbody(fingerL);
        NeutralizeFingerRigidbody(fingerR);
    }

    void NeutralizeFingerRigidbody(Transform finger)
    {
        if (finger == null) return;

        Rigidbody rb = finger.GetComponent<Rigidbody>();
        if (rb == null) return;

        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        rb.isKinematic = true;
        rb.detectCollisions = false;

        if (!childRigidbodyWarningShown)
        {
            Debug.LogWarning($"{nameof(GripperSimpleGamepad)} disabled a child Rigidbody on '{finger.name}' to avoid nested Rigidbody physics conflicts.", finger);
            childRigidbodyWarningShown = true;
        }
    }

    void OnValidate()
    {
        if (maxGap < minGap)
        {
            maxGap = minGap;
        }

        gap = Mathf.Clamp(gap, minGap, maxGap);
        hasCachedFingerPositions = false;
        missingFingerWarningShown = false;
        missingInputWarningShown = false;
        childRigidbodyWarningShown = false;
        EnsureKeyboardDefaults();

        if (fingerL != null && fingerR != null)
        {
            NeutralizeFingerRigidbodies();
            CacheFingerPositions();
        }
    }

    void EnsureKeyboardDefaults()
    {
        if (keyboardCloseKey == Key.None) keyboardCloseKey = Key.Z;
        if (keyboardOpenKey == Key.None) keyboardOpenKey = Key.X;
    }

    bool TryReadGripInput(out float close, out float open)
    {
        if (TryReadKeyboardGripInput(out close, out open))
        {
            missingInputWarningShown = false;
            return true;
        }

        Gamepad gamepad = Gamepad.current;
        if (gamepad == null && Gamepad.all.Count > 0)
        {
            gamepad = Gamepad.all[0];
        }

        if (gamepad != null)
        {
            close = gamepad.leftTrigger.ReadValue();
            open = gamepad.rightTrigger.ReadValue();
            missingInputWarningShown = false;
            return true;
        }

        if (allowJoystickFallback)
        {
            Joystick joystick = Joystick.current;
            if (joystick == null && Joystick.all.Count > 0)
            {
                joystick = Joystick.all[0];
            }

            if (joystick != null)
            {
                close = ReadJoystickButton(joystick, "trigger");
                open = Mathf.Max(
                    ReadJoystickButton(joystick, "button2"),
                    ReadJoystickButton(joystick, "button3"));
                missingInputWarningShown = false;
                return true;
            }
        }

        close = 0f;
        open = 0f;

        if (!missingInputWarningShown)
        {
            Debug.LogWarning($"{nameof(GripperSimpleGamepad)} on '{name}' did not find a Gamepad or fallback input device.", this);
            missingInputWarningShown = true;
        }

        return false;
    }

    bool TryReadKeyboardGripInput(out float close, out float open)
    {
        close = 0f;
        open = 0f;

        if (!allowKeyboardFallback || Keyboard.current == null) return false;

        close = IsKeyPressed(keyboardCloseKey) ? 1f : 0f;
        open = IsKeyPressed(keyboardOpenKey) ? 1f : 0f;
        return close > 0f || open > 0f;
    }

    static bool IsKeyPressed(Key key)
    {
        if (Keyboard.current == null) return false;
        KeyControl control = Keyboard.current[key];
        return control != null && control.isPressed;
    }

    static float ReadJoystickButton(Joystick joystick, string controlName)
    {
        if (joystick == null) return 0f;
        ButtonControl button = joystick.TryGetChildControl<ButtonControl>(controlName);
        return (button != null && button.isPressed) ? 1f : 0f;
    }
}
