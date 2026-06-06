using UnityEngine;
using UnityEngine.InputSystem;

[RequireComponent(typeof(Rigidbody))]
public class SonarRigGamepadController : MonoBehaviour
{
    [Header("Speeds")]
    public float moveSpeed = 2.0f;
    public float verticalSpeed = 1.0f;
    public float yawSpeed = 90.0f;
    public float pitchSpeed = 60.0f;
    public float rollSpeed = 60.0f;

    [Header("Options")]
    [Range(0f, 0.5f)] public float deadzone = 0.12f;
    public bool useRightStickForLook = true;
    public bool enablePitch = false;
    public bool enableRoll = false;

    Rigidbody rb;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        rb.useGravity = false;
        rb.interpolation = RigidbodyInterpolation.Interpolate;
    }

    void FixedUpdate()
    {
        Vector2 move = Vector2.zero;
        Vector2 look = Vector2.zero;
        float upDown = 0f;

        Gamepad gp = Gamepad.current;
        if (gp != null)
        {
            move = gp.leftStick.ReadValue();
            if (useRightStickForLook) look = gp.rightStick.ReadValue();

            float up = gp.rightTrigger.ReadValue();
            float down = gp.leftTrigger.ReadValue();
            upDown = up - down;
        }

        move = ApplyDeadzone(move, deadzone);
        look = ApplyDeadzone(look, deadzone);
        upDown = Mathf.Abs(upDown) < deadzone ? 0f : upDown;

        Vector3 localVel = new Vector3(move.x * moveSpeed, upDown * verticalSpeed, move.y * moveSpeed);
        Vector3 worldVel = transform.TransformDirection(localVel);
        rb.linearVelocity = worldVel;

        float yaw = look.x * yawSpeed;
        float pitch = enablePitch ? -look.y * pitchSpeed : 0f;
        float roll = 0f;

        if (enableRoll && gp != null)
        {
            float rollInput = (gp.rightShoulder.isPressed ? 1f : 0f) - (gp.leftShoulder.isPressed ? 1f : 0f);
            roll = rollInput * rollSpeed;
        }

        Quaternion dq = Quaternion.Euler(
            pitch * Time.fixedDeltaTime,
            yaw * Time.fixedDeltaTime,
            roll * Time.fixedDeltaTime);
        rb.MoveRotation(rb.rotation * dq);
    }

    static Vector2 ApplyDeadzone(Vector2 v, float dz)
    {
        float m = v.magnitude;
        if (m < dz) return Vector2.zero;

        float t = (m - dz) / (1f - dz);
        return v.normalized * t;
    }
}
