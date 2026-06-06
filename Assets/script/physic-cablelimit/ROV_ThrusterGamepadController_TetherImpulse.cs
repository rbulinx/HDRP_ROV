using UnityEngine;
using UnityEngine.InputSystem;

[RequireComponent(typeof(Rigidbody))]
public class ROV_ThrusterGamepadController_TetherImpulse : MonoBehaviour
{
    [Header("References (optional)")]
    public Transform mainCamera;

    [Header("Tether Cable (NEW)")]
    public CableLMM_UnderwaterWinch_Collision_ConstraintImpulse tetherCable;
    public bool enableTetherLimit = true;

    [Header("Thrusters (Force-based)")]
    [Tooltip("前後左右（水平面）推力[N]")]
    public float horizontalThrustN = 40f;

    [Tooltip("上下推力[N]")]
    public float verticalThrustN = 35f;

    [Tooltip("ヨー回転トルク[Nm]")]
    public float yawTorqueNm = 12f;

    [Tooltip("入力デッドゾーン")]
    public float inputDeadzone = 0.08f;

    [Header("Camera Tilt")]
    public float tiltSpeedDeg = 45f;
    public float minTiltDeg = -60f;
    public float maxTiltDeg = 30f;
    public bool invertTilt = false;

    [Header("Physics")]
    public bool useGravity = false;
    public float linearDrag = 2.0f;
    public float angularDrag = 2.0f;

    [Header("Upright Stabilizer")]
    public bool keepUpright = true;
    public float uprightStrength = 20f;
    public float uprightDamping = 6f;

    Rigidbody rb;
    float camPitch;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        rb.useGravity = useGravity;

#if UNITY_6000_0_OR_NEWER
        rb.linearDamping = linearDrag;
        rb.angularDamping = angularDrag;
#else
        rb.drag = linearDrag;
        rb.angularDrag = angularDrag;
#endif

        AutoAssignCameraIfNeeded();

        if (mainCamera != null)
        {
            camPitch = NormalizeAngle(mainCamera.localEulerAngles.x);
            camPitch = Mathf.Clamp(camPitch, minTiltDeg, maxTiltDeg);
            ApplyCameraPitch();
        }

        if (tetherCable == null)
        {
#if UNITY_2023_1_OR_NEWER || UNITY_6000_0_OR_NEWER
            tetherCable = Object.FindFirstObjectByType<CableLMM_UnderwaterWinch_Collision_ConstraintImpulse>();
#else
            tetherCable = Object.FindObjectOfType<CableLMM_UnderwaterWinch_Collision_ConstraintImpulse>();
#endif
        }
    }

    void AutoAssignCameraIfNeeded()
    {
        if (mainCamera != null) return;

        var camInChildren = GetComponentInChildren<Camera>(true);
        if (camInChildren != null)
        {
            mainCamera = camInChildren.transform;
            return;
        }

        var camByTag = GameObject.FindGameObjectWithTag("MainCamera");
        if (camByTag != null)
        {
            mainCamera = camByTag.transform;
            return;
        }

        var goByName = GameObject.Find("Main Camera");
        if (goByName != null)
        {
            mainCamera = goByName.transform;
            return;
        }
    }

    void FixedUpdate()
    {
        var pad = Gamepad.current;
        if (pad == null) return;

        Vector2 left = ApplyDeadzone(pad.leftStick.ReadValue(), inputDeadzone);
        Vector2 right = ApplyDeadzone(pad.rightStick.ReadValue(), inputDeadzone);

        // --- 推力入力（物理） ---
        // 左：水平（ローカル XZ）
        Vector3 thrustLocal = new Vector3(left.x, 0f, left.y) * horizontalThrustN;

        // 右：上下（ローカルY）
        float vertical = right.y * verticalThrustN;

        Vector3 thrustWorld = transform.TransformDirection(thrustLocal);
        thrustWorld.y += vertical;

        rb.AddForce(thrustWorld, ForceMode.Force);

        // 右：Yaw（トルク）
        float yaw = right.x * yawTorqueNm;
        if (Mathf.Abs(yaw) > 1e-4f)
        {
            rb.AddTorque(Vector3.up * yaw, ForceMode.Force);
        }

        // --- カメラチルト（任意） ---
        if (mainCamera != null)
        {
            float tiltInput = 0f;

            if (pad.rightShoulder.isPressed) tiltInput += 1f;
            if (pad.leftShoulder.isPressed)  tiltInput -= 1f;

            if (Mathf.Approximately(tiltInput, 0f))
            {
                float rt = pad.rightTrigger.ReadValue();
                float lt = pad.leftTrigger.ReadValue();
                tiltInput = rt - lt;
            }

            if (invertTilt) tiltInput *= -1f;

            if (Mathf.Abs(tiltInput) > 0.0001f)
            {
                camPitch += tiltInput * tiltSpeedDeg * Time.fixedDeltaTime;
                camPitch = Mathf.Clamp(camPitch, minTiltDeg, maxTiltDeg);
                ApplyCameraPitch();
            }
        }

        if (keepUpright) StabilizeUpright();

        // --- ケーブル制限（拘束インパルス） ---
        if (enableTetherLimit && tetherCable != null)
        {
            tetherCable.ApplyTetherConstraintImpulse(rb, Time.fixedDeltaTime);
        }
    }

    void ApplyCameraPitch()
    {
        Vector3 e = mainCamera.localEulerAngles;
        e.x = camPitch;
        e.y = 0f;
        e.z = 0f;
        mainCamera.localEulerAngles = e;
    }

    static Vector2 ApplyDeadzone(Vector2 v, float dz)
    {
        if (v.magnitude < dz) return Vector2.zero;
        return v;
    }

    static float NormalizeAngle(float deg)
    {
        while (deg > 180f) deg -= 360f;
        while (deg < -180f) deg += 360f;
        return deg;
    }

    void StabilizeUpright()
    {
        float yaw = rb.rotation.eulerAngles.y;
        Quaternion target = Quaternion.Euler(0f, yaw, 0f);

        Quaternion q = target * Quaternion.Inverse(rb.rotation);

        q.ToAngleAxis(out float angleDeg, out Vector3 axis);
        if (angleDeg > 180f) angleDeg -= 360f;

        if (float.IsNaN(axis.x)) return;

        float angleRad = angleDeg * Mathf.Deg2Rad;

        Vector3 w = rb.angularVelocity;
        Vector3 wYaw = Vector3.Project(w, Vector3.up);
        Vector3 wNoYaw = w - wYaw;

        Vector3 torque = axis.normalized * (angleRad * uprightStrength) - wNoYaw * uprightDamping;
        rb.AddTorque(torque, ForceMode.Acceleration);
    }
}
