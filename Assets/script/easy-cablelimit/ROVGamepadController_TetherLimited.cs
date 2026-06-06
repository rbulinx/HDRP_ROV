using UnityEngine;
using UnityEngine.InputSystem;

[RequireComponent(typeof(Rigidbody))]
public class ROVGamepadController_TetherLimited : MonoBehaviour
{
    [Header("References (optional)")]
    [Tooltip("未設定でも自動で探します（ROV配下のCamera→MainCameraタグ→名前検索）")]
    public Transform mainCamera;

    [Header("Optional Tether Cable (NEW)")]
    [Tooltip("ケーブル制限をかける場合に設定（CableLMM_UnderwaterWinch_Collision_TensionLimit）")]
    public CableLMM_UnderwaterWinch_Collision_TensionLimit tetherCable;

    [Tooltip("ケーブル制限を有効化")]
    public bool enableTetherLimit = true;

    [Header("ROV Move")]
    public float moveSpeed = 1.5f;
    public float verticalSpeed = 1.0f;
    public float yawSpeedDeg = 90f;
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
    public float uprightStrength = 20f;  // 戻す強さ
    public float uprightDamping  = 6f;   // 揺れ止め

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
        else
        {
            Debug.LogWarning("[ROVGamepadController_TetherLimited] mainCamera が見つかりません。カメラチルトは無効になります。");
        }

        // tetherCable 自動探索（任意）
        if (tetherCable == null)
        {
#if UNITY_2023_1_OR_NEWER || UNITY_6000_0_OR_NEWER
            // 最初に見つかったもの（順序が重要なら First、どれでも良いなら Any の方が速い）
            tetherCable = Object.FindFirstObjectByType<CableLMM_UnderwaterWinch_Collision_TensionLimit>();
#else
            tetherCable = Object.FindObjectOfType<CableLMM_UnderwaterWinch_Collision_TensionLimit>();
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

        // 左：XZ移動（ローカル）
        Vector3 localMove = new Vector3(left.x, 0f, left.y) * moveSpeed;

        // 右：上下移動（ローカルY）
        float vertical = right.y * verticalSpeed;

        // 右：Yaw旋回
        float yawDegPerSec = right.x * yawSpeedDeg;

        Vector3 worldVel = transform.TransformDirection(localMove);
        worldVel.y = vertical;

        // 既存挙動（速度ベース）を維持
#if UNITY_6000_0_OR_NEWER
        rb.linearVelocity = worldVel;
#else
        rb.velocity = worldVel;
#endif

        if (Mathf.Abs(yawDegPerSec) > 0.0001f)
        {
            Quaternion delta = Quaternion.Euler(0f, yawDegPerSec * Time.fixedDeltaTime, 0f);
            rb.MoveRotation(rb.rotation * delta);
        }

        // カメラが無いならチルト処理はスキップ
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

        // --- ケーブル制限（最後に適用して、外向き速度を抑制＆硬制限で戻す） ---
        if (enableTetherLimit && tetherCable != null)
        {
            tetherCable.ApplyTetherToRigidbody(rb, Time.fixedDeltaTime);
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
