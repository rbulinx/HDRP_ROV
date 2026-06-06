using UnityEngine;
using UnityEngine.InputSystem;

[RequireComponent(typeof(Rigidbody))]
public class ROVGamepadController : MonoBehaviour
{
    [Header("References (optional)")]
    [Tooltip("未設定でも自動で探します（ROV配下のCamera→MainCameraタグ→名前検索）")]
    public Transform mainCamera;

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

    // 旧: rb.drag = linearDrag;
    // 旧: rb.angularDrag = angularDrag;

    rb.linearDamping = linearDrag;
    rb.angularDamping = angularDrag;

    AutoAssignCameraIfNeeded();

    if (mainCamera != null)
    {
        camPitch = NormalizeAngle(mainCamera.localEulerAngles.x);
        camPitch = Mathf.Clamp(camPitch, minTiltDeg, maxTiltDeg);
        ApplyCameraPitch();
    }
    else
    {
        Debug.LogWarning("[ROVGamepadController] mainCamera が見つかりません。カメラチルトは無効になります。");
    }
}


    void AutoAssignCameraIfNeeded()
    {
        if (mainCamera != null) return;

        // (C) ROV配下の Camera を優先的に探す（子も含む）
        var camInChildren = GetComponentInChildren<Camera>(true);
        if (camInChildren != null)
        {
            mainCamera = camInChildren.transform;
            return;
        }

        // (B) シーン内の MainCamera タグ
        var camByTag = GameObject.FindGameObjectWithTag("MainCamera");
        if (camByTag != null)
        {
            mainCamera = camByTag.transform;
            return;
        }

        // (A) 名前で探す（完全一致）
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
        rb.linearVelocity = worldVel;

        if (Mathf.Abs(yawDegPerSec) > 0.0001f)
        {
            Quaternion delta = Quaternion.Euler(0f, yawDegPerSec * Time.fixedDeltaTime, 0f);
            rb.MoveRotation(rb.rotation * delta);
        }

        // カメラが無いならチルト処理はスキップ
        if (mainCamera == null) return;

        float tiltInput = 0f;

        // RB/LB（バンパー）
        if (pad.rightShoulder.isPressed) tiltInput += 1f;
        if (pad.leftShoulder.isPressed)  tiltInput -= 1f;

        // フォールバック（RT/LT）
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
        
        if (keepUpright) StabilizeUpright();

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
        // 現在のYawは維持して、Pitch/Rollだけ0にした姿勢を目標にする
        float yaw = rb.rotation.eulerAngles.y;
        Quaternion target = Quaternion.Euler(0f, yaw, 0f);

        Quaternion q = target * Quaternion.Inverse(rb.rotation);

        // 角度誤差を軸角で取得
        q.ToAngleAxis(out float angleDeg, out Vector3 axis);
        if (angleDeg > 180f) angleDeg -= 360f;

        // axisが不正な場合の保険
        if (float.IsNaN(axis.x)) return;

        float angleRad = angleDeg * Mathf.Deg2Rad;

        // Yaw成分は抑制しない（世界up方向の回転を除いた角速度を使う）
        Vector3 w = rb.angularVelocity;
        Vector3 wYaw = Vector3.Project(w, Vector3.up);
        Vector3 wNoYaw = w - wYaw;

        // PD制御トルク（加速度モードで質量に依らず効かせる）
        Vector3 torque = axis.normalized * (angleRad * uprightStrength) - wNoYaw * uprightDamping;
        rb.AddTorque(torque, ForceMode.Acceleration);
    }

}
