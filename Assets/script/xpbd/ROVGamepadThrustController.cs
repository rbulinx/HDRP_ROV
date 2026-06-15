using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.Controls;
using UnityEngine.UI;

// HDRP Water
using UnityEngine.Rendering.HighDefinition;

[RequireComponent(typeof(Rigidbody))]
public class ROVGamepadThrustController : MonoBehaviour
{
    enum InputSource
    {
        None,
        Gamepad,
        Joystick,
        Keyboard
    }

    public enum InputSelectionMode
    {
        AutoPreferGamepad,
        ManualAtStartup,
        ForceGamepad,
        ForceJoystick,
        ForceKeyboard
    }

    public enum ExternalInputDevice
    {
        Gamepad,
        Joystick,
        Keyboard
    }

    [System.Serializable]
    public class HorizontalThruster
    {
        public string name = "Horizontal Thruster";
        public bool enabled = true;
        [Tooltip("Optional. If assigned, position uses this Transform and direction uses its local forward axis.")]
        public Transform mount;
        public Vector3 localPosition = Vector3.zero;
        public Vector3 localDirection = Vector3.forward;
        public float maxForwardThrustN = 49f;
        public float maxReverseThrustN = 49f;
    }

    [System.Serializable]
    public class VerticalThruster
    {
        public string name = "Vertical Thruster";
        public bool enabled = true;
        [Tooltip("Optional. If assigned, position uses this Transform and direction uses its local forward axis.")]
        public Transform mount;
        public Vector3 localPosition = Vector3.zero;
        public Vector3 localDirection = Vector3.up;
        public float maxForwardThrustN = 49f;
        public float maxReverseThrustN = 49f;
    }

    [Header("References (optional)")]
    public Transform mainCamera;

    [Header("Thrust (Force)")]
    public float maxPlanarThrustN = 80f;
    public float maxHeaveThrustN = 80f;
    public float maxYawTorqueNm = 60f;
    public bool useAccelerationMode = false;
    public bool heaveWorldUp = true;

    [Header("Physical Horizontal Thrusters")]
    public HorizontalThruster[] horizontalThrusters = new HorizontalThruster[]
    {
        new HorizontalThruster { name = "Front Left",  localPosition = new Vector3(-0.17f, -0.045f,  0.20f), localDirection = new Vector3( 1f, 0f,  1f).normalized, maxForwardThrustN = 49f, maxReverseThrustN = 49f },
        new HorizontalThruster { name = "Front Right", localPosition = new Vector3( 0.17f, -0.045f,  0.20f), localDirection = new Vector3( 1f, 0f, -1f).normalized, maxForwardThrustN = 49f, maxReverseThrustN = 49f },
        new HorizontalThruster { name = "Rear Left",   localPosition = new Vector3(-0.17f, -0.045f, -0.20f), localDirection = new Vector3( 1f, 0f, -1f).normalized, maxForwardThrustN = 49f, maxReverseThrustN = 49f },
        new HorizontalThruster { name = "Rear Right",  localPosition = new Vector3( 0.17f, -0.045f, -0.20f), localDirection = new Vector3( 1f, 0f,  1f).normalized, maxForwardThrustN = 49f, maxReverseThrustN = 49f },
    };
    [Tooltip("Small damping for the thruster mixer. Increase if thrust allocation jitters near singular layouts.")]
    public float horizontalThrusterMixerRegularization = 0.001f;
    [Range(0f, 1f)]
    [Tooltip("When thrusters saturate, 0 scales translation/yaw equally; 1 preserves yaw first and reduces translation.")]
    public float horizontalThrusterYawPriority = 0.65f;

    [Header("Physical Vertical Thrusters")]
    public VerticalThruster[] verticalThrusters = new VerticalThruster[]
    {
        new VerticalThruster { name = "Left Vertical",  localPosition = new Vector3(-0.12f, 0f, 0f), localDirection = Vector3.up, maxForwardThrustN = 49f, maxReverseThrustN = 49f },
        new VerticalThruster { name = "Right Vertical", localPosition = new Vector3( 0.12f, 0f, 0f), localDirection = Vector3.up, maxForwardThrustN = 49f, maxReverseThrustN = 49f },
    };
    [Tooltip("Small damping for the vertical thruster force mixer.")]
    public float verticalThrusterMixerRegularization = 0.001f;

    [Header("Control Gain")]
    [Tooltip("機体操作の反応倍率。推力・上下推力・ヨーに掛かります。")]
    public float controlResponseGain = 1f;
    public float minControlResponseGain = 0.25f;
    public float maxControlResponseGain = 3f;
    public float controlResponseGainStep = 0.1f;

    [Header("ROV Lights")]
    public Light[] rovLights;
    public ROVLightEmitter[] rovLightEmitters;
    public bool autoFindChildLights = true;
    public float lightIntensityStep = 5000f;
    public float minLightIntensity = 0f;
    public float maxLightIntensity = 60000f;

    [Header("Input")]
    public float inputDeadzone = 0.08f;
    public bool invertJoystickHeave = true;
    [Range(-1f, 1f)] public float joystickHeaveCenter = 0f;
    public float joystickHeaveDeadzone = 0.3f;
    public InputSelectionMode inputSelectionMode = InputSelectionMode.ManualAtStartup;
    [Tooltip("ManualAtStartup時: ONでGamepad、OFFでJoystickを使用")]
    public bool manualUseGamepadAtStartup = true;
    [Tooltip("ManualAtStartup時に起動時入力選択を要求する")]
    public bool requireInputSelectionAtStartup = true;
    [Tooltip("起動時入力選択中に画面ガイドを表示する")]
    public bool showInputSelectionOverlay = true;
    public bool enableKeyboardInput = true;
    public bool enableLegacyInputSelectionOverlay = false;

    [Header("Input Lag")]
    public bool simulateInputLag = true;
    [Tooltip("Input response time constant [s]. Larger values make thrust/yaw commands respond more slowly.")]
    [Range(0f, 2f)] public float inputLagTimeConstantSeconds = 0.25f;
    [Tooltip("Grace period [s] before clearing input when the selected device is briefly not reported by the Input System.")]
    [Range(0f, 1f)] public float inputDeviceLostGraceSeconds = 0.35f;

    [Header("MAVLink Input")]
    [Tooltip("Accept normalized MAVLink control input when no local manual input is active.")]
    public bool acceptMavlinkInput = true;
    [Tooltip("Seconds after the last MAVLink control message before input is treated as lost.")]
    [Range(0.05f, 5f)] public float mavlinkInputTimeoutSeconds = 0.75f;

    [Header("Physics")]
    public float linearDamping = 4.0f;
    public float angularDamping = 3.0f;

    [Header("Heading Lock")]
    public bool enableHeadingLockFeature = true;
    public bool headingLockEnabled = false;
    public float headingLockTorqueNmPerDeg = 0.7f;
    public float headingLockDampingNmPerDegPerSec = 0.5f;
    public float headingLockMaxTorqueNm = 60f;

    [Header("Altitude Hold / Terrain Follow")]
    public bool enableAltitudeHoldFeature = true;
    public bool altitudeHoldEnabled = false;
    public LayerMask altitudeTerrainMask = ~0;
    public bool altitudeRayLocalDown = true;
    public bool altitudeHoldForceAlongRay = true;
    public float altitudeRayLengthMeters = 30f;
    public float altitudeRayOriginUpOffset = 0.25f;
    public float altitudeHoldForceNPerM = 260f;
    public float altitudeHoldDampingNPerMps = 360f;
    public float altitudeHoldMaxForceN = 120f;
    public float altitudeHoldTrimMetersPerSecond = 0.35f;
    public float altitudeHoldManualDeadzone = 0.15f;
    public float altitudeHoldMinTargetMeters = 0.3f;
    public float altitudeHoldMaxTargetMeters = 50f;
    [Range(2f, 60f)] public float altitudeMeasureHz = 15f;

    [Header("Auto Tether Pay")]
    public bool enableAutoTetherPayFeature = true;
    public bool autoTetherPayEnabled = false;
    public CableXPBD xpbdCable;
    public CableLMM_UnderwaterWinch_Collision_TensionLimit tensionLimitCable;
    public bool autoFindTetherCable = true;
    public float autoPayStartTensionNewton = 350f;
    public float autoPayStopTensionNewton = 220f;
    public float autoPayRateMetersPerSecond = 1.3f;
    public float autoPayMaxExtraMeters = 25f;

    [Header("Current")]
    public Vector3 currentVelocity = Vector3.zero;
    public float currentDragLinear = 40f;
    public float currentDragQuadratic = 10f;

    [Header("Speed Limit (optional)")]
    public float maxSpeed = 0f;

    [Header("Camera Tilt")]
    public float tiltSpeedDeg = 45f;
    public float minTiltDeg = -60f;
    public float maxTiltDeg = 30f;
    public bool invertTilt = false;

    [Header("Upright Stabilizer")]
    public bool keepUpright = true;
    public float uprightStrength = 20f;
    public float uprightDamping = 12f;

    // =========================================================
    [Header("Water Surface Clamp (No Breach)")]
    public bool clampToWaterSurface = true;

    [Tooltip("WaterSurface未設定時の水面Y（通常0）")]
    public float fallbackWaterSurfaceY = 0f;

    [Tooltip("上昇入力ブロックのしきい値：基準点を水面より下に維持する[m]")]
    public float keepBelowSurfaceMeters = 0.05f;

    [Tooltip("最終安全柵：これより上に行ったら強制的に戻す[m]")]
    public float hardClampBelowSurfaceMeters = 0.02f;

    [Tooltip("水面近傍では上向きヒーブ入力をブロック")]
    public bool blockUpwardHeaveNearSurface = true;

    [Tooltip("クランプ時に上向き速度を0")]
    public bool zeroUpwardVelocityOnClamp = true;
    // =========================================================

    // =========================================================
    [Header("Wave Rocking (Inspector ON/OFF)")]
    public bool enableWaveRocking = true;

    [Tooltip("波の影響は水面からこの深さ[m]まで（深いほど影響ゼロ）")]
    public float waveAffectMaxDepthMeters = 0.6f;

    [Tooltip("サンプル点を手動指定（空なら自動生成）")]
    public Transform[] waveFloatPoints;

    [Tooltip("コライダーから自動で4点（四隅）を生成します")]
    public bool autoGenerateWavePointsFromColliders = true;

    [Range(0.2f, 1.0f)]
    public float autoPointSpread = 0.9f;

    [Header("Wave Rocking: Tilt (no net vertical hold)")]
    [Tooltip("波面の凹凸差でロール/ピッチを作る強さ[N/m]（合計上下力はほぼ0）")]
    public float waveTiltForceNPerM = 250f;

    [Tooltip("Tiltの上下速度減衰[N/(m/s)]")]
    public float waveTiltDampNPerMS = 180f;

    [Tooltip("1点あたり最大力[N]")]
    public float maxWaveForcePerPoint = 180f;

    [Header("Wave Rocking: Optional Bob (small vertical)")]
    [Tooltip("上下ボブを有効化（小さめ推奨）。TiltだけならOFFでもOK")]
    public bool enableWaveBob = false;

    [Tooltip("ボブの目標沈み量[m]（水面からどのくらい下）")]
    public float waveDesiredSubmergeMeters = 0.10f;

    [Tooltip("ボブばね[N/m]（大きいと水面に貼り付きやすいので控えめ推奨）")]
    public float waveBobSpringNPerM = 200f;

    [Tooltip("ボブ減衰[N/(m/s)]")]
    public float waveBobDampNPerMS = 240f;

    [Tooltip("潜航入力（下向き）中はボブを弱める")]
    public bool suppressBobWhileDiving = true;

    [Tooltip("この値より下向き入力が大きいとボブを抑制（例: -0.2）")]
    public float diveSuppressHeaveThreshold = -0.2f;
    // =========================================================

    // =========================================================
    [Header("HDRP WaterSurface (Assign here)")]
    public WaterSurface waterSurface;
    public float waterQueryError = 0.01f;
    public int waterQueryMaxIterations = 8;
    // =========================================================

    [Header("Collision Audio")]
    public bool enableCollisionSound = true;
    public AudioClip collisionClip;
    [Range(0f, 1f)] public float collisionVolume = 0.75f;
    public float minCollisionImpulse = 1.5f;
    public float collisionCooldown = 0.12f;

    [Header("Collision Count")]
    public bool enableCollisionCount = true;
    public float minCollisionCountImpact = 0.2f;
    public Transform[] ignoredCollisionRoots;
    public string[] ignoredCollisionNameKeywords = { "Gripper", "Finger" };
    // =========================================================

    Rigidbody rb;
    AudioSource collisionAudioSource;
    float camPitch;
    int collisionCount;

    Vector3[] _autoLocalPoints = null;

    WaterSearchParameters _wsp;
    WaterSearchResult _wsr;
    bool _hasWSRCandidate = false;
    InputSource _selectedInput = InputSource.None;
    bool _awaitingStartupInputSelection = false;
    Canvas _startupInputCanvas;
    float _lastCollisionSoundTime = -999f;
    Vector2 _cachedMoveInput = Vector2.zero;
    Vector2 _cachedLookInput = Vector2.zero;
    float _cachedTiltInput = 0f;
    float _lastMoveInputReadTime = -999f;
    Vector2 _mavlinkMoveInput = Vector2.zero;
    Vector2 _mavlinkLookInput = Vector2.zero;
    float _lastMavlinkInputTime = -999f;
    bool _hasMavlinkInput;
    Gamepad _lastPreferredGamepad;
    Joystick _lastPreferredJoystick;
    float _lightIntensity;
    bool _warnedNoRovLights;
    Vector3[] _waveSamplePointsWorld;
    float _headingLockTargetDeg;
    float _headingLockErrorDeg;
    float _altitudeHoldTargetMeters = 2f;
    float _altitudeHoldMeasuredMeters;
    bool _altitudeHoldHasGround;
    float _nextAltitudeMeasureTime;
    readonly RaycastHit[] _altitudeRaycastHits = new RaycastHit[8];
    float _autoPayBaseTargetLength;
    float _autoPayLastTensionN;
    HorizontalThruster[] _thrusterScratchActive;
    Vector3[] _thrusterScratchPositions;
    Vector3[] _thrusterScratchDirections;
    float[] _thrusterScratchForceX;
    float[] _thrusterScratchForceZ;
    float[] _thrusterScratchTorqueY;
    float[] _thrusterScratchCommand;
    float[] _thrusterScratchPlanarCommand;
    float[] _thrusterScratchYawCommand;
    bool _warnedInvalidHorizontalThrusters;
    VerticalThruster[] _verticalThrusterScratchActive;
    Vector3[] _verticalThrusterScratchPositions;
    Vector3[] _verticalThrusterScratchDirections;
    float[] _verticalThrusterScratchForceX;
    float[] _verticalThrusterScratchForceY;
    float[] _verticalThrusterScratchForceZ;
    float[] _verticalThrusterScratchCommand;
    bool _warnedInvalidVerticalThrusters;
    bool _autoPayIsPaying;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        collisionAudioSource = GetComponent<AudioSource>();
        if (collisionAudioSource == null) collisionAudioSource = gameObject.AddComponent<AudioSource>();
        ConfigureCollisionAudioSource();

        if (!enableLegacyInputSelectionOverlay)
        {
            requireInputSelectionAtStartup = false;
            showInputSelectionOverlay = false;
        }

        // ※ Rigidbody.useGravity は一切触りません
        rb.linearDamping = linearDamping;
        rb.angularDamping = angularDamping;

        AutoAssignCameraIfNeeded();
        EnsureRuntimeTuningDefaults();
        AutoAssignLightsIfNeeded();
        InitializeLightIntensity();
        ClampAndApplyControlGain();

        if (mainCamera != null)
        {
            camPitch = NormalizeAngle(mainCamera.localEulerAngles.x);
            camPitch = Mathf.Clamp(camPitch, minTiltDeg, maxTiltDeg);
            ApplyCameraPitch();
        }

        BuildAutoWavePointsIfNeeded();

        _wsp = new WaterSearchParameters();
        _wsr = new WaterSearchResult();

        if (inputSelectionMode == InputSelectionMode.ForceGamepad)
            _selectedInput = InputSource.Gamepad;
        else if (inputSelectionMode == InputSelectionMode.ForceJoystick)
            _selectedInput = InputSource.Joystick;
        else if (inputSelectionMode == InputSelectionMode.ForceKeyboard)
            _selectedInput = InputSource.Keyboard;
        else if (inputSelectionMode == InputSelectionMode.ManualAtStartup)
        {
            if (requireInputSelectionAtStartup)
            {
                _selectedInput = InputSource.None;
                _awaitingStartupInputSelection = true;
                EnsureStartupSelectionUI();
            }
            else
            {
                _selectedInput = manualUseGamepadAtStartup ? InputSource.Gamepad : InputSource.Joystick;
            }
        }
    }

    void OnCollisionEnter(Collision collision)
    {
        TryCountCollision(collision);
        TryPlayCollisionSound(collision);
    }

    void OnCollisionStay(Collision collision)
    {
        TryPlayCollisionSound(collision);
    }

    void Update()
    {
        if (enableLegacyInputSelectionOverlay && _awaitingStartupInputSelection)
        {
            EnsureStartupSelectionUI();
            PollStartupInputSelection();
        }

        HandleRuntimeAdjustmentInput();
        CacheInputForPhysics();
    }

    void FixedUpdate()
    {
        if (_awaitingStartupInputSelection) return;

        Vector2 left = _cachedMoveInput;
        Vector2 right = _cachedLookInput;

        Vector3 localPlanar = new Vector3(left.x, 0f, left.y);
        float heave = right.y;
        float yaw   = right.x;
        float manualHeaveInput = heave;
        bool altitudeControlActive = false;

        if (enableAltitudeHoldFeature)
            UpdateAltitudeHoldMeasurementIfDue();

        if (altitudeHoldEnabled)
        {
            altitudeControlActive = _altitudeHoldHasGround;
            if (altitudeControlActive)
            {
                TrimAltitudeHoldTarget(manualHeaveInput);
                heave = 0f;
            }
        }

        // 水面近傍では上昇入力をブロック（飛び出し抑制）
        if (clampToWaterSurface && blockUpwardHeaveNearSurface && heave > 0f)
        {
            float surfaceY = GetWaterSurfaceYAt(rb.position);
            float limitY = surfaceY - Mathf.Max(0f, keepBelowSurfaceMeters);
            if (rb.position.y >= limitY) heave = 0f;
        }

        // 推力
        float responseGain = Mathf.Clamp(controlResponseGain, minControlResponseGain, maxControlResponseGain);
        Vector3 localForce = localPlanar * (maxPlanarThrustN * responseGain);

        Vector3 heaveForce = heaveWorldUp
            ? Vector3.up * (heave * maxHeaveThrustN * responseGain)
            : transform.up * (heave * maxHeaveThrustN * responseGain);

        ForceMode fm = useAccelerationMode ? ForceMode.Acceleration : ForceMode.Force;
        float yawTorqueNm = yaw * maxYawTorqueNm * responseGain + ComputeHeadingLockTorqueNm() * responseGain;
        ApplyPhysicalHorizontalThrusters(localForce, yawTorqueNm, fm);
        ApplyPhysicalVerticalThrusters(heaveForce + ComputeAltitudeHoldForce(responseGain), fm);

        ApplyAutoTetherPay();
        ApplyCurrentForce();

        // 速度上限
        if (maxSpeed > 0f)
        {
            Vector3 v = rb.linearVelocity;
            float sp = v.magnitude;
            if (sp > maxSpeed)
            {
                Vector3 vClamped = v.normalized * maxSpeed;
                rb.linearVelocity = Vector3.Lerp(v, vClamped, 0.25f);
            }
        }

        // カメラチルト
        if (mainCamera != null)
        {
            float tiltInput = _cachedTiltInput;

            if (invertTilt) tiltInput *= -1f;

            if (Mathf.Abs(tiltInput) > 0.0001f)
            {
                camPitch += tiltInput * tiltSpeedDeg * Time.fixedDeltaTime;
                camPitch = Mathf.Clamp(camPitch, minTiltDeg, maxTiltDeg);
                ApplyCameraPitch();
            }
        }

        // 姿勢安定（波のロール/ピッチを見せたいなら strength を下げる or OFF）
        if (keepUpright) StabilizeUpright();

        // 波の影響（潜航を邪魔しない “Tilt中心”）
        if (enableWaveRocking)
            ApplyWaveRockingNearSurface(heave);

        // 最終安全柵：水面を飛び出さない
        if (clampToWaterSurface)
            EnforceNoBreachClamp();
    }

    void CacheInputForPhysics()
    {
        if (_awaitingStartupInputSelection)
        {
            _cachedMoveInput = Vector2.zero;
            _cachedLookInput = Vector2.zero;
            _cachedTiltInput = 0f;
            return;
        }

        Vector2 move;
        Vector2 look;
        if (TryReadMoveInput(out move, out look))
        {
            _lastMoveInputReadTime = Time.unscaledTime;
            ApplyInputLag(move, look);
        }
        else
        {
            if (Time.unscaledTime - _lastMoveInputReadTime <= inputDeviceLostGraceSeconds)
                return;

            ApplyInputLag(Vector2.zero, Vector2.zero);
        }

        _cachedTiltInput = ReadTiltInput();
    }

    void ApplyInputLag(Vector2 targetMove, Vector2 targetLook)
    {
        if (!simulateInputLag || inputLagTimeConstantSeconds <= 0.0001f)
        {
            _cachedMoveInput = targetMove;
            _cachedLookInput = targetLook;
            return;
        }

        float alpha = 1f - Mathf.Exp(-Time.deltaTime / inputLagTimeConstantSeconds);
        _cachedMoveInput = Vector2.Lerp(_cachedMoveInput, targetMove, alpha);
        _cachedLookInput = Vector2.Lerp(_cachedLookInput, targetLook, alpha);
    }

    // ---------------- Wave (no sticky) ----------------

    void ApplyWaveRockingNearSurface(float heaveInput)
    {
        Vector3[] points = GetWaveSamplePointsWorld();
        if (points == null || points.Length == 0) return;

        // 各点の水面Yを取得
        int n = points.Length;
        float[] surfaceY = new float[n];
        float avgSurface = 0f;

        for (int i = 0; i < n; i++)
        {
            surfaceY[i] = GetWaterSurfaceYAt(points[i]);
            avgSurface += surfaceY[i];
        }
        avgSurface /= Mathf.Max(1, n);

        // ROV中心（基準点）の深さで影響減衰（深いほど0）
        float depthCenter = avgSurface - rb.position.y; // >0:水中
        if (depthCenter <= 0f) return;

        float maxDepth = Mathf.Max(0.001f, waveAffectMaxDepthMeters);
        float k = 1f - Mathf.Clamp01(depthCenter / maxDepth);
        if (k <= 0f) return;

        // 1) Tilt：平均水面からの“凹凸差”だけで力を入れる（合計上下力は理論上0）
        for (int i = 0; i < n; i++)
        {
            float delta = surfaceY[i] - avgSurface; // 平均より高い→その点を押し上げる、低い→押し下げる
            float vy = rb.GetPointVelocity(points[i]).y;

            float f = (delta * waveTiltForceNPerM - vy * waveTiltDampNPerMS) * k;
            f = Mathf.Clamp(f, -maxWaveForcePerPoint, +maxWaveForcePerPoint);

            rb.AddForceAtPosition(Vector3.up * f, points[i], ForceMode.Force);
        }

        // 2) Optional Bob：上下ボブ（弱く、潜航中は抑制）
        if (enableWaveBob)
        {
            float bobK = k;

            if (suppressBobWhileDiving && heaveInput < diveSuppressHeaveThreshold)
            {
                // 潜航操作中はボブをほぼ無効化（“水面に捕まる”の防止）
                bobK *= 0.05f;
            }

            float targetY = avgSurface - Mathf.Max(0f, waveDesiredSubmergeMeters);
            float err = targetY - rb.position.y;
            float vyCenter = rb.linearVelocity.y;

            float bobF = (err * waveBobSpringNPerM - vyCenter * waveBobDampNPerMS) * bobK;
            rb.AddForce(Vector3.up * bobF, ForceMode.Force);
        }
    }

    void ApplyCurrentForce()
    {
        Vector3 rel = currentVelocity - rb.linearVelocity;
        if (rel.sqrMagnitude <= 1e-8f) return;

        Vector3 force = rel * currentDragLinear;
        if (currentDragQuadratic > 0f)
            force += rel * (rel.magnitude * currentDragQuadratic);

        rb.AddForce(force, ForceMode.Force);
    }

    void HandleRuntimeAdjustmentInput()
    {
        if (_awaitingStartupInputSelection) return;

        int lightDelta = 0;
        int gainDelta = 0;

        Gamepad pad = GetPreferredGamepad();
        if (pad != null)
        {
            if (pad.dpad.right.wasPressedThisFrame) lightDelta += 1;
            if (pad.dpad.left.wasPressedThisFrame) lightDelta -= 1;
            if (pad.dpad.up.wasPressedThisFrame) gainDelta += 1;
            if (pad.dpad.down.wasPressedThisFrame) gainDelta -= 1;
        }

        if (IsKeyboardAllowed())
        {
            Keyboard kb = Keyboard.current;
            if (kb != null)
            {
                if (kb.rightBracketKey.wasPressedThisFrame) lightDelta += 1;
                if (kb.leftBracketKey.wasPressedThisFrame) lightDelta -= 1;
                if (kb.equalsKey.wasPressedThisFrame || kb.numpadPlusKey.wasPressedThisFrame) gainDelta += 1;
                if (kb.minusKey.wasPressedThisFrame || kb.numpadMinusKey.wasPressedThisFrame) gainDelta -= 1;
            }
        }

        if (lightDelta != 0)
            AdjustLightIntensity(lightDelta);

        if (gainDelta != 0)
            AdjustControlResponseGain(gainDelta);

        if (WasHeadingLockTogglePressed())
            ToggleHeadingLock();

        if (WasAltitudeHoldTogglePressed())
            ToggleAltitudeHold();

        if (WasAutoTetherPayTogglePressed())
            ToggleAutoTetherPay();
    }

    bool WasHeadingLockTogglePressed()
    {
        if (!enableHeadingLockFeature) return false;

        Gamepad pad = GetPreferredGamepad();
        if (pad != null && pad.buttonNorth.wasPressedThisFrame)
            return true;

        Keyboard kb = Keyboard.current;
        if (kb != null && kb.hKey.wasPressedThisFrame)
            return true;

        return false;
    }

    bool WasAltitudeHoldTogglePressed()
    {
        if (!enableAltitudeHoldFeature) return false;

        Gamepad pad = GetPreferredGamepad();
        if (pad != null && pad.buttonWest.wasPressedThisFrame)
            return true;

        Keyboard kb = Keyboard.current;
        if (kb != null && kb.jKey.wasPressedThisFrame)
            return true;

        return false;
    }

    bool WasAutoTetherPayTogglePressed()
    {
        if (!enableAutoTetherPayFeature) return false;

        Gamepad pad = GetPreferredGamepad();
        if (pad != null && pad.buttonEast.wasPressedThisFrame)
            return true;

        Keyboard kb = Keyboard.current;
        if (kb != null && kb.kKey.wasPressedThisFrame)
            return true;

        return false;
    }

    void ToggleHeadingLock()
    {
        SetHeadingLockEnabled(!headingLockEnabled);
    }

    public void SetHeadingLockEnabled(bool enabled)
    {
        if (!enableHeadingLockFeature)
            enabled = false;

        headingLockEnabled = enabled;
        if (headingLockEnabled)
        {
            _headingLockTargetDeg = rb != null ? rb.rotation.eulerAngles.y : transform.eulerAngles.y;
            _headingLockErrorDeg = 0f;
        }

        Debug.Log($"[ROVInput] Heading lock {(headingLockEnabled ? "ON" : "OFF")} target={_headingLockTargetDeg:0.0} deg");
    }

    float ComputeHeadingLockTorqueNm()
    {
        if (!enableHeadingLockFeature || !headingLockEnabled || rb == null) return 0f;

        float currentHeading = rb.rotation.eulerAngles.y;
        _headingLockErrorDeg = Mathf.DeltaAngle(currentHeading, _headingLockTargetDeg);

        float yawRateDegPerSec = Vector3.Dot(rb.angularVelocity, Vector3.up) * Mathf.Rad2Deg;
        float torque =
            _headingLockErrorDeg * Mathf.Max(0f, headingLockTorqueNmPerDeg) -
            yawRateDegPerSec * Mathf.Max(0f, headingLockDampingNmPerDegPerSec);

        torque = Mathf.Clamp(torque, -Mathf.Max(0f, headingLockMaxTorqueNm), Mathf.Max(0f, headingLockMaxTorqueNm));
        return torque;
    }

    void ToggleAltitudeHold()
    {
        SetAltitudeHoldEnabled(!altitudeHoldEnabled);
    }

    public void SetAltitudeHoldEnabled(bool enabled)
    {
        if (!enableAltitudeHoldFeature)
            enabled = false;

        altitudeHoldEnabled = enabled;
        if (altitudeHoldEnabled)
        {
            if (UpdateAltitudeHoldMeasurement())
            {
                _altitudeHoldTargetMeters = Mathf.Clamp(
                    _altitudeHoldMeasuredMeters,
                    Mathf.Max(0.05f, altitudeHoldMinTargetMeters),
                    Mathf.Max(altitudeHoldMinTargetMeters, altitudeHoldMaxTargetMeters));
            }
            else
            {
                altitudeHoldEnabled = false;
            }
        }

        Debug.Log($"[ROVInput] Alt hold {(altitudeHoldEnabled ? "ON" : "OFF")} target={_altitudeHoldTargetMeters:0.00} m");
    }

    bool UpdateAltitudeHoldMeasurement()
    {
        _altitudeHoldHasGround = TryMeasureTerrainAltitude(out _altitudeHoldMeasuredMeters);
        _nextAltitudeMeasureTime = Time.time + GetAltitudeMeasureInterval();
        return _altitudeHoldHasGround;
    }

    void UpdateAltitudeHoldMeasurementIfDue()
    {
        if (Time.time < _nextAltitudeMeasureTime)
            return;

        UpdateAltitudeHoldMeasurement();
    }

    float GetAltitudeMeasureInterval()
    {
        return 1f / Mathf.Clamp(altitudeMeasureHz, 2f, 60f);
    }

    void TrimAltitudeHoldTarget(float heaveInput)
    {
        if (Mathf.Abs(heaveInput) <= Mathf.Max(0f, altitudeHoldManualDeadzone))
            return;

        _altitudeHoldTargetMeters += heaveInput * Mathf.Max(0f, altitudeHoldTrimMetersPerSecond) * Time.fixedDeltaTime;
        _altitudeHoldTargetMeters = Mathf.Clamp(
            _altitudeHoldTargetMeters,
            Mathf.Max(0.05f, altitudeHoldMinTargetMeters),
            Mathf.Max(altitudeHoldMinTargetMeters, altitudeHoldMaxTargetMeters));
    }

    Vector3 ComputeAltitudeHoldForce(float responseGain)
    {
        if (!enableAltitudeHoldFeature || !altitudeHoldEnabled || !_altitudeHoldHasGround || rb == null)
            return Vector3.zero;

        float error = _altitudeHoldTargetMeters - _altitudeHoldMeasuredMeters;
        Vector3 holdAxis = altitudeHoldForceAlongRay ? GetAltitudeUpDirection() : Vector3.up;
        float verticalVelocity = Vector3.Dot(rb.linearVelocity, holdAxis);
        float force = error * Mathf.Max(0f, altitudeHoldForceNPerM) - verticalVelocity * Mathf.Max(0f, altitudeHoldDampingNPerMps);
        force = Mathf.Clamp(force, -Mathf.Max(0f, altitudeHoldMaxForceN), Mathf.Max(0f, altitudeHoldMaxForceN));

        return holdAxis * (force * responseGain);
    }

    void ToggleAutoTetherPay()
    {
        SetAutoTetherPayEnabled(!autoTetherPayEnabled);
    }

    public void SetAutoTetherPayEnabled(bool enabled)
    {
        if (!enableAutoTetherPayFeature)
            enabled = false;

        ResolveTetherCableIfNeeded();
        if (enabled && xpbdCable == null && tensionLimitCable == null)
            enabled = false;

        autoTetherPayEnabled = enabled;
        _autoPayIsPaying = false;
        _autoPayBaseTargetLength = GetTetherTargetLength();
        Debug.Log($"[ROVInput] Auto tether pay {(autoTetherPayEnabled ? "ON" : "OFF")} base={_autoPayBaseTargetLength:0.00} m");
    }

    void ApplyAutoTetherPay()
    {
        if (!enableAutoTetherPayFeature || !autoTetherPayEnabled)
            return;

        ResolveTetherCableIfNeeded();
        if (xpbdCable == null && tensionLimitCable == null)
            return;

        _autoPayLastTensionN = GetTetherTensionNewton();

        if (_autoPayLastTensionN >= autoPayStartTensionNewton)
            _autoPayIsPaying = true;
        else if (_autoPayLastTensionN <= autoPayStopTensionNewton)
            _autoPayIsPaying = false;

        if (!_autoPayIsPaying)
            return;

        float currentTarget = GetTetherTargetLength();
        float maxTarget = _autoPayBaseTargetLength + Mathf.Max(0f, autoPayMaxExtraMeters);
        float nextTarget = Mathf.Min(maxTarget, currentTarget + Mathf.Max(0f, autoPayRateMetersPerSecond) * Time.fixedDeltaTime);
        SetTetherTargetLength(nextTarget);
    }

    void ResolveTetherCableIfNeeded()
    {
        if (!autoFindTetherCable) return;
        if (xpbdCable != null || tensionLimitCable != null) return;

        xpbdCable = FindFirstObjectByType<CableXPBD>();
        if (xpbdCable == null)
            tensionLimitCable = FindFirstObjectByType<CableLMM_UnderwaterWinch_Collision_TensionLimit>();
    }

    float GetTetherTensionNewton()
    {
        if (xpbdCable != null)
            return xpbdCable.GetTensionNewton();
        if (tensionLimitCable != null)
            return tensionLimitCable.GetTensionNewton();
        return 0f;
    }

    float GetTetherTargetLength()
    {
        if (xpbdCable != null)
            return xpbdCable.targetDeployedLength;
        if (tensionLimitCable != null)
            return tensionLimitCable.targetDeployedLength;
        return 0f;
    }

    void SetTetherTargetLength(float lengthMeters)
    {
        if (xpbdCable != null)
        {
            xpbdCable.SetTargetDeployedLength(lengthMeters);
            return;
        }

        if (tensionLimitCable != null)
            tensionLimitCable.targetDeployedLength = Mathf.Clamp(lengthMeters, tensionLimitCable.minLength, tensionLimitCable.maxLength);
    }

    bool TryMeasureTerrainAltitude(out float altitudeMeters)
    {
        altitudeMeters = 0f;
        if (rb == null) return false;

        Vector3 upDir = GetAltitudeUpDirection();
        Vector3 rayDir = -upDir;
        Vector3 origin = rb.position + upDir * Mathf.Max(0f, altitudeRayOriginUpOffset);
        int hitCount = Physics.RaycastNonAlloc(
            origin,
            rayDir,
            _altitudeRaycastHits,
            Mathf.Max(0.1f, altitudeRayLengthMeters + Mathf.Max(0f, altitudeRayOriginUpOffset)),
            altitudeTerrainMask,
            QueryTriggerInteraction.Ignore);

        if (hitCount <= 0)
            return false;

        float bestDistance = float.PositiveInfinity;
        bool found = false;
        for (int i = 0; i < hitCount; i++)
        {
            RaycastHit hit = _altitudeRaycastHits[i];
            Collider col = hit.collider;
            if (col == null) continue;
            if (col.transform == transform || col.transform.IsChildOf(transform)) continue;
            if (IsIgnoredCollider(col)) continue;

            if (hit.distance < bestDistance)
            {
                bestDistance = hit.distance;
                found = true;
            }
        }

        if (!found)
            return false;

        altitudeMeters = Mathf.Max(0f, bestDistance - Mathf.Max(0f, altitudeRayOriginUpOffset));
        return true;
    }

    Vector3 GetAltitudeUpDirection()
    {
        if (!altitudeRayLocalDown)
            return Vector3.up;

        Vector3 up = transform.up;
        if (up.sqrMagnitude <= 1e-8f)
            return Vector3.up;

        return up.normalized;
    }

    void AdjustLightIntensity(int stepCount)
    {
        AutoAssignLightsIfNeeded();
        if ((rovLights == null || rovLights.Length == 0) &&
            (rovLightEmitters == null || rovLightEmitters.Length == 0))
        {
            if (!_warnedNoRovLights)
            {
                Debug.LogWarning("[ROVInput] No child ROV lights found. Put the light prefabs under the ROV object that has ROVGamepadThrustController, or assign rovLights/rovLightEmitters manually.");
                _warnedNoRovLights = true;
            }
            return;
        }

        SetLightIntensity(_lightIntensity + lightIntensityStep * stepCount);
        Debug.Log($"[ROVInput] Light intensity: {_lightIntensity:0.##}");
    }

    public void SetLightIntensity(float intensity)
    {
        AutoAssignLightsIfNeeded();
        _lightIntensity = Mathf.Clamp(intensity, minLightIntensity, maxLightIntensity);
        ApplyLightIntensity();
    }

    void AdjustControlResponseGain(int stepCount)
    {
        controlResponseGain = Mathf.Clamp(
            controlResponseGain + controlResponseGainStep * stepCount,
            minControlResponseGain,
            maxControlResponseGain);
        Debug.Log($"[ROVInput] Control response gain: {controlResponseGain:0.00}");
    }

    void AutoAssignLightsIfNeeded()
    {
        if (!autoFindChildLights) return;

        Light[] childLights = GetComponentsInChildren<Light>(true);
        if (childLights != null && childLights.Length > 0)
        {
            int count = 0;
            for (int i = 0; i < childLights.Length; i++)
            {
                if (childLights[i] != null && childLights[i].type != LightType.Directional)
                    count++;
            }

            if (count > 0)
            {
                rovLights = new Light[count];
                int writeIndex = 0;
                for (int i = 0; i < childLights.Length; i++)
                {
                    if (childLights[i] != null && childLights[i].type != LightType.Directional)
                        rovLights[writeIndex++] = childLights[i];
                }
            }
        }

        rovLightEmitters = GetComponentsInChildren<ROVLightEmitter>(true);
        _warnedNoRovLights = false;
    }

    void InitializeLightIntensity()
    {
        AutoAssignLightsIfNeeded();
        if (rovLights == null || rovLights.Length == 0)
        {
            _lightIntensity = Mathf.Clamp(lightIntensityStep, minLightIntensity, maxLightIntensity);
            ApplyLightIntensity();
            return;
        }

        for (int i = 0; i < rovLights.Length; i++)
        {
            if (rovLights[i] != null)
            {
                _lightIntensity = Mathf.Clamp(rovLights[i].intensity, minLightIntensity, maxLightIntensity);
                ApplyLightIntensity();
                return;
            }
        }

        _lightIntensity = Mathf.Clamp(lightIntensityStep, minLightIntensity, maxLightIntensity);
        ApplyLightIntensity();
    }

    void ApplyLightIntensity()
    {
        if (rovLights != null)
        {
            for (int i = 0; i < rovLights.Length; i++)
            {
                Light lightComponent = rovLights[i];
                if (lightComponent == null) continue;

                lightComponent.enabled = _lightIntensity > minLightIntensity + 0.001f;
                lightComponent.intensity = _lightIntensity;

                HDAdditionalLightData hdLight = lightComponent.GetComponent<HDAdditionalLightData>();
                if (hdLight != null)
                {
                    hdLight.lightDimmer = lightComponent.enabled ? 1f : 0f;
                    hdLight.volumetricDimmer = lightComponent.enabled ? 1f : 0f;
                }
            }
        }

        float level01 = LightLevel01;
        if (rovLightEmitters == null) return;

        for (int i = 0; i < rovLightEmitters.Length; i++)
        {
            ROVLightEmitter emitter = rovLightEmitters[i];
            if (emitter != null)
                emitter.ApplyLightLevel(level01);
        }
    }

    void ClampAndApplyControlGain()
    {
        minControlResponseGain = Mathf.Max(0.01f, minControlResponseGain);
        maxControlResponseGain = Mathf.Max(minControlResponseGain, maxControlResponseGain);
        controlResponseGainStep = Mathf.Max(0.01f, controlResponseGainStep);
        controlResponseGain = Mathf.Clamp(controlResponseGain, minControlResponseGain, maxControlResponseGain);
    }

    void EnsureRuntimeTuningDefaults()
    {
        if (controlResponseGain <= 0f)
            controlResponseGain = 1f;

        if (minControlResponseGain <= 0f)
            minControlResponseGain = 0.25f;

        if (maxControlResponseGain <= minControlResponseGain)
            maxControlResponseGain = 3f;

        if (controlResponseGainStep <= 0f)
            controlResponseGainStep = 0.1f;

        inputLagTimeConstantSeconds = Mathf.Max(0f, inputLagTimeConstantSeconds);
        mavlinkInputTimeoutSeconds = Mathf.Max(0.05f, mavlinkInputTimeoutSeconds);

        if (lightIntensityStep < 5000f)
            lightIntensityStep = 5000f;

        if (maxLightIntensity < 60000f)
            maxLightIntensity = 60000f;
    }

    void ConfigureCollisionAudioSource()
    {
        if (collisionAudioSource == null) return;

        collisionAudioSource.playOnAwake = false;
        collisionAudioSource.loop = false;
        collisionAudioSource.spatialBlend = 1f;
        collisionAudioSource.rolloffMode = AudioRolloffMode.Linear;
        collisionAudioSource.minDistance = 1f;
        collisionAudioSource.maxDistance = 20f;
    }

    public int CollisionCount
    {
        get { return collisionCount; }
    }

    public float LightIntensity
    {
        get { return _lightIntensity; }
    }

    public float LightLevel01
    {
        get
        {
            float range = maxLightIntensity - minLightIntensity;
            if (range <= 0.0001f) return 0f;
            return Mathf.Clamp01((_lightIntensity - minLightIntensity) / range);
        }
    }

    public float ControlResponseGain
    {
        get { return controlResponseGain; }
    }

    public bool MavlinkInputActive
    {
        get { return IsMavlinkInputFresh(); }
    }

    public float LastMavlinkInputAgeSeconds
    {
        get { return _hasMavlinkInput ? Time.unscaledTime - _lastMavlinkInputTime : float.PositiveInfinity; }
    }

    public void SetMavlinkControlInput(float surge, float sway, float heave, float yaw)
    {
        _mavlinkMoveInput = new Vector2(
            Mathf.Clamp(sway, -1f, 1f),
            Mathf.Clamp(surge, -1f, 1f));
        _mavlinkLookInput = new Vector2(
            Mathf.Clamp(yaw, -1f, 1f),
            Mathf.Clamp(heave, -1f, 1f));
        _lastMavlinkInputTime = Time.unscaledTime;
        _hasMavlinkInput = true;
    }

    public void SetMavlinkControlInput(Vector2 move, Vector2 look)
    {
        SetMavlinkControlInput(move.y, move.x, look.y, look.x);
    }

    public void ClearMavlinkControlInput()
    {
        _mavlinkMoveInput = Vector2.zero;
        _mavlinkLookInput = Vector2.zero;
        _hasMavlinkInput = false;
        _lastMavlinkInputTime = -999f;
    }

    public bool HeadingLockEnabled
    {
        get { return headingLockEnabled; }
    }

    public float HeadingLockTargetDeg
    {
        get { return _headingLockTargetDeg; }
    }

    public float HeadingLockErrorDeg
    {
        get { return _headingLockErrorDeg; }
    }

    public bool AltitudeHoldEnabled
    {
        get { return altitudeHoldEnabled; }
    }

    public bool AltitudeHoldHasGround
    {
        get { return _altitudeHoldHasGround; }
    }

    public float AltitudeHoldTargetMeters
    {
        get { return _altitudeHoldTargetMeters; }
    }

    public float AltitudeHoldMeasuredMeters
    {
        get { return _altitudeHoldMeasuredMeters; }
    }

    public bool AutoTetherPayEnabled
    {
        get { return autoTetherPayEnabled; }
    }

    public bool AutoTetherPayIsPaying
    {
        get { return _autoPayIsPaying; }
    }

    public float AutoTetherPayTensionNewton
    {
        get { return _autoPayLastTensionN; }
    }

    public void ResetCollisionCount()
    {
        collisionCount = 0;
    }

    void TryCountCollision(Collision collision)
    {
        if (!enableCollisionCount) return;
        if (collision == null) return;
        if (IsIgnoredCollision(collision)) return;
        if (GetCollisionImpact(collision) < minCollisionCountImpact) return;

        collisionCount++;
    }

    float GetCollisionImpact(Collision collision)
    {
        if (collision == null) return 0f;

        float impact = collision.relativeVelocity.magnitude;
        if (collision.impulse.sqrMagnitude > 0f)
            impact = Mathf.Max(impact, collision.impulse.magnitude);

        return impact;
    }

    bool IsIgnoredCollision(Collision collision)
    {
        if (collision == null) return false;

        for (int i = 0; i < collision.contactCount; i++)
        {
            ContactPoint contact = collision.GetContact(i);
            if (IsIgnoredCollider(contact.thisCollider) || IsIgnoredCollider(contact.otherCollider))
                return true;
        }

        return IsIgnoredCollider(collision.collider);
    }

    bool IsIgnoredCollider(Collider col)
    {
        if (col == null) return false;

        Transform t = col.transform;
        if (ignoredCollisionRoots != null)
        {
            for (int i = 0; i < ignoredCollisionRoots.Length; i++)
            {
                Transform root = ignoredCollisionRoots[i];
                if (root != null && t.IsChildOf(root))
                    return true;
            }
        }

        while (t != null)
        {
            if (NameMatchesIgnoredKeyword(t.name))
                return true;

            if (t == transform)
                break;

            t = t.parent;
        }

        return false;
    }

    bool NameMatchesIgnoredKeyword(string objectName)
    {
        if (string.IsNullOrEmpty(objectName)) return false;
        if (ignoredCollisionNameKeywords == null) return false;

        for (int i = 0; i < ignoredCollisionNameKeywords.Length; i++)
        {
            string keyword = ignoredCollisionNameKeywords[i];
            if (string.IsNullOrWhiteSpace(keyword)) continue;
            if (objectName.IndexOf(keyword, System.StringComparison.OrdinalIgnoreCase) >= 0)
                return true;
        }

        return false;
    }

    void TryPlayCollisionSound(Collision collision)
    {
        if (!enableCollisionSound) return;
        if (collisionClip == null) return;
        if (collisionAudioSource == null) return;
        if (collision == null) return;
        if (Time.time < _lastCollisionSoundTime + Mathf.Max(0.01f, collisionCooldown)) return;

        float impactStrength = collision.relativeVelocity.magnitude;
        if (collision.impulse.sqrMagnitude > 0f)
            impactStrength = Mathf.Max(impactStrength, collision.impulse.magnitude);

        if (impactStrength < minCollisionImpulse) return;

        float volumeScale = Mathf.Clamp01(impactStrength / Mathf.Max(minCollisionImpulse, 0.01f));
        collisionAudioSource.PlayOneShot(collisionClip, collisionVolume * volumeScale);
        _lastCollisionSoundTime = Time.time;
    }

    // ---------------- Water ----------------

    float GetWaterSurfaceYAt(Vector3 worldPos)
    {
        if (waterSurface != null && TryGetHDRPWaterSurface(worldPos, out float y))
            return y;

        return fallbackWaterSurfaceY;
    }

    bool TryGetHDRPWaterSurface(Vector3 worldPos, out float surfaceY)
    {
        // WaterSurface 側で Script Interactions (cpuSimulation) が有効である必要があります
        if (!_hasWSRCandidate)
        {
            _wsr.candidateLocationWS = worldPos;
            _hasWSRCandidate = true;
        }

        _wsp.startPositionWS = _wsr.candidateLocationWS;
        _wsp.targetPositionWS = worldPos;
        _wsp.error = waterQueryError;
        _wsp.maxIterations = waterQueryMaxIterations;
        _wsp.outputNormal = false;

        bool ok = waterSurface.ProjectPointOnWaterSurface(_wsp, out _wsr);
        if (ok)
        {
            surfaceY = _wsr.projectedPositionWS.y;
            return true;
        }

        _hasWSRCandidate = false;
        surfaceY = fallbackWaterSurfaceY;
        return false;
    }

    void EnforceNoBreachClamp()
    {
        float surfaceY = GetWaterSurfaceYAt(rb.position);
        float hardLimitY = surfaceY - Mathf.Max(0f, hardClampBelowSurfaceMeters);

        Vector3 p = rb.position;
        if (p.y > hardLimitY)
        {
            p.y = hardLimitY;
            rb.MovePosition(p);

            if (zeroUpwardVelocityOnClamp)
            {
                Vector3 v = rb.linearVelocity;
                if (v.y > 0f) v.y = 0f;
                rb.linearVelocity = v;
            }
        }
    }

    // ---------------- Points ----------------

    Vector3[] GetWaveSamplePointsWorld()
    {
        if (waveFloatPoints != null && waveFloatPoints.Length > 0)
        {
            int n = waveFloatPoints.Length;
            if (_waveSamplePointsWorld == null || _waveSamplePointsWorld.Length != n)
                _waveSamplePointsWorld = new Vector3[n];
            for (int i = 0; i < n; i++)
                _waveSamplePointsWorld[i] = (waveFloatPoints[i] != null) ? waveFloatPoints[i].position : transform.position;
            return _waveSamplePointsWorld;
        }

        if (_autoLocalPoints != null && _autoLocalPoints.Length > 0)
        {
            int n = _autoLocalPoints.Length;
            if (_waveSamplePointsWorld == null || _waveSamplePointsWorld.Length != n)
                _waveSamplePointsWorld = new Vector3[n];
            for (int i = 0; i < n; i++)
                _waveSamplePointsWorld[i] = TransformLocalPointWithoutScale(_autoLocalPoints[i]);
            return _waveSamplePointsWorld;
        }

        if (_waveSamplePointsWorld == null || _waveSamplePointsWorld.Length != 1)
            _waveSamplePointsWorld = new Vector3[1];
        _waveSamplePointsWorld[0] = transform.position;
        return _waveSamplePointsWorld;
    }

    void BuildAutoWavePointsIfNeeded()
    {
        if (!autoGenerateWavePointsFromColliders)
        {
            _autoLocalPoints = null;
            return;
        }

        var cols = GetComponentsInChildren<Collider>(true);
        if (cols == null || cols.Length == 0)
        {
            _autoLocalPoints = null;
            return;
        }

        Bounds b = cols[0].bounds;
        for (int i = 1; i < cols.Length; i++) b.Encapsulate(cols[i].bounds);

        Vector3 centerLocal = InverseTransformPointWithoutScale(b.center);
        Vector3 ext = b.extents * autoPointSpread;

        _autoLocalPoints = new Vector3[4];
        _autoLocalPoints[0] = centerLocal + new Vector3(+ext.x, 0f, +ext.z);
        _autoLocalPoints[1] = centerLocal + new Vector3(-ext.x, 0f, +ext.z);
        _autoLocalPoints[2] = centerLocal + new Vector3(+ext.x, 0f, -ext.z);
        _autoLocalPoints[3] = centerLocal + new Vector3(-ext.x, 0f, -ext.z);
    }

    // ---------------- Camera / Upright ----------------

    void AutoAssignCameraIfNeeded()
    {
        if (mainCamera != null) return;

        var camInChildren = GetComponentInChildren<Camera>(true);
        if (camInChildren != null) { mainCamera = camInChildren.transform; return; }

        var camByTag = GameObject.FindGameObjectWithTag("MainCamera");
        if (camByTag != null) { mainCamera = camByTag.transform; return; }

        var goByName = GameObject.Find("Main Camera");
        if (goByName != null) mainCamera = goByName.transform;
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

    bool TryReadMoveInput(out Vector2 move, out Vector2 look)
    {
        if (_awaitingStartupInputSelection)
        {
            move = Vector2.zero;
            look = Vector2.zero;
            return false;
        }

        var pad = GetPreferredGamepad();
        var joystick = GetPreferredJoystick();

        Vector2 padMove = Vector2.zero;
        Vector2 padLook = Vector2.zero;
        if (pad != null)
        {
            padMove = ApplyDeadzone(pad.leftStick.ReadValue(), inputDeadzone);
            padLook = ApplyDeadzone(pad.rightStick.ReadValue(), inputDeadzone);
        }

        Vector2 joyMove = Vector2.zero;
        Vector2 joyLook = Vector2.zero;
        if (joystick != null)
        {
            joyMove = ApplyDeadzone(joystick.stick.ReadValue(), inputDeadzone);
            float yaw = joystick.twist != null ? joystick.twist.ReadValue() : 0f;
            float heave = ReadJoystickHeave(joystick);
            joyLook = ApplyDeadzone(new Vector2(yaw, heave), inputDeadzone);
        }

        Vector2 kbMove = Vector2.zero;
        Vector2 kbLook = Vector2.zero;
        if (IsKeyboardAllowed())
        {
            ReadKeyboardMoveLook(out kbMove, out kbLook);
            kbMove = ApplyDeadzone(kbMove, inputDeadzone);
            kbLook = ApplyDeadzone(kbLook, inputDeadzone);
        }

        // Both connected: prefer gamepad when both have active input.
        if (pad != null && HasAnyMoveInput(padMove, padLook))
        {
            move = padMove;
            look = padLook;
            return true;
        }

        if (joystick != null && HasAnyMoveInput(joyMove, joyLook))
        {
            move = joyMove;
            look = joyLook;
            return true;
        }

        if (IsKeyboardAllowed() && HasAnyMoveInput(kbMove, kbLook))
        {
            move = kbMove;
            look = kbLook;
            return true;
        }

        if (TryReadMavlinkInput(out move, out look))
            return true;

        if (pad != null)
        {
            move = padMove;
            look = padLook;
            return true;
        }

        if (joystick != null)
        {
            move = joyMove;
            look = joyLook;
            return true;
        }

        if (IsKeyboardAllowed())
        {
            move = kbMove;
            look = kbLook;
            return true;
        }

        move = Vector2.zero;
        look = Vector2.zero;
        return false;
    }

    bool TryReadMavlinkInput(out Vector2 move, out Vector2 look)
    {
        if (IsMavlinkInputFresh())
        {
            move = _mavlinkMoveInput;
            look = _mavlinkLookInput;
            return true;
        }

        move = Vector2.zero;
        look = Vector2.zero;
        return false;
    }

    bool IsMavlinkInputFresh()
    {
        if (!acceptMavlinkInput || !_hasMavlinkInput)
            return false;

        float timeout = Mathf.Max(0.05f, mavlinkInputTimeoutSeconds);
        return Time.unscaledTime - _lastMavlinkInputTime <= timeout;
    }

    float ReadTiltInput()
    {
        var pad = GetPreferredGamepad();
        var joystick = GetPreferredJoystick();

        float padTilt = ReadGamepadTiltInput(pad);
        float joyTilt = ReadJoystickTiltInput(joystick);
        float kbTilt = ReadKeyboardTiltInput();

        if (pad != null && Mathf.Abs(padTilt) > 0.0001f) return padTilt;
        if (joystick != null && Mathf.Abs(joyTilt) > 0.0001f) return joyTilt;
        if (IsKeyboardAllowed() && Mathf.Abs(kbTilt) > 0.0001f) return kbTilt;
        if (pad != null) return padTilt;
        if (joystick != null) return joyTilt;
        if (IsKeyboardAllowed()) return kbTilt;
        return 0f;
    }

    static bool HasAnyMoveInput(Vector2 move, Vector2 look)
    {
        return move.sqrMagnitude > 0f || look.sqrMagnitude > 0f;
    }

    void PollStartupInputSelection()
    {
        Keyboard kb = Keyboard.current;
        if (kb != null)
        {
            if (kb.digit1Key.wasPressedThisFrame || kb.numpad1Key.wasPressedThisFrame)
            {
                SetSelectedInput(InputSource.Gamepad);
                return;
            }

            if (kb.digit2Key.wasPressedThisFrame || kb.numpad2Key.wasPressedThisFrame)
            {
                SetSelectedInput(InputSource.Joystick);
                return;
            }

            if (kb.digit3Key.wasPressedThisFrame || kb.numpad3Key.wasPressedThisFrame)
            {
                SetSelectedInput(InputSource.Keyboard);
                return;
            }
        }

        Gamepad pad = Gamepad.current;
        if (pad != null && (pad.buttonSouth.wasPressedThisFrame || pad.startButton.wasPressedThisFrame))
        {
            SetSelectedInput(InputSource.Gamepad);
            return;
        }

        Joystick joy = Joystick.current;
        if (joy != null && joy.trigger.wasPressedThisFrame)
        {
            SetSelectedInput(InputSource.Joystick);
        }
    }

    void SetSelectedInput(InputSource source)
    {
        _selectedInput = source;
        _awaitingStartupInputSelection = false;
        DestroyStartupSelectionUI();
        Debug.Log($"[ROVInput] Selected input: {source}");
    }

    Gamepad GetPreferredGamepad()
    {
        if (inputSelectionMode == InputSelectionMode.ForceJoystick || inputSelectionMode == InputSelectionMode.ForceKeyboard) return null;
        if (inputSelectionMode == InputSelectionMode.ManualAtStartup &&
            (_selectedInput == InputSource.Joystick || _selectedInput == InputSource.Keyboard)) return null;

        Gamepad pad = Gamepad.current;
        if (pad == null && IsKnownGamepad(_lastPreferredGamepad))
            pad = _lastPreferredGamepad;
        if (pad == null && Gamepad.all.Count > 0)
            pad = Gamepad.all[0];

        if (pad != null)
            _lastPreferredGamepad = pad;

        return pad;
    }

    Joystick GetPreferredJoystick()
    {
        if (inputSelectionMode == InputSelectionMode.ForceGamepad || inputSelectionMode == InputSelectionMode.ForceKeyboard) return null;
        if (inputSelectionMode == InputSelectionMode.ManualAtStartup &&
            (_selectedInput == InputSource.Gamepad || _selectedInput == InputSource.Keyboard)) return null;

        Joystick joystick = Joystick.current;
        if (joystick == null && IsKnownJoystick(_lastPreferredJoystick))
            joystick = _lastPreferredJoystick;
        if (joystick == null && Joystick.all.Count > 0)
            joystick = Joystick.all[0];

        if (joystick != null)
            _lastPreferredJoystick = joystick;

        return joystick;
    }

    static bool IsKnownGamepad(Gamepad target)
    {
        if (target == null) return false;
        var pads = Gamepad.all;
        for (int i = 0; i < pads.Count; i++)
        {
            if (pads[i] == target) return true;
        }
        return false;
    }

    static bool IsKnownJoystick(Joystick target)
    {
        if (target == null) return false;
        var joysticks = Joystick.all;
        for (int i = 0; i < joysticks.Count; i++)
        {
            if (joysticks[i] == target) return true;
        }
        return false;
    }

    static float ReadGamepadTiltInput(Gamepad pad)
    {
        if (pad == null) return 0f;

        float tiltInput = 0f;

        if (pad.rightShoulder.isPressed) tiltInput += 1f;
        if (pad.leftShoulder.isPressed) tiltInput -= 1f;

        return tiltInput;
    }

    static float ReadJoystickTiltInput(Joystick joystick)
    {
        if (joystick == null) return 0f;

        float hatY = ReadJoystickHatY(joystick);
        if (Mathf.Abs(hatY) > 0.0001f) return Mathf.Clamp(hatY, -1f, 1f);

        // Fallback for devices that expose camera tilt on buttons.
        float tiltInput = 0f;
        ButtonControl button3 = joystick.TryGetChildControl<ButtonControl>("button3");
        ButtonControl button2 = joystick.TryGetChildControl<ButtonControl>("button2");
        if (button3 != null && button3.isPressed) tiltInput += 1f;
        if (button2 != null && button2.isPressed) tiltInput -= 1f;
        return tiltInput;
    }

    static float ReadJoystickHatY(Joystick joystick)
    {
        if (joystick.hatswitch != null)
            return joystick.hatswitch.ReadValue().y;

        Vector2Control hatVec =
            joystick.TryGetChildControl<Vector2Control>("hatswitch") ??
            joystick.TryGetChildControl<Vector2Control>("hat") ??
            joystick.TryGetChildControl<Vector2Control>("pov") ??
            joystick.TryGetChildControl<Vector2Control>("dpad");
        if (hatVec != null)
            return hatVec.ReadValue().y;

        AxisControl hatY =
            joystick.TryGetChildControl<AxisControl>("hatswitch/y") ??
            joystick.TryGetChildControl<AxisControl>("hat/y") ??
            joystick.TryGetChildControl<AxisControl>("pov/y") ??
            joystick.TryGetChildControl<AxisControl>("dpad/y");
        if (hatY != null)
            return hatY.ReadValue();

        ButtonControl hatUp =
            joystick.TryGetChildControl<ButtonControl>("hat/up") ??
            joystick.TryGetChildControl<ButtonControl>("pov/up") ??
            joystick.TryGetChildControl<ButtonControl>("dpad/up");
        ButtonControl hatDown =
            joystick.TryGetChildControl<ButtonControl>("hat/down") ??
            joystick.TryGetChildControl<ButtonControl>("pov/down") ??
            joystick.TryGetChildControl<ButtonControl>("dpad/down");

        float y = 0f;
        if (hatUp != null && hatUp.isPressed) y += 1f;
        if (hatDown != null && hatDown.isPressed) y -= 1f;
        return y;
    }

    float ReadJoystickHeave(Joystick joystick)
    {
        if (joystick == null) return 0f;

        AxisControl axis =
            joystick.TryGetChildControl<AxisControl>("slider") ??
            joystick.TryGetChildControl<AxisControl>("throttle") ??
            joystick.TryGetChildControl<AxisControl>("z") ??
            joystick.TryGetChildControl<AxisControl>("rz");

        if (axis != null)
        {
            return NormalizeJoystickHeave(axis.ReadValue());
        }

        return 0f;
    }

    bool IsKeyboardAllowed()
    {
        if (!enableKeyboardInput) return false;
        if (Keyboard.current == null) return false;
        if (inputSelectionMode == InputSelectionMode.ForceGamepad || inputSelectionMode == InputSelectionMode.ForceJoystick) return false;
        if (inputSelectionMode == InputSelectionMode.ManualAtStartup &&
            (_selectedInput == InputSource.Gamepad || _selectedInput == InputSource.Joystick)) return false;
        return true;
    }

    void ReadKeyboardMoveLook(out Vector2 move, out Vector2 look)
    {
        Keyboard kb = Keyboard.current;
        if (kb == null)
        {
            move = Vector2.zero;
            look = Vector2.zero;
            return;
        }

        float moveX = 0f;
        if (kb.aKey.isPressed || kb.leftArrowKey.isPressed) moveX -= 1f;
        if (kb.dKey.isPressed || kb.rightArrowKey.isPressed) moveX += 1f;

        float moveY = 0f;
        if (kb.sKey.isPressed) moveY -= 1f;
        if (kb.wKey.isPressed) moveY += 1f;

        float yaw = 0f;
        if (kb.qKey.isPressed) yaw -= 1f;
        if (kb.eKey.isPressed) yaw += 1f;

        float heave = 0f;
        if (kb.fKey.isPressed) heave -= 1f;
        if (kb.rKey.isPressed || kb.spaceKey.isPressed) heave += 1f;

        move = new Vector2(moveX, moveY);
        if (move.sqrMagnitude > 1f) move.Normalize();

        look = new Vector2(yaw, heave);
        if (look.sqrMagnitude > 1f) look.Normalize();
    }

    float ReadKeyboardTiltInput()
    {
        if (!IsKeyboardAllowed()) return 0f;
        Keyboard kb = Keyboard.current;
        if (kb == null) return 0f;

        float tilt = 0f;
        if (kb.upArrowKey.isPressed || kb.pageUpKey.isPressed) tilt += 1f;
        if (kb.downArrowKey.isPressed || kb.pageDownKey.isPressed) tilt -= 1f;
        return tilt;
    }

    float NormalizeJoystickHeave(float value)
    {
        value = NormalizeRawJoystickAxis(value);
        value -= joystickHeaveCenter;

        if (Mathf.Abs(value) < joystickHeaveDeadzone)
        {
            return 0f;
        }

        if (invertJoystickHeave)
        {
            value = -value;
        }

        return Mathf.Clamp(value, -1f, 1f);
    }

    static float NormalizeRawJoystickAxis(float value)
    {
        if (value >= 0f && value <= 1f)
        {
            return value * 2f - 1f;
        }

        return Mathf.Clamp(value, -1f, 1f);
    }

    void EnsureStartupSelectionUI()
    {
        if (!enableLegacyInputSelectionOverlay) return;
        if (!_awaitingStartupInputSelection || !showInputSelectionOverlay) return;
        if (_startupInputCanvas != null) return;

        GameObject canvasObj = new GameObject("ROV_InputSelectCanvas");
        _startupInputCanvas = canvasObj.AddComponent<Canvas>();
        _startupInputCanvas.renderMode = RenderMode.ScreenSpaceOverlay;
        _startupInputCanvas.sortingOrder = 2100;

        CanvasScaler scaler = canvasObj.AddComponent<CanvasScaler>();
        scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
        canvasObj.AddComponent<GraphicRaycaster>();

        GameObject panel = CreateUiObject("Panel", canvasObj.transform);
        Image panelImage = panel.AddComponent<Image>();
        panelImage.color = new Color(0f, 0.05f, 0.1f, 1f);

        RectTransform panelRect = panel.GetComponent<RectTransform>();
        panelRect.anchorMin = new Vector2(0.5f, 0.5f);
        panelRect.anchorMax = new Vector2(0.5f, 0.5f);
        panelRect.pivot = new Vector2(0.5f, 0.5f);
        panelRect.sizeDelta = new Vector2(860f, 240f);

        Text title = CreateText("Title", panel.transform, 34, FontStyle.Bold);
        title.text = "Select Input Device";
        SetRect(title.rectTransform, new Vector2(0.5f, 0.78f), new Vector2(540f, 56f));

        Text hint = CreateText("Hint", panel.transform, 20, FontStyle.Normal);
        hint.text = "Gamepad: A/Start or key 1   |   Joystick: Trigger or key 2   |   Keyboard: key 3";
        SetRect(hint.rectTransform, new Vector2(0.5f, 0.60f), new Vector2(780f, 44f));

        Button gamepadButton = CreateButton("GamepadButton", panel.transform, new Vector2(-275f, -30f), new Vector2(240f, 64f), "Gamepad");
        gamepadButton.onClick.AddListener(() => SetSelectedInput(InputSource.Gamepad));

        Button joystickButton = CreateButton("JoystickButton", panel.transform, new Vector2(0f, -30f), new Vector2(240f, 64f), "Joystick");
        joystickButton.onClick.AddListener(() => SetSelectedInput(InputSource.Joystick));

        Button keyboardButton = CreateButton("KeyboardButton", panel.transform, new Vector2(275f, -30f), new Vector2(240f, 64f), "Keyboard");
        keyboardButton.onClick.AddListener(() => SetSelectedInput(InputSource.Keyboard));
    }

    void DestroyStartupSelectionUI()
    {
        if (_startupInputCanvas == null) return;
        Destroy(_startupInputCanvas.gameObject);
        _startupInputCanvas = null;
    }

    void OnDisable()
    {
        DestroyStartupSelectionUI();
    }

    static GameObject CreateUiObject(string name, Transform parent)
    {
        GameObject go = new GameObject(name);
        go.transform.SetParent(parent, false);
        go.AddComponent<RectTransform>();
        return go;
    }

    static Text CreateText(string name, Transform parent, int fontSize, FontStyle fontStyle)
    {
        GameObject go = CreateUiObject(name, parent);
        Text text = go.AddComponent<Text>();
        text.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
        text.fontSize = fontSize;
        text.fontStyle = fontStyle;
        text.color = Color.white;
        text.alignment = TextAnchor.MiddleCenter;
        return text;
    }

    static Button CreateButton(string name, Transform parent, Vector2 anchoredPosition, Vector2 size, string label)
    {
        GameObject buttonObject = CreateUiObject(name, parent);
        Image image = buttonObject.AddComponent<Image>();
        image.color = new Color(0.12f, 0.5f, 0.7f, 0.95f);

        Button button = buttonObject.AddComponent<Button>();
        ColorBlock colors = button.colors;
        colors.normalColor = image.color;
        colors.highlightedColor = new Color(0.2f, 0.62f, 0.84f, 1f);
        colors.pressedColor = new Color(0.08f, 0.35f, 0.5f, 1f);
        colors.selectedColor = colors.highlightedColor;
        button.colors = colors;

        RectTransform rect = buttonObject.GetComponent<RectTransform>();
        rect.anchorMin = new Vector2(0.5f, 0.5f);
        rect.anchorMax = new Vector2(0.5f, 0.5f);
        rect.anchoredPosition = anchoredPosition;
        rect.sizeDelta = size;

        Text text = CreateText("Label", buttonObject.transform, 28, FontStyle.Bold);
        text.text = label;
        RectTransform textRect = text.rectTransform;
        textRect.anchorMin = Vector2.zero;
        textRect.anchorMax = Vector2.one;
        textRect.offsetMin = Vector2.zero;
        textRect.offsetMax = Vector2.zero;

        return button;
    }

    static void SetRect(RectTransform rect, Vector2 anchor, Vector2 size)
    {
        rect.anchorMin = anchor;
        rect.anchorMax = anchor;
        rect.pivot = new Vector2(0.5f, 0.5f);
        rect.anchoredPosition = Vector2.zero;
        rect.sizeDelta = size;
    }

    void ApplyPhysicalHorizontalThrusters(Vector3 desiredLocalForce, float desiredYawTorqueNm, ForceMode forceMode)
    {
        if (!TryBuildHorizontalThrusterMixer(
                out HorizontalThruster[] thrusters,
                out Vector3[] localPositions,
                out Vector3[] localDirections,
                out float[] forceX,
                out float[] forceZ,
                out float[] torqueY,
                out int count))
        {
            WarnInvalidHorizontalThrustersOnce();
            return;
        }

        float b0 = desiredLocalForce.x;
        float b1 = desiredLocalForce.z;
        float b2 = desiredYawTorqueNm;

        float m00 = Mathf.Max(0f, horizontalThrusterMixerRegularization);
        float m01 = 0f;
        float m02 = 0f;
        float m11 = Mathf.Max(0f, horizontalThrusterMixerRegularization);
        float m12 = 0f;
        float m22 = Mathf.Max(0f, horizontalThrusterMixerRegularization);

        for (int i = 0; i < count; i++)
        {
            float ax = forceX[i];
            float az = forceZ[i];
            float ay = torqueY[i];

            m00 += ax * ax;
            m01 += ax * az;
            m02 += ax * ay;
            m11 += az * az;
            m12 += az * ay;
            m22 += ay * ay;
        }

        if (!SolveSymmetric3x3(m00, m01, m02, m11, m12, m22, b0, b1, 0f, out Vector3 planarMixer) ||
            !SolveSymmetric3x3(m00, m01, m02, m11, m12, m22, 0f, 0f, b2, out Vector3 yawMixer))
        {
            WarnInvalidHorizontalThrustersOnce();
            return;
        }

        for (int i = 0; i < count; i++)
        {
            _thrusterScratchPlanarCommand[i] =
                forceX[i] * planarMixer.x + forceZ[i] * planarMixer.y + torqueY[i] * planarMixer.z;
            _thrusterScratchYawCommand[i] =
                forceX[i] * yawMixer.x + forceZ[i] * yawMixer.y + torqueY[i] * yawMixer.z;
            _thrusterScratchCommand[i] = _thrusterScratchPlanarCommand[i] + _thrusterScratchYawCommand[i];
        }

        float uniformScale = GetCommonThrusterScale(thrusters, _thrusterScratchCommand, count);
        float yawScale = GetCommonThrusterScale(thrusters, _thrusterScratchYawCommand, count);
        float planarScale = GetPlanarScaleWithYawReserved(thrusters, _thrusterScratchPlanarCommand, _thrusterScratchYawCommand, yawScale, count);
        float yawPriority = Mathf.Clamp01(horizontalThrusterYawPriority);

        for (int i = 0; i < count; i++)
        {
            float uniformThrust = _thrusterScratchCommand[i] * uniformScale;
            float yawPriorityThrust =
                _thrusterScratchYawCommand[i] * yawScale +
                _thrusterScratchPlanarCommand[i] * planarScale;
            float thrust = Mathf.Lerp(uniformThrust, yawPriorityThrust, yawPriority);
            if (Mathf.Abs(thrust) <= 1e-4f)
                continue;

            Vector3 worldPosition = TransformLocalPointWithoutScale(localPositions[i]);
            Vector3 worldDirection = transform.TransformDirection(localDirections[i]).normalized;
            rb.AddForceAtPosition(worldDirection * thrust, worldPosition, forceMode);
        }
    }

    bool TryBuildHorizontalThrusterMixer(
        out HorizontalThruster[] activeThrusters,
        out Vector3[] localPositions,
        out Vector3[] localDirections,
        out float[] forceX,
        out float[] forceZ,
        out float[] torqueY,
        out int count)
    {
        int capacity = horizontalThrusters != null ? horizontalThrusters.Length : 0;
        EnsureHorizontalThrusterScratchCapacity(capacity);

        activeThrusters = _thrusterScratchActive;
        localPositions = _thrusterScratchPositions;
        localDirections = _thrusterScratchDirections;
        forceX = _thrusterScratchForceX;
        forceZ = _thrusterScratchForceZ;
        torqueY = _thrusterScratchTorqueY;
        count = 0;

        if (capacity == 0)
            return false;

        for (int i = 0; i < horizontalThrusters.Length; i++)
        {
            HorizontalThruster thruster = horizontalThrusters[i];
            if (thruster == null || !thruster.enabled)
                continue;

            Vector3 localPosition;
            Vector3 localDirection;
            if (thruster.mount != null)
            {
                localPosition = InverseTransformPointWithoutScale(thruster.mount.position);
                localDirection = transform.InverseTransformDirection(thruster.mount.forward);
            }
            else
            {
                localPosition = thruster.localPosition;
                localDirection = thruster.localDirection;
            }

            localDirection.y = 0f;
            if (localDirection.sqrMagnitude <= 1e-8f)
                continue;

            localDirection.Normalize();

            activeThrusters[count] = thruster;
            localPositions[count] = localPosition;
            localDirections[count] = localDirection;
            forceX[count] = localDirection.x;
            forceZ[count] = localDirection.z;
            torqueY[count] = Vector3.Cross(localPosition, localDirection).y;
            count++;
        }

        return count > 0;
    }

    void WarnInvalidHorizontalThrustersOnce()
    {
        if (_warnedInvalidHorizontalThrusters)
            return;

        _warnedInvalidHorizontalThrusters = true;
        Debug.LogWarning("[ROVInput] Horizontal thruster setup is empty or singular. Horizontal force/yaw are disabled until thruster positions and directions are valid.");
    }

    void ApplyPhysicalVerticalThrusters(Vector3 desiredWorldForce, ForceMode forceMode)
    {
        if (desiredWorldForce.sqrMagnitude <= 1e-8f)
            return;

        if (!TryBuildVerticalThrusterMixer(
                out VerticalThruster[] thrusters,
                out Vector3[] localPositions,
                out Vector3[] localDirections,
                out float[] forceX,
                out float[] forceY,
                out float[] forceZ,
                out int count))
        {
            WarnInvalidVerticalThrustersOnce();
            return;
        }

        Vector3 desiredLocalForce = transform.InverseTransformDirection(desiredWorldForce);

        float m00 = Mathf.Max(0f, verticalThrusterMixerRegularization);
        float m01 = 0f;
        float m02 = 0f;
        float m11 = Mathf.Max(0f, verticalThrusterMixerRegularization);
        float m12 = 0f;
        float m22 = Mathf.Max(0f, verticalThrusterMixerRegularization);

        for (int i = 0; i < count; i++)
        {
            float ax = forceX[i];
            float ay = forceY[i];
            float az = forceZ[i];

            m00 += ax * ax;
            m01 += ax * ay;
            m02 += ax * az;
            m11 += ay * ay;
            m12 += ay * az;
            m22 += az * az;
        }

        if (!SolveSymmetric3x3(
                m00, m01, m02,
                m11, m12,
                m22,
                desiredLocalForce.x, desiredLocalForce.y, desiredLocalForce.z,
                out Vector3 mixer))
        {
            WarnInvalidVerticalThrustersOnce();
            return;
        }

        for (int i = 0; i < count; i++)
        {
            _verticalThrusterScratchCommand[i] =
                forceX[i] * mixer.x + forceY[i] * mixer.y + forceZ[i] * mixer.z;
        }

        float commandScale = GetCommonVerticalThrusterScale(thrusters, _verticalThrusterScratchCommand, count);
        for (int i = 0; i < count; i++)
        {
            float thrust = _verticalThrusterScratchCommand[i] * commandScale;
            if (Mathf.Abs(thrust) <= 1e-4f)
                continue;

            Vector3 worldPosition = TransformLocalPointWithoutScale(localPositions[i]);
            Vector3 worldDirection = transform.TransformDirection(localDirections[i]).normalized;
            rb.AddForceAtPosition(worldDirection * thrust, worldPosition, forceMode);
        }
    }

    bool TryBuildVerticalThrusterMixer(
        out VerticalThruster[] activeThrusters,
        out Vector3[] localPositions,
        out Vector3[] localDirections,
        out float[] forceX,
        out float[] forceY,
        out float[] forceZ,
        out int count)
    {
        int capacity = verticalThrusters != null ? verticalThrusters.Length : 0;
        EnsureVerticalThrusterScratchCapacity(capacity);

        activeThrusters = _verticalThrusterScratchActive;
        localPositions = _verticalThrusterScratchPositions;
        localDirections = _verticalThrusterScratchDirections;
        forceX = _verticalThrusterScratchForceX;
        forceY = _verticalThrusterScratchForceY;
        forceZ = _verticalThrusterScratchForceZ;
        count = 0;

        if (capacity == 0)
            return false;

        for (int i = 0; i < verticalThrusters.Length; i++)
        {
            VerticalThruster thruster = verticalThrusters[i];
            if (thruster == null || !thruster.enabled)
                continue;

            Vector3 localPosition;
            Vector3 localDirection;
            if (thruster.mount != null)
            {
                localPosition = InverseTransformPointWithoutScale(thruster.mount.position);
                localDirection = transform.InverseTransformDirection(thruster.mount.forward);
            }
            else
            {
                localPosition = thruster.localPosition;
                localDirection = thruster.localDirection;
            }

            if (localDirection.sqrMagnitude <= 1e-8f)
                continue;

            localDirection.Normalize();

            activeThrusters[count] = thruster;
            localPositions[count] = localPosition;
            localDirections[count] = localDirection;
            forceX[count] = localDirection.x;
            forceY[count] = localDirection.y;
            forceZ[count] = localDirection.z;
            count++;
        }

        return count > 0;
    }

    static float GetCommonVerticalThrusterScale(VerticalThruster[] thrusters, float[] commands, int count)
    {
        float maxRatio = 1f;
        for (int i = 0; i < count; i++)
        {
            float thrust = commands[i];
            float limit = thrust >= 0f
                ? Mathf.Max(0f, thrusters[i].maxForwardThrustN)
                : Mathf.Max(0f, thrusters[i].maxReverseThrustN);

            if (limit > 1e-6f)
                maxRatio = Mathf.Max(maxRatio, Mathf.Abs(thrust) / limit);
            else if (Mathf.Abs(thrust) > 1e-4f)
                return 0f;
        }

        return 1f / maxRatio;
    }

    void EnsureVerticalThrusterScratchCapacity(int capacity)
    {
        if (_verticalThrusterScratchActive != null && _verticalThrusterScratchActive.Length == capacity)
            return;

        _verticalThrusterScratchActive = new VerticalThruster[capacity];
        _verticalThrusterScratchPositions = new Vector3[capacity];
        _verticalThrusterScratchDirections = new Vector3[capacity];
        _verticalThrusterScratchForceX = new float[capacity];
        _verticalThrusterScratchForceY = new float[capacity];
        _verticalThrusterScratchForceZ = new float[capacity];
        _verticalThrusterScratchCommand = new float[capacity];
    }

    void WarnInvalidVerticalThrustersOnce()
    {
        if (_warnedInvalidVerticalThrusters)
            return;

        _warnedInvalidVerticalThrusters = true;
        Debug.LogWarning("[ROVInput] Vertical thruster setup is empty or singular. Heave and altitude hold forces are disabled until vertical thruster positions and directions are valid.");
    }

    Vector3 TransformLocalPointWithoutScale(Vector3 localPoint)
    {
        return transform.position + transform.rotation * localPoint;
    }

    Vector3 InverseTransformPointWithoutScale(Vector3 worldPoint)
    {
        return Quaternion.Inverse(transform.rotation) * (worldPoint - transform.position);
    }

    static float GetCommonThrusterScale(HorizontalThruster[] thrusters, float[] commands, int count)
    {
        float maxRatio = 1f;
        for (int i = 0; i < count; i++)
        {
            float thrust = commands[i];
            float limit = thrust >= 0f
                ? Mathf.Max(0f, thrusters[i].maxForwardThrustN)
                : Mathf.Max(0f, thrusters[i].maxReverseThrustN);

            if (limit > 1e-6f)
                maxRatio = Mathf.Max(maxRatio, Mathf.Abs(thrust) / limit);
            else if (Mathf.Abs(thrust) > 1e-4f)
                return 0f;
        }

        return 1f / maxRatio;
    }

    static float GetPlanarScaleWithYawReserved(
        HorizontalThruster[] thrusters,
        float[] planarCommands,
        float[] yawCommands,
        float yawScale,
        int count)
    {
        float planarScale = 1f;
        for (int i = 0; i < count; i++)
        {
            float yaw = yawCommands[i] * yawScale;
            float planar = planarCommands[i];
            if (Mathf.Abs(planar) <= 1e-6f)
                continue;

            float maxForward = Mathf.Max(0f, thrusters[i].maxForwardThrustN);
            float maxReverse = Mathf.Max(0f, thrusters[i].maxReverseThrustN);
            float upper = planar > 0f
                ? (maxForward - yaw) / planar
                : (-maxReverse - yaw) / planar;

            planarScale = Mathf.Min(planarScale, Mathf.Max(0f, upper));
        }

        return Mathf.Clamp01(planarScale);
    }

    void EnsureHorizontalThrusterScratchCapacity(int capacity)
    {
        if (_thrusterScratchActive != null && _thrusterScratchActive.Length == capacity)
            return;

        _thrusterScratchActive = new HorizontalThruster[capacity];
        _thrusterScratchPositions = new Vector3[capacity];
        _thrusterScratchDirections = new Vector3[capacity];
        _thrusterScratchForceX = new float[capacity];
        _thrusterScratchForceZ = new float[capacity];
        _thrusterScratchTorqueY = new float[capacity];
        _thrusterScratchCommand = new float[capacity];
        _thrusterScratchPlanarCommand = new float[capacity];
        _thrusterScratchYawCommand = new float[capacity];
    }

    static bool SolveSymmetric3x3(
        float m00, float m01, float m02,
        float m11, float m12,
        float m22,
        float b0, float b1, float b2,
        out Vector3 x)
    {
        float c00 = m11 * m22 - m12 * m12;
        float c01 = m02 * m12 - m01 * m22;
        float c02 = m01 * m12 - m02 * m11;
        float c11 = m00 * m22 - m02 * m02;
        float c12 = m01 * m02 - m00 * m12;
        float c22 = m00 * m11 - m01 * m01;

        float det = m00 * c00 + m01 * c01 + m02 * c02;
        if (Mathf.Abs(det) <= 1e-8f)
        {
            x = Vector3.zero;
            return false;
        }

        float invDet = 1f / det;
        x = new Vector3(
            (c00 * b0 + c01 * b1 + c02 * b2) * invDet,
            (c01 * b0 + c11 * b1 + c12 * b2) * invDet,
            (c02 * b0 + c12 * b1 + c22 * b2) * invDet);
        return !(float.IsNaN(x.x) || float.IsNaN(x.y) || float.IsNaN(x.z));
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

    public void ApplyMenuInputSelection(ExternalInputDevice device)
    {
        requireInputSelectionAtStartup = false;
        showInputSelectionOverlay = false;
        _awaitingStartupInputSelection = false;
        DestroyStartupSelectionUI();

        switch (device)
        {
            case ExternalInputDevice.Gamepad:
                inputSelectionMode = InputSelectionMode.ForceGamepad;
                _selectedInput = InputSource.Gamepad;
                break;
            case ExternalInputDevice.Joystick:
                inputSelectionMode = InputSelectionMode.ForceJoystick;
                _selectedInput = InputSource.Joystick;
                break;
            case ExternalInputDevice.Keyboard:
                enableKeyboardInput = true;
                inputSelectionMode = InputSelectionMode.ForceKeyboard;
                _selectedInput = InputSource.Keyboard;
                break;
        }
    }

    void OnDrawGizmosSelected()
    {
        DrawHorizontalThrusterGizmos();
        DrawVerticalThrusterGizmos();
    }

    void DrawHorizontalThrusterGizmos()
    {
        if (horizontalThrusters == null)
            return;

        Gizmos.color = new Color(0.1f, 0.8f, 1f, 1f);
        for (int i = 0; i < horizontalThrusters.Length; i++)
        {
            HorizontalThruster thruster = horizontalThrusters[i];
            if (thruster == null || !thruster.enabled)
                continue;

            Vector3 worldPosition;
            Vector3 worldDirection;
            if (thruster.mount != null)
            {
                worldPosition = thruster.mount.position;
                worldDirection = thruster.mount.forward;
            }
            else
            {
                Vector3 localDirection = thruster.localDirection;
                localDirection.y = 0f;
                if (localDirection.sqrMagnitude <= 1e-8f)
                    continue;

                worldPosition = TransformLocalPointWithoutScale(thruster.localPosition);
                worldDirection = transform.TransformDirection(localDirection.normalized);
            }

            Gizmos.DrawSphere(worldPosition, 0.035f);
            Gizmos.DrawLine(worldPosition, worldPosition + worldDirection.normalized * 0.35f);
        }
    }

    void DrawVerticalThrusterGizmos()
    {
        if (verticalThrusters == null)
            return;

        Gizmos.color = new Color(1f, 0.85f, 0.15f, 1f);
        for (int i = 0; i < verticalThrusters.Length; i++)
        {
            VerticalThruster thruster = verticalThrusters[i];
            if (thruster == null || !thruster.enabled)
                continue;

            Vector3 worldPosition;
            Vector3 worldDirection;
            if (thruster.mount != null)
            {
                worldPosition = thruster.mount.position;
                worldDirection = thruster.mount.forward;
            }
            else
            {
                Vector3 localDirection = thruster.localDirection;
                if (localDirection.sqrMagnitude <= 1e-8f)
                    continue;

                worldPosition = TransformLocalPointWithoutScale(thruster.localPosition);
                worldDirection = transform.TransformDirection(localDirection.normalized);
            }

            Gizmos.DrawSphere(worldPosition, 0.035f);
            Gizmos.DrawLine(worldPosition, worldPosition + worldDirection.normalized * 0.35f);
        }
    }
}
