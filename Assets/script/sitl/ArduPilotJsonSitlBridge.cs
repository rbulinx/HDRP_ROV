using System;
using System.Globalization;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;
using UnityEngine.Serialization;

[DisallowMultipleComponent]
[RequireComponent(typeof(Rigidbody))]
public class ArduPilotJsonSitlBridge : MonoBehaviour
{
    public enum WorldFrameMapping
    {
        UnityForwardZRightXUpY,
        LegacyReferenceBoat
    }

    public enum ServoOutputMode
    {
        MotorOutputs,
        AxisMixedOutputs
    }

    [Serializable]
    public class JsonImuData
    {
        public float[] gyro = { 0f, 0f, 0f };
        public float[] accel_body = { 0f, 0f, -9.8f };
    }

    [Serializable]
    public class JsonOutputPacket
    {
        public float timestamp;
        public JsonImuData imu = new JsonImuData();
        public float[] position = { 0f, 0f, 0f };
        public float[] attitude = { 0f, 0f, 0f };
        public float[] velocity = { 0f, 0f, 0f };
    }

    [Serializable]
    public class SitlThrusterSpec
    {
        public string name = "Thruster";
        public int servoChannel = 1;
        public Vector3 localPosition = Vector3.zero;
        public Vector3 localDirection = Vector3.forward;
        public float maxForwardThrustN = 49f;
        public float maxReverseThrustN = 49f;
    }

    struct TelemetryState
    {
        public float timestamp;
        public bool stabilizeEkf;
        public Vector3 unityPosition;
        public Quaternion unityRotation;
        public Vector3 unityVelocity;
        public Vector3 unityAngularVelocity;
        public Vector3 localPosition;
        public Vector3 positionNed;
        public Vector3 velocityNed;
        public Vector3 accelerationNed;
        public Vector3 attitudeRadians;
        public Vector3 gyroFrd;
        public Vector3 accelBodyFrd;
        public float dt;
        public float yawDeltaRadians;
        public Vector3 positionDeltaNed;
        public Vector3 positionDerivedVelocityNed;
        public Vector3 velocityMismatchNed;
    }

    [Header("UDP JSON SITL")]
    [FormerlySerializedAs("localPort")] public int _1Port = 9002;
    [HideInInspector] public bool connectOnEnable = true;
    [HideInInspector] public int maxPacketsPerUpdate = 16;
    [HideInInspector] public int receiveBufferSizeBytes = 262144;

    [Header("Target")]
    [FormerlySerializedAs("telemetrySource")] public Transform _2Target;

    [Header("Vehicle Model")]
    [FormerlySerializedAs("vehicleMassKg")] public float _3MassKg = 10f;
    [FormerlySerializedAs("linearDragCoefficient")] public float _4LinDrag = 35f;
    [FormerlySerializedAs("angularDragCoefficient")] public float _5AngDrag = 5f;
    [FormerlySerializedAs("defaultHorizontalThrusterMaxN")] public float _6HThrustN = 100f;
    [FormerlySerializedAs("defaultVerticalThrusterMaxN")] public float _7VThrustN = 200f;
    [FormerlySerializedAs("sitlThrusters")] public SitlThrusterSpec[] _8Thrusters = Array.Empty<SitlThrusterSpec>();

    [Header("Water Current")]
    [FormerlySerializedAs("applyWaterCurrent")] public bool _9UseCurrent = false;
    [FormerlySerializedAs("autoApplyWaterCurrentWhenCurrentVelocityIsNonZero")] public bool _10AutoCurrent = true;
    [FormerlySerializedAs("currentVelocity")] public Vector3 _11CurrentVel = Vector3.zero;
    [HideInInspector] public bool logWaterCurrentForces;
    [HideInInspector] public float waterCurrentLogIntervalSeconds = 1f;

    [Header("Water Surface - Buoyancy Spring")]
    [FormerlySerializedAs("applyWaterSurfaceForces")] public bool _12UseSurface = true;
    [FormerlySerializedAs("waterSurfaceDesiredSubmergeMeters")] public float _13SubmergeM = 0.5f;
    [FormerlySerializedAs("holdDepthRelativeToWaterSurface")] public bool _14HoldDepth = false;
    [FormerlySerializedAs("waterSurfaceVerticalSpringNPerM")] public float _15VSpring = 100f;
    [FormerlySerializedAs("waterSurfaceVerticalDampingNPerMS")] public float _16VDamping = 280f;

    [Header("Water Surface - Tilt Response")]
    [FormerlySerializedAs("applyWaterSurfaceTiltForces")] public bool _17UseTilt = true;
    [FormerlySerializedAs("waterSurfaceTiltSpringNPerM")] public float _18TiltSpring = 50f;
    [FormerlySerializedAs("waterSurfaceTiltDampingNPerMS")] public float _19TiltDamping = 80f;
    [FormerlySerializedAs("maxWaterSurfaceForcePerPointN")] public float _20MaxPointN = 40f;
    [FormerlySerializedAs("waterSurfaceLocalSamplePoints")]
    public Vector3[] _21SamplePts =
    {
        new Vector3(0.75f, 0f, 0.75f),
        new Vector3(-0.75f, 0f, 0.75f),
        new Vector3(0.75f, 0f, -0.75f),
        new Vector3(-0.75f, 0f, -0.75f)
    };

    [Header("Water Surface - Breach / Clamp Safety")]
    [FormerlySerializedAs("waterSurfaceClampBelowSurfaceMeters")] public float _22ClampBelowM = 0.05f;
    [FormerlySerializedAs("waterSurfaceClampGuardMeters")] public float _23ClampGuardM = 0.08f;
    [FormerlySerializedAs("zeroUpwardVelocityOnWaterSurfaceClamp")] public bool _24ZeroUpVel = true;
    [FormerlySerializedAs("waterSurfaceClampSoftenTimeSeconds")] public float _25ClampSoftenS = 0.15f;
    [FormerlySerializedAs("waterSurfaceClampHardMarginMeters")] public float _26ClampHardM = 0.1f;
    [FormerlySerializedAs("waterSurfaceBreachSpringMultiplier")] public float _27BreachMult = 3f;
    [FormerlySerializedAs("waterSurfaceBreachAnticipationMeters")] public float _28BreachAntM = 0.1f;
    [FormerlySerializedAs("suppressWaterSurfaceForcesWhileDiving")] public bool _29SuppressDive = false;
    [FormerlySerializedAs("waterSurfaceDiveForceThresholdN")] public float _30DiveThreshN = 0.5f;

    [Header("Water Surface - Signal Smoothing")]
    [FormerlySerializedAs("lowPassWaterSurfaceForces")] public bool _31UseLowPass = true;
    [FormerlySerializedAs("waterSurfaceLowPassTimeSeconds")] public float _32LowPassS = 2.0f;

    [Header("Water Surface - Wave Auto-Tune (from Wind)")]
    [FormerlySerializedAs("autoTuneWaveResponseFromOceanWind")] public bool _33AutoTuneWind = true;
    [FormerlySerializedAs("oceanWindReferenceSpeedKmh")] public float _34WindRefKmh = 30f;
    [FormerlySerializedAs("minAutoWaveLengthMeters")] public float _35MinWaveLenM = 4f;
    [FormerlySerializedAs("maxAutoWaveLengthMeters")] public float _36MaxWaveLenM = 24f;
    [FormerlySerializedAs("minAutoWaterParticleMotionScale")] public float _37MinMotionScale = 0.4f;
    [FormerlySerializedAs("maxAutoWaterParticleMotionScale")] public float _38MaxMotionScale = 2.5f;

    [Header("Water Surface - Wave Motion")]
    [FormerlySerializedAs("waterSurfaceWaveLengthMeters")] public float _39WaveLenM = 12f;
    [FormerlySerializedAs("waterSurfaceWavePeriodSeconds")] public float _40WavePeriodS = 4f;
    [FormerlySerializedAs("waterParticleMotionScale")] public float _41MotionScale = 2f;
    [FormerlySerializedAs("waterParticleAccelerationScale")] public float _42AccelScale = 1.2f;
    [FormerlySerializedAs("maxWaterParticleVerticalForceN")] public float _43MaxVForceN = 800f;
    [FormerlySerializedAs("minimumWaterSurfaceWaveInfluence")] public float _44MinWaveInfluence = 0f;

    [Header("Water Surface - Horizontal Drift")]
    [FormerlySerializedAs("waterParticleHorizontalDirection")] public Vector3 _45HDirection = Vector3.forward;
    [FormerlySerializedAs("waterParticlePrimaryDirectionWeight")] public float _46PrimaryWeight = 1f;
    [FormerlySerializedAs("waterParticleCrossDirectionWeight")] public float _47CrossWeight = 0.2f;
    [FormerlySerializedAs("waterParticleObliqueDirectionWeight")] public float _48ObliqueWeight = 0.1f;
    [FormerlySerializedAs("waterParticleHorizontalMotionScale")] public float _49HMotionScale = 0.5f;
    [FormerlySerializedAs("waterParticleHorizontalDampingNPerMS")] public float _50HDampingN = 120f;
    [FormerlySerializedAs("maxWaterParticleHorizontalForceN")] public float _51MaxHForceN = 300f;

    [Header("Servo Mapping")]
    [FormerlySerializedAs("horizontalServoChannels")] public int[] _52HChannels = { 1, 2, 3, 4 };
    [FormerlySerializedAs("verticalServoChannels")] public int[] _53VChannels = { 5, 6 };
    [FormerlySerializedAs("horizontalOutputScales")] public float[] _54HScales = { 1f, 1f, 1f, 1f };
    [FormerlySerializedAs("verticalOutputScales")] public float[] _55VScales = { -1f, -1f };
    [HideInInspector] public float servoPwmMin = 1100f;
    [HideInInspector] public float servoPwmNeutral = 1500f;
    [HideInInspector] public float servoPwmMax = 1900f;
    [HideInInspector] public float servoPwmDeadzone = 25f;
    [HideInInspector] public float actuatorOutputTimeoutSeconds = 0.5f;
    [HideInInspector] public bool invertHorizontalMotorOutputs = false;
    [HideInInspector] [Range(0.01f, 1f)] public float actuatorOutputSmoothing = 0.12f;
    [HideInInspector] public float maxActuatorOutputSlewPerSecond = 1.5f;
    [FormerlySerializedAs("directThrustScale")] public float _56ThrustScale = 1f;
    [HideInInspector] public ServoOutputMode servoOutputMode = ServoOutputMode.MotorOutputs;
    [HideInInspector] [Range(-2f, 2f)] public float horizontalSurgeScale = 1f;
    [HideInInspector] [Range(-2f, 2f)] public float horizontalSwayScale = 1f;
    [HideInInspector] [Range(-2f, 2f)] public float horizontalYawScale = 0.2f;
    [HideInInspector] [Range(0f, 1f)] public float motorOutputYawScale = 1f;
    [HideInInspector] [Range(0f, 2f)] public float verticalHeaveScale = 1f;
    [HideInInspector] [Range(0f, 2f)] public float verticalDifferentialScale = 1f;
    [HideInInspector] public bool applyThrusterForces = true;
    [HideInInspector] public bool requireServoOutputForces = true;
    [HideInInspector] public float forceOutputDeadzone = 0.01f;

    [Header("State Mapping")]
    [HideInInspector] public bool zeroUnityPositionAtStart = true;
    [FormerlySerializedAs("useWaterSurfaceAsVerticalOrigin")] public bool _57UseSurfaceOrigin = true;
    [FormerlySerializedAs("waterSurface")] public WaterSurface _58Surface;
    [FormerlySerializedAs("waterSurfaceY")] public float _59SurfaceY = 0f;
    [FormerlySerializedAs("waterQueryError")] public float _60QueryError = 0.01f;
    [FormerlySerializedAs("waterQueryMaxIterations")] public int _61QueryIter = 8;
    [HideInInspector] public WorldFrameMapping worldFrameMapping = WorldFrameMapping.UnityForwardZRightXUpY;
    [HideInInspector] public bool invertReportedHorizontalPosition = false;
    [HideInInspector] public bool freezeVerticalState = false;
    [HideInInspector] public bool freezeYawRate = false;
    [HideInInspector] public bool reportYawRate = true;
    [HideInInspector] public bool reportVerticalState = true;
    [HideInInspector] public bool zeroReportedRollPitchAtStart = true;
    [HideInInspector] [Range(-1f, 1f)] public float yawAngleSign = 1f;
    [HideInInspector] [Range(-1f, 1f)] public float yawRateSign = 1f;
    [HideInInspector] [Range(-1f, 1f)] public float rollAngleSign = -1f;
    [HideInInspector] [Range(-1f, 1f)] public float pitchAngleSign = 1f;
    [HideInInspector] public bool forceLevelRollPitchReport = true;
    [HideInInspector] public bool zeroRollPitchGyroWhenLevel = true;
    [HideInInspector] public bool stabilizeEkfBeforeActuatorOutput = false;
    [HideInInspector] public float maxReportedRollPitchDegrees = 10f;
    [HideInInspector] public Vector3 bodyEulerOffsetDegrees = Vector3.zero;
    [HideInInspector] public bool filterReportedAttitude = false;
    [HideInInspector] [Range(0.01f, 1f)] public float reportedAttitudeSmoothing = 1f;
    [HideInInspector] public bool deriveReportedYawRateFromAttitude = true;
    [HideInInspector] public float maxReportedYawRateRadiansPerSecond = 2.5f;
    [HideInInspector] public bool deriveVelocityFromPosition = false;
    [HideInInspector] public bool usePositionVelocityWhenRigidbodyIsStopped = true;
    [HideInInspector] public bool deriveVerticalVelocityFromPosition = false;
    [HideInInspector] public bool forceStableVerticalReport = false;
    [HideInInspector] public bool filterReportedPosition = false;
    [HideInInspector] public float positionTinyValueEpsilon = 0.001f;
    [HideInInspector] public float maxReportedHorizontalSpeed = 5f;
    [HideInInspector] public float maxReportedVerticalSpeed = 2f;
    [HideInInspector] public float rigidbodyVelocityDeadzone = 0.02f;
    [HideInInspector] public float verticalVelocityDeadzone = 0.03f;
    [HideInInspector] [Range(0.01f, 1f)] public float verticalVelocitySmoothing = 0.2f;
    [HideInInspector] public float maxReportedHorizontalStepMeters = 0.25f;
    [HideInInspector] [Range(0.01f, 1f)] public float reportedPositionSmoothing = 0.5f;
    [HideInInspector] public float gravityMetersPerSecondSquared = 9.80665f;
    [HideInInspector] public bool includeLinearAccelerationInImu = true;
    [HideInInspector] public float accelerationSmoothing = 0.35f;
    [HideInInspector] public float maxReportedAccelerationMetersPerSecondSquared = 3f;

    [HideInInspector] public bool logConnection;
    [HideInInspector] public bool logServoPackets = true;
    [HideInInspector] public float servoLogIntervalSeconds = 0.25f;
    [HideInInspector] public bool logForceApplication;
    [HideInInspector] public float forceLogIntervalSeconds = 0.25f;
    [HideInInspector] public bool warnUnmappedActiveServoOutputs = true;
    [HideInInspector] public int[] ignoredUnmappedServoChannels = { 9, 10 };
    [HideInInspector] public bool logTelemetryPackets;
    [HideInInspector] public float telemetryLogIntervalSeconds = 1f;
    [HideInInspector] public float minTelemetryLogIntervalSeconds = 0.1f;
    [HideInInspector] public bool logEkfDiagnostics = true;
    [HideInInspector] public bool warnEkfDiagnosticJumps = true;
    [HideInInspector] public float ekfDiagnosticLogIntervalSeconds = 0.25f;
    [HideInInspector] public float ekfYawJumpWarningDegrees = 10f;
    [HideInInspector] public float ekfPositionJumpWarningMeters = 0.5f;
    [HideInInspector] public float ekfVelocityMismatchWarningMetersPerSecond = 1f;
    [HideInInspector] public float ekfGyroZWarningRadiansPerSecond = 1f;
    [HideInInspector] public float ekfAccelWarningMetersPerSecondSquared = 2f;
    [HideInInspector] public bool logJsonSendErrors;
    [HideInInspector] public float jsonTinyValueEpsilon = 1e-6f;

    const float TelemetrySendIntervalSeconds = 0.02f;

    UdpClient socketReceive;
    IPEndPoint remoteEndpoint = new IPEndPoint(IPAddress.Any, 0);
    bool hasRemoteConnection;
    Rigidbody rb;
    Rigidbody telemetryRb;
    Transform resolvedTelemetryTransform;
    Vector3 unityOriginPosition;
    float lastOutputTimestamp = -1f;
    float telemetryStartRealtime = -1f;
    double lastPositionFilterTimestamp = -1.0;
    double lastVelocityEstimateTimestamp = -1.0;
    Vector3 lastReportedPositionNed;
    Vector3 filteredPositionNed;
    Vector3 filteredVelocityNed;
    Vector3 lastVelocityNed;
    Vector3 filteredAccelNed;
    Vector3 filteredAttitudeRadians;
    Vector2 initialReportedRollPitchRadians;
    float initialReportedYawRadians;
    float lastFilteredYawRadians;
    double lastAccelerationTimestamp = -1.0;
    double lastAttitudeFilterTimestamp = -1.0;
    double lastVerticalVelocityTimestamp = -1.0;
    double lastDepthSlewTimestamp = -1.0;
    float lastVerticalPositionDown;
    float slewLimitedDepthDown;
    float filteredVerticalVelocityDown;
    bool hasFilteredPosition;
    bool hasFilteredAttitude;
    bool hasInitialReportedRollPitch;
    bool hasPreviousVelocityNed;
    bool hasPreviousVerticalPosition;
    bool hasSlewLimitedDepth;
    bool hasSeenActiveThrusterOutput;
    float lastActuatorOutputTime = -999f;
    float lastServoLogTime = -999f;
    float lastForceLogTime = -999f;
    float lastTelemetryLogTime = -999f;
    float lastTelemetrySendTime = -999f;
    float lastUnmappedServoWarningTime = -999f;
    float lastEkfDiagnosticLogTime = -999f;
    float lastWaterCurrentLogTime = -999f;
    bool hasLastTelemetryDiagnosticState;
    TelemetryState lastTelemetryDiagnosticState;
    readonly ushort[] servoOutputs = new ushort[32];
    float[] horizontalOutputTargets = Array.Empty<float>();
    float[] verticalOutputTargets = Array.Empty<float>();
    float[] horizontalOutputs = Array.Empty<float>();
    float[] verticalOutputs = Array.Empty<float>();
    Vector3[] surfaceWorldSamplePoints = Array.Empty<Vector3>();
    float[] filteredWaterSurfaceSampleY = Array.Empty<float>();
    bool[] validWaterSurfaceSampleY = Array.Empty<bool>();
    bool hasFilteredWaterSurfaceSamples;
    float lastWaterSurfaceWaveDisplacementY;
    bool hasLastWaterSurfaceWaveDisplacementY;
    float lastWaterParticleVelocityY;
    bool hasLastWaterParticleVelocityY;
    readonly JsonOutputPacket outputPacket = new JsonOutputPacket();
    WaterSearchParameters waterSearchParameters;
    WaterSearchResult waterSearchResult;
    bool hasWaterSearchCandidate;

    public bool IsConnected => socketReceive != null;
    public float LastActuatorOutputAgeSeconds =>
        lastActuatorOutputTime > 0f ? Time.unscaledTime - lastActuatorOutputTime : float.PositiveInfinity;

    void Reset()
    {
        EnsureInspectorDefaults();
    }

    void OnValidate()
    {
        EnsureInspectorDefaults();
    }

    void OnEnable()
    {
        EnsureInspectorDefaults();

        rb = GetComponent<Rigidbody>();
        ResolveTelemetrySource();
        ResolveWaterSurfaceIfNeeded();
        waterSearchParameters = new WaterSearchParameters();
        waterSearchResult = new WaterSearchResult();
        hasWaterSearchCandidate = false;
        ResetFilteredWaterSurface();
        RefreshVehicleModel();
        ApplyVehicleModelToRigidbody();
        unityOriginPosition = GetTelemetryPosition();
        if (_57UseSurfaceOrigin)
            unityOriginPosition.y = GetWaterSurfaceY();
        lastOutputTimestamp = -1f;
        telemetryStartRealtime = Time.unscaledTime;
        lastPositionFilterTimestamp = -1.0;
        lastVelocityEstimateTimestamp = -1.0;
        lastReportedPositionNed = Vector3.zero;
        filteredPositionNed = Vector3.zero;
        filteredVelocityNed = Vector3.zero;
        lastVelocityNed = Vector3.zero;
        filteredAccelNed = Vector3.zero;
        filteredAttitudeRadians = Vector3.zero;
        initialReportedRollPitchRadians = Vector2.zero;
        initialReportedYawRadians = 0f;
        lastFilteredYawRadians = 0f;
        lastAccelerationTimestamp = -1.0;
        lastAttitudeFilterTimestamp = -1.0;
        lastVerticalVelocityTimestamp = -1.0;
        lastDepthSlewTimestamp = -1.0;
        lastVerticalPositionDown = 0f;
        slewLimitedDepthDown = 0f;
        filteredVerticalVelocityDown = 0f;
        lastServoLogTime = -999f;
        lastForceLogTime = -999f;
        lastTelemetryLogTime = -999f;
        lastTelemetrySendTime = -999f;
        lastUnmappedServoWarningTime = -999f;
        lastEkfDiagnosticLogTime = -999f;
        lastWaterCurrentLogTime = -999f;
        hasFilteredPosition = false;
        hasFilteredAttitude = false;
        hasInitialReportedRollPitch = false;
        hasPreviousVelocityNed = false;
        hasPreviousVerticalPosition = false;
        hasSlewLimitedDepth = false;
        hasSeenActiveThrusterOutput = false;
        hasLastTelemetryDiagnosticState = false;
        lastTelemetryDiagnosticState = default;

        CaptureInitialReportedRollPitch();

        if (connectOnEnable)
            StartBridge();
    }

    void OnDisable()
    {
        StopBridge();
    }

    void Update()
    {
        ReceiveServoPackets();
        SendTelemetryIfDue();
    }

    void FixedUpdate()
    {
        ApplyVehicleModelToRigidbody();
        ApplyWaterCurrentForces();
        ApplyWaterSurfaceForces();

        float outputAge = Time.unscaledTime - lastActuatorOutputTime;
        float timeout = Mathf.Max(0.05f, actuatorOutputTimeoutSeconds);
        bool hasFreshOutput = outputAge <= timeout;
        bool hasActiveOutput = HasActiveThrusterOutput();
        bool shouldApply = rb != null
            && hasFreshOutput
            && applyThrusterForces
            && (!requireServoOutputForces || hasActiveOutput);

        if (ShouldLogForceApplication())
        {
            Debug.Log(
                $"[ArduPilot JSON] force apply={shouldApply} fresh={hasFreshOutput} age={outputAge:0.000}s timeout={timeout:0.000}s active={hasActiveOutput} scale={_56ThrustScale:0.###} H=[{string.Join(", ", horizontalOutputs)}] V=[{string.Join(", ", verticalOutputs)}]");
        }

        if (shouldApply)
            ApplySitlThrusterForces();

        EnforceWaterSurfaceNoBreachClamp();

        SendTelemetryIfDue();
    }

    void ApplyWaterCurrentForces()
    {
        if (rb == null || !IsWaterCurrentActive())
            return;

#if UNITY_6000_0_OR_NEWER
        Vector3 velocity = rb.linearVelocity;
#else
        Vector3 velocity = rb.velocity;
#endif
        Vector3 relativeVelocity = velocity - _11CurrentVel;
        float linearDrag = Mathf.Max(0f, _4LinDrag);
        if (linearDrag <= 0f || relativeVelocity.sqrMagnitude <= 1e-8f)
            return;

        Vector3 force = -relativeVelocity * linearDrag;
        rb.AddForce(force, ForceMode.Force);

        if (ShouldLogWaterCurrentForce())
        {
            Debug.Log(
                $"[ArduPilot JSON] current velocity=[{_11CurrentVel.x:0.###}, {_11CurrentVel.y:0.###}, {_11CurrentVel.z:0.###}] relative=[{relativeVelocity.x:0.###}, {relativeVelocity.y:0.###}, {relativeVelocity.z:0.###}] drag={linearDrag:0.###} force=[{force.x:0.###}, {force.y:0.###}, {force.z:0.###}]");
        }
    }

    bool IsWaterCurrentActive()
    {
        return _9UseCurrent
            || (_10AutoCurrent && _11CurrentVel.sqrMagnitude > 1e-8f);
    }

    void ApplyOceanWindWaveAutoTune()
    {
        if (!_33AutoTuneWind || _58Surface == null)
            return;

        float windRatio = Mathf.Clamp01(_58Surface.largeWindSpeed / Mathf.Max(0.001f, _34WindRefKmh));
        float lengthRatio = windRatio * windRatio;
        _39WaveLenM = Mathf.Lerp(_35MinWaveLenM, _36MaxWaveLenM, lengthRatio);
        _41MotionScale = Mathf.Lerp(_37MinMotionScale, _38MaxMotionScale, windRatio);
        _40WavePeriodS = Mathf.Sqrt(
            Mathf.PI * 2f * _39WaveLenM / Mathf.Max(0.001f, gravityMetersPerSecondSquared));

        float orientationRadians = _58Surface.largeOrientationValue * Mathf.Deg2Rad;
        _45HDirection = new Vector3(Mathf.Sin(orientationRadians), 0f, Mathf.Cos(orientationRadians));
    }

    void ApplyWaterSurfaceForces()
    {
        if (rb == null || !_12UseSurface)
            return;

        if (ShouldSuppressWaterSurfaceForcesForDive())
        {
            ResetFilteredWaterSurface();
            return;
        }

        ResolveWaterSurfaceIfNeeded();
        if (_58Surface == null)
            return;

        ApplyOceanWindWaveAutoTune();

        Vector3[] samplePoints = GetWaterSurfaceSamplePoints();
        if (samplePoints == null || samplePoints.Length == 0)
            return;

        EnsureWaterSurfaceSampleBuffers(samplePoints.Length);
        float averageSurfaceY = 0f;
        int validCount = 0;
        for (int i = 0; i < samplePoints.Length; i++)
        {
            if (!TryGetWaterSurfaceY(samplePoints[i], out float sampleSurfaceY))
            {
                validWaterSurfaceSampleY[i] = false;
                continue;
            }

            validWaterSurfaceSampleY[i] = true;
            float filteredSurfaceY = FilterWaterSurfaceSampleY(i, sampleSurfaceY);
            averageSurfaceY += filteredSurfaceY;
            validCount++;
        }

        if (validCount <= 0)
            return;

        if (!hasFilteredWaterSurfaceSamples)
            hasFilteredWaterSurfaceSamples = true;

        averageSurfaceY /= validCount;
        float meanSurfaceY = _59SurfaceY;
        float depthCenter = meanSurfaceY - rb.position.y;

        float waveInfluence = GetWaterSurfaceWaveInfluence(depthCenter);

        if (_17UseTilt)
        {
            float tiltSpring = Mathf.Max(0f, _18TiltSpring);
            float tiltDamping = Mathf.Max(0f, _19TiltDamping);
            float maxForce = Mathf.Max(0f, _20MaxPointN);

            for (int i = 0; i < samplePoints.Length; i++)
            {
                if (i >= filteredWaterSurfaceSampleY.Length || !validWaterSurfaceSampleY[i])
                    continue;

                float surfaceDelta = (filteredWaterSurfaceSampleY[i] - averageSurfaceY) * waveInfluence;
                float pointVerticalVelocity = rb.GetPointVelocity(samplePoints[i]).y;
                float pointForce = (surfaceDelta * tiltSpring - pointVerticalVelocity * tiltDamping) * waveInfluence;
                if (maxForce > 0f)
                    pointForce = Mathf.Clamp(pointForce, -maxForce, maxForce);

                rb.AddForceAtPosition(Vector3.up * pointForce, samplePoints[i], ForceMode.Force);
            }
        }

#if UNITY_6000_0_OR_NEWER
        Vector3 velocity = rb.linearVelocity;
#else
        Vector3 velocity = rb.velocity;
#endif
        float effectiveSurfaceY = meanSurfaceY + (averageSurfaceY - meanSurfaceY) * waveInfluence;
        EstimateWaterParticleVerticalKinematics(
            averageSurfaceY - meanSurfaceY,
            waveInfluence,
            out float waterParticleVelocityY,
            out float waterParticleAccelerationY);
        float waterParticleAccelerationForce = rb.mass * waterParticleAccelerationY * Mathf.Max(0f, _42AccelScale);
        float verticalForce;
        if (_14HoldDepth)
        {
            float hardLimitY = effectiveSurfaceY - Mathf.Max(0f, _22ClampBelowM);
            float targetY = effectiveSurfaceY - Mathf.Max(0f, _13SubmergeM);
            float guard = Mathf.Max(0f, _23ClampGuardM);
            if (guard > 0f)
                targetY = Mathf.Min(targetY, hardLimitY - guard);

            verticalForce = (targetY - rb.position.y) * Mathf.Max(0f, _15VSpring)
                + (waterParticleVelocityY - velocity.y) * Mathf.Max(0f, _16VDamping)
                + waterParticleAccelerationForce;
        }
        else
        {
            float waveDisplacement = effectiveSurfaceY - meanSurfaceY;
            verticalForce = (waveDisplacement * Mathf.Max(0f, _15VSpring)
                + (waterParticleVelocityY - velocity.y) * Mathf.Max(0f, _16VDamping)
                + waterParticleAccelerationForce);

            float breachOvershoot = rb.position.y - effectiveSurfaceY + Mathf.Max(0f, _28BreachAntM);
            if (breachOvershoot > 0f)
                verticalForce -= breachOvershoot * Mathf.Max(0f, _15VSpring) * Mathf.Max(0f, _27BreachMult);
        }

        float maxVerticalForce = Mathf.Max(0f, _43MaxVForceN);
        if (maxVerticalForce > 0f)
            verticalForce = Mathf.Clamp(verticalForce, -maxVerticalForce, maxVerticalForce);

        rb.AddForce(Vector3.up * verticalForce, ForceMode.Force);
        ApplyWaterParticleHorizontalOrbitalForce(averageSurfaceY - meanSurfaceY, waveInfluence, velocity);
    }

    void ApplyWaterParticleHorizontalOrbitalForce(float surfaceWaveDisplacementY, float waveInfluence, Vector3 velocity)
    {
        Vector3 direction = GetWaterParticleHorizontalDirection();
        if (direction.sqrMagnitude <= 1e-8f)
            return;

        float angularFrequency = Mathf.PI * 2f / Mathf.Max(0.001f, _40WavePeriodS);
        float targetVelocityAmplitude = surfaceWaveDisplacementY
            * angularFrequency
            * waveInfluence
            * Mathf.Max(0f, _41MotionScale)
            * Mathf.Max(0f, _49HMotionScale);

        Vector3 force = Vector3.zero;
        force += CalculateWaterParticleDirectionalForce(direction, targetVelocityAmplitude, Mathf.Max(0f, _46PrimaryWeight), velocity, waveInfluence);
        force += CalculateWaterParticleDirectionalForce(new Vector3(-direction.z, 0f, direction.x), targetVelocityAmplitude, Mathf.Max(0f, _47CrossWeight), velocity, waveInfluence);
        force += CalculateWaterParticleDirectionalForce((direction + new Vector3(direction.z, 0f, -direction.x)).normalized, targetVelocityAmplitude, Mathf.Max(0f, _48ObliqueWeight), velocity, waveInfluence);

        float maxForce = Mathf.Max(0f, _51MaxHForceN);
        if (maxForce > 0f && force.sqrMagnitude > maxForce * maxForce)
            force = force.normalized * maxForce;

        rb.AddForce(force, ForceMode.Force);
    }

    Vector3 CalculateWaterParticleDirectionalForce(Vector3 direction, float targetVelocityAmplitude, float weight, Vector3 velocity, float waveInfluence)
    {
        if (weight <= 0f || direction.sqrMagnitude <= 1e-8f)
            return Vector3.zero;

        direction = direction.normalized;
        float targetVelocity = targetVelocityAmplitude * weight;
        if (IsWaterCurrentActive())
            targetVelocity += Vector3.Dot(_11CurrentVel, direction);

        float currentAlongDirection = Vector3.Dot(velocity, direction);
        float dampingGain = Mathf.Max(0f, _50HDampingN) * Mathf.Clamp01(waveInfluence);
        float force = (targetVelocity - currentAlongDirection) * dampingGain;
        return direction * force;
    }

    Vector3 GetWaterParticleHorizontalDirection()
    {
        Vector3 direction = _45HDirection;
        direction.y = 0f;
        if (direction.sqrMagnitude <= 1e-8f)
            direction = Vector3.forward;

        return direction.normalized;
    }

    void EstimateWaterParticleVerticalKinematics(
        float surfaceWaveDisplacementY,
        float waveInfluence,
        out float velocityY,
        out float accelerationY)
    {
        float dt = Mathf.Max(0.001f, Time.fixedDeltaTime);
        if (!hasLastWaterSurfaceWaveDisplacementY)
        {
            lastWaterSurfaceWaveDisplacementY = surfaceWaveDisplacementY;
            hasLastWaterSurfaceWaveDisplacementY = true;
            velocityY = 0f;
            accelerationY = 0f;
            return;
        }

        float surfaceWaveVelocityY = (surfaceWaveDisplacementY - lastWaterSurfaceWaveDisplacementY) / dt;
        lastWaterSurfaceWaveDisplacementY = surfaceWaveDisplacementY;
        velocityY = surfaceWaveVelocityY * waveInfluence * Mathf.Max(0f, _41MotionScale);

        if (!hasLastWaterParticleVelocityY)
        {
            lastWaterParticleVelocityY = velocityY;
            hasLastWaterParticleVelocityY = true;
            accelerationY = 0f;
            return;
        }

        accelerationY = (velocityY - lastWaterParticleVelocityY) / dt;
        lastWaterParticleVelocityY = velocityY;
    }

    float GetWaterSurfaceWaveInfluence(float depthMeters)
    {
        float minInfluence = Mathf.Clamp01(_44MinWaveInfluence);
        float wavelength = Mathf.Max(0.001f, _39WaveLenM);
        float influence = Mathf.Exp(-Mathf.PI * 2f * Mathf.Max(0f, depthMeters) / wavelength);
        return Mathf.Max(minInfluence, Mathf.Clamp01(influence));
    }

    public void StartBridge()
    {
        if (socketReceive != null)
            return;

        try
        {
            socketReceive = new UdpClient(Mathf.Clamp(_1Port, 1, 65535));
            socketReceive.Client.ReceiveBufferSize = Mathf.Max(8192, receiveBufferSizeBytes);
            socketReceive.Client.Blocking = false;
            DisableUdpConnectionResetOnWindows(socketReceive.Client);
        }
        catch (Exception e)
        {
            StopBridge();
            Debug.LogWarning($"[ArduPilot JSON] Failed to bind UDP {_1Port}: {e.Message}");
        }
    }

    public void StopBridge()
    {
        socketReceive?.Close();
        socketReceive = null;
        hasRemoteConnection = false;
    }

    void ReceiveServoPackets()
    {
        if (socketReceive == null)
            return;

        try
        {
            int packetsProcessed = 0;
            int packetLimit = Mathf.Max(1, maxPacketsPerUpdate);
            while (socketReceive.Available > 0 && packetsProcessed < packetLimit)
            {
                packetsProcessed++;
                byte[] data = socketReceive.Receive(ref remoteEndpoint);
                if (!hasRemoteConnection)
                {
                    hasRemoteConnection = true;
                    lastTelemetrySendTime = -999f;
                    if (logConnection)
                        Debug.Log($"[ArduPilot JSON] Received new connection from SITL: {remoteEndpoint}");
                }

                if (TryParseServoPacket(data, out ushort frameRate, out uint frameCount, out int servoCount))
                {
                    UpdateMappedOutputsFromServoPwm();
                    if (ShouldLogServoPacket())
                        Debug.Log($"[ArduPilot JSON] frame={frameCount} rate={frameRate} servos={servoCount} active=[{FormatActiveServoOutputs(servoCount)}] H=[{string.Join(", ", horizontalOutputs)}] V=[{string.Join(", ", verticalOutputs)}]");
                    WarnUnmappedActiveServoOutputs(servoCount);
                }

                SendTelemetryIfDue();
            }
        }
        catch (SocketException e)
        {
            if (e.SocketErrorCode != SocketError.WouldBlock && e.SocketErrorCode != SocketError.ConnectionReset)
                Debug.LogWarning($"[ArduPilot JSON] UDP receive failed: {e.Message}");
        }
        catch (ObjectDisposedException)
        {
        }
    }

    void SendTelemetryIfDue()
    {
        if (!hasRemoteConnection || socketReceive == null)
            return;

        if (Time.unscaledTime - lastTelemetrySendTime < TelemetrySendIntervalSeconds)
            return;

        SendTelemetry();
    }

    bool TryParseServoPacket(byte[] packet, out ushort frameRate, out uint frameCount, out int servoCount)
    {
        frameRate = 0;
        frameCount = 0;
        servoCount = 0;

        if (packet == null || packet.Length < 40)
            return false;

        using var reader = new BinaryReader(new MemoryStream(packet), Encoding.UTF8, false);
        ushort magic = reader.ReadUInt16();
        if (magic == 18458 && packet.Length >= 40)
            servoCount = 16;
        else if (magic == 29569 && packet.Length >= 72)
            servoCount = 32;
        else
            return false;

        frameRate = reader.ReadUInt16();
        frameCount = reader.ReadUInt32();
        Array.Clear(servoOutputs, 0, servoOutputs.Length);

        int n = Mathf.Min(servoCount, servoOutputs.Length);
        for (int i = 0; i < n; i++)
            servoOutputs[i] = reader.ReadUInt16();

        return true;
    }

    void SendTelemetry()
    {
        if (!hasRemoteConnection || socketReceive == null)
            return;

        UpdateOutputPacket();
        string json = BuildTelemetryJson();
        if (ShouldLogTelemetryPacket())
            Debug.Log($"[ArduPilot JSON] TX {json}");

        json += "\n";
        byte[] bytes = Encoding.UTF8.GetBytes(json);

        try
        {
            socketReceive.Send(bytes, bytes.Length, remoteEndpoint);
            lastTelemetrySendTime = Time.unscaledTime;
        }
        catch (Exception e)
        {
            if (logJsonSendErrors)
                Debug.LogWarning($"[ArduPilot JSON] UDP send failed: {e.Message}");
        }
    }

    void ApplySitlThrusterForces()
    {
        if (rb == null || _8Thrusters == null)
            return;

        float scale = Mathf.Max(0f, _56ThrustScale);
        for (int i = 0; i < _8Thrusters.Length; i++)
        {
            SitlThrusterSpec thruster = _8Thrusters[i];
            if (thruster == null)
                continue;

            float command = Mathf.Clamp(GetMappedThrusterOutput(thruster.servoChannel), -1f, 1f);
            if (Mathf.Abs(command) <= Mathf.Max(0f, forceOutputDeadzone))
                continue;

            Vector3 localDirection = thruster.localDirection;
            if (localDirection.sqrMagnitude <= 1e-8f)
                continue;

            float limit = command >= 0f
                ? Mathf.Max(0f, thruster.maxForwardThrustN)
                : Mathf.Max(0f, thruster.maxReverseThrustN);
            float thrust = command * limit * scale;
            if (Mathf.Abs(thrust) <= 1e-4f)
                continue;

            Vector3 worldPosition = transform.TransformPoint(thruster.localPosition);
            Vector3 worldDirection = transform.TransformDirection(localDirection.normalized);
            rb.AddForceAtPosition(worldDirection * thrust, worldPosition, ForceMode.Force);
        }
    }

    float GetMappedThrusterOutput(int oneBasedServoChannel)
    {
        if (_52HChannels != null && horizontalOutputs != null)
        {
            int count = Mathf.Min(_52HChannels.Length, horizontalOutputs.Length);
            for (int i = 0; i < count; i++)
            {
                if (_52HChannels[i] == oneBasedServoChannel)
                    return horizontalOutputs[i];
            }
        }

        if (_53VChannels != null && verticalOutputs != null)
        {
            int count = Mathf.Min(_53VChannels.Length, verticalOutputs.Length);
            for (int i = 0; i < count; i++)
            {
                if (_53VChannels[i] == oneBasedServoChannel)
                    return verticalOutputs[i];
            }
        }

        return 0f;
    }

    float GetCommandedWorldVerticalThrusterForce()
    {
        if (_8Thrusters == null)
            return 0f;

        float verticalForce = 0f;
        float scale = Mathf.Max(0f, _56ThrustScale);
        for (int i = 0; i < _8Thrusters.Length; i++)
        {
            SitlThrusterSpec thruster = _8Thrusters[i];
            if (thruster == null)
                continue;

            float command = Mathf.Clamp(GetMappedThrusterOutput(thruster.servoChannel), -1f, 1f);
            if (Mathf.Abs(command) <= Mathf.Max(0f, forceOutputDeadzone))
                continue;

            Vector3 localDirection = thruster.localDirection;
            if (localDirection.sqrMagnitude <= 1e-8f)
                continue;

            float limit = command >= 0f
                ? Mathf.Max(0f, thruster.maxForwardThrustN)
                : Mathf.Max(0f, thruster.maxReverseThrustN);
            float thrust = command * limit * scale;
            if (Mathf.Abs(thrust) <= 1e-4f)
                continue;

            Vector3 worldDirection = transform.TransformDirection(localDirection.normalized);
            verticalForce += worldDirection.y * thrust;
        }

        return verticalForce;
    }

    bool ShouldSuppressWaterSurfaceForcesForDive()
    {
        return _29SuppressDive
            && GetCommandedWorldVerticalThrusterForce() < -Mathf.Max(0f, _30DiveThreshN);
    }

    bool ShouldLogServoPacket()
    {
        if (!logServoPackets)
            return false;

        float interval = Mathf.Max(0f, servoLogIntervalSeconds);
        if (interval <= 0f)
            return true;

        if (Time.unscaledTime - lastServoLogTime < interval)
            return false;

        lastServoLogTime = Time.unscaledTime;
        return true;
    }

    bool ShouldLogForceApplication()
    {
        if (!logForceApplication)
            return false;

        float interval = Mathf.Max(0f, forceLogIntervalSeconds);
        if (interval <= 0f)
            return true;

        if (Time.unscaledTime - lastForceLogTime < interval)
            return false;

        lastForceLogTime = Time.unscaledTime;
        return true;
    }

    bool ShouldLogTelemetryPacket()
    {
        if (!logTelemetryPackets)
            return false;

        float interval = Mathf.Max(0f, telemetryLogIntervalSeconds);
        interval = Mathf.Max(interval, Mathf.Max(0.01f, minTelemetryLogIntervalSeconds));

        if (Time.unscaledTime - lastTelemetryLogTime < interval)
            return false;

        lastTelemetryLogTime = Time.unscaledTime;
        return true;
    }

    bool ShouldLogWaterCurrentForce()
    {
        if (!logWaterCurrentForces)
            return false;

        float interval = Mathf.Max(0f, waterCurrentLogIntervalSeconds);
        if (interval <= 0f)
            return true;

        if (Time.unscaledTime - lastWaterCurrentLogTime < interval)
            return false;

        lastWaterCurrentLogTime = Time.unscaledTime;
        return true;
    }

    void WarnUnmappedActiveServoOutputs(int servoCount)
    {
        if (!warnUnmappedActiveServoOutputs)
            return;
        if (Time.unscaledTime - lastUnmappedServoWarningTime < 1f)
            return;

        string activeUnmapped = FormatActiveServoOutputs(servoCount, onlyUnmapped: true);
        if (string.IsNullOrEmpty(activeUnmapped))
            return;

        lastUnmappedServoWarningTime = Time.unscaledTime;
        Debug.LogWarning($"[ArduPilot JSON] Active PWM on unmapped channels: [{activeUnmapped}]. Current thruster mapping H=[{string.Join(", ", _52HChannels)}] V=[{string.Join(", ", _53VChannels)}]. If these are motors, assign them in the bridge inspector; if they are lights/camera, ignore them.");
    }

    string FormatActiveServoOutputs(int servoCount, bool onlyUnmapped = false)
    {
        int count = Mathf.Clamp(servoCount, 0, servoOutputs.Length);
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < count; i++)
        {
            ushort pwm = servoOutputs[i];
            if (pwm == 0 || Mathf.Abs(pwm - servoPwmNeutral) <= Mathf.Max(0f, servoPwmDeadzone))
                continue;
            if (IsIgnoredUnmappedServoChannel(i + 1))
                continue;
            if (onlyUnmapped && IsMappedServoChannel(i + 1))
                continue;

            if (builder.Length > 0)
                builder.Append(", ");
            builder.Append(i + 1);
            builder.Append(":");
            builder.Append(pwm);
        }

        return builder.ToString();
    }

    bool IsMappedServoChannel(int oneBasedChannel)
    {
        return ContainsChannel(_52HChannels, oneBasedChannel) || ContainsChannel(_53VChannels, oneBasedChannel);
    }

    bool IsIgnoredUnmappedServoChannel(int oneBasedChannel)
    {
        return ContainsChannel(ignoredUnmappedServoChannels, oneBasedChannel);
    }

    static bool ContainsChannel(int[] channels, int oneBasedChannel)
    {
        if (channels == null)
            return false;

        for (int i = 0; i < channels.Length; i++)
        {
            if (channels[i] == oneBasedChannel)
                return true;
        }

        return false;
    }

    void UpdateOutputPacket()
    {
        TelemetryState state = BuildTelemetryState();
        outputPacket.timestamp = state.timestamp;

        outputPacket.imu.gyro = new[]
        {
            CleanJsonFloat(state.gyroFrd.x),
            CleanJsonFloat(state.gyroFrd.y),
            CleanJsonFloat(ShouldReportYawRate() ? state.gyroFrd.z : 0f)
        };
        outputPacket.imu.accel_body = new[]
        {
            CleanJsonFloat(state.accelBodyFrd.x),
            CleanJsonFloat(state.accelBodyFrd.y),
            CleanJsonFloat(state.accelBodyFrd.z)
        };

        outputPacket.position = new[]
        {
            CleanPositionFloat(state.positionNed.x),
            CleanPositionFloat(state.positionNed.y),
            CleanPositionFloat(state.positionNed.z)
        };
        outputPacket.attitude = new[]
        {
            CleanJsonFloat(state.attitudeRadians.x),
            CleanJsonFloat(state.attitudeRadians.y),
            CleanJsonFloat(state.attitudeRadians.z)
        };
        outputPacket.velocity = new[]
        {
            CleanJsonFloat(state.velocityNed.x),
            CleanJsonFloat(state.velocityNed.y),
            CleanJsonFloat(state.velocityNed.z)
        };
    }

    TelemetryState BuildTelemetryState()
    {
        TelemetryState state = new TelemetryState
        {
            timestamp = GetTelemetryTimestampSeconds(),
            stabilizeEkf = ShouldStabilizeEkfBeforeActuatorOutput(),
            unityPosition = GetTelemetryPosition(),
            unityRotation = GetTelemetryRotation(),
            unityVelocity = GetTelemetryVelocity(),
            unityAngularVelocity = GetTelemetryAngularVelocity()
        };

        state.localPosition = zeroUnityPositionAtStart
            ? state.unityPosition - unityOriginPosition
            : state.unityPosition;

        Vector3 rawPositionNed = UnityWorldToNedPosition(state.localPosition);
        state.positionNed = state.stabilizeEkf
            ? Vector3.zero
            : FilterReportedPosition(rawPositionNed, state.timestamp);

        bool clampedAtSurface = state.positionNed.z < 0f;
        if (clampedAtSurface)
            state.positionNed.z = 0f;
        state.positionNed.z = LimitReportedDepthSlew(state.positionNed.z, state.timestamp);

        Vector3 rigidbodyVelocityNed = LimitReportedVelocity(UnityWorldToNedVector(state.unityVelocity));
        Vector3 positionVelocityNed = EstimateVelocityFromPosition(state.positionNed, state.timestamp);
        state.velocityNed = ShouldUsePositionDerivedVelocity(rigidbodyVelocityNed)
            ? positionVelocityNed
            : rigidbodyVelocityNed;

        if (deriveVerticalVelocityFromPosition && ShouldReportVerticalState())
            state.velocityNed.z = EstimateVerticalVelocityFromPosition(state.positionNed.z, state.timestamp);

        if (clampedAtSurface && state.velocityNed.z < 0f)
            state.velocityNed.z = 0f;

        if (state.stabilizeEkf)
            state.velocityNed = Vector3.zero;

        state.accelerationNed = EstimateAccelerationFromVelocity(state.velocityNed, state.timestamp);
        if (state.stabilizeEkf)
        {
            state.accelerationNed = Vector3.zero;
            ResetMotionEstimates(state.positionNed, state.velocityNed, state.timestamp);
        }

        Vector3 rawAttitude = state.stabilizeEkf
            ? ComputeLevelRollPitchAttitudeRadians(state.unityRotation)
            : ComputeAttitudeRadians(state.unityRotation);
        state.attitudeRadians = FilterReportedAttitude(rawAttitude, state.timestamp, out float derivedYawRate);

        state.gyroFrd = UnityAngularVelocityToBodyFrd(state.unityAngularVelocity, state.unityRotation);
        if (deriveReportedYawRateFromAttitude)
            state.gyroFrd.z = derivedYawRate;
        state.gyroFrd.z = LimitYawRate(state.gyroFrd.z);
        if (state.stabilizeEkf)
            state.gyroFrd = Vector3.zero;
        if (zeroRollPitchGyroWhenLevel)
        {
            state.gyroFrd.x = 0f;
            state.gyroFrd.y = 0f;
        }

        float gravity = Mathf.Max(0f, gravityMetersPerSecondSquared);
        Vector3 reportedLinearAccelerationNed = includeLinearAccelerationInImu
            ? LimitReportedAcceleration(state.accelerationNed)
            : Vector3.zero;
        Vector3 specificForceNed = state.stabilizeEkf
            ? new Vector3(0f, 0f, -gravity)
            : reportedLinearAccelerationNed - new Vector3(0f, 0f, gravity);

        state.accelBodyFrd = state.stabilizeEkf
            ? new Vector3(0f, 0f, -gravity)
            : NedVectorToBodyFrd(specificForceNed, state.attitudeRadians);

        UpdateEkfDiagnostics(ref state);
        return state;
    }

    void UpdateEkfDiagnostics(ref TelemetryState state)
    {
        if (hasLastTelemetryDiagnosticState)
        {
            state.dt = Mathf.Max(0.001f, state.timestamp - lastTelemetryDiagnosticState.timestamp);
            state.yawDeltaRadians = NormalizeRadians(state.attitudeRadians.z - lastTelemetryDiagnosticState.attitudeRadians.z);
            state.positionDeltaNed = state.positionNed - lastTelemetryDiagnosticState.positionNed;
            state.positionDerivedVelocityNed = state.positionDeltaNed / state.dt;
            state.velocityMismatchNed = state.velocityNed - state.positionDerivedVelocityNed;
        }
        else
        {
            state.dt = 0f;
            state.yawDeltaRadians = 0f;
            state.positionDeltaNed = Vector3.zero;
            state.positionDerivedVelocityNed = state.velocityNed;
            state.velocityMismatchNed = Vector3.zero;
        }

        LogEkfDiagnostics(state);
        lastTelemetryDiagnosticState = state;
        hasLastTelemetryDiagnosticState = true;
    }

    void LogEkfDiagnostics(TelemetryState state)
    {
        bool hasPrevious = hasLastTelemetryDiagnosticState && state.dt > 0f;
        float yawJumpDeg = Mathf.Abs(state.yawDeltaRadians) * Mathf.Rad2Deg;
        float positionJump = state.positionDeltaNed.magnitude;
        float velocityMismatch = state.velocityMismatchNed.magnitude;
        float gyroZ = Mathf.Abs(state.gyroFrd.z);
        float accel = state.accelBodyFrd.magnitude;

        bool warning = warnEkfDiagnosticJumps && hasPrevious
            && (yawJumpDeg > Mathf.Max(0f, ekfYawJumpWarningDegrees)
                || positionJump > Mathf.Max(0f, ekfPositionJumpWarningMeters)
                || velocityMismatch > Mathf.Max(0f, ekfVelocityMismatchWarningMetersPerSecond)
                || gyroZ > Mathf.Max(0f, ekfGyroZWarningRadiansPerSecond)
                || accel > Mathf.Max(0f, ekfAccelWarningMetersPerSecondSquared) + Mathf.Max(0f, gravityMetersPerSecondSquared));

        if (warning)
        {
            Debug.LogWarning(FormatEkfDiagnosticMessage("[ArduPilot JSON EKF WARN]", state, yawJumpDeg, positionJump, velocityMismatch, gyroZ, accel));
            lastEkfDiagnosticLogTime = Time.unscaledTime;
            return;
        }

        if (!logEkfDiagnostics)
            return;

        float interval = Mathf.Max(0f, ekfDiagnosticLogIntervalSeconds);
        if (interval > 0f && Time.unscaledTime - lastEkfDiagnosticLogTime < interval)
            return;

        Debug.Log(FormatEkfDiagnosticMessage("[ArduPilot JSON EKF]", state, yawJumpDeg, positionJump, velocityMismatch, gyroZ, accel));
        lastEkfDiagnosticLogTime = Time.unscaledTime;
    }

    string FormatEkfDiagnosticMessage(
        string prefix,
        TelemetryState state,
        float yawJumpDeg,
        float positionJump,
        float velocityMismatch,
        float gyroZ,
        float accel)
    {
        return string.Format(
            CultureInfo.InvariantCulture,
            "{0} t={1:0.000} dt={2:0.000} stabilize={3} pos=[{4:0.###},{5:0.###},{6:0.###}] dpos={7:0.###} vel=[{8:0.###},{9:0.###},{10:0.###}] posVel=[{11:0.###},{12:0.###},{13:0.###}] velMismatch={14:0.###} att=[{15:0.###},{16:0.###},{17:0.###}] yawJumpDeg={18:0.###} gyro=[{19:0.###},{20:0.###},{21:0.###}] gyroZAbs={22:0.###} accelBody=[{23:0.###},{24:0.###},{25:0.###}] accelMag={26:0.###}",
            prefix,
            state.timestamp,
            state.dt,
            state.stabilizeEkf,
            state.positionNed.x,
            state.positionNed.y,
            state.positionNed.z,
            positionJump,
            state.velocityNed.x,
            state.velocityNed.y,
            state.velocityNed.z,
            state.positionDerivedVelocityNed.x,
            state.positionDerivedVelocityNed.y,
            state.positionDerivedVelocityNed.z,
            velocityMismatch,
            state.attitudeRadians.x,
            state.attitudeRadians.y,
            state.attitudeRadians.z,
            yawJumpDeg,
            state.gyroFrd.x,
            state.gyroFrd.y,
            state.gyroFrd.z,
            gyroZ,
            state.accelBodyFrd.x,
            state.accelBodyFrd.y,
            state.accelBodyFrd.z,
            accel);
    }

    float CleanJsonFloat(float value)
    {
        if (float.IsNaN(value) || float.IsInfinity(value))
            return 0f;

        float epsilon = Mathf.Max(0f, jsonTinyValueEpsilon);
        return Mathf.Abs(value) <= epsilon ? 0f : value;
    }

    float CleanPositionFloat(float value)
    {
        if (float.IsNaN(value) || float.IsInfinity(value))
            return 0f;

        float epsilon = Mathf.Max(0f, positionTinyValueEpsilon);
        return Mathf.Abs(value) <= epsilon ? 0f : value;
    }

    bool ShouldUsePositionDerivedVelocity(Vector3 rigidbodyVelocityNed)
    {
        if (deriveVelocityFromPosition)
            return true;

        if (!usePositionVelocityWhenRigidbodyIsStopped)
            return false;

        float deadzone = Mathf.Max(0f, rigidbodyVelocityDeadzone);
        return rigidbodyVelocityNed.sqrMagnitude <= deadzone * deadzone;
    }

    float GetTelemetryTimestampSeconds()
    {
        if (telemetryStartRealtime < 0f)
            telemetryStartRealtime = Time.unscaledTime;

        float timestamp = Mathf.Max(0f, Time.unscaledTime - telemetryStartRealtime);

        if (lastOutputTimestamp >= 0f && timestamp <= lastOutputTimestamp)
            timestamp = lastOutputTimestamp + 0.001f;

        lastOutputTimestamp = timestamp;
        return timestamp;
    }

    Vector3 UnityWorldToNedPosition(Vector3 localPosition)
    {
        return UnityWorldToNedVector(localPosition);
    }

    Vector3 GetTelemetryVelocity()
    {
#if UNITY_6000_0_OR_NEWER
        return telemetryRb != null ? telemetryRb.linearVelocity : Vector3.zero;
#else
        return telemetryRb != null ? telemetryRb.velocity : Vector3.zero;
#endif
    }

    Vector3 UnityWorldToNedVector(Vector3 unityVector)
    {
        Vector3 ned = UnityWorldToNedRawVector(unityVector);
        if (invertReportedHorizontalPosition)
        {
            ned.x = -ned.x;
            ned.y = -ned.y;
        }

        if (!ShouldReportVerticalState())
            ned.z = 0f;

        return ned;
    }

    Vector3 UnityWorldToNedRawVector(Vector3 unityVector)
    {
        return worldFrameMapping == WorldFrameMapping.LegacyReferenceBoat
            ? new Vector3(-unityVector.x, unityVector.z, -unityVector.y)
            : new Vector3(unityVector.z, unityVector.x, -unityVector.y);
    }

    Vector3 NedVectorToUnityWorld(Vector3 nedVector)
    {
        if (invertReportedHorizontalPosition)
        {
            nedVector.x = -nedVector.x;
            nedVector.y = -nedVector.y;
        }

        if (worldFrameMapping == WorldFrameMapping.LegacyReferenceBoat)
            return new Vector3(-nedVector.x, -nedVector.z, nedVector.y);

        return new Vector3(nedVector.y, -nedVector.z, nedVector.x);
    }

    Vector3 UnityAngularVelocityToBodyFrd(Vector3 worldAngularVelocity)
    {
        return UnityAngularVelocityToBodyFrd(worldAngularVelocity, GetTelemetryRotation());
    }

    Vector3 UnityAngularVelocityToBodyFrd(Vector3 worldAngularVelocity, Quaternion unityRotation)
    {
        Quaternion bodyRotation = GetBodyWorldRotation(unityRotation);
        Vector3 local = Quaternion.Inverse(bodyRotation) * worldAngularVelocity;
        if (worldFrameMapping == WorldFrameMapping.LegacyReferenceBoat)
        {
            return new Vector3(
                local.x,
                local.z,
                yawRateSign * local.y);
        }

        return new Vector3(
            -rollAngleSign * local.z,
            -pitchAngleSign * local.x,
            yawRateSign * local.y);
    }

    bool ShouldReportYawRate()
    {
        return reportYawRate && !freezeYawRate;
    }

    bool ShouldReportVerticalState()
    {
        return reportVerticalState
            && !freezeVerticalState
            && !forceStableVerticalReport
            && !ShouldStabilizeEkfBeforeActuatorOutput();
    }

    bool ShouldStabilizeEkfBeforeActuatorOutput()
    {
        return stabilizeEkfBeforeActuatorOutput && !hasSeenActiveThrusterOutput;
    }

    Vector3 NedVectorToBodyFrd(Vector3 nedVector)
    {
        Vector3 unityWorld = NedVectorToUnityWorld(nedVector);
        Vector3 local = BodyInverseTransformDirection(unityWorld);
        return new Vector3(local.z, local.x, -local.y);
    }

    static Vector3 NedVectorToBodyFrd(Vector3 nedVector, Vector3 attitudeRadians)
    {
        float roll = attitudeRadians.x;
        float pitch = attitudeRadians.y;
        float yaw = attitudeRadians.z;

        float cr = Mathf.Cos(roll);
        float sr = Mathf.Sin(roll);
        float cp = Mathf.Cos(pitch);
        float sp = Mathf.Sin(pitch);
        float cy = Mathf.Cos(yaw);
        float sy = Mathf.Sin(yaw);

        float r00 = cp * cy;
        float r01 = sr * sp * cy - cr * sy;
        float r02 = cr * sp * cy + sr * sy;
        float r10 = cp * sy;
        float r11 = sr * sp * sy + cr * cy;
        float r12 = cr * sp * sy - sr * cy;
        float r20 = -sp;
        float r21 = sr * cp;
        float r22 = cr * cp;

        return new Vector3(
            r00 * nedVector.x + r10 * nedVector.y + r20 * nedVector.z,
            r01 * nedVector.x + r11 * nedVector.y + r21 * nedVector.z,
            r02 * nedVector.x + r12 * nedVector.y + r22 * nedVector.z);
    }

    Vector3 ComputeAttitudeRadians()
    {
        return ComputeAttitudeRadians(GetTelemetryRotation());
    }

    Vector3 ComputeAttitudeRadians(Quaternion unityRotation)
    {
        Vector3 attitude = ComputeRawAttitudeRadians(unityRotation);
        if (zeroReportedRollPitchAtStart && hasInitialReportedRollPitch)
        {
            attitude.x = NormalizeRadians(attitude.x - initialReportedRollPitchRadians.x);
            attitude.y = NormalizeRadians(attitude.y - initialReportedRollPitchRadians.y);
        }

        return attitude;
    }

    Vector3 ComputeLevelRollPitchAttitudeRadians()
    {
        return ComputeLevelRollPitchAttitudeRadians(GetTelemetryRotation());
    }

    Vector3 ComputeLevelRollPitchAttitudeRadians(Quaternion unityRotation)
    {
        Vector3 attitude = ComputeAttitudeRadians(unityRotation);
        attitude.x = 0f;
        attitude.y = 0f;
        if (hasInitialReportedRollPitch)
            attitude.z = initialReportedYawRadians;
        return attitude;
    }

    Vector3 ComputeRawAttitudeRadians()
    {
        return ComputeRawAttitudeRadians(GetTelemetryRotation());
    }

    Vector3 ComputeRawAttitudeRadians(Quaternion unityRotation)
    {
        Quaternion bodyRotation = GetBodyWorldRotation(unityRotation);
        Vector3 forwardNed = UnityWorldToNedRawVector(bodyRotation * Vector3.forward).normalized;
        Vector3 rightNed = UnityWorldToNedRawVector(bodyRotation * Vector3.right).normalized;
        Vector3 downNed = UnityWorldToNedRawVector(-(bodyRotation * Vector3.up)).normalized;

        float maxDegrees = Mathf.Max(0f, maxReportedRollPitchDegrees);
        float pitch = -Mathf.Asin(Mathf.Clamp(forwardNed.z, -1f, 1f));
        float roll = Mathf.Atan2(rightNed.z, downNed.z);
        float yaw = Mathf.Atan2(forwardNed.y, forwardNed.x);

        if (forceLevelRollPitchReport)
        {
            roll = 0f;
            pitch = 0f;
        }

        roll *= rollAngleSign;
        pitch *= pitchAngleSign;
        yaw *= yawAngleSign;

        if (maxDegrees > 0f)
        {
            float maxRad = maxDegrees * Mathf.Deg2Rad;
            roll = Mathf.Clamp(roll, -maxRad, maxRad);
            pitch = Mathf.Clamp(pitch, -maxRad, maxRad);
        }

        return new Vector3(roll, pitch, NormalizeRadians(yaw));
    }

    void CaptureInitialReportedRollPitch()
    {
        if (!zeroReportedRollPitchAtStart)
            return;

        Vector3 initialAttitude = ComputeRawAttitudeRadians();
        initialReportedRollPitchRadians = new Vector2(initialAttitude.x, initialAttitude.y);
        initialReportedYawRadians = initialAttitude.z;
        hasInitialReportedRollPitch = true;
    }

    Vector3 FilterReportedAttitude(Vector3 rawAttitude, double timestamp, out float yawRate)
    {
        rawAttitude.z = NormalizeRadians(rawAttitude.z);
        yawRate = 0f;

        if (!filterReportedAttitude)
        {
            if (hasFilteredAttitude && lastAttitudeFilterTimestamp >= 0.0 && timestamp > lastAttitudeFilterTimestamp)
            {
                float dt = Mathf.Clamp((float)(timestamp - lastAttitudeFilterTimestamp), 0.001f, 0.2f);
                float previousUnfilteredYaw = lastFilteredYawRadians;
                rawAttitude.z = LimitYawSlew(previousUnfilteredYaw, rawAttitude.z, dt);
                yawRate = NormalizeRadians(rawAttitude.z - previousUnfilteredYaw) / dt;
            }

            filteredAttitudeRadians = rawAttitude;
            lastFilteredYawRadians = rawAttitude.z;
            lastAttitudeFilterTimestamp = timestamp;
            hasFilteredAttitude = true;
            return filteredAttitudeRadians;
        }

        if (!hasFilteredAttitude || lastAttitudeFilterTimestamp < 0.0 || timestamp <= lastAttitudeFilterTimestamp)
        {
            filteredAttitudeRadians = rawAttitude;
            lastFilteredYawRadians = rawAttitude.z;
            lastAttitudeFilterTimestamp = timestamp;
            hasFilteredAttitude = true;
            return filteredAttitudeRadians;
        }

        float dtFiltered = Mathf.Clamp((float)(timestamp - lastAttitudeFilterTimestamp), 0.001f, 0.2f);
        float previousYaw = filteredAttitudeRadians.z;
        float alpha = Mathf.Clamp01(reportedAttitudeSmoothing);

        filteredAttitudeRadians.x = Mathf.Lerp(filteredAttitudeRadians.x, rawAttitude.x, alpha);
        filteredAttitudeRadians.y = Mathf.Lerp(filteredAttitudeRadians.y, rawAttitude.y, alpha);
        filteredAttitudeRadians.z = NormalizeRadians(filteredAttitudeRadians.z + NormalizeRadians(rawAttitude.z - filteredAttitudeRadians.z) * alpha);
        filteredAttitudeRadians.z = LimitYawSlew(previousYaw, filteredAttitudeRadians.z, dtFiltered);

        yawRate = NormalizeRadians(filteredAttitudeRadians.z - previousYaw) / dtFiltered;
        lastFilteredYawRadians = filteredAttitudeRadians.z;
        lastAttitudeFilterTimestamp = timestamp;
        return filteredAttitudeRadians;
    }

    float LimitYawSlew(float previousYaw, float targetYaw, float dt)
    {
        float limit = Mathf.Max(0f, maxReportedYawRateRadiansPerSecond);
        if (limit <= 0f)
            return NormalizeRadians(targetYaw);

        float maxStep = limit * Mathf.Max(0.001f, dt);
        float delta = Mathf.Clamp(NormalizeRadians(targetYaw - previousYaw), -maxStep, maxStep);
        return NormalizeRadians(previousYaw + delta);
    }

    float LimitYawRate(float yawRate)
    {
        float limit = Mathf.Max(0f, maxReportedYawRateRadiansPerSecond);
        return limit > 0f ? Mathf.Clamp(yawRate, -limit, limit) : yawRate;
    }

    Quaternion GetBodyWorldRotation()
    {
        return GetBodyWorldRotation(GetTelemetryRotation());
    }

    Quaternion GetBodyWorldRotation(Quaternion unityRotation)
    {
        return unityRotation * Quaternion.Euler(bodyEulerOffsetDegrees);
    }

    Transform GetTelemetryTransform()
    {
        if (resolvedTelemetryTransform == null)
            ResolveTelemetrySource();

        return resolvedTelemetryTransform != null ? resolvedTelemetryTransform : transform;
    }

    void ResolveTelemetrySource()
    {
        resolvedTelemetryTransform = _2Target != null
            ? _2Target
            : transform;

        telemetryRb = resolvedTelemetryTransform != null
            ? resolvedTelemetryTransform.GetComponentInParent<Rigidbody>()
            : null;
        if (telemetryRb == null)
            telemetryRb = rb;
    }

    void RefreshVehicleModel()
    {
        Rigidbody sourceRb = telemetryRb != null ? telemetryRb : rb;
        if (sourceRb != null)
            _3MassKg = Mathf.Max(0.001f, sourceRb.mass);

        if (_8Thrusters == null || _8Thrusters.Length == 0)
            _8Thrusters = CreateDefaultVectoredThrusterSpecs();

        NormalizeSitlThrusterSpecs();
    }

    SitlThrusterSpec BuildSitlThrusterSpec(
        string thrusterName,
        int servoChannel,
        Vector3 localPosition,
        Vector3 localDirection,
        float maxForwardThrustN,
        float maxReverseThrustN)
    {
        if (localDirection.sqrMagnitude <= 1e-8f)
            localDirection = Vector3.forward;

        return new SitlThrusterSpec
        {
            name = thrusterName,
            servoChannel = Mathf.Clamp(servoChannel, 1, 32),
            localPosition = localPosition,
            localDirection = localDirection.normalized,
            maxForwardThrustN = Mathf.Max(0f, maxForwardThrustN),
            maxReverseThrustN = Mathf.Max(0f, maxReverseThrustN)
        };
    }

    SitlThrusterSpec[] CreateDefaultVectoredThrusterSpecs()
    {
        const float x = 0.17f;
        const float y = -0.045f;
        const float z = 0.20f;
        const float verticalX = 0.12f;
        float horizontalThrust = Mathf.Max(0f, _6HThrustN);
        float verticalThrust = Mathf.Max(0f, _7VThrustN);

        return new[]
        {
            BuildSitlThrusterSpec("M1 Front Right", GetOneBasedChannel(_52HChannels, 0), new Vector3( x, y,  z), new Vector3( 1f, 0f, -1f), horizontalThrust, horizontalThrust),
            BuildSitlThrusterSpec("M2 Front Left",  GetOneBasedChannel(_52HChannels, 1), new Vector3(-x, y,  z), new Vector3(-1f, 0f, -1f), horizontalThrust, horizontalThrust),
            BuildSitlThrusterSpec("M3 Rear Right",  GetOneBasedChannel(_52HChannels, 2), new Vector3( x, y, -z), new Vector3( 1f, 0f,  1f), horizontalThrust, horizontalThrust),
            BuildSitlThrusterSpec("M4 Rear Left",   GetOneBasedChannel(_52HChannels, 3), new Vector3(-x, y, -z), new Vector3(-1f, 0f,  1f), horizontalThrust, horizontalThrust),
            BuildSitlThrusterSpec("M5 Left Vertical",  GetOneBasedChannel(_53VChannels, 0), new Vector3(-verticalX, 0f, 0f), Vector3.up, verticalThrust, verticalThrust),
            BuildSitlThrusterSpec("M6 Right Vertical", GetOneBasedChannel(_53VChannels, 1), new Vector3( verticalX, 0f, 0f), Vector3.up, verticalThrust, verticalThrust),
        };
    }

    void NormalizeSitlThrusterSpecs()
    {
        if (_8Thrusters == null)
            return;

        for (int i = 0; i < _8Thrusters.Length; i++)
        {
            SitlThrusterSpec thruster = _8Thrusters[i];
            if (thruster == null)
            {
                thruster = BuildSitlThrusterSpec($"Thruster {i + 1}", 1, Vector3.zero, Vector3.forward, 0f, 0f);
                _8Thrusters[i] = thruster;
            }

            thruster.servoChannel = Mathf.Clamp(thruster.servoChannel, 1, 32);
            if (thruster.localDirection.sqrMagnitude <= 1e-8f)
                thruster.localDirection = Vector3.forward;
            else
                thruster.localDirection.Normalize();
            thruster.maxForwardThrustN = Mathf.Max(0f, thruster.maxForwardThrustN);
            thruster.maxReverseThrustN = Mathf.Max(0f, thruster.maxReverseThrustN);
        }
    }

    void ApplyVehicleModelToRigidbody()
    {
        if (rb == null)
            return;

        rb.mass = Mathf.Max(0.001f, _3MassKg);
#if UNITY_6000_0_OR_NEWER
        rb.linearDamping = IsWaterCurrentActive() ? 0f : Mathf.Max(0f, _4LinDrag);
#else
        rb.drag = IsWaterCurrentActive() ? 0f : Mathf.Max(0f, _4LinDrag);
#endif
        rb.angularDamping = Mathf.Max(0f, _5AngDrag);
    }

    Vector3 GetTelemetryPosition()
    {
        return telemetryRb != null ? telemetryRb.position : GetTelemetryTransform().position;
    }

    Quaternion GetTelemetryRotation()
    {
        return telemetryRb != null ? telemetryRb.rotation : GetTelemetryTransform().rotation;
    }

    Vector3 GetTelemetryAngularVelocity()
    {
        return telemetryRb != null ? telemetryRb.angularVelocity : Vector3.zero;
    }

    float GetWaterSurfaceY()
    {
        ResolveWaterSurfaceIfNeeded();
        if (_58Surface != null && TryGetWaterSurfaceY(GetTelemetryPosition(), out float y))
            return y;

        return _59SurfaceY;
    }

    void ResolveWaterSurfaceIfNeeded()
    {
        if (_58Surface != null)
            return;

        _58Surface = FindFirstObjectByType<WaterSurface>();
        hasWaterSearchCandidate = false;
        ResetFilteredWaterSurface();
    }

    bool TryGetWaterSurfaceY(Vector3 worldPosition, out float surfaceY)
    {
        if (!hasWaterSearchCandidate)
        {
            waterSearchResult.candidateLocationWS = worldPosition;
            hasWaterSearchCandidate = true;
        }

        waterSearchParameters.startPositionWS = waterSearchResult.candidateLocationWS;
        waterSearchParameters.targetPositionWS = worldPosition;
        waterSearchParameters.error = _60QueryError;
        waterSearchParameters.maxIterations = _61QueryIter;
        waterSearchParameters.outputNormal = false;

        if (_58Surface.ProjectPointOnWaterSurface(waterSearchParameters, out waterSearchResult))
        {
            surfaceY = waterSearchResult.projectedPositionWS.y;
            return true;
        }

        hasWaterSearchCandidate = false;
        surfaceY = _59SurfaceY;
        return false;
    }

    Vector3[] GetWaterSurfaceSamplePoints()
    {
        if (_21SamplePts == null || _21SamplePts.Length == 0)
            return null;

        if (surfaceWorldSamplePoints == null || surfaceWorldSamplePoints.Length != _21SamplePts.Length)
            surfaceWorldSamplePoints = new Vector3[_21SamplePts.Length];

        Transform referenceTransform = GetTelemetryTransform();
        for (int i = 0; i < _21SamplePts.Length; i++)
            surfaceWorldSamplePoints[i] = referenceTransform.TransformPoint(_21SamplePts[i]);

        return surfaceWorldSamplePoints;
    }

    void EnsureWaterSurfaceSampleBuffers(int count)
    {
        count = Mathf.Max(0, count);
        if (filteredWaterSurfaceSampleY == null || filteredWaterSurfaceSampleY.Length != count)
        {
            filteredWaterSurfaceSampleY = new float[count];
            hasFilteredWaterSurfaceSamples = false;
        }
        if (validWaterSurfaceSampleY == null || validWaterSurfaceSampleY.Length != count)
            validWaterSurfaceSampleY = new bool[count];
    }

    float FilterWaterSurfaceSampleY(int index, float rawSurfaceY)
    {
        if (!_31UseLowPass || _32LowPassS <= 0f)
        {
            filteredWaterSurfaceSampleY[index] = rawSurfaceY;
            return rawSurfaceY;
        }

        if (!hasFilteredWaterSurfaceSamples)
        {
            filteredWaterSurfaceSampleY[index] = rawSurfaceY;
            if (index == filteredWaterSurfaceSampleY.Length - 1)
                hasFilteredWaterSurfaceSamples = true;
            return rawSurfaceY;
        }

        filteredWaterSurfaceSampleY[index] = Mathf.Lerp(filteredWaterSurfaceSampleY[index], rawSurfaceY, GetWaterSurfaceLowPassAlpha());
        return filteredWaterSurfaceSampleY[index];
    }

    float GetWaterSurfaceLowPassAlpha()
    {
        float tau = Mathf.Max(0.001f, _32LowPassS);
        float dt = Mathf.Max(0.001f, Time.fixedDeltaTime);
        return Mathf.Clamp01(1f - Mathf.Exp(-dt / tau));
    }

    void ResetFilteredWaterSurface()
    {
        hasFilteredWaterSurfaceSamples = false;
        hasLastWaterSurfaceWaveDisplacementY = false;
        hasLastWaterParticleVelocityY = false;
        if (validWaterSurfaceSampleY != null)
            Array.Clear(validWaterSurfaceSampleY, 0, validWaterSurfaceSampleY.Length);
    }

    void EnforceWaterSurfaceNoBreachClamp()
    {
        if (rb == null || !_12UseSurface)
            return;

        if (ShouldSuppressWaterSurfaceForcesForDive())
            return;

        ResolveWaterSurfaceIfNeeded();
        if (_58Surface == null)
            return;

        float hardLimitY = GetWaterSurfaceY() - Mathf.Max(0f, _22ClampBelowM);
        float overshoot = rb.position.y - hardLimitY;
        if (overshoot <= 0f)
            return;

        float softenTime = Mathf.Max(0f, _25ClampSoftenS);
        float alpha = softenTime > 0f
            ? 1f - Mathf.Exp(-Mathf.Max(0.0001f, Time.fixedDeltaTime) / softenTime)
            : 1f;

        float maxOvershoot = Mathf.Max(0f, _26ClampHardM);
        float correctedOvershoot = overshoot * (1f - alpha);
        bool hitHardMargin = correctedOvershoot > maxOvershoot;
        if (hitHardMargin)
            correctedOvershoot = maxOvershoot;

        Vector3 position = rb.position;
        position.y = hardLimitY + correctedOvershoot;
        rb.position = position;

        if (!_24ZeroUpVel)
            return;

#if UNITY_6000_0_OR_NEWER
        Vector3 velocity = rb.linearVelocity;
        if (velocity.y > 0f)
        {
            velocity.y = hitHardMargin ? 0f : velocity.y * (1f - alpha);
            rb.linearVelocity = velocity;
        }
#else
        Vector3 velocity = rb.velocity;
        if (velocity.y > 0f)
        {
            velocity.y = hitHardMargin ? 0f : velocity.y * (1f - alpha);
            rb.velocity = velocity;
        }
#endif
    }

    Vector3 BodyInverseTransformDirection(Vector3 worldDirection)
    {
        return Quaternion.Inverse(GetBodyWorldRotation()) * worldDirection;
    }

    Vector3 FilterReportedPosition(Vector3 rawPositionNed, double timestamp)
    {
        if (!filterReportedPosition)
        {
            filteredPositionNed = rawPositionNed;
            hasFilteredPosition = true;
            return rawPositionNed;
        }

        if (!hasFilteredPosition || lastPositionFilterTimestamp < 0.0)
        {
            filteredPositionNed = rawPositionNed;
            hasFilteredPosition = true;
            lastPositionFilterTimestamp = timestamp;
            return filteredPositionNed;
        }

        float dt = Mathf.Clamp((float)(timestamp - lastPositionFilterTimestamp), 0.001f, 0.2f);
        Vector3 delta = rawPositionNed - filteredPositionNed;
        if (!ShouldReportVerticalState())
            delta.z = 0f;

        Vector2 horizontalDelta = new Vector2(delta.x, delta.y);
        float maxStepFromSpeed = Mathf.Max(0.01f, maxReportedHorizontalSpeed) * dt;
        float maxStep = Mathf.Max(0.01f, Mathf.Min(Mathf.Max(0.01f, maxReportedHorizontalStepMeters), maxStepFromSpeed));
        if (horizontalDelta.sqrMagnitude > maxStep * maxStep)
        {
            horizontalDelta = horizontalDelta.normalized * maxStep;
            delta.x = horizontalDelta.x;
            delta.y = horizontalDelta.y;
        }

        if (ShouldReportVerticalState())
        {
            float verticalLimit = Mathf.Max(0f, maxReportedVerticalSpeed);
            float maxVerticalStep = verticalLimit > 0f ? verticalLimit * dt : 0f;
            delta.z = maxVerticalStep > 0f ? Mathf.Clamp(delta.z, -maxVerticalStep, maxVerticalStep) : 0f;
        }

        filteredPositionNed += delta * Mathf.Clamp01(reportedPositionSmoothing);
        if (!ShouldReportVerticalState())
            filteredPositionNed.z = 0f;

        lastPositionFilterTimestamp = timestamp;
        return filteredPositionNed;
    }

    Vector3 EstimateVelocityFromPosition(Vector3 positionNed, double timestamp)
    {
        if (lastVelocityEstimateTimestamp < 0.0 || timestamp <= lastVelocityEstimateTimestamp)
        {
            lastVelocityEstimateTimestamp = timestamp;
            lastReportedPositionNed = positionNed;
            filteredVelocityNed = Vector3.zero;
            return filteredVelocityNed;
        }

        float dt = Mathf.Clamp((float)(timestamp - lastVelocityEstimateTimestamp), 0.001f, 0.2f);
        Vector3 velocityNed = (positionNed - lastReportedPositionNed) / dt;
        velocityNed = LimitReportedVelocity(velocityNed);
        filteredVelocityNed = Vector3.Lerp(filteredVelocityNed, velocityNed, 0.35f);

        lastVelocityEstimateTimestamp = timestamp;
        lastReportedPositionNed = positionNed;
        return filteredVelocityNed;
    }

    float EstimateVerticalVelocityFromPosition(float positionDownMeters, double timestamp)
    {
        if (!hasPreviousVerticalPosition || lastVerticalVelocityTimestamp < 0.0 || timestamp <= lastVerticalVelocityTimestamp)
        {
            lastVerticalPositionDown = positionDownMeters;
            filteredVerticalVelocityDown = 0f;
            lastVerticalVelocityTimestamp = timestamp;
            hasPreviousVerticalPosition = true;
            return filteredVerticalVelocityDown;
        }

        float dt = Mathf.Clamp((float)(timestamp - lastVerticalVelocityTimestamp), 0.001f, 0.2f);
        float velocityDown = (positionDownMeters - lastVerticalPositionDown) / dt;
        float verticalLimit = Mathf.Max(0f, maxReportedVerticalSpeed);
        if (verticalLimit > 0f)
            velocityDown = Mathf.Clamp(velocityDown, -verticalLimit, verticalLimit);

        if (Mathf.Abs(velocityDown) <= Mathf.Max(0f, verticalVelocityDeadzone))
            velocityDown = 0f;

        filteredVerticalVelocityDown = Mathf.Lerp(filteredVerticalVelocityDown, velocityDown, Mathf.Clamp01(verticalVelocitySmoothing));
        lastVerticalPositionDown = positionDownMeters;
        lastVerticalVelocityTimestamp = timestamp;
        return filteredVerticalVelocityDown;
    }

    float LimitReportedDepthSlew(float rawDepthDownMeters, double timestamp)
    {
        float depth = Mathf.Max(0f, rawDepthDownMeters);
        if (!ShouldReportVerticalState())
        {
            slewLimitedDepthDown = 0f;
            lastDepthSlewTimestamp = timestamp;
            hasSlewLimitedDepth = true;
            return 0f;
        }

        if (!hasSlewLimitedDepth || lastDepthSlewTimestamp < 0.0 || timestamp <= lastDepthSlewTimestamp)
        {
            slewLimitedDepthDown = depth;
            lastDepthSlewTimestamp = timestamp;
            hasSlewLimitedDepth = true;
            return slewLimitedDepthDown;
        }

        float dt = Mathf.Clamp((float)(timestamp - lastDepthSlewTimestamp), 0.001f, 0.2f);
        float verticalLimit = Mathf.Max(0f, maxReportedVerticalSpeed);
        if (verticalLimit <= 0f)
        {
            slewLimitedDepthDown = depth;
        }
        else
        {
            float maxStep = verticalLimit * dt;
            slewLimitedDepthDown += Mathf.Clamp(depth - slewLimitedDepthDown, -maxStep, maxStep);
            if (slewLimitedDepthDown < 0f)
                slewLimitedDepthDown = 0f;
        }

        lastDepthSlewTimestamp = timestamp;
        return slewLimitedDepthDown;
    }

    Vector3 LimitReportedVelocity(Vector3 velocityNed)
    {
        float horizontalLimit = Mathf.Max(0f, maxReportedHorizontalSpeed);
        Vector2 horizontal = new Vector2(velocityNed.x, velocityNed.y);
        if (horizontalLimit > 0f && horizontal.sqrMagnitude > horizontalLimit * horizontalLimit)
        {
            horizontal = horizontal.normalized * horizontalLimit;
            velocityNed.x = horizontal.x;
            velocityNed.y = horizontal.y;
        }

        if (!ShouldReportVerticalState())
        {
            velocityNed.z = 0f;
            return velocityNed;
        }

        float verticalLimit = Mathf.Max(0f, maxReportedVerticalSpeed);
        if (verticalLimit > 0f)
            velocityNed.z = Mathf.Clamp(velocityNed.z, -verticalLimit, verticalLimit);
        return velocityNed;
    }

    Vector3 EstimateAccelerationFromVelocity(Vector3 velocityNed, double timestamp)
    {
        if (!hasPreviousVelocityNed || lastAccelerationTimestamp < 0.0 || timestamp <= lastAccelerationTimestamp)
        {
            lastVelocityNed = velocityNed;
            filteredAccelNed = Vector3.zero;
            lastAccelerationTimestamp = timestamp;
            hasPreviousVelocityNed = true;
            return filteredAccelNed;
        }

        float dt = Mathf.Clamp((float)(timestamp - lastAccelerationTimestamp), 0.001f, 0.2f);
        Vector3 accelerationNed = (velocityNed - lastVelocityNed) / dt;
        filteredAccelNed = Vector3.Lerp(filteredAccelNed, accelerationNed, Mathf.Clamp01(accelerationSmoothing));
        lastVelocityNed = velocityNed;
        lastAccelerationTimestamp = timestamp;
        return filteredAccelNed;
    }

    Vector3 LimitReportedAcceleration(Vector3 accelerationNed)
    {
        float limit = Mathf.Max(0f, maxReportedAccelerationMetersPerSecondSquared);
        if (limit <= 0f || accelerationNed.sqrMagnitude <= limit * limit)
            return accelerationNed;

        return accelerationNed.normalized * limit;
    }

    void ResetMotionEstimates(Vector3 positionNed, Vector3 velocityNed, double timestamp)
    {
        filteredPositionNed = positionNed;
        lastReportedPositionNed = positionNed;
        filteredVelocityNed = velocityNed;
        lastVelocityNed = velocityNed;
        filteredAccelNed = Vector3.zero;
        lastVerticalPositionDown = positionNed.z;
        slewLimitedDepthDown = Mathf.Max(0f, positionNed.z);
        filteredVerticalVelocityDown = 0f;
        lastPositionFilterTimestamp = timestamp;
        lastVelocityEstimateTimestamp = timestamp;
        lastAccelerationTimestamp = timestamp;
        lastVerticalVelocityTimestamp = timestamp;
        lastDepthSlewTimestamp = timestamp;
        hasFilteredPosition = true;
        hasPreviousVelocityNed = true;
        hasPreviousVerticalPosition = true;
        hasSlewLimitedDepth = true;
    }

    string BuildTelemetryJson()
    {
        return string.Format(
            CultureInfo.InvariantCulture,
            "{{\"timestamp\":{0:R},\"no_time_sync\":true,\"no_lockstep\":true,\"imu\":{{\"gyro\":[{1:R},{2:R},{3:R}],\"accel_body\":[{4:R},{5:R},{6:R}]}},\"position\":[{7:R},{8:R},{9:R}],\"attitude\":[{10:R},{11:R},{12:R}],\"velocity\":[{13:R},{14:R},{15:R}]}}",
            (double)outputPacket.timestamp,
            (double)outputPacket.imu.gyro[0],
            (double)outputPacket.imu.gyro[1],
            (double)outputPacket.imu.gyro[2],
            (double)outputPacket.imu.accel_body[0],
            (double)outputPacket.imu.accel_body[1],
            (double)outputPacket.imu.accel_body[2],
            (double)outputPacket.position[0],
            (double)outputPacket.position[1],
            (double)outputPacket.position[2],
            (double)outputPacket.attitude[0],
            (double)outputPacket.attitude[1],
            (double)outputPacket.attitude[2],
            (double)outputPacket.velocity[0],
            (double)outputPacket.velocity[1],
            (double)outputPacket.velocity[2]);
    }

    void UpdateMappedOutputsFromServoPwm()
    {
        EnsureOutputArrays();
        float horizontalSign = invertHorizontalMotorOutputs ? -1f : 1f;
        for (int i = 0; i < horizontalOutputTargets.Length; i++)
        {
            int channel = GetChannelIndex(_52HChannels, i);
            horizontalOutputTargets[i] = channel >= 0 && channel < servoOutputs.Length
                ? NormalizeServoPwm(servoOutputs[channel]) * GetOutputScale(_54HScales, i) * horizontalSign
                : 0f;
        }

        for (int i = 0; i < verticalOutputTargets.Length; i++)
        {
            int channel = GetChannelIndex(_53VChannels, i);
            verticalOutputTargets[i] = channel >= 0 && channel < servoOutputs.Length
                ? NormalizeServoPwm(servoOutputs[channel]) * GetOutputScale(_55VScales, i)
                : 0f;
        }

        if (servoOutputMode == ServoOutputMode.MotorOutputs)
        {
            ScaleHorizontalYawComponent(horizontalOutputTargets);
            ApplyVerticalAxisScales(verticalOutputTargets);
        }
        else if (servoOutputMode == ServoOutputMode.AxisMixedOutputs)
        {
            ApplyHorizontalAxisScales(horizontalOutputTargets);
            ApplyVerticalAxisScales(verticalOutputTargets);
        }

        SmoothOutputArray(horizontalOutputTargets, horizontalOutputs);
        SmoothOutputArray(verticalOutputTargets, verticalOutputs);
        lastActuatorOutputTime = Time.unscaledTime;
        if (HasActiveThrusterOutput())
            hasSeenActiveThrusterOutput = true;
    }

    bool HasActiveThrusterOutput()
    {
        float threshold = Mathf.Max(0f, forceOutputDeadzone);
        if (HasActiveOutput(horizontalOutputs, threshold))
            return true;
        if (HasActiveOutput(verticalOutputs, threshold))
            return true;

        return false;
    }

    static bool HasActiveOutput(float[] values, float threshold)
    {
        if (values == null)
            return false;

        for (int i = 0; i < values.Length; i++)
        {
            if (Mathf.Abs(values[i]) > threshold)
                return true;
        }

        return false;
    }

    void EnsureOutputArrays()
    {
        int horizontalCount = _52HChannels != null ? _52HChannels.Length : 0;
        int verticalCount = _53VChannels != null ? _53VChannels.Length : 0;

        if (horizontalOutputs == null || horizontalOutputs.Length != horizontalCount)
            horizontalOutputs = new float[horizontalCount];
        if (verticalOutputs == null || verticalOutputs.Length != verticalCount)
            verticalOutputs = new float[verticalCount];
        if (horizontalOutputTargets == null || horizontalOutputTargets.Length != horizontalCount)
            horizontalOutputTargets = new float[horizontalCount];
        if (verticalOutputTargets == null || verticalOutputTargets.Length != verticalCount)
            verticalOutputTargets = new float[verticalCount];
    }

    void SmoothOutputArray(float[] target, float[] output)
    {
        if (target == null || output == null)
            return;

        float dt = Mathf.Max(0.001f, Time.unscaledDeltaTime);
        float alpha = Mathf.Clamp01(actuatorOutputSmoothing);
        float maxStep = Mathf.Max(0f, maxActuatorOutputSlewPerSecond) * dt;
        int count = Mathf.Min(target.Length, output.Length);
        for (int i = 0; i < count; i++)
        {
            float value = Mathf.Lerp(output[i], target[i], alpha);
            output[i] = maxStep > 0f
                ? Mathf.MoveTowards(output[i], value, maxStep)
                : value;
        }
    }

    void ApplyHorizontalAxisScales(float[] outputs)
    {
        if (outputs == null || outputs.Length < 4)
            return;

        float h0 = outputs[0];
        float h1 = outputs[1];
        float h2 = outputs[2];
        float h3 = outputs[3];

        float surge = (h0 + h1 + h2 + h3) * 0.25f * horizontalSurgeScale;
        float sway = (h0 - h1 - h2 + h3) * 0.25f * horizontalSwayScale;
        float yaw = (h0 + h1 - h2 - h3) * 0.25f * horizontalYawScale;

        outputs[0] = Mathf.Clamp(surge + sway + yaw, -1f, 1f);
        outputs[1] = Mathf.Clamp(surge - sway + yaw, -1f, 1f);
        outputs[2] = Mathf.Clamp(surge - sway - yaw, -1f, 1f);
        outputs[3] = Mathf.Clamp(surge + sway - yaw, -1f, 1f);
    }

    void ScaleHorizontalYawComponent(float[] outputs)
    {
        if (outputs == null || outputs.Length < 4)
            return;

        float h0 = outputs[0];
        float h1 = outputs[1];
        float h2 = outputs[2];
        float h3 = outputs[3];

        float yaw = (h0 - h1 - h2 + h3) * 0.25f;
        float yawDelta = (Mathf.Clamp01(motorOutputYawScale) - 1f) * yaw;

        outputs[0] = Mathf.Clamp(h0 + yawDelta, -1f, 1f);
        outputs[1] = Mathf.Clamp(h1 - yawDelta, -1f, 1f);
        outputs[2] = Mathf.Clamp(h2 - yawDelta, -1f, 1f);
        outputs[3] = Mathf.Clamp(h3 + yawDelta, -1f, 1f);
    }

    void ApplyVerticalAxisScales(float[] outputs)
    {
        if (outputs == null || outputs.Length < 2)
            return;

        float left = outputs[0];
        float right = outputs[1];
        float heave = (left + right) * 0.5f * verticalHeaveScale;
        float differential = (left - right) * 0.5f * verticalDifferentialScale;

        outputs[0] = Mathf.Clamp(heave + differential, -1f, 1f);
        outputs[1] = Mathf.Clamp(heave - differential, -1f, 1f);
    }

    float NormalizeServoPwm(ushort pwm)
    {
        if (pwm == 0)
            return 0f;

        float delta = pwm - servoPwmNeutral;
        if (Mathf.Abs(delta) <= Mathf.Max(0f, servoPwmDeadzone))
            return 0f;

        float range = delta >= 0f
            ? Mathf.Max(1f, servoPwmMax - servoPwmNeutral)
            : Mathf.Max(1f, servoPwmNeutral - servoPwmMin);
        return Mathf.Clamp(delta / range, -1f, 1f);
    }

    void EnsureInspectorDefaults()
    {
        _1Port = Mathf.Clamp(_1Port, 1, 65535);
        maxPacketsPerUpdate = Mathf.Max(1, maxPacketsPerUpdate);
        receiveBufferSizeBytes = Mathf.Max(8192, receiveBufferSizeBytes);
        _3MassKg = Mathf.Max(0.001f, _3MassKg);
        _4LinDrag = Mathf.Max(0f, _4LinDrag);
        _5AngDrag = Mathf.Max(0f, _5AngDrag);
        _6HThrustN = Mathf.Max(0f, _6HThrustN);
        _7VThrustN = Mathf.Max(0f, _7VThrustN);
        _11CurrentVel = new Vector3(
            float.IsFinite(_11CurrentVel.x) ? _11CurrentVel.x : 0f,
            float.IsFinite(_11CurrentVel.y) ? _11CurrentVel.y : 0f,
            float.IsFinite(_11CurrentVel.z) ? _11CurrentVel.z : 0f);
        waterCurrentLogIntervalSeconds = Mathf.Max(0f, waterCurrentLogIntervalSeconds);
        if (_8Thrusters == null)
            _8Thrusters = Array.Empty<SitlThrusterSpec>();

        _52HChannels = new[] { 1, 2, 3, 4 };
        _53VChannels = new[] { 5, 6 };
        _54HScales = new[] { 1f, 1f, 1f, 1f };
        _55VScales = CreateFilledArray(_53VChannels.Length, -1f);
        if (_8Thrusters == null || _8Thrusters.Length == 0)
            _8Thrusters = CreateDefaultVectoredThrusterSpecs();
        if (ignoredUnmappedServoChannels == null)
            ignoredUnmappedServoChannels = Array.Empty<int>();

        for (int i = 0; i < _52HChannels.Length; i++)
        {
            _52HChannels[i] = Mathf.Clamp(_52HChannels[i], 1, 32);
            _54HScales[i] = Mathf.Clamp(_54HScales[i], -2f, 2f);
        }
        for (int i = 0; i < _53VChannels.Length; i++)
        {
            _53VChannels[i] = Mathf.Clamp(_53VChannels[i], 1, 32);
            _55VScales[i] = Mathf.Clamp(_55VScales[i], -2f, 2f);
        }
        for (int i = 0; i < ignoredUnmappedServoChannels.Length; i++)
            ignoredUnmappedServoChannels[i] = Mathf.Clamp(ignoredUnmappedServoChannels[i], 1, 32);

        servoPwmMin = Mathf.Max(1f, servoPwmMin);
        servoPwmMax = Mathf.Max(servoPwmMin + 1f, servoPwmMax);
        servoPwmNeutral = Mathf.Clamp(servoPwmNeutral, servoPwmMin, servoPwmMax);
        servoPwmDeadzone = Mathf.Max(0f, servoPwmDeadzone);
        actuatorOutputTimeoutSeconds = Mathf.Max(0.05f, actuatorOutputTimeoutSeconds);
        invertHorizontalMotorOutputs = false;
        actuatorOutputSmoothing = Mathf.Clamp(actuatorOutputSmoothing, 0.05f, 1f);
        maxActuatorOutputSlewPerSecond = Mathf.Max(1f, maxActuatorOutputSlewPerSecond);
        _56ThrustScale = Mathf.Clamp(_56ThrustScale, 0f, 1f);
        horizontalSurgeScale = Mathf.Clamp(horizontalSurgeScale, -2f, 2f);
        horizontalSwayScale = Mathf.Clamp(horizontalSwayScale, -2f, 2f);
        if (Mathf.Approximately(horizontalYawScale, -0.2f))
            horizontalYawScale = 0.2f;
        horizontalYawScale = Mathf.Clamp(horizontalYawScale, -2f, 2f);
        motorOutputYawScale = Mathf.Clamp(motorOutputYawScale, 0f, 1f);
        verticalHeaveScale = Mathf.Clamp(verticalHeaveScale, 0f, 2f);
        verticalDifferentialScale = Mathf.Clamp(verticalDifferentialScale, 0f, 2f);
        applyThrusterForces = true;
        forceOutputDeadzone = Mathf.Max(0f, forceOutputDeadzone);
        forceLogIntervalSeconds = Mathf.Max(0f, forceLogIntervalSeconds);
        freezeVerticalState = false;
        freezeYawRate = false;
        reportYawRate = true;
        reportVerticalState = true;
        zeroReportedRollPitchAtStart = true;
        invertReportedHorizontalPosition = false;
        motorOutputYawScale = Mathf.Clamp01(motorOutputYawScale);
        verticalDifferentialScale = 1f;
        yawAngleSign = 1f;
        yawRateSign = 1f;
        rollAngleSign = -1f;
        forceLevelRollPitchReport = false;
        zeroRollPitchGyroWhenLevel = false;
        stabilizeEkfBeforeActuatorOutput = false;
        forceStableVerticalReport = false;
        maxReportedRollPitchDegrees = 0f;
        filterReportedAttitude = false;
        reportedAttitudeSmoothing = 1f;
        deriveReportedYawRateFromAttitude = true;
        maxReportedYawRateRadiansPerSecond = 2.5f;
        _59SurfaceY = float.IsFinite(_59SurfaceY) ? _59SurfaceY : 0f;
        _60QueryError = Mathf.Max(0.001f, _60QueryError);
        _61QueryIter = Mathf.Max(1, _61QueryIter);
        _13SubmergeM = Mathf.Max(0f, _13SubmergeM);
        _15VSpring = Mathf.Max(0f, _15VSpring);
        _16VDamping = Mathf.Max(0f, _16VDamping);
        _18TiltSpring = Mathf.Max(0f, _18TiltSpring);
        _19TiltDamping = Mathf.Max(0f, _19TiltDamping);
        _20MaxPointN = Mathf.Max(0f, _20MaxPointN);
        _22ClampBelowM = Mathf.Max(0f, _22ClampBelowM);
        _23ClampGuardM = Mathf.Max(0f, _23ClampGuardM);
        _30DiveThreshN = Mathf.Max(0f, _30DiveThreshN);
        _32LowPassS = Mathf.Max(0f, _32LowPassS);
        _34WindRefKmh = Mathf.Max(0.001f, _34WindRefKmh);
        _35MinWaveLenM = Mathf.Max(0.001f, _35MinWaveLenM);
        _36MaxWaveLenM = Mathf.Max(_35MinWaveLenM, _36MaxWaveLenM);
        _37MinMotionScale = Mathf.Max(0f, _37MinMotionScale);
        _38MaxMotionScale = Mathf.Max(_37MinMotionScale, _38MaxMotionScale);
        _39WaveLenM = Mathf.Max(0.001f, _39WaveLenM);
        _41MotionScale = Mathf.Max(0f, _41MotionScale);
        _42AccelScale = Mathf.Max(0f, _42AccelScale);
        _43MaxVForceN = Mathf.Max(0f, _43MaxVForceN);
        _40WavePeriodS = Mathf.Max(0.001f, _40WavePeriodS);
        _46PrimaryWeight = Mathf.Max(0f, _46PrimaryWeight);
        _47CrossWeight = Mathf.Max(0f, _47CrossWeight);
        _48ObliqueWeight = Mathf.Max(0f, _48ObliqueWeight);
        _49HMotionScale = Mathf.Max(0f, _49HMotionScale);
        _50HDampingN = Mathf.Max(0f, _50HDampingN);
        _51MaxHForceN = Mathf.Max(0f, _51MaxHForceN);
        _44MinWaveInfluence = Mathf.Clamp01(_44MinWaveInfluence);
        if (_21SamplePts == null)
            _21SamplePts = Array.Empty<Vector3>();
        deriveVelocityFromPosition = false;
        usePositionVelocityWhenRigidbodyIsStopped = true;
        deriveVerticalVelocityFromPosition = false;
        maxReportedHorizontalSpeed = Mathf.Max(0.1f, maxReportedHorizontalSpeed);
        maxReportedVerticalSpeed = Mathf.Max(2f, maxReportedVerticalSpeed);
        positionTinyValueEpsilon = 0f;
        rigidbodyVelocityDeadzone = 0f;
        verticalVelocityDeadzone = 0f;
        verticalVelocitySmoothing = Mathf.Clamp(verticalVelocitySmoothing, 0.01f, 1f);
        maxReportedHorizontalStepMeters = Mathf.Max(0.01f, maxReportedHorizontalStepMeters);
        reportedPositionSmoothing = Mathf.Clamp(reportedPositionSmoothing, 0.01f, 1f);
        gravityMetersPerSecondSquared = Mathf.Clamp(gravityMetersPerSecondSquared, 0f, 20f);
        includeLinearAccelerationInImu = true;
        maxReportedAccelerationMetersPerSecondSquared = Mathf.Max(3f, maxReportedAccelerationMetersPerSecondSquared);
        accelerationSmoothing = Mathf.Clamp01(accelerationSmoothing);
        servoLogIntervalSeconds = Mathf.Max(0f, servoLogIntervalSeconds);
        telemetryLogIntervalSeconds = Mathf.Max(0f, telemetryLogIntervalSeconds);
        minTelemetryLogIntervalSeconds = Mathf.Max(0.01f, minTelemetryLogIntervalSeconds);
        logEkfDiagnostics = true;
        warnEkfDiagnosticJumps = true;
        ekfDiagnosticLogIntervalSeconds = Mathf.Max(0.05f, ekfDiagnosticLogIntervalSeconds);
        ekfYawJumpWarningDegrees = Mathf.Max(0.1f, ekfYawJumpWarningDegrees);
        ekfPositionJumpWarningMeters = Mathf.Max(0.01f, ekfPositionJumpWarningMeters);
        ekfVelocityMismatchWarningMetersPerSecond = Mathf.Max(0.01f, ekfVelocityMismatchWarningMetersPerSecond);
        ekfGyroZWarningRadiansPerSecond = Mathf.Max(0.01f, ekfGyroZWarningRadiansPerSecond);
        ekfAccelWarningMetersPerSecondSquared = Mathf.Max(0.1f, ekfAccelWarningMetersPerSecondSquared);
        jsonTinyValueEpsilon = Mathf.Max(0f, jsonTinyValueEpsilon);
    }

    static int GetChannelIndex(int[] channels, int outputIndex)
    {
        if (channels == null || outputIndex < 0 || outputIndex >= channels.Length)
            return -1;

        return Mathf.Max(0, channels[outputIndex] - 1);
    }

    static int GetOneBasedChannel(int[] channels, int outputIndex)
    {
        if (channels == null || outputIndex < 0 || outputIndex >= channels.Length)
            return 1;

        return Mathf.Clamp(channels[outputIndex], 1, 32);
    }

    static float GetOutputScale(float[] scales, int outputIndex)
    {
        if (scales == null || outputIndex < 0 || outputIndex >= scales.Length)
            return 1f;

        return scales[outputIndex];
    }

    static float[] CreateFilledArray(int length, float value)
    {
        length = Mathf.Max(0, length);
        float[] values = new float[length];
        for (int i = 0; i < values.Length; i++)
            values[i] = value;
        return values;
    }

    static float NormalizeRadians(float radians)
    {
        while (radians > Mathf.PI) radians -= Mathf.PI * 2f;
        while (radians < -Mathf.PI) radians += Mathf.PI * 2f;
        return radians;
    }

    static void DisableUdpConnectionResetOnWindows(Socket socket)
    {
        if (socket == null)
            return;

        try
        {
            const int SIO_UDP_CONNRESET = -1744830452;
            socket.IOControl((IOControlCode)SIO_UDP_CONNRESET, new byte[] { 0, 0, 0, 0 }, null);
        }
        catch
        {
        }
    }
}
