using System;
using System.Globalization;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;

namespace RbulinX.SitlEngine
{
    /// <summary>
    /// Vehicle-agnostic ArduPilot SIM_JSON SITL bridge: UDP transport, JSON telemetry,
    /// coordinate frame conversion, PWM decoding, EKF diagnostics and FixedUpdate-driven
    /// timing. Vehicle-specific physics is supplied by an IVehicleForceModule.
    /// </summary>
    [DisallowMultipleComponent]
    [RequireComponent(typeof(Rigidbody))]
    public class SitlEngineCore : MonoBehaviour
    {
        public enum WorldFrameMapping
        {
            UnityForwardZRightXUpY,
            LegacyReferenceBoat
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
        public int port = 9002;
        public bool connectOnEnable = true;
        public int maxPacketsPerUpdate = 16;
        public int receiveBufferSizeBytes = 262144;

        [Header("Target")]
        public Transform telemetrySource;

        [Header("Vehicle Model")]
        public float vehicleMassKg = 10f;
        public float linearDragCoefficient = 35f;
        public float angularDragCoefficient = 5f;

        [Header("Vehicle Module")]
        [Tooltip("A component implementing IVehicleForceModule (e.g. RovWaterPhysicsModule).")]
        public MonoBehaviour vehicleModuleBehaviour;

        [Header("Actuator Input")]
        public float servoPwmMin = 1100f;
        public float servoPwmNeutral = 1500f;
        public float servoPwmMax = 1900f;
        public float servoPwmDeadzone = 25f;
        public float actuatorOutputTimeoutSeconds = 0.5f;
        public float forceOutputDeadzone = 0.01f;

        [Header("State Mapping")]
        public bool zeroUnityPositionAtStart = true;
        public WorldFrameMapping worldFrameMapping = WorldFrameMapping.UnityForwardZRightXUpY;
        public bool invertReportedHorizontalPosition = false;
        public bool freezeVerticalState = false;
        public bool freezeYawRate = false;
        public bool reportYawRate = true;
        public bool reportVerticalState = true;
        public bool zeroReportedRollPitchAtStart = true;
        [Range(-1f, 1f)] public float yawAngleSign = 1f;
        [Range(-1f, 1f)] public float yawRateSign = 1f;
        [Range(-1f, 1f)] public float rollAngleSign = -1f;
        [Range(-1f, 1f)] public float pitchAngleSign = 1f;
        public bool forceLevelRollPitchReport = false;
        public bool zeroRollPitchGyroWhenLevel = false;
        public bool stabilizeEkfBeforeActuatorOutput = false;
        public float maxReportedRollPitchDegrees = 0f;
        public Vector3 bodyEulerOffsetDegrees = Vector3.zero;
        public bool filterReportedAttitude = false;
        [Range(0.01f, 1f)] public float reportedAttitudeSmoothing = 1f;
        public bool deriveReportedYawRateFromAttitude = true;
        public float maxReportedYawRateRadiansPerSecond = 2.5f;
        public bool deriveVelocityFromPosition = false;
        public bool usePositionVelocityWhenRigidbodyIsStopped = true;
        public bool deriveVerticalVelocityFromPosition = false;
        public bool forceStableVerticalReport = false;
        public bool filterReportedPosition = false;
        public float positionTinyValueEpsilon = 0f;
        public float maxReportedHorizontalSpeed = 6.5f;
        public float maxReportedVerticalSpeed = 6f;
        public float rigidbodyVelocityDeadzone = 0f;
        public float verticalVelocityDeadzone = 0f;
        [Range(0.01f, 1f)] public float verticalVelocitySmoothing = 0.2f;
        public float maxReportedHorizontalStepMeters = 0.25f;
        [Range(0.01f, 1f)] public float reportedPositionSmoothing = 0.5f;
        public float gravityMetersPerSecondSquared = 9.80665f;
        public bool includeLinearAccelerationInImu = true;
        public float accelerationSmoothing = 0.35f;
        public float maxReportedAccelerationMetersPerSecondSquared = 3f;

        [Header("Logging")]
        public bool logConnection;
        public bool logServoPackets;
        public float servoLogIntervalSeconds = 0.25f;
        public bool logTelemetryPackets;
        public float telemetryLogIntervalSeconds = 1f;
        public float minTelemetryLogIntervalSeconds = 0.1f;
        public bool logEkfDiagnostics = true;
        public bool warnEkfDiagnosticJumps = true;
        public float ekfDiagnosticLogIntervalSeconds = 0.25f;
        public float ekfYawJumpWarningDegrees = 10f;
        public float ekfPositionJumpWarningMeters = 0.5f;
        public float ekfVelocityMismatchWarningMetersPerSecond = 1f;
        public float ekfGyroZWarningRadiansPerSecond = 1f;
        public float ekfAccelWarningMetersPerSecondSquared = 2f;
        public bool logJsonSendErrors;
        public float jsonTinyValueEpsilon = 1e-6f;

        const float TelemetrySendIntervalSeconds = 0.02f;
        const int MaxServoChannels = 32;

        UdpClient socketReceive;
        IPEndPoint remoteEndpoint = new IPEndPoint(IPAddress.Any, 0);
        bool hasRemoteConnection;
        Rigidbody rb;
        Rigidbody telemetryRb;
        Transform resolvedTelemetryTransform;
        IVehicleForceModule vehicleModule;
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
        bool hasSeenActiveActuatorOutput;
        float lastActuatorOutputTime = -999f;
        float lastServoLogTime = -999f;
        float lastTelemetryLogTime = -999f;
        float lastTelemetrySendTime = -999f;
        float lastEkfDiagnosticLogTime = -999f;
        bool hasLastTelemetryDiagnosticState;
        TelemetryState lastTelemetryDiagnosticState;
        readonly ushort[] servoOutputs = new ushort[MaxServoChannels];
        readonly float[] normalizedChannelOutputs = new float[MaxServoChannels];
        readonly JsonOutputPacket outputPacket = new JsonOutputPacket();

        public bool IsConnected => socketReceive != null;
        public float LastActuatorOutputAgeSeconds =>
            lastActuatorOutputTime > 0f ? Time.unscaledTime - lastActuatorOutputTime : float.PositiveInfinity;

        /// <summary>Normalized -1..1 output for a one-based servo channel (1-32).</summary>
        public float GetChannelOutput(int oneBasedChannel)
        {
            int index = oneBasedChannel - 1;
            return index >= 0 && index < normalizedChannelOutputs.Length ? normalizedChannelOutputs[index] : 0f;
        }

        /// <summary>Raw PWM microseconds for a one-based servo channel (1-32), 0 if unset.</summary>
        public ushort GetRawServoPwm(int oneBasedChannel)
        {
            int index = oneBasedChannel - 1;
            return index >= 0 && index < servoOutputs.Length ? servoOutputs[index] : (ushort)0;
        }

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
            vehicleModule = vehicleModuleBehaviour as IVehicleForceModule;
            if (vehicleModuleBehaviour != null && vehicleModule == null)
                Debug.LogWarning("[SitlEngineCore] vehicleModuleBehaviour does not implement IVehicleForceModule.");

            vehicleModule?.OnSitlEnable(this, rb, transform);

            unityOriginPosition = GetTelemetryPosition();
            if (vehicleModule != null && vehicleModule.TryGetVerticalOriginY(unityOriginPosition, out float originY))
                unityOriginPosition.y = originY;

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
            lastTelemetryLogTime = -999f;
            lastTelemetrySendTime = -999f;
            lastEkfDiagnosticLogTime = -999f;
            hasFilteredPosition = false;
            hasFilteredAttitude = false;
            hasInitialReportedRollPitch = false;
            hasPreviousVelocityNed = false;
            hasPreviousVerticalPosition = false;
            hasSlewLimitedDepth = false;
            hasSeenActiveActuatorOutput = false;
            hasLastTelemetryDiagnosticState = false;
            lastTelemetryDiagnosticState = default;

            CaptureInitialReportedRollPitch();

            if (connectOnEnable)
                StartBridge();
        }

        void OnDisable()
        {
            StopBridge();
            vehicleModule?.OnSitlDisable();
        }

        void Update()
        {
            ReceiveServoPackets();
            SendTelemetryIfDue();
        }

        void FixedUpdate()
        {
            ApplyVehicleModelToRigidbody();

            float outputAge = Time.unscaledTime - lastActuatorOutputTime;
            float timeout = Mathf.Max(0.05f, actuatorOutputTimeoutSeconds);
            bool actuatorOutputIsFresh = outputAge <= timeout;

            vehicleModule?.ApplyForces(this, rb, transform, Time.fixedDeltaTime, actuatorOutputIsFresh);

            SendTelemetryIfDue();
        }

        void ApplyVehicleModelToRigidbody()
        {
            if (rb == null)
                return;

            rb.mass = Mathf.Max(0.001f, vehicleMassKg);
#if UNITY_6000_0_OR_NEWER
            rb.linearDamping = Mathf.Max(0f, linearDragCoefficient);
#else
            rb.drag = Mathf.Max(0f, linearDragCoefficient);
#endif
            rb.angularDamping = Mathf.Max(0f, angularDragCoefficient);
        }

        public void StartBridge()
        {
            if (socketReceive != null)
                return;

            try
            {
                socketReceive = new UdpClient(Mathf.Clamp(port, 1, 65535));
                socketReceive.Client.ReceiveBufferSize = Mathf.Max(8192, receiveBufferSizeBytes);
                socketReceive.Client.Blocking = false;
                DisableUdpConnectionResetOnWindows(socketReceive.Client);
            }
            catch (Exception e)
            {
                StopBridge();
                Debug.LogWarning($"[SitlEngineCore] Failed to bind UDP {port}: {e.Message}");
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
                            Debug.Log($"[SitlEngineCore] Received new connection from SITL: {remoteEndpoint}");
                    }

                    if (TryParseServoPacket(data, out ushort frameRate, out uint frameCount, out int servoCount))
                    {
                        UpdateNormalizedChannelOutputs();
                        lastActuatorOutputTime = Time.unscaledTime;
                        if (HasActiveChannelOutput())
                            hasSeenActiveActuatorOutput = true;

                        if (ShouldLogServoPacket())
                            Debug.Log($"[SitlEngineCore] frame={frameCount} rate={frameRate} servos={servoCount} active=[{FormatActiveServoOutputs(servoCount)}]");
                    }

                    SendTelemetryIfDue();
                }
            }
            catch (SocketException e)
            {
                if (e.SocketErrorCode != SocketError.WouldBlock && e.SocketErrorCode != SocketError.ConnectionReset)
                    Debug.LogWarning($"[SitlEngineCore] UDP receive failed: {e.Message}");
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

        void UpdateNormalizedChannelOutputs()
        {
            for (int i = 0; i < servoOutputs.Length; i++)
                normalizedChannelOutputs[i] = NormalizeServoPwm(servoOutputs[i]);
        }

        bool HasActiveChannelOutput()
        {
            float threshold = Mathf.Max(0f, forceOutputDeadzone);
            for (int i = 0; i < normalizedChannelOutputs.Length; i++)
            {
                if (Mathf.Abs(normalizedChannelOutputs[i]) > threshold)
                    return true;
            }

            return false;
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

        void SendTelemetry()
        {
            if (!hasRemoteConnection || socketReceive == null)
                return;

            UpdateOutputPacket();
            string json = BuildTelemetryJson();
            if (ShouldLogTelemetryPacket())
                Debug.Log($"[SitlEngineCore] TX {json}");

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
                    Debug.LogWarning($"[SitlEngineCore] UDP send failed: {e.Message}");
            }
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

        string FormatActiveServoOutputs(int servoCount)
        {
            int count = Mathf.Clamp(servoCount, 0, servoOutputs.Length);
            StringBuilder builder = new StringBuilder();
            for (int i = 0; i < count; i++)
            {
                ushort pwm = servoOutputs[i];
                if (pwm == 0 || Mathf.Abs(pwm - servoPwmNeutral) <= Mathf.Max(0f, servoPwmDeadzone))
                    continue;

                if (builder.Length > 0)
                    builder.Append(", ");
                builder.Append(i + 1);
                builder.Append(":");
                builder.Append(pwm);
            }

            return builder.ToString();
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
                Debug.LogWarning(FormatEkfDiagnosticMessage("[SitlEngineCore EKF WARN]", state, yawJumpDeg, positionJump, velocityMismatch, gyroZ, accel));
                lastEkfDiagnosticLogTime = Time.unscaledTime;
                return;
            }

            if (!logEkfDiagnostics)
                return;

            float interval = Mathf.Max(0f, ekfDiagnosticLogIntervalSeconds);
            if (interval > 0f && Time.unscaledTime - lastEkfDiagnosticLogTime < interval)
                return;

            Debug.Log(FormatEkfDiagnosticMessage("[SitlEngineCore EKF]", state, yawJumpDeg, positionJump, velocityMismatch, gyroZ, accel));
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
            return stabilizeEkfBeforeActuatorOutput && !hasSeenActiveActuatorOutput;
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
            resolvedTelemetryTransform = telemetrySource != null
                ? telemetrySource
                : transform;

            telemetryRb = resolvedTelemetryTransform != null
                ? resolvedTelemetryTransform.GetComponentInParent<Rigidbody>()
                : null;
            if (telemetryRb == null)
                telemetryRb = rb;
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

        void EnsureInspectorDefaults()
        {
            port = Mathf.Clamp(port, 1, 65535);
            maxPacketsPerUpdate = Mathf.Max(1, maxPacketsPerUpdate);
            receiveBufferSizeBytes = Mathf.Max(8192, receiveBufferSizeBytes);
            vehicleMassKg = Mathf.Max(0.001f, vehicleMassKg);
            linearDragCoefficient = Mathf.Max(0f, linearDragCoefficient);
            angularDragCoefficient = Mathf.Max(0f, angularDragCoefficient);

            servoPwmMin = Mathf.Max(1f, servoPwmMin);
            servoPwmMax = Mathf.Max(servoPwmMin + 1f, servoPwmMax);
            servoPwmNeutral = Mathf.Clamp(servoPwmNeutral, servoPwmMin, servoPwmMax);
            servoPwmDeadzone = Mathf.Max(0f, servoPwmDeadzone);
            actuatorOutputTimeoutSeconds = Mathf.Max(0.05f, actuatorOutputTimeoutSeconds);
            forceOutputDeadzone = Mathf.Max(0f, forceOutputDeadzone);

            maxReportedRollPitchDegrees = Mathf.Max(0f, maxReportedRollPitchDegrees);
            reportedAttitudeSmoothing = Mathf.Clamp(reportedAttitudeSmoothing, 0.01f, 1f);
            maxReportedYawRateRadiansPerSecond = Mathf.Max(0f, maxReportedYawRateRadiansPerSecond);
            maxReportedHorizontalSpeed = Mathf.Max(0.1f, maxReportedHorizontalSpeed);
            maxReportedVerticalSpeed = Mathf.Max(0.1f, maxReportedVerticalSpeed);
            rigidbodyVelocityDeadzone = Mathf.Max(0f, rigidbodyVelocityDeadzone);
            verticalVelocityDeadzone = Mathf.Max(0f, verticalVelocityDeadzone);
            verticalVelocitySmoothing = Mathf.Clamp(verticalVelocitySmoothing, 0.01f, 1f);
            maxReportedHorizontalStepMeters = Mathf.Max(0.01f, maxReportedHorizontalStepMeters);
            reportedPositionSmoothing = Mathf.Clamp(reportedPositionSmoothing, 0.01f, 1f);
            gravityMetersPerSecondSquared = Mathf.Clamp(gravityMetersPerSecondSquared, 0f, 20f);
            maxReportedAccelerationMetersPerSecondSquared = Mathf.Max(0.1f, maxReportedAccelerationMetersPerSecondSquared);
            accelerationSmoothing = Mathf.Clamp01(accelerationSmoothing);
            positionTinyValueEpsilon = Mathf.Max(0f, positionTinyValueEpsilon);

            servoLogIntervalSeconds = Mathf.Max(0f, servoLogIntervalSeconds);
            telemetryLogIntervalSeconds = Mathf.Max(0f, telemetryLogIntervalSeconds);
            minTelemetryLogIntervalSeconds = Mathf.Max(0.01f, minTelemetryLogIntervalSeconds);
            ekfDiagnosticLogIntervalSeconds = Mathf.Max(0.05f, ekfDiagnosticLogIntervalSeconds);
            ekfYawJumpWarningDegrees = Mathf.Max(0.1f, ekfYawJumpWarningDegrees);
            ekfPositionJumpWarningMeters = Mathf.Max(0.01f, ekfPositionJumpWarningMeters);
            ekfVelocityMismatchWarningMetersPerSecond = Mathf.Max(0.01f, ekfVelocityMismatchWarningMetersPerSecond);
            ekfGyroZWarningRadiansPerSecond = Mathf.Max(0.01f, ekfGyroZWarningRadiansPerSecond);
            ekfAccelWarningMetersPerSecondSquared = Mathf.Max(0.1f, ekfAccelWarningMetersPerSecondSquared);
            jsonTinyValueEpsilon = Mathf.Max(0f, jsonTinyValueEpsilon);
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
}
