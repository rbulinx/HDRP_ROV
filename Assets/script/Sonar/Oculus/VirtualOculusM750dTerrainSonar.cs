using System;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Collections.Generic;
using UnityEngine;

namespace Robalink.OculusEmulator
{
    [DisallowMultipleComponent]
    public class VirtualOculusM750dTerrainSonar : MonoBehaviour
    {
        [Header("Oculus identity")]
        public ushort deviceId = 17;
        public uint deviceId32 = 17;
        public OculusPartNumberType partNumber = OculusPartNumberType.M750d;
        public string advertisedIp = "auto";
        public bool preferPrivateLanAddress = true;
        public string subnetMask = "255.255.255.0";
        public bool dhcpEnabled = false;

        [Header("Ports")]
        public int tcpPort = OculusProtocol.DataPort;
        public int udpStatusPort = OculusProtocol.StatusPort;
        public string broadcastAddress = "255.255.255.255";
        public float statusBroadcastHz = 1f;

        [Header("Sonar mode")]
        public OculusFrequencyMode frequencyMode = OculusFrequencyMode.High;
        public PingRateType pingRate = PingRateType.Normal;
        public double highFrequencyHz = 1200000.0;
        public double lowFrequencyHz = 750000.0;
        public float highFrequencyHorizontalFovDeg = 80f;
        public float lowFrequencyHorizontalFovDeg = 130f;
        public float highFrequencyVerticalFovDeg = 24f;
        public float lowFrequencyVerticalFovDeg = 36f;
        public float highFrequencyMaxRangeMeters = 40f;
        public float lowFrequencyMaxRangeMeters = 120f;
        public int beamCount = 256;
        public int rangeCount = 1024;
        [Range(0, 255)] public byte gammaCorrection = 127;
        public byte flags = 0;
        [Tooltip("0 means use the range requested by the Viewer. Set > 0 to force a test range in meters.")]
        public float overrideRequestedRangeMeters = 0f;

        [Header("Scene sensing")]
        public LayerMask targetLayers = ~0;
        public string ignoreTag = "ROV";
        public float minRangeMeters = 0.3f;
        [Range(1, 64)] public int verticalSamplesPerBeam = 12;
        [Range(0f, 1f)] public float backgroundLevel = 0.005f;
        [Range(0f, 1f)] public float maxEchoLevel = 1.0f;
        public float noiseSigma = 0f;
        [Header("Water-column false echoes")]
        public bool enableWaterColumnNoise = true;
        public SonarWaterColumnNoise waterColumnNoise = new SonarWaterColumnNoise();

        [Header("Scene echo response")]
        public float incidenceWeight = 0.35f;
        public float incidenceExponent = 2.0f;
        public float attenuationAlpha = 0.03f;
        public float echoWidthMeters = 0.08f;
        public float echoSigmaMeters = 0.03f;
        [Range(1, 16)] public int maxEchoesPerBeam = 3;
        public bool firstReturnOnly = false;
        public float minEchoSeparationMeters = 0.12f;
        public float repeatedHitBoost = 0.18f;
        public bool useSphereCast = true;
        public float sphereCastRadius = 0.08f;
        public QueryTriggerInteraction triggerInteraction = QueryTriggerInteraction.Collide;

        [Header("Frame defaults")]
        public double defaultTemperatureDegC = 15.0;
        public double defaultSpeedOfSoundMps = 1500.0;
        public double defaultGainPercent = 50.0;
        public double defaultRangePercent = 1.0;
        public double defaultSalinityPpt = 35.0;

        [Header("ViewPoint attitude")]
        public bool zeroRollInViewPoint = true;
        public bool zeroPitchInViewPoint = false;

        [Header("Connection recovery")]
        [Tooltip("Enable TCP keepalive so dead ViewPoint sessions are detected more reliably.")]
        public bool enableTcpKeepAlive = true;

        [Header("Debug")]
        public bool autoRespondToFire = true;
        public bool drawBeamRays = false;
        public Color hitRayColor = Color.cyan;
        public Color missRayColor = Color.gray;
        public float debugRayDuration = 0.12f;
        public bool debugRayDepthTest = false;
        public bool drawBeamRayObjects = false;
        public bool drawBeamRaysWhenIdle = false;
        public Material debugRayMaterial;
        [Range(1, 64)] public int debugRayBeamStride = 8;
        [Range(1, 64)] public int debugRayVerticalSampleStride = 4;
        [Range(0.001f, 0.2f)] public float debugRayObjectWidth = 0.02f;
        [Range(0.5f, 20f)] public float idleDebugRayHz = 10f;

        UdpClient statusUdp;
        TcpListener listener;
        Thread networkThread;
        volatile bool running;
        readonly object clientLock = new object();
        TcpClient client;
        NetworkStream clientStream;
        IPEndPoint clientEndPoint;
        float nextStatusTime;
        System.Random rng;
        double startTime;
        uint msgId = 1;
        uint pingId = 1;
        int sonarFrameIndex;
        float requestedRangeMeters = -1f;
        readonly object requestLock = new object();
        bool pendingFire;
        SimpleFireRequest pendingFireRequest;
        bool fireSessionActive;
        float nextPingTime;
        float nextIdleDebugRayTime;
        Transform debugRayRoot;
        Material runtimeDebugRayMaterial;

        struct BeamEcho
        {
            public float Distance;
            public float Score;
            public float AggregateScore;
            public int SampleCount;
            public RaycastHit Hit;
        }

        void OnEnable()
        {
            if (rng == null) rng = new System.Random(12345);
            startTime = Time.realtimeSinceStartupAsDouble;
            if (statusUdp == null) StartStatusBroadcaster();
            if (!running) StartTcpServer();
        }

        void OnDisable()
        {
            StopTcpServer();
            StopStatusBroadcaster();
        }

        void Update()
        {
            CloseStaleClientIfNeeded();

            if (Time.time >= nextStatusTime)
            {
                nextStatusTime = Time.time + 1f / Mathf.Max(0.1f, statusBroadcastHz);
                BroadcastStatus();
            }

            DrawIdleDebugRaysIfNeeded();

            if (!autoRespondToFire) return;

            bool hasPending = false;
            lock (requestLock)
            {
                if (pendingFire)
                {
                    hasPending = true;
                    pendingFire = false;
                }
            }

            if (hasPending)
            {
                ApplyFireRequest(pendingFireRequest);
                fireSessionActive = pingRate != PingRateType.Standby;
                nextPingTime = Time.time;
            }

            if (!fireSessionActive) return;
            if (Time.time < nextPingTime) return;

            SendPingFrame();
            nextPingTime = Time.time + GetPingIntervalSeconds(pingRate);
        }

        [ContextMenu("Send Test Ping")]
        public void SendTestPing()
        {
            SendPingFrame();
        }

        void StartStatusBroadcaster()
        {
            statusUdp = new UdpClient();
            statusUdp.EnableBroadcast = true;
            nextStatusTime = Time.time + 0.25f;
        }

        void StopStatusBroadcaster()
        {
            statusUdp?.Dispose();
            statusUdp = null;
        }

        void BroadcastStatus()
        {
            try
            {
                if (statusUdp == null) return;
                IPAddress ip = ResolveAdvertisedIp();
                IPAddress mask = ParseIpv4OrLoopback(subnetMask);
                IPAddress connectedIp = clientEndPoint?.Address ?? IPAddress.Parse("0.0.0.0");
                byte[] packet = OculusProtocol.BuildStatusPacket(deviceId32, deviceId, partNumber, ip, mask, connectedIp);
                statusUdp.Send(packet, packet.Length, new IPEndPoint(ParseIpv4OrLoopback(broadcastAddress), udpStatusPort));
            }
            catch (Exception)
            {
            }
        }

        void StartTcpServer()
        {
            running = true;
            listener = new TcpListener(IPAddress.Any, tcpPort);
            listener.Start();
            networkThread = new Thread(NetworkLoop)
            {
                IsBackground = true,
                Name = "VirtualOculusTcpServer"
            };
            networkThread.Start();
        }

        void StopTcpServer()
        {
            running = false;
            try { listener?.Stop(); } catch { }
            lock (clientLock)
            {
                CloseClientConnection();
            }

            if (networkThread != null && networkThread.IsAlive)
            {
                networkThread.Join(500);
            }

            listener = null;
            networkThread = null;
        }

        void NetworkLoop()
        {
            while (running)
            {
                TcpClient accepted = null;
                try
                {
                    accepted = listener.AcceptTcpClient();
                    accepted.NoDelay = true;
                    accepted.ReceiveTimeout = 200;
                    accepted.SendTimeout = 1000;

                    lock (clientLock)
                    {
                        CloseClientConnection();
                        client = accepted;
                        clientStream = accepted.GetStream();
                        clientEndPoint = accepted.Client.RemoteEndPoint as IPEndPoint;
                        if (enableTcpKeepAlive)
                            accepted.Client.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.KeepAlive, true);
                    }

                    ReadClientLoop(accepted);
                }
                catch (SocketException)
                {
                    if (!running) break;
                }
                catch (Exception)
                {
                }
                finally
                {
                    lock (clientLock)
                    {
                        if (client == accepted)
                        {
                            CloseClientConnection();
                        }
                    }
                }
            }
        }

        void ReadClientLoop(TcpClient activeClient)
        {
            NetworkStream stream = activeClient.GetStream();
            byte[] buffer = new byte[2048];
            List<byte> pendingBytes = new List<byte>(4096);

            while (running && activeClient.Connected)
            {
                try
                {
                    if (!stream.DataAvailable)
                    {
                        Thread.Sleep(10);
                        continue;
                    }

                    int count = stream.Read(buffer, 0, buffer.Length);
                    if (count <= 0) break;

                    for (int i = 0; i < count; i++)
                    {
                        pendingBytes.Add(buffer[i]);
                    }

                    ProcessPendingMessages(pendingBytes);
                }
                catch (IOException)
                {
                    break;
                }
                catch (Exception)
                {
                    break;
                }
            }
        }

        void ProcessPendingMessages(List<byte> pendingBytes)
        {
            while (pendingBytes.Count >= OculusProtocol.MessageHeaderSize)
            {
                byte[] headerBytes = pendingBytes.GetRange(0, OculusProtocol.MessageHeaderSize).ToArray();
                if (!OculusProtocol.TryReadHeader(headerBytes, 0, out OculusMessageHeaderFields header))
                {
                    pendingBytes.RemoveAt(0);
                    continue;
                }

                int messageLength = OculusProtocol.MessageHeaderSize + checked((int)header.payloadSize);
                if (messageLength <= 0)
                {
                    pendingBytes.RemoveRange(0, OculusProtocol.MessageHeaderSize);
                    continue;
                }

                if (pendingBytes.Count < messageLength)
                {
                    return;
                }

                byte[] msg = pendingBytes.GetRange(0, messageLength).ToArray();
                pendingBytes.RemoveRange(0, messageLength);
                HandleClientMessage(msg);
            }
        }

        void HandleClientMessage(byte[] msg)
        {
            if (OculusProtocol.TryParseSimpleFireRequest(msg, out SimpleFireRequest fireRequest))
            {
                lock (requestLock)
                {
                    pendingFireRequest = fireRequest;
                    pendingFire = true;
                }
                return;
            }

            if (OculusProtocol.IsUserConfigRequest(msg))
            {
                IPAddress ip = ResolveAdvertisedIp();
                IPAddress mask = ParseIpv4OrLoopback(subnetMask);
                byte[] packet = OculusProtocol.BuildUserConfigPacket(
                    deviceId,
                    0,
                    OculusProtocol.ToUInt32LE(ip),
                    OculusProtocol.ToUInt32LE(mask),
                    dhcpEnabled);
                SendToClient(packet);
                return;
            }
        }

        void SendToClient(byte[] data)
        {
            if (data == null || data.Length == 0) return;

            lock (clientLock)
            {
                try
                {
                    if (clientStream != null && client != null && client.Connected)
                    {
                        clientStream.Write(data, 0, data.Length);
                        clientStream.Flush();
                    }
                }
                catch (Exception)
                {
                    CloseClientConnection();
                }
            }
        }

        void CloseClientConnection()
        {
            try { clientStream?.Close(); } catch { }
            try { client?.Close(); } catch { }
            clientStream = null;
            client = null;
            clientEndPoint = null;
        }

        void ApplyFireRequest(SimpleFireRequest request)
        {
            frequencyMode = request.MasterMode == OculusFrequencyMode.Low ? OculusFrequencyMode.Low : OculusFrequencyMode.High;
            pingRate = request.PingRate;
            gammaCorrection = request.GammaCorrection;
            flags = request.Flags;

            if (request.SpeedOfSound > 1000.0 && request.SpeedOfSound < 2000.0)
            {
                defaultSpeedOfSoundMps = request.SpeedOfSound;
            }

            if (request.GainPercent >= 0.0)
            {
                defaultGainPercent = request.GainPercent;
            }

            if (request.RangePercentOrMeters > 0.0)
            {
                float frequencyMaxRange = frequencyMode == OculusFrequencyMode.High ? highFrequencyMaxRangeMeters : lowFrequencyMaxRangeMeters;
                float requestedRangePercent = Mathf.Clamp((float)request.RangePercentOrMeters, 0f, 100f);
                float requestedMeters = frequencyMaxRange * (requestedRangePercent / 100f);
                float effectiveRequestedMeters = overrideRequestedRangeMeters > 0f ? overrideRequestedRangeMeters : requestedMeters;
                requestedRangeMeters = Mathf.Clamp(effectiveRequestedMeters, minRangeMeters, frequencyMaxRange);
                defaultRangePercent = requestedRangeMeters / Mathf.Max(0.001f, frequencyMaxRange) * 100.0;
            }

            if (request.Salinity >= 0.0)
            {
                defaultSalinityPpt = request.Salinity;
            }

        }

        void SendPingFrame()
        {
            ushort dstDeviceId = pendingFireRequest.Header.srcDeviceId;
            VirtualSonarPingFrame frame = BuildFrameFromScene();
            byte[] packet = OculusProtocol.BuildSimplePingResult2Packet(
                frame,
                deviceId,
                dstDeviceId,
                msgId++,
                frequencyMode,
                pingRate,
                gammaCorrection,
                flags,
                pingId++,
                CurrentFrequencyHz,
                1.0,
                DataSizeType.Data8Bit);
            SendToClient(packet);
        }

        static float GetPingIntervalSeconds(PingRateType rate)
        {
            switch (rate)
            {
                case PingRateType.Highest:
                    return 1f / 40f;
                case PingRateType.High:
                    return 1f / 15f;
                case PingRateType.Low:
                    return 1f / 5f;
                case PingRateType.Lowest:
                    return 1f / 2f;
                case PingRateType.Standby:
                    return float.PositiveInfinity;
                case PingRateType.Normal:
                default:
                    return 1f / 10f;
            }
        }

        IPAddress ResolveAdvertisedIp()
        {
            if (!string.Equals(advertisedIp, "auto", StringComparison.OrdinalIgnoreCase) &&
                IPAddress.TryParse(advertisedIp, out IPAddress configured))
            {
                return configured;
            }

            try
            {
                List<IPAddress> candidates = new List<IPAddress>();
                IPAddress[] addressList = Dns.GetHostEntry(Dns.GetHostName()).AddressList;
                for (int i = 0; i < addressList.Length; i++)
                {
                    IPAddress ip = addressList[i];
                    if (ip.AddressFamily != AddressFamily.InterNetwork) continue;
                    if (IPAddress.IsLoopback(ip)) continue;
                    if (candidates.Contains(ip)) continue;
                    candidates.Add(ip);
                }

                if (candidates.Count == 0)
                {
                    return IPAddress.Loopback;
                }

                IPAddress selected = candidates[0];

                if (preferPrivateLanAddress)
                {
                    IPAddress privateCandidate = null;
                    for (int i = 0; i < candidates.Count; i++)
                    {
                        if (!IsPrivateLanIpv4(candidates[i])) continue;
                        privateCandidate = candidates[i];
                        break;
                    }
                    if (privateCandidate != null)
                    {
                        selected = privateCandidate;
                    }
                    else
                    {
                        IPAddress nonCgnatCandidate = null;
                        for (int i = 0; i < candidates.Count; i++)
                        {
                            if (IsCarrierGradeNatIpv4(candidates[i])) continue;
                            nonCgnatCandidate = candidates[i];
                            break;
                        }
                        if (nonCgnatCandidate != null)
                        {
                            selected = nonCgnatCandidate;
                        }
                    }
                }

                return selected;
            }
            catch
            {
            }

            return IPAddress.Loopback;
        }

        static IPAddress ParseIpv4OrLoopback(string value)
        {
            return IPAddress.TryParse(value, out IPAddress parsed) ? parsed : IPAddress.Loopback;
        }

        static bool IsPrivateLanIpv4(IPAddress ip)
        {
            byte[] bytes = ip.GetAddressBytes();
            if (bytes.Length != 4) return false;

            if (bytes[0] == 10) return true;
            if (bytes[0] == 172 && bytes[1] >= 16 && bytes[1] <= 31) return true;
            if (bytes[0] == 192 && bytes[1] == 168) return true;
            return false;
        }

        static bool IsCarrierGradeNatIpv4(IPAddress ip)
        {
            byte[] bytes = ip.GetAddressBytes();
            if (bytes.Length != 4) return false;
            return bytes[0] == 100 && bytes[1] >= 64 && bytes[1] <= 127;
        }

        double CurrentFrequencyHz => frequencyMode == OculusFrequencyMode.High ? highFrequencyHz : lowFrequencyHz;
        float CurrentHorizontalFovDeg => frequencyMode == OculusFrequencyMode.High ? highFrequencyHorizontalFovDeg : lowFrequencyHorizontalFovDeg;
        float CurrentVerticalFovDeg => frequencyMode == OculusFrequencyMode.High ? highFrequencyVerticalFovDeg : lowFrequencyVerticalFovDeg;
        float CurrentMaxRangeMeters
        {
            get
            {
                float modeMax = frequencyMode == OculusFrequencyMode.High ? highFrequencyMaxRangeMeters : lowFrequencyMaxRangeMeters;
                return requestedRangeMeters > 0f ? Mathf.Min(requestedRangeMeters, modeMax) : modeMax;
            }
        }

        VirtualSonarPingFrame CreateEmptyFrame()
        {
            float maxRange = CurrentMaxRangeMeters;
            float fovDeg = CurrentHorizontalFovDeg;
            float headingDeg = transform.eulerAngles.y;
            float pitchDeg = NormalizeSignedAngle(transform.eulerAngles.x);
            float rollDeg = NormalizeSignedAngle(transform.eulerAngles.z);

            if (zeroPitchInViewPoint) pitchDeg = 0f;
            if (zeroRollInViewPoint) rollDeg = 0f;

            var frame = new VirtualSonarPingFrame
            {
                PingStartTimeSec = Time.realtimeSinceStartupAsDouble - startTime,
                HeadingDeg = headingDeg,
                PitchDeg = pitchDeg,
                RollDeg = rollDeg,
                TemperatureDegC = defaultTemperatureDegC,
                SpeedOfSoundMps = defaultSpeedOfSoundMps,
                GainPercent = defaultGainPercent,
                RangePercent = defaultRangePercent,
                SalinityPpt = defaultSalinityPpt,
                RangeResolutionMeters = maxRange / Mathf.Max(1, rangeCount),
                AzimuthsRad = new double[beamCount],
                Intensities8 = new byte[rangeCount, beamCount],
            };

            double halfFov = fovDeg * 0.5 * Math.PI / 180.0;
            for (int beamIndex = 0; beamIndex < beamCount; beamIndex++)
            {
                double t = beamCount == 1 ? 0.5 : (double)beamIndex / (beamCount - 1);
                frame.AzimuthsRad[beamIndex] = -halfFov + t * 2.0 * halfFov;
            }

            return frame;
        }

        public VirtualSonarPingFrame BuildFrameFromScene()
        {
            int frameIndex = sonarFrameIndex++;
            VirtualSonarPingFrame frame = CreateEmptyFrame();
            FillBackground(frame);

            float verticalHalfDeg = CurrentVerticalFovDeg * 0.5f;
            float maxRange = CurrentMaxRangeMeters;
            float rangeResolution = (float)frame.RangeResolutionMeters;
            Quaternion sensingRotation = GetViewPointRotation();

            for (int beamIndex = 0; beamIndex < beamCount; beamIndex++)
            {
                float azimuthDeg = (float)(frame.AzimuthsRad[beamIndex] * 180.0 / Math.PI);
                List<BeamEcho> beamEchoes = new List<BeamEcho>();
                int samples = Mathf.Max(1, verticalSamplesPerBeam);

                for (int sampleIndex = 0; sampleIndex < samples; sampleIndex++)
                {
                    float t = samples == 1 ? 0.5f : (float)sampleIndex / (samples - 1);
                    float elevationDeg = Mathf.Lerp(-verticalHalfDeg, verticalHalfDeg, t);
                    Vector3 localDirection = Quaternion.Euler(elevationDeg, azimuthDeg, 0f) * Vector3.forward;
                    Vector3 direction = sensingRotation * localDirection;

                    if (TrySceneHits(direction, maxRange, out RaycastHit[] hits))
                    {
                        for (int hitIndex = 0; hitIndex < hits.Length; hitIndex++)
                        {
                            RaycastHit hit = hits[hitIndex];
                            float score = ComputeHitScore(hit, direction, maxRange);
                            AddBeamEchoCandidate(beamEchoes, hit, score);

                            if (drawBeamRays)
                            {
                                Debug.DrawRay(transform.position, direction * hit.distance, hitRayColor, debugRayDuration, debugRayDepthTest);
                            }

                            if (ShouldDrawBeamRayObject(beamIndex, sampleIndex))
                            {
                                DrawBeamRayObject(transform.position, transform.position + direction * hit.distance, hitRayColor);
                            }
                        }
                    }
                    else
                    {
                        if (drawBeamRays)
                        {
                            Debug.DrawRay(transform.position, direction * maxRange, missRayColor, debugRayDuration, debugRayDepthTest);
                        }

                        if (ShouldDrawBeamRayObject(beamIndex, sampleIndex))
                        {
                            DrawBeamRayObject(transform.position, transform.position + direction * maxRange, missRayColor);
                        }
                    }
                }

                if (beamEchoes.Count > 0)
                {
                    beamEchoes.Sort((a, b) => a.Distance.CompareTo(b.Distance));

                    for (int echoIndex = 0; echoIndex < beamEchoes.Count; echoIndex++)
                    {
                        BeamEcho echo = beamEchoes[echoIndex];
                        WriteEcho(frame, beamIndex, echo, rangeResolution);
                    }
                }

                float nearestSolidDistance = beamEchoes.Count > 0 ? beamEchoes[0].Distance : float.PositiveInfinity;
                if (enableWaterColumnNoise && waterColumnNoise != null && waterColumnNoise.enabled)
                    waterColumnNoise.ApplyToRangeAzimuthFrame(frame, beamIndex, maxRange, nearestSolidDistance, frameIndex);

                float centerElevationDeg = 0f;
                Vector3 centerLocalDirection = Quaternion.Euler(centerElevationDeg, azimuthDeg, 0f) * Vector3.forward;
                Vector3 centerDirection = sensingRotation * centerLocalDirection;
                UnderwaterWorksiteDebrisField.ApplyToRangeAzimuthFrameForAll(
                    frame,
                    beamIndex,
                    transform.position,
                    centerDirection,
                    maxRange,
                    nearestSolidDistance,
                    frameIndex);
            }

            if (noiseSigma > 0f)
            {
                AddNoise(frame);
            }

            return frame;
        }

        public void SetWaterColumnNoiseEnabled(bool value)
        {
            enableWaterColumnNoise = value;
            if (waterColumnNoise != null) waterColumnNoise.enabled = value;
        }

        public void ToggleWaterColumnNoise()
        {
            SetWaterColumnNoiseEnabled(!enableWaterColumnNoise);
        }

        void CloseStaleClientIfNeeded()
        {
            lock (clientLock)
            {
                if (client == null) return;
                if (!SocketLooksDisconnected(client)) return;

                Debug.Log("[VirtualOculus] Closing stale client session to allow reconnection.");
                CloseClientConnection();
                fireSessionActive = false;
            }
        }

        static bool SocketLooksDisconnected(TcpClient tcpClient)
        {
            try
            {
                if (tcpClient == null || tcpClient.Client == null) return true;
                Socket socket = tcpClient.Client;
                return socket.Poll(0, SelectMode.SelectRead) && socket.Available == 0;
            }
            catch
            {
                return true;
            }
        }

        Quaternion GetViewPointRotation()
        {
            Vector3 euler = transform.eulerAngles;
            if (zeroPitchInViewPoint) euler.x = 0f;
            if (zeroRollInViewPoint) euler.z = 0f;
            return Quaternion.Euler(euler);
        }

        bool ShouldDrawBeamRayObject(int beamIndex, int sampleIndex)
        {
            if (!drawBeamRayObjects) return false;

            int beamStride = Mathf.Max(1, debugRayBeamStride);
            int sampleStride = Mathf.Max(1, debugRayVerticalSampleStride);
            return beamIndex % beamStride == 0 && sampleIndex % sampleStride == 0;
        }

        void DrawIdleDebugRaysIfNeeded()
        {
            if (!drawBeamRaysWhenIdle) return;
            if (fireSessionActive) return;
            if (!drawBeamRays && !drawBeamRayObjects) return;
            if (Time.time < nextIdleDebugRayTime) return;

            nextIdleDebugRayTime = Time.time + 1f / Mathf.Max(0.5f, idleDebugRayHz);
            BuildFrameFromScene();
        }

        void DrawBeamRayObject(Vector3 start, Vector3 end, Color color)
        {
            if (debugRayRoot == null)
            {
                GameObject root = new GameObject("VirtualOculusDebugRays");
                root.transform.SetParent(transform, false);
                debugRayRoot = root.transform;
            }

            GameObject go = new GameObject("DebugRay");
            go.transform.SetParent(debugRayRoot, false);

            LineRenderer line = go.AddComponent<LineRenderer>();
            line.useWorldSpace = true;
            line.positionCount = 2;
            line.SetPosition(0, start);
            line.SetPosition(1, end);
            line.startWidth = debugRayObjectWidth;
            line.endWidth = debugRayObjectWidth;
            line.numCapVertices = 2;
            line.material = GetDebugRayMaterial();
            line.startColor = color;
            line.endColor = color;

            if (Application.isPlaying)
                Destroy(go, Mathf.Max(0.12f, debugRayDuration));
            else
                DestroyImmediate(go);
        }

        Material GetDebugRayMaterial()
        {
            if (debugRayMaterial != null) return debugRayMaterial;
            if (runtimeDebugRayMaterial != null) return runtimeDebugRayMaterial;

            Shader shader = Shader.Find("HDRP/Unlit");
            if (shader == null) shader = Shader.Find("Unlit/Color");
            if (shader == null) shader = Shader.Find("Sprites/Default");

            runtimeDebugRayMaterial = new Material(shader);
            if (runtimeDebugRayMaterial.HasProperty("_BaseColor"))
                runtimeDebugRayMaterial.SetColor("_BaseColor", Color.white);
            if (runtimeDebugRayMaterial.HasProperty("_Color"))
                runtimeDebugRayMaterial.SetColor("_Color", Color.white);
            return runtimeDebugRayMaterial;
        }

        bool TrySceneHits(Vector3 direction, float maxRange, out RaycastHit[] hits)
        {
            RaycastHit[] rawHits = useSphereCast
                ? Physics.SphereCastAll(transform.position, sphereCastRadius, direction, maxRange, targetLayers, triggerInteraction)
                : Physics.RaycastAll(transform.position, direction, maxRange, targetLayers, triggerInteraction);

            if (rawHits == null || rawHits.Length == 0)
            {
                hits = Array.Empty<RaycastHit>();
                return false;
            }

            Array.Sort(rawHits, (a, b) => a.distance.CompareTo(b.distance));
            List<RaycastHit> filteredHits = new List<RaycastHit>(rawHits.Length);
            for (int i = 0; i < rawHits.Length; i++)
            {
                RaycastHit hit = rawHits[i];
                if (ShouldIgnoreHitByTag(hit.collider)) continue;
                if (hit.distance < minRangeMeters) continue;
                filteredHits.Add(hit);
            }

            hits = filteredHits.ToArray();
            return hits.Length > 0;
        }

        void AddBeamEchoCandidate(List<BeamEcho> beamEchoes, RaycastHit hit, float score)
        {
            float minSeparation = Mathf.Max(minEchoSeparationMeters, 0.5f * echoSigmaMeters);
            int maxEchoCount = Mathf.Max(1, maxEchoesPerBeam);

            if (firstReturnOnly)
            {
                if (beamEchoes.Count == 0)
                {
                    beamEchoes.Add(new BeamEcho
                    {
                        Distance = hit.distance,
                        Score = score,
                        AggregateScore = score,
                        SampleCount = 1,
                        Hit = hit,
                    });
                    return;
                }

                BeamEcho first = beamEchoes[0];
                if (hit.distance < first.Distance - minSeparation)
                {
                    beamEchoes[0] = new BeamEcho
                    {
                        Distance = hit.distance,
                        Score = score,
                        AggregateScore = score,
                        SampleCount = 1,
                        Hit = hit,
                    };
                    return;
                }

                if (Mathf.Abs(first.Distance - hit.distance) <= minSeparation)
                {
                    if (score > first.Score)
                    {
                        first.Distance = hit.distance;
                        first.Score = score;
                        first.Hit = hit;
                    }

                    first.AggregateScore += score;
                    first.SampleCount += 1;
                    beamEchoes[0] = first;
                }

                return;
            }

            for (int i = 0; i < beamEchoes.Count; i++)
            {
                BeamEcho existing = beamEchoes[i];
                if (Mathf.Abs(existing.Distance - hit.distance) > minSeparation) continue;

                if (score > existing.Score)
                {
                    beamEchoes[i] = new BeamEcho
                    {
                        Distance = hit.distance,
                        Score = score,
                        AggregateScore = existing.AggregateScore + score,
                        SampleCount = existing.SampleCount + 1,
                        Hit = hit,
                    };
                }
                else
                {
                    existing.AggregateScore += score;
                    existing.SampleCount += 1;
                    beamEchoes[i] = existing;
                }
                return;
            }

            beamEchoes.Add(new BeamEcho
            {
                Distance = hit.distance,
                Score = score,
                AggregateScore = score,
                SampleCount = 1,
                Hit = hit,
            });

            beamEchoes.Sort((a, b) => b.Score.CompareTo(a.Score));
            if (beamEchoes.Count > maxEchoCount)
            {
                beamEchoes.RemoveRange(maxEchoCount, beamEchoes.Count - maxEchoCount);
            }
        }

        bool ShouldIgnoreHitByTag(Collider collider)
        {
            if (collider == null || string.IsNullOrEmpty(ignoreTag)) return false;
            string colliderTag = collider.tag;
            return string.Equals(colliderTag, ignoreTag, StringComparison.Ordinal);
        }

        float ComputeHitScore(RaycastHit hit, Vector3 direction, float maxRange)
        {
            float distanceTerm = 1f - Mathf.Clamp01(hit.distance / maxRange);
            float incidence = Mathf.Clamp01(Vector3.Dot(-direction.normalized, hit.normal.normalized));
            return 0.65f * distanceTerm + 0.35f * incidence;
        }

        void FillBackground(VirtualSonarPingFrame frame)
        {
            byte background = (byte)Mathf.Clamp(Mathf.RoundToInt(backgroundLevel * 255f), 0, 255);
            for (int rangeIndex = 0; rangeIndex < frame.RangeCount; rangeIndex++)
            {
                for (int beamIndex = 0; beamIndex < frame.BeamCount; beamIndex++)
                {
                    frame.Intensities8[rangeIndex, beamIndex] = background;
                }
            }
        }

        void WriteEcho(VirtualSonarPingFrame frame, int beamIndex, BeamEcho echo, float rangeResolution)
        {
            float distance = echo.Distance;
            RaycastHit hit = echo.Hit;
            int centerBin = Mathf.Clamp(Mathf.RoundToInt(distance / Mathf.Max(rangeResolution, 1e-6f)) - 1, 0, frame.RangeCount - 1);
            float rawIncidence = Mathf.Clamp01(Vector3.Dot(-(hit.point - transform.position).normalized, hit.normal.normalized));
            float incidence = Mathf.Pow(rawIncidence, Mathf.Max(0.01f, incidenceExponent));
            float attenuation = Mathf.Exp(-attenuationAlpha * distance);
            float repeatedHitFactor = 1f + repeatedHitBoost * Mathf.Max(0, echo.SampleCount - 1);
            float aggregateFactor = 1f + 0.15f * Mathf.Max(0f, echo.AggregateScore - echo.Score);
            float strength = maxEchoLevel * attenuation * Mathf.Lerp(1f - incidenceWeight, 1f, incidence) * repeatedHitFactor * aggregateFactor;
            strength = Mathf.Min(strength, maxEchoLevel * 2.5f);
            float echoWidth = Mathf.Max(rangeResolution, echoWidthMeters);
            float echoSigma = Mathf.Max(rangeResolution * 0.5f, echoSigmaMeters);
            int halfBins = Mathf.Max(1, Mathf.RoundToInt(echoWidth / Mathf.Max(rangeResolution, 1e-6f)));

            for (int delta = -halfBins; delta <= halfBins; delta++)
            {
                int rangeIndex = centerBin + delta;
                if (rangeIndex < 0 || rangeIndex >= frame.RangeCount) continue;

                float rangeOffset = delta * rangeResolution;
                float weight = Mathf.Exp(-0.5f * (rangeOffset * rangeOffset) / Mathf.Max(1e-6f, echoSigma * echoSigma));
                float value = 255f * strength * weight;
                int updated = frame.Intensities8[rangeIndex, beamIndex] + Mathf.RoundToInt(value);
                frame.Intensities8[rangeIndex, beamIndex] = (byte)Mathf.Clamp(updated, 0, 255);
            }
        }

        void AddNoise(VirtualSonarPingFrame frame)
        {
            for (int rangeIndex = 0; rangeIndex < frame.RangeCount; rangeIndex++)
            {
                for (int beamIndex = 0; beamIndex < frame.BeamCount; beamIndex++)
                {
                    float noise = NextGaussian() * noiseSigma * 255f;
                    int updated = frame.Intensities8[rangeIndex, beamIndex] + Mathf.RoundToInt(noise);
                    frame.Intensities8[rangeIndex, beamIndex] = (byte)Mathf.Clamp(updated, 0, 255);
                }
            }
        }

        float NextGaussian()
        {
            double u1 = 1.0 - rng.NextDouble();
            double u2 = 1.0 - rng.NextDouble();
            double randStdNormal = Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Sin(2.0 * Math.PI * u2);
            return (float)randStdNormal;
        }

        static float NormalizeSignedAngle(float degrees)
        {
            float angle = degrees;
            while (angle > 180f) angle -= 360f;
            while (angle < -180f) angle += 360f;
            return angle;
        }
    }
}
