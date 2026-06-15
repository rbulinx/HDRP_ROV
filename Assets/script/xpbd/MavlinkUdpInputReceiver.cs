using System;
using System.Net;
using System.Net.Sockets;
using UnityEngine;

[DisallowMultipleComponent]
public class MavlinkUdpInputReceiver : MonoBehaviour
{
    public enum ManualControlZMode
    {
        SignedMinus1000To1000,
        Centered0To1000
    }

    [Header("UDP")]
    public int listenPort = 14550;
    public bool listenOnEnable = true;

    [Header("Target")]
    public ROVGamepadThrustController rovController;
    public bool autoFindRovController = true;

    [Header("MANUAL_CONTROL Mapping")]
    public ManualControlZMode zMode = ManualControlZMode.SignedMinus1000To1000;
    public bool invertSway;
    public bool invertHeave;
    public bool invertYaw;

    [Header("Debug")]
    public bool logPackets;

    UdpClient udp;
    IPEndPoint remoteEndPoint;
    float lastPacketTime = -999f;
    int receivedManualControlCount;

    public bool IsListening => udp != null;
    public int ReceivedManualControlCount => receivedManualControlCount;
    public float LastPacketAgeSeconds => lastPacketTime > 0f ? Time.unscaledTime - lastPacketTime : float.PositiveInfinity;

    void OnEnable()
    {
        if (autoFindRovController && rovController == null)
            rovController = GetComponentInParent<ROVGamepadThrustController>();

        if (listenOnEnable)
            StartListening();
    }

    void OnDisable()
    {
        StopListening();
    }

    void Update()
    {
        if (udp == null)
            return;

        try
        {
            while (udp.Available > 0)
            {
                byte[] packet = udp.Receive(ref remoteEndPoint);
                if (TryParseManualControl(packet, out float surge, out float sway, out float heave, out float yaw))
                {
                    if (rovController != null)
                        rovController.SetMavlinkControlInput(surge, sway, heave, yaw);

                    receivedManualControlCount++;
                    lastPacketTime = Time.unscaledTime;

                    if (logPackets)
                        Debug.Log($"[MAVLink] MANUAL_CONTROL surge={surge:0.00} sway={sway:0.00} heave={heave:0.00} yaw={yaw:0.00}");
                }
            }
        }
        catch (SocketException e)
        {
            if (e.SocketErrorCode != SocketError.WouldBlock)
                Debug.LogWarning($"[MAVLink] UDP receive failed: {e.Message}");
        }
        catch (ObjectDisposedException)
        {
        }
    }

    public void StartListening()
    {
        if (udp != null)
            return;

        try
        {
            udp = new UdpClient();
            udp.Client.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ReuseAddress, true);
            udp.Client.Blocking = false;
            udp.Client.Bind(new IPEndPoint(IPAddress.Any, Mathf.Clamp(listenPort, 1, 65535)));
            remoteEndPoint = new IPEndPoint(IPAddress.Any, 0);
        }
        catch (Exception e)
        {
            udp = null;
            Debug.LogWarning($"[MAVLink] Failed to listen on UDP {listenPort}: {e.Message}");
        }
    }

    public void StopListening()
    {
        if (udp == null)
            return;

        udp.Close();
        udp = null;
    }

    bool TryParseManualControl(byte[] packet, out float surge, out float sway, out float heave, out float yaw)
    {
        surge = 0f;
        sway = 0f;
        heave = 0f;
        yaw = 0f;

        if (packet == null || packet.Length < 8)
            return false;

        byte magic = packet[0];
        int payloadLength;
        int payloadOffset;
        int messageId;

        if (magic == 0xFE)
        {
            payloadLength = packet[1];
            payloadOffset = 6;
            if (packet.Length < payloadOffset + payloadLength + 2)
                return false;

            messageId = packet[5];
        }
        else if (magic == 0xFD)
        {
            payloadLength = packet[1];
            payloadOffset = 10;
            if (packet.Length < payloadOffset + payloadLength + 2)
                return false;

            messageId = packet[7] | (packet[8] << 8) | (packet[9] << 16);
        }
        else
        {
            return false;
        }

        if (messageId != 69 || payloadLength < 10)
            return false;

        int x = ReadInt16LE(packet, payloadOffset + 0);
        int y = ReadInt16LE(packet, payloadOffset + 2);
        int z = ReadInt16LE(packet, payloadOffset + 4);
        int r = ReadInt16LE(packet, payloadOffset + 6);

        surge = NormalizeSignedAxis(x);
        sway = NormalizeSignedAxis(y);
        heave = zMode == ManualControlZMode.Centered0To1000
            ? Mathf.Clamp((z - 500f) / 500f, -1f, 1f)
            : NormalizeSignedAxis(z);
        yaw = NormalizeSignedAxis(r);

        if (invertSway) sway = -sway;
        if (invertHeave) heave = -heave;
        if (invertYaw) yaw = -yaw;

        return true;
    }

    static short ReadInt16LE(byte[] bytes, int offset)
    {
        return unchecked((short)(bytes[offset] | (bytes[offset + 1] << 8)));
    }

    static float NormalizeSignedAxis(int value)
    {
        return Mathf.Clamp(value / 1000f, -1f, 1f);
    }
}
