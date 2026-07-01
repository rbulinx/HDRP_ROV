using System;
using System.IO;
using System.Net;

namespace Robalink.OculusEmulator
{
    public enum OculusMessageType : ushort
    {
        SimpleFire = 0x15,
        PingResult = 0x22,
        SimplePingResult = 0x23,
        UserConfig = 0x55,
        Dummy = 0xff,
    }

    public enum PingRateType : byte
    {
        Normal = 0x00,
        High = 0x01,
        Highest = 0x02,
        Low = 0x03,
        Lowest = 0x04,
        Standby = 0x05,
    }

    public enum DataSizeType : byte
    {
        Data8Bit = 0,
        Data16Bit = 1,
        Data24Bit = 2,
        Data32Bit = 3,
    }

    public enum OculusPartNumberType : ushort
    {
        Undefined = 0,
        M750d = 1032,
    }

    public enum OculusDeviceType : ushort
    {
        Undefined = 0,
        ImagingSonar = 1,
    }

    public enum OculusFrequencyMode : byte
    {
        Low = 1,
        High = 2,
    }

    public struct OculusMessageHeaderFields
    {
        public ushort oculusId;
        public ushort srcDeviceId;
        public ushort dstDeviceId;
        public ushort msgId;
        public ushort msgVersion;
        public uint payloadSize;
        public ushort spare2;
    }

    public struct SimpleFireRequest
    {
        public OculusMessageHeaderFields Header;
        public OculusFrequencyMode MasterMode;
        public PingRateType PingRate;
        public byte GammaCorrection;
        public byte Flags;
        public double RangePercentOrMeters;
        public bool RangeIsMeters;
        public double GainPercent;
        public double SpeedOfSound;
        public double Salinity;
    }

    public static class OculusProtocol
    {
        public const ushort OculusCheckId = 0x4f53;
        public const int StatusPort = 52102;
        public const int DataPort = 52100;
        public const int MessageHeaderSize = 16;
        public const int SimpleFireMessageSize = 52;
        public const int SimpleFireMessage2Size = 100;

        public static ushort DegreesToBearing01(double deg)
        {
            return (ushort)Math.Round(deg * 100.0);
        }

        public static bool TryReadHeader(byte[] data, int offset, out OculusMessageHeaderFields header)
        {
            header = default;
            if (data == null || data.Length - offset < MessageHeaderSize)
            {
                return false;
            }

            using var ms = new MemoryStream(data, offset, MessageHeaderSize);
            using var br = new BinaryReader(ms);
            header.oculusId = br.ReadUInt16();
            header.srcDeviceId = br.ReadUInt16();
            header.dstDeviceId = br.ReadUInt16();
            header.msgId = br.ReadUInt16();
            header.msgVersion = br.ReadUInt16();
            header.payloadSize = br.ReadUInt32();
            header.spare2 = br.ReadUInt16();
            return header.oculusId == OculusCheckId;
        }

        public static bool TryParseSimpleFireRequest(byte[] data, out SimpleFireRequest request)
        {
            request = default;
            if (!TryReadHeader(data, 0, out var header))
            {
                return false;
            }
            if (header.msgId != (ushort)OculusMessageType.SimpleFire)
            {
                return false;
            }

            request.Header = header;
            using var ms = new MemoryStream(data);
            using var br = new BinaryReader(ms);
            ms.Position = MessageHeaderSize;

            if (data.Length >= SimpleFireMessage2Size)
            {
                request.MasterMode = (OculusFrequencyMode)br.ReadByte();
                request.PingRate = (PingRateType)br.ReadByte();
                br.ReadByte();
                request.GammaCorrection = br.ReadByte();
                request.Flags = br.ReadByte();
                request.RangePercentOrMeters = br.ReadDouble();
                request.RangeIsMeters = true;
                request.GainPercent = br.ReadDouble();
                request.SpeedOfSound = br.ReadDouble();
                request.Salinity = br.ReadDouble();
                return true;
            }
            if (data.Length >= SimpleFireMessageSize)
            {
                request.MasterMode = (OculusFrequencyMode)br.ReadByte();
                request.PingRate = (PingRateType)br.ReadByte();
                br.ReadByte();
                request.GammaCorrection = br.ReadByte();
                request.Flags = br.ReadByte();
                request.RangePercentOrMeters = br.ReadDouble();
                request.RangeIsMeters = true;
                request.GainPercent = br.ReadDouble();
                request.SpeedOfSound = br.ReadDouble();
                request.Salinity = br.ReadDouble();
                return true;
            }

            return false;
        }

        public static bool IsUserConfigRequest(byte[] data)
        {
            return TryReadHeader(data, 0, out var header) && header.msgId == (ushort)OculusMessageType.UserConfig;
        }

        public static byte[] BuildUserConfigPacket(ushort srcDeviceId, ushort dstDeviceId, uint ipAddr, uint ipMask, bool dhcpEnable)
        {
            using var ms = new MemoryStream();
            using var bw = new BinaryWriter(ms);
            bw.Write(OculusCheckId);
            bw.Write(srcDeviceId);
            bw.Write(dstDeviceId);
            bw.Write((ushort)OculusMessageType.UserConfig);
            bw.Write((ushort)0);
            bw.Write((uint)12);
            bw.Write((ushort)0);
            bw.Write(ipAddr);
            bw.Write(ipMask);
            bw.Write(dhcpEnable ? 1u : 0u);
            return ms.ToArray();
        }

        public static byte[] BuildStatusPacket(
            uint deviceId,
            ushort srcDeviceId,
            OculusPartNumberType partNumber,
            IPAddress ipAddr,
            IPAddress ipMask,
            IPAddress connectedIpAddr)
        {
            using var ms = new MemoryStream();
            using var bw = new BinaryWriter(ms);
            bw.Write(OculusCheckId);
            bw.Write(srcDeviceId);
            bw.Write((ushort)0);
            bw.Write((ushort)0);
            bw.Write((ushort)0);
            bw.Write((uint)0);
            bw.Write((ushort)0);

            bw.Write(deviceId);
            bw.Write((ushort)OculusDeviceType.ImagingSonar);
            bw.Write((ushort)partNumber);
            bw.Write((uint)0);

            for (int i = 0; i < 6; i++) bw.Write((uint)0);

            bw.Write(ToUInt32LE(ipAddr));
            bw.Write(ToUInt32LE(ipMask));
            bw.Write(ToUInt32LE(connectedIpAddr));

            bw.Write((byte)0x00);
            bw.Write((byte)0x11);
            bw.Write((byte)0x22);
            bw.Write((byte)0x33);
            bw.Write((byte)0x44);
            bw.Write((byte)0x55);

            for (int i = 0; i < 8; i++) bw.Write(15.0 + i * 0.1);
            bw.Write(1.0);
            bw.Seek(10, SeekOrigin.Begin);
            bw.Write((uint)(ms.Length - MessageHeaderSize));
            return ms.ToArray();
        }

        public static byte[] BuildSimplePingResult2Packet(
            VirtualSonarPingFrame frame,
            ushort srcDeviceId,
            ushort dstDeviceId,
            uint messageSequenceId,
            OculusFrequencyMode frequencyMode,
            PingRateType pingRate,
            byte gammaCorrection,
            byte flags,
            uint pingId,
            double frequencyHz,
            double pressureBar,
            DataSizeType dataSize)
        {
            if (frame == null) throw new ArgumentNullException(nameof(frame));
            if (frame.AzimuthsRad == null) throw new ArgumentNullException(nameof(frame.AzimuthsRad));
            if (frame.Intensities8 == null) throw new ArgumentNullException(nameof(frame.Intensities8));
            if (frame.Intensities8.GetLength(0) != frame.RangeCount) throw new ArgumentException("Intensities8 range dimension mismatch.");
            if (frame.Intensities8.GetLength(1) != frame.BeamCount) throw new ArgumentException("Intensities8 beam dimension mismatch.");
            if (frame.AzimuthsRad.Length != frame.BeamCount) throw new ArgumentException("Azimuth count mismatch.");

            int imageBytes = frame.RangeCount * frame.BeamCount;

            using var ms = new MemoryStream();
            using var bw = new BinaryWriter(ms);

            bw.Write(OculusCheckId);
            bw.Write(srcDeviceId);
            bw.Write(dstDeviceId);
            bw.Write((ushort)OculusMessageType.SimplePingResult);
            bw.Write((ushort)2);
            bw.Write((uint)0);
            bw.Write((ushort)0);

            bw.Write((byte)frequencyMode);
            bw.Write((byte)pingRate);
            bw.Write((byte)0);
            bw.Write(gammaCorrection);
            bw.Write(flags);
            bw.Write(frame.RangePercent);
            bw.Write(frame.GainPercent);
            bw.Write(frame.SpeedOfSoundMps);
            bw.Write(frame.SalinityPpt);
            bw.Write((uint)0);
            bw.Write((uint)0);
            bw.Write((uint)0);
            bw.Write((uint)0);
            for (int i = 0; i < 5; i++) bw.Write((uint)0);

            bw.Write(pingId);
            bw.Write(messageSequenceId);
            bw.Write(frequencyHz);
            bw.Write(frame.TemperatureDegC);
            bw.Write(pressureBar);
            bw.Write(frame.HeadingDeg);
            bw.Write(frame.PitchDeg);
            bw.Write(frame.RollDeg);
            bw.Write(frame.SpeedOfSoundMps);
            bw.Write(frame.PingStartTimeSec);
            bw.Write((byte)dataSize);
            bw.Write(frame.RangeResolutionMeters);
            bw.Write((ushort)frame.RangeCount);
            bw.Write((ushort)frame.BeamCount);
            bw.Write((uint)0);
            bw.Write((uint)0);
            bw.Write((uint)0);
            bw.Write((uint)0);

            long imageOffsetPosition = ms.Position;
            bw.Write((uint)0);
            bw.Write((uint)0);
            bw.Write((uint)0);

            for (int i = 0; i < frame.BeamCount; i++)
            {
                double deg = frame.AzimuthsRad[i] * 180.0 / Math.PI;
                bw.Write((short)DegreesToBearing01(deg));
            }

            int imageOffset = checked((int)ms.Position);

            for (int r = 0; r < frame.RangeCount; r++)
            {
                for (int b = 0; b < frame.BeamCount; b++)
                {
                    bw.Write(frame.Intensities8[r, b]);
                }
            }

            int messageSize = checked((int)ms.Length);
            uint payloadSize = (uint)(messageSize - MessageHeaderSize);
            bw.Seek(10, SeekOrigin.Begin);
            bw.Write(payloadSize);
            bw.Seek((int)imageOffsetPosition, SeekOrigin.Begin);
            bw.Write((uint)imageOffset);
            bw.Write((uint)imageBytes);
            bw.Write((uint)messageSize);

            return ms.ToArray();
        }

        public static uint ToUInt32LE(IPAddress ip)
        {
            if (ip == null) return 0;
            byte[] bytes = ip.GetAddressBytes();
            if (bytes.Length != 4) return 0;
            return (uint)(bytes[0] | (bytes[1] << 8) | (bytes[2] << 16) | (bytes[3] << 24));
        }
    }
}
