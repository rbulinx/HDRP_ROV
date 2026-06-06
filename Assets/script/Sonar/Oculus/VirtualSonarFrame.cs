using System;

namespace Robalink.OculusEmulator
{
    [Serializable]
    public class VirtualSonarPingFrame
    {
        public double PingStartTimeSec;
        public double HeadingDeg;
        public double PitchDeg;
        public double RollDeg;
        public double TemperatureDegC = 15.0;
        public double SpeedOfSoundMps = 1500.0;
        public double GainPercent = 50.0;
        public double RangePercent = 1.0;
        public double SalinityPpt = 35.0;
        public double RangeResolutionMeters = 0.01;

        public double[] AzimuthsRad;
        public byte[,] Intensities8;

        public int BeamCount => AzimuthsRad?.Length ?? 0;
        public int RangeCount => Intensities8?.GetLength(0) ?? 0;
    }
}
