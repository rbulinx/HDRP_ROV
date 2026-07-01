using System;
using UnityEditor;
using UnityEngine;
using RbulinX.SitlEngine;

/// <summary>
/// Editor-only utility: adds SitlEngineCore + RovWaterPhysicsModule to the same
/// GameObject as the existing, tuned ArduPilotJsonSitlBridge and copies every tuned
/// value across. Both new components are left disabled so the existing bridge stays
/// the active driver until you manually flip the enabled checkboxes to A/B test.
/// ArduPilotJsonSitlBridge itself is never modified.
/// </summary>
public static class AttachSitlEngineModules
{
    [MenuItem("Tools/SITL Engine/Attach New Engine To Selected ArduPilotJsonSitlBridge")]
    static void AttachModules()
    {
        ArduPilotJsonSitlBridge source = Selection.activeGameObject != null
            ? Selection.activeGameObject.GetComponentInParent<ArduPilotJsonSitlBridge>()
            : null;
        if (source == null)
            source = UnityEngine.Object.FindFirstObjectByType<ArduPilotJsonSitlBridge>();

        if (source == null)
        {
            EditorUtility.DisplayDialog("SITL Engine", "No ArduPilotJsonSitlBridge found in the scene (select the ROV object first).", "OK");
            return;
        }

        GameObject target = source.gameObject;

        RovWaterPhysicsModule module = target.GetComponent<RovWaterPhysicsModule>();
        if (module == null)
            module = (RovWaterPhysicsModule)Undo.AddComponent(target, typeof(RovWaterPhysicsModule));
        else
            Undo.RecordObject(module, "Attach SITL Engine Modules");

        SitlEngineCore core = target.GetComponent<SitlEngineCore>();
        if (core == null)
            core = (SitlEngineCore)Undo.AddComponent(target, typeof(SitlEngineCore));
        else
            Undo.RecordObject(core, "Attach SITL Engine Modules");

        CopyFields(source, core, module);
        core.vehicleModuleBehaviour = module;

        // Keep the new engine off by default; ArduPilotJsonSitlBridge stays the active driver.
        core.enabled = false;
        module.enabled = false;

        EditorUtility.SetDirty(target);
        EditorUtility.SetDirty(core);
        EditorUtility.SetDirty(module);
        Selection.activeGameObject = target;
        Debug.Log($"[SITL Engine] Attached SitlEngineCore + RovWaterPhysicsModule to '{target.name}' (disabled) with tuned values copied from ArduPilotJsonSitlBridge. " +
            "To A/B test: disable ArduPilotJsonSitlBridge and enable the two new components - they share the same UDP port, so only enable one driver at a time.");
    }

    static void CopyFields(ArduPilotJsonSitlBridge src, SitlEngineCore core, RovWaterPhysicsModule module)
    {
        core.port = src._1Port;
        core.connectOnEnable = src.connectOnEnable;
        core.maxPacketsPerUpdate = src.maxPacketsPerUpdate;
        core.receiveBufferSizeBytes = src.receiveBufferSizeBytes;
        core.telemetrySource = src._2Target;
        core.vehicleMassKg = src._3MassKg;
        core.linearDragCoefficient = src._4LinDrag;
        core.angularDragCoefficient = src._5AngDrag;

        core.servoPwmMin = src.servoPwmMin;
        core.servoPwmNeutral = src.servoPwmNeutral;
        core.servoPwmMax = src.servoPwmMax;
        core.servoPwmDeadzone = src.servoPwmDeadzone;
        core.actuatorOutputTimeoutSeconds = src.actuatorOutputTimeoutSeconds;
        core.forceOutputDeadzone = src.forceOutputDeadzone;

        core.zeroUnityPositionAtStart = src.zeroUnityPositionAtStart;
        core.worldFrameMapping = (SitlEngineCore.WorldFrameMapping)(int)src.worldFrameMapping;
        core.invertReportedHorizontalPosition = src.invertReportedHorizontalPosition;
        core.freezeVerticalState = src.freezeVerticalState;
        core.freezeYawRate = src.freezeYawRate;
        core.reportYawRate = src.reportYawRate;
        core.reportVerticalState = src.reportVerticalState;
        core.zeroReportedRollPitchAtStart = src.zeroReportedRollPitchAtStart;
        core.yawAngleSign = src.yawAngleSign;
        core.yawRateSign = src.yawRateSign;
        core.rollAngleSign = src.rollAngleSign;
        core.pitchAngleSign = src.pitchAngleSign;
        core.forceLevelRollPitchReport = src.forceLevelRollPitchReport;
        core.zeroRollPitchGyroWhenLevel = src.zeroRollPitchGyroWhenLevel;
        core.stabilizeEkfBeforeActuatorOutput = src.stabilizeEkfBeforeActuatorOutput;
        core.maxReportedRollPitchDegrees = src.maxReportedRollPitchDegrees;
        core.bodyEulerOffsetDegrees = src.bodyEulerOffsetDegrees;
        core.filterReportedAttitude = src.filterReportedAttitude;
        core.reportedAttitudeSmoothing = src.reportedAttitudeSmoothing;
        core.deriveReportedYawRateFromAttitude = src.deriveReportedYawRateFromAttitude;
        core.maxReportedYawRateRadiansPerSecond = src.maxReportedYawRateRadiansPerSecond;
        core.deriveVelocityFromPosition = src.deriveVelocityFromPosition;
        core.usePositionVelocityWhenRigidbodyIsStopped = src.usePositionVelocityWhenRigidbodyIsStopped;
        core.deriveVerticalVelocityFromPosition = src.deriveVerticalVelocityFromPosition;
        core.forceStableVerticalReport = src.forceStableVerticalReport;
        core.filterReportedPosition = src.filterReportedPosition;
        core.positionTinyValueEpsilon = src.positionTinyValueEpsilon;
        core.maxReportedHorizontalSpeed = src.maxReportedHorizontalSpeed;
        core.maxReportedVerticalSpeed = src.maxReportedVerticalSpeed;
        core.rigidbodyVelocityDeadzone = src.rigidbodyVelocityDeadzone;
        core.verticalVelocityDeadzone = src.verticalVelocityDeadzone;
        core.verticalVelocitySmoothing = src.verticalVelocitySmoothing;
        core.maxReportedHorizontalStepMeters = src.maxReportedHorizontalStepMeters;
        core.reportedPositionSmoothing = src.reportedPositionSmoothing;
        core.gravityMetersPerSecondSquared = src.gravityMetersPerSecondSquared;
        core.includeLinearAccelerationInImu = src.includeLinearAccelerationInImu;
        core.accelerationSmoothing = src.accelerationSmoothing;
        core.maxReportedAccelerationMetersPerSecondSquared = src.maxReportedAccelerationMetersPerSecondSquared;

        core.logConnection = src.logConnection;
        core.logServoPackets = src.logServoPackets;
        core.servoLogIntervalSeconds = src.servoLogIntervalSeconds;
        core.logTelemetryPackets = src.logTelemetryPackets;
        core.telemetryLogIntervalSeconds = src.telemetryLogIntervalSeconds;
        core.minTelemetryLogIntervalSeconds = src.minTelemetryLogIntervalSeconds;
        core.logEkfDiagnostics = src.logEkfDiagnostics;
        core.warnEkfDiagnosticJumps = src.warnEkfDiagnosticJumps;
        core.ekfDiagnosticLogIntervalSeconds = src.ekfDiagnosticLogIntervalSeconds;
        core.ekfYawJumpWarningDegrees = src.ekfYawJumpWarningDegrees;
        core.ekfPositionJumpWarningMeters = src.ekfPositionJumpWarningMeters;
        core.ekfVelocityMismatchWarningMetersPerSecond = src.ekfVelocityMismatchWarningMetersPerSecond;
        core.ekfGyroZWarningRadiansPerSecond = src.ekfGyroZWarningRadiansPerSecond;
        core.ekfAccelWarningMetersPerSecondSquared = src.ekfAccelWarningMetersPerSecondSquared;
        core.logJsonSendErrors = src.logJsonSendErrors;
        core.jsonTinyValueEpsilon = src.jsonTinyValueEpsilon;

        module.defaultHorizontalThrusterMaxN = src._6HThrustN;
        module.defaultVerticalThrusterMaxN = src._7VThrustN;
        module.sitlThrusters = ConvertThrusters(src._8Thrusters);
        module.directThrustScale = src._56ThrustScale;

        module.applyWaterCurrent = src._9UseCurrent;
        module.autoApplyWaterCurrentWhenCurrentVelocityIsNonZero = src._10AutoCurrent;
        module.currentVelocity = src._11CurrentVel;
        module.logWaterCurrentForces = src.logWaterCurrentForces;
        module.waterCurrentLogIntervalSeconds = src.waterCurrentLogIntervalSeconds;

        module.applyWaterSurfaceForces = src._12UseSurface;
        module.waterSurfaceDesiredSubmergeMeters = src._13SubmergeM;
        module.holdDepthRelativeToWaterSurface = src._14HoldDepth;
        module.waterSurfaceVerticalSpringNPerM = src._15VSpring;
        module.waterSurfaceVerticalDampingNPerMS = src._16VDamping;

        module.applyWaterSurfaceTiltForces = src._17UseTilt;
        module.waterSurfaceTiltSpringNPerM = src._18TiltSpring;
        module.waterSurfaceTiltDampingNPerMS = src._19TiltDamping;
        module.maxWaterSurfaceForcePerPointN = src._20MaxPointN;
        module.waterSurfaceLocalSamplePoints = src._21SamplePts != null ? (Vector3[])src._21SamplePts.Clone() : module.waterSurfaceLocalSamplePoints;

        module.waterSurfaceClampBelowSurfaceMeters = src._22ClampBelowM;
        module.waterSurfaceClampGuardMeters = src._23ClampGuardM;
        module.zeroUpwardVelocityOnWaterSurfaceClamp = src._24ZeroUpVel;
        module.waterSurfaceClampSoftenTimeSeconds = src._25ClampSoftenS;
        module.waterSurfaceClampHardMarginMeters = src._26ClampHardM;
        module.waterSurfaceBreachSpringMultiplier = src._27BreachMult;
        module.waterSurfaceBreachAnticipationMeters = src._28BreachAntM;
        module.suppressWaterSurfaceForcesWhileDiving = src._29SuppressDive;
        module.waterSurfaceDiveForceThresholdN = src._30DiveThreshN;

        module.lowPassWaterSurfaceForces = src._31UseLowPass;
        module.waterSurfaceLowPassTimeSeconds = src._32LowPassS;

        module.autoTuneWaveResponseFromOceanWind = src._33AutoTuneWind;
        module.oceanWindReferenceSpeedKmh = src._34WindRefKmh;
        module.minAutoWaveLengthMeters = src._35MinWaveLenM;
        module.maxAutoWaveLengthMeters = src._36MaxWaveLenM;
        module.minAutoWaterParticleMotionScale = src._37MinMotionScale;
        module.maxAutoWaterParticleMotionScale = src._38MaxMotionScale;

        module.waterSurfaceWaveLengthMeters = src._39WaveLenM;
        module.waterSurfaceWavePeriodSeconds = src._40WavePeriodS;
        module.waterParticleMotionScale = src._41MotionScale;
        module.waterParticleAccelerationScale = src._42AccelScale;
        module.maxWaterParticleVerticalForceN = src._43MaxVForceN;
        module.minimumWaterSurfaceWaveInfluence = src._44MinWaveInfluence;

        module.waterParticleHorizontalDirection = src._45HDirection;
        module.waterParticlePrimaryDirectionWeight = src._46PrimaryWeight;
        module.waterParticleCrossDirectionWeight = src._47CrossWeight;
        module.waterParticleObliqueDirectionWeight = src._48ObliqueWeight;
        module.waterParticleHorizontalMotionScale = src._49HMotionScale;
        module.waterParticleHorizontalDampingNPerMS = src._50HDampingN;
        module.maxWaterParticleHorizontalForceN = src._51MaxHForceN;

        module.horizontalServoChannels = src._52HChannels != null ? (int[])src._52HChannels.Clone() : module.horizontalServoChannels;
        module.verticalServoChannels = src._53VChannels != null ? (int[])src._53VChannels.Clone() : module.verticalServoChannels;
        module.horizontalOutputScales = src._54HScales != null ? (float[])src._54HScales.Clone() : module.horizontalOutputScales;
        module.verticalOutputScales = src._55VScales != null ? (float[])src._55VScales.Clone() : module.verticalOutputScales;
        module.invertHorizontalMotorOutputs = src.invertHorizontalMotorOutputs;
        module.actuatorOutputSmoothing = src.actuatorOutputSmoothing;
        module.maxActuatorOutputSlewPerSecond = src.maxActuatorOutputSlewPerSecond;
        module.servoOutputMode = (RovWaterPhysicsModule.ServoOutputMode)(int)src.servoOutputMode;
        module.horizontalSurgeScale = src.horizontalSurgeScale;
        module.horizontalSwayScale = src.horizontalSwayScale;
        module.horizontalYawScale = src.horizontalYawScale;
        module.motorOutputYawScale = src.motorOutputYawScale;
        module.verticalHeaveScale = src.verticalHeaveScale;
        module.verticalDifferentialScale = src.verticalDifferentialScale;
        module.applyThrusterForces = src.applyThrusterForces;
        module.requireServoOutputForces = src.requireServoOutputForces;
        module.forceOutputDeadzone = src.forceOutputDeadzone;

        module.useWaterSurfaceAsVerticalOrigin = src._57UseSurfaceOrigin;
        module.waterSurface = src._58Surface;
        module.waterSurfaceY = src._59SurfaceY;
        module.waterQueryError = src._60QueryError;
        module.waterQueryMaxIterations = src._61QueryIter;

        module.logForceApplication = src.logForceApplication;
        module.forceLogIntervalSeconds = src.forceLogIntervalSeconds;
    }

    static RovWaterPhysicsModule.SitlThrusterSpec[] ConvertThrusters(ArduPilotJsonSitlBridge.SitlThrusterSpec[] source)
    {
        if (source == null)
            return Array.Empty<RovWaterPhysicsModule.SitlThrusterSpec>();

        var result = new RovWaterPhysicsModule.SitlThrusterSpec[source.Length];
        for (int i = 0; i < source.Length; i++)
        {
            ArduPilotJsonSitlBridge.SitlThrusterSpec s = source[i];
            result[i] = s == null ? null : new RovWaterPhysicsModule.SitlThrusterSpec
            {
                name = s.name,
                servoChannel = s.servoChannel,
                localPosition = s.localPosition,
                localDirection = s.localDirection,
                maxForwardThrustN = s.maxForwardThrustN,
                maxReverseThrustN = s.maxReverseThrustN
            };
        }
        return result;
    }
}
