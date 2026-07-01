using System;
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;
using RbulinX.SitlEngine;

/// <summary>
/// ROV-specific IVehicleForceModule: buoyancy/wave water surface response, water current
/// drag, vectored thruster mixing and force application. Plugs into SitlEngineCore via
/// its vehicleModuleBehaviour slot. This is a fresh component (not yet wired into the
/// existing tuned scene) built alongside ArduPilotJsonSitlBridge, which remains untouched.
/// </summary>
[DisallowMultipleComponent]
public class RovWaterPhysicsModule : MonoBehaviour, IVehicleForceModule
{
    public enum ServoOutputMode
    {
        MotorOutputs,
        AxisMixedOutputs
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

    [Header("Thrusters")]
    public float defaultHorizontalThrusterMaxN = 100f;
    public float defaultVerticalThrusterMaxN = 200f;
    public SitlThrusterSpec[] sitlThrusters = Array.Empty<SitlThrusterSpec>();
    public float directThrustScale = 1f;

    [Header("Water Current")]
    public bool applyWaterCurrent = false;
    public bool autoApplyWaterCurrentWhenCurrentVelocityIsNonZero = true;
    public Vector3 currentVelocity = Vector3.zero;
    public bool logWaterCurrentForces;
    public float waterCurrentLogIntervalSeconds = 1f;

    [Header("Water Surface - Buoyancy Spring")]
    public bool applyWaterSurfaceForces = true;
    public float waterSurfaceDesiredSubmergeMeters = 0.5f;
    public bool holdDepthRelativeToWaterSurface = false;
    public float waterSurfaceVerticalSpringNPerM = 100f;
    public float waterSurfaceVerticalDampingNPerMS = 280f;

    [Header("Water Surface - Tilt Response")]
    public bool applyWaterSurfaceTiltForces = true;
    public float waterSurfaceTiltSpringNPerM = 50f;
    public float waterSurfaceTiltDampingNPerMS = 80f;
    public float maxWaterSurfaceForcePerPointN = 40f;
    public Vector3[] waterSurfaceLocalSamplePoints =
    {
        new Vector3(0.75f, 0f, 0.75f),
        new Vector3(-0.75f, 0f, 0.75f),
        new Vector3(0.75f, 0f, -0.75f),
        new Vector3(-0.75f, 0f, -0.75f)
    };

    [Header("Water Surface - Breach / Clamp Safety")]
    public float waterSurfaceClampBelowSurfaceMeters = 0.05f;
    public float waterSurfaceClampGuardMeters = 0.08f;
    public bool zeroUpwardVelocityOnWaterSurfaceClamp = true;
    public float waterSurfaceClampSoftenTimeSeconds = 0.15f;
    public float waterSurfaceClampHardMarginMeters = 0.1f;
    public float waterSurfaceBreachSpringMultiplier = 3f;
    public float waterSurfaceBreachAnticipationMeters = 0.1f;
    public bool suppressWaterSurfaceForcesWhileDiving = false;
    public float waterSurfaceDiveForceThresholdN = 0.5f;

    [Header("Water Surface - Signal Smoothing")]
    public bool lowPassWaterSurfaceForces = true;
    public float waterSurfaceLowPassTimeSeconds = 2.0f;

    [Header("Water Surface - Wave Auto-Tune (from Wind)")]
    public bool autoTuneWaveResponseFromOceanWind = true;
    public float oceanWindReferenceSpeedKmh = 30f;
    public float minAutoWaveLengthMeters = 4f;
    public float maxAutoWaveLengthMeters = 24f;
    public float minAutoWaterParticleMotionScale = 0.4f;
    public float maxAutoWaterParticleMotionScale = 2.5f;

    [Header("Water Surface - Wave Motion")]
    public float waterSurfaceWaveLengthMeters = 12f;
    public float waterSurfaceWavePeriodSeconds = 4f;
    public float waterParticleMotionScale = 2f;
    public float waterParticleAccelerationScale = 1.2f;
    public float maxWaterParticleVerticalForceN = 800f;
    public float minimumWaterSurfaceWaveInfluence = 0f;

    [Header("Water Surface - Horizontal Drift")]
    public Vector3 waterParticleHorizontalDirection = Vector3.forward;
    public float waterParticlePrimaryDirectionWeight = 1f;
    public float waterParticleCrossDirectionWeight = 0.2f;
    public float waterParticleObliqueDirectionWeight = 0.1f;
    public float waterParticleHorizontalMotionScale = 0.5f;
    public float waterParticleHorizontalDampingNPerMS = 120f;
    public float maxWaterParticleHorizontalForceN = 300f;

    [Header("Water Surface Reference")]
    public bool useWaterSurfaceAsVerticalOrigin = true;
    public WaterSurface waterSurface;
    public float waterSurfaceY = 0f;
    public float waterQueryError = 0.01f;
    public int waterQueryMaxIterations = 8;

    [Header("Servo Mapping")]
    public int[] horizontalServoChannels = { 1, 2, 3, 4 };
    public int[] verticalServoChannels = { 5, 6 };
    public float[] horizontalOutputScales = { 1f, 1f, 1f, 1f };
    public float[] verticalOutputScales = { -1f, -1f };
    public bool invertHorizontalMotorOutputs = false;
    [Range(0.01f, 1f)] public float actuatorOutputSmoothing = 0.12f;
    public float maxActuatorOutputSlewPerSecond = 1.5f;
    public ServoOutputMode servoOutputMode = ServoOutputMode.MotorOutputs;
    [Range(-2f, 2f)] public float horizontalSurgeScale = 1f;
    [Range(-2f, 2f)] public float horizontalSwayScale = 1f;
    [Range(-2f, 2f)] public float horizontalYawScale = 0.2f;
    [Range(0f, 1f)] public float motorOutputYawScale = 1f;
    [Range(0f, 2f)] public float verticalHeaveScale = 1f;
    [Range(0f, 2f)] public float verticalDifferentialScale = 1f;
    public bool applyThrusterForces = true;
    public bool requireServoOutputForces = true;
    public float forceOutputDeadzone = 0.01f;

    [Header("Logging")]
    public bool logForceApplication;
    public float forceLogIntervalSeconds = 0.25f;

    Rigidbody rb;
    Transform vehicleTransform;
    Vector3[] surfaceWorldSamplePoints = Array.Empty<Vector3>();
    float[] filteredWaterSurfaceSampleY = Array.Empty<float>();
    bool[] validWaterSurfaceSampleY = Array.Empty<bool>();
    bool hasFilteredWaterSurfaceSamples;
    float lastWaterSurfaceWaveDisplacementY;
    bool hasLastWaterSurfaceWaveDisplacementY;
    float lastWaterParticleVelocityY;
    bool hasLastWaterParticleVelocityY;
    float[] horizontalOutputTargets = Array.Empty<float>();
    float[] verticalOutputTargets = Array.Empty<float>();
    float[] horizontalOutputs = Array.Empty<float>();
    float[] verticalOutputs = Array.Empty<float>();
    WaterSearchParameters waterSearchParameters;
    WaterSearchResult waterSearchResult;
    bool hasWaterSearchCandidate;
    float lastWaterCurrentLogTime = -999f;
    float lastForceLogTime = -999f;

    void Reset() => EnsureInspectorDefaults();
    void OnValidate() => EnsureInspectorDefaults();

    public void OnSitlEnable(SitlEngineCore core, Rigidbody rigidbody, Transform transformRef)
    {
        EnsureInspectorDefaults();
        rb = rigidbody;
        vehicleTransform = transformRef;

        ResolveWaterSurfaceIfNeeded();
        waterSearchParameters = new WaterSearchParameters();
        waterSearchResult = new WaterSearchResult();
        hasWaterSearchCandidate = false;
        ResetFilteredWaterSurface();
        RefreshVehicleModel();
        EnsureOutputArrays();
        Array.Clear(horizontalOutputTargets, 0, horizontalOutputTargets.Length);
        Array.Clear(verticalOutputTargets, 0, verticalOutputTargets.Length);
        Array.Clear(horizontalOutputs, 0, horizontalOutputs.Length);
        Array.Clear(verticalOutputs, 0, verticalOutputs.Length);
        lastWaterCurrentLogTime = -999f;
        lastForceLogTime = -999f;
    }

    public void OnSitlDisable()
    {
    }

    public bool TryGetVerticalOriginY(Vector3 worldPosition, out float originY)
    {
        if (!useWaterSurfaceAsVerticalOrigin)
        {
            originY = 0f;
            return false;
        }

        originY = GetWaterSurfaceY(worldPosition);
        return true;
    }

    public void ApplyForces(SitlEngineCore core, Rigidbody rigidbody, Transform transformRef, float fixedDeltaTime, bool actuatorOutputIsFresh)
    {
        rb = rigidbody;
        vehicleTransform = transformRef;

        UpdateMappedOutputsFromServoPwm(core);
        ApplyWaterCurrentForces(core);
        ApplyWaterSurfaceForces();

        bool hasActiveOutput = HasActiveThrusterOutput();
        bool shouldApply = rb != null
            && actuatorOutputIsFresh
            && applyThrusterForces
            && (!requireServoOutputForces || hasActiveOutput);

        if (ShouldLogForceApplication())
        {
            Debug.Log(
                $"[RovWaterPhysicsModule] force apply={shouldApply} fresh={actuatorOutputIsFresh} active={hasActiveOutput} scale={directThrustScale:0.###} H=[{string.Join(", ", horizontalOutputs)}] V=[{string.Join(", ", verticalOutputs)}]");
        }

        if (shouldApply)
            ApplySitlThrusterForces();

        EnforceWaterSurfaceNoBreachClamp();
    }

    void ApplyWaterCurrentForces(SitlEngineCore core)
    {
        if (rb == null || !IsWaterCurrentActive())
            return;

#if UNITY_6000_0_OR_NEWER
        rb.linearDamping = 0f;
        Vector3 velocity = rb.linearVelocity;
#else
        rb.drag = 0f;
        Vector3 velocity = rb.velocity;
#endif
        Vector3 relativeVelocity = velocity - currentVelocity;
        float linearDrag = Mathf.Max(0f, core.linearDragCoefficient);
        if (linearDrag <= 0f || relativeVelocity.sqrMagnitude <= 1e-8f)
            return;

        Vector3 force = -relativeVelocity * linearDrag;
        rb.AddForce(force, ForceMode.Force);

        if (ShouldLogWaterCurrentForce())
        {
            Debug.Log(
                $"[RovWaterPhysicsModule] current velocity=[{currentVelocity.x:0.###}, {currentVelocity.y:0.###}, {currentVelocity.z:0.###}] relative=[{relativeVelocity.x:0.###}, {relativeVelocity.y:0.###}, {relativeVelocity.z:0.###}] drag={linearDrag:0.###} force=[{force.x:0.###}, {force.y:0.###}, {force.z:0.###}]");
        }
    }

    bool IsWaterCurrentActive()
    {
        return applyWaterCurrent
            || (autoApplyWaterCurrentWhenCurrentVelocityIsNonZero && currentVelocity.sqrMagnitude > 1e-8f);
    }

    void ApplyOceanWindWaveAutoTune()
    {
        if (!autoTuneWaveResponseFromOceanWind || waterSurface == null)
            return;

        float windRatio = Mathf.Clamp01(waterSurface.largeWindSpeed / Mathf.Max(0.001f, oceanWindReferenceSpeedKmh));
        float lengthRatio = windRatio * windRatio;
        waterSurfaceWaveLengthMeters = Mathf.Lerp(minAutoWaveLengthMeters, maxAutoWaveLengthMeters, lengthRatio);
        waterParticleMotionScale = Mathf.Lerp(minAutoWaterParticleMotionScale, maxAutoWaterParticleMotionScale, windRatio);
        waterSurfaceWavePeriodSeconds = Mathf.Sqrt(
            Mathf.PI * 2f * waterSurfaceWaveLengthMeters / Mathf.Max(0.001f, 9.80665f));

        float orientationRadians = waterSurface.largeOrientationValue * Mathf.Deg2Rad;
        waterParticleHorizontalDirection = new Vector3(Mathf.Sin(orientationRadians), 0f, Mathf.Cos(orientationRadians));
    }

    void ApplyWaterSurfaceForces()
    {
        if (rb == null || !applyWaterSurfaceForces)
            return;

        if (ShouldSuppressWaterSurfaceForcesForDive())
        {
            ResetFilteredWaterSurface();
            return;
        }

        ResolveWaterSurfaceIfNeeded();
        if (waterSurface == null)
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
        float meanSurfaceY = waterSurfaceY;
        float depthCenter = meanSurfaceY - rb.position.y;

        float waveInfluence = GetWaterSurfaceWaveInfluence(depthCenter);

        if (applyWaterSurfaceTiltForces)
        {
            float tiltSpring = Mathf.Max(0f, waterSurfaceTiltSpringNPerM);
            float tiltDamping = Mathf.Max(0f, waterSurfaceTiltDampingNPerMS);
            float maxForce = Mathf.Max(0f, maxWaterSurfaceForcePerPointN);

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
        float waterParticleAccelerationForce = rb.mass * waterParticleAccelerationY * Mathf.Max(0f, waterParticleAccelerationScale);
        float verticalForce;
        if (holdDepthRelativeToWaterSurface)
        {
            float hardLimitY = effectiveSurfaceY - Mathf.Max(0f, waterSurfaceClampBelowSurfaceMeters);
            float targetY = effectiveSurfaceY - Mathf.Max(0f, waterSurfaceDesiredSubmergeMeters);
            float guard = Mathf.Max(0f, waterSurfaceClampGuardMeters);
            if (guard > 0f)
                targetY = Mathf.Min(targetY, hardLimitY - guard);

            verticalForce = (targetY - rb.position.y) * Mathf.Max(0f, waterSurfaceVerticalSpringNPerM)
                + (waterParticleVelocityY - velocity.y) * Mathf.Max(0f, waterSurfaceVerticalDampingNPerMS)
                + waterParticleAccelerationForce;
        }
        else
        {
            float waveDisplacement = effectiveSurfaceY - meanSurfaceY;
            verticalForce = (waveDisplacement * Mathf.Max(0f, waterSurfaceVerticalSpringNPerM)
                + (waterParticleVelocityY - velocity.y) * Mathf.Max(0f, waterSurfaceVerticalDampingNPerMS)
                + waterParticleAccelerationForce);

            float breachOvershoot = rb.position.y - effectiveSurfaceY + Mathf.Max(0f, waterSurfaceBreachAnticipationMeters);
            if (breachOvershoot > 0f)
                verticalForce -= breachOvershoot * Mathf.Max(0f, waterSurfaceVerticalSpringNPerM) * Mathf.Max(0f, waterSurfaceBreachSpringMultiplier);
        }

        float maxVerticalForce = Mathf.Max(0f, maxWaterParticleVerticalForceN);
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

        float angularFrequency = Mathf.PI * 2f / Mathf.Max(0.001f, waterSurfaceWavePeriodSeconds);
        float targetVelocityAmplitude = surfaceWaveDisplacementY
            * angularFrequency
            * waveInfluence
            * Mathf.Max(0f, waterParticleMotionScale)
            * Mathf.Max(0f, waterParticleHorizontalMotionScale);

        Vector3 force = Vector3.zero;
        force += CalculateWaterParticleDirectionalForce(direction, targetVelocityAmplitude, Mathf.Max(0f, waterParticlePrimaryDirectionWeight), velocity, waveInfluence);
        force += CalculateWaterParticleDirectionalForce(new Vector3(-direction.z, 0f, direction.x), targetVelocityAmplitude, Mathf.Max(0f, waterParticleCrossDirectionWeight), velocity, waveInfluence);
        force += CalculateWaterParticleDirectionalForce((direction + new Vector3(direction.z, 0f, -direction.x)).normalized, targetVelocityAmplitude, Mathf.Max(0f, waterParticleObliqueDirectionWeight), velocity, waveInfluence);

        float maxForce = Mathf.Max(0f, maxWaterParticleHorizontalForceN);
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
            targetVelocity += Vector3.Dot(currentVelocity, direction);

        float currentAlongDirection = Vector3.Dot(velocity, direction);
        float dampingGain = Mathf.Max(0f, waterParticleHorizontalDampingNPerMS) * Mathf.Clamp01(waveInfluence);
        float force = (targetVelocity - currentAlongDirection) * dampingGain;
        return direction * force;
    }

    Vector3 GetWaterParticleHorizontalDirection()
    {
        Vector3 direction = waterParticleHorizontalDirection;
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
        velocityY = surfaceWaveVelocityY * waveInfluence * Mathf.Max(0f, waterParticleMotionScale);

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
        float minInfluence = Mathf.Clamp01(minimumWaterSurfaceWaveInfluence);
        float wavelength = Mathf.Max(0.001f, waterSurfaceWaveLengthMeters);
        float influence = Mathf.Exp(-Mathf.PI * 2f * Mathf.Max(0f, depthMeters) / wavelength);
        return Mathf.Max(minInfluence, Mathf.Clamp01(influence));
    }

    void ApplySitlThrusterForces()
    {
        if (rb == null || sitlThrusters == null)
            return;

        float scale = Mathf.Max(0f, directThrustScale);
        for (int i = 0; i < sitlThrusters.Length; i++)
        {
            SitlThrusterSpec thruster = sitlThrusters[i];
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

            Vector3 worldPosition = vehicleTransform.TransformPoint(thruster.localPosition);
            Vector3 worldDirection = vehicleTransform.TransformDirection(localDirection.normalized);
            rb.AddForceAtPosition(worldDirection * thrust, worldPosition, ForceMode.Force);
        }
    }

    float GetMappedThrusterOutput(int oneBasedServoChannel)
    {
        if (horizontalServoChannels != null && horizontalOutputs != null)
        {
            int count = Mathf.Min(horizontalServoChannels.Length, horizontalOutputs.Length);
            for (int i = 0; i < count; i++)
            {
                if (horizontalServoChannels[i] == oneBasedServoChannel)
                    return horizontalOutputs[i];
            }
        }

        if (verticalServoChannels != null && verticalOutputs != null)
        {
            int count = Mathf.Min(verticalServoChannels.Length, verticalOutputs.Length);
            for (int i = 0; i < count; i++)
            {
                if (verticalServoChannels[i] == oneBasedServoChannel)
                    return verticalOutputs[i];
            }
        }

        return 0f;
    }

    float GetCommandedWorldVerticalThrusterForce()
    {
        if (sitlThrusters == null)
            return 0f;

        float verticalForce = 0f;
        float scale = Mathf.Max(0f, directThrustScale);
        for (int i = 0; i < sitlThrusters.Length; i++)
        {
            SitlThrusterSpec thruster = sitlThrusters[i];
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

            Vector3 worldDirection = vehicleTransform.TransformDirection(localDirection.normalized);
            verticalForce += worldDirection.y * thrust;
        }

        return verticalForce;
    }

    bool ShouldSuppressWaterSurfaceForcesForDive()
    {
        return suppressWaterSurfaceForcesWhileDiving
            && GetCommandedWorldVerticalThrusterForce() < -Mathf.Max(0f, waterSurfaceDiveForceThresholdN);
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

    void RefreshVehicleModel()
    {
        if (sitlThrusters == null || sitlThrusters.Length == 0)
            sitlThrusters = CreateDefaultVectoredThrusterSpecs();

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
        float horizontalThrust = Mathf.Max(0f, defaultHorizontalThrusterMaxN);
        float verticalThrust = Mathf.Max(0f, defaultVerticalThrusterMaxN);

        return new[]
        {
            BuildSitlThrusterSpec("M1 Front Right", GetOneBasedChannel(horizontalServoChannels, 0), new Vector3( x, y,  z), new Vector3( 1f, 0f, -1f), horizontalThrust, horizontalThrust),
            BuildSitlThrusterSpec("M2 Front Left",  GetOneBasedChannel(horizontalServoChannels, 1), new Vector3(-x, y,  z), new Vector3(-1f, 0f, -1f), horizontalThrust, horizontalThrust),
            BuildSitlThrusterSpec("M3 Rear Right",  GetOneBasedChannel(horizontalServoChannels, 2), new Vector3( x, y, -z), new Vector3( 1f, 0f,  1f), horizontalThrust, horizontalThrust),
            BuildSitlThrusterSpec("M4 Rear Left",   GetOneBasedChannel(horizontalServoChannels, 3), new Vector3(-x, y, -z), new Vector3(-1f, 0f,  1f), horizontalThrust, horizontalThrust),
            BuildSitlThrusterSpec("M5 Left Vertical",  GetOneBasedChannel(verticalServoChannels, 0), new Vector3(-verticalX, 0f, 0f), Vector3.up, verticalThrust, verticalThrust),
            BuildSitlThrusterSpec("M6 Right Vertical", GetOneBasedChannel(verticalServoChannels, 1), new Vector3( verticalX, 0f, 0f), Vector3.up, verticalThrust, verticalThrust),
        };
    }

    void NormalizeSitlThrusterSpecs()
    {
        if (sitlThrusters == null)
            return;

        for (int i = 0; i < sitlThrusters.Length; i++)
        {
            SitlThrusterSpec thruster = sitlThrusters[i];
            if (thruster == null)
            {
                thruster = BuildSitlThrusterSpec($"Thruster {i + 1}", 1, Vector3.zero, Vector3.forward, 0f, 0f);
                sitlThrusters[i] = thruster;
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

    float GetWaterSurfaceY(Vector3 worldPosition)
    {
        ResolveWaterSurfaceIfNeeded();
        if (waterSurface != null && TryGetWaterSurfaceY(worldPosition, out float y))
            return y;

        return waterSurfaceY;
    }

    void ResolveWaterSurfaceIfNeeded()
    {
        if (waterSurface != null)
            return;

        waterSurface = FindFirstObjectByType<WaterSurface>();
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
        waterSearchParameters.error = waterQueryError;
        waterSearchParameters.maxIterations = waterQueryMaxIterations;
        waterSearchParameters.outputNormal = false;

        if (waterSurface.ProjectPointOnWaterSurface(waterSearchParameters, out waterSearchResult))
        {
            surfaceY = waterSearchResult.projectedPositionWS.y;
            return true;
        }

        hasWaterSearchCandidate = false;
        surfaceY = waterSurfaceY;
        return false;
    }

    Vector3[] GetWaterSurfaceSamplePoints()
    {
        if (waterSurfaceLocalSamplePoints == null || waterSurfaceLocalSamplePoints.Length == 0)
            return null;

        if (surfaceWorldSamplePoints == null || surfaceWorldSamplePoints.Length != waterSurfaceLocalSamplePoints.Length)
            surfaceWorldSamplePoints = new Vector3[waterSurfaceLocalSamplePoints.Length];

        for (int i = 0; i < waterSurfaceLocalSamplePoints.Length; i++)
            surfaceWorldSamplePoints[i] = vehicleTransform.TransformPoint(waterSurfaceLocalSamplePoints[i]);

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
        if (!lowPassWaterSurfaceForces || waterSurfaceLowPassTimeSeconds <= 0f)
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
        float tau = Mathf.Max(0.001f, waterSurfaceLowPassTimeSeconds);
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
        if (rb == null || !applyWaterSurfaceForces)
            return;

        if (ShouldSuppressWaterSurfaceForcesForDive())
            return;

        ResolveWaterSurfaceIfNeeded();
        if (waterSurface == null)
            return;

        float hardLimitY = GetWaterSurfaceY(rb.position) - Mathf.Max(0f, waterSurfaceClampBelowSurfaceMeters);
        float overshoot = rb.position.y - hardLimitY;
        if (overshoot <= 0f)
            return;

        float softenTime = Mathf.Max(0f, waterSurfaceClampSoftenTimeSeconds);
        float alpha = softenTime > 0f
            ? 1f - Mathf.Exp(-Mathf.Max(0.0001f, Time.fixedDeltaTime) / softenTime)
            : 1f;

        float maxOvershoot = Mathf.Max(0f, waterSurfaceClampHardMarginMeters);
        float correctedOvershoot = overshoot * (1f - alpha);
        bool hitHardMargin = correctedOvershoot > maxOvershoot;
        if (hitHardMargin)
            correctedOvershoot = maxOvershoot;

        Vector3 position = rb.position;
        position.y = hardLimitY + correctedOvershoot;
        rb.position = position;

        if (!zeroUpwardVelocityOnWaterSurfaceClamp)
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

    void UpdateMappedOutputsFromServoPwm(SitlEngineCore core)
    {
        EnsureOutputArrays();
        float horizontalSign = invertHorizontalMotorOutputs ? -1f : 1f;
        for (int i = 0; i < horizontalOutputTargets.Length; i++)
        {
            int channel = i < horizontalServoChannels.Length ? horizontalServoChannels[i] : 0;
            horizontalOutputTargets[i] = channel > 0
                ? core.GetChannelOutput(channel) * GetOutputScale(horizontalOutputScales, i) * horizontalSign
                : 0f;
        }

        for (int i = 0; i < verticalOutputTargets.Length; i++)
        {
            int channel = i < verticalServoChannels.Length ? verticalServoChannels[i] : 0;
            verticalOutputTargets[i] = channel > 0
                ? core.GetChannelOutput(channel) * GetOutputScale(verticalOutputScales, i)
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
    }

    bool HasActiveThrusterOutput()
    {
        float threshold = Mathf.Max(0f, forceOutputDeadzone);
        return HasActiveOutput(horizontalOutputs, threshold) || HasActiveOutput(verticalOutputs, threshold);
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
        int horizontalCount = horizontalServoChannels != null ? horizontalServoChannels.Length : 0;
        int verticalCount = verticalServoChannels != null ? verticalServoChannels.Length : 0;

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

        float dt = Mathf.Max(0.001f, Time.fixedDeltaTime);
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

    void EnsureInspectorDefaults()
    {
        defaultHorizontalThrusterMaxN = Mathf.Max(0f, defaultHorizontalThrusterMaxN);
        defaultVerticalThrusterMaxN = Mathf.Max(0f, defaultVerticalThrusterMaxN);
        if (sitlThrusters == null)
            sitlThrusters = Array.Empty<SitlThrusterSpec>();
        directThrustScale = Mathf.Clamp(directThrustScale, 0f, 1f);

        currentVelocity = new Vector3(
            float.IsFinite(currentVelocity.x) ? currentVelocity.x : 0f,
            float.IsFinite(currentVelocity.y) ? currentVelocity.y : 0f,
            float.IsFinite(currentVelocity.z) ? currentVelocity.z : 0f);
        waterCurrentLogIntervalSeconds = Mathf.Max(0f, waterCurrentLogIntervalSeconds);

        if (horizontalServoChannels == null || horizontalServoChannels.Length == 0)
            horizontalServoChannels = new[] { 1, 2, 3, 4 };
        if (verticalServoChannels == null || verticalServoChannels.Length == 0)
            verticalServoChannels = new[] { 5, 6 };
        if (horizontalOutputScales == null || horizontalOutputScales.Length != horizontalServoChannels.Length)
            horizontalOutputScales = CreateFilledArray(horizontalServoChannels.Length, 1f);
        if (verticalOutputScales == null || verticalOutputScales.Length != verticalServoChannels.Length)
            verticalOutputScales = CreateFilledArray(verticalServoChannels.Length, -1f);

        for (int i = 0; i < horizontalServoChannels.Length; i++)
        {
            horizontalServoChannels[i] = Mathf.Clamp(horizontalServoChannels[i], 1, 32);
            horizontalOutputScales[i] = Mathf.Clamp(horizontalOutputScales[i], -2f, 2f);
        }
        for (int i = 0; i < verticalServoChannels.Length; i++)
        {
            verticalServoChannels[i] = Mathf.Clamp(verticalServoChannels[i], 1, 32);
            verticalOutputScales[i] = Mathf.Clamp(verticalOutputScales[i], -2f, 2f);
        }

        if (sitlThrusters == null || sitlThrusters.Length == 0)
            sitlThrusters = CreateDefaultVectoredThrusterSpecs();

        actuatorOutputSmoothing = Mathf.Clamp(actuatorOutputSmoothing, 0.05f, 1f);
        maxActuatorOutputSlewPerSecond = Mathf.Max(1f, maxActuatorOutputSlewPerSecond);
        horizontalSurgeScale = Mathf.Clamp(horizontalSurgeScale, -2f, 2f);
        horizontalSwayScale = Mathf.Clamp(horizontalSwayScale, -2f, 2f);
        horizontalYawScale = Mathf.Clamp(horizontalYawScale, -2f, 2f);
        motorOutputYawScale = Mathf.Clamp01(motorOutputYawScale);
        verticalHeaveScale = Mathf.Clamp(verticalHeaveScale, 0f, 2f);
        verticalDifferentialScale = Mathf.Clamp(verticalDifferentialScale, 0f, 2f);
        forceOutputDeadzone = Mathf.Max(0f, forceOutputDeadzone);
        forceLogIntervalSeconds = Mathf.Max(0f, forceLogIntervalSeconds);

        waterSurfaceY = float.IsFinite(waterSurfaceY) ? waterSurfaceY : 0f;
        waterQueryError = Mathf.Max(0.001f, waterQueryError);
        waterQueryMaxIterations = Mathf.Max(1, waterQueryMaxIterations);
        waterSurfaceDesiredSubmergeMeters = Mathf.Max(0f, waterSurfaceDesiredSubmergeMeters);
        waterSurfaceVerticalSpringNPerM = Mathf.Max(0f, waterSurfaceVerticalSpringNPerM);
        waterSurfaceVerticalDampingNPerMS = Mathf.Max(0f, waterSurfaceVerticalDampingNPerMS);
        waterSurfaceTiltSpringNPerM = Mathf.Max(0f, waterSurfaceTiltSpringNPerM);
        waterSurfaceTiltDampingNPerMS = Mathf.Max(0f, waterSurfaceTiltDampingNPerMS);
        maxWaterSurfaceForcePerPointN = Mathf.Max(0f, maxWaterSurfaceForcePerPointN);
        waterSurfaceClampBelowSurfaceMeters = Mathf.Max(0f, waterSurfaceClampBelowSurfaceMeters);
        waterSurfaceClampGuardMeters = Mathf.Max(0f, waterSurfaceClampGuardMeters);
        waterSurfaceDiveForceThresholdN = Mathf.Max(0f, waterSurfaceDiveForceThresholdN);
        waterSurfaceLowPassTimeSeconds = Mathf.Max(0f, waterSurfaceLowPassTimeSeconds);
        oceanWindReferenceSpeedKmh = Mathf.Max(0.001f, oceanWindReferenceSpeedKmh);
        minAutoWaveLengthMeters = Mathf.Max(0.001f, minAutoWaveLengthMeters);
        maxAutoWaveLengthMeters = Mathf.Max(minAutoWaveLengthMeters, maxAutoWaveLengthMeters);
        minAutoWaterParticleMotionScale = Mathf.Max(0f, minAutoWaterParticleMotionScale);
        maxAutoWaterParticleMotionScale = Mathf.Max(minAutoWaterParticleMotionScale, maxAutoWaterParticleMotionScale);
        waterSurfaceWaveLengthMeters = Mathf.Max(0.001f, waterSurfaceWaveLengthMeters);
        waterParticleMotionScale = Mathf.Max(0f, waterParticleMotionScale);
        waterParticleAccelerationScale = Mathf.Max(0f, waterParticleAccelerationScale);
        maxWaterParticleVerticalForceN = Mathf.Max(0f, maxWaterParticleVerticalForceN);
        waterSurfaceWavePeriodSeconds = Mathf.Max(0.001f, waterSurfaceWavePeriodSeconds);
        waterParticlePrimaryDirectionWeight = Mathf.Max(0f, waterParticlePrimaryDirectionWeight);
        waterParticleCrossDirectionWeight = Mathf.Max(0f, waterParticleCrossDirectionWeight);
        waterParticleObliqueDirectionWeight = Mathf.Max(0f, waterParticleObliqueDirectionWeight);
        waterParticleHorizontalMotionScale = Mathf.Max(0f, waterParticleHorizontalMotionScale);
        waterParticleHorizontalDampingNPerMS = Mathf.Max(0f, waterParticleHorizontalDampingNPerMS);
        maxWaterParticleHorizontalForceN = Mathf.Max(0f, maxWaterParticleHorizontalForceN);
        minimumWaterSurfaceWaveInfluence = Mathf.Clamp01(minimumWaterSurfaceWaveInfluence);
        if (waterSurfaceLocalSamplePoints == null)
            waterSurfaceLocalSamplePoints = Array.Empty<Vector3>();
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
}
