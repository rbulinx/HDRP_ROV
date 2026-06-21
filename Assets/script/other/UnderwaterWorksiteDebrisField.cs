using System.Collections.Generic;
using UnityEngine;
using Robalink.OculusEmulator;

[ExecuteAlways]
[DisallowMultipleComponent]
public class UnderwaterWorksiteDebrisField : MonoBehaviour
{
    public enum FieldShape
    {
        Ellipsoid,
        Box,
    }

    [Header("Field")]
    public bool debrisEnabled = true;
    public FieldShape fieldShape = FieldShape.Ellipsoid;
    [Range(0, 1000)] public int debrisCount = 120;
    public Vector3 fieldSize = new Vector3(8f, 3f, 8f);
    public int seed = 20260604;

    [Header("Debris look")]
    public Vector2 sizeRange = new Vector2(0.03f, 0.18f);
    [Range(0f, 1f)] public float alpha = 0.55f;
    public Color darkColor = new Color(0.32f, 0.28f, 0.20f, 1f);
    public Color lightColor = new Color(0.78f, 0.72f, 0.55f, 1f);
    [Range(0f, 1f)] public float plateRatio = 0.7f;
    public bool useRoundVisualDebris = true;
    [Range(0.1f, 1f)] public float visualSizeMultiplier = 0.55f;

    [Header("Motion")]
    public bool animateInPlayMode = true;
    public bool currentVelocityIsWorldSpace = true;
    public Vector3 currentVelocity = new Vector3(0.004f, 0.001f, -0.006f);
    public Vector2 randomDriftSpeed = new Vector2(0.001f, 0.01f);
    public Vector2 spinSpeedDeg = new Vector2(0.2f, 3f);
    public float bobAmplitude = 0.025f;
    public float bobFrequency = 0.12f;

    [Header("Water surface")]
    public bool hideAboveWaterSurface = true;
    public float waterSurfaceY = 0f;
    public float waterSurfaceMargin = 0.05f;

    [Header("Sonar return")]
    public bool visibleToSonar = true;
    public bool sonarUsesIndividualDebris = true;
    [Range(0f, 20f)] public float sonarEchoDensityPerMeter = 5f;
    [Range(0f, 1f)] public float sonarReflectivity = 0.75f;
    [Range(0f, 1f)] public float sonarSpeckle = 0.4f;
    [Range(0f, 0.2f)] public float sonarAbsorption = 0.025f;
    [Range(0f, 1f)] public float sonarThreshold = 0.015f;
    [Range(1, 32)] public int maxSonarEchoesPerRay = 8;
    [Range(0.01f, 0.5f)] public float sonarEchoWidthMeters = 0.08f;
    [Range(0.2f, 4f)] public float sonarEchoWidthSizeScale = 1.4f;
    [Range(0.02f, 1f)] public float sonarDebrisHitRadius = 0.18f;

    static readonly List<UnderwaterWorksiteDebrisField> ActiveFields = new List<UnderwaterWorksiteDebrisField>();

    readonly List<DebrisItem> items = new List<DebrisItem>();
    readonly List<Material> materials = new List<Material>();
    Mesh roundMesh;
    Mesh shardMesh;
    int builtSeed;
    int builtCount;

    class DebrisItem
    {
        public Transform transform;
        public Vector3 localPosition;
        public Vector3 baseLocalPosition;
        public Vector3 drift;
        public Vector3 spin;
        public float bobPhase;
        public float sonarRadius;
        public float sonarStrength;
        public float sonarEchoWidth;
    }

    void OnEnable()
    {
        if (!ActiveFields.Contains(this))
            ActiveFields.Add(this);

        if (!Application.isPlaying)
            RemoveSerializedDebrisChildren();

        RebuildIfNeeded();
    }

    void Update()
    {
        if (!debrisEnabled || debrisCount <= 0)
        {
            ClearDebris();
            return;
        }

        RebuildIfNeeded();

        if (Application.isPlaying && animateInPlayMode)
            AnimateDebris();
    }

    void OnValidate()
    {
        fieldSize = new Vector3(Mathf.Max(0.1f, fieldSize.x), Mathf.Max(0.1f, fieldSize.y), Mathf.Max(0.1f, fieldSize.z));
        sizeRange.x = Mathf.Max(0.001f, sizeRange.x);
        sizeRange.y = Mathf.Max(sizeRange.x, sizeRange.y);
        randomDriftSpeed.y = Mathf.Max(randomDriftSpeed.x, randomDriftSpeed.y);
        spinSpeedDeg.y = Mathf.Max(spinSpeedDeg.x, spinSpeedDeg.y);
    }

    void OnDisable()
    {
        ActiveFields.Remove(this);

        if (!Application.isPlaying)
            ClearDebris();
    }

    void OnDestroy()
    {
        ActiveFields.Remove(this);
        ClearDebris();
        DestroyRuntimeAssets();
    }

    public static void AppendSonarPointsForAll(
        List<SonarWaterColumnNoise.Point> points,
        Vector3 origin,
        Vector3 direction,
        float maxRangeMeters,
        float nearestSolidDistanceMeters,
        int rayIndex,
        int frameIndex)
    {
        if (points == null) return;

        float limit = Mathf.Clamp(
            float.IsPositiveInfinity(nearestSolidDistanceMeters) ? maxRangeMeters : nearestSolidDistanceMeters,
            0f,
            maxRangeMeters);

        for (int i = 0; i < ActiveFields.Count; i++)
        {
            UnderwaterWorksiteDebrisField field = ActiveFields[i];
            if (field == null || !field.debrisEnabled || !field.visibleToSonar) continue;

            field.AppendSonarPoints(points, origin, direction, limit, rayIndex, frameIndex);
        }
    }

    public static void ApplyToRangeAzimuthFrameForAll(
        VirtualSonarPingFrame frame,
        int beamIndex,
        Vector3 origin,
        Vector3 direction,
        float maxRangeMeters,
        float nearestSolidDistanceMeters,
        int frameIndex)
    {
        if (frame == null || frame.Intensities8 == null) return;

        float limit = Mathf.Clamp(
            float.IsPositiveInfinity(nearestSolidDistanceMeters) ? maxRangeMeters : nearestSolidDistanceMeters,
            0f,
            maxRangeMeters);

        for (int i = 0; i < ActiveFields.Count; i++)
        {
            UnderwaterWorksiteDebrisField field = ActiveFields[i];
            if (field == null || !field.debrisEnabled || !field.visibleToSonar) continue;

            field.WriteSonarEchoes(frame, beamIndex, origin, direction, limit, frameIndex);
        }
    }

    void AppendSonarPoints(
        List<SonarWaterColumnNoise.Point> points,
        Vector3 origin,
        Vector3 direction,
        float maxDistance,
        int rayIndex,
        int frameIndex)
    {
        if (sonarUsesIndividualDebris)
        {
            AppendIndividualDebrisSonarPoints(points, origin, direction, maxDistance, rayIndex, frameIndex);
            return;
        }

        if (!TryRaycastField(origin, direction, maxDistance, out float enter, out float exit)) return;

        float length = Mathf.Max(0f, exit - enter);
        int count = SampleSonarEchoCount(length, rayIndex, frameIndex);
        for (int i = 0; i < count; i++)
        {
            uint state = Hash((uint)(seed + rayIndex * 92821 + frameIndex * 68917 + i * 313));
            float distance = Mathf.Lerp(enter, exit, Next01(ref state));
            Vector3 point = origin + direction.normalized * distance;
            if (hideAboveWaterSurface && point.y > waterSurfaceY - waterSurfaceMargin) continue;

            float intensity = ComputeSonarIntensity(distance, ref state);
            if (intensity < sonarThreshold) continue;

            points.Add(new SonarWaterColumnNoise.Point
            {
                position = point,
                rangeMeters = distance,
                intensity01 = intensity,
                isFalseEcho = true,
            });
        }
    }

    void WriteSonarEchoes(
        VirtualSonarPingFrame frame,
        int beamIndex,
        Vector3 origin,
        Vector3 direction,
        float maxDistance,
        int frameIndex)
    {
        if (sonarUsesIndividualDebris)
        {
            WriteIndividualDebrisSonarEchoes(frame, beamIndex, origin, direction, maxDistance, frameIndex);
            return;
        }

        if (!TryRaycastField(origin, direction, maxDistance, out float enter, out float exit)) return;

        float length = Mathf.Max(0f, exit - enter);
        int count = SampleSonarEchoCount(length, beamIndex, frameIndex);
        float rangeResolution = Mathf.Max((float)frame.RangeResolutionMeters, 1e-5f);
        int halfBins = Mathf.Max(1, Mathf.RoundToInt(sonarEchoWidthMeters / rangeResolution));

        for (int i = 0; i < count; i++)
        {
            uint state = Hash((uint)(seed + beamIndex * 92821 + frameIndex * 68917 + i * 313));
            float distance = Mathf.Lerp(enter, exit, Next01(ref state));
            Vector3 point = origin + direction.normalized * distance;
            if (hideAboveWaterSurface && point.y > waterSurfaceY - waterSurfaceMargin) continue;

            float intensity = ComputeSonarIntensity(distance, ref state);
            if (intensity < sonarThreshold) continue;

            int centerBin = Mathf.Clamp(Mathf.RoundToInt(distance / rangeResolution) - 1, 0, frame.RangeCount - 1);
            for (int delta = -halfBins; delta <= halfBins; delta++)
            {
                int rangeIndex = centerBin + delta;
                if (rangeIndex < 0 || rangeIndex >= frame.RangeCount) continue;

                float rangeOffset = delta * rangeResolution;
                float sigma = Mathf.Max(rangeResolution * 0.5f, sonarEchoWidthMeters * 0.35f);
                float weight = Mathf.Exp(-0.5f * rangeOffset * rangeOffset / Mathf.Max(1e-6f, sigma * sigma));
                int add = Mathf.RoundToInt(255f * intensity * weight);
                int updated = frame.Intensities8[rangeIndex, beamIndex] + add;
                frame.Intensities8[rangeIndex, beamIndex] = (byte)Mathf.Clamp(updated, 0, 255);
            }
        }
    }

    void AppendIndividualDebrisSonarPoints(
        List<SonarWaterColumnNoise.Point> points,
        Vector3 origin,
        Vector3 direction,
        float maxDistance,
        int rayIndex,
        int frameIndex)
    {
        direction.Normalize();
        int emitted = 0;

        for (int i = 0; i < items.Count && emitted < maxSonarEchoesPerRay; i++)
        {
            DebrisItem item = items[i];
            if (item.transform == null) continue;

            Vector3 world = item.transform.position;
            if (hideAboveWaterSurface && world.y > waterSurfaceY - waterSurfaceMargin) continue;

            float t = Vector3.Dot(world - origin, direction);
            if (t <= 0f || t >= maxDistance) continue;

            Vector3 closest = origin + direction * t;
            float lateral = Vector3.Distance(world, closest);
            float hitRadius = Mathf.Max(sonarDebrisHitRadius, item.sonarRadius);
            if (lateral > hitRadius) continue;

            uint state = Hash((uint)(seed + rayIndex * 92821 + frameIndex * 68917 + i * 313));
            float intensity = ComputeIndividualSonarIntensity(t, lateral, hitRadius, item.sonarStrength, ref state);
            if (intensity < sonarThreshold) continue;

            points.Add(new SonarWaterColumnNoise.Point
            {
                position = closest,
                rangeMeters = t,
                intensity01 = intensity,
                sizeMultiplier = Mathf.Clamp(item.sonarEchoWidth / Mathf.Max(0.01f, sonarEchoWidthMeters), 0.75f, 3f),
                isFalseEcho = true,
            });
            emitted++;
        }
    }

    void WriteIndividualDebrisSonarEchoes(
        VirtualSonarPingFrame frame,
        int beamIndex,
        Vector3 origin,
        Vector3 direction,
        float maxDistance,
        int frameIndex)
    {
        direction.Normalize();
        float rangeResolution = Mathf.Max((float)frame.RangeResolutionMeters, 1e-5f);
        int emitted = 0;

        for (int i = 0; i < items.Count && emitted < maxSonarEchoesPerRay; i++)
        {
            DebrisItem item = items[i];
            if (item.transform == null) continue;

            Vector3 world = item.transform.position;
            if (hideAboveWaterSurface && world.y > waterSurfaceY - waterSurfaceMargin) continue;

            float t = Vector3.Dot(world - origin, direction);
            if (t <= 0f || t >= maxDistance) continue;

            Vector3 closest = origin + direction * t;
            float lateral = Vector3.Distance(world, closest);
            float hitRadius = Mathf.Max(sonarDebrisHitRadius, item.sonarRadius);
            if (lateral > hitRadius) continue;

            uint state = Hash((uint)(seed + beamIndex * 92821 + frameIndex * 68917 + i * 313));
            float intensity = ComputeIndividualSonarIntensity(t, lateral, hitRadius, item.sonarStrength, ref state);
            if (intensity < sonarThreshold) continue;

            WriteRangeAzimuthEcho(frame, beamIndex, t, intensity, rangeResolution, item.sonarEchoWidth);
            emitted++;
        }
    }

    void WriteRangeAzimuthEcho(VirtualSonarPingFrame frame, int beamIndex, float distance, float intensity, float rangeResolution, float echoWidthMeters)
    {
        float width = Mathf.Max(rangeResolution, echoWidthMeters);
        int halfBins = Mathf.Max(1, Mathf.RoundToInt(width / rangeResolution));
        int centerBin = Mathf.Clamp(Mathf.RoundToInt(distance / rangeResolution) - 1, 0, frame.RangeCount - 1);

        for (int delta = -halfBins; delta <= halfBins; delta++)
        {
            int rangeIndex = centerBin + delta;
            if (rangeIndex < 0 || rangeIndex >= frame.RangeCount) continue;

            float rangeOffset = delta * rangeResolution;
            float sigma = Mathf.Max(rangeResolution * 0.5f, width * 0.35f);
            float weight = Mathf.Exp(-0.5f * rangeOffset * rangeOffset / Mathf.Max(1e-6f, sigma * sigma));
            int add = Mathf.RoundToInt(255f * intensity * weight);
            int updated = frame.Intensities8[rangeIndex, beamIndex] + add;
            frame.Intensities8[rangeIndex, beamIndex] = (byte)Mathf.Clamp(updated, 0, 255);
        }
    }

    float ComputeIndividualSonarIntensity(float distance, float lateral, float hitRadius, float itemStrength, ref uint state)
    {
        float lateralWeight = Mathf.Clamp01(1f - lateral / Mathf.Max(0.001f, hitRadius));
        lateralWeight *= lateralWeight;
        float attenuation = Mathf.Exp(-sonarAbsorption * Mathf.Max(0f, distance));
        float speckle = Mathf.Lerp(1f - sonarSpeckle, 1f + sonarSpeckle, Next01(ref state));
        return Mathf.Clamp01(sonarReflectivity * itemStrength * lateralWeight * attenuation * speckle);
    }

    int SampleSonarEchoCount(float length, int rayIndex, int frameIndex)
    {
        if (length <= 0f || sonarEchoDensityPerMeter <= 0f || sonarReflectivity <= 0f) return 0;

        float expected = Mathf.Min(maxSonarEchoesPerRay, length * sonarEchoDensityPerMeter);
        uint state = Hash((uint)(seed + rayIndex * 73856093 + frameIndex * 19349663));
        int whole = Mathf.FloorToInt(expected);
        int count = whole;
        if (Next01(ref state) < expected - whole)
            count++;

        return Mathf.Clamp(count, 0, Mathf.Max(0, maxSonarEchoesPerRay));
    }

    float ComputeSonarIntensity(float distance, ref uint state)
    {
        float attenuation = Mathf.Exp(-sonarAbsorption * Mathf.Max(0f, distance));
        float speckle = Mathf.Lerp(1f - sonarSpeckle, 1f + sonarSpeckle, Next01(ref state));
        return Mathf.Clamp01(sonarReflectivity * attenuation * speckle);
    }

    bool TryRaycastField(Vector3 origin, Vector3 direction, float maxDistance, out float enter, out float exit)
    {
        direction.Normalize();
        enter = 0f;
        exit = 0f;

        if (fieldShape == FieldShape.Box)
            return TryRaycastBox(origin, direction, maxDistance, out enter, out exit);

        return TryRaycastEllipsoid(origin, direction, maxDistance, out enter, out exit);
    }

    bool TryRaycastBox(Vector3 origin, Vector3 direction, float maxDistance, out float enter, out float exit)
    {
        Vector3 o = transform.InverseTransformPoint(origin);
        Vector3 d = transform.InverseTransformDirection(direction);
        Vector3 half = HalfSize();

        enter = 0f;
        exit = maxDistance;

        if (!ClipSlab(o.x, d.x, -half.x, half.x, ref enter, ref exit)) return false;
        if (!ClipSlab(o.y, d.y, -half.y, half.y, ref enter, ref exit)) return false;
        if (!ClipSlab(o.z, d.z, -half.z, half.z, ref enter, ref exit)) return false;

        return exit >= 0f && enter <= maxDistance;
    }

    bool TryRaycastEllipsoid(Vector3 origin, Vector3 direction, float maxDistance, out float enter, out float exit)
    {
        Vector3 half = HalfSize();
        Vector3 o = transform.InverseTransformPoint(origin);
        Vector3 d = transform.InverseTransformDirection(direction);

        o = new Vector3(o.x / half.x, o.y / half.y, o.z / half.z);
        d = new Vector3(d.x / half.x, d.y / half.y, d.z / half.z);

        float a = Vector3.Dot(d, d);
        float b = 2f * Vector3.Dot(o, d);
        float c = Vector3.Dot(o, o) - 1f;
        float disc = b * b - 4f * a * c;

        enter = 0f;
        exit = 0f;

        if (disc < 0f || a <= 1e-6f) return false;

        float root = Mathf.Sqrt(disc);
        float t0 = (-b - root) / (2f * a);
        float t1 = (-b + root) / (2f * a);
        if (t1 < 0f || t0 > maxDistance) return false;

        enter = Mathf.Clamp(t0, 0f, maxDistance);
        exit = Mathf.Clamp(t1, 0f, maxDistance);
        return exit >= enter;
    }

    static bool ClipSlab(float origin, float direction, float min, float max, ref float enter, ref float exit)
    {
        if (Mathf.Abs(direction) < 1e-6f)
            return origin >= min && origin <= max;

        float t0 = (min - origin) / direction;
        float t1 = (max - origin) / direction;
        if (t0 > t1)
        {
            float tmp = t0;
            t0 = t1;
            t1 = tmp;
        }

        enter = Mathf.Max(enter, t0);
        exit = Mathf.Min(exit, t1);
        return enter <= exit;
    }

    [ContextMenu("Rebuild Debris")]
    public void ForceRebuild()
    {
        ClearDebris();
        if (!Application.isPlaying)
            RemoveSerializedDebrisChildren();
        RebuildIfNeeded(true);
    }

    void RemoveSerializedDebrisChildren()
    {
        for (int i = transform.childCount - 1; i >= 0; i--)
        {
            Transform child = transform.GetChild(i);
            if (child != null && child.name == "WorksiteDebris")
                DestroyImmediate(child.gameObject);
        }
    }

    void RebuildIfNeeded(bool force = false)
    {
        if (!debrisEnabled || debrisCount <= 0) return;
        if (!force && items.Count == debrisCount && builtSeed == seed && builtCount == debrisCount) return;

        ClearDebris();
        builtSeed = seed;
        builtCount = debrisCount;

        for (int i = 0; i < debrisCount; i++)
            items.Add(CreateDebris(i));
    }

    DebrisItem CreateDebris(int index)
    {
        uint state = Hash((uint)(seed + index * 1009));

        GameObject go = new GameObject("WorksiteDebris");
        if (!Application.isPlaying)
            go.hideFlags = HideFlags.HideAndDontSave;
        go.transform.SetParent(transform, false);

        bool plate = useRoundVisualDebris || Next01(ref state) < plateRatio;
        MeshFilter filter = go.AddComponent<MeshFilter>();
        filter.sharedMesh = plate ? GetRoundMesh() : GetShardMesh();

        MeshRenderer renderer = go.AddComponent<MeshRenderer>();
        renderer.sharedMaterial = CreateMaterial(ref state);
        renderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        renderer.receiveShadows = false;

        float s = Mathf.Lerp(sizeRange.x, sizeRange.y, Next01(ref state));
        float aspect = plate ? Mathf.Lerp(0.25f, 1.6f, Next01(ref state)) : Mathf.Lerp(0.45f, 1.2f, Next01(ref state));
        float visualSize = s * Mathf.Clamp(visualSizeMultiplier, 0.1f, 1f);
        go.transform.localScale = new Vector3(visualSize * aspect, visualSize, visualSize * 0.08f);

        Vector3 position = fieldShape == FieldShape.Box
            ? RandomPointInBox(ref state)
            : RandomPointInEllipsoid(ref state);

        DebrisItem item = new DebrisItem
        {
            transform = go.transform,
            localPosition = position,
            baseLocalPosition = position,
            drift = RandomUnitVector(ref state) * Mathf.Lerp(randomDriftSpeed.x, randomDriftSpeed.y, Next01(ref state)),
            spin = RandomUnitVector(ref state) * Mathf.Lerp(spinSpeedDeg.x, spinSpeedDeg.y, Next01(ref state)),
            bobPhase = Next01(ref state) * Mathf.PI * 2f,
            sonarRadius = Mathf.Max(0.02f, s * Mathf.Max(0.6f, aspect) * 0.9f),
            sonarStrength = Mathf.Lerp(0.45f, 1.15f, Next01(ref state)),
            sonarEchoWidth = Mathf.Clamp(
                sonarEchoWidthMeters * Mathf.Lerp(0.55f, sonarEchoWidthSizeScale, Mathf.InverseLerp(sizeRange.x, sizeRange.y, s)) * Mathf.Lerp(0.8f, 1.25f, Next01(ref state)),
                0.01f,
                1.5f),
        };

        item.transform.localPosition = ApplyWaterCull(position);
        item.transform.localRotation = RandomRotation(ref state);
        return item;
    }

    void AnimateDebris()
    {
        Vector3 half = HalfSize();
        float time = Time.time;
        Vector3 currentLocalVelocity = GetCurrentLocalVelocity();

        for (int i = 0; i < items.Count; i++)
        {
            DebrisItem item = items[i];
            if (item.transform == null) continue;

            item.localPosition += (currentLocalVelocity + item.drift) * Time.deltaTime;
            item.localPosition = fieldShape == FieldShape.Box
                ? WrapBox(item.localPosition, half)
                : WrapBox(item.localPosition, half);

            Vector3 animated = item.localPosition;
            animated.y += Mathf.Sin(time * bobFrequency * Mathf.PI * 2f + item.bobPhase) * bobAmplitude;

            item.transform.localPosition = ApplyWaterCull(animated);
            item.transform.Rotate(item.spin * Time.deltaTime, Space.Self);
        }
    }

    Vector3 GetCurrentLocalVelocity()
    {
        if (!currentVelocityIsWorldSpace)
            return currentVelocity;

        return transform.InverseTransformVector(currentVelocity);
    }

    Vector3 ApplyWaterCull(Vector3 local)
    {
        if (!hideAboveWaterSurface) return local;

        float worldY = transform.TransformPoint(local).y;
        if (worldY <= waterSurfaceY - waterSurfaceMargin) return local;

        float deltaWorld = worldY - (waterSurfaceY - waterSurfaceMargin);
        local.y -= deltaWorld / Mathf.Max(0.0001f, transform.lossyScale.y);
        return local;
    }

    Vector3 RandomPointInBox(ref uint state)
    {
        Vector3 half = HalfSize();
        return new Vector3(
            Mathf.Lerp(-half.x, half.x, Next01(ref state)),
            Mathf.Lerp(-half.y, half.y, Next01(ref state)),
            Mathf.Lerp(-half.z, half.z, Next01(ref state)));
    }

    Vector3 RandomPointInEllipsoid(ref uint state)
    {
        Vector3 half = HalfSize();
        Vector3 p;
        int guard = 0;
        do
        {
            p = RandomPointInBox(ref state);
            guard++;
        }
        while (guard < 16 &&
               (p.x * p.x) / (half.x * half.x) +
               (p.y * p.y) / (half.y * half.y) +
               (p.z * p.z) / (half.z * half.z) > 1f);

        return p;
    }

    Vector3 HalfSize()
    {
        return new Vector3(
            Mathf.Max(0.1f, fieldSize.x) * 0.5f,
            Mathf.Max(0.1f, fieldSize.y) * 0.5f,
            Mathf.Max(0.1f, fieldSize.z) * 0.5f);
    }

    static Vector3 WrapBox(Vector3 p, Vector3 half)
    {
        if (p.x < -half.x) p.x = half.x;
        if (p.x > half.x) p.x = -half.x;
        if (p.y < -half.y) p.y = half.y;
        if (p.y > half.y) p.y = -half.y;
        if (p.z < -half.z) p.z = half.z;
        if (p.z > half.z) p.z = -half.z;
        return p;
    }

    Material CreateMaterial(ref uint state)
    {
        Shader shader = Shader.Find("HDRP/Unlit");
        if (shader == null) shader = Shader.Find("Unlit/Transparent");
        if (shader == null) shader = Shader.Find("Sprites/Default");

        Material material = new Material(shader)
        {
            name = "Runtime Worksite Debris Material",
            hideFlags = HideFlags.DontSave
        };

        Color color = Color.Lerp(darkColor, lightColor, Next01(ref state));
        color.a = alpha * Mathf.Lerp(0.55f, 1f, Next01(ref state));

        if (material.HasProperty("_BaseColor")) material.SetColor("_BaseColor", color);
        if (material.HasProperty("_Color")) material.SetColor("_Color", color);
        if (material.HasProperty("_SurfaceType")) material.SetFloat("_SurfaceType", 1f);
        if (material.HasProperty("_Surface")) material.SetFloat("_Surface", 1f);
        if (material.HasProperty("_AlphaCutoffEnable")) material.SetFloat("_AlphaCutoffEnable", 0f);

        materials.Add(material);
        return material;
    }

    Mesh GetRoundMesh()
    {
        if (roundMesh != null) return roundMesh;

        const int segments = 18;
        Vector3[] vertices = new Vector3[segments + 1];
        Vector2[] uvs = new Vector2[segments + 1];
        int[] triangles = new int[segments * 3];

        vertices[0] = Vector3.zero;
        uvs[0] = new Vector2(0.5f, 0.5f);

        for (int i = 0; i < segments; i++)
        {
            float a = (Mathf.PI * 2f * i) / segments;
            float r = 0.5f * (0.86f + 0.10f * Mathf.Sin(i * 1.7f));
            float x = Mathf.Cos(a) * r;
            float y = Mathf.Sin(a) * r;
            vertices[i + 1] = new Vector3(x, y, 0f);
            uvs[i + 1] = new Vector2(x + 0.5f, y + 0.5f);
        }

        for (int i = 0; i < segments; i++)
        {
            int tri = i * 3;
            triangles[tri] = 0;
            triangles[tri + 1] = i + 1;
            triangles[tri + 2] = i == segments - 1 ? 1 : i + 2;
        }

        roundMesh = new Mesh { name = "Runtime Worksite Debris Round", hideFlags = HideFlags.DontSave };
        roundMesh.vertices = vertices;
        roundMesh.uv = uvs;
        roundMesh.triangles = triangles;
        roundMesh.RecalculateNormals();
        roundMesh.RecalculateBounds();
        return roundMesh;
    }

    Mesh GetShardMesh()
    {
        if (shardMesh != null) return shardMesh;

        shardMesh = new Mesh { name = "Runtime Worksite Debris Shard", hideFlags = HideFlags.DontSave };
        shardMesh.vertices = new[]
        {
            new Vector3(-0.46f, -0.22f, 0f),
            new Vector3(-0.18f, -0.42f, 0f),
            new Vector3(0.30f, -0.34f, 0f),
            new Vector3(0.50f, -0.04f, 0f),
            new Vector3(0.28f, 0.34f, 0f),
            new Vector3(-0.20f, 0.42f, 0f),
            new Vector3(-0.44f, 0.16f, 0f),
        };
        shardMesh.triangles = new[] { 0, 1, 2, 0, 2, 6, 6, 2, 5, 5, 2, 4, 4, 2, 3 };
        shardMesh.RecalculateNormals();
        shardMesh.RecalculateBounds();
        return shardMesh;
    }

    void ClearDebris()
    {
        for (int i = 0; i < items.Count; i++)
        {
            if (items[i].transform == null) continue;
            GameObject go = items[i].transform.gameObject;
            if (Application.isPlaying) Destroy(go);
            else DestroyImmediate(go);
        }

        items.Clear();
    }

    void DestroyRuntimeAssets()
    {
        for (int i = 0; i < materials.Count; i++)
        {
            if (materials[i] == null) continue;
            if (Application.isPlaying) Destroy(materials[i]);
            else DestroyImmediate(materials[i]);
        }
        materials.Clear();

        if (roundMesh != null)
        {
            Mesh mesh = roundMesh;
            roundMesh = null;
            if (Application.isPlaying) Destroy(mesh);
            else DestroyImmediate(mesh);
        }

        if (shardMesh != null)
        {
            Mesh mesh = shardMesh;
            shardMesh = null;
            if (Application.isPlaying) Destroy(mesh);
            else DestroyImmediate(mesh);
        }
    }

    static Quaternion RandomRotation(ref uint state)
    {
        return Quaternion.Euler(Next01(ref state) * 360f, Next01(ref state) * 360f, Next01(ref state) * 360f);
    }

    static Vector3 RandomUnitVector(ref uint state)
    {
        Vector3 v = new Vector3(
            Next01(ref state) * 2f - 1f,
            Next01(ref state) * 2f - 1f,
            Next01(ref state) * 2f - 1f);
        return v.sqrMagnitude < 0.0001f ? Vector3.forward : v.normalized;
    }

    static uint Hash(uint x)
    {
        x ^= x >> 16;
        x *= 0x7feb352du;
        x ^= x >> 15;
        x *= 0x846ca68bu;
        x ^= x >> 16;
        return x == 0 ? 1u : x;
    }

    static float Next01(ref uint state)
    {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        return (state & 0x00ffffff) / 16777216f;
    }
}
