using UnityEngine;
using UnityEngine.UI;

[ExecuteAlways]
public class ImagingSonarSim : MonoBehaviour
{
    [Header("Fan Mask / Outline")]
    public bool drawFanMask = true;

    [Tooltip("扇形の外側をどれだけ暗くするか (1=真っ黒, 0=何もしない)")]
    [Range(0f, 1f)] public float outsideDarken = 1.0f;

    [Tooltip("扇形の輪郭の明るさ(0..1)")]
    [Range(0f, 1f)] public float fanOutlineIntensity01 = 0.15f;

    [Range(1, 6)] public int fanOutlineThickness = 2;
    
    [Header("Output")]
    public RawImage targetImage;

    [Header("Beam / FOV")]
    [Range(10f, 180f)] public float horizontalFovDeg = 130f;
    [Range(1, 2048)] public int beams = 512;

    [Tooltip("上下方向に何本サンプルするか（垂直開口の近似）。3〜5推奨。")]
    [Range(1, 20)] public int elevSamples = 10;
    [Range(1f, 120f)] public float verticalFovDeg = 20f;

    [Header("Range / Rate")]
    public float maxRangeMeters = 40f;
    [Range(1f, 60f)] public float scanHz = 15f;

    [Header("Range UI (optional)")]
    public Button range2mButton;
    public Button range5mButton;
    public Button range10mButton;
    public Button range40mButton;
    public bool autoDetectRangeButtons = true;
    public Color rangeButtonNormalColor = Color.white;
    public Color rangeButtonSelectedColor = new Color(0.1f, 0.7f, 0.3f, 1f);

    [Header("Map")]
    [Range(128, 2048)] public int mapResolution = 512;
    [Range(0.5f, 5f)] public float mapRadiusScale = 1.0f;
    [Range(1, 9)] public int pointSize = 2;

    [Tooltip("残像：0=毎回クリア、0.7〜0.9=残す")]
    [Range(0f, 0.98f)] public float persistence = 0.0f;

    [Header("Map Origin (0..1 in texture)")]
    [Range(0f, 1f)] public float originX01 = 0.5f;
    [Range(0f, 1f)] public float originY01 = 0.30f;

    [Header("Physics")]
    public LayerMask obstacleMask = ~0;
    public QueryTriggerInteraction triggerInteraction = QueryTriggerInteraction.Ignore;

    [Header("Noise / Background")]
    [Range(0f, 1f)] public float background = 0.04f;
    [Range(0f, 1f)] public float speckle = 0.10f;

    [Header("Water-column false echoes")]
    public bool enableWaterColumnNoise = true;
    public SonarWaterColumnNoise waterColumnNoise = new SonarWaterColumnNoise();

    [Header("Visible suspended particles")]
    public bool enableVisibleSuspendedParticles = false;
    [Range(0, 10000)] public int visibleParticleMaxCount = 1200;
    [Range(0f, 2000f)] public float visibleParticleEmissionRate = 140f;
    public Vector3 visibleParticleBoxSize = new Vector3(8f, 4f, 10f);
    public Vector3 visibleParticleLocalOffset = new Vector3(0f, 0f, 4f);
    public Vector2 visibleParticleLifetime = new Vector2(8f, 18f);
    public Vector2 visibleParticleSize = new Vector2(0.012f, 0.045f);
    public Vector2 visibleParticleSpeed = new Vector2(0.01f, 0.08f);
    [Range(0f, 1f)] public float visibleParticleAlpha = 0.32f;

    [Header("Range Attenuation")]
    [Tooltip("0=なし, 2=1/r^2相当")]
    [Range(0f, 4f)] public float spreadingPower = 2.0f;
    [Tooltip("吸収 exp(-a r) の a [1/m]")]
    [Range(0f, 0.2f)] public float absorptionPerMeter = 0.03f;
    [Range(0f, 10f)] public float gain = 2.0f;
    [Range(0.0f, 5.0f)] public float minRangeMeters = 0.5f;

    [Header("Angle response (front vs grazing)")]
    [Range(0.5f, 8f)] public float anglePower = 2.0f;
    [Range(0.5f, 8f)] public float grazingPower = 2.0f;
    [Range(0f, 1f)] public float grazingMix = 0.6f;
    [Range(0f, 0.2f)] public float backscatterFloor = 0.02f;

    [Header("Specular (optional)")]
    [Range(0f, 1f)] public float specular = 0.10f;
    [Range(1f, 64f)] public float specPower = 16f;

    [Header("Return shaping")]
    [Range(1f, 4f)] public float firstReturnBoost = 1.4f;

    [Header("Front-most priority (fix near hidden by far)")]
    public bool frontMostWins = true;
    [Range(0f, 0.2f)] public float depthEpsilonMeters = 0.02f;

    [Header("Vertical FOV in top-down")]
    public bool useHorizontalRange = true;

    [Header("Acoustic Shadow")]
    public bool enableShadow = true;
    [Range(0f, 1f)] public float shadowStrength = 0.6f;
    [Range(1, 4)] public int shadowStepPix = 1;

    [Header("Color")]
    public bool useColorMap = true;
    public Color lowColor = new Color(0.02f, 0.02f, 0.03f, 1f);     // 暗部（青黒）
    public Color highColor = new Color(1.0f, 0.55f, 0.0f, 1f);      // 明部（オレンジ）

    [Header("Hit visibility")]
    [Range(0f, 1f)] public float minHitIntensity01 = 0.15f; // まず0.1〜0.2

    // ---- internal ----
    Texture2D mapTex;
    Color32[] mapPixels;
    float[] depthBuf;         // pixel depth (meters)
    byte[] intensityBuf;      // pixel brightness 0..255 (for comparisons)
    Color32[] lut256;         // intensity -> color

    Vector3[] beamDirsLocal;
    float[] sinTheta;
    float[] cosTheta;
    float[] elevAnglesDeg;
    float[] elevWeights;

    float tAcc;
    System.Random rng;
    bool rangeButtonsSearched;
    int sonarFrameIndex;
    readonly System.Collections.Generic.List<SonarWaterColumnNoise.Point> waterNoisePoints =
        new System.Collections.Generic.List<SonarWaterColumnNoise.Point>(16);
    ParticleSystem visibleSuspendedParticles;
    Material visibleParticleMaterial;

    void OnEnable()
    {
        EnsureInit();
        SyncRangeButtonVisual();
        RenderOnce();
    }

    void OnValidate()
    {
        EnsureInit();
        SyncRangeButtonVisual();
        RenderOnce();
    }

    void Update()
    {
        if (!Application.isPlaying) return;

        UpdateVisibleSuspendedParticles();

        tAcc += Time.deltaTime;
        float period = 1f / Mathf.Max(1f, scanHz);
        if (tAcc >= period)
        {
            tAcc -= period;
            RenderOnce();
        }
    }

    void OnDisable()
    {
        DestroyVisibleSuspendedParticles();
    }

    void EnsureInit()
    {
        if (rng == null) rng = new System.Random(12345);

        beams = Mathf.Clamp(beams, 1, 2048);
        elevSamples = Mathf.Clamp(elevSamples, 1, 20);
        mapResolution = Mathf.Clamp(mapResolution, 128, 2048);
        maxRangeMeters = Mathf.Max(0.1f, maxRangeMeters);

        int N = mapResolution * mapResolution;

        if (mapTex == null || mapTex.width != mapResolution || mapTex.height != mapResolution)
        {
            mapTex = new Texture2D(mapResolution, mapResolution, TextureFormat.RGBA32, false, true);
            mapTex.wrapMode = TextureWrapMode.Clamp;
            mapTex.filterMode = FilterMode.Point;

            mapPixels = new Color32[N];
            depthBuf = new float[N];
            intensityBuf = new byte[N];
        }
        else
        {
            if (mapPixels == null || mapPixels.Length != N) mapPixels = new Color32[N];
            if (depthBuf == null || depthBuf.Length != N) depthBuf = new float[N];
            if (intensityBuf == null || intensityBuf.Length != N) intensityBuf = new byte[N];
        }

        BuildLUT();
        PrecomputeDirsAndWeights();

        if (targetImage != null && targetImage.texture != mapTex)
            targetImage.texture = mapTex;
    }

    void BuildLUT()
    {
        if (lut256 == null || lut256.Length != 256) lut256 = new Color32[256];

        for (int i = 0; i < 256; i++)
        {
            float t = i / 255f;
            Color c = useColorMap ? Color.Lerp(lowColor, highColor, t) : new Color(t, t, t, 1f);
            lut256[i] = (Color32)c;
        }
    }

    void PrecomputeDirsAndWeights()
    {
        beamDirsLocal = new Vector3[beams];
        sinTheta = new float[beams];
        cosTheta = new float[beams];

        for (int i = 0; i < beams; i++)
        {
            float u = (beams == 1) ? 0.5f : (float)i / (beams - 1);
            float yawDeg = Mathf.Lerp(-horizontalFovDeg * 0.5f, horizontalFovDeg * 0.5f, u);
            float theta = yawDeg * Mathf.Deg2Rad;

            sinTheta[i] = Mathf.Sin(theta);
            cosTheta[i] = Mathf.Cos(theta);

            beamDirsLocal[i] = Quaternion.Euler(0f, yawDeg, 0f) * Vector3.forward;
        }

        elevAnglesDeg = new float[elevSamples];
        elevWeights = new float[elevSamples];

        if (elevSamples == 1)
        {
            elevAnglesDeg[0] = 0f;
            elevWeights[0] = 1f;
        }
        else
        {
            for (int j = 0; j < elevSamples; j++)
            {
                float v = (float)j / (elevSamples - 1);
                elevAnglesDeg[j] = Mathf.Lerp(-verticalFovDeg * 0.5f, verticalFovDeg * 0.5f, v);
            }

            // 中央強め（ガウス風）
            float center = (elevSamples - 1) * 0.5f;
            float sigma = Mathf.Max(0.5f, elevSamples * 0.35f);
            float sum = 0f;
            for (int j = 0; j < elevSamples; j++)
            {
                float x = (j - center) / sigma;
                float w = Mathf.Exp(-0.5f * x * x);
                elevWeights[j] = w;
                sum += w;
            }
            if (sum > 0f)
            {
                for (int j = 0; j < elevSamples; j++)
                    elevWeights[j] /= sum;
            }
        }
    }

    public void SetMaxRangeMeters(float meters)
    {
        ApplyRangeMeters(meters);
    }

    public void SetRange2m()
    {
        ApplyRangeMeters(2f);
    }

    public void SetRange5m()
    {
        ApplyRangeMeters(5f);
    }

    public void SetRange10m()
    {
        ApplyRangeMeters(10f);
    }

    public void SetRange40m()
    {
        ApplyRangeMeters(40f);
    }

    void ApplyRangeMeters(float meters)
    {
        maxRangeMeters = Mathf.Max(0.1f, meters);
        EnsureInit();
        SyncRangeButtonVisual();
        RenderOnce();
    }

    void SyncRangeButtonVisual()
    {
        CacheRangeButtonsIfNeeded();

        const float eps = 0.01f;
        bool is2m = Mathf.Abs(maxRangeMeters - 2f) <= eps;
        bool is5m = Mathf.Abs(maxRangeMeters - 5f) <= eps;
        bool is10m = Mathf.Abs(maxRangeMeters - 10f) <= eps;
        bool is40m = Mathf.Abs(maxRangeMeters - 40f) <= eps;

        SetRangeButtonState(range2mButton, is2m);
        SetRangeButtonState(range5mButton, is5m);
        SetRangeButtonState(range10mButton, is10m);
        SetRangeButtonState(range40mButton, is40m);
    }

    void CacheRangeButtonsIfNeeded()
    {
        if (!autoDetectRangeButtons) return;
        if (rangeButtonsSearched) return;
        rangeButtonsSearched = true;

        Button[] buttons = FindObjectsByType<Button>(FindObjectsInactive.Include, FindObjectsSortMode.None);
        for (int b = 0; b < buttons.Length; b++)
        {
            Button button = buttons[b];
            if (button == null) continue;

            int calls = button.onClick.GetPersistentEventCount();
            for (int i = 0; i < calls; i++)
            {
                if (button.onClick.GetPersistentTarget(i) != this) continue;

                string method = button.onClick.GetPersistentMethodName(i);
                if (method == nameof(SetRange2m) && range2mButton == null) range2mButton = button;
                else if (method == nameof(SetRange5m) && range5mButton == null) range5mButton = button;
                else if (method == nameof(SetRange10m) && range10mButton == null) range10mButton = button;
                else if (method == nameof(SetRange40m) && range40mButton == null) range40mButton = button;
            }
        }
    }

    void SetRangeButtonState(Button button, bool selected)
    {
        if (button == null) return;

        Image image = button.targetGraphic as Image;
        if (image != null) image.color = selected ? rangeButtonSelectedColor : rangeButtonNormalColor;

        ColorBlock colors = button.colors;
        Color baseColor = selected ? rangeButtonSelectedColor : rangeButtonNormalColor;
        colors.normalColor = baseColor;
        colors.highlightedColor = Color.Lerp(baseColor, Color.white, 0.18f);
        colors.pressedColor = Color.Lerp(baseColor, Color.black, 0.20f);
        colors.selectedColor = colors.highlightedColor;
        button.colors = colors;
    }


    // =========================
    // Render
    // =========================
    void RenderOnce()
    {
        if (mapTex == null || mapPixels == null || depthBuf == null || intensityBuf == null) return;
        if (rng == null) rng = new System.Random(12345);

        int R = mapTex.width;
        int N = mapPixels.Length;

        // 原点（ソナー位置）をテクスチャ内で配置
        int cx = Mathf.RoundToInt((R - 1) * originX01);
        int cy = Mathf.RoundToInt((R - 1) * originY01);

        // はみ出さない最大半径
        int margin = 2;
        int maxRadiusPix = Mathf.Min(cx, (R - 1) - cx, cy, (R - 1) - cy) - margin;
        maxRadiusPix = Mathf.Max(1, maxRadiusPix);
        float radiusPix = maxRadiusPix * mapRadiusScale;

        // depth buffer reset（手前優先のため）
        for (int k = 0; k < N; k++) depthBuf[k] = float.PositiveInfinity;

        // 背景更新（逆転しない残像：飽和しないように背景へ戻す）
        if (persistence <= 0f)
            FillBackgroundNoise(mapPixels, intensityBuf, background, speckle, rng, lut256);
        else
            FadeTowardBackground(mapPixels, intensityBuf, persistence, background, speckle, rng, lut256);

        // 扇形マスク（扇形外を暗くする）
        if (drawFanMask)
        {
            float halfFovRad = 0.5f * horizontalFovDeg * Mathf.Deg2Rad;
            ApplyFanMask(mapPixels, intensityBuf, lut256, R, cx, cy, radiusPix, halfFovRad, outsideDarken);
        }

        Vector3 origin = transform.position;
        Quaternion rot = transform.rotation;
        int frameIndex = sonarFrameIndex++;

        // ====== A方式：各レイで「最初に当たった点だけ」を描く（当たらないところは何もしない） ======
        // 影は「当たらない＝描かれない」ことで自然に背景のままになります。

        for (int i = 0; i < beams; i++)
        {
            Vector3 baseDirLocal = beamDirsLocal[i];
            float nearestSolidDistance = float.PositiveInfinity;

            for (int j = 0; j < elevSamples; j++)
            {
                float wElev = elevWeights[j];
                if (wElev <= 0f) continue;

                Quaternion elevRot = Quaternion.Euler(elevAnglesDeg[j], 0f, 0f);
                Vector3 dirWorld = rot * (elevRot * baseDirLocal);
                Vector3 dirN = dirWorld.normalized;

                if (!Physics.Raycast(origin, dirN, out RaycastHit hit, maxRangeMeters, obstacleMask, triggerInteraction))
                    continue;

                float dist = hit.distance;
                if (dist <= 0f) continue;
                if (dist < nearestSolidDistance) nearestSolidDistance = dist;

                // ---- 角度応答（正面＋斜め）----
                float nDotV = Mathf.Clamp01(Vector3.Dot(hit.normal, -dirN));
                float face = Mathf.Pow(nDotV, anglePower);
                float graze = Mathf.Pow(1.0f - nDotV, grazingPower);
                float backscatter = Mathf.Lerp(face, graze, grazingMix);
                backscatter = Mathf.Clamp01(backscatter + backscatterFloor);

                // 鏡面（任意）
                float spec = 0f;
                if (specular > 0f)
                {
                    Vector3 reflDir = Vector3.Reflect(dirN, hit.normal).normalized;
                    float rDotV = Mathf.Clamp01(Vector3.Dot(reflDir, -dirN));
                    spec = specular * Mathf.Pow(rDotV, specPower);
                }

                // ---- 距離減衰（拡散×吸収）----
                float rangeM = Mathf.Max(dist, minRangeMeters);
                float spread = (spreadingPower <= 0f) ? 1.0f : (1.0f / Mathf.Pow(rangeM, spreadingPower));
                float absorb = (absorptionPerMeter <= 0f) ? 1.0f : Mathf.Exp(-absorptionPerMeter * dist);
                float att = gain * spread * absorb;

                float intensity = Mathf.Clamp01(att * backscatter + spec);

                // 垂直重み（開口の中心を少し強める）
                intensity *= wElev;

                // 当たった点は最低輝度を保証
                intensity = Mathf.Max(intensity, minHitIntensity01);

                // ---- 投影半径（トップダウン）----
                float distForMap = dist;
                if (useHorizontalRange)
                {
                    float elevRad = elevAnglesDeg[j] * Mathf.Deg2Rad;
                    distForMap = dist * Mathf.Cos(elevRad);
                }

                float r01 = Mathf.Clamp01(distForMap / maxRangeMeters);
                float rPix = radiusPix * r01;

                float px = cx + sinTheta[i] * rPix;
                float py = cy + cosTheta[i] * rPix;

                byte intenByte = Float01ToByte(intensity);

                DrawPointFront(
                    mapPixels, intensityBuf, depthBuf, lut256, R,
                    Mathf.RoundToInt(px), Mathf.RoundToInt(py),
                    dist, intenByte, pointSize,
                    frontMostWins, depthEpsilonMeters
                );

                // --- Simple acoustic shadow: darken behind the first-hit point for this ray ---
                if (enableShadow)
                {
                    DarkenAlongRayBackgroundOnly(
                        mapPixels, intensityBuf, depthBuf, lut256, R,
                        cx, cy, sinTheta[i], cosTheta[i],
                        dist, rPix, radiusPix,
                        shadowStrength * wElev, shadowStepPix, depthEpsilonMeters
                    );
                }

            }

            if (enableWaterColumnNoise && waterColumnNoise != null && waterColumnNoise.enabled)
            {
                waterNoisePoints.Clear();
                Vector3 beamDirWorld = (rot * baseDirLocal).normalized;
                waterColumnNoise.AppendFalsePoints(
                    waterNoisePoints,
                    origin,
                    beamDirWorld,
                    maxRangeMeters,
                    nearestSolidDistance,
                    i,
                    frameIndex);

                for (int n = 0; n < waterNoisePoints.Count; n++)
                {
                    SonarWaterColumnNoise.Point p = waterNoisePoints[n];
                    float r01 = Mathf.Clamp01(p.rangeMeters / maxRangeMeters);
                    float rPix = radiusPix * r01;
                    int px = Mathf.RoundToInt(cx + sinTheta[i] * rPix);
                    int py = Mathf.RoundToInt(cy + cosTheta[i] * rPix);
                    int drawSize = Mathf.Max(1, Mathf.RoundToInt(pointSize * Mathf.Max(0.5f, p.sizeMultiplier)));
                    AddPointAdditive(mapPixels, intensityBuf, lut256, R, px, py, Float01ToByte(p.intensity01), drawSize);
                }
            }

            waterNoisePoints.Clear();
            Vector3 worksiteBeamDirWorld = (rot * baseDirLocal).normalized;
            UnderwaterWorksiteDebrisField.AppendSonarPointsForAll(
                waterNoisePoints,
                origin,
                worksiteBeamDirWorld,
                maxRangeMeters,
                nearestSolidDistance,
                i,
                frameIndex);

            for (int n = 0; n < waterNoisePoints.Count; n++)
            {
                SonarWaterColumnNoise.Point p = waterNoisePoints[n];
                float r01 = Mathf.Clamp01(p.rangeMeters / maxRangeMeters);
                float rPix = radiusPix * r01;
                int px = Mathf.RoundToInt(cx + sinTheta[i] * rPix);
                int py = Mathf.RoundToInt(cy + cosTheta[i] * rPix);
                int drawSize = Mathf.Max(1, Mathf.RoundToInt(pointSize * Mathf.Max(0.5f, p.sizeMultiplier)));
                AddPointAdditive(mapPixels, intensityBuf, lut256, R, px, py, Float01ToByte(p.intensity01), drawSize);
            }
        }

        // 扇形の輪郭

        if (drawFanMask && fanOutlineIntensity01 > 0f)
        {
            float halfFovRad = 0.5f * horizontalFovDeg * Mathf.Deg2Rad;
            DrawFanOutline(mapPixels, intensityBuf, lut256, R, cx, cy, radiusPix, halfFovRad,
                Float01ToByte(fanOutlineIntensity01), fanOutlineThickness);
        }

        mapTex.SetPixels32(mapPixels);
        mapTex.Apply(false, false);

        if (targetImage != null && targetImage.texture != mapTex)
            targetImage.texture = mapTex;
    }

    // =========================
    // Helpers
    // =========================
    static byte Float01ToByte(float v01)
    {
        v01 = Mathf.Clamp01(v01);
        return (byte)Mathf.RoundToInt(v01 * 255f);
    }
    static void FillBackgroundNoise(Color32[] pix, byte[] inten, float bg, float sp, System.Random rng, Color32[] lut)
    {
        int n = pix.Length;
        for (int k = 0; k < n; k++)
        {
            float n01 = (float)rng.NextDouble();
            float v = Mathf.Clamp01(bg + (n01 - 0.5f) * 2f * sp * bg);
            byte b = Float01ToByte(v);
            inten[k] = b;
            pix[k] = lut[b];
        }
    }

    public void SetWaterColumnNoiseEnabled(bool value)
    {
        enableWaterColumnNoise = value;
        if (waterColumnNoise != null) waterColumnNoise.enabled = value;
        RenderOnce();
    }

    public void ToggleWaterColumnNoise()
    {
        SetWaterColumnNoiseEnabled(!enableWaterColumnNoise);
    }

    public void SetVisibleSuspendedParticlesEnabled(bool value)
    {
        enableVisibleSuspendedParticles = value;
        UpdateVisibleSuspendedParticles();
    }

    public void ToggleVisibleSuspendedParticles()
    {
        SetVisibleSuspendedParticlesEnabled(!enableVisibleSuspendedParticles);
    }

    void UpdateVisibleSuspendedParticles()
    {
        if (!Application.isPlaying) return;

        if (!enableVisibleSuspendedParticles || visibleParticleMaxCount <= 0 || visibleParticleEmissionRate <= 0f)
        {
            DestroyVisibleSuspendedParticles();
            return;
        }

        EnsureVisibleSuspendedParticles();
        ConfigureVisibleSuspendedParticles();

        if (!visibleSuspendedParticles.isPlaying)
            visibleSuspendedParticles.Play();
    }

    void EnsureVisibleSuspendedParticles()
    {
        if (visibleSuspendedParticles != null) return;

        GameObject go = new GameObject("VisibleSuspendedParticles");
        go.transform.SetParent(transform, false);
        go.transform.localPosition = visibleParticleLocalOffset;
        go.transform.localRotation = Quaternion.identity;
        go.transform.localScale = Vector3.one;

        visibleSuspendedParticles = go.AddComponent<ParticleSystem>();
        ParticleSystemRenderer renderer = go.GetComponent<ParticleSystemRenderer>();
        renderer.renderMode = ParticleSystemRenderMode.Billboard;
        renderer.alignment = ParticleSystemRenderSpace.View;
        renderer.material = GetVisibleParticleMaterial();
    }

    void ConfigureVisibleSuspendedParticles()
    {
        if (visibleSuspendedParticles == null) return;

        Transform psTransform = visibleSuspendedParticles.transform;
        psTransform.localPosition = visibleParticleLocalOffset;
        psTransform.localRotation = Quaternion.identity;
        psTransform.localScale = Vector3.one;

        var main = visibleSuspendedParticles.main;
        main.loop = true;
        main.playOnAwake = true;
        main.simulationSpace = ParticleSystemSimulationSpace.World;
        main.maxParticles = Mathf.Max(1, visibleParticleMaxCount);
        main.gravityModifier = 0f;
        main.startLifetime = new ParticleSystem.MinMaxCurve(
            Mathf.Max(0.1f, visibleParticleLifetime.x),
            Mathf.Max(0.1f, visibleParticleLifetime.y));
        main.startSize = new ParticleSystem.MinMaxCurve(
            Mathf.Max(0.001f, visibleParticleSize.x),
            Mathf.Max(0.001f, visibleParticleSize.y));
        main.startSpeed = new ParticleSystem.MinMaxCurve(
            Mathf.Max(0f, visibleParticleSpeed.x),
            Mathf.Max(0f, visibleParticleSpeed.y));
        main.startColor = new ParticleSystem.MinMaxGradient(
            new Color(0.65f, 0.72f, 0.68f, visibleParticleAlpha * 0.45f),
            new Color(1f, 1f, 0.9f, visibleParticleAlpha));

        var emission = visibleSuspendedParticles.emission;
        emission.enabled = true;
        emission.rateOverTime = Mathf.Max(0f, visibleParticleEmissionRate);

        var shape = visibleSuspendedParticles.shape;
        shape.enabled = true;
        shape.shapeType = ParticleSystemShapeType.Box;
        shape.scale = new Vector3(
            Mathf.Max(0.1f, visibleParticleBoxSize.x),
            Mathf.Max(0.1f, visibleParticleBoxSize.y),
            Mathf.Max(0.1f, visibleParticleBoxSize.z));

        var velocity = visibleSuspendedParticles.velocityOverLifetime;
        velocity.enabled = true;
        velocity.space = ParticleSystemSimulationSpace.World;
        velocity.x = new ParticleSystem.MinMaxCurve(-0.025f, 0.025f);
        velocity.y = new ParticleSystem.MinMaxCurve(-0.01f, 0.015f);
        velocity.z = new ParticleSystem.MinMaxCurve(-0.05f, -0.01f);

        var noise = visibleSuspendedParticles.noise;
        noise.enabled = true;
        noise.strength = 0.08f;
        noise.frequency = 0.18f;
        noise.scrollSpeed = 0.05f;

        var colorOverLifetime = visibleSuspendedParticles.colorOverLifetime;
        colorOverLifetime.enabled = true;
        Gradient gradient = new Gradient();
        gradient.SetKeys(
            new[]
            {
                new GradientColorKey(new Color(0.65f, 0.72f, 0.68f), 0f),
                new GradientColorKey(new Color(1f, 0.96f, 0.82f), 0.55f),
                new GradientColorKey(new Color(0.55f, 0.62f, 0.58f), 1f),
            },
            new[]
            {
                new GradientAlphaKey(0f, 0f),
                new GradientAlphaKey(visibleParticleAlpha, 0.18f),
                new GradientAlphaKey(visibleParticleAlpha * 0.65f, 0.75f),
                new GradientAlphaKey(0f, 1f),
            });
        colorOverLifetime.color = gradient;

        ParticleSystemRenderer renderer = visibleSuspendedParticles.GetComponent<ParticleSystemRenderer>();
        if (renderer != null)
        {
            renderer.renderMode = ParticleSystemRenderMode.Billboard;
            renderer.alignment = ParticleSystemRenderSpace.View;
            renderer.material = GetVisibleParticleMaterial();
        }
    }

    Material GetVisibleParticleMaterial()
    {
        if (visibleParticleMaterial != null) return visibleParticleMaterial;

        Shader shader = Shader.Find("Particles/Standard Unlit");
        if (shader == null) shader = Shader.Find("Sprites/Default");
        if (shader == null) shader = Shader.Find("Unlit/Color");

        visibleParticleMaterial = new Material(shader)
        {
            name = "Runtime Visible Suspended Particle Material",
            hideFlags = HideFlags.DontSave
        };

        if (visibleParticleMaterial.HasProperty("_Color"))
            visibleParticleMaterial.SetColor("_Color", Color.white);
        if (visibleParticleMaterial.HasProperty("_BaseColor"))
            visibleParticleMaterial.SetColor("_BaseColor", Color.white);
        if (visibleParticleMaterial.HasProperty("_Surface"))
            visibleParticleMaterial.SetFloat("_Surface", 1f);
        if (visibleParticleMaterial.HasProperty("_Blend"))
            visibleParticleMaterial.SetFloat("_Blend", 0f);

        return visibleParticleMaterial;
    }

    void DestroyVisibleSuspendedParticles()
    {
        if (visibleSuspendedParticles != null)
        {
            GameObject go = visibleSuspendedParticles.gameObject;
            visibleSuspendedParticles = null;
            if (Application.isPlaying) Destroy(go);
            else DestroyImmediate(go);
        }

        if (visibleParticleMaterial != null)
        {
            Material material = visibleParticleMaterial;
            visibleParticleMaterial = null;
            if (Application.isPlaying) Destroy(material);
            else DestroyImmediate(material);
        }
    }


    static void FadeTowardBackground(Color32[] pix, byte[] inten, float persistence, float bg, float sp, System.Random rng, Color32[] lut)
    {
        float keep = Mathf.Clamp01(persistence);
        float toBg = 1.0f - keep;

        int n = pix.Length;
        for (int k = 0; k < n; k++)
        {
            float cur = inten[k] / 255f;

            float n01 = (float)rng.NextDouble();
            float bgv = Mathf.Clamp01(bg + (n01 - 0.5f) * 2f * sp * bg);

            float v = cur * keep + bgv * toBg;
            byte b = Float01ToByte(v);

            inten[k] = b;
            pix[k] = lut[b];
        }
    }

    static void DrawPointFront(
        Color32[] pix, byte[] inten, float[] depth, Color32[] lut, int R,
        int cx, int cy,
        float distM, byte newInten, int size,
        bool frontMostWins, float epsM)
    {
        int half = Mathf.Max(0, size / 2);

        for (int dy = -half; dy <= half; dy++)
        {
            int y = cy + dy;
            if ((uint)y >= (uint)R) continue;

            for (int dx = -half; dx <= half; dx++)
            {
                int x = cx + dx;
                if ((uint)x >= (uint)R) continue;

                int idx = y * R + x;

                if (frontMostWins)
                {
                    float z = depth[idx];

                    // 手前なら採用
                    if (distM + epsM < z)
                    {
                        // 深度は手前で更新（重要）
                        depth[idx] = distM;

                        // ただし明るさは暗くしない（既存より明るい時だけ更新）
                        if (newInten > inten[idx])
                        {
                            inten[idx] = newInten;
                            pix[idx] = lut[newInten];
                        }
                    }
                    // ほぼ同距離なら明るい方
                    else if (Mathf.Abs(distM - z) <= epsM)
                    {
                        if (newInten > inten[idx])
                        {
                            inten[idx] = newInten;
                            pix[idx] = lut[newInten];
                        }
                    }
                    // 奥は捨てる
                }
                else
                {
                    if (newInten > inten[idx])
                    {
                        inten[idx] = newInten;
                        pix[idx] = lut[newInten];
                    }
                }
            }
        }
    }

    static void AddPointAdditive(Color32[] pix, byte[] inten, Color32[] lut, int R, int cx, int cy, byte addInten, int size)
    {
        if (addInten == 0) return;

        int half = Mathf.Max(0, size / 2);
        for (int dy = -half; dy <= half; dy++)
        {
            int y = cy + dy;
            if ((uint)y >= (uint)R) continue;

            for (int dx = -half; dx <= half; dx++)
            {
                int x = cx + dx;
                if ((uint)x >= (uint)R) continue;

                int idx = y * R + x;
                int updated = inten[idx] + addInten;
                byte v = (byte)Mathf.Clamp(updated, 0, 255);
                inten[idx] = v;
                pix[idx] = lut[v];
            }
        }
    }

    static void DarkenAlongRayBackgroundOnly(
        Color32[] pix, byte[] inten, float[] depth, Color32[] lut, int R,
        int ox, int oy, float sinT, float cosT,
        float firstDistM, float firstPix, float endPix,
        float strength, int stepPix, float epsM)
    {
        if (endPix <= firstPix) return;

        int step = Mathf.Max(1, stepPix);
        float s = Mathf.Clamp01(strength);
        float keep = 1.0f - s;

        // firstPixの少し先から
        for (float rp = firstPix + step; rp <= endPix; rp += step)
        {
            int x = ox + Mathf.RoundToInt(sinT * rp);
            int y = oy + Mathf.RoundToInt(cosT * rp);
            if ((uint)x >= (uint)R || (uint)y >= (uint)R) continue;

            int idx = y * R + x;

            // 近距離のエコーが既にあるところは暗くしない
            float z = depth[idx];
            if (!float.IsInfinity(z) && z <= firstDistM + epsM)
                continue;

            byte cur = inten[idx];
            byte darker = (byte)Mathf.RoundToInt(cur * keep);
            if (darker < cur)
            {
                inten[idx] = darker;
                pix[idx] = lut[darker];
            }
        }
    }

        static void ApplyFanMask(Color32[] pix, byte[] inten, Color32[] lut, int R,
        int ox, int oy, float radiusPix, float halfFovRad, float outsideDarken01)
    {
        float r2Max = radiusPix * radiusPix;
        float darkKeep = 1f - Mathf.Clamp01(outsideDarken01); // 0=真っ黒, 1=そのまま

        for (int y = 0; y < R; y++)
        {
            int row = y * R;
            float dy = y - oy; // forward方向成分（画像では +Y が上だが、ここは座標差分で扱う）
            for (int x = 0; x < R; x++)
            {
                float dx = x - ox; // 右方向

                float r2 = dx * dx + dy * dy;
                bool inside = true;

                if (r2 > r2Max) inside = false;
                else
                {
                    // 角度：atan2(dx, dy) で「上方向（+dy）」が0度、右が+角
                    float ang = Mathf.Atan2(dx, dy);
                    if (Mathf.Abs(ang) > halfFovRad) inside = false;

                    // 原点より“後ろ”（dy<=0）は扇形外にする（必要なら）
                    if (dy <= 0f) inside = false;
                }

                if (!inside)
                {
                    int idx = row + x;
                    byte cur = inten[idx];
                    byte v = (byte)Mathf.RoundToInt(cur * darkKeep);
                    inten[idx] = v;
                    pix[idx] = lut[v];
                }
            }
        }
    }

    static void DrawFanOutline(Color32[] pix, byte[] inten, Color32[] lut, int R,
        int ox, int oy, float radiusPix, float halfFovRad, byte v, int thickness)
    {
        thickness = Mathf.Clamp(thickness, 1, 6);

        // 左右の境界線
        DrawPolarLine(pix, inten, lut, R, ox, oy, radiusPix, -halfFovRad, v, thickness);
        DrawPolarLine(pix, inten, lut, R, ox, oy, radiusPix, +halfFovRad, v, thickness);

        // 外周の弧（最大レンジ）
        DrawArc(pix, inten, lut, R, ox, oy, radiusPix, -halfFovRad, +halfFovRad, v, thickness);

        // 原点マーク（小さな点）
        DrawDot(pix, inten, lut, R, ox, oy, v, thickness);
    }

    static void DrawPolarLine(Color32[] pix, byte[] inten, Color32[] lut, int R,
        int ox, int oy, float radiusPix, float angRad, byte v, int thickness)
    {
        float s = Mathf.Sin(angRad);
        float c = Mathf.Cos(angRad);

        int steps = Mathf.Max(1, Mathf.RoundToInt(radiusPix));
        for (int i = 0; i <= steps; i++)
        {
            float r = (i / (float)steps) * radiusPix;
            int x = ox + Mathf.RoundToInt(s * r);
            int y = oy + Mathf.RoundToInt(c * r);
            DrawDot(pix, inten, lut, R, x, y, v, thickness);
        }
    }

    static void DrawArc(Color32[] pix, byte[] inten, Color32[] lut, int R,
        int ox, int oy, float radiusPix, float a0, float a1, byte v, int thickness)
    {
        // 角度刻み：解像度に応じて
        int steps = Mathf.Clamp(Mathf.RoundToInt(Mathf.Abs(a1 - a0) * radiusPix), 64, 4096);
        for (int i = 0; i <= steps; i++)
        {
            float t = i / (float)steps;
            float a = Mathf.Lerp(a0, a1, t);
            float s = Mathf.Sin(a);
            float c = Mathf.Cos(a);

            int x = ox + Mathf.RoundToInt(s * radiusPix);
            int y = oy + Mathf.RoundToInt(c * radiusPix);
            DrawDot(pix, inten, lut, R, x, y, v, thickness);
        }
    }

    static void DrawDot(Color32[] pix, byte[] inten, Color32[] lut, int R,
        int cx, int cy, byte v, int thickness)
    {
        int half = Mathf.Max(0, thickness / 2);
        for (int dy = -half; dy <= half; dy++)
        {
            int y = cy + dy;
            if ((uint)y >= (uint)R) continue;

            for (int dx = -half; dx <= half; dx++)
            {
                int x = cx + dx;
                if ((uint)x >= (uint)R) continue;

                int idx = y * R + x;
                if (v > inten[idx])
                {
                    inten[idx] = v;
                    pix[idx] = lut[v];
                }
            }
        }
    }

}
