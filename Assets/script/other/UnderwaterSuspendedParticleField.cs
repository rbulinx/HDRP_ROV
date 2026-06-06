using UnityEngine;

[DisallowMultipleComponent]
public class UnderwaterSuspendedParticleField : MonoBehaviour
{
    public enum VolumeShape
    {
        Ellipsoid,
        Box,
    }

    public bool particlesEnabled = true;
    [Range(0, 10000)] public int maxParticles = 1200;
    [Range(0f, 2000f)] public float emissionRate = 140f;
    public VolumeShape volumeShape = VolumeShape.Ellipsoid;
    public Vector3 boxSize = new Vector3(9f, 5f, 12f);
    public Vector3 localOffset = new Vector3(0f, 0f, 5f);
    public Vector2 lifetime = new Vector2(8f, 18f);
    public Vector2 size = new Vector2(0.006f, 0.024f);
    public Vector2 speed = new Vector2(0.002f, 0.018f);
    [Range(0f, 1f)] public float alpha = 0.32f;
    public bool onlyBelowWaterSurface = true;
    public float waterSurfaceY = 0f;
    public float waterSurfaceMargin = 0.15f;
    public Vector3 driftVelocity = new Vector3(0.003f, 0.001f, -0.01f);

    ParticleSystem particleSystemInstance;
    Material runtimeMaterial;
    Texture2D runtimeParticleTexture;

    void OnEnable()
    {
        UpdateParticles();
    }

    void Update()
    {
        UpdateParticles();
    }

    void OnDisable()
    {
        if (particleSystemInstance != null)
            particleSystemInstance.Stop(true, ParticleSystemStopBehavior.StopEmittingAndClear);
    }

    void OnDestroy()
    {
        DestroyParticles();
    }

    public void SetParticlesEnabled(bool value)
    {
        particlesEnabled = value;
        UpdateParticles();
    }

    void UpdateParticles()
    {
        if (!Application.isPlaying) return;

        bool underwater = !onlyBelowWaterSurface || transform.position.y < waterSurfaceY - waterSurfaceMargin;
        if (!underwater || !particlesEnabled || maxParticles <= 0 || emissionRate <= 0f)
        {
            DestroyParticles();
            return;
        }

        EnsureParticles();
        ConfigureParticles();

        if (!particleSystemInstance.isPlaying)
            particleSystemInstance.Play();
    }

    void EnsureParticles()
    {
        if (particleSystemInstance != null) return;

        GameObject go = new GameObject("UnderwaterSuspendedParticles");
        go.transform.SetParent(transform, false);
        go.transform.localPosition = localOffset;
        go.transform.localRotation = Quaternion.identity;
        go.transform.localScale = Vector3.one;

        particleSystemInstance = go.AddComponent<ParticleSystem>();
        ParticleSystemRenderer renderer = go.GetComponent<ParticleSystemRenderer>();
        renderer.renderMode = ParticleSystemRenderMode.Billboard;
        renderer.alignment = ParticleSystemRenderSpace.View;
        renderer.material = GetRuntimeMaterial();
    }

    void ConfigureParticles()
    {
        if (particleSystemInstance == null) return;

        Transform psTransform = particleSystemInstance.transform;
        psTransform.localPosition = localOffset;
        psTransform.localRotation = Quaternion.identity;
        psTransform.localScale = Vector3.one;

        var main = particleSystemInstance.main;
        main.loop = true;
        main.playOnAwake = true;
        main.simulationSpace = ParticleSystemSimulationSpace.World;
        main.maxParticles = Mathf.Max(1, maxParticles);
        main.gravityModifier = 0f;
        main.startLifetime = new ParticleSystem.MinMaxCurve(Mathf.Max(0.1f, lifetime.x), Mathf.Max(0.1f, lifetime.y));
        main.startSize = new ParticleSystem.MinMaxCurve(Mathf.Max(0.001f, size.x), Mathf.Max(0.001f, size.y));
        main.startSpeed = new ParticleSystem.MinMaxCurve(Mathf.Max(0f, speed.x), Mathf.Max(0f, speed.y));
        main.startColor = new ParticleSystem.MinMaxGradient(
            new Color(0.62f, 0.70f, 0.66f, alpha * 0.45f),
            new Color(1f, 0.96f, 0.82f, alpha));

        var emission = particleSystemInstance.emission;
        emission.enabled = true;
        emission.rateOverTime = Mathf.Max(0f, emissionRate);

        var shape = particleSystemInstance.shape;
        shape.enabled = true;
        shape.shapeType = volumeShape == VolumeShape.Ellipsoid
            ? ParticleSystemShapeType.Sphere
            : ParticleSystemShapeType.Box;
        shape.scale = new Vector3(Mathf.Max(0.1f, boxSize.x), Mathf.Max(0.1f, boxSize.y), Mathf.Max(0.1f, boxSize.z));

        var velocity = particleSystemInstance.velocityOverLifetime;
        velocity.enabled = true;
        velocity.space = ParticleSystemSimulationSpace.World;
        velocity.x = new ParticleSystem.MinMaxCurve(driftVelocity.x - 0.006f, driftVelocity.x + 0.006f);
        velocity.y = new ParticleSystem.MinMaxCurve(driftVelocity.y - 0.003f, driftVelocity.y + 0.003f);
        velocity.z = new ParticleSystem.MinMaxCurve(driftVelocity.z - 0.006f, driftVelocity.z + 0.006f);

        var noise = particleSystemInstance.noise;
        noise.enabled = true;
        noise.strength = 0.025f;
        noise.frequency = 0.08f;
        noise.scrollSpeed = 0.015f;

        var colorOverLifetime = particleSystemInstance.colorOverLifetime;
        colorOverLifetime.enabled = true;
        Gradient gradient = new Gradient();
        gradient.SetKeys(
            new[]
            {
                new GradientColorKey(new Color(0.62f, 0.70f, 0.66f), 0f),
                new GradientColorKey(new Color(1f, 0.96f, 0.82f), 0.55f),
                new GradientColorKey(new Color(0.50f, 0.58f, 0.54f), 1f),
            },
            new[]
            {
                new GradientAlphaKey(0f, 0f),
                new GradientAlphaKey(alpha, 0.18f),
                new GradientAlphaKey(alpha * 0.65f, 0.75f),
                new GradientAlphaKey(0f, 1f),
            });
        colorOverLifetime.color = gradient;

        ParticleSystemRenderer renderer = particleSystemInstance.GetComponent<ParticleSystemRenderer>();
        if (renderer != null)
        {
            renderer.renderMode = ParticleSystemRenderMode.Billboard;
            renderer.alignment = ParticleSystemRenderSpace.View;
            renderer.material = GetRuntimeMaterial();
        }
    }

    Material GetRuntimeMaterial()
    {
        if (runtimeMaterial != null) return runtimeMaterial;

        Shader shader = Shader.Find("Particles/Standard Unlit");
        if (shader == null) shader = Shader.Find("Sprites/Default");
        if (shader == null) shader = Shader.Find("Unlit/Color");

        runtimeMaterial = new Material(shader)
        {
            name = "Runtime Underwater Suspended Particle Material",
            hideFlags = HideFlags.DontSave
        };

        if (runtimeMaterial.HasProperty("_Color"))
            runtimeMaterial.SetColor("_Color", Color.white);
        if (runtimeMaterial.HasProperty("_BaseColor"))
            runtimeMaterial.SetColor("_BaseColor", Color.white);
        if (runtimeMaterial.HasProperty("_MainTex"))
            runtimeMaterial.SetTexture("_MainTex", GetRuntimeParticleTexture());
        if (runtimeMaterial.HasProperty("_BaseMap"))
            runtimeMaterial.SetTexture("_BaseMap", GetRuntimeParticleTexture());
        if (runtimeMaterial.HasProperty("_Surface"))
            runtimeMaterial.SetFloat("_Surface", 1f);
        if (runtimeMaterial.HasProperty("_Blend"))
            runtimeMaterial.SetFloat("_Blend", 0f);

        return runtimeMaterial;
    }

    Texture2D GetRuntimeParticleTexture()
    {
        if (runtimeParticleTexture != null) return runtimeParticleTexture;

        const int sizePx = 32;
        runtimeParticleTexture = new Texture2D(sizePx, sizePx, TextureFormat.RGBA32, false)
        {
            name = "Runtime Underwater Round Particle Texture",
            hideFlags = HideFlags.DontSave,
            wrapMode = TextureWrapMode.Clamp,
            filterMode = FilterMode.Bilinear,
        };

        Color[] pixels = new Color[sizePx * sizePx];
        float center = (sizePx - 1) * 0.5f;
        for (int y = 0; y < sizePx; y++)
        {
            for (int x = 0; x < sizePx; x++)
            {
                float dx = (x - center) / center;
                float dy = (y - center) / center;
                float d = Mathf.Sqrt(dx * dx + dy * dy);
                float a = Mathf.Clamp01(1f - Mathf.SmoothStep(0.58f, 1f, d));
                pixels[y * sizePx + x] = new Color(1f, 1f, 1f, a);
            }
        }

        runtimeParticleTexture.SetPixels(pixels);
        runtimeParticleTexture.Apply(false, true);
        return runtimeParticleTexture;
    }

    void DestroyParticles()
    {
        if (particleSystemInstance != null)
        {
            GameObject go = particleSystemInstance.gameObject;
            particleSystemInstance = null;
            if (Application.isPlaying) Destroy(go);
            else DestroyImmediate(go);
        }

        if (runtimeMaterial != null)
        {
            Material material = runtimeMaterial;
            runtimeMaterial = null;
            if (Application.isPlaying) Destroy(material);
            else DestroyImmediate(material);
        }

        if (runtimeParticleTexture != null)
        {
            Texture2D texture = runtimeParticleTexture;
            runtimeParticleTexture = null;
            if (Application.isPlaying) Destroy(texture);
            else DestroyImmediate(texture);
        }
    }
}
