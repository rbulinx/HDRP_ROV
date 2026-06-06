using UnityEngine;

[DisallowMultipleComponent]
[AddComponentMenu("ROV/Light Emitter")]
public class ROVLightEmitter : MonoBehaviour
{
    public Renderer targetRenderer;
    public Color emissiveColor = new Color(0.86f, 0.97f, 1f, 1f);
    public float minEmissionIntensityNits = 0f;
    public float maxEmissionIntensityNits = 5000f;
    public bool hideRendererWhenOff = false;
    public bool useMaterialInstance = true;

    MaterialPropertyBlock propertyBlock;
    Material runtimeMaterial;
    static readonly int EmissiveColorId = Shader.PropertyToID("_EmissiveColor");
    static readonly int EmissiveColorLdrId = Shader.PropertyToID("_EmissiveColorLDR");
    static readonly int LegacyEmissionColorId = Shader.PropertyToID("_EmissionColor");
    static readonly int EmissiveIntensityId = Shader.PropertyToID("_EmissiveIntensity");
    static readonly int UseEmissiveIntensityId = Shader.PropertyToID("_UseEmissiveIntensity");

    void Reset()
    {
        FindTargetRendererIfNeeded();
    }

    void Awake()
    {
        FindTargetRendererIfNeeded();
        CacheMaterialInstanceIfNeeded();
    }

    public void ApplyLightLevel(float level01)
    {
        FindTargetRendererIfNeeded();
        if (targetRenderer == null) return;

        level01 = Mathf.Clamp01(level01);
        float intensity = Mathf.Lerp(minEmissionIntensityNits, maxEmissionIntensityNits, level01);
        Color finalColor = emissiveColor * intensity;
        finalColor.a = emissiveColor.a;

        if (useMaterialInstance)
        {
            CacheMaterialInstanceIfNeeded();
            ApplyToMaterial(runtimeMaterial, intensity, finalColor);
        }

        if (propertyBlock == null)
            propertyBlock = new MaterialPropertyBlock();

        targetRenderer.GetPropertyBlock(propertyBlock);
        propertyBlock.SetColor(EmissiveColorId, finalColor);
        propertyBlock.SetColor(EmissiveColorLdrId, emissiveColor);
        propertyBlock.SetColor(LegacyEmissionColorId, finalColor);
        propertyBlock.SetFloat(EmissiveIntensityId, intensity);
        propertyBlock.SetFloat(UseEmissiveIntensityId, 1f);
        targetRenderer.SetPropertyBlock(propertyBlock);

        if (hideRendererWhenOff)
            targetRenderer.enabled = level01 > 0.001f;
    }

    void FindTargetRendererIfNeeded()
    {
        if (targetRenderer != null) return;

        targetRenderer = GetComponent<Renderer>();
        if (targetRenderer == null)
            targetRenderer = GetComponentInChildren<Renderer>(true);
    }

    void CacheMaterialInstanceIfNeeded()
    {
        if (!useMaterialInstance || targetRenderer == null || runtimeMaterial != null) return;
        runtimeMaterial = targetRenderer.material;
    }

    void ApplyToMaterial(Material material, float intensity, Color finalColor)
    {
        if (material == null) return;

        material.EnableKeyword("_EMISSION");
        if (material.HasProperty(UseEmissiveIntensityId)) material.SetFloat(UseEmissiveIntensityId, 1f);
        if (material.HasProperty(EmissiveColorId)) material.SetColor(EmissiveColorId, finalColor);
        if (material.HasProperty(EmissiveColorLdrId)) material.SetColor(EmissiveColorLdrId, emissiveColor);
        if (material.HasProperty(LegacyEmissionColorId)) material.SetColor(LegacyEmissionColorId, finalColor);
        if (material.HasProperty(EmissiveIntensityId)) material.SetFloat(EmissiveIntensityId, intensity);
        material.globalIlluminationFlags = MaterialGlobalIlluminationFlags.None;
    }
}
