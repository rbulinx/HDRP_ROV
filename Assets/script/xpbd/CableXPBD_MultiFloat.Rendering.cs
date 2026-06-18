using UnityEngine;

public partial class CableXPBD_MultiFloat
{
    void UpdateLineRenderer()
    {
        if (!lr || x == null) return;
        if (!IsCableStateFinite())
        {
            lr.enabled = false;
            return;
        }

        lr.enabled = renderLine;
        if (lr.positionCount != x.Length) lr.positionCount = x.Length;
        ApplyInspectionCableLineStyleIfNeeded();
        lr.SetPositions(x);
    }

    void ApplyInspectionCableLineStyleIfNeeded()
    {
        if (!applyInspectionCableLineStyle || lr == null)
            return;

        float width = Mathf.Max(0.001f, inspectionCableLineWidth);
        lr.widthMultiplier = width;
        lr.startWidth = width;
        lr.endWidth = width;
        lr.startColor = inspectionCableLineColor;
        lr.endColor = inspectionCableLineColor;
        lr.numCapVertices = Mathf.Max(lr.numCapVertices, 4);
        lr.numCornerVertices = Mathf.Max(lr.numCornerVertices, 4);

        Material material = inspectionCableLineMaterial != null
            ? inspectionCableLineMaterial
            : GetRuntimeLineMaterial();

        if (material != null)
        {
            SetMaterialColor(material, inspectionCableLineColor);
            lr.sharedMaterial = material;
        }
    }

    Material GetRuntimeLineMaterial()
    {
        if (runtimeLineMaterial != null)
        {
            SetMaterialColor(runtimeLineMaterial, inspectionCableLineColor);
            return runtimeLineMaterial;
        }

        Shader shader = Shader.Find("HDRP/Unlit");
        if (shader == null) shader = Shader.Find("Universal Render Pipeline/Unlit");
        if (shader == null) shader = Shader.Find("Unlit/Color");
        if (shader == null) shader = Shader.Find("Sprites/Default");
        if (shader == null) return null;

        runtimeLineMaterial = new Material(shader)
        {
            name = "Runtime Inspection Cable Line Material",
            hideFlags = HideFlags.DontSave
        };
        SetMaterialColor(runtimeLineMaterial, inspectionCableLineColor);
        return runtimeLineMaterial;
    }

    void DestroyRuntimeLineMaterial()
    {
        if (runtimeLineMaterial == null)
            return;

        Material material = runtimeLineMaterial;
        runtimeLineMaterial = null;

#if UNITY_EDITOR
        DestroyImmediate(material);
#else
        Destroy(material);
#endif
    }
}
