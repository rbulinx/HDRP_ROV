using UnityEngine;

public partial class CableXPBD_MultiFloat
{
    bool ShouldUpdateSonarColliders()
    {
        if (!enableCableSonarColliders && !enableFloatSonarColliders)
            return false;

        int stride = Mathf.Max(1, sonarColliderUpdateStride);
        sonarColliderUpdateTick = (sonarColliderUpdateTick + 1) % stride;
        return sonarColliderUpdateTick == 0;
    }

    void UpdateFloatVisuals(bool updateSonarColliders = true)
    {
        if (!enableFloatSections || floatInstances.Count == 0)
            return;

        for (int i = 0; i < floatInstances.Count; i++)
        {
            FloatInstance instance = floatInstances[i];
            if (instance == null || instance.visual == null || instance.section == null)
                continue;

            SampleCableState(
                instance.normalizedPosition,
                out Vector3 position,
                out Vector3 tangent,
                out _,
                out _,
                out _,
                out _);

            FloatSection section = instance.section;
            instance.visual.position = position;

            Vector3 axis = tangent.sqrMagnitude > 1e-12f ? tangent.normalized : Vector3.up;
            instance.visual.rotation = Quaternion.FromToRotation(Vector3.up, axis);

            float scale = Mathf.Max(0.01f, section.visualScale);
            float diameter = Mathf.Max(0.001f, section.diameter) * scale;
            float length = Mathf.Max(0.001f, section.length) * scale;
            instance.visual.localScale = new Vector3(diameter, length * 0.5f, diameter);

            SetMaterialColor(instance.runtimeMaterial, section.color);
            if (updateSonarColliders)
                EnsureFloatSonarCollider(instance);
        }
    }

    void UpdateCableSonarColliders()
    {
        if (!enableCableSonarColliders || x == null || x.Length < 2)
        {
            SetCableSonarCollidersEnabled(false);
            return;
        }

        int segmentCount = x.Length - 1;
        EnsureCableSonarColliderCount(segmentCount);

        float radius = Mathf.Max(0.001f, cableSonarRadius);
        bool isTrigger = cableSonarCollidersAreTrigger;
        int layer = MaskToSingleLayer(cableSonarLayer);

        for (int i = 0; i < segmentCount; i++)
        {
            Transform tf = cableSonarTransforms[i];
            CapsuleCollider col = cableSonarColliders[i];
            if (tf == null || col == null)
                continue;

            Vector3 p0 = x[i];
            Vector3 p1 = x[i + 1];
            Vector3 d = p1 - p0;
            float length = d.magnitude;

            if (!IsFinite(p0) || !IsFinite(p1) || length < 1e-6f)
            {
                col.enabled = false;
                continue;
            }

            Vector3 dir = d / length;
            float overlap = Mathf.Max(0f, cableSonarEndOverlap);
            Vector3 center = (p0 + p1) * 0.5f;
            float colliderLength = length + overlap * 2f;

            tf.position = center;
            tf.rotation = Quaternion.LookRotation(dir, Vector3.up);
            if (layer >= 0)
                tf.gameObject.layer = layer;

            MeshCollider meshCol = GetCableSonarMeshCollider(i);
            if (useCableMeshSonarColliders && !isTrigger && meshCol != null)
            {
                tf.localScale = new Vector3(radius * 2f, colliderLength * 0.5f, radius * 2f);
                tf.rotation = Quaternion.FromToRotation(Vector3.up, dir);
                meshCol.enabled = true;
                col.enabled = false;
            }
            else
            {
                tf.localScale = Vector3.one;
                tf.rotation = Quaternion.LookRotation(dir, Vector3.up);
                col.enabled = true;
                col.direction = 2;
                col.isTrigger = isTrigger;
                col.radius = radius;
                col.height = Mathf.Max(radius * 2f, colliderLength + radius * 2f);
                col.center = Vector3.zero;

                if (meshCol != null)
                    meshCol.enabled = false;
            }
        }
    }

    void EnsureCableSonarColliderCount(int segmentCount)
    {
        segmentCount = Mathf.Max(0, segmentCount);
        if (IsCableSonarColliderCacheValid(segmentCount))
            return;

        ClearCableSonarColliders();

        cableSonarTransforms = new Transform[segmentCount];
        cableSonarColliders = new CapsuleCollider[segmentCount];
        cableSonarMeshColliders = useCableMeshSonarColliders ? new MeshCollider[segmentCount] : null;

        if (segmentCount == 0)
            return;

        if (cableSonarRoot == null)
        {
            GameObject root = new GameObject("Cable Sonar Colliders");
            root.transform.SetParent(transform, false);
            root.hideFlags = HideFlags.None;
            cableSonarRoot = root.transform;
        }

        int layer = MaskToSingleLayer(cableSonarLayer);
        for (int i = 0; i < segmentCount; i++)
        {
            GameObject go = new GameObject($"CableSonarSeg_{i}");
            go.name = $"CableSonarSeg_{i}";
            go.hideFlags = HideFlags.None;
            go.transform.SetParent(cableSonarRoot, false);
            if (layer >= 0)
                go.layer = layer;

            CapsuleCollider col = go.AddComponent<CapsuleCollider>();
            col.direction = 2;
            col.isTrigger = cableSonarCollidersAreTrigger;
            col.radius = Mathf.Max(0.001f, cableSonarRadius);
            col.height = col.radius * 2f;

            MeshCollider meshCollider = null;
            if (useCableMeshSonarColliders)
            {
                meshCollider = go.AddComponent<MeshCollider>();
                meshCollider.sharedMesh = GetSharedCylinderMeshForSonar();
                meshCollider.convex = false;
                meshCollider.isTrigger = false;
                meshCollider.enabled = !cableSonarCollidersAreTrigger;
                col.enabled = !meshCollider.enabled;
            }

            cableSonarTransforms[i] = go.transform;
            cableSonarColliders[i] = col;
            if (cableSonarMeshColliders != null)
                cableSonarMeshColliders[i] = meshCollider;
        }
    }

    bool IsCableSonarColliderCacheValid(int segmentCount)
    {
        if (cableSonarTransforms == null || cableSonarColliders == null)
            return false;

        if (cableSonarTransforms.Length != segmentCount || cableSonarColliders.Length != segmentCount)
            return false;

        if (useCableMeshSonarColliders)
        {
            if (cableSonarMeshColliders == null || cableSonarMeshColliders.Length != segmentCount)
                return false;
        }
        else if (cableSonarMeshColliders != null && cableSonarMeshColliders.Length != segmentCount)
        {
            return false;
        }

        for (int i = 0; i < segmentCount; i++)
        {
            if (cableSonarTransforms[i] == null || cableSonarColliders[i] == null)
                return false;

            if (useCableMeshSonarColliders && cableSonarMeshColliders[i] == null)
                return false;
        }

        return true;
    }

    MeshCollider GetCableSonarMeshCollider(int index)
    {
        if (cableSonarMeshColliders == null)
            return null;

        if (index < 0 || index >= cableSonarMeshColliders.Length)
            return null;

        return cableSonarMeshColliders[index];
    }

    void SetCableSonarCollidersEnabled(bool enabled)
    {
        if (cableSonarColliders == null)
            return;

        for (int i = 0; i < cableSonarColliders.Length; i++)
        {
            if (cableSonarColliders[i] != null)
                cableSonarColliders[i].enabled = enabled;
        }
    }

    void ClearCableSonarColliders()
    {
        if (cableSonarRoot != null)
        {
#if UNITY_EDITOR
            DestroyImmediate(cableSonarRoot.gameObject);
#else
            Destroy(cableSonarRoot.gameObject);
#endif
        }

        cableSonarRoot = null;
        cableSonarTransforms = null;
        cableSonarColliders = null;
        cableSonarMeshColliders = null;
    }

    public void ClearGeneratedSonarColliders()
    {
        for (int i = 0; i < floatInstances.Count; i++)
            DestroyFloatSonarColliders(floatInstances[i]);

        ClearCableSonarColliders();
    }

    public void RebuildGeneratedFloatVisuals()
    {
        RebuildFloatInstances();
    }

    static Mesh GetSharedCylinderMeshForSonar()
    {
        if (sharedCylinderMeshForSonar != null)
            return sharedCylinderMeshForSonar;

        GameObject temp = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        MeshFilter meshFilter = temp.GetComponent<MeshFilter>();
        if (meshFilter != null)
            sharedCylinderMeshForSonar = meshFilter.sharedMesh;

#if UNITY_EDITOR
        DestroyImmediate(temp);
#else
        Destroy(temp);
#endif

        return sharedCylinderMeshForSonar;
    }
}
