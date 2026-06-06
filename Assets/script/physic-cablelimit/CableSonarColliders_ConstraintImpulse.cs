using UnityEngine;

public class CableSonarColliders_ConstraintImpulse : MonoBehaviour
{
    [Header("Source Cable")]
    public CableLMM_UnderwaterWinch_Collision_ConstraintImpulse cable;

    [Header("Collider Settings")]
    public bool isTrigger = true;
    public float radius = 0.02f;
    public LayerMask sonarLayer = 0;
    public bool matchCableRadius = true;

    [Header("Performance")]
    public bool rebuildEveryStart = true;

    Transform[] segTf;
    CapsuleCollider[] segCol;

    void Reset()
    {
        cable = GetComponent<CableLMM_UnderwaterWinch_Collision_ConstraintImpulse>();
    }

    void Start()
    {
        if (!cable) cable = GetComponent<CableLMM_UnderwaterWinch_Collision_ConstraintImpulse>();
        if (rebuildEveryStart) Rebuild();
    }

    void OnValidate()
    {
        if (!cable) cable = GetComponent<CableLMM_UnderwaterWinch_Collision_ConstraintImpulse>();
    }

    public void Rebuild()
    {
        if (!cable) return;

        if (segTf != null)
        {
            for (int i = 0; i < segTf.Length; i++)
            {
                if (segTf[i])
                {
#if UNITY_EDITOR
                    DestroyImmediate(segTf[i].gameObject);
#else
                    Destroy(segTf[i].gameObject);
#endif
                }
            }
        }

        int n = cable.GetNodeCount();
        int segCount = Mathf.Max(1, n - 1);

        segTf = new Transform[segCount];
        segCol = new CapsuleCollider[segCount];

        for (int i = 0; i < segCount; i++)
        {
            var go = new GameObject($"CableSegCol_ConstraintImpulse_{i}");
            go.transform.SetParent(transform, false);

            if (sonarLayer.value != 0)
            {
                int layer = MaskToSingleLayer(sonarLayer);
                if (layer >= 0) go.layer = layer;
            }

            var cc = go.AddComponent<CapsuleCollider>();
            cc.direction = 2; // Z
            cc.isTrigger = isTrigger;
            cc.radius = Mathf.Max(1e-4f, radius);
            cc.height = cc.radius * 2f;

            segTf[i] = go.transform;
            segCol[i] = cc;
        }
    }

    void FixedUpdate()
    {
        if (!cable) return;

        int n = cable.GetNodeCount();
        int segCount = Mathf.Max(1, n - 1);

        if (segTf == null || segTf.Length != segCount)
        {
            Rebuild();
            if (segTf == null || segTf.Length != segCount) return;
        }

        float r = matchCableRadius ? Mathf.Max(1e-4f, cable.nodeRadius) : Mathf.Max(1e-4f, radius);

        for (int i = 0; i < segCount; i++)
        {
            Vector3 p0 = cable.GetNodePosition(i);
            Vector3 p1 = cable.GetNodePosition(i + 1);
            Vector3 d = (p1 - p0);
            float len = d.magnitude;
            if (len < 1e-6f) continue;

            Transform t = segTf[i];
            t.position = (p0 + p1) * 0.5f;
            t.rotation = Quaternion.LookRotation(d / len, Vector3.up);

            var cc = segCol[i];
            cc.isTrigger = isTrigger;
            cc.radius = r;
            cc.height = Mathf.Max(2f * r, len + 2f * r);
        }
    }

    static int MaskToSingleLayer(LayerMask mask)
    {
        int v = mask.value;
        if (v == 0) return -1;
        for (int i = 0; i < 32; i++)
            if ((v & (1 << i)) != 0) return i;
        return -1;
    }
}
