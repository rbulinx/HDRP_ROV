using System;
using UnityEngine;

public class CableSonarColliders_UniqueNames_MultiSource : MonoBehaviour
{
    [Header("Source Cable (Drag your cable component here)")]
    [Tooltip("CableLMM... 系コンポーネントをドラッグしてください（型は何でも可）")]
    public MonoBehaviour sourceCable;

    [Tooltip("未設定なら同一GameObjectから自動取得を試みる")]
    public bool autoFindOnSameGameObject = true;

    [Header("Collider Settings")]
    public bool isTrigger = true;
    public float radius = 0.02f;
    public LayerMask sonarLayer = 0;
    public bool matchCableRadius = true;

    [Header("Naming (IMPORTANT)")]
    [Tooltip("空欄なら GameObject名 + InstanceID を使って一意にします")]
    public string namePrefix = "";

    [Tooltip("Rebuild時、prefix一致の既存子コライダを掃除して作り直す")]
    public bool cleanupByPrefix = true;

    [Header("Performance")]
    public bool rebuildEveryStart = true;

    // ---- bound delegates (cable-agnostic) ----
    Func<int> _getNodeCount;
    Func<int, Vector3> _getNodePos;
    Func<float> _getNodeRadius;

    Transform[] segTf;
    CapsuleCollider[] segCol;

    void Reset()
    {
        if (sourceCable == null && autoFindOnSameGameObject)
            TryAutoFind();
        BindOrWarn();
    }

    void Start()
    {
        if (sourceCable == null && autoFindOnSameGameObject)
            TryAutoFind();

        if (BindOrWarn() && rebuildEveryStart)
            Rebuild();
    }

    void OnValidate()
    {
        if (!Application.isPlaying)
        {
            if (sourceCable == null && autoFindOnSameGameObject)
                TryAutoFind();
            BindOrWarn();
        }
    }

    void TryAutoFind()
    {
        // 同一GameObject上の候補を探す（優先順は適宜）
        sourceCable =
            (MonoBehaviour)GetComponent<CableLMM_UnderwaterWinch_Collision_ConstraintImpulse>() ??
            (MonoBehaviour)GetComponent<CableLMM_UnderwaterWinch_Collision_TensionLimit>() ??
            (MonoBehaviour)GetComponent<CableLMM_UnderwaterWinch_Collision>();
    }

    bool BindOrWarn()
    {
        _getNodeCount = null;
        _getNodePos = null;
        _getNodeRadius = null;

        if (sourceCable == null)
        {
            // 参照が無い
            return false;
        }

        // --- 既知のケーブル型へバインド ---
        if (sourceCable is CableLMM_UnderwaterWinch_Collision_ConstraintImpulse c3)
        {
            _getNodeCount = c3.GetNodeCount;
            _getNodePos = c3.GetNodePosition;
            _getNodeRadius = () => c3.nodeRadius;
            return true;
        }

        if (sourceCable is CableLMM_UnderwaterWinch_Collision_TensionLimit c2)
        {
            _getNodeCount = c2.GetNodeCount;
            _getNodePos = c2.GetNodePosition;
            _getNodeRadius = () => c2.nodeRadius;
            return true;
        }

        if (sourceCable is CableLMM_UnderwaterWinch_Collision c1)
        {
            _getNodeCount = c1.GetNodeCount;
            _getNodePos = c1.GetNodePosition;
            _getNodeRadius = () => c1.nodeRadius;
            return true;
        }

        Debug.LogError($"[{nameof(CableSonarColliders_UniqueNames_MultiSource)}] sourceCable の型 {sourceCable.GetType().Name} は未対応です。");
        sourceCable = null;
        return false;
    }

    public void Rebuild()
    {
        if (!BindOrWarn()) return;

        string prefix = GetPrefix();

        if (cleanupByPrefix)
        {
            for (int i = transform.childCount - 1; i >= 0; i--)
            {
                Transform ch = transform.GetChild(i);
                if (ch != null && ch.name.StartsWith(prefix + "_CableSegCol_"))
                {
#if UNITY_EDITOR
                    DestroyImmediate(ch.gameObject);
#else
                    Destroy(ch.gameObject);
#endif
                }
            }
        }
        else
        {
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
        }

        int n = _getNodeCount();
        int segCount = Mathf.Max(1, n - 1);

        segTf = new Transform[segCount];
        segCol = new CapsuleCollider[segCount];

        for (int i = 0; i < segCount; i++)
        {
            var go = new GameObject($"{prefix}_CableSegCol_{i}");
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
        if (!BindOrWarn()) return;

        int n = _getNodeCount();
        int segCount = Mathf.Max(1, n - 1);

        if (segTf == null || segTf.Length != segCount)
        {
            Rebuild();
            if (segTf == null || segTf.Length != segCount) return;
        }

        float r = matchCableRadius ? Mathf.Max(1e-4f, _getNodeRadius()) : Mathf.Max(1e-4f, radius);

        for (int i = 0; i < segCount; i++)
        {
            Vector3 p0 = _getNodePos(i);
            Vector3 p1 = _getNodePos(i + 1);
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

    string GetPrefix()
    {
        if (!string.IsNullOrWhiteSpace(namePrefix)) return namePrefix.Trim();
        return $"{gameObject.name}_{gameObject.GetInstanceID()}";
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
