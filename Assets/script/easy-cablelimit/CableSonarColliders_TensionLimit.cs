using UnityEngine;

public class CableSonarColliders_TensionLimit : MonoBehaviour
{
    [Header("Source Cable")]
    public CableLMM_UnderwaterWinch_Collision_TensionLimit cable;

    [Header("Collider Settings")]
    public bool isTrigger = true;
    public float radius = 0.02f;               // ソナーに当てたい“太さ”
    public LayerMask sonarLayer = 0;           // ここは「Layer番号」を入れる（Inspectorで選ぶのが安全）
    public bool matchCableRadius = true;       // cable.nodeRadius を使うなら true

    [Header("Performance")]
    public bool rebuildEveryStart = true;

    Transform[] segTf;
    CapsuleCollider[] segCol;

    void Reset()
    {
        cable = GetComponent<CableLMM_UnderwaterWinch_Collision_TensionLimit>();
    }

    void Start()
    {
        if (!cable) cable = GetComponent<CableLMM_UnderwaterWinch_Collision_TensionLimit>();
        if (rebuildEveryStart) Rebuild();
    }

    void OnValidate()
    {
        if (!cable) cable = GetComponent<CableLMM_UnderwaterWinch_Collision_TensionLimit>();
    }

    public void Rebuild()
    {
        if (!cable) return;

        // 既存削除
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
            var go = new GameObject($"CableSegCol_TensionLimit_{i}");
            go.transform.SetParent(transform, false);

            // レイヤ設定（Inspectorでレイヤを選ぶのが確実）
            if (sonarLayer.value != 0)
            {
                int layer = MaskToSingleLayer(sonarLayer);
                if (layer >= 0) go.layer = layer;
            }

            var cc = go.AddComponent<CapsuleCollider>();
            cc.direction = 2; // Z軸方向
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

            if (len < 1e-6f)
                continue;

            // 中点に配置
            Transform t = segTf[i];
            t.position = (p0 + p1) * 0.5f;

            // Z軸がセグメント方向を向くよう回転
            t.rotation = Quaternion.LookRotation(d / len, Vector3.up);

            // コライダ寸法
            var cc = segCol[i];
            cc.isTrigger = isTrigger;
            cc.radius = r;
            // CapsuleCollider.height は「端の半球込み」なので、最低 2r より大きく
            cc.height = Mathf.Max(2f * r, len + 2f * r);
        }
    }

    static int MaskToSingleLayer(LayerMask mask)
    {
        int v = mask.value;
        if (v == 0) return -1;

        // 最下位bitのレイヤ番号を返す（単一レイヤ運用推奨）
        for (int i = 0; i < 32; i++)
        {
            if ((v & (1 << i)) != 0) return i;
        }
        return -1;
    }
}
