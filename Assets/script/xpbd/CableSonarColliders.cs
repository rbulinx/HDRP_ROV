using System;
using System.Reflection;
using UnityEngine;

namespace ROVSim.XPBD
{
    /// <summary>
    /// CableSonarColliders（XPBD用・汎用）
    /// ------------------------------------------------------------
    /// 目的：ケーブルのノード列を CapsuleCollider セグメントに変換し、ソナー（Physicsクエリ）に当てる。
    ///
    /// 改良点：
    ///  - 旧：CableLMM_* 型に固定 → CableXPBD等に差し替えると参照不能
    ///  - 新：MonoBehaviour参照＋リフレクションで GetNodeCount / GetNodePosition を呼び出し、
    ///       どのケーブル実装でも使えるようにする。
    ///
    /// 必須（ケーブル側に必要）：
    ///   int GetNodeCount()
    ///   Vector3 GetNodePosition(int i)
    ///
    /// 注意：
    ///  - ソナー側が Trigger を無視している場合は isTrigger=false にしてください。
    ///  - layer は sonarLayer を「単一レイヤ」で指定するのが安全です（複数bitは先頭bitを採用）。
    /// </summary>
    public class CableSonarColliders : MonoBehaviour
    {
        [Header("Source Cable (Any cable component)")]
        [Tooltip("CableXPBD など、GetNodeCount / GetNodePosition を持つコンポーネントを指定")]
        public MonoBehaviour sourceCable;

        [Tooltip("未指定なら、このGameObject上から自動でケーブルを探す")]
        public bool autoFindOnSameGameObject = true;

        [Header("Collider Settings")]
        [Tooltip("ソナー側がTriggerを拾わない場合は OFF")]
        public bool isTrigger = true;

        [Tooltip("ソナーに当てたい“太さ”")]
        public float radius = 0.02f;

        [Tooltip("コライダを配置するレイヤ（単一レイヤ運用推奨）")]
        public LayerMask sonarLayer = 0;

        [Tooltip("ケーブル側に nodeRadius があればそれを使う")]
        public bool matchCableRadius = true;

        [Header("Performance")]
        public bool rebuildEveryStart = true;

        Transform[] segTf;
        CapsuleCollider[] segCol;

        // ---- cached reflection ----
        MethodInfo miGetNodeCount;
        MethodInfo miGetNodePosition;
        FieldInfo fiNodeRadius;
        PropertyInfo piNodeRadius;

        void Reset()
        {
            ResolveCableReference();
        }

        void Start()
        {
            ResolveCableReference();
            if (rebuildEveryStart) Rebuild();
        }

        void OnValidate()
        {
            ResolveCableReference();
        }

        /// <summary>
        /// ケーブル参照を確定し、必要なメソッドをキャッシュする
        /// </summary>
        void ResolveCableReference()
        {
            if (sourceCable == null && autoFindOnSameGameObject)
            {
                // 同一GO上から「必要メソッドを持つ」MonoBehaviourを探す
                var mbs = GetComponents<MonoBehaviour>();
                for (int i = 0; i < mbs.Length; i++)
                {
                    if (mbs[i] == null) continue;
                    if (TryCacheCableAPI(mbs[i]))
                    {
                        sourceCable = mbs[i];
                        break;
                    }
                }
            }
            else
            {
                if (sourceCable != null)
                    TryCacheCableAPI(sourceCable);
            }
        }

        bool TryCacheCableAPI(MonoBehaviour mb)
        {
            if (mb == null) return false;

            Type t = mb.GetType();

            // int GetNodeCount()
            miGetNodeCount = t.GetMethod("GetNodeCount",
                BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic,
                null, Type.EmptyTypes, null);

            // Vector3 GetNodePosition(int)
            miGetNodePosition = t.GetMethod("GetNodePosition",
                BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic,
                null, new[] { typeof(int) }, null);

            if (miGetNodeCount == null || miGetNodePosition == null)
                return false;

            // optional nodeRadius (field or property)
            fiNodeRadius = t.GetField("nodeRadius", BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
            piNodeRadius = t.GetProperty("nodeRadius", BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);

            return true;
        }

        int CableGetNodeCount()
        {
            if (sourceCable == null || miGetNodeCount == null) return 0;
            try { return (int)miGetNodeCount.Invoke(sourceCable, null); }
            catch { return 0; }
        }

        Vector3 CableGetNodePosition(int i)
        {
            if (sourceCable == null || miGetNodePosition == null) return Vector3.zero;
            try { return (Vector3)miGetNodePosition.Invoke(sourceCable, new object[] { i }); }
            catch { return Vector3.zero; }
        }

        float CableGetNodeRadiusOrDefault(float fallback)
        {
            if (sourceCable == null) return fallback;

            try
            {
                if (fiNodeRadius != null && fiNodeRadius.FieldType == typeof(float))
                    return (float)fiNodeRadius.GetValue(sourceCable);

                if (piNodeRadius != null && piNodeRadius.PropertyType == typeof(float))
                    return (float)piNodeRadius.GetValue(sourceCable);

                return fallback;
            }
            catch
            {
                return fallback;
            }
        }

        public void Rebuild()
        {
            ResolveCableReference();
            if (sourceCable == null || miGetNodeCount == null || miGetNodePosition == null) return;

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

            int n = CableGetNodeCount();
            int segCount = Mathf.Max(1, n - 1);

            segTf = new Transform[segCount];
            segCol = new CapsuleCollider[segCount];

            for (int i = 0; i < segCount; i++)
            {
                var go = new GameObject($"CableSegCol_{i}");
                go.transform.SetParent(transform, false);

                // レイヤ設定（単一レイヤ想定：最下位bitを採用）
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
            ResolveCableReference();
            if (sourceCable == null || miGetNodeCount == null || miGetNodePosition == null) return;

            int n = CableGetNodeCount();
            int segCount = Mathf.Max(1, n - 1);

            if (segTf == null || segTf.Length != segCount)
            {
                Rebuild();
                if (segTf == null || segTf.Length != segCount) return;
            }

            float r = matchCableRadius
                ? Mathf.Max(1e-4f, CableGetNodeRadiusOrDefault(radius))
                : Mathf.Max(1e-4f, radius);

            for (int i = 0; i < segCount; i++)
            {
                Vector3 p0 = CableGetNodePosition(i);
                Vector3 p1 = CableGetNodePosition(i + 1);
                Vector3 d = (p1 - p0);
                float len = d.magnitude;

                if (len < 1e-6f)
                    continue;

                // 中点に配置
                Transform t = segTf[i];
                t.position = (p0 + p1) * 0.5f;

                // Z軸がセグメント方向を向く
                t.rotation = Quaternion.LookRotation(d / len, Vector3.up);

                // コライダ寸法
                var cc = segCol[i];
                cc.isTrigger = isTrigger;
                cc.radius = r;
                // CapsuleCollider.height は半球込みなので最低2r
                cc.height = Mathf.Max(2f * r, len + 2f * r);
            }
        }

        static int MaskToSingleLayer(LayerMask mask)
        {
            int v = mask.value;
            if (v == 0) return -1;
            for (int i = 0; i < 32; i++)
            {
                if ((v & (1 << i)) != 0) return i;
            }
            return -1;
        }
    }
}
