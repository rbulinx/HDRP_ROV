using UnityEngine;

/// <summary>
/// 水中テザー（LMM + PBD距離制約）＋ 張力計算 ＋ ROV移動の長さ制限
/// - 上端：固定（topAnchor）
/// - 下端：ROV接続点に固定追従（rovAttach）
/// - 浮力：buoyancyFactorで ± 調整（1=中性、>1=浮く、<1=沈む）
/// - 抗力：線形＋二乗
/// - ウインチ：deployedLength（展開長）をtargetDeployedLengthへwinchSpeedで追従
/// - コリジョン：各ノードを球として障害物から押し戻し（ComputePenetration）
///
/// 追加機能：
/// - 張力（tensionN）を計算：
///   * 直線距離が deployedLength を超えた分（excess）に比例して張力を生成（ばね＋減衰）
/// - ROVの移動制限：
///   * 超過時、外向き速度をカットし、必要なら硬い位置補正（sphere上へ戻す）
///
/// ※既存プロジェクトを壊さないため、クラス名/ファイル名を変更した別実装です。
/// </summary>
[RequireComponent(typeof(LineRenderer))]
public class CableLMM_UnderwaterWinch_Collision_TensionLimit : MonoBehaviour
{
    [Header("Nodes")]
    [Min(2)] public int nodeCount = 40;

    [Tooltip("現在の展開長[m]")]
    public float deployedLength = 10f;

    [Tooltip("目標展開長[m]（外部からここを書き換えると伸縮します）")]
    public float targetDeployedLength = 10f;

    public float minLength = 1f;
    public float maxLength = 50f;

    [Tooltip("伸縮速度[m/s]")]
    public float winchSpeed = 0.5f;

    [Header("Mass & Damping")]
    public float massPerNode = 0.2f;

    [Tooltip("数値安定用の簡易減衰（0〜0.1程度）")]
    public float linearDamping = 0.02f;

    [Header("Underwater Forces")]
    [Tooltip("1=中性浮力（重力と相殺）, >1=浮く, <1=沈む")]
    public float buoyancyFactor = 0.8f;

    [Tooltip("線形抗力係数（簡単・安定）")]
    public float dragLinear = 0.5f;

    [Tooltip("二乗抗力係数（強すぎ注意。まず0でOK）")]
    public float dragQuadratic = 0.0f;

    [Tooltip("潮流（ワールド座標）m/s")]
    public Vector3 currentVelocity = Vector3.zero;

    [Header("Solver")]
    [Tooltip("距離制約の反復回数（増やすほど伸びにくく硬くなります）")]
    [Range(1, 80)] public int solverIterations = 15;

    [Tooltip("サブステップ（貫通や暴れ対策。水中は2〜6が目安）")]
    [Range(1, 12)] public int substeps = 3;

    [Header("Anchors")]
    [Tooltip("上端固定点（必須推奨）")]
    public Transform topAnchor;

    [Tooltip("ROV側接続点（必須推奨）")]
    public Transform rovAttach;

    [Header("Collision (Node Spheres)")]
    public bool enableCollision = true;

    [Tooltip("ノード球半径[m]（ケーブル太さの代替）")]
    public float nodeRadius = 0.03f;

    [Tooltip("衝突対象レイヤ")]
    public LayerMask collisionMask = ~0;

    [Tooltip("1ノードあたりの最大検出数（多いほど重い）")]
    [Range(1, 32)] public int maxOverlapsPerNode = 8;

    [Header("Tension / ROV Limit (NEW)")]
    [Tooltip("張力ばね係数 [N/m]（直線距離がdeployedLengthを超えた分に比例）")]
    public float tensionStiffness = 800f;

    [Tooltip("張力減衰係数 [N/(m/s)]（外向き速度に比例。伸びる方向だけに効かせます）")]
    public float tensionDamping = 150f;

    [Tooltip("張力の上限 [N]")]
    public float maxTension = 8000f;

    [Tooltip("ケーブルが張ったとき、ROVに張力を加える（推奨：ON）")]
    public bool applyTensionToRov = true;

    [Tooltip("deployedLength超過を絶対に許さない位置補正を入れる（推奨：ON）")]
    public bool enforceHardLengthLimit = true;

    [Tooltip("硬制限の許容スラック[m]（0に近いほどガチガチになります）")]
    public float hardLimitSlack = 0.02f;

    [Tooltip("ROV Rigidbody（未設定なら rovAttach の親から自動探索）")]
    public Rigidbody rovRigidbody;

    [Tooltip("張力/制限を Cable 側 FixedUpdate でも適用する（通常OFF推奨。ROV側スクリプトから呼ぶ方が確実）")]
    public bool applyToRovInCableFixedUpdate = false;

    [Header("Gizmos")]
    public bool drawGizmos = false;

    // ---- internal ----
    Vector3[] x;          // positions
    Vector3[] v;          // velocities
    float[] invM;         // inverse mass (0 = fixed)
    float segLen;

    LineRenderer lr;

    // penetration probe
    GameObject probeGO;
    SphereCollider probe;
    Collider[] overlapBuf;

    // tension state
    float tensionN;
    float lastExcess;
    Vector3 lastDirN;

    void Awake()
    {
        lr = GetComponent<LineRenderer>();
        SetupProbe();
        InitCable();
        AutoAssignRovRigidbodyIfNeeded();
    }

    void OnDestroy()
    {
        if (probeGO != null)
        {
#if UNITY_EDITOR
            DestroyImmediate(probeGO);
#else
            Destroy(probeGO);
#endif
        }
    }

    void AutoAssignRovRigidbodyIfNeeded()
    {
        if (rovRigidbody != null) return;
        if (rovAttach == null) return;
        rovRigidbody = rovAttach.GetComponentInParent<Rigidbody>();
    }

    void SetupProbe()
    {
        overlapBuf = new Collider[Mathf.Max(1, maxOverlapsPerNode)];

        probeGO = new GameObject("CableProbeCollider_TensionLimit");
        probeGO.hideFlags = HideFlags.HideAndDontSave;

        // 物理クエリから極力除外（OverlapSphereでTrigger無視にしているので基本ヒットしません）
        probeGO.layer = 2; // Ignore Raycast（collisionMaskから外す運用推奨）

        probe = probeGO.AddComponent<SphereCollider>();
        probe.isTrigger = true;
        probe.radius = Mathf.Max(1e-4f, nodeRadius);
    }

    void InitCable()
    {
        nodeCount = Mathf.Max(2, nodeCount);
        deployedLength = Mathf.Clamp(deployedLength, minLength, maxLength);
        targetDeployedLength = Mathf.Clamp(targetDeployedLength, minLength, maxLength);

        x = new Vector3[nodeCount];
        v = new Vector3[nodeCount];
        invM = new float[nodeCount];

        Vector3 p0 = topAnchor ? topAnchor.position : transform.position;
        Vector3 p1 = rovAttach ? rovAttach.position : (p0 + Vector3.down * deployedLength);

        for (int i = 0; i < nodeCount; i++)
        {
            float t = (float)i / (nodeCount - 1);
            x[i] = Vector3.Lerp(p0, p1, t);
            v[i] = Vector3.zero;
            invM[i] = 1f / Mathf.Max(1e-6f, massPerNode);
        }

        // 端点固定（上端・下端を固定追従）
        invM[0] = 0f;
        invM[nodeCount - 1] = 0f;

        segLen = deployedLength / (nodeCount - 1);

        lr.positionCount = nodeCount;
        UpdateLineRenderer();
    }

    void FixedUpdate()
    {
        if (x == null || x.Length != nodeCount) InitCable();
        if (overlapBuf == null || overlapBuf.Length != maxOverlapsPerNode)
            overlapBuf = new Collider[Mathf.Max(1, maxOverlapsPerNode)];

        AutoAssignRovRigidbodyIfNeeded();

        // ウインチ：展開長を目標へ追従
        float dt = Time.fixedDeltaTime;
        targetDeployedLength = Mathf.Clamp(targetDeployedLength, minLength, maxLength);
        deployedLength = Mathf.MoveTowards(deployedLength, targetDeployedLength, winchSpeed * dt);
        segLen = deployedLength / (nodeCount - 1);

        // 端点追従
        if (topAnchor) x[0] = topAnchor.position;
        if (rovAttach) x[nodeCount - 1] = rovAttach.position;

        // コリジョン半径追従
        if (probe && Mathf.Abs(probe.radius - nodeRadius) > 1e-6f)
            probe.radius = Mathf.Max(1e-4f, nodeRadius);

        float h = dt / Mathf.Max(1, substeps);
        for (int s = 0; s < substeps; s++)
            Step(h);

        UpdateLineRenderer();

        // 張力計算（状態更新）
        ComputeTensionState(dt);

        // Cable側でROVに適用したい場合のみ（通常はROV側から呼ぶことを推奨）
        if (applyToRovInCableFixedUpdate && rovRigidbody != null)
        {
            ApplyTetherToRigidbody(rovRigidbody, dt);
        }
    }

    void Step(float dt)
    {
        // 予測（semi-implicit Euler）
        Vector3[] xPrev = new Vector3[nodeCount];

        for (int i = 0; i < nodeCount; i++)
        {
            xPrev[i] = x[i];

            if (invM[i] == 0f) continue; // 固定点

            // 重力＋浮力（簡易：有効重力で調整）
            Vector3 gEff = Physics.gravity * (1f - buoyancyFactor);

            // 抗力：相対速度（潮流との差）
            Vector3 vRel = v[i] - currentVelocity;

            Vector3 drag = -dragLinear * vRel;
            if (dragQuadratic > 0f)
            {
                float sp = vRel.magnitude;
                drag += -dragQuadratic * sp * vRel;
            }

            Vector3 a = gEff + drag / Mathf.Max(1e-6f, massPerNode);

            v[i] = (v[i] + a * dt);
            v[i] *= (1f - linearDamping);
            x[i] += v[i] * dt;
        }

        // 制約反復：距離制約 → 衝突（どちらも「位置修正」）
        for (int it = 0; it < solverIterations; it++)
        {
            SolveDistanceConstraints();

            if (enableCollision)
                SolveCollisions_NodeSpheres();

            // 端点は毎回拘束点に戻す
            if (topAnchor) x[0] = topAnchor.position;
            if (rovAttach) x[nodeCount - 1] = rovAttach.position;
        }

        // 速度更新（位置修正を反映）
        for (int i = 0; i < nodeCount; i++)
        {
            if (invM[i] == 0f)
            {
                v[i] = Vector3.zero;
                continue;
            }
            v[i] = (x[i] - xPrev[i]) / dt;
        }
    }

    void SolveDistanceConstraints()
    {
        for (int i = 0; i < nodeCount - 1; i++)
        {
            int j = i + 1;

            Vector3 delta = x[j] - x[i];
            float dist = delta.magnitude;
            if (dist < 1e-8f) continue;

            float w1 = invM[i];
            float w2 = invM[j];
            float wSum = w1 + w2;
            if (wSum <= 0f) continue;

            // C = |xj-xi| - segLen = 0
            float C = dist - segLen;
            Vector3 n = delta / dist;

            // PBD補正
            Vector3 corr = (C / wSum) * n;

            if (w1 > 0f) x[i] += w1 * corr;
            if (w2 > 0f) x[j] -= w2 * corr;
        }
    }

    void SolveCollisions_NodeSpheres()
    {
        if (!probe) return;

        float r = Mathf.Max(1e-4f, nodeRadius);

        for (int i = 0; i < nodeCount; i++)
        {
            if (invM[i] == 0f) continue; // 固定点は動かさない

            int hitCount = Physics.OverlapSphereNonAlloc(
                x[i], r, overlapBuf, collisionMask, QueryTriggerInteraction.Ignore);

            for (int h = 0; h < hitCount; h++)
            {
                Collider col = overlapBuf[h];
                if (!col) continue;

                Vector3 dir;
                float dist;
                bool overlapped = Physics.ComputePenetration(
                    probe, x[i], Quaternion.identity,
                    col, col.transform.position, col.transform.rotation,
                    out dir, out dist);

                if (overlapped && dist > 0f)
                {
                    x[i] += dir * dist;
                }
            }
        }
    }

    void UpdateLineRenderer()
    {
        if (!lr) return;
        for (int i = 0; i < nodeCount; i++)
            lr.SetPosition(i, x[i]);
    }

    void ComputeTensionState(float dt)
    {
        tensionN = 0f;
        lastExcess = 0f;
        lastDirN = Vector3.zero;

        Vector3 anchorPos = topAnchor ? topAnchor.position : (x != null && x.Length > 0 ? x[0] : transform.position);
        Vector3 attachPos = rovAttach ? rovAttach.position : (x != null && x.Length > 0 ? x[nodeCount - 1] : transform.position);

        Vector3 d = attachPos - anchorPos;
        float dist = d.magnitude;
        if (dist < 1e-6f) return;

        Vector3 dirN = d / dist;
        lastDirN = dirN;

        float excess = dist - deployedLength;
        if (excess <= 0f) return; // たるみ → 張力なし

        // 外向き速度（伸び方向の速度）だけ減衰を足す
        float radialOutVel = 0f;
        if (rovRigidbody != null && rovAttach != null)
        {
            Vector3 vPoint = rovRigidbody.GetPointVelocity(rovAttach.position);
            radialOutVel = Mathf.Max(0f, Vector3.Dot(vPoint, dirN));
        }

        float t = tensionStiffness * excess + tensionDamping * radialOutVel;
        tensionN = Mathf.Clamp(t, 0f, maxTension);
        lastExcess = excess;
    }

    /// <summary>
    /// ROV側（推力ベース移動）から呼ぶことを推奨。
    /// 1) 張力を AddForceAtPosition で付与
    /// 2) 必要なら deployedLength を超えた分を硬く位置補正
    /// 3) 外向き速度（伸びる方向）をカットして、無限伸長を防止
    /// </summary>
    public void ApplyTetherToRigidbody(Rigidbody rb, float dt)
    {
        if (rb == null || rovAttach == null) return;

        Vector3 anchorPos = topAnchor ? topAnchor.position : (x != null && x.Length > 0 ? x[0] : transform.position);
        Vector3 attachPos = rovAttach.position;

        Vector3 d = attachPos - anchorPos;
        float dist = d.magnitude;
        if (dist < 1e-6f) return;

        Vector3 dirN = d / dist;

        float maxLen = Mathf.Max(0.01f, deployedLength);
        float excess = dist - maxLen;

        // (A) 張力（ばね＋減衰）
        if (applyTensionToRov)
        {
            // 伸びているときだけ張力
            if (excess > 0f)
            {
                float radialOutVel = 0f;
                Vector3 vPoint = rb.GetPointVelocity(attachPos);
                radialOutVel = Mathf.Max(0f, Vector3.Dot(vPoint, dirN));

                float t = tensionStiffness * excess + tensionDamping * radialOutVel;
                t = Mathf.Clamp(t, 0f, maxTension);

                rb.AddForceAtPosition(-dirN * t, attachPos, ForceMode.Force);
            }
        }

        // (B) 硬い長さ制限（位置補正）
        if (enforceHardLengthLimit && excess > hardLimitSlack)
        {
            // ROVの位置を、attach点が半径maxLenの球面上に来るよう補正する
            Vector3 targetAttach = anchorPos + dirN * maxLen;

            // rb.position から attach までのオフセットを維持して MovePosition する
            Vector3 offset = attachPos - rb.position;
            Vector3 targetRbPos = targetAttach - offset;

            rb.MovePosition(targetRbPos);

            // 伸び方向（外向き）速度を削る
#if UNITY_6000_0_OR_NEWER
            Vector3 vel = rb.linearVelocity;
#else
            Vector3 vel = rb.velocity;
#endif
            float outComp = Vector3.Dot(vel, dirN);
            if (outComp > 0f)
            {
                vel -= dirN * outComp;
#if UNITY_6000_0_OR_NEWER
                rb.linearVelocity = vel;
#else
                rb.velocity = vel;
#endif
            }
        }
    }

    // --- Public getters ---
    public int GetNodeCount() => nodeCount;
    public Vector3 GetNodePosition(int i) => x[i];

    public float GetTensionNewton() => tensionN;
    public float GetStraightExcessMeters() => Mathf.Max(0f, lastExcess);
    public bool IsTaut() => lastExcess > 0f;

    void OnValidate()
    {
        nodeCount = Mathf.Max(2, nodeCount);
        minLength = Mathf.Max(0.01f, minLength);
        maxLength = Mathf.Max(minLength, maxLength);
        deployedLength = Mathf.Clamp(deployedLength, minLength, maxLength);
        targetDeployedLength = Mathf.Clamp(targetDeployedLength, minLength, maxLength);
        massPerNode = Mathf.Max(1e-4f, massPerNode);
        nodeRadius = Mathf.Max(1e-4f, nodeRadius);

        tensionStiffness = Mathf.Max(0f, tensionStiffness);
        tensionDamping = Mathf.Max(0f, tensionDamping);
        maxTension = Mathf.Max(0f, maxTension);
        hardLimitSlack = Mathf.Max(0f, hardLimitSlack);
    }

    void OnDrawGizmosSelected()
    {
        if (!drawGizmos || x == null) return;

        Gizmos.matrix = Matrix4x4.identity;
        Gizmos.color = Color.white;

        float r = Mathf.Max(1e-4f, nodeRadius);
        for (int i = 0; i < x.Length; i++)
            Gizmos.DrawWireSphere(x[i], r);
    }
}
