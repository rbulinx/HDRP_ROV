using UnityEngine;

/// <summary>
/// LMM(PBD)ケーブル + ノード球コリジョン + ウインチ長制御
/// 追加：ROV側を「拘束インパルス」で引き戻し、deployedLength を超えて伸びないようにする（より物理寄り）
/// 
/// - ケーブル形状（たるみ等）は従来通り PBD(LMM) で更新
/// - 長さ制限は「端点間の不等式拘束」 dist <= deployedLength を、Rigidbodyに対してインパルスで実現
///   * MovePositionの瞬間移動は行わない（必要なら最後の保険としてオプションで可能）
/// - 張力はインパルスから推定：tension[N] ≈ impulse[N·s] / dt
/// </summary>
[RequireComponent(typeof(LineRenderer))]
public class CableLMM_UnderwaterWinch_Collision_ConstraintImpulse : MonoBehaviour
{
    [Header("Nodes")]
    [Min(2)] public int nodeCount = 40;

    [Tooltip("現在の展開長[m]")]
    public float deployedLength = 10f;

    [Tooltip("目標展開長[m]")]
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
    public float dragLinear = 0.5f;
    public float dragQuadratic = 0.0f;
    public Vector3 currentVelocity = Vector3.zero;

    [Header("Solver")]
    [Range(1, 80)] public int solverIterations = 15;
    [Range(1, 12)] public int substeps = 3;

    [Header("Anchors")]
    public Transform topAnchor;
    public Transform rovAttach;

    [Header("Collision (Node Spheres)")]
    public bool enableCollision = true;
    public float nodeRadius = 0.03f;
    public LayerMask collisionMask = ~0;
    [Range(1, 32)] public int maxOverlapsPerNode = 8;

    [Header("Tether Constraint (NEW, Physics-ish)")]
    [Tooltip("制限が効き始める手前の余裕[m]（ここより近づくと外向き速度を抑え始める）")]
    public float activationSlack = 0.03f;

    [Tooltip("位置誤差(excess)を速度バイアスで戻す係数（Baumgarte）。0.1〜0.3推奨")]
    [Range(0f, 1f)] public float baumgarteBeta = 0.2f;

    [Tooltip("張力上限[N]（インパルスを clamp する）")]
    public float maxTensionNewton = 8000f;

    [Tooltip("張力をROVに適用する")]
    public bool applyConstraintToRov = true;

    [Tooltip("ROV Rigidbody（未設定ならrovAttach親から自動探索）")]
    public Rigidbody rovRigidbody;

    [Header("Safety (optional)")]
    [Tooltip("最終保険：どうしても越える場合だけ、球面へ投影して戻す（通常OFF推奨）")]
    public bool emergencyProjectToSphere = false;

    [Tooltip("投影の許容超過[m]（これ以上超えたら投影）")]
    public float emergencyProjectThreshold = 0.20f;

    [Header("Gizmos")]
    public bool drawGizmos = false;

    // --- internal ---
    Vector3[] x;
    Vector3[] v;
    float[] invM;
    float segLen;

    LineRenderer lr;

    GameObject probeGO;
    SphereCollider probe;
    Collider[] overlapBuf;

    // constraint state (for UI/log)
    float lastTensionN;
    float lastImpulseNs;
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

        probeGO = new GameObject("CableProbeCollider_ConstraintImpulse");
        probeGO.hideFlags = HideFlags.HideAndDontSave;
        probeGO.layer = 2; // Ignore Raycast

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

        float dt = Time.fixedDeltaTime;

        // ウインチ
        targetDeployedLength = Mathf.Clamp(targetDeployedLength, minLength, maxLength);
        deployedLength = Mathf.MoveTowards(deployedLength, targetDeployedLength, winchSpeed * dt);
        segLen = deployedLength / (nodeCount - 1);

        // 端点追従
        if (topAnchor) x[0] = topAnchor.position;
        if (rovAttach) x[nodeCount - 1] = rovAttach.position;

        if (probe && Mathf.Abs(probe.radius - nodeRadius) > 1e-6f)
            probe.radius = Mathf.Max(1e-4f, nodeRadius);

        float h = dt / Mathf.Max(1, substeps);
        for (int s = 0; s < substeps; s++)
            Step(h);

        UpdateLineRenderer();

        // 拘束インパルス（ROV側に適用）
        lastImpulseNs = 0f;
        lastTensionN = 0f;
        lastExcess = 0f;
        lastDirN = Vector3.zero;

        if (applyConstraintToRov && rovRigidbody != null && rovAttach != null)
        {
            ApplyTetherConstraintImpulse(rovRigidbody, dt);
        }
        else
        {
            // 状態だけ更新（UI用）
            ComputeConstraintStateOnly();
        }
    }

    void Step(float dt)
    {
        Vector3[] xPrev = new Vector3[nodeCount];

        for (int i = 0; i < nodeCount; i++)
        {
            xPrev[i] = x[i];
            if (invM[i] == 0f) continue;

            Vector3 gEff = Physics.gravity * (1f - buoyancyFactor);

            Vector3 vRel = v[i] - currentVelocity;

            Vector3 drag = -dragLinear * vRel;
            if (dragQuadratic > 0f)
            {
                float sp = vRel.magnitude;
                drag += -dragQuadratic * sp * vRel;
            }

            Vector3 a = gEff + drag / Mathf.Max(1e-6f, massPerNode);

            v[i] = v[i] + a * dt;
            v[i] *= (1f - linearDamping);
            x[i] += v[i] * dt;
        }

        for (int it = 0; it < solverIterations; it++)
        {
            SolveDistanceConstraints();

            if (enableCollision)
                SolveCollisions_NodeSpheres();

            if (topAnchor) x[0] = topAnchor.position;
            if (rovAttach) x[nodeCount - 1] = rovAttach.position;
        }

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

            float C = dist - segLen;
            Vector3 n = delta / dist;

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
            if (invM[i] == 0f) continue;

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

    void ComputeConstraintStateOnly()
    {
        Vector3 anchorPos = topAnchor ? topAnchor.position : (x != null && x.Length > 0 ? x[0] : transform.position);
        Vector3 attachPos = rovAttach ? rovAttach.position : (x != null && x.Length > 0 ? x[nodeCount - 1] : transform.position);

        Vector3 d = attachPos - anchorPos;
        float dist = d.magnitude;
        if (dist < 1e-6f) return;

        lastDirN = d / dist;
        lastExcess = Mathf.Max(0f, dist - deployedLength);
    }

    /// <summary>
    /// 端点間の不等式拘束 dist <= deployedLength を、Rigidbodyに対してインパルスで満たす。
    /// - dist が L に近づいて外向き速度がある場合：外向き速度を打ち消すインパルス
    /// - dist が L を超えている場合：Baumgarteで戻す速度バイアスを追加
    /// </summary>
    public void ApplyTetherConstraintImpulse(Rigidbody rb, float dt)
    {
        Vector3 anchorPos = topAnchor ? topAnchor.position : (x != null && x.Length > 0 ? x[0] : transform.position);
        Vector3 attachPos = rovAttach.position;

        Vector3 d = attachPos - anchorPos;
        float dist = d.magnitude;
        if (dist < 1e-6f) return;

        Vector3 n = d / dist;
        lastDirN = n;

        float L = Mathf.Max(0.01f, deployedLength);
        float excess = dist - L;
        lastExcess = Mathf.Max(0f, excess);

        // 外向き（伸ばす方向）の点速度
        Vector3 vPoint = rb.GetPointVelocity(attachPos);
        float vOut = Vector3.Dot(vPoint, n); // +: 外向き

        // 拘束をかける条件：
        // 1) 超過している（excess > 0）
        // 2) 超過はしていないが、L - activationSlack より外側にいて外向き速度がある
        bool shouldConstrain = (excess > 0f) || (dist > (L - Mathf.Max(0f, activationSlack)) && vOut > 0f);
        if (!shouldConstrain) return;

        // Baumgarte：excessを dt で戻すための「内向き目標速度」
        float bias = 0f;
        if (excess > 0f)
        {
            float beta = Mathf.Clamp01(baumgarteBeta);
            bias = beta * (excess / Mathf.Max(1e-4f, dt)); // [m/s]
        }

        // 有効質量 k = n·M^-1·n（接続点）
        float k = ComputeEffectiveMassAlong(rb, attachPos, n);
        if (k < 1e-8f) return;

        // 目標：vOut_new <= -bias
        // impulse J を -n方向に加えると、vOut_new = vOut - J*k
        // よって J = (vOut + bias)/k  （J>=0 のみ＝引っ張りのみ）
        float J = (vOut + bias) / k;
        if (J <= 0f) return;

        // 張力上限でインパルスを制限（maxTension[N] * dt = maxImpulse[Ns]）
        float maxImpulse = Mathf.Max(0f, maxTensionNewton) * Mathf.Max(1e-4f, dt);
        if (maxImpulse > 0f) J = Mathf.Min(J, maxImpulse);

        rb.AddForceAtPosition(-n * J, attachPos, ForceMode.Impulse);

        lastImpulseNs = J;
        lastTensionN = J / Mathf.Max(1e-4f, dt);

        // 最終保険：極端に飛び出したら球面へ投影（通常OFF）
        if (emergencyProjectToSphere && excess > Mathf.Max(0.01f, emergencyProjectThreshold))
        {
            Vector3 targetAttach = anchorPos + n * L;
            Vector3 offset = attachPos - rb.position;
            Vector3 targetRbPos = targetAttach - offset;
            rb.MovePosition(targetRbPos);
        }
    }

    /// <summary>
    /// 有効質量 k = n·M^-1·n を、Rigidbodyの質量＋慣性テンソルから計算
    /// </summary>
    static float ComputeEffectiveMassAlong(Rigidbody rb, Vector3 pointWorld, Vector3 nWorld)
    {
        // 並進
        float invMass = 1f / Mathf.Max(1e-6f, rb.mass);

        // 回転
        Vector3 r = pointWorld - rb.worldCenterOfMass;
        Vector3 c = Vector3.Cross(r, nWorld);

        // inv inertia world
        Matrix4x4 invI = GetInverseInertiaWorld(rb);
        Vector3 invIc = Mul33(invI, c);
        float angular = Vector3.Dot(c, invIc); // c·I^-1·c

        // k = invMass + angular
        return invMass + angular;
    }

    static Matrix4x4 GetInverseInertiaWorld(Rigidbody rb)
    {
        Vector3 I = rb.inertiaTensor;
        Vector3 invI = new Vector3(
            1f / Mathf.Max(1e-6f, I.x),
            1f / Mathf.Max(1e-6f, I.y),
            1f / Mathf.Max(1e-6f, I.z)
        );

        Quaternion q = rb.rotation * rb.inertiaTensorRotation;
        Matrix4x4 R = Matrix4x4.Rotate(q);
        Matrix4x4 Rt = R.transpose;

        Matrix4x4 D = Matrix4x4.zero;
        D.m00 = invI.x; D.m11 = invI.y; D.m22 = invI.z; D.m33 = 1f;

        return R * D * Rt;
    }

    static Vector3 Mul33(Matrix4x4 m, Vector3 v)
    {
        return new Vector3(
            m.m00 * v.x + m.m01 * v.y + m.m02 * v.z,
            m.m10 * v.x + m.m11 * v.y + m.m12 * v.z,
            m.m20 * v.x + m.m21 * v.y + m.m22 * v.z
        );
    }

    // --- Public getters ---
    public int GetNodeCount() => nodeCount;
    public Vector3 GetNodePosition(int i) => x[i];
    public float GetTensionNewton() => lastTensionN;
    public float GetLastImpulseNs() => lastImpulseNs;
    public float GetStraightExcessMeters() => lastExcess;
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

        activationSlack = Mathf.Max(0f, activationSlack);
        maxTensionNewton = Mathf.Max(0f, maxTensionNewton);
        emergencyProjectThreshold = Mathf.Max(0f, emergencyProjectThreshold);
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
