using UnityEngine;

[DefaultExecutionOrder(-200)] // ソナー側より先に動くように（可能な限り）
[RequireComponent(typeof(LineRenderer))]
public class CableLMM_UnderwaterWinch_Collision_EA_Bending_Generic : MonoBehaviour
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

    [Header("Mass & Damping (Cable Nodes)")]
    public float massPerNode = 0.2f;
    [Tooltip("数値安定用の簡易減衰（0〜0.2程度）")]
    public float linearDamping = 0.05f;

    [Header("Underwater Forces (Cable Nodes)")]
    [Tooltip("1=中性浮力, >1=浮く, <1=沈む")]
    public float buoyancyFactor = 0.8f;
    public float dragLinear = 2.0f;
    public float dragQuadratic = 0.0f;
    public Vector3 currentVelocity = Vector3.zero;

    [Header("Solver")]
    [Range(1, 80)] public int solverIterations = 35;
    [Range(1, 12)] public int substeps = 8;

    [Header("Anchors")]
    public Transform topAnchor;
    public Transform bottomAttach;

    [Header("Collision (Node Spheres)")]
    public bool enableCollision = true;
    public float nodeRadius = 0.03f;
    public LayerMask collisionMask = ~0;
    [Range(1, 32)] public int maxOverlapsPerNode = 8;

    [Header("Bending Rigidity (NEW)")]
    public bool enableBending = true;
    [Range(0f, 1f)] public float bendStiffness = 0.12f;
    public bool usePhysicalEI = false;
    public float bendingRigidityEI = 10.0f;
    public float bendMaxCorrection = 0.15f;

    [Header("Axial Rigidity EA (Tether Reaction)")]
    public bool applyTetherToBottom = true;
    public Rigidbody bottomRigidbody;
    public float axialRigidityEA = 2.0e6f;
    public float axialDamping = 1.0e4f;
    public float maxTensionNewton = 8000f;
    public float slackMeters = 0.01f;
    public float maxStretchMeters = 0f;

    [Header("Constraint Damping (stabilizer)")]
    [Range(0f, 1f)] public float constraintVelocityDamping = 0.15f;

    [Header("Gizmos")]
    public bool drawGizmos = false;

    // ---- internal ----
    Vector3[] x;
    Vector3[] v;
    float[] invM;
    float segLen;

    LineRenderer lr;

    GameObject probeGO;
    SphereCollider probe;
    Collider[] overlapBuf;

    // debug/hud
    float lastTensionN;
    float lastStretch;
    float lastStraightDist;
    Vector3 lastDirN;

    int _lastInitializedNodeCount = -1;

    void Awake()
    {
        lr = GetComponent<LineRenderer>();
        SetupProbe();
        EnsureInitialized(force: true);
        AutoAssignBottomRigidbodyIfNeeded();
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

    void AutoAssignBottomRigidbodyIfNeeded()
    {
        if (bottomRigidbody != null) return;
        if (bottomAttach == null) return;
        bottomRigidbody = bottomAttach.GetComponentInParent<Rigidbody>();
    }

    void SetupProbe()
    {
        overlapBuf = new Collider[Mathf.Max(1, maxOverlapsPerNode)];

        probeGO = new GameObject("CableProbeCollider_EA_Bending_Generic");
        probeGO.hideFlags = HideFlags.HideAndDontSave;
        probeGO.layer = 2; // Ignore Raycast

        probe = probeGO.AddComponent<SphereCollider>();
        probe.isTrigger = true;
        probe.radius = Mathf.Max(1e-4f, nodeRadius);
    }

    void EnsureInitialized(bool force = false)
    {
        nodeCount = Mathf.Max(2, nodeCount);

        bool need =
            force ||
            x == null || v == null || invM == null ||
            x.Length != nodeCount || v.Length != nodeCount || invM.Length != nodeCount;

        if (!need) return;

        InitCable();
        _lastInitializedNodeCount = nodeCount;
    }

    void InitCable()
    {
        deployedLength = Mathf.Clamp(deployedLength, minLength, maxLength);
        targetDeployedLength = Mathf.Clamp(targetDeployedLength, minLength, maxLength);

        x = new Vector3[nodeCount];
        v = new Vector3[nodeCount];
        invM = new float[nodeCount];

        Vector3 p0 = topAnchor ? topAnchor.position : transform.position;
        Vector3 p1 = bottomAttach ? bottomAttach.position : (p0 + Vector3.down * deployedLength);

        for (int i = 0; i < nodeCount; i++)
        {
            float t = (float)i / (nodeCount - 1);
            x[i] = Vector3.Lerp(p0, p1, t);
            v[i] = Vector3.zero;
            invM[i] = 1f / Mathf.Max(1e-6f, massPerNode);
        }

        // 端点固定（アンカー追従）
        invM[0] = 0f;
        invM[nodeCount - 1] = 0f;

        segLen = deployedLength / (nodeCount - 1);

        if (lr)
        {
            lr.positionCount = nodeCount;
            UpdateLineRenderer();
        }
    }

    void FixedUpdate()
    {
        // ★ここが重要：nodeCount変更があっても必ず配列同期
        EnsureInitialized(force: false);

        if (overlapBuf == null || overlapBuf.Length != maxOverlapsPerNode)
            overlapBuf = new Collider[Mathf.Max(1, maxOverlapsPerNode)];

        AutoAssignBottomRigidbodyIfNeeded();

        float dt = Time.fixedDeltaTime;

        // ウインチ
        targetDeployedLength = Mathf.Clamp(targetDeployedLength, minLength, maxLength);
        deployedLength = Mathf.MoveTowards(deployedLength, targetDeployedLength, winchSpeed * dt);
        segLen = deployedLength / (nodeCount - 1);

        // 端点追従
        if (topAnchor) x[0] = topAnchor.position;
        if (bottomAttach) x[nodeCount - 1] = bottomAttach.position;

        // probe半径追従
        if (probe && Mathf.Abs(probe.radius - nodeRadius) > 1e-6f)
            probe.radius = Mathf.Max(1e-4f, nodeRadius);

        float h = dt / Mathf.Max(1, substeps);
        for (int s = 0; s < substeps; s++)
            Step(h);

        UpdateLineRenderer();

        // 張力反力（EA）
        lastTensionN = 0f;
        lastStretch = 0f;
        lastStraightDist = 0f;
        lastDirN = Vector3.zero;

        if (applyTetherToBottom && bottomRigidbody != null && topAnchor != null && bottomAttach != null)
            ApplyTetherEAForce(bottomRigidbody);
        else
            UpdateTetherStateOnly();
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

            v[i] = (v[i] + a * dt);
            v[i] *= (1f - Mathf.Clamp01(linearDamping));
            x[i] += v[i] * dt;
        }

        for (int it = 0; it < solverIterations; it++)
        {
            SolveDistanceConstraints();

            if (enableBending)
                SolveBendingConstraints(dt);

            if (enableCollision)
                SolveCollisions_NodeSpheres();

            if (topAnchor) x[0] = topAnchor.position;
            if (bottomAttach) x[nodeCount - 1] = bottomAttach.position;
        }

        for (int i = 0; i < nodeCount; i++)
        {
            if (invM[i] == 0f)
            {
                v[i] = Vector3.zero;
                continue;
            }

            v[i] = (x[i] - xPrev[i]) / dt;

            // ★制約由来の微振動を止める
            float cd = Mathf.Clamp01(constraintVelocityDamping);
            if (cd > 0f) v[i] *= (1f - cd);
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

    void SolveBendingConstraints(float dt)
    {
        float s = Mathf.Clamp01(bendStiffness);
        if (s <= 0f) return;

        if (usePhysicalEI)
        {
            float ds = Mathf.Max(1e-4f, segLen);
            float denom = Mathf.Max(1e-6f, massPerNode) * Mathf.Pow(ds, 4f);
            float scaled = (Mathf.Max(0f, bendingRigidityEI) * dt * dt) / denom;
            s *= Mathf.Clamp01(scaled);
        }

        float maxCorr = Mathf.Max(0f, bendMaxCorrection);

        for (int i = 1; i < nodeCount - 1; i++)
        {
            if (invM[i] == 0f) continue;

            Vector3 target = 0.5f * (x[i - 1] + x[i + 1]);
            Vector3 corr = (target - x[i]) * s;

            float mag = corr.magnitude;
            if (maxCorr > 0f && mag > maxCorr)
                corr *= (maxCorr / mag);

            x[i] += corr;
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
                    x[i] += dir * dist;
            }
        }
    }

    void UpdateLineRenderer()
    {
        if (!lr || x == null) return;
        if (lr.positionCount != x.Length) lr.positionCount = x.Length;

        for (int i = 0; i < x.Length; i++)
            lr.SetPosition(i, x[i]);
    }

    void UpdateTetherStateOnly()
    {
        if (topAnchor == null || bottomAttach == null) return;

        Vector3 d = bottomAttach.position - topAnchor.position;
        float dist = d.magnitude;
        if (dist < 1e-6f) return;

        lastStraightDist = dist;
        lastDirN = d / dist;

        float L0 = Mathf.Max(0.01f, deployedLength);
        lastStretch = dist - L0;
        lastTensionN = 0f;
    }

    public void ApplyTetherEAForce(Rigidbody rb)
    {
        Vector3 anchorPos = topAnchor.position;
        Vector3 attachPos = bottomAttach.position;

        Vector3 d = attachPos - anchorPos;
        float dist = d.magnitude;
        if (dist < 1e-6f) return;

        Vector3 n = d / dist;

        lastStraightDist = dist;
        lastDirN = n;

        float L0 = Mathf.Max(0.01f, deployedLength);
        float stretch = dist - L0;
        lastStretch = stretch;

        float effectiveStretch = stretch - Mathf.Max(0f, slackMeters);
        if (effectiveStretch <= 0f)
        {
            lastTensionN = 0f;
            return;
        }

        if (maxStretchMeters > 0f)
            effectiveStretch = Mathf.Min(effectiveStretch, maxStretchMeters);

        float k = Mathf.Max(0f, axialRigidityEA) / L0;

        float vOut = Vector3.Dot(rb.GetPointVelocity(attachPos), n);
        float damp = Mathf.Max(0f, axialDamping) * Mathf.Max(0f, vOut);

        float tension = k * effectiveStretch + damp;
        tension = Mathf.Clamp(tension, 0f, Mathf.Max(0f, maxTensionNewton));

        rb.AddForceAtPosition(-n * tension, attachPos, ForceMode.Force);

        lastTensionN = tension;
    }

    // ---- Public API（ソナー側が呼ぶ） ----

    public int GetNodeCount()
    {
        EnsureInitialized(force: false);
        return (x != null) ? x.Length : Mathf.Max(2, nodeCount);
    }

    public Vector3 GetNodePosition(int i)
    {
        EnsureInitialized(force: false);

        if (x == null || x.Length == 0)
            return (topAnchor ? topAnchor.position : transform.position);

        if (i <= 0) return x[0];
        if (i >= x.Length) return x[x.Length - 1];
        return x[i];
    }

    public float GetTensionNewton() => lastTensionN;
    public float GetStraightDistanceMeters() => lastStraightDist;
    public float GetStretchMeters() => lastStretch;

    void OnValidate()
    {
        nodeCount = Mathf.Max(2, nodeCount);
        minLength = Mathf.Max(0.01f, minLength);
        maxLength = Mathf.Max(minLength, maxLength);

        deployedLength = Mathf.Clamp(deployedLength, minLength, maxLength);
        targetDeployedLength = Mathf.Clamp(targetDeployedLength, minLength, maxLength);

        massPerNode = Mathf.Max(1e-4f, massPerNode);
        nodeRadius = Mathf.Max(1e-4f, nodeRadius);

        bendMaxCorrection = Mathf.Max(0f, bendMaxCorrection);
        bendingRigidityEI = Mathf.Max(0f, bendingRigidityEI);

        axialRigidityEA = Mathf.Max(0f, axialRigidityEA);
        axialDamping = Mathf.Max(0f, axialDamping);
        maxTensionNewton = Mathf.Max(0f, maxTensionNewton);
        slackMeters = Mathf.Max(0f, slackMeters);
        maxStretchMeters = Mathf.Max(0f, maxStretchMeters);

        maxOverlapsPerNode = Mathf.Clamp(maxOverlapsPerNode, 1, 32);
        solverIterations = Mathf.Clamp(solverIterations, 1, 80);
        substeps = Mathf.Clamp(substeps, 1, 12);
    }

    void OnDrawGizmosSelected()
    {
        if (!drawGizmos || x == null) return;
        float r = Mathf.Max(1e-4f, nodeRadius);
        for (int i = 0; i < x.Length; i++)
            Gizmos.DrawWireSphere(x[i], r);
    }
}
