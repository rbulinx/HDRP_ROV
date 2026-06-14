using UnityEngine;

/// <summary>
/// Stable CableXPBD implementation.
/// Mitigates stretch spikes around collision corners and applies normal gravity above the water surface.
/// </summary>
[DefaultExecutionOrder(-200)]
[RequireComponent(typeof(LineRenderer))]
public class CableXPBD : MonoBehaviour
{
    // -------------------------
    // Cable length and winch controls
    // -------------------------
    [Header("Nodes / Winch")]
    [Min(2)] public int nodeCount = 60;

    public float deployedLength = 10f;

    public float targetDeployedLength = 10f;

    public float minLength = 1f;
    public float maxLength = 80f;

    public float winchSpeed = 0.5f;
    public float winchStepMeters = 5f;
    public bool autoAdjustNodesWithLength = true;
    public float targetSegmentLength = 0.5f;

    float segLen;

    // -------------------------
    // Anchor endpoints
    // -------------------------
    [Header("Anchors")]
    public Transform topAnchor;
    public Transform bottomAttach;

    // -------------------------
    // Water surface handling
    // -------------------------
    [Header("Water Surface / Air (A: normal gravity above surface)")]
    public bool enableWaterSurface = true;

    public float waterLevelY = 0f;

    public Transform waterSurfaceTransform;

    public float waterSurfaceYOffset = 0f;

    public bool disableCurrentAboveSurface = true;

    // -------------------------
    // Node mass, damping, and hydrodynamic forces
    // -------------------------
    [Header("Mass / Damping (Cable Nodes)")]
    public float massPerNode = 0.2f;

    [Range(0f, 0.3f)] public float linearDamping = 0.05f;

    [Header("Underwater Forces (Cable Nodes)")]
    public float buoyancyFactor = 0.8f;

    public float dragLinear = 2.0f;
    public float dragQuadratic = 0.0f;

    public Vector3 currentVelocity = Vector3.zero;

    [Header("Current Load -> Tension")]
    public bool applyCurrentLoadToBottom = true;
    [Range(0f, 1f)] public float currentLoadBottomShare = 0.5f;
    public float currentTensionScale = 1f;
    public float maxCurrentTensionNewton = 1500f;

    [Header("Cable Buoyancy Load -> Bottom")]
    public bool applyCableBuoyancyLoadToBottom = true;
    [Range(0f, 1f)] public float cableBuoyancyBottomShare = 0.35f;
    public float cableBuoyancyLoadScale = 1f;
    public float maxCableBuoyancyLoadNewton = 300f;

    // -------------------------
    // XPBD solver settings
    // -------------------------
    [Header("Solver (XPBD)")]
    [Range(1, 12)] public int substeps = 6;
    [Range(1, 80)] public int solverIterations = 25;

    [Range(0f, 1f)] public float constraintVelocityDamping = 0.15f;

    // -------------------------
    // Axial distance constraint
    // -------------------------
    [Header("Axial (Distance Constraint)")]
    public bool enableDistanceConstraint = true;

    public float distanceCompliance = 0f;

    public bool useEAForDistanceCompliance = true;

    public float axialRigidityEA = 2.0e6f;

    // -------------------------
    // Bending constraint
    // -------------------------
    [Header("Bending (Curvature XPBD)")]
    public bool enableBending = true;

    public float bendingCompliance = 0.0f;

    public bool usePhysicalEI = true;

    public float bendingRigidityEI = 20.0f;

    public float bendingMaxCorrection = 0.0f;

    // -------------------------
    // Collision against scene colliders
    // -------------------------
    [Header("Collision (Node Spheres / Segment Capsules)")]
    public bool enableCollision = true;
    public bool useCapsuleCollision = true;
    [Tooltip("衝突解決を何ソルバー反復ごとに行うか。1=毎回、2〜4で軽量化。")]
    [Range(1, 8)] public int collisionIterationStride = 3;
    public float nodeRadius = 0.03f;
    public LayerMask collisionMask = ~0;
    [Range(1, 32)] public int maxOverlapsPerNode = 8;
    [Min(0)] public int ignoreCollisionSegmentsNearTopAnchor = 2;
    [Min(0)] public int ignoreCollisionSegmentsNearBottomAttach = 2;

    public float maxCollisionCorrection = 0.02f;

    [Range(0f, 1f)] public float collisionVelocityDamping = 0.35f;

    // -------------------------
    // Tension model
    // -------------------------
    public enum TensionMode
    {
        CableLengthEA_Stabilized,
        EndToEndLimit_NoThrust
    }

    [Header("Tension Mode")]
    public TensionMode tensionMode = TensionMode.EndToEndLimit_NoThrust;

    [Header("Tension (End-to-End Limit, No Thrust Command)")]
    public float endToEndStiffnessNPerM = 12000f;

    public float endToEndDampingNPerMps = 1800f;

    [Range(0f, 1f)] public float reelInSpringScale = 0.25f;

    [Header("Tension (EA) using Cable Length (Stabilized)")]
    public bool applyTensionToBottom = true;

    public Rigidbody bottomRigidbody;

    public float slackMeters = 0.02f;

    public float requireNearTautMeters = 0.20f;

    public float axialDamping = 500f;

    public float maxTensionNewton = 1500f;

    public float maxTensionRate = 4000f;

    public bool applyForceAtAttachPoint = false;
    public bool useBottomSegmentDirectionForTension = true;
    public bool useBottomSegmentConstraintTension = true;
    public float bottomSegmentConstraintTensionScale = 0.25f;
    public float maxBottomSegmentConstraintTensionNewton = 500f;

    public float tensionSmoothingHz = 8f;

    public float directionSmoothingHz = 10f;

    // -------------------------
    // Rendering and debug display
    // -------------------------
    [Header("Render")]
    public bool renderLine = true;

    [Header("Debug")]
    public bool drawGizmos = false;

    // Simulation state for node positions, velocities, and inverse masses.
    Vector3[] x;
    Vector3[] v;
    Vector3[] xPrev;
    float[] invM;

    // XPBD accumulated Lagrange multipliers.
    float[] lambdaDist;     // per segment
    Vector3[] lambdaBend;   // per internal node

    // Tracks nodes that touched collision this step so their velocity can be damped.
    bool[] collidedThisStep;

    LineRenderer lr;

    GameObject probeGO;
    SphereCollider probe;
    CapsuleCollider capsuleProbe;
    Collider[] overlapBuf;

    // Runtime measurements exposed to UI and other systems.
    float lastCableLength;
    float lastStretch;
    float lastTensionN;
    float lastCurrentTensionN;
    float lastCableBuoyancyLoadN;
    float lastBottomSegmentTensionN;
    Vector3 currentLoadOnBottom = Vector3.zero;
    Vector3 cableBuoyancyLoadOnBottom = Vector3.zero;

    float tensionFiltered = 0f;
    Vector3 dirFiltered = Vector3.forward;
    float lengthFiltered = 0f;

    void Awake()
    {
        lr = GetComponent<LineRenderer>();
        if (collisionIterationStride <= 0)
            collisionIterationStride = 3;
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

    float GetWaterLevelY()
    {
        if (!enableWaterSurface) return float.NegativeInfinity;
        if (waterSurfaceTransform != null) return waterSurfaceTransform.position.y + waterSurfaceYOffset;
        return waterLevelY;
    }

    // -------------------------
    // Initialization
    // -------------------------
    void EnsureInitialized(bool force)
    {
        ApplyAutoNodeCountForLength();
        nodeCount = Mathf.Max(2, nodeCount);

        bool need =
            force ||
            x == null || v == null || invM == null ||
            x.Length != nodeCount || v.Length != nodeCount || invM.Length != nodeCount ||
            xPrev == null || xPrev.Length != nodeCount ||
            lambdaDist == null || lambdaDist.Length != Mathf.Max(1, nodeCount - 1) ||
            lambdaBend == null || lambdaBend.Length != nodeCount ||
            collidedThisStep == null || collidedThisStep.Length != nodeCount;

        if (!need) return;

        if (!force && x != null && v != null && x.Length >= 2)
        {
            ResizeCablePreservingShape(nodeCount);
            return;
        }

        InitCable();
    }

    void ApplyAutoNodeCountForLength()
    {
        if (!autoAdjustNodesWithLength) return;

        float targetLen = Mathf.Clamp(deployedLength, minLength, maxLength);
        float segmentTarget = Mathf.Max(0.05f, targetSegmentLength);
        int desiredNodeCount = Mathf.Max(2, Mathf.CeilToInt(targetLen / segmentTarget) + 1);

        if (nodeCount == desiredNodeCount) return;
        nodeCount = desiredNodeCount;
    }

    void ResizeCablePreservingShape(int newNodeCount)
    {
        newNodeCount = Mathf.Max(2, newNodeCount);

        Vector3[] oldX = x;
        Vector3[] oldV = v;
        int oldCount = oldX != null ? oldX.Length : 0;

        x = new Vector3[newNodeCount];
        v = new Vector3[newNodeCount];
        xPrev = new Vector3[newNodeCount];
        invM = new float[newNodeCount];
        lambdaDist = new float[Mathf.Max(1, newNodeCount - 1)];
        lambdaBend = new Vector3[newNodeCount];
        collidedThisStep = new bool[newNodeCount];

        if (oldCount >= 2)
        {
            for (int i = 0; i < newNodeCount; i++)
            {
                float t = (float)i / (newNodeCount - 1);
                x[i] = SampleOldValuesByIndex(oldX, t);
                v[i] = SampleOldValuesByIndex(oldV, t);
                invM[i] = 1f / Mathf.Max(1e-6f, massPerNode);
            }
        }
        else
        {
            Vector3 p0 = topAnchor ? topAnchor.position : transform.position;
            Vector3 p1 = bottomAttach ? bottomAttach.position : (p0 + Vector3.down * deployedLength);

            for (int i = 0; i < newNodeCount; i++)
            {
                float t = (float)i / (newNodeCount - 1);
                x[i] = Vector3.Lerp(p0, p1, t);
                v[i] = Vector3.zero;
                invM[i] = 1f / Mathf.Max(1e-6f, massPerNode);
            }
        }

        invM[0] = 0f;
        invM[newNodeCount - 1] = 0f;

        if (topAnchor) x[0] = topAnchor.position;
        if (bottomAttach) x[newNodeCount - 1] = bottomAttach.position;

        segLen = deployedLength / (newNodeCount - 1);
        lastCableLength = ComputeCableLength();
        lengthFiltered = lastCableLength;
        dirFiltered = ComputeEndToEndDirSafe();

        if (lr)
        {
            lr.positionCount = newNodeCount;
            if (renderLine) UpdateLineRenderer();
        }
    }

    static Vector3 SampleOldValuesByIndex(Vector3[] oldValues, float normalizedIndex)
    {
        if (oldValues == null || oldValues.Length == 0) return Vector3.zero;
        if (oldValues.Length == 1) return oldValues[0];

        float f = Mathf.Clamp01(normalizedIndex) * (oldValues.Length - 1);
        int i0 = Mathf.FloorToInt(f);
        int i1 = Mathf.Min(i0 + 1, oldValues.Length - 1);
        float t = f - i0;
        return Vector3.Lerp(oldValues[i0], oldValues[i1], t);
    }


    void InitCable()
    {
        deployedLength = Mathf.Clamp(deployedLength, minLength, maxLength);
        targetDeployedLength = Mathf.Clamp(targetDeployedLength, minLength, maxLength);

        x = new Vector3[nodeCount];
        v = new Vector3[nodeCount];
        xPrev = new Vector3[nodeCount];
        invM = new float[nodeCount];

        lambdaDist = new float[Mathf.Max(1, nodeCount - 1)];
        lambdaBend = new Vector3[nodeCount];
        collidedThisStep = new bool[nodeCount];

        Vector3 p0 = topAnchor ? topAnchor.position : transform.position;
        Vector3 p1 = bottomAttach ? bottomAttach.position : (p0 + Vector3.down * deployedLength);

        for (int i = 0; i < nodeCount; i++)
        {
            float t = (float)i / (nodeCount - 1);
            x[i] = Vector3.Lerp(p0, p1, t);
            v[i] = Vector3.zero;
            invM[i] = 1f / Mathf.Max(1e-6f, massPerNode);
            collidedThisStep[i] = false;
        }

        // Pin both endpoints; their positions are driven by the anchors.
        invM[0] = 0f;
        invM[nodeCount - 1] = 0f;

        segLen = deployedLength / (nodeCount - 1);

        lastCableLength = ComputeCableLength();
        lengthFiltered = lastCableLength;
        tensionFiltered = 0f;

        dirFiltered = ComputeEndToEndDirSafe();

        if (lr)
        {
            lr.positionCount = nodeCount;
            if (renderLine) UpdateLineRenderer();
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

        probeGO = new GameObject("CableXPBD_Probe");
        probeGO.hideFlags = HideFlags.HideAndDontSave;
        probeGO.layer = 2; // Ignore Raycast

        probe = probeGO.AddComponent<SphereCollider>();
        probe.isTrigger = true;
        probe.radius = Mathf.Max(1e-4f, nodeRadius);

        capsuleProbe = probeGO.AddComponent<CapsuleCollider>();
        capsuleProbe.isTrigger = true;
        capsuleProbe.direction = 1; // local Y axis
        capsuleProbe.enabled = false;
        capsuleProbe.radius = Mathf.Max(1e-4f, nodeRadius);
        capsuleProbe.height = Mathf.Max(capsuleProbe.radius * 2f, capsuleProbe.radius * 2f + 1e-4f);
    }

    // -------------------------
    // Fixed-step simulation loop
    // -------------------------
    void FixedUpdate()
    {
        EnsureInitialized(force: false);

        if (overlapBuf == null || overlapBuf.Length != maxOverlapsPerNode)
            overlapBuf = new Collider[Mathf.Max(1, maxOverlapsPerNode)];

        AutoAssignBottomRigidbodyIfNeeded();

        float dt = Time.fixedDeltaTime;

        targetDeployedLength = Mathf.Clamp(targetDeployedLength, minLength, maxLength);
        deployedLength = Mathf.MoveTowards(deployedLength, targetDeployedLength, winchSpeed * dt);
        EnsureInitialized(force: false);
        segLen = deployedLength / (nodeCount - 1);

        if (topAnchor) x[0] = topAnchor.position;
        if (bottomAttach) x[nodeCount - 1] = bottomAttach.position;

        if (probe && Mathf.Abs(probe.radius - nodeRadius) > 1e-6f)
            probe.radius = Mathf.Max(1e-4f, nodeRadius);

        float h = dt / Mathf.Max(1, substeps);
        for (int s = 0; s < substeps; s++)
            Step(h);

        if (renderLine) UpdateLineRenderer();

        lastCableLength = ComputeCableLength();
        lastStretch = lastCableLength - deployedLength;
        currentLoadOnBottom = EstimateCurrentLoadOnBottom();
        lastCurrentTensionN = currentLoadOnBottom.magnitude;
        cableBuoyancyLoadOnBottom = EstimateCableBuoyancyLoadOnBottom();
        lastBottomSegmentTensionN = EstimateBottomSegmentConstraintTension(h);

        lastTensionN = 0f;
        if (applyTensionToBottom && bottomRigidbody != null && topAnchor != null && bottomAttach != null)
        {
            if (tensionMode == TensionMode.CableLengthEA_Stabilized)
                ApplyTensionEA_Stabilized(bottomRigidbody, dt);
            else
                ApplyTension_EndToEndLimit_NoThrust(bottomRigidbody, dt);
        }
        else
        {
            tensionFiltered = 0f;
            Vector3 F = currentLoadOnBottom + cableBuoyancyLoadOnBottom;
            if (bottomRigidbody != null && bottomAttach != null && F.sqrMagnitude > 1e-8f)
            {
                if (applyForceAtAttachPoint)
                    bottomRigidbody.AddForceAtPosition(F, bottomAttach.position, ForceMode.Force);
                else
                    bottomRigidbody.AddForce(F, ForceMode.Force);
            }

            lastTensionN = Mathf.Sqrt(lastCurrentTensionN * lastCurrentTensionN + lastCableBuoyancyLoadN * lastCableBuoyancyLoadN);
        }
    }

    // -------------------------
    // Single XPBD substep
    // -------------------------
    void Step(float dt)
    {
        for (int i = 0; i < lambdaDist.Length; i++) lambdaDist[i] = 0f;
        for (int i = 0; i < lambdaBend.Length; i++) lambdaBend[i] = Vector3.zero;

        for (int i = 0; i < collidedThisStep.Length; i++) collidedThisStep[i] = false;

        if (xPrev == null || xPrev.Length != nodeCount)
            xPrev = new Vector3[nodeCount];

        float surfaceY = GetWaterLevelY();

        for (int i = 0; i < nodeCount; i++)
        {
            xPrev[i] = x[i];
            if (invM[i] == 0f) continue;

            bool isAboveSurface = enableWaterSurface && (x[i].y > surfaceY);

            // Above the surface the cable uses normal gravity; underwater it is buoyancy-reduced.
            Vector3 gEff = isAboveSurface
                ? Physics.gravity
                : Physics.gravity * (1f - buoyancyFactor);

            // Current is optionally disabled for cable nodes above the surface.
            Vector3 curVel = currentVelocity;
            if (isAboveSurface && disableCurrentAboveSurface) curVel = Vector3.zero;

            // Drag is computed from velocity relative to the water current.
            Vector3 vRel = v[i] - curVel;
            Vector3 drag = -dragLinear * vRel;
            if (dragQuadratic > 0f)
            {
                float sp = vRel.magnitude;
                drag += -dragQuadratic * sp * vRel;
            }

            Vector3 a = gEff + drag / Mathf.Max(1e-6f, massPerNode);

            v[i] = v[i] + a * dt;
            v[i] *= (1f - linearDamping);
            x[i] = x[i] + v[i] * dt;
        }

        for (int it = 0; it < solverIterations; it++)
        {
            // Re-solving distance constraints after collision helps suppress corner stretch spikes.
            if (enableDistanceConstraint)
                SolveDistanceXPBD(dt);

            if (enableBending)
                SolveBendingCurvatureXPBD(dt);

            if (enableCollision && ShouldSolveCollisionThisIteration(it))
                SolveCollisions();

            if (enableDistanceConstraint)
                SolveDistanceXPBD(dt);

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

            float cd = Mathf.Clamp01(constraintVelocityDamping);
            if (cd > 0f) v[i] *= (1f - cd);

            float cvd = Mathf.Clamp01(collisionVelocityDamping);
            if (cvd > 0f && collidedThisStep[i]) v[i] *= (1f - cvd);
        }
    }

    // -------------------------
    // XPBD distance constraint
    // -------------------------
    void SolveDistanceXPBD(float dt)
    {
        float compliance = Mathf.Max(0f, distanceCompliance);
        if (useEAForDistanceCompliance)
        {
            float EA = Mathf.Max(1e-6f, axialRigidityEA);
            // Approximation: k = EA / segmentLength, so compliance = segmentLength / EA.
            compliance = segLen / EA;
        }

        float alpha = compliance / Mathf.Max(1e-8f, dt * dt);

        for (int i = 0; i < nodeCount - 1; i++)
        {
            int j = i + 1;

            float w1 = invM[i];
            float w2 = invM[j];
            float wSum = w1 + w2;
            if (wSum <= 0f) continue;

            Vector3 d = x[j] - x[i];
            float dist = d.magnitude;
            if (dist < 1e-8f) continue;

            Vector3 n = d / dist;

            float C = dist - segLen;

            float dl = -(C + alpha * lambdaDist[i]) / (wSum + alpha);

            Vector3 corr = dl * n;
            if (w1 > 0f) x[i] -= w1 * corr;
            if (w2 > 0f) x[j] += w2 * corr;

            lambdaDist[i] += dl;
        }
    }

    // -------------------------
    // XPBD bending constraint
    // -------------------------
    void SolveBendingCurvatureXPBD(float dt)
    {
        float compliance = Mathf.Max(0f, bendingCompliance);

        if (usePhysicalEI)
        {
            float EI = Mathf.Max(1e-8f, bendingRigidityEI);
            float ds = Mathf.Max(1e-4f, segLen);
            // Simple curvature-compliance approximation based on segment length and EI.
            compliance = (ds * ds * ds * ds) / EI;
        }

        float alpha = compliance / Mathf.Max(1e-8f, dt * dt);

        for (int i = 1; i < nodeCount - 1; i++)
        {
            int im = i - 1;
            int ip = i + 1;

            float w_im = invM[im];
            float w_i = invM[i];
            float w_ip = invM[ip];

            float denom = (w_im + 4f * w_i + w_ip);
            if (denom <= 0f) continue;

            Vector3 C = x[im] - 2f * x[i] + x[ip];

            Vector3 dl = -(C + alpha * lambdaBend[i]) / (denom + alpha);

            Vector3 dx_im = w_im * dl;
            Vector3 dx_i = -2f * w_i * dl;
            Vector3 dx_ip = w_ip * dl;

            if (bendingMaxCorrection > 0f)
            {
                float maxC = bendingMaxCorrection;

                float m1 = dx_im.magnitude;
                if (m1 > maxC) dx_im *= (maxC / m1);

                float m2 = dx_i.magnitude;
                if (m2 > maxC) dx_i *= (maxC / m2);

                float m3 = dx_ip.magnitude;
                if (m3 > maxC) dx_ip *= (maxC / m3);
            }

            if (w_im > 0f) x[im] += dx_im;
            if (w_i > 0f) x[i] += dx_i;
            if (w_ip > 0f) x[ip] += dx_ip;

            lambdaBend[i] += dl;
        }
    }

    // -------------------------
    // Collision resolution
    // -------------------------
    void SolveCollisions()
    {
        if (useCapsuleCollision)
            SolveCollisions_SegmentCapsules();

        SolveCollisions_NodeSpheres();
    }

    bool ShouldSolveCollisionThisIteration(int iteration)
    {
        int stride = Mathf.Max(1, collisionIterationStride);
        return iteration == solverIterations - 1 || (iteration % stride) == stride - 1;
    }

    void SolveCollisions_NodeSpheres()
    {
        if (!probe) return;

        float r = Mathf.Max(1e-4f, nodeRadius);
        float maxCorr = Mathf.Max(0f, maxCollisionCorrection);

        for (int i = 0; i < nodeCount; i++)
        {
            if (invM[i] == 0f) continue;

            int hitCount = Physics.OverlapSphereNonAlloc(
                x[i], r, overlapBuf, collisionMask, QueryTriggerInteraction.Ignore);

            for (int h = 0; h < hitCount; h++)
            {
                Collider col = overlapBuf[h];
                if (!col) continue;
                if (ShouldIgnoreCollisionCollider(col, i, false)) continue;

                Vector3 dir;
                float dist;
                bool overlapped = Physics.ComputePenetration(
                    probe, x[i], Quaternion.identity,
                    col, col.transform.position, col.transform.rotation,
                    out dir, out dist);

                if (overlapped && dist > 0f)
                {
                    float d = dist;
                    if (maxCorr > 0f) d = Mathf.Min(d, maxCorr);
                    x[i] += dir * d;
                    collidedThisStep[i] = true;
                }
            }
        }
    }

    void SolveCollisions_SegmentCapsules()
    {
        if (!capsuleProbe) return;
        if (x == null || x.Length < 2) return;

        float r = Mathf.Max(1e-4f, nodeRadius);
        float maxCorr = Mathf.Max(0f, maxCollisionCorrection);

        capsuleProbe.enabled = true;
        capsuleProbe.radius = r;

        for (int i = 0; i < nodeCount - 1; i++)
        {
            int j = i + 1;

            float w0 = invM[i];
            float w1 = invM[j];
            float wSum = w0 + w1;
            if (wSum <= 0f) continue;

            Vector3 p0 = x[i];
            Vector3 p1 = x[j];
            Vector3 axis = p1 - p0;
            float len = axis.magnitude;
            if (len < 1e-6f) continue;

            int hitCount = Physics.OverlapCapsuleNonAlloc(
                p0, p1, r, overlapBuf, collisionMask, QueryTriggerInteraction.Ignore);

            if (hitCount <= 0) continue;

            Vector3 mid = (p0 + p1) * 0.5f;
            Quaternion rot = Quaternion.FromToRotation(Vector3.up, axis / len);
            capsuleProbe.height = Mathf.Max(r * 2f, len + r * 2f);

            for (int h = 0; h < hitCount; h++)
            {
                Collider col = overlapBuf[h];
                if (!col) continue;
                if (ShouldIgnoreCollisionCollider(col, i, true)) continue;

                Vector3 dir;
                float dist;
                bool overlapped = Physics.ComputePenetration(
                    capsuleProbe, mid, rot,
                    col, col.transform.position, col.transform.rotation,
                    out dir, out dist);

                if (!overlapped || dist <= 0f) continue;

                float d = dist;
                if (maxCorr > 0f) d = Mathf.Min(d, maxCorr);

                Vector3 corr = dir * d;
                x[i] += corr * (w0 / wSum);
                x[j] += corr * (w1 / wSum);

                if (w0 > 0f) collidedThisStep[i] = true;
                if (w1 > 0f) collidedThisStep[j] = true;
            }
        }

        capsuleProbe.enabled = false;
    }

    bool ShouldIgnoreCollisionCollider(Collider col, int cableIndex, bool indexIsSegment)
    {
        if (col == null) return true;
        if (probe != null && col == probe) return true;
        if (capsuleProbe != null && col == capsuleProbe) return true;

        Rigidbody attachedRb = col.attachedRigidbody;
        Transform colTransform = col.transform;
        if (colTransform == null) return false;

        if (colTransform == transform || colTransform.IsChildOf(transform))
            return true;

        bool nearTop = IsCableIndexNearTopAnchor(cableIndex, indexIsSegment);
        bool nearBottom = IsCableIndexNearBottomAttach(cableIndex, indexIsSegment);

        if (nearBottom && attachedRb != null && bottomRigidbody != null && attachedRb == bottomRigidbody)
            return true;

        if (nearBottom && bottomAttach != null && (colTransform == bottomAttach || colTransform.IsChildOf(bottomAttach)))
            return true;

        if (nearTop && topAnchor != null && (colTransform == topAnchor || colTransform.IsChildOf(topAnchor)))
            return true;

        return false;
    }

    bool IsCableIndexNearTopAnchor(int cableIndex, bool indexIsSegment)
    {
        int ignore = Mathf.Max(0, ignoreCollisionSegmentsNearTopAnchor);
        if (ignore <= 0) return false;
        return indexIsSegment ? cableIndex < ignore : cableIndex <= ignore;
    }

    bool IsCableIndexNearBottomAttach(int cableIndex, bool indexIsSegment)
    {
        int ignore = Mathf.Max(0, ignoreCollisionSegmentsNearBottomAttach);
        if (ignore <= 0) return false;

        int lastSegment = Mathf.Max(0, nodeCount - 2);
        int lastNode = Mathf.Max(0, nodeCount - 1);
        return indexIsSegment
            ? cableIndex >= lastSegment - ignore + 1
            : cableIndex >= lastNode - ignore;
    }

    Vector3 EstimateCurrentLoadOnBottom()
    {
        if (!applyCurrentLoadToBottom) return Vector3.zero;
        if (x == null || v == null || x.Length < 2) return Vector3.zero;
        if (currentVelocity.sqrMagnitude <= 1e-8f) return Vector3.zero;

        float surfaceY = GetWaterLevelY();
        Vector3 totalDrag = Vector3.zero;

        // Sum the drag on the cable and pass a configurable share to the bottom body.
        for (int i = 0; i < x.Length; i++)
        {
            bool isAboveSurface = enableWaterSurface && (x[i].y > surfaceY);
            if (isAboveSurface && disableCurrentAboveSurface)
                continue;

            Vector3 vRel = v[i] - currentVelocity;
            Vector3 drag = -dragLinear * vRel;
            if (dragQuadratic > 0f)
            {
                float sp = vRel.magnitude;
                drag += -dragQuadratic * sp * vRel;
            }

            float weight = (i == 0 || i == x.Length - 1) ? 0.5f : 1f;
            totalDrag += drag * weight;
        }

        Vector3 load = totalDrag * Mathf.Clamp01(currentLoadBottomShare) * Mathf.Max(0f, currentTensionScale);
        float maxLoad = Mathf.Max(0f, maxCurrentTensionNewton);
        if (maxLoad > 0f && load.magnitude > maxLoad)
            load = load.normalized * maxLoad;

        return load;
    }

    Vector3 EstimateCableBuoyancyLoadOnBottom()
    {
        lastCableBuoyancyLoadN = 0f;

        if (!applyCableBuoyancyLoadToBottom) return Vector3.zero;
        if (x == null || x.Length < 2) return Vector3.zero;

        float surfaceY = GetWaterLevelY();
        float totalWeightedNodes = 0f;

        for (int i = 0; i < x.Length; i++)
        {
            bool isAboveSurface = enableWaterSurface && (x[i].y > surfaceY);
            if (isAboveSurface)
                continue;

            float weight = (i == 0 || i == x.Length - 1) ? 0.5f : 1f;
            totalWeightedNodes += weight;
        }

        if (totalWeightedNodes <= 0f) return Vector3.zero;

        Vector3 effectiveCableWeight = Physics.gravity * massPerNode * totalWeightedNodes * (1f - buoyancyFactor);
        Vector3 load = effectiveCableWeight * Mathf.Clamp01(cableBuoyancyBottomShare) * Mathf.Max(0f, cableBuoyancyLoadScale);

        float maxLoad = Mathf.Max(0f, maxCableBuoyancyLoadNewton);
        if (maxLoad > 0f && load.magnitude > maxLoad)
            load = load.normalized * maxLoad;

        lastCableBuoyancyLoadN = load.magnitude;
        return load;
    }

    // -------------------------
    // Stabilized tension application
    // -------------------------
    void ApplyTension_EndToEndLimit_NoThrust(Rigidbody rb, float dt)
    {
        float L0 = Mathf.Max(0.01f, deployedLength);
        float limit = L0 + Mathf.Max(0f, slackMeters);

        float endDist = Vector3.Distance(topAnchor.position, bottomAttach.position);

        Vector3 dir = ComputeTensionDirectionSafe();

        float aDir = 1f - Mathf.Exp(-dt * Mathf.Max(0.1f, directionSmoothingHz));
        dirFiltered = Vector3.Slerp(dirFiltered, dir, aDir);

        float stretch = Mathf.Max(0f, endDist - limit);

        float tensionTarget = 0f;

        if (stretch > 0f)
        {
            float k = Mathf.Max(0f, endToEndStiffnessNPerM);
            float spring = k * stretch;

            float vOut = 0f;
            if (applyForceAtAttachPoint)
                vOut = Vector3.Dot(rb.GetPointVelocity(bottomAttach.position), dirFiltered);
            else
                vOut = Vector3.Dot(rb.linearVelocity, dirFiltered);

            float damp = 0f;
            if (vOut > 0f)
                damp = Mathf.Max(0f, endToEndDampingNPerMps) * vOut;

            float springScale = (vOut < 0f) ? Mathf.Clamp01(reelInSpringScale) : 1f;

            tensionTarget = springScale * spring + damp;
            tensionTarget = Mathf.Clamp(tensionTarget, 0f, Mathf.Max(0f, maxTensionNewton));
        }

        tensionTarget = Mathf.Max(tensionTarget, lastBottomSegmentTensionN);

        float maxStep = Mathf.Max(0f, maxTensionRate) * dt;
        tensionFiltered = Mathf.MoveTowards(tensionFiltered, tensionTarget, maxStep);

        float aTen = 1f - Mathf.Exp(-dt * Mathf.Max(0.1f, tensionSmoothingHz));
        tensionFiltered = Mathf.Lerp(tensionFiltered, tensionTarget, aTen);

        // Tension pulls toward the top anchor; distributed cable loads add their bottom reaction.
        Vector3 F = -dirFiltered * tensionFiltered + currentLoadOnBottom + cableBuoyancyLoadOnBottom;

        if (applyForceAtAttachPoint)
            rb.AddForceAtPosition(F, bottomAttach.position, ForceMode.Force);
        else
            rb.AddForce(F, ForceMode.Force);

        lastTensionN = Mathf.Sqrt(tensionFiltered * tensionFiltered + lastCurrentTensionN * lastCurrentTensionN + lastCableBuoyancyLoadN * lastCableBuoyancyLoadN);
    }

    void ApplyTensionEA_Stabilized(Rigidbody rb, float dt)
    {
        float L0 = Mathf.Max(0.01f, deployedLength);

        float endDist = Vector3.Distance(topAnchor.position, bottomAttach.position);

        bool nearTaut = endDist >= (L0 - Mathf.Max(0f, requireNearTautMeters));

        float aLen = 1f - Mathf.Exp(-dt * Mathf.Max(0.1f, tensionSmoothingHz));
        lengthFiltered = Mathf.Lerp(lengthFiltered, lastCableLength, aLen);

        float stretch = (lengthFiltered - L0);
        float effectiveStretch = stretch - Mathf.Max(0f, slackMeters);

        float tensionTarget = 0f;

        if (nearTaut && effectiveStretch > 0f)
        {
            float EA = Mathf.Max(0f, axialRigidityEA);
            float k = EA / L0;

            Vector3 dir = ComputeTensionDirectionSafe();

            float aDir = 1f - Mathf.Exp(-dt * Mathf.Max(0.1f, directionSmoothingHz));
            dirFiltered = Vector3.Slerp(dirFiltered, dir, aDir);

            Vector3 velPoint = rb.GetPointVelocity(bottomAttach.position);
            float vOut = Vector3.Dot(velPoint, dirFiltered);

            float damp = Mathf.Max(0f, axialDamping) * Mathf.Max(0f, vOut);

            tensionTarget = k * effectiveStretch + damp;
            tensionTarget = Mathf.Clamp(tensionTarget, 0f, Mathf.Max(0f, maxTensionNewton));
        }

        tensionTarget = Mathf.Max(tensionTarget, lastBottomSegmentTensionN);

        float maxStep = Mathf.Max(0f, maxTensionRate) * dt;
        tensionFiltered = Mathf.MoveTowards(tensionFiltered, tensionTarget, maxStep);

        float aTen = 1f - Mathf.Exp(-dt * Mathf.Max(0.1f, tensionSmoothingHz));
        tensionFiltered = Mathf.Lerp(tensionFiltered, tensionTarget, aTen);

        // Tension pulls toward the top anchor; distributed cable loads add their bottom reaction.
        Vector3 F = -dirFiltered * tensionFiltered + currentLoadOnBottom + cableBuoyancyLoadOnBottom;

        if (applyForceAtAttachPoint)
            rb.AddForceAtPosition(F, bottomAttach.position, ForceMode.Force);
        else
            rb.AddForce(F, ForceMode.Force);

        lastTensionN = Mathf.Sqrt(tensionFiltered * tensionFiltered + lastCurrentTensionN * lastCurrentTensionN + lastCableBuoyancyLoadN * lastCableBuoyancyLoadN);
    }

    Vector3 ComputeEndToEndDirSafe()
    {
        if (topAnchor == null || bottomAttach == null) return Vector3.forward;

        Vector3 d = (bottomAttach.position - topAnchor.position);
        float m = d.magnitude;
        if (m > 1e-6f) return d / m;
        return Vector3.forward;
    }

    Vector3 ComputeTensionDirectionSafe()
    {
        if (!useBottomSegmentDirectionForTension)
            return ComputeEndToEndDirSafe();

        if (x != null && x.Length >= 2 && bottomAttach != null)
        {
            Vector3 d = bottomAttach.position - x[x.Length - 2];
            float m = d.magnitude;
            if (m > 1e-6f) return d / m;
        }

        return ComputeEndToEndDirSafe();
    }

    float EstimateBottomSegmentConstraintTension(float substepDt)
    {
        if (!useBottomSegmentConstraintTension) return 0f;
        if (lambdaDist == null || lambdaDist.Length == 0 || nodeCount < 2) return 0f;

        int lastSegment = Mathf.Clamp(nodeCount - 2, 0, lambdaDist.Length - 1);
        float lambda = lambdaDist[lastSegment];

        // In this solver, a stretched segment accumulates negative lambda.
        float tension = Mathf.Max(0f, -lambda) / Mathf.Max(1e-8f, substepDt * substepDt);
        tension *= Mathf.Max(0f, bottomSegmentConstraintTensionScale);

        float maxTension = Mathf.Max(0f, maxBottomSegmentConstraintTensionNewton);
        if (maxTension > 0f)
            tension = Mathf.Min(tension, maxTension);

        return tension;
    }

    // -------------------------
    // Utility methods
    // -------------------------
    float ComputeCableLength()
    {
        float L = 0f;
        for (int i = 0; i < nodeCount - 1; i++)
            L += (x[i + 1] - x[i]).magnitude;
        return L;
    }

    void UpdateLineRenderer()
    {
        if (!lr || x == null) return;
        if (lr.positionCount != x.Length) lr.positionCount = x.Length;
        lr.SetPositions(x);
    }

    // -------------------------
    // Public API for UI, sonar, and other systems
    // -------------------------
    public int GetNodeCount()
    {
        EnsureInitialized(force: false);
        return x != null ? x.Length : Mathf.Max(2, nodeCount);
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

    public float GetCableLengthMeters() => lastCableLength;
    public float GetTensionNewton() => lastTensionN;
    public float GetCurrentTensionNewton() => lastCurrentTensionN;
    public float GetCableBuoyancyLoadNewton() => lastCableBuoyancyLoadN;
    public float GetBottomSegmentTensionNewton() => lastBottomSegmentTensionN;
    public float GetStretchMeters() => lastStretch;

    public void ReelOutStep()
    {
        SetTargetDeployedLength(targetDeployedLength + winchStepMeters);
    }

    public void ReelInStep()
    {
        SetTargetDeployedLength(targetDeployedLength - winchStepMeters);
    }

    public void SetTargetDeployedLength(float lengthMeters)
    {
        targetDeployedLength = Mathf.Clamp(lengthMeters, minLength, maxLength);
    }

    // -------------------------
    // Inspector validation
    // -------------------------
    void OnValidate()
    {
        nodeCount = Mathf.Max(2, nodeCount);
        minLength = Mathf.Max(0.01f, minLength);
        maxLength = Mathf.Max(minLength, maxLength);

        deployedLength = Mathf.Clamp(deployedLength, minLength, maxLength);
        targetDeployedLength = Mathf.Clamp(targetDeployedLength, minLength, maxLength);

        massPerNode = Mathf.Max(1e-4f, massPerNode);
        nodeRadius = Mathf.Max(1e-4f, nodeRadius);
        targetSegmentLength = Mathf.Max(0.05f, targetSegmentLength);
        ApplyAutoNodeCountForLength();

        solverIterations = Mathf.Clamp(solverIterations, 1, 80);
        substeps = Mathf.Clamp(substeps, 1, 12);
        maxOverlapsPerNode = Mathf.Clamp(maxOverlapsPerNode, 1, 32);
        collisionIterationStride = Mathf.Clamp(collisionIterationStride, 1, 8);
        ignoreCollisionSegmentsNearTopAnchor = Mathf.Max(0, ignoreCollisionSegmentsNearTopAnchor);
        ignoreCollisionSegmentsNearBottomAttach = Mathf.Max(0, ignoreCollisionSegmentsNearBottomAttach);

        axialRigidityEA = Mathf.Max(0f, axialRigidityEA);
        bendingRigidityEI = Mathf.Max(0f, bendingRigidityEI);

        distanceCompliance = Mathf.Max(0f, distanceCompliance);
        bendingCompliance = Mathf.Max(0f, bendingCompliance);

        slackMeters = Mathf.Max(0f, slackMeters);
        requireNearTautMeters = Mathf.Max(0f, requireNearTautMeters);

        axialDamping = Mathf.Max(0f, axialDamping);
        maxTensionNewton = Mathf.Max(0f, maxTensionNewton);
        maxTensionRate = Mathf.Max(0f, maxTensionRate);
        bottomSegmentConstraintTensionScale = Mathf.Max(0f, bottomSegmentConstraintTensionScale);
        maxBottomSegmentConstraintTensionNewton = Mathf.Max(0f, maxBottomSegmentConstraintTensionNewton);

        tensionSmoothingHz = Mathf.Max(0.1f, tensionSmoothingHz);
        directionSmoothingHz = Mathf.Max(0.1f, directionSmoothingHz);
        bendingMaxCorrection = Mathf.Max(0f, bendingMaxCorrection);

        maxCollisionCorrection = Mathf.Max(0f, maxCollisionCorrection);
        collisionVelocityDamping = Mathf.Clamp01(collisionVelocityDamping);
        currentLoadBottomShare = Mathf.Clamp01(currentLoadBottomShare);
        currentTensionScale = Mathf.Max(0f, currentTensionScale);
        maxCurrentTensionNewton = Mathf.Max(0f, maxCurrentTensionNewton);
        cableBuoyancyBottomShare = Mathf.Clamp01(cableBuoyancyBottomShare);
        cableBuoyancyLoadScale = Mathf.Max(0f, cableBuoyancyLoadScale);
        maxCableBuoyancyLoadNewton = Mathf.Max(0f, maxCableBuoyancyLoadNewton);
        winchStepMeters = Mathf.Max(0.1f, winchStepMeters);

        if (winchSpeed < 0f) winchSpeed = 0f;
    }

    void OnDrawGizmosSelected()
    {
        if (!drawGizmos || x == null) return;
        float r = Mathf.Max(1e-4f, nodeRadius);
        for (int i = 0; i < x.Length; i++)
            Gizmos.DrawWireSphere(x[i], r);
    }
}
