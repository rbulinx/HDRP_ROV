using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// CableXPBD variant with multiple cylindrical float sections.
/// Existing CableXPBD is left unchanged; this class adds discrete float buoyancy, drag, added mass, and visuals.
/// </summary>
[DefaultExecutionOrder(-200)]
[RequireComponent(typeof(LineRenderer))]
public partial class CableXPBD_MultiFloat : MonoBehaviour
{
    [Serializable]
    public class FloatSection
    {
        public bool enabled = true;
        public string name = "Float Section";

        [Range(0f, 1f)] public float startNormalized = 0.3f;
        [Range(0f, 1f)] public float endNormalized = 0.6f;
        [Min(0.01f)] public float spacingMeters = 1f;

        [Header("Cylinder")]
        [Min(0.001f)] public float diameter = 0.18f;
        [Min(0.001f)] public float length = 0.35f;
        [Min(0f)] public float massKg = 0.05f;

        [Header("Hydrodynamics")]
        public float buoyancyScale = 1f;
        public float normalDragCd = 1.1f;
        public float axialDragCd = 0.2f;
        public float normalAddedMassCa = 1.0f;
        public float axialAddedMassCa = 0.1f;

        [Header("Visual")]
        public Color color = Color.yellow;
        public Material material;
        [Min(0.01f)] public float visualScale = 1.2f;
        public GameObject prefab;
    }

    class FloatInstance
    {
        public FloatSection section;
        public float normalizedPosition;
        public Transform visual;
        public Renderer renderer;
        public Material runtimeMaterial;
        public MeshCollider sonarMeshCollider;
        public CapsuleCollider sonarFallbackCollider;
        public Vector3 smoothedForce;
    }

    public enum HydrodynamicDragModel
    {
        LegacyCoefficients,
        Morison
    }

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

    [Range(0f, 0.3f)] public float linearDamping = 0.10f;

    [Header("Underwater Forces (Cable Nodes)")]
    public float buoyancyFactor = 0.8f;

    [Tooltip("LegacyCoefficients keeps the previous linear/quadratic coefficients. Morison uses cable diameter, water density, and Cd/Ct.")]
    public HydrodynamicDragModel hydrodynamicDragModel = HydrodynamicDragModel.LegacyCoefficients;

    [Tooltip("Linear drag along the cable tangent. Usually much smaller than crossflow drag.")]
    public float dragLinearAlong = 0.05f;

    [Tooltip("Linear drag perpendicular to the cable tangent.")]
    public float dragLinearAcross = 0.3f;

    [Tooltip("Quadratic drag along the cable tangent.")]
    public float dragQuadraticAlong = 0.0f;

    [Tooltip("Quadratic drag perpendicular to the cable tangent.")]
    public float dragQuadraticAcross = 0.0f;

    [Header("Morison Drag (Cable Nodes)")]
    [Tooltip("Sea water density used by the Morison drag term.")]
    public float morisonWaterDensity = 1025f;

    [Tooltip("Tether outside diameter in meters.")]
    public float morisonCableDiameter = 0.03f;

    [Tooltip("Normal drag coefficient Cd for cross-flow over the tether.")]
    public float morisonNormalDragCoefficient = 1.2f;

    [Tooltip("Tangential skin/friction coefficient used for flow along the tether.")]
    public float morisonTangentialDragCoefficient = 0.02f;

    [Tooltip("Scale multiplier for the final Morison drag force.")]
    public float morisonDragScale = 1f;

    [Tooltip("Caps hydrodynamic acceleration on each cable node to keep explicit drag integration stable. 0 disables the cap.")]
    public float maxHydrodynamicAcceleration = 40f;

    [Tooltip("Caps cable node speed after force integration to prevent one-frame explosions. 0 disables the cap.")]
    public float maxCableNodeSpeed = 5f;

    public Vector3 currentVelocity = Vector3.zero;

    [Header("Cylinder Floats")]
    public bool enableFloatSections = true;
    public List<FloatSection> floatSections = new List<FloatSection>();
    public float floatWaterDensity = 1025f;
    public float floatGravity = 9.80665f;
    public float maxFloatAcceleration = 80f;
    public float maxFloatForce = 250f;
    [Range(0f, 1f)] public float floatForceSmoothing = 0.2f;
    public Transform floatVisualRoot;
    public bool enableFloatCollision = true;
    public float floatCollisionRadiusScale = 1.05f;
    public float maxFloatCollisionCorrection = 0.12f;
    const string FloatVisualRootName = "Cable Float Visuals";

    [Header("Float Sonar Colliders")]
    public bool enableFloatSonarColliders = true;
    public bool useFloatMeshSonarColliders = true;
    public bool floatSonarCollidersAreTrigger = false;
    public float floatSonarColliderScale = 1.0f;
    [Tooltip("Optional single layer for generated float sonar colliders.")]
    public LayerMask floatSonarLayer = 1 << 0;

    [Header("Cable Sonar Colliders")]
    public bool enableCableSonarColliders = true;
    public bool useCableMeshSonarColliders = false;
    public bool cableSonarCollidersAreTrigger = false;
    public float cableSonarRadius = 0.018f;
    public float cableSonarEndOverlap = 0f;
    public LayerMask cableSonarLayer = 1 << 0;
    [Range(1, 12)] public int sonarColliderUpdateStride = 3;

    [Header("Current Load -> Tension")]
    public bool applyCurrentLoadToBottom = true;
    public float maxCurrentTensionNewton = 1500f;

    [Header("Cable Buoyancy Load -> Bottom")]
    public bool applyCableBuoyancyLoadToBottom = true;
    public float maxCableBuoyancyLoadNewton = 300f;

    // -------------------------
    // XPBD solver settings
    // -------------------------
    [Header("Solver (XPBD)")]
    [Range(1, 12)] public int substeps = 6;
    [Range(1, 80)] public int solverIterations = 25;

    [Range(0f, 1f)] public float constraintVelocityDamping = 0.30f;

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

    [Range(0f, 1f)] public float collisionVelocityDamping = 0.70f;

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
    public float endToEndStiffnessNPerM = 3000f;

    public float endToEndDampingNPerMps = 1000f;

    [Range(0f, 1f)] public float reelInSpringScale = 0.25f;

    [Header("Tension (EA) using Cable Length (Stabilized)")]
    public bool applyTensionToBottom = true;

    public Rigidbody bottomRigidbody;

    public float slackMeters = 0.10f;

    public float requireNearTautMeters = 0.20f;

    public float axialDamping = 600f;

    public float maxTensionNewton = 400f;

    public float maxTensionRate = 800f;

    [Tooltip("ON: cable load creates torque around the attach point. OFF: apply through the Rigidbody center so light tether tension does not rotate the ROV.")]
    public bool applyForceAtAttachPoint = false;

    public bool useBottomSegmentDirectionForTension = true;
    public bool useBottomSegmentConstraintTension = false;
    public float maxBottomSegmentConstraintTensionNewton = 250f;

    public float tensionSmoothingHz = 3f;

    public float directionSmoothingHz = 2f;

    // -------------------------
    // Rendering and debug display
    // -------------------------
    [Header("Render")]
    public bool renderLine = true;
    public bool applyInspectionCableLineStyle = true;
    public float inspectionCableLineWidth = 0.08f;
    public Color inspectionCableLineColor = new Color(1f, 0.1f, 0.9f, 1f);
    public Material inspectionCableLineMaterial;

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
    Material runtimeLineMaterial;
    Transform cableSonarRoot;
    Transform[] cableSonarTransforms;
    CapsuleCollider[] cableSonarColliders;
    MeshCollider[] cableSonarMeshColliders;
    static Mesh sharedCylinderMeshForSonar;

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
    Vector3 floatLoadOnBottom = Vector3.zero;

    float tensionFiltered = 0f;
    Vector3 dirFiltered = Vector3.forward;
    float lengthFiltered = 0f;

    readonly List<FloatInstance> floatInstances = new List<FloatInstance>();
    bool floatInstancesDirty = true;
    int sonarColliderUpdateTick;

    void Awake()
    {
        lr = GetComponent<LineRenderer>();
        if (collisionIterationStride <= 0)
            collisionIterationStride = 3;
        SetupProbe();
        EnsureInitialized(force: true);
        AutoAssignBottomRigidbodyIfNeeded();
        RebuildFloatInstances();
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

        ClearFloatInstances();
        ClearCableSonarColliders();
        DestroyRuntimeLineMaterial();
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

        probeGO = new GameObject("CableXPBD_MultiFloat_Probe");
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

    void RebuildFloatInstances()
    {
        ClearFloatInstances();
        floatInstancesDirty = false;

        if (!enableFloatSections || floatSections == null || floatSections.Count == 0)
            return;

        if (floatVisualRoot == null)
        {
            GameObject root = new GameObject(FloatVisualRootName);
            root.transform.SetParent(transform, false);
            floatVisualRoot = root.transform;
        }
        else
        {
            floatVisualRoot.name = FloatVisualRootName;
        }

        DestroyDuplicateFloatVisualRoots();

        float length = Mathf.Max(0.01f, deployedLength);

        for (int s = 0; s < floatSections.Count; s++)
        {
            FloatSection section = floatSections[s];
            if (section == null || !section.enabled)
                continue;

            float start = Mathf.Clamp01(Mathf.Min(section.startNormalized, section.endNormalized));
            float end = Mathf.Clamp01(Mathf.Max(section.startNormalized, section.endNormalized));
            if (end <= start)
                continue;

            float sectionLength = (end - start) * length;
            float spacing = Mathf.Max(0.01f, section.spacingMeters);
            int count = Mathf.Max(1, Mathf.FloorToInt(sectionLength / spacing) + 1);

            for (int i = 0; i < count; i++)
            {
                float t = count == 1 ? 0.5f : (float)i / (count - 1);
                FloatInstance instance = new FloatInstance
                {
                    section = section,
                    normalizedPosition = Mathf.Lerp(start, end, t),
                };

                CreateFloatVisual(instance, s, i);
                floatInstances.Add(instance);
            }
        }
    }

    void ClearFloatInstances()
    {
        for (int i = 0; i < floatInstances.Count; i++)
        {
            FloatInstance instance = floatInstances[i];
            if (instance == null)
                continue;

            if (instance.runtimeMaterial != null)
            {
#if UNITY_EDITOR
                DestroyImmediate(instance.runtimeMaterial);
#else
                Destroy(instance.runtimeMaterial);
#endif
            }
        }

        floatInstances.Clear();
        ClearFloatVisualRootChildren();
    }

    void ClearFloatVisualRootChildren()
    {
        ResolveFloatVisualRootFromChildren();
        if (floatVisualRoot == null)
            return;

        for (int i = floatVisualRoot.childCount - 1; i >= 0; i--)
            DestroyGeneratedObject(floatVisualRoot.GetChild(i).gameObject);
    }

    void ResolveFloatVisualRootFromChildren()
    {
        if (floatVisualRoot != null)
            return;

        for (int i = 0; i < transform.childCount; i++)
        {
            Transform child = transform.GetChild(i);
            if (child != null && child.name == FloatVisualRootName)
            {
                floatVisualRoot = child;
                return;
            }
        }
    }

    void DestroyDuplicateFloatVisualRoots()
    {
        if (floatVisualRoot == null)
            return;

        for (int i = transform.childCount - 1; i >= 0; i--)
        {
            Transform child = transform.GetChild(i);
            if (child == null || child == floatVisualRoot || child.name != FloatVisualRootName)
                continue;

            DestroyGeneratedObject(child.gameObject);
        }
    }

    void DestroyGeneratedObject(UnityEngine.Object obj)
    {
        if (obj == null)
            return;

#if UNITY_EDITOR
        if (!Application.isPlaying)
            DestroyImmediate(obj);
        else
            Destroy(obj);
#else
        Destroy(obj);
#endif
    }

    void CreateFloatVisual(FloatInstance instance, int sectionIndex, int floatIndex)
    {
        FloatSection section = instance.section;
        GameObject go;

        if (section.prefab != null)
            go = Instantiate(section.prefab, floatVisualRoot);
        else
            go = GameObject.CreatePrimitive(PrimitiveType.Cylinder);

        go.name = $"{section.name}_{sectionIndex}_{floatIndex}";
        go.transform.SetParent(floatVisualRoot, true);

        RemoveVisualColliders(go);

        ApplySonarLayer(go);

        instance.visual = go.transform;
        instance.renderer = go.GetComponentInChildren<Renderer>();
        EnsureFloatSonarCollider(instance);

        if (instance.renderer != null)
        {
            if (section.material != null)
            {
                instance.runtimeMaterial = new Material(section.material);
                SetMaterialColor(instance.runtimeMaterial, section.color);
                instance.renderer.sharedMaterial = instance.runtimeMaterial;
            }
            else
            {
                Material material = CreateDefaultFloatMaterial(section.color);
                if (material != null)
                {
                    instance.runtimeMaterial = material;
                    instance.renderer.sharedMaterial = instance.runtimeMaterial;
                }
            }
        }
    }

    void EnsureFloatSonarCollider(FloatInstance instance)
    {
        if (instance == null || instance.visual == null)
            return;

        if (!enableFloatSonarColliders)
        {
            DestroyFloatSonarColliders(instance);
            return;
        }

        FloatSection section = instance.section;
        float scale = Mathf.Max(0.01f, section.visualScale) * Mathf.Max(0.01f, floatSonarColliderScale);
        float radius = Mathf.Max(0.001f, section.diameter * 0.5f * scale);
        float length = Mathf.Max(radius * 2f, section.length * scale);

        if (useFloatMeshSonarColliders && !floatSonarCollidersAreTrigger)
        {
            DestroyFloatFallbackCollider(instance);
            EnsureFloatMeshCollider(instance);
            return;
        }

        DestroyFloatMeshCollider(instance);

        if (instance.sonarFallbackCollider == null)
            instance.sonarFallbackCollider = instance.visual.gameObject.AddComponent<CapsuleCollider>();

        instance.sonarFallbackCollider.direction = 1;
        instance.sonarFallbackCollider.isTrigger = floatSonarCollidersAreTrigger;
        instance.sonarFallbackCollider.radius = radius;
        instance.sonarFallbackCollider.height = length;
        instance.sonarFallbackCollider.center = Vector3.zero;
        instance.sonarFallbackCollider.enabled = true;
    }

    void RemoveVisualColliders(GameObject go)
    {
        if (go == null)
            return;

        Collider[] colliders = go.GetComponentsInChildren<Collider>(true);
        for (int i = colliders.Length - 1; i >= 0; i--)
            DestroyGeneratedObject(colliders[i]);
    }

    void EnsureFloatMeshCollider(FloatInstance instance)
    {
        MeshFilter meshFilter = instance.visual.GetComponentInChildren<MeshFilter>();
        if (meshFilter == null || meshFilter.sharedMesh == null)
        {
            EnsureFloatFallbackCollider(instance);
            return;
        }

        if (instance.sonarMeshCollider == null)
            instance.sonarMeshCollider = instance.visual.gameObject.AddComponent<MeshCollider>();

        instance.sonarMeshCollider.sharedMesh = meshFilter.sharedMesh;
        instance.sonarMeshCollider.convex = false;
        instance.sonarMeshCollider.isTrigger = false;
        instance.sonarMeshCollider.enabled = true;
    }

    void EnsureFloatFallbackCollider(FloatInstance instance)
    {
        FloatSection section = instance.section;
        float scale = Mathf.Max(0.01f, section.visualScale) * Mathf.Max(0.01f, floatSonarColliderScale);
        float radius = Mathf.Max(0.001f, section.diameter * 0.5f * scale);
        float length = Mathf.Max(radius * 2f, section.length * scale);

        if (instance.sonarFallbackCollider == null)
            instance.sonarFallbackCollider = instance.visual.gameObject.AddComponent<CapsuleCollider>();

        instance.sonarFallbackCollider.direction = 1;
        instance.sonarFallbackCollider.isTrigger = false;
        instance.sonarFallbackCollider.radius = radius;
        instance.sonarFallbackCollider.height = length;
        instance.sonarFallbackCollider.center = Vector3.zero;
        instance.sonarFallbackCollider.enabled = true;
    }

    void DestroyFloatSonarColliders(FloatInstance instance)
    {
        DestroyFloatMeshCollider(instance);
        DestroyFloatFallbackCollider(instance);
    }

    void DestroyFloatMeshCollider(FloatInstance instance)
    {
        if (instance == null || instance.sonarMeshCollider == null)
            return;

#if UNITY_EDITOR
        DestroyImmediate(instance.sonarMeshCollider);
#else
        Destroy(instance.sonarMeshCollider);
#endif
        instance.sonarMeshCollider = null;
    }

    void DestroyFloatFallbackCollider(FloatInstance instance)
    {
        if (instance == null || instance.sonarFallbackCollider == null)
            return;

#if UNITY_EDITOR
        DestroyImmediate(instance.sonarFallbackCollider);
#else
        Destroy(instance.sonarFallbackCollider);
#endif
        instance.sonarFallbackCollider = null;
    }

    void ApplySonarLayer(GameObject go)
    {
        if (go == null || floatSonarLayer.value == 0)
            return;

        int layer = MaskToSingleLayer(floatSonarLayer);
        if (layer < 0)
            return;

        SetLayerRecursive(go, layer);
    }

    static void SetLayerRecursive(GameObject go, int layer)
    {
        go.layer = layer;
        Transform t = go.transform;
        for (int i = 0; i < t.childCount; i++)
            SetLayerRecursive(t.GetChild(i).gameObject, layer);
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

    static Material CreateDefaultFloatMaterial(Color color)
    {
        Shader shader = Shader.Find("HDRP/Lit");
        if (shader == null) shader = Shader.Find("Universal Render Pipeline/Lit");
        if (shader == null) shader = Shader.Find("Standard");
        if (shader == null) return null;

        Material material = new Material(shader);
        SetMaterialColor(material, color);
        return material;
    }

    static void SetMaterialColor(Material material, Color color)
    {
        if (material == null) return;

        if (material.HasProperty("_BaseColor"))
            material.SetColor("_BaseColor", color);

        if (material.HasProperty("_BaseColorMap"))
            material.SetTexture("_BaseColorMap", null);

        if (material.HasProperty("_MainColor"))
            material.SetColor("_MainColor", color);

        if (material.HasProperty("_UnlitColor"))
            material.SetColor("_UnlitColor", color);

        if (material.HasProperty("_Color"))
            material.SetColor("_Color", color);

        if (material.HasProperty("_EmissionColor"))
            material.SetColor("_EmissionColor", color);

        material.color = color;
    }

    // -------------------------
    // Fixed-step simulation loop
    // -------------------------
    void FixedUpdate()
    {
        EnsureInitialized(force: false);
        if (floatInstancesDirty)
            RebuildFloatInstances();

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

        if (!IsCableStateFinite())
        {
            Debug.LogWarning(BuildNonFiniteCableStateMessage(), this);
            InitCable();
            if (!IsCableStateFinite())
            {
                if (lr) lr.enabled = false;
                return;
            }
        }

        if (renderLine) UpdateLineRenderer();
        bool updateSonarColliders = ShouldUpdateSonarColliders();
        if (updateSonarColliders)
        {
            UpdateCableSonarColliders();
            UpdateFloatVisuals();
            Physics.SyncTransforms();
        }
        else
        {
            UpdateFloatVisuals(false);
        }

        lastCableLength = ComputeCableLength();
        lastStretch = lastCableLength - deployedLength;
        currentLoadOnBottom = EstimateCurrentLoadOnBottom();
        lastCurrentTensionN = currentLoadOnBottom.magnitude;
        cableBuoyancyLoadOnBottom = EstimateCableBuoyancyLoadOnBottom();
        floatLoadOnBottom = EstimateFloatLoadOnBottom();
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
            Vector3 F = currentLoadOnBottom + cableBuoyancyLoadOnBottom + floatLoadOnBottom;
            if (bottomRigidbody != null && bottomAttach != null && F.sqrMagnitude > 1e-8f)
                ApplyCableLoadToBottom(bottomRigidbody, F);

            lastTensionN = Mathf.Sqrt(lastCurrentTensionN * lastCurrentTensionN + (lastCableBuoyancyLoadN + floatLoadOnBottom.magnitude) * (lastCableBuoyancyLoadN + floatLoadOnBottom.magnitude));
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
            Vector3 drag = ComputeHydrodynamicDrag(i, vRel);
            Vector3 dragA = drag / Mathf.Max(1e-6f, massPerNode);

            float maxDragA = Mathf.Max(0f, maxHydrodynamicAcceleration);
            if (maxDragA > 0f && dragA.sqrMagnitude > maxDragA * maxDragA)
                dragA = dragA.normalized * maxDragA;

            Vector3 a = gEff + dragA + ComputeFloatAccelerationForNode(i, dt, isAboveSurface);

            v[i] = v[i] + a * dt;
            if (!IsFinite(v[i]))
            {
                v[i] = Vector3.zero;
                continue;
            }

            LimitCableNodeVelocity(ref v[i]);
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
            if (!IsFinite(v[i]))
            {
                v[i] = Vector3.zero;
                continue;
            }

            float cd = Mathf.Clamp01(constraintVelocityDamping);
            if (cd > 0f) v[i] *= (1f - cd);

            float cvd = Mathf.Clamp01(collisionVelocityDamping);
            if (cvd > 0f && collidedThisStep[i]) v[i] *= (1f - cvd);

            LimitCableNodeVelocity(ref v[i]);
        }
    }

    Vector3 ComputeFloatAccelerationForNode(int nodeIndex, float dt, bool nodeAboveSurface)
    {
        if (!enableFloatSections || floatInstances.Count == 0 || x == null || v == null || nodeAboveSurface)
            return Vector3.zero;

        Vector3 acceleration = Vector3.zero;

        for (int i = 0; i < floatInstances.Count; i++)
        {
            FloatInstance instance = floatInstances[i];
            if (instance == null || instance.section == null || !instance.section.enabled)
                continue;

            SampleCableState(
                instance.normalizedPosition,
                out _,
                out Vector3 tangent,
                out Vector3 floatVelocity,
                out int nodeA,
                out int nodeB,
                out float weightB);

            float nodeWeight = 0f;
            if (nodeIndex == nodeA)
                nodeWeight += 1f - weightB;
            if (nodeIndex == nodeB)
                nodeWeight += weightB;
            if (nodeWeight <= 0f)
                continue;

            FloatSection section = instance.section;
            Vector3 force = ComputeCylinderFloatForce(section, tangent, floatVelocity);

            float smooth = Mathf.Clamp01(floatForceSmoothing);
            if (smooth > 0f)
            {
                float alpha = 1f - Mathf.Pow(smooth, Mathf.Max(1f, dt * 60f));
                instance.smoothedForce = Vector3.Lerp(instance.smoothedForce, force, alpha);
                force = instance.smoothedForce;
            }
            else
            {
                instance.smoothedForce = force;
            }

            float maxForce = Mathf.Max(0f, maxFloatForce);
            if (maxForce > 0f && force.sqrMagnitude > maxForce * maxForce)
                force = force.normalized * maxForce;

            float addedMass = ComputeDirectionalAddedMass(section, tangent, force);
            float effectiveMass = Mathf.Max(1e-6f, massPerNode + addedMass * nodeWeight);
            acceleration += force * nodeWeight / effectiveMass;
        }

        float maxA = Mathf.Max(0f, maxFloatAcceleration);
        if (maxA > 0f && acceleration.sqrMagnitude > maxA * maxA)
            acceleration = acceleration.normalized * maxA;

        return acceleration;
    }

    Vector3 ComputeCylinderFloatForce(FloatSection section, Vector3 axis, Vector3 floatVelocity)
    {
        float radius = Mathf.Max(0.0005f, section.diameter * 0.5f);
        float length = Mathf.Max(0.001f, section.length);
        float volume = Mathf.PI * radius * radius * length;
        float rho = Mathf.Max(0f, floatWaterDensity);
        float g = Mathf.Max(0f, floatGravity);

        float buoyancy = rho * g * volume * Mathf.Max(0f, section.buoyancyScale);
        float weight = Mathf.Max(0f, section.massKg) * g;
        Vector3 netBuoyancy = Vector3.up * (buoyancy - weight);

        Vector3 tangent = axis.sqrMagnitude > 1e-12f ? axis.normalized : Vector3.up;
        Vector3 relativeVelocity = floatVelocity - currentVelocity;
        Vector3 vAxial = Vector3.Project(relativeVelocity, tangent);
        Vector3 vNormal = relativeVelocity - vAxial;

        float axialArea = Mathf.PI * radius * radius;
        float normalArea = Mathf.Max(0.000001f, section.diameter * length);

        Vector3 axialDrag = ComputeQuadraticDrag(vAxial, rho, section.axialDragCd, axialArea);
        Vector3 normalDrag = ComputeQuadraticDrag(vNormal, rho, section.normalDragCd, normalArea);

        return netBuoyancy + axialDrag + normalDrag;
    }

    static Vector3 ComputeQuadraticDrag(Vector3 velocity, float density, float dragCoefficient, float area)
    {
        if (velocity.sqrMagnitude <= 1e-12f)
            return Vector3.zero;

        float coefficient = 0.5f * Mathf.Max(0f, density) * Mathf.Max(0f, dragCoefficient) * Mathf.Max(0f, area);
        return -coefficient * velocity.magnitude * velocity;
    }

    float ComputeDirectionalAddedMass(FloatSection section, Vector3 axis, Vector3 force)
    {
        if (force.sqrMagnitude <= 1e-12f)
            return 0f;

        float radius = Mathf.Max(0.0005f, section.diameter * 0.5f);
        float volume = Mathf.PI * radius * radius * Mathf.Max(0.001f, section.length);
        float displacedMass = Mathf.Max(0f, floatWaterDensity) * volume;

        Vector3 tangent = axis.sqrMagnitude > 1e-12f ? axis.normalized : Vector3.up;
        Vector3 direction = force.normalized;
        float axialWeight = Mathf.Clamp01(Mathf.Abs(Vector3.Dot(direction, tangent)));
        axialWeight *= axialWeight;
        float normalWeight = 1f - axialWeight;

        float caAxial = Mathf.Max(0f, section.axialAddedMassCa);
        float caNormal = Mathf.Max(0f, section.normalAddedMassCa);
        return displacedMass * (caAxial * axialWeight + caNormal * normalWeight);
    }

    void SampleCableState(
        float normalizedPosition,
        out Vector3 position,
        out Vector3 tangent,
        out Vector3 velocity,
        out int nodeA,
        out int nodeB,
        out float weightB)
    {
        if (x == null || x.Length == 0)
        {
            position = transform.position;
            tangent = Vector3.up;
            velocity = Vector3.zero;
            nodeA = 0;
            nodeB = 0;
            weightB = 0f;
            return;
        }

        if (x.Length == 1)
        {
            position = x[0];
            tangent = Vector3.up;
            velocity = v != null && v.Length > 0 ? v[0] : Vector3.zero;
            nodeA = 0;
            nodeB = 0;
            weightB = 0f;
            return;
        }

        float f = Mathf.Clamp01(normalizedPosition) * (x.Length - 1);
        nodeA = Mathf.Clamp(Mathf.FloorToInt(f), 0, x.Length - 1);
        nodeB = Mathf.Clamp(nodeA + 1, 0, x.Length - 1);
        weightB = nodeA == nodeB ? 0f : f - nodeA;

        position = Vector3.Lerp(x[nodeA], x[nodeB], weightB);

        Vector3 d = x[nodeB] - x[nodeA];
        tangent = d.sqrMagnitude > 1e-12f ? d.normalized : GetCableTangentAtNode(nodeA);

        Vector3 va = v != null && nodeA < v.Length ? v[nodeA] : Vector3.zero;
        Vector3 vb = v != null && nodeB < v.Length ? v[nodeB] : va;
        velocity = Vector3.Lerp(va, vb, weightB);
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
        float EI = Mathf.Max(1e-8f, bendingRigidityEI);
        float ds = Mathf.Max(1e-4f, segLen);
        // Simple curvature-compliance approximation based on segment length and EI.
        float compliance = (ds * ds * ds * ds) / EI;

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
        SolveFloatCollisions();
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

    void SolveFloatCollisions()
    {
        if (!enableFloatCollision || !capsuleProbe) return;
        if (!enableFloatSections || floatInstances.Count == 0) return;
        if (x == null || x.Length < 2) return;

        float maxCorr = Mathf.Max(0f, maxFloatCollisionCorrection);
        capsuleProbe.enabled = true;

        for (int i = 0; i < floatInstances.Count; i++)
        {
            FloatInstance instance = floatInstances[i];
            if (instance == null || instance.section == null)
                continue;

            SampleCableState(instance.normalizedPosition, out Vector3 position, out Vector3 tangent, out _, out _, out _, out _);

            FloatSection section = instance.section;
            float scale = Mathf.Max(0.01f, section.visualScale);
            float radius = Mathf.Max(0.001f, section.diameter * 0.5f * scale * Mathf.Max(0.01f, floatCollisionRadiusScale));
            float halfLength = Mathf.Max(0.001f, section.length * 0.5f * scale);
            Vector3 axis = tangent.sqrMagnitude > 1e-8f ? tangent.normalized : Vector3.up;
            Vector3 p0 = position - axis * halfLength;
            Vector3 p1 = position + axis * halfLength;
            Quaternion rot = Quaternion.FromToRotation(Vector3.up, axis);

            int hitCount = Physics.OverlapCapsuleNonAlloc(
                p0, p1, radius, overlapBuf, collisionMask, QueryTriggerInteraction.Ignore);

            if (hitCount <= 0)
                continue;

            capsuleProbe.radius = radius;
            capsuleProbe.height = Mathf.Max(radius * 2f, halfLength * 2f + radius * 2f);

            for (int h = 0; h < hitCount; h++)
            {
                Collider col = overlapBuf[h];
                if (!col) continue;
                if (ShouldIgnoreCollisionCollider(col, NormalizedToNearestNodeIndex(instance.normalizedPosition), false)) continue;

                bool overlapped = Physics.ComputePenetration(
                    capsuleProbe, position, rot,
                    col, col.transform.position, col.transform.rotation,
                    out Vector3 dir, out float dist);

                if (!overlapped || dist <= 0f)
                    continue;

                float d = maxCorr > 0f ? Mathf.Min(dist, maxCorr) : dist;
                ApplyFloatCollisionCorrection(instance.normalizedPosition, dir * d);
            }
        }

        capsuleProbe.enabled = false;
    }

    int NormalizedToNearestNodeIndex(float normalized)
    {
        int last = Mathf.Max(0, nodeCount - 1);
        return Mathf.Clamp(Mathf.RoundToInt(Mathf.Clamp01(normalized) * last), 0, last);
    }

    void ApplyFloatCollisionCorrection(float normalized, Vector3 correction)
    {
        if (x == null || invM == null || x.Length < 2)
            return;

        float f = Mathf.Clamp01(normalized) * (x.Length - 1);
        int i0 = Mathf.Clamp(Mathf.FloorToInt(f), 0, x.Length - 1);
        int i1 = Mathf.Clamp(i0 + 1, 0, x.Length - 1);
        float t = Mathf.Clamp01(f - i0);

        float w0Shape = 1f - t;
        float w1Shape = t;
        float w0 = invM[i0] * w0Shape;
        float w1 = invM[i1] * w1Shape;
        float sum = w0 + w1;
        if (sum <= 1e-8f)
            return;

        Vector3 c0 = correction * (w0 / sum);
        Vector3 c1 = correction * (w1 / sum);

        if (invM[i0] > 0f)
        {
            x[i0] += c0;
            collidedThisStep[i0] = true;
        }

        if (i1 != i0 && invM[i1] > 0f)
        {
            x[i1] += c1;
            collidedThisStep[i1] = true;
        }
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

        float surfaceY = GetWaterLevelY();
        Vector3 totalDrag = Vector3.zero;

        // Sum the cable hydrodynamic drag from node-water relative velocity and pass it to the bottom body.
        for (int i = 0; i < x.Length; i++)
        {
            bool isAboveSurface = enableWaterSurface && (x[i].y > surfaceY);
            if (isAboveSurface && disableCurrentAboveSurface)
                continue;

            Vector3 vRel = v[i] - currentVelocity;
            if (vRel.sqrMagnitude <= 1e-12f)
                continue;

            Vector3 drag = ComputeHydrodynamicDrag(i, vRel);
            drag = ClampHydrodynamicDrag(drag);

            float weight = (i == 0 || i == x.Length - 1) ? 0.5f : 1f;
            totalDrag += drag * weight;
        }

        Vector3 load = totalDrag;
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
        Vector3 load = effectiveCableWeight;

        float maxLoad = Mathf.Max(0f, maxCableBuoyancyLoadNewton);
        if (maxLoad > 0f && load.magnitude > maxLoad)
            load = load.normalized * maxLoad;

        lastCableBuoyancyLoadN = load.magnitude;
        return load;
    }

    Vector3 EstimateFloatLoadOnBottom()
    {
        if (!enableFloatSections || floatInstances.Count == 0 || x == null || x.Length < 2)
            return Vector3.zero;

        float surfaceY = GetWaterLevelY();
        Vector3 load = Vector3.zero;

        for (int i = 0; i < floatInstances.Count; i++)
        {
            FloatInstance instance = floatInstances[i];
            if (instance == null || instance.section == null || !instance.section.enabled)
                continue;

            SampleCableState(
                instance.normalizedPosition,
                out Vector3 position,
                out Vector3 tangent,
                out Vector3 floatVelocity,
                out _,
                out _,
                out _);

            bool isAboveSurface = enableWaterSurface && (position.y > surfaceY);
            if (isAboveSurface)
                continue;

            Vector3 force = ComputeCylinderFloatForce(instance.section, tangent, floatVelocity);
            float maxForce = Mathf.Max(0f, maxFloatForce);
            if (maxForce > 0f && force.sqrMagnitude > maxForce * maxForce)
                force = force.normalized * maxForce;

            load += force * Mathf.Clamp01(instance.normalizedPosition);
        }

        return load;
    }

    Vector3 ComputeHydrodynamicDrag(int nodeIndex, Vector3 relativeVelocity)
    {
        if (relativeVelocity.sqrMagnitude <= 1e-12f)
            return Vector3.zero;

        if (hydrodynamicDragModel == HydrodynamicDragModel.Morison)
            return ComputeMorisonDrag(nodeIndex, relativeVelocity);

        Vector3 tangent = GetCableTangentAtNode(nodeIndex);
        if (tangent.sqrMagnitude <= 1e-12f)
        {
            float linear = Mathf.Max(0f, dragLinearAcross);
            float quadratic = Mathf.Max(0f, dragQuadraticAcross);
            float speed = relativeVelocity.magnitude;
            return -linear * relativeVelocity - quadratic * speed * relativeVelocity;
        }

        Vector3 vAlong = Vector3.Project(relativeVelocity, tangent);
        Vector3 vAcross = relativeVelocity - vAlong;

        Vector3 drag = -Mathf.Max(0f, dragLinearAlong) * vAlong
                       -Mathf.Max(0f, dragLinearAcross) * vAcross;

        float qAlong = Mathf.Max(0f, dragQuadraticAlong);
        if (qAlong > 0f)
            drag += -qAlong * vAlong.magnitude * vAlong;

        float qAcross = Mathf.Max(0f, dragQuadraticAcross);
        if (qAcross > 0f)
            drag += -qAcross * vAcross.magnitude * vAcross;

        return drag;
    }

    Vector3 ClampHydrodynamicDrag(Vector3 drag)
    {
        if (!IsFinite(drag))
            return Vector3.zero;

        float maxDragA = Mathf.Max(0f, maxHydrodynamicAcceleration);
        if (maxDragA <= 0f)
            return drag;

        float maxForce = Mathf.Max(1e-6f, massPerNode) * maxDragA;
        if (drag.sqrMagnitude > maxForce * maxForce)
            return drag.normalized * maxForce;

        return drag;
    }

    Vector3 ComputeMorisonDrag(int nodeIndex, Vector3 relativeVelocity)
    {
        Vector3 tangent = GetCableTangentAtNode(nodeIndex);
        if (tangent.sqrMagnitude <= 1e-12f)
        {
            float speed = relativeVelocity.magnitude;
            float areaLength = Mathf.Max(1e-5f, morisonCableDiameter) * GetNodeTributaryLength(nodeIndex);
            float coefficient = 0.5f
                                * Mathf.Max(0f, morisonWaterDensity)
                                * Mathf.Max(0f, morisonNormalDragCoefficient)
                                * areaLength
                                * Mathf.Max(0f, morisonDragScale);
            return -coefficient * speed * relativeVelocity;
        }

        Vector3 vAlong = Vector3.Project(relativeVelocity, tangent);
        Vector3 vAcross = relativeVelocity - vAlong;

        float rho = Mathf.Max(0f, morisonWaterDensity);
        float diameter = Mathf.Max(1e-5f, morisonCableDiameter);
        float length = GetNodeTributaryLength(nodeIndex);
        float scale = Mathf.Max(0f, morisonDragScale);

        float normalArea = diameter * length;
        float tangentialArea = Mathf.PI * diameter * length;

        float normalCoeff = 0.5f * rho * Mathf.Max(0f, morisonNormalDragCoefficient) * normalArea * scale;
        float tangentialCoeff = 0.5f * rho * Mathf.Max(0f, morisonTangentialDragCoefficient) * tangentialArea * scale;

        return -normalCoeff * vAcross.magnitude * vAcross
               -tangentialCoeff * vAlong.magnitude * vAlong;
    }

    float GetNodeTributaryLength(int nodeIndex)
    {
        float restLength = Mathf.Max(1e-4f, segLen);
        if (x == null || x.Length < 2)
            return restLength;

        int i = Mathf.Clamp(nodeIndex, 0, x.Length - 1);
        float length = 0f;

        if (i > 0)
            length += 0.5f * Vector3.Distance(x[i], x[i - 1]);

        if (i < x.Length - 1)
            length += 0.5f * Vector3.Distance(x[i + 1], x[i]);

        if (!IsFinite(length))
            return restLength;

        return Mathf.Clamp(length, restLength * 0.25f, restLength * 2f);
    }

    Vector3 GetCableTangentAtNode(int nodeIndex)
    {
        if (x == null || x.Length < 2)
            return Vector3.zero;

        int i = Mathf.Clamp(nodeIndex, 0, x.Length - 1);
        Vector3 d;

        if (i <= 0)
            d = x[1] - x[0];
        else if (i >= x.Length - 1)
            d = x[x.Length - 1] - x[x.Length - 2];
        else
            d = x[i + 1] - x[i - 1];

        float m = d.magnitude;
        return m > 1e-6f ? d / m : Vector3.zero;
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
            vOut = Vector3.Dot(GetCableLoadPointVelocity(rb), dirFiltered);

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
        Vector3 F = -dirFiltered * tensionFiltered + currentLoadOnBottom + cableBuoyancyLoadOnBottom + floatLoadOnBottom;

        ApplyCableLoadToBottom(rb, F);

        float buoyantLoad = lastCableBuoyancyLoadN + floatLoadOnBottom.magnitude;
        lastTensionN = Mathf.Sqrt(tensionFiltered * tensionFiltered + lastCurrentTensionN * lastCurrentTensionN + buoyantLoad * buoyantLoad);
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

            Vector3 velPoint = GetCableLoadPointVelocity(rb);
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
        Vector3 F = -dirFiltered * tensionFiltered + currentLoadOnBottom + cableBuoyancyLoadOnBottom + floatLoadOnBottom;

        ApplyCableLoadToBottom(rb, F);

        float buoyantLoad = lastCableBuoyancyLoadN + floatLoadOnBottom.magnitude;
        lastTensionN = Mathf.Sqrt(tensionFiltered * tensionFiltered + lastCurrentTensionN * lastCurrentTensionN + buoyantLoad * buoyantLoad);
    }

    Vector3 ComputeEndToEndDirSafe()
    {
        if (topAnchor == null || bottomAttach == null) return Vector3.forward;

        Vector3 d = (bottomAttach.position - topAnchor.position);
        float m = d.magnitude;
        if (m > 1e-6f) return d / m;
        return Vector3.forward;
    }

    Vector3 GetCableLoadApplicationPoint(Rigidbody rb)
    {
        if (!applyForceAtAttachPoint || rb == null || bottomAttach == null)
            return rb != null ? rb.worldCenterOfMass : Vector3.zero;

        return bottomAttach.position;
    }

    Vector3 GetCableLoadPointVelocity(Rigidbody rb)
    {
        if (rb == null) return Vector3.zero;
        if (!applyForceAtAttachPoint) return rb.linearVelocity;

        return rb.GetPointVelocity(GetCableLoadApplicationPoint(rb));
    }

    void ApplyCableLoadToBottom(Rigidbody rb, Vector3 force)
    {
        if (rb == null) return;
        if (!IsFinite(force)) return;

        if (applyForceAtAttachPoint)
            rb.AddForceAtPosition(force, GetCableLoadApplicationPoint(rb), ForceMode.Force);
        else
            rb.AddForce(force, ForceMode.Force);
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

    bool IsCableStateFinite()
    {
        if (x == null || v == null) return false;

        for (int i = 0; i < x.Length; i++)
        {
            if (!IsFinite(x[i]))
                return false;
        }

        for (int i = 0; i < v.Length; i++)
        {
            if (!IsFinite(v[i]))
                return false;
        }

        return true;
    }

    static bool IsFinite(float value)
    {
        return !float.IsNaN(value) && !float.IsInfinity(value);
    }

    static bool IsFinite(Vector3 value)
    {
        return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
    }

    void LimitCableNodeVelocity(ref Vector3 velocity)
    {
        if (!IsFinite(velocity))
        {
            velocity = Vector3.zero;
            return;
        }

        float maxSpeed = Mathf.Max(0f, maxCableNodeSpeed);
        if (maxSpeed > 0f && velocity.sqrMagnitude > maxSpeed * maxSpeed)
            velocity = velocity.normalized * maxSpeed;
    }

    string BuildNonFiniteCableStateMessage()
    {
        float maxSpeed = 0f;
        if (v != null)
        {
            for (int i = 0; i < v.Length; i++)
            {
                if (IsFinite(v[i]))
                    maxSpeed = Mathf.Max(maxSpeed, v[i].magnitude);
            }
        }

        float maxSegment = 0f;
        if (x != null && x.Length >= 2)
        {
            for (int i = 0; i < x.Length - 1; i++)
            {
                Vector3 d = x[i + 1] - x[i];
                if (IsFinite(d))
                    maxSegment = Mathf.Max(maxSegment, d.magnitude);
            }
        }

        float endDistance = 0f;
        if (topAnchor != null && bottomAttach != null)
            endDistance = Vector3.Distance(topAnchor.position, bottomAttach.position);

        return $"[{nameof(CableXPBD_MultiFloat)}] Non-finite cable state detected; reinitializing cable. " +
               $"drag={hydrodynamicDragModel}, current={currentVelocity.magnitude:0.###} m/s, " +
               $"maxNodeSpeed={maxSpeed:0.###} m/s, maxSegment={maxSegment:0.###} m, " +
               $"segLen={segLen:0.###} m, endDist={endDistance:0.###} m, " +
               $"hydroAmax={maxHydrodynamicAcceleration:0.###}, nodeSpeedMax={maxCableNodeSpeed:0.###}";
    }

}
