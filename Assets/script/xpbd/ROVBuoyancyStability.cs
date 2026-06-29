using UnityEngine;
using UnityEngine.Rendering.HighDefinition;

[RequireComponent(typeof(Rigidbody))]
public class ROVBuoyancyStability : MonoBehaviour
{
    [Header("Centers (local space)")]
    public Vector3 centerOfMassLocalOffset = new Vector3(0f, -0.05f, 0f);
    public Vector3 centerOfBuoyancyLocalOffset = new Vector3(0f, 0.10f, 0f);

    [Header("Rest Attitude Trim")]
    public bool useTrimmedRestAttitude = false;
    public float restPitchDeg = 0f;
    public float restRollDeg = 0f;
    public float metacentricHeight = 0.15f;
    public bool useMetacentricRestoringTorque = true;

    [Header("Buoyancy")]
    public bool setRigidbodyCenterOfMass = true;
    public bool useUnityGravity = true;
    public float buoyancyScale = 1.0f;
    public float verticalDampingNPerMps = 240f;
    public bool limitBuoyancyToWaterSurface = true;
    public WaterSurface waterSurface;
    public float fallbackWaterSurfaceY = 0f;
    public float fullBuoyancyDepthMeters = 0.15f;
    public bool filterWaterSurfaceHeight = true;
    public float waterSurfaceLowPassTimeSeconds = 3f;
    public float waterSurfaceQueryError = 0.01f;
    public int waterSurfaceQueryMaxIterations = 8;

    [Header("Rotation Damping")]
    public float rollPitchAngularDamping = 0.9f;

    Rigidbody rb;
    WaterSearchParameters waterSearchParameters;
    WaterSearchResult waterSearchResult;
    bool hasWaterSearchCandidate;
    bool hasFilteredWaterSurfaceY;
    float filteredWaterSurfaceY;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        waterSearchParameters = new WaterSearchParameters();
        waterSearchResult = new WaterSearchResult();
        ResolveWaterSurfaceIfNeeded();
        ApplyRigidbodySettings();
    }

    void OnValidate()
    {
        rb = GetComponent<Rigidbody>();
        fullBuoyancyDepthMeters = Mathf.Max(0.001f, fullBuoyancyDepthMeters);
        waterSurfaceLowPassTimeSeconds = Mathf.Max(0f, waterSurfaceLowPassTimeSeconds);
        waterSurfaceQueryError = Mathf.Max(0.001f, waterSurfaceQueryError);
        waterSurfaceQueryMaxIterations = Mathf.Max(1, waterSurfaceQueryMaxIterations);
        ApplyRigidbodySettings();
    }

    void FixedUpdate()
    {
        ApplyRigidbodySettings();

        Vector3 gravity = Physics.gravity;
        float gravityMagnitude = gravity.magnitude;
        if (gravityMagnitude <= 1e-6f)
            return;

        float buoyancyFactor = Mathf.Max(0f, buoyancyScale) * GetWaterSurfaceBuoyancyFactor();
        Vector3 buoyancyForce = -gravity.normalized * (rb.mass * gravityMagnitude * buoyancyFactor);

        if (useMetacentricRestoringTorque)
        {
            rb.AddForce(buoyancyForce, ForceMode.Force);
            ApplyMetacentricRestoringTorque(gravityMagnitude);
        }
        else
        {
            Vector3 buoyancyPoint = TransformLocalPointWithoutScale(GetEffectiveCenterOfBuoyancyLocal());
            rb.AddForceAtPosition(buoyancyForce, buoyancyPoint, ForceMode.Force);
        }

        if (verticalDampingNPerMps > 0f && buoyancyFactor > 0f)
        {
            float verticalSpeed = Vector3.Dot(rb.linearVelocity, Vector3.up);
            rb.AddForce(Vector3.up * (-verticalSpeed * verticalDampingNPerMps * Mathf.Clamp01(buoyancyFactor)), ForceMode.Force);
        }

        if (rollPitchAngularDamping > 0f)
        {
            Vector3 yawAngularVelocity = Vector3.Project(rb.angularVelocity, Vector3.up);
            Vector3 rollPitchAngularVelocity = rb.angularVelocity - yawAngularVelocity;
            rb.AddTorque(-rollPitchAngularVelocity * rollPitchAngularDamping, ForceMode.Acceleration);
        }
    }

    void ApplyRigidbodySettings()
    {
        if (rb == null)
            return;

        rb.useGravity = useUnityGravity;

        if (setRigidbodyCenterOfMass)
            rb.centerOfMass = centerOfMassLocalOffset;
    }

    Vector3 GetEffectiveCenterOfBuoyancyLocal()
    {
        return centerOfBuoyancyLocalOffset;
    }

    float GetWaterSurfaceBuoyancyFactor()
    {
        if (!limitBuoyancyToWaterSurface)
            return 1f;

        float surfaceY = GetFilteredWaterSurfaceY(rb.position);
        float depth = surfaceY - rb.position.y;
        return Mathf.Clamp01(depth / Mathf.Max(0.001f, fullBuoyancyDepthMeters));
    }

    float GetFilteredWaterSurfaceY(Vector3 worldPosition)
    {
        float rawSurfaceY = GetWaterSurfaceY(worldPosition);
        if (!filterWaterSurfaceHeight)
        {
            filteredWaterSurfaceY = rawSurfaceY;
            hasFilteredWaterSurfaceY = true;
            return rawSurfaceY;
        }

        if (!hasFilteredWaterSurfaceY)
        {
            filteredWaterSurfaceY = rawSurfaceY;
            hasFilteredWaterSurfaceY = true;
            return filteredWaterSurfaceY;
        }

        float tau = Mathf.Max(0f, waterSurfaceLowPassTimeSeconds);
        float alpha = tau <= 0f
            ? 1f
            : Mathf.Clamp01(1f - Mathf.Exp(-Mathf.Max(0.001f, Time.fixedDeltaTime) / tau));
        filteredWaterSurfaceY = Mathf.Lerp(filteredWaterSurfaceY, rawSurfaceY, alpha);
        return filteredWaterSurfaceY;
    }

    float GetWaterSurfaceY(Vector3 worldPosition)
    {
        ResolveWaterSurfaceIfNeeded();
        if (waterSurface != null && TryGetWaterSurfaceY(worldPosition, out float surfaceY))
            return surfaceY;

        return fallbackWaterSurfaceY;
    }

    void ResolveWaterSurfaceIfNeeded()
    {
        if (waterSurface != null)
            return;

        waterSurface = FindFirstObjectByType<WaterSurface>();
        hasWaterSearchCandidate = false;
        hasFilteredWaterSurfaceY = false;
    }

    bool TryGetWaterSurfaceY(Vector3 worldPosition, out float surfaceY)
    {
        if (!hasWaterSearchCandidate)
        {
            waterSearchResult.candidateLocationWS = worldPosition;
            hasWaterSearchCandidate = true;
        }

        waterSearchParameters.startPositionWS = waterSearchResult.candidateLocationWS;
        waterSearchParameters.targetPositionWS = worldPosition;
        waterSearchParameters.error = waterSurfaceQueryError;
        waterSearchParameters.maxIterations = waterSurfaceQueryMaxIterations;
        waterSearchParameters.outputNormal = false;

        if (waterSurface.ProjectPointOnWaterSurface(waterSearchParameters, out waterSearchResult))
        {
            surfaceY = waterSearchResult.projectedPositionWS.y;
            return true;
        }

        hasWaterSearchCandidate = false;
        surfaceY = fallbackWaterSurfaceY;
        return false;
    }

    void ApplyMetacentricRestoringTorque(float gravityMagnitude)
    {
        Quaternion yawOnly = Quaternion.Euler(0f, rb.rotation.eulerAngles.y, 0f);
        Quaternion restLocal = useTrimmedRestAttitude
            ? Quaternion.Euler(restPitchDeg, 0f, restRollDeg)
            : Quaternion.identity;
        Quaternion target = yawOnly * restLocal;

        Quaternion q = target * Quaternion.Inverse(rb.rotation);
        q.ToAngleAxis(out float angleDeg, out Vector3 axis);
        if (angleDeg > 180f) angleDeg -= 360f;
        if (float.IsNaN(axis.x) || axis.sqrMagnitude <= 1e-8f)
            return;

        Vector3 axisWorld = axis.normalized;
        Vector3 yawComponent = Vector3.Project(axisWorld, Vector3.up);
        Vector3 rollPitchAxis = axisWorld - yawComponent;
        if (rollPitchAxis.sqrMagnitude <= 1e-8f)
            return;

        float angleRad = angleDeg * Mathf.Deg2Rad;
        float torqueMagnitude = rb.mass * gravityMagnitude * Mathf.Max(0f, metacentricHeight) * Mathf.Sin(angleRad);
        rb.AddTorque(rollPitchAxis.normalized * torqueMagnitude, ForceMode.Force);
    }

    void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.yellow;
        Gizmos.DrawSphere(TransformLocalPointWithoutScale(centerOfMassLocalOffset), 0.08f);

        Gizmos.color = Color.cyan;
        Vector3 buoyancyLocal = GetEffectiveCenterOfBuoyancyLocal();
        Gizmos.DrawSphere(TransformLocalPointWithoutScale(buoyancyLocal), 0.08f);
        Gizmos.DrawLine(TransformLocalPointWithoutScale(centerOfMassLocalOffset), TransformLocalPointWithoutScale(buoyancyLocal));
    }

    Vector3 TransformLocalPointWithoutScale(Vector3 localPoint)
    {
        return transform.position + transform.rotation * localPoint;
    }
}
