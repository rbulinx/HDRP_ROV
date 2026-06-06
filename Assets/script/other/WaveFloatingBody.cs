using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;
using UnityEngine.SceneManagement;

[DisallowMultipleComponent]
public class WaveFloatingBody : MonoBehaviour
{
    [Header("Target")]
    public string targetName = "akship";
    public bool autoAttachToNamedObject = true;

    [Header("Water")]
    public WaterSurface waterSurface;
    public float fallbackWaterSurfaceY = 0f;
    public float waterQueryError = 0.01f;
    public int waterQueryMaxIterations = 8;

    [Header("Motion")]
    public bool followSurfaceHeight = true;
    public bool followSurfaceTilt = true;
    public float heightLerpSpeed = 2.5f;
    public float rotationLerpSpeed = 2.5f;
    public float maxTiltDeg = 12f;

    [Header("Sampling")]
    public bool autoSampleFromColliders = true;
    [Range(0.2f, 1f)] public float sampleSpread = 0.85f;
    public Vector3 manualHalfExtents = new Vector3(2f, 0f, 6f);

    Rigidbody rb;
    Vector3 initialPosition;
    Quaternion initialRotation;
    float initialSurfaceOffset;
    Vector3[] localSamplePoints;

    WaterSearchParameters waterSearchParameters;
    WaterSearchResult waterSearchResult;
    bool hasWaterCandidate;

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    static void AutoAttachForAkship()
    {
        if (SceneSelector.IsMenuSceneActive() || SceneSelector.IsWaveEvaluationSceneActive()) return;

        Transform[] transforms = Object.FindObjectsByType<Transform>(FindObjectsInactive.Exclude, FindObjectsSortMode.None);
        for (int i = 0; i < transforms.Length; i++)
        {
            Transform tr = transforms[i];
            if (tr == null) continue;
            if (!tr.name.Equals("akship", System.StringComparison.OrdinalIgnoreCase)) continue;
            if (tr.GetComponent<WaveFloatingBody>() != null) return;

            WaveFloatingBody body = tr.gameObject.AddComponent<WaveFloatingBody>();
            body.autoAttachToNamedObject = false;
            body.targetName = tr.name;
            Debug.Log("[WaveFloatingBody] Auto-attached to akship.");
            return;
        }
    }

    void Awake()
    {
        if (autoAttachToNamedObject && !string.IsNullOrWhiteSpace(targetName) &&
            !name.Equals(targetName, System.StringComparison.OrdinalIgnoreCase))
        {
            enabled = false;
            return;
        }

        rb = GetComponent<Rigidbody>();
        if (waterSurface == null)
            waterSurface = FindFirstObjectByType<WaterSurface>();

        initialPosition = transform.position;
        initialRotation = transform.rotation;

        BuildSamplePoints();
        float initialSurface = GetAverageSurfaceY(transform.position, transform.rotation);
        initialSurfaceOffset = initialPosition.y - initialSurface;
    }

    void FixedUpdate()
    {
        if (waterSurface == null)
            waterSurface = FindFirstObjectByType<WaterSurface>();

        Vector3 targetPosition = transform.position;
        Quaternion targetRotation = transform.rotation;

        float avgSurfaceY = GetAverageSurfaceY(transform.position, transform.rotation);

        if (followSurfaceHeight)
        {
            targetPosition.y = avgSurfaceY + initialSurfaceOffset;
        }

        if (followSurfaceTilt)
        {
            Quaternion rawRotation = ComputeSurfaceRotation();
            targetRotation = ClampTilt(rawRotation, initialRotation, maxTiltDeg);
        }

        float posT = 1f - Mathf.Exp(-heightLerpSpeed * Time.fixedDeltaTime);
        float rotT = 1f - Mathf.Exp(-rotationLerpSpeed * Time.fixedDeltaTime);
        Vector3 nextPosition = Vector3.Lerp(transform.position, targetPosition, posT);
        Quaternion nextRotation = Quaternion.Slerp(transform.rotation, targetRotation, rotT);

        if (rb != null && !rb.isKinematic)
        {
            rb.MovePosition(nextPosition);
            rb.MoveRotation(nextRotation);
        }
        else
        {
            transform.SetPositionAndRotation(nextPosition, nextRotation);
        }
    }

    void BuildSamplePoints()
    {
        if (!autoSampleFromColliders)
        {
            localSamplePoints = BuildPointsFromHalfExtents(manualHalfExtents);
            return;
        }

        Collider[] colliders = GetComponentsInChildren<Collider>(true);
        if (colliders == null || colliders.Length == 0)
        {
            localSamplePoints = BuildPointsFromHalfExtents(manualHalfExtents);
            return;
        }

        Bounds bounds = colliders[0].bounds;
        for (int i = 1; i < colliders.Length; i++)
            bounds.Encapsulate(colliders[i].bounds);

        Vector3 centerLocal = transform.InverseTransformPoint(bounds.center);
        Vector3 extentsLocal = Vector3.Scale(bounds.extents, new Vector3(sampleSpread, 0f, sampleSpread));

        localSamplePoints = new[]
        {
            centerLocal + new Vector3(-extentsLocal.x, 0f,  extentsLocal.z),
            centerLocal + new Vector3( extentsLocal.x, 0f,  extentsLocal.z),
            centerLocal + new Vector3(-extentsLocal.x, 0f, -extentsLocal.z),
            centerLocal + new Vector3( extentsLocal.x, 0f, -extentsLocal.z)
        };
    }

    static Vector3[] BuildPointsFromHalfExtents(Vector3 halfExtents)
    {
        return new[]
        {
            new Vector3(-halfExtents.x, 0f,  halfExtents.z),
            new Vector3( halfExtents.x, 0f,  halfExtents.z),
            new Vector3(-halfExtents.x, 0f, -halfExtents.z),
            new Vector3( halfExtents.x, 0f, -halfExtents.z)
        };
    }

    float GetAverageSurfaceY(Vector3 position, Quaternion rotation)
    {
        if (localSamplePoints == null || localSamplePoints.Length == 0)
            return GetWaterSurfaceYAt(position);

        float sum = 0f;
        for (int i = 0; i < localSamplePoints.Length; i++)
        {
            Vector3 world = position + rotation * localSamplePoints[i];
            sum += GetWaterSurfaceYAt(world);
        }
        return sum / localSamplePoints.Length;
    }

    Quaternion ComputeSurfaceRotation()
    {
        if (localSamplePoints == null || localSamplePoints.Length < 4)
            return initialRotation;

        Vector3 p0 = transform.position + transform.rotation * localSamplePoints[0];
        Vector3 p1 = transform.position + transform.rotation * localSamplePoints[1];
        Vector3 p2 = transform.position + transform.rotation * localSamplePoints[2];
        Vector3 p3 = transform.position + transform.rotation * localSamplePoints[3];

        p0.y = GetWaterSurfaceYAt(p0);
        p1.y = GetWaterSurfaceYAt(p1);
        p2.y = GetWaterSurfaceYAt(p2);
        p3.y = GetWaterSurfaceYAt(p3);

        Vector3 front = (p0 + p1) * 0.5f;
        Vector3 back = (p2 + p3) * 0.5f;
        Vector3 left = (p0 + p2) * 0.5f;
        Vector3 right = (p1 + p3) * 0.5f;

        Vector3 forwardVec = (front - back).normalized;
        Vector3 rightVec = (right - left).normalized;
        Vector3 upVec = Vector3.Cross(forwardVec, rightVec).normalized;
        if (upVec.y < 0f) upVec = -upVec;
        if (upVec.sqrMagnitude <= 1e-8f) return initialRotation;

        return Quaternion.FromToRotation(initialRotation * Vector3.up, upVec) * initialRotation;
    }

    static Quaternion ClampTilt(Quaternion target, Quaternion reference, float maxTiltDeg)
    {
        Quaternion delta = target * Quaternion.Inverse(reference);
        delta.ToAngleAxis(out float angle, out Vector3 axis);
        if (float.IsNaN(axis.x) || axis.sqrMagnitude <= 1e-8f) return reference;
        if (angle > 180f) angle -= 360f;
        angle = Mathf.Clamp(angle, -maxTiltDeg, maxTiltDeg);
        return Quaternion.AngleAxis(angle, axis.normalized) * reference;
    }

    float GetWaterSurfaceYAt(Vector3 worldPos)
    {
        if (waterSurface != null && TryGetHDRPWaterSurface(worldPos, out float y))
            return y;

        return fallbackWaterSurfaceY;
    }

    bool TryGetHDRPWaterSurface(Vector3 worldPos, out float surfaceY)
    {
        if (!hasWaterCandidate)
        {
            waterSearchResult.candidateLocationWS = worldPos;
            hasWaterCandidate = true;
        }

        waterSearchParameters.startPositionWS = waterSearchResult.candidateLocationWS;
        waterSearchParameters.targetPositionWS = worldPos;
        waterSearchParameters.error = waterQueryError;
        waterSearchParameters.maxIterations = waterQueryMaxIterations;
        waterSearchParameters.outputNormal = false;

        bool ok = waterSurface.ProjectPointOnWaterSurface(waterSearchParameters, out waterSearchResult);
        if (ok)
        {
            surfaceY = waterSearchResult.projectedPositionWS.y;
            return true;
        }

        hasWaterCandidate = false;
        surfaceY = fallbackWaterSurfaceY;
        return false;
    }
}
