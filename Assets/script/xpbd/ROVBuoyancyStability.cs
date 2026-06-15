using UnityEngine;

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

    [Header("Rotation Damping")]
    public float rollPitchAngularDamping = 0.9f;

    Rigidbody rb;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        ApplyRigidbodySettings();
    }

    void OnValidate()
    {
        rb = GetComponent<Rigidbody>();
        ApplyRigidbodySettings();
    }

    void FixedUpdate()
    {
        ApplyRigidbodySettings();

        Vector3 gravity = Physics.gravity;
        float gravityMagnitude = gravity.magnitude;
        if (gravityMagnitude <= 1e-6f)
            return;

        Vector3 buoyancyForce = -gravity.normalized * (rb.mass * gravityMagnitude * Mathf.Max(0f, buoyancyScale));

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

        if (verticalDampingNPerMps > 0f)
        {
            float verticalSpeed = Vector3.Dot(rb.linearVelocity, Vector3.up);
            rb.AddForce(Vector3.up * (-verticalSpeed * verticalDampingNPerMps), ForceMode.Force);
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
