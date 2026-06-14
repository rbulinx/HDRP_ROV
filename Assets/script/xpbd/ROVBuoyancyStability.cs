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

    [Header("Buoyancy")]
    public bool setRigidbodyCenterOfMass = true;
    public bool useUnityGravity = true;
    public float buoyancyScale = 1.0f;
    public float verticalDampingNPerMps = 120f;

    [Header("Rotation Damping")]
    public float rollPitchAngularDamping = 0.25f;

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

        Vector3 buoyancyPoint = transform.TransformPoint(GetEffectiveCenterOfBuoyancyLocal());
        Vector3 buoyancyForce = -gravity.normalized * (rb.mass * gravityMagnitude * Mathf.Max(0f, buoyancyScale));
        rb.AddForceAtPosition(buoyancyForce, buoyancyPoint, ForceMode.Force);

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
        if (!useTrimmedRestAttitude)
            return centerOfBuoyancyLocalOffset;

        Quaternion rest = Quaternion.Euler(restPitchDeg, 0f, restRollDeg);
        Vector3 separation = Quaternion.Inverse(rest) * (Vector3.up * Mathf.Max(0.001f, metacentricHeight));
        return centerOfMassLocalOffset + separation;
    }

    void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.yellow;
        Gizmos.DrawSphere(transform.TransformPoint(centerOfMassLocalOffset), 0.08f);

        Gizmos.color = Color.cyan;
        Vector3 buoyancyLocal = GetEffectiveCenterOfBuoyancyLocal();
        Gizmos.DrawSphere(transform.TransformPoint(buoyancyLocal), 0.08f);
        Gizmos.DrawLine(transform.TransformPoint(centerOfMassLocalOffset), transform.TransformPoint(buoyancyLocal));
    }
}
