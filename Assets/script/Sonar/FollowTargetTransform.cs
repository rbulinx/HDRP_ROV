using UnityEngine;

public class FollowTargetTransform : MonoBehaviour
{
    [Header("Target")]
    public Transform target;

    [Header("Options")]
    public bool followPosition = true;
    public bool followRotation = true;
    public bool followInLateUpdate = true;

    Vector3 initialLocalOffset;
    Quaternion initialLocalRotationOffset;

    void Awake()
    {
        if (target == null) return;
        initialLocalOffset = Quaternion.Inverse(target.rotation) * (transform.position - target.position);
        initialLocalRotationOffset = Quaternion.Inverse(target.rotation) * transform.rotation;
    }

    void Update()
    {
        if (!followInLateUpdate) ApplyFollow();
    }

    void LateUpdate()
    {
        if (followInLateUpdate) ApplyFollow();
    }

    void ApplyFollow()
    {
        if (target == null) return;

        if (followPosition)
        {
            transform.position = target.position + target.rotation * initialLocalOffset;
        }

        if (followRotation)
        {
            transform.rotation = target.rotation * initialLocalRotationOffset;
        }
    }
}
