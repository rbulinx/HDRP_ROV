using UnityEngine;

public class uboat : MonoBehaviour
{
    [SerializeField] private float surfaceY = 0f;
    [SerializeField] private float submergedY = -5f;
    [SerializeField] private float moveSpeed = 2f;
    [SerializeField] private float surfaceWaitTime = 3f;
    [SerializeField] private float submergedWaitTime = 3f;

    private float waitTimer;
    private bool isSubmerged;
    private bool isWaiting;

    void Start()
    {
        Vector3 position = transform.position;
        position.y = surfaceY;
        transform.position = position;
        waitTimer = surfaceWaitTime;
        isSubmerged = false;
        isWaiting = true;
    }

    void Update()
    {
        if (isWaiting)
        {
            waitTimer -= Time.deltaTime;
            if (waitTimer <= 0f)
            {
                isWaiting = false;
            }

            return;
        }

        float targetY = isSubmerged ? surfaceY : submergedY;
        Vector3 position = transform.position;
        position.y = Mathf.MoveTowards(position.y, targetY, moveSpeed * Time.deltaTime);
        transform.position = position;

        if (Mathf.Approximately(position.y, targetY))
        {
            isSubmerged = !isSubmerged;
            isWaiting = true;
            waitTimer = isSubmerged ? submergedWaitTime : surfaceWaitTime;
        }
    }
}
