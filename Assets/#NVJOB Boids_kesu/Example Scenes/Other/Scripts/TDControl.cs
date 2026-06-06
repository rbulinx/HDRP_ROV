using UnityEngine;
using UnityEngine.InputSystem;

[AddComponentMenu("#NVJOB/Tools/TDControl")]
public class TDControl : MonoBehaviour
{
    [Header("Rotation Settings")]
    public float rotationSpeed = 180;
    public Vector2 mouseVerticaleClamp = new Vector2(-20, 20);
    public float smoothMouse = 3;

    [Header("Lift Settings")]
    public bool liftOn = true;
    public Vector2 liftClamp = new Vector2(-20, 20);
    public float smoothLift = 0.5f;

    [Header("Camera Settings")]
    public Transform camTransform;
    public Vector2 camClamp = new Vector2(-20, 20);
    public float smoothCam = 0.5f;

    Transform tr;
    Vector3 rotationStart;
    Vector3 positionStart;
    Vector3 cameraStart;
    float mouseX;
    float mouseY;
    float upCh;
    float upChCur;
    float upChVel;
    float camhVel;
    float camCh;
    float camChCur;

    void Awake()
    {
        tr = transform;
        rotationStart = tr.eulerAngles;
        positionStart = tr.position;
        cameraStart = camTransform.localPosition;
    }

    void LateUpdate()
    {
        Rotation();
        CameraTransform();
        if (liftOn) Lift();
    }

    void Rotation()
    {
        Vector2 mouseDelta = Mouse.current != null ? Mouse.current.delta.ReadValue() : Vector2.zero;
        if (mouseDelta.sqrMagnitude > 0f)
        {
            mouseX += rotationSpeed * 0.01f * mouseDelta.x * 0.3f;
            mouseY -= rotationSpeed * 0.01f * mouseDelta.y * 0.3f;
        }

        mouseY = Mathf.Clamp(mouseY, mouseVerticaleClamp.x, mouseVerticaleClamp.y);
        tr.rotation = Quaternion.Slerp(
            tr.rotation,
            Quaternion.Euler(mouseY, mouseX + rotationStart.y, 0),
            smoothMouse * Time.deltaTime);
    }

    void Lift()
    {
        float verticalInput = 0f;
        if (Keyboard.current != null)
        {
            if (Keyboard.current.wKey.isPressed || Keyboard.current.upArrowKey.isPressed) verticalInput += 1f;
            if (Keyboard.current.sKey.isPressed || Keyboard.current.downArrowKey.isPressed) verticalInput -= 1f;
        }

        upCh += verticalInput * 0.2f;
        upCh = Mathf.Clamp(upCh, liftClamp.x, liftClamp.y);
        upChCur = Mathf.SmoothDamp(upChCur, upCh, ref upChVel, smoothLift);
        tr.position = tr.TransformDirection(new Vector3(0, positionStart.y + upChCur, 0));
    }

    void CameraTransform()
    {
        float scrollY = Mouse.current != null ? Mouse.current.scroll.ReadValue().y : 0f;
        camCh += scrollY * 0.5f;
        camCh = Mathf.Clamp(camCh, camClamp.x, camClamp.y);
        camChCur = Mathf.SmoothDamp(camChCur, camCh, ref camhVel, smoothCam);
        camTransform.localPosition = new Vector3(cameraStart.x, cameraStart.y, cameraStart.z + camChCur);
    }
}
