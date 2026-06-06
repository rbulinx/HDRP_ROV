using System.Collections;
using UnityEngine;
using UnityEngine.UI;

[RequireComponent(typeof(Camera))]
public class CameraImageDelay : MonoBehaviour
{
    public bool delayEnabled = false;
    [Range(0.03f, 2f)] public float delaySeconds = 0.3f;
    [Range(10, 60)] public int bufferFps = 30;
    [Range(0.25f, 1f)] public float renderScale = 0.75f;
    public int overlaySortingOrder = -1000;

    Camera sourceCamera;
    RenderTexture[] buffers;
    RenderTexture originalTargetTexture;
    Canvas canvas;
    RawImage outputImage;
    Camera displayCamera;
    Coroutine frameLoop;
    int writeIndex;
    int framesCaptured;
    int bufferWidth;
    int bufferHeight;
    bool delayActive;

    void Awake()
    {
        sourceCamera = GetComponent<Camera>();
        originalTargetTexture = sourceCamera.targetTexture;
    }

    void OnEnable()
    {
        ApplyState();
    }

    void OnDisable()
    {
        StopDelay();
    }

    void OnDestroy()
    {
        StopDelay();
        ReleaseBuffers();
        if (canvas != null)
            Destroy(canvas.gameObject);
        if (displayCamera != null)
            Destroy(displayCamera.gameObject);
    }

    void Update()
    {
        ApplyState();
    }

    void LateUpdate()
    {
        if (!delayActive || buffers == null || buffers.Length == 0 || outputImage == null)
            return;

        sourceCamera.targetTexture = buffers[writeIndex];

        int delayFrames = GetDelayFrames();
        int readIndex = Mathf.Max(0, framesCaptured) >= delayFrames
            ? (writeIndex - delayFrames + buffers.Length) % buffers.Length
            : 0;

        outputImage.texture = buffers[readIndex];
    }

    public void Configure(bool enabled, float seconds, int fps, float scale)
    {
        bool needsRebuild =
            Mathf.Abs(delaySeconds - seconds) > 0.0001f ||
            bufferFps != fps ||
            Mathf.Abs(renderScale - scale) > 0.0001f;

        delayEnabled = enabled;
        delaySeconds = Mathf.Clamp(seconds, 0.03f, 2f);
        bufferFps = Mathf.Clamp(fps, 10, 60);
        renderScale = Mathf.Clamp(scale, 0.25f, 1f);

        if (needsRebuild)
        {
            StopDelay();
            ReleaseBuffers();
        }

        ApplyState();
    }

    void ApplyState()
    {
        if (sourceCamera == null)
            sourceCamera = GetComponent<Camera>();

        if (delayEnabled && isActiveAndEnabled && sourceCamera.enabled)
            StartDelay();
        else
            StopDelay();
    }

    void StartDelay()
    {
        if (delayActive)
            return;

        originalTargetTexture = sourceCamera.targetTexture;
        AllocateBuffers();
        CreateOverlay();
        CreateDisplayCamera();

        writeIndex = 0;
        framesCaptured = 0;
        delayActive = true;

        if (frameLoop == null)
            frameLoop = StartCoroutine(AdvanceAtEndOfFrame());
    }

    void StopDelay()
    {
        if (frameLoop != null)
        {
            StopCoroutine(frameLoop);
            frameLoop = null;
        }

        if (sourceCamera != null && sourceCamera.targetTexture != originalTargetTexture)
            sourceCamera.targetTexture = originalTargetTexture;

        if (canvas != null)
            canvas.gameObject.SetActive(false);

        if (displayCamera != null)
            displayCamera.gameObject.SetActive(false);

        delayActive = false;
    }

    IEnumerator AdvanceAtEndOfFrame()
    {
        WaitForEndOfFrame wait = new WaitForEndOfFrame();
        while (true)
        {
            yield return wait;

            if (!delayActive || buffers == null || buffers.Length == 0)
                continue;

            framesCaptured++;
            writeIndex = (writeIndex + 1) % buffers.Length;
        }
    }

    void AllocateBuffers()
    {
        int count = GetBufferCount();
        int width = Mathf.Max(16, Mathf.RoundToInt(Screen.width * renderScale));
        int height = Mathf.Max(16, Mathf.RoundToInt(Screen.height * renderScale));
        if (buffers != null && buffers.Length == count && bufferWidth == width && bufferHeight == height)
            return;

        ReleaseBuffers();

        bufferWidth = width;
        bufferHeight = height;

        buffers = new RenderTexture[count];
        for (int i = 0; i < buffers.Length; i++)
        {
            RenderTexture rt = new RenderTexture(width, height, 24, RenderTextureFormat.ARGB32);
            rt.name = string.Format("{0}_DelayedFrame_{1:00}", sourceCamera.name, i);
            rt.filterMode = FilterMode.Bilinear;
            rt.wrapMode = TextureWrapMode.Clamp;
            rt.Create();
            buffers[i] = rt;
        }
    }

    void ReleaseBuffers()
    {
        if (buffers == null)
            return;

        for (int i = 0; i < buffers.Length; i++)
        {
            if (buffers[i] == null)
                continue;

            buffers[i].Release();
            Destroy(buffers[i]);
        }

        buffers = null;
    }

    void CreateOverlay()
    {
        if (canvas != null)
        {
            canvas.gameObject.SetActive(true);
            canvas.sortingOrder = overlaySortingOrder;
            return;
        }

        GameObject canvasObject = new GameObject(sourceCamera.name + "_DelayedImageCanvas");
        canvas = canvasObject.AddComponent<Canvas>();
        canvas.renderMode = RenderMode.ScreenSpaceOverlay;
        canvas.sortingOrder = overlaySortingOrder;

        CanvasScaler scaler = canvasObject.AddComponent<CanvasScaler>();
        scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
        scaler.referenceResolution = new Vector2(1920f, 1080f);

        GameObject imageObject = new GameObject("DelayedCameraImage");
        imageObject.transform.SetParent(canvasObject.transform, false);

        outputImage = imageObject.AddComponent<RawImage>();
        outputImage.raycastTarget = false;
        outputImage.color = Color.white;

        RectTransform rect = outputImage.rectTransform;
        rect.anchorMin = Vector2.zero;
        rect.anchorMax = Vector2.one;
        rect.pivot = new Vector2(0.5f, 0.5f);
        rect.anchoredPosition = Vector2.zero;
        rect.sizeDelta = Vector2.zero;
    }

    void CreateDisplayCamera()
    {
        if (displayCamera != null)
        {
            displayCamera.gameObject.SetActive(true);
            return;
        }

        GameObject cameraObject = new GameObject(sourceCamera.name + "_DelayedDisplayCamera");
        displayCamera = cameraObject.AddComponent<Camera>();
        displayCamera.clearFlags = CameraClearFlags.SolidColor;
        displayCamera.backgroundColor = Color.black;
        displayCamera.cullingMask = 0;
        displayCamera.orthographic = true;
        displayCamera.nearClipPlane = 0.01f;
        displayCamera.farClipPlane = 1f;
        displayCamera.depth = sourceCamera.depth - 100f;
        displayCamera.targetDisplay = sourceCamera.targetDisplay;
        displayCamera.allowHDR = false;
        displayCamera.allowMSAA = false;
    }

    int GetDelayFrames()
    {
        return Mathf.Max(1, Mathf.RoundToInt(delaySeconds * Mathf.Max(1, bufferFps)));
    }

    int GetBufferCount()
    {
        return Mathf.Clamp(GetDelayFrames() + 2, 3, 180);
    }
}
