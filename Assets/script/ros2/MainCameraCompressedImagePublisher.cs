using System;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEngine.Rendering;

/// <summary>
/// Publishes a UI-free copy of the currently active Main Camera as a ROS 2
/// sensor_msgs/msg/CompressedImage. The capture camera is separate from the
/// display camera, so this component never changes Camera.targetTexture.
/// </summary>
[DisallowMultipleComponent]
public sealed class MainCameraCompressedImagePublisher : MonoBehaviour
{
    const string BootstrapObjectName = "ROS2 Main Camera Publisher";

    [Header("ROS 2")]
    [SerializeField] string rosIPAddress = "192.168.50.188";
    [SerializeField, Range(1, 65535)] int rosPort = 10000;
    [SerializeField] string topicName = "/rov/camera/image/compressed";
    [SerializeField] string frameId = "main_camera_optical_frame";

    [Header("Image")]
    [SerializeField, Min(16)] int width = 1280;
    [SerializeField, Min(16)] int height = 720;
    [SerializeField, Range(1f, 60f)] float publishRateHz = 20f;
    [SerializeField, Range(1, 100)] int jpegQuality = 85;
    [SerializeField] bool flipVertically = false;

    ROSConnection ros;
    Camera captureCamera;
    RenderTexture captureTexture;
    Texture2D cpuTexture;
    Camera sourceCamera;
    Camera hdrpSettingsSource;
    float nextCaptureTime;
    float nextCameraSearchTime;
    bool captureScheduled;
    bool readbackPending;
    bool publisherRegistered;

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    static void Bootstrap()
    {
        MainCameraCompressedImagePublisher existing =
            FindFirstObjectByType<MainCameraCompressedImagePublisher>();
        if (existing != null)
            return;

        GameObject publisherObject = new GameObject(BootstrapObjectName);
        DontDestroyOnLoad(publisherObject);
        publisherObject.AddComponent<MainCameraCompressedImagePublisher>();
    }

    void OnEnable()
    {
        width = Mathf.Max(16, width);
        height = Mathf.Max(16, height);
        publishRateHz = Mathf.Max(1f, publishRateHz);

        ros = ROSConnection.GetOrCreateInstance();
        ros.RosIPAddress = rosIPAddress;
        ros.RosPort = rosPort;
        ros.ConnectOnStart = true;
        ros.RegisterPublisher<CompressedImageMsg>(topicName, queue_size: 1, latch: false);
        publisherRegistered = true;

        RenderPipelineManager.endCameraRendering += OnEndCameraRendering;
        CreateCaptureResources();
        FindMainCamera();
        nextCaptureTime = Time.unscaledTime;
    }

    void LateUpdate()
    {
        if (!publisherRegistered || captureScheduled || readbackPending)
            return;

        if (sourceCamera == null || !sourceCamera.isActiveAndEnabled ||
            !sourceCamera.CompareTag("MainCamera"))
        {
            if (Time.unscaledTime < nextCameraSearchTime)
                return;

            FindMainCamera();
            if (sourceCamera == null)
                return;
        }

        if (Time.unscaledTime < nextCaptureTime)
            return;

        nextCaptureTime = Time.unscaledTime + 1f / publishRateHz;
        CaptureCurrentCamera();
    }

    void FindMainCamera()
    {
        nextCameraSearchTime = Time.unscaledTime + 0.25f;
        Camera candidate = Camera.main;
        sourceCamera = candidate != null && candidate.isActiveAndEnabled
            ? candidate
            : null;
    }

    void CreateCaptureResources()
    {
        ReleaseCaptureResources();

        captureTexture = new RenderTexture(
            width,
            height,
            24,
            RenderTextureFormat.ARGB32,
            RenderTextureReadWrite.sRGB)
        {
            name = "ROS2_MainCamera_Capture",
            filterMode = FilterMode.Bilinear,
            wrapMode = TextureWrapMode.Clamp,
            useMipMap = false,
            autoGenerateMips = false
        };
        captureTexture.Create();

        cpuTexture = new Texture2D(width, height, TextureFormat.RGBA32, false, false)
        {
            name = "ROS2_MainCamera_JPEG_Encoder"
        };

        GameObject cameraObject = new GameObject("ROS2 Capture Camera");
        cameraObject.transform.SetParent(transform, false);
        captureCamera = cameraObject.AddComponent<Camera>();
        captureCamera.enabled = false;
    }

    void CaptureCurrentCamera()
    {
        if (captureCamera == null || captureTexture == null || sourceCamera == null)
            return;

        captureCamera.CopyFrom(sourceCamera);
        captureCamera.enabled = false;
        captureCamera.targetTexture = captureTexture;
        CopyHdrpCameraSettingsWhenSourceChanges();
        captureCamera.transform.SetPositionAndRotation(
            sourceCamera.transform.position,
            sourceCamera.transform.rotation);

        // Let HDRP render this camera normally. The endCameraRendering callback
        // starts the readback, avoiding Camera.Render(), which is fragile in SRP.
        captureScheduled = true;
        captureCamera.enabled = true;
    }

    void CopyHdrpCameraSettingsWhenSourceChanges()
    {
        if (sourceCamera == hdrpSettingsSource)
            return;

        hdrpSettingsSource = sourceCamera;
        Component sourceData = sourceCamera.GetComponent("HDAdditionalCameraData");
        if (sourceData == null)
            return;

        Type dataType = sourceData.GetType();
        Component captureData = captureCamera.GetComponent(dataType);
        if (captureData == null)
            captureData = captureCamera.gameObject.AddComponent(dataType);

        // Keep HDRP frame settings, post-processing and volume options aligned
        // without depending on version-specific HDAdditionalCameraData APIs.
        JsonUtility.FromJsonOverwrite(JsonUtility.ToJson(sourceData), captureData);
    }

    void OnEndCameraRendering(ScriptableRenderContext context, Camera renderedCamera)
    {
        if (!captureScheduled || renderedCamera != captureCamera)
            return;

        captureCamera.enabled = false;
        captureScheduled = false;

        readbackPending = true;
        AsyncGPUReadback.Request(
            captureTexture,
            0,
            TextureFormat.RGBA32,
            OnReadbackComplete);
    }

    void OnReadbackComplete(AsyncGPUReadbackRequest request)
    {
        readbackPending = false;

        if (this == null || !isActiveAndEnabled || request.hasError || cpuTexture == null)
            return;

        byte[] rgba = request.GetData<byte>().ToArray();
        if (flipVertically)
            FlipRowsInPlace(rgba, width, height, 4);

        cpuTexture.LoadRawTextureData(rgba);
        cpuTexture.Apply(false, false);
        byte[] jpeg = cpuTexture.EncodeToJPG(jpegQuality);

        CompressedImageMsg message = new CompressedImageMsg
        {
            header = new HeaderMsg
            {
                stamp = CreateRosTimestamp(),
                frame_id = frameId
            },
            format = "jpeg",
            data = jpeg
        };

        ros.Publish(topicName, message);
    }

    static TimeMsg CreateRosTimestamp()
    {
        DateTime utcNow = DateTime.UtcNow;
        long ticksSinceEpoch = utcNow.Ticks - DateTime.UnixEpoch.Ticks;
        long seconds = ticksSinceEpoch / TimeSpan.TicksPerSecond;
        long remainingTicks = ticksSinceEpoch % TimeSpan.TicksPerSecond;

        return new TimeMsg
        {
#if ROS2
            sec = (int)seconds,
#else
            sec = (uint)seconds,
#endif
            nanosec = (uint)(remainingTicks * 100L)
        };
    }

    static void FlipRowsInPlace(byte[] pixels, int imageWidth, int imageHeight, int bytesPerPixel)
    {
        int rowBytes = imageWidth * bytesPerPixel;
        byte[] rowBuffer = new byte[rowBytes];

        for (int y = 0; y < imageHeight / 2; y++)
        {
            int top = y * rowBytes;
            int bottom = (imageHeight - 1 - y) * rowBytes;

            Buffer.BlockCopy(pixels, top, rowBuffer, 0, rowBytes);
            Buffer.BlockCopy(pixels, bottom, pixels, top, rowBytes);
            Buffer.BlockCopy(rowBuffer, 0, pixels, bottom, rowBytes);
        }
    }

    void OnDisable()
    {
        RenderPipelineManager.endCameraRendering -= OnEndCameraRendering;
        captureScheduled = false;
        sourceCamera = null;
        hdrpSettingsSource = null;
        ReleaseCaptureResources();
    }

    void ReleaseCaptureResources()
    {
        if (captureCamera != null)
        {
            Destroy(captureCamera.gameObject);
            captureCamera = null;
        }

        if (captureTexture != null)
        {
            captureTexture.Release();
            Destroy(captureTexture);
            captureTexture = null;
        }

        if (cpuTexture != null)
        {
            Destroy(cpuTexture);
            cpuTexture = null;
        }
    }
}
