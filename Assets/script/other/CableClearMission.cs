using UnityEngine;
using UnityEngine.UI;
using UnityEngine.SceneManagement;
[DefaultExecutionOrder(200)]
public class CableClearMission : MonoBehaviour
{
    const string MissionLogicVersion = "raycast-no-ignore-v2";

    [Header("Auto Find")]
    public string sceneName = SceneSelector.UBoatSceneName;
    public string targetObjectName = "U_Boat";
    public string cableObjectName = "power-cable";

    [Header("Mission Condition")]
    public float clearHoldSeconds = 1.5f;
    public int ignoreTopAnchorSegments = 0;
    public int ignoreBottomAnchorSegments = 0;
    public float rayStartOffsetY = 0.2f;
    public float raycastDistance = 80f;
    public int segmentSampleDivisions = 2;
    [Tooltip("ミッション判定の実行間隔。毎フレームではなく間引いて負荷を下げます。")]
    public float detectionIntervalSeconds = 0.35f;
    public int maxRaycastHits = 16;
    public float targetBoundsPadding = 0.75f;
    public bool verboseDetectionLogs = false;

    [Header("Completion Motion")]
    public float riseSpeedMetersPerSecond = 0.4f;
    public float riseDistanceMeters = 8f;

    [Header("UI")]
    public int fontSize = 26;
    public Color activeColor = new Color(1f, 0.9f, 0.3f, 1f);
    public Color completeColor = new Color(0.3f, 1f, 0.4f, 1f);

    Transform targetTransform;
    CableXPBD cable;
    Text missionText;
    GameObject missionCanvasObject;
    bool missionCompleted;
    bool missionArmed;
    bool lastCableOverTarget;
    bool referencesLogged;
    float clearTimer;
    float nextDetectionTime;
    bool cachedCableOverTarget;
    Vector3 completionTargetPosition;
    bool completionMotionInitialized;
    RaycastHit[] hitBuffer;
    Collider[] targetColliders;
    Bounds targetBounds;
    bool hasTargetBounds;

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    static void CreateIfNeeded()
    {
        EnsureInstance();
    }

    public static void EnsureInstance()
    {
        if (!SceneSelector.SelectedMissionEnabled)
            return;

        if (SceneManager.GetActiveScene().name != SceneSelector.UBoatSceneName)
            return;

        if (Object.FindFirstObjectByType<CableClearMission>() != null)
            return;

        GameObject go = new GameObject("CableClearMission");
        go.AddComponent<CableClearMission>();
    }

    public static void RemoveInstances()
    {
        CableClearMission[] missions = Object.FindObjectsByType<CableClearMission>(FindObjectsInactive.Include, FindObjectsSortMode.None);
        for (int i = 0; i < missions.Length; i++)
        {
            if (missions[i] != null)
                Object.Destroy(missions[i].gameObject);
        }
    }

    void Awake()
    {
        if (!SceneSelector.SelectedMissionEnabled)
        {
            Destroy(gameObject);
            return;
        }

        if (SceneManager.GetActiveScene().name != sceneName)
        {
            enabled = false;
            return;
        }

        TryResolveReferences();
        hitBuffer = new RaycastHit[Mathf.Max(1, maxRaycastHits)];
        BuildUi();
        UpdateMissionText(false);
    }

    void OnDestroy()
    {
        if (missionCanvasObject != null)
            Destroy(missionCanvasObject);
    }

    void Update()
    {
        if (missionCompleted) return;

        if (targetTransform == null || cable == null)
            TryResolveReferences();

        if (targetTransform == null)
        {
            if (missionText != null)
                missionText.text = "MISSION: resolving target/cable...";
            return;
        }

        bool cableOverTarget = cachedCableOverTarget;
        if (Time.time >= nextDetectionTime)
        {
            nextDetectionTime = Time.time + Mathf.Max(0.02f, detectionIntervalSeconds);
            cableOverTarget = IsCableOverTarget();
            cachedCableOverTarget = cableOverTarget;
        }

        if (cableOverTarget != lastCableOverTarget)
        {
            lastCableOverTarget = cableOverTarget;
            Debug.Log($"[Mission] cableOverTarget={cableOverTarget} clearTimer={clearTimer:0.00}");
        }

        if (cableOverTarget)
        {
            if (!missionArmed)
            {
                missionArmed = true;
                Debug.Log("[Mission] Mission armed: cable detected above U-boat.");
            }

            clearTimer = 0f;
            UpdateMissionText(false);
            return;
        }

        if (!missionArmed)
        {
            clearTimer = 0f;
            UpdateMissionText(false);
            return;
        }

        clearTimer += Time.deltaTime;
        if (clearTimer >= clearHoldSeconds)
        {
            missionCompleted = true;
            BeginCompletionMotion();
            UpdateMissionText(true);
            Debug.Log("[Mission] Mission complete: power cable cleared from above U-boat.");
        }
        else
        {
            UpdateMissionText(false, true);
        }
    }

    void LateUpdate()
    {
        if (!missionCompleted || targetTransform == null || !completionMotionInitialized)
            return;

        Vector3 current = targetTransform.position;
        Vector3 next = Vector3.MoveTowards(current, completionTargetPosition, riseSpeedMetersPerSecond * Time.deltaTime);
        targetTransform.position = next;
    }

    void TryResolveReferences()
    {
        if (targetTransform == null)
            targetTransform = GameObject.Find(targetObjectName)?.transform;

        if (cable == null)
        {
            GameObject cableObject = GameObject.Find(cableObjectName);
            cable = cableObject != null ? cableObject.GetComponent<CableXPBD>() : null;
            cable ??= FindFirstObjectByType<CableXPBD>();
        }

        if (!referencesLogged && targetTransform != null)
        {
            RefreshTargetBounds();
            referencesLogged = true;
            Debug.Log(
                $"[Mission] logic={MissionLogicVersion} target='{targetTransform.name}' cable='{(cable != null ? cable.name : "null")}' " +
                $"nodes={(cable != null ? cable.GetNodeCount() : 0)} rayStartOffsetY={rayStartOffsetY} raycastDistance={raycastDistance}");
        }
    }

    bool IsCableOverTarget()
    {
        if (cable == null) return false;
        RefreshTargetBounds();

        int nodeCount = cable.GetNodeCount();
        if (nodeCount < 2) return false;

        int firstNode = Mathf.Max(0, ignoreBottomAnchorSegments);
        int lastNodeExclusive = Mathf.Max(firstNode + 1, nodeCount - Mathf.Max(0, ignoreTopAnchorSegments));
        int divisions = Mathf.Max(1, segmentSampleDivisions);

        for (int i = firstNode; i < lastNodeExclusive - 1; i++)
        {
            Vector3 p0 = cable.GetNodePosition(i);
            Vector3 p1 = cable.GetNodePosition(i + 1);

            for (int s = 0; s <= divisions; s++)
            {
                float t = s / (float)divisions;
                Vector3 sample = Vector3.Lerp(p0, p1, t);
                if (!IsNearTargetXZ(sample))
                    continue;

                if (HasTargetBelow(sample, out RaycastHit hit))
                {
                    LogDetection($"Node segment over target: node={i} t={t:0.00}", hit);
                    return true;
                }
            }
        }

        return false;
    }

    void RefreshTargetBounds()
    {
        hasTargetBounds = false;
        if (targetTransform == null) return;

        if (targetColliders == null || targetColliders.Length == 0)
            targetColliders = targetTransform.GetComponentsInChildren<Collider>(true);

        if (targetColliders == null || targetColliders.Length == 0) return;

        for (int i = 0; i < targetColliders.Length; i++)
        {
            Collider col = targetColliders[i];
            if (col == null || !col.enabled) continue;

            if (!hasTargetBounds)
            {
                targetBounds = col.bounds;
                hasTargetBounds = true;
            }
            else
            {
                targetBounds.Encapsulate(col.bounds);
            }
        }
    }

    bool IsNearTargetXZ(Vector3 worldPoint)
    {
        if (!hasTargetBounds) return true;

        float padding = Mathf.Max(0f, targetBoundsPadding);
        return worldPoint.x >= targetBounds.min.x - padding &&
               worldPoint.x <= targetBounds.max.x + padding &&
               worldPoint.z >= targetBounds.min.z - padding &&
               worldPoint.z <= targetBounds.max.z + padding;
    }

    bool HasTargetBelow(Vector3 worldPoint, out RaycastHit targetHit)
    {
        if (hitBuffer == null || hitBuffer.Length != Mathf.Max(1, maxRaycastHits))
            hitBuffer = new RaycastHit[Mathf.Max(1, maxRaycastHits)];

        Vector3 origin = worldPoint + Vector3.up * Mathf.Max(0f, rayStartOffsetY);
        Ray ray = new Ray(origin, Vector3.down);
        int hitCount = Physics.RaycastNonAlloc(ray, hitBuffer, Mathf.Max(0.1f, raycastDistance), ~0, QueryTriggerInteraction.Collide);

        for (int i = 0; i < hitCount; i++)
        {
            if (hitBuffer[i].collider == null) continue;
            Transform hitTransform = hitBuffer[i].collider.transform;
            if (hitTransform == targetTransform || hitTransform.IsChildOf(targetTransform))
            {
                targetHit = hitBuffer[i];
                return true;
            }
        }

        targetHit = default;
        return false;
    }

    void LogDetection(string prefix, RaycastHit hit)
    {
        if (!verboseDetectionLogs || targetTransform == null)
            return;

        Vector3 localHit = targetTransform.InverseTransformPoint(hit.point);
        Debug.Log($"[Mission] {prefix} hit='{hit.collider.name}' hitLocal={localHit}");
    }

    void BeginCompletionMotion()
    {
        if (targetTransform == null)
            return;

        Vector3 startPosition = targetTransform.position;
        completionTargetPosition = startPosition + Vector3.up * Mathf.Max(0f, riseDistanceMeters);
        completionMotionInitialized = true;
        Debug.Log($"[Mission] U-boat rising: startY={startPosition.y:0.00} targetY={completionTargetPosition.y:0.00} speed={riseSpeedMetersPerSecond:0.00}");
    }

    void BuildUi()
    {
        GameObject canvasObject = new GameObject("MissionCanvas");
        missionCanvasObject = canvasObject;
        Canvas canvas = canvasObject.AddComponent<Canvas>();
        canvas.renderMode = RenderMode.ScreenSpaceOverlay;
        canvas.sortingOrder = 1200;
        canvasObject.AddComponent<CanvasScaler>();
        canvasObject.AddComponent<GraphicRaycaster>();

        GameObject textObject = new GameObject("MissionText");
        textObject.transform.SetParent(canvasObject.transform, false);
        missionText = textObject.AddComponent<Text>();
        missionText.font = LoadMissionFontV2();
        missionText.fontSize = fontSize;
        missionText.fontStyle = FontStyle.Bold;
        missionText.alignment = TextAnchor.UpperCenter;
        missionText.horizontalOverflow = HorizontalWrapMode.Overflow;
        missionText.verticalOverflow = VerticalWrapMode.Overflow;

        Shadow shadow = textObject.AddComponent<Shadow>();
        shadow.effectDistance = new Vector2(2f, -2f);
        shadow.effectColor = new Color(0f, 0f, 0f, 0.8f);

        RectTransform rect = missionText.rectTransform;
        rect.anchorMin = new Vector2(0.5f, 1f);
        rect.anchorMax = new Vector2(0.5f, 1f);
        rect.pivot = new Vector2(0.5f, 1f);
        rect.anchoredPosition = new Vector2(0f, -16f);
        rect.sizeDelta = new Vector2(900f, 60f);
    }

    static Font LoadMissionFontV2()
    {
        try
        {
            Font font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            if (font != null)
                return font;
        }
        catch (System.Exception ex)
        {
            Debug.LogWarning($"[Mission] LoadMissionFontV2 failed: {ex.Message}");
        }

        Debug.LogWarning("[Mission] LoadMissionFontV2 could not resolve LegacyRuntime.ttf.");
        return null;
    }

    void UpdateMissionText(bool completed, bool clearingInProgress = false)
    {
        if (missionText == null) return;

        if (completed)
        {
            missionText.color = completeColor;
            missionText.text = "MISSION COMPLETE\nPower cable cleared from above U-boat";
            return;
        }

        missionText.color = activeColor;
        if (clearingInProgress)
            missionText.text = $"MISSION\nMove the power cable off the U-boat ({clearTimer:0.0}/{clearHoldSeconds:0.0}s)";
        else
            missionText.text = "MISSION\nMove the power cable off the U-boat";
    }
}
