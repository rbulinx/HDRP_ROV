using System;
using System.Reflection;
using UnityEngine;
using UnityEngine.UI;

public class ROVHUD : MonoBehaviour
{
    public Transform target;

    [Header("Depth")]
    public bool depthPositiveDown = true;
    public float referenceY = 0f;
    public string depthUnit = "m";

    [Header("Tension (Cable)")]
    public MonoBehaviour sourceCable;
    public bool autoFindCable = true;
    public bool showTension = true;
    public bool tensionInNewton = true;
    public string tensionLabel = "Tether";
    public string tensionUnavailableText = "Tether: ---";
    public float tensionWarningNewton = 250f;
    public float tensionCriticalNewton = 600f;
    public float tensionWarningStretchMeters = 0.10f;
    public string tensionNormalColor = "#FFFFFF";
    public bool showCableLengthDebug = false;

    [Header("Collision Count")]
    public ROVGamepadThrustController sourceController;
    public bool autoFindController = true;
    public bool showCollisionCount = true;
    public string collisionCountLabel = "Collisions";

    [Header("ROV Controls")]
    public bool showLightLevel = true;
    public string lightLevelLabel = "Light";
    public bool showControlGain = true;
    public string controlGainLabel = "Gain";
    public bool showHeadingLock = true;
    public string headingLockLabel = "Heading Lock";
    public string headingLockOnColor = "#66D9FF";
    public bool showAltitudeHold = true;
    public string altitudeHoldLabel = "ALT";
    public string altitudeHoldOnColor = "#66D9FF";
    public bool showAutoTetherPay = true;
    public string autoTetherPayLabel = "Auto Pay";
    public string autoTetherPayOnColor = "#66D9FF";
    public string autoTetherPayPayingColor = "#FF3333";

    [Header("Attitude")]
    public bool showAttitude = true;
    public string attitudeLabel = "Attitude";

    [Header("Turn Count")]
    public bool showTurnCount = true;
    public bool showTotalTurnCount = false;
    public string turnCountLabel = "Twist";
    public float twistWarningRevolutions = 1f;
    public float twistCriticalRevolutions = 2f;
    public string twistWarningColor = "#FFD24A";
    public string twistCriticalColor = "#FF0000";

    [Header("UI")]
    public int fontSize = 24;
    public Vector2 margin = new Vector2(16, 16);
    public float hudUpdateIntervalSeconds = 0.2f;

    Text hudText;
    bool hasPreviousHeading;
    float previousHeading;
    float accumulatedAbsYawDeg;
    float accumulatedSignedYawDeg;
    float nextHudUpdateTime;

    MethodInfo miGetTensionNewton;
    MethodInfo miGetCurrentTensionNewton;
    MethodInfo miGetCableBuoyancyLoadNewton;
    MethodInfo miGetBottomSegmentTensionNewton;
    MethodInfo miGetCableLengthMeters;
    MethodInfo miGetStretchMeters;

    void Awake()
    {
        EnsureControlReadoutsEnabled();

        if (target == null) target = transform;

        ResolveCableReference();
        ResolveControllerReference();
        CreateHudText();
    }

    void OnEnable()
    {
        hasPreviousHeading = false;
        previousHeading = 0f;
        accumulatedAbsYawDeg = 0f;
        accumulatedSignedYawDeg = 0f;
    }

    void OnValidate()
    {
        EnsureControlReadoutsEnabled();
        ResolveCableReference();
        ResolveControllerReference();
    }

    void EnsureControlReadoutsEnabled()
    {
        showLightLevel = true;
        showControlGain = true;

        if (string.IsNullOrWhiteSpace(lightLevelLabel))
            lightLevelLabel = "Light";

        if (string.IsNullOrWhiteSpace(controlGainLabel))
            controlGainLabel = "Gain";
    }

    void CreateHudText()
    {
        GameObject canvasGO = new GameObject("ROV_HUD_Canvas");
        Canvas canvas = canvasGO.AddComponent<Canvas>();
        canvas.renderMode = RenderMode.ScreenSpaceOverlay;
        canvas.sortingOrder = 999;

        canvasGO.AddComponent<CanvasScaler>();

        GameObject textGO = new GameObject("ROV_HUD_Text");
        textGO.transform.SetParent(canvasGO.transform, false);

        hudText = textGO.AddComponent<Text>();
        hudText.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
        hudText.fontSize = fontSize;
        hudText.alignment = TextAnchor.UpperLeft;
        hudText.horizontalOverflow = HorizontalWrapMode.Overflow;
        hudText.verticalOverflow = VerticalWrapMode.Overflow;
        hudText.supportRichText = true;
        hudText.color = Color.white;
        hudText.raycastTarget = false;

        Shadow shadow = textGO.AddComponent<Shadow>();
        shadow.effectDistance = new Vector2(1.5f, -1.5f);
        shadow.effectColor = new Color(0f, 0f, 0f, 0.8f);

        RectTransform rt = hudText.rectTransform;
        rt.anchorMin = new Vector2(0f, 1f);
        rt.anchorMax = new Vector2(0f, 1f);
        rt.pivot = new Vector2(0f, 1f);
        rt.anchoredPosition = new Vector2(margin.x, -margin.y);
        rt.sizeDelta = new Vector2(900f, 320f);
    }

    void ResolveCableReference()
    {
        if (sourceCable != null)
        {
            CacheCableAPIs(sourceCable);
            return;
        }

        if (!autoFindCable) return;

        MonoBehaviour[] mbs = UnityEngine.Object.FindObjectsByType<MonoBehaviour>(FindObjectsSortMode.None);
        for (int i = 0; i < mbs.Length; i++)
        {
            MonoBehaviour mb = mbs[i];
            if (mb == null) continue;

            if (TryCacheCableAPIs(mb))
            {
                sourceCable = mb;
                return;
            }
        }
    }

    void ResolveControllerReference()
    {
        if (sourceController != null) return;
        if (!autoFindController) return;

        if (target != null)
            sourceController = target.GetComponentInParent<ROVGamepadThrustController>();

        if (sourceController == null)
            sourceController = UnityEngine.Object.FindFirstObjectByType<ROVGamepadThrustController>();
    }

    void CacheCableAPIs(MonoBehaviour mb)
    {
        TryCacheCableAPIs(mb);
    }

    bool TryCacheCableAPIs(MonoBehaviour mb)
    {
        if (mb == null) return false;

        Type t = mb.GetType();

        miGetTensionNewton = t.GetMethod(
            "GetTensionNewton",
            BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic,
            null,
            Type.EmptyTypes,
            null);

        if (miGetTensionNewton == null) return false;
        if (miGetTensionNewton.ReturnType != typeof(float)) return false;

        miGetCableLengthMeters = t.GetMethod(
            "GetCableLengthMeters",
            BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic,
            null,
            Type.EmptyTypes,
            null);

        miGetCurrentTensionNewton = t.GetMethod(
            "GetCurrentTensionNewton",
            BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic,
            null,
            Type.EmptyTypes,
            null);

        miGetCableBuoyancyLoadNewton = t.GetMethod(
            "GetCableBuoyancyLoadNewton",
            BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic,
            null,
            Type.EmptyTypes,
            null);

        miGetBottomSegmentTensionNewton = t.GetMethod(
            "GetBottomSegmentTensionNewton",
            BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic,
            null,
            Type.EmptyTypes,
            null);

        miGetStretchMeters = t.GetMethod(
            "GetStretchMeters",
            BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic,
            null,
            Type.EmptyTypes,
            null);

        return true;
    }

    bool TryGetTension(out float tensionN)
    {
        tensionN = 0f;
        if (sourceCable == null) return false;

        if (sourceCable is CableXPBD xpbdCable)
        {
            tensionN = xpbdCable.GetTensionNewton();
            return !float.IsNaN(tensionN) && !float.IsInfinity(tensionN);
        }

        if (sourceCable is CableLMM_UnderwaterWinch_Collision_TensionLimit tensionLimitCable)
        {
            tensionN = tensionLimitCable.GetTensionNewton();
            return !float.IsNaN(tensionN) && !float.IsInfinity(tensionN);
        }

        if (miGetTensionNewton == null) return false;

        try
        {
            tensionN = (float)miGetTensionNewton.Invoke(sourceCable, null);
            return !float.IsNaN(tensionN) && !float.IsInfinity(tensionN);
        }
        catch
        {
            return false;
        }
    }

    bool TryGetCableLength(out float cableLength)
    {
        cableLength = 0f;
        if (sourceCable == null) return false;

        if (sourceCable is CableXPBD xpbdCable)
        {
            cableLength = xpbdCable.GetCableLengthMeters();
            return !float.IsNaN(cableLength) && !float.IsInfinity(cableLength);
        }

        if (miGetCableLengthMeters == null) return false;

        try
        {
            cableLength = (float)miGetCableLengthMeters.Invoke(sourceCable, null);
            return !float.IsNaN(cableLength) && !float.IsInfinity(cableLength);
        }
        catch
        {
            return false;
        }
    }

    bool TryGetCurrentTension(out float currentTensionN)
    {
        currentTensionN = 0f;
        if (sourceCable == null) return false;

        if (sourceCable is CableXPBD xpbdCable)
        {
            currentTensionN = xpbdCable.GetCurrentTensionNewton();
            return !float.IsNaN(currentTensionN) && !float.IsInfinity(currentTensionN);
        }

        if (miGetCurrentTensionNewton == null) return false;

        try
        {
            currentTensionN = (float)miGetCurrentTensionNewton.Invoke(sourceCable, null);
            return !float.IsNaN(currentTensionN) && !float.IsInfinity(currentTensionN);
        }
        catch
        {
            return false;
        }
    }

    bool TryGetCableBuoyancyLoad(out float cableBuoyancyLoadN)
    {
        cableBuoyancyLoadN = 0f;
        if (sourceCable == null) return false;

        if (sourceCable is CableXPBD xpbdCable)
        {
            cableBuoyancyLoadN = xpbdCable.GetCableBuoyancyLoadNewton();
            return !float.IsNaN(cableBuoyancyLoadN) && !float.IsInfinity(cableBuoyancyLoadN);
        }

        if (miGetCableBuoyancyLoadNewton == null) return false;

        try
        {
            cableBuoyancyLoadN = (float)miGetCableBuoyancyLoadNewton.Invoke(sourceCable, null);
            return !float.IsNaN(cableBuoyancyLoadN) && !float.IsInfinity(cableBuoyancyLoadN);
        }
        catch
        {
            return false;
        }
    }

    bool TryGetBottomSegmentTension(out float bottomSegmentTensionN)
    {
        bottomSegmentTensionN = 0f;
        if (sourceCable == null) return false;

        if (sourceCable is CableXPBD xpbdCable)
        {
            bottomSegmentTensionN = xpbdCable.GetBottomSegmentTensionNewton();
            return !float.IsNaN(bottomSegmentTensionN) && !float.IsInfinity(bottomSegmentTensionN);
        }

        if (miGetBottomSegmentTensionNewton == null) return false;

        try
        {
            bottomSegmentTensionN = (float)miGetBottomSegmentTensionNewton.Invoke(sourceCable, null);
            return !float.IsNaN(bottomSegmentTensionN) && !float.IsInfinity(bottomSegmentTensionN);
        }
        catch
        {
            return false;
        }
    }

    bool TryGetStretch(out float stretch)
    {
        stretch = 0f;
        if (sourceCable == null) return false;

        if (sourceCable is CableXPBD xpbdCable)
        {
            stretch = xpbdCable.GetStretchMeters();
            return !float.IsNaN(stretch) && !float.IsInfinity(stretch);
        }

        if (sourceCable is CableLMM_UnderwaterWinch_Collision_TensionLimit tensionLimitCable)
        {
            stretch = tensionLimitCable.GetStraightExcessMeters();
            return !float.IsNaN(stretch) && !float.IsInfinity(stretch);
        }

        if (miGetStretchMeters == null) return false;

        try
        {
            stretch = (float)miGetStretchMeters.Invoke(sourceCable, null);
            return !float.IsNaN(stretch) && !float.IsInfinity(stretch);
        }
        catch
        {
            return false;
        }
    }

    void Update()
    {
        if (target == null || hudText == null) return;

        float y = target.position.y;
        float depth = y - referenceY;
        if (depthPositiveDown) depth = -depth;

        float heading = target.eulerAngles.y;
        UpdateTurnCount(heading);

        if (Time.unscaledTime < nextHudUpdateTime)
            return;

        nextHudUpdateTime = Time.unscaledTime + Mathf.Max(0.05f, hudUpdateIntervalSeconds);

        if ((showCollisionCount || showLightLevel || showControlGain || showHeadingLock || showAltitudeHold || showAutoTetherPay) && sourceController == null)
            ResolveControllerReference();

        string tensionLine = "";
        string cableDebugLines = "";

        if (showTension)
        {
            if (sourceCable == null || miGetTensionNewton == null)
                ResolveCableReference();

            if (TryGetTension(out float tensionN))
            {
                float tensionStretch = 0f;
                TryGetStretch(out tensionStretch);
                tensionLine = BuildTensionLine(tensionN, tensionStretch);

                if (showCableLengthDebug)
                {
                    if (TryGetCableLength(out float cableLength))
                        cableDebugLines += $"\nCable L: {cableLength:0.00} m";

                    if (TryGetCurrentTension(out float currentTensionN))
                        cableDebugLines += $"\nCurrent T: {currentTensionN:0} N";

                    if (TryGetCableBuoyancyLoad(out float cableBuoyancyLoadN))
                        cableDebugLines += $"\nCable buoy: {cableBuoyancyLoadN:0} N";

                    if (TryGetBottomSegmentTension(out float bottomSegmentTensionN))
                        cableDebugLines += $"\nBottom seg T: {bottomSegmentTensionN:0} N";

                    if (TryGetStretch(out float debugStretch))
                        cableDebugLines += $"\nStretch: {debugStretch:0.000} m";
                }
            }
            else
            {
                tensionLine = tensionUnavailableText;
            }
        }

        hudText.text =
            $"Depth(Y): {depth:0.00} {depthUnit}\n" +
            $"Heading: {heading:0.0} deg" +
            (showAttitude ? "\n" + BuildAttitudeLine() : "") +
            (showTurnCount ? "\n" + BuildTurnCountLine() : "") +
            (showCollisionCount ? "\n" + BuildCollisionCountLine() : "") +
            (showLightLevel ? "\n" + BuildLightLevelLine() : "") +
            (showControlGain ? "\n" + BuildControlGainLine() : "") +
            (showHeadingLock ? "\n" + BuildHeadingLockLine() : "") +
            (showAltitudeHold ? "\n" + BuildAltitudeHoldLine() : "") +
            (showAutoTetherPay ? "\n" + BuildAutoTetherPayLine() : "") +
            (showTension ? "\n" + tensionLine + cableDebugLines : "");
    }

    public void ResetTurnCount()
    {
        hasPreviousHeading = false;
        previousHeading = 0f;
        accumulatedAbsYawDeg = 0f;
        accumulatedSignedYawDeg = 0f;
    }

    void UpdateTurnCount(float heading)
    {
        if (!hasPreviousHeading)
        {
            previousHeading = heading;
            hasPreviousHeading = true;
            return;
        }

        float delta = Mathf.DeltaAngle(previousHeading, heading);
        accumulatedAbsYawDeg += Mathf.Abs(delta);
        accumulatedSignedYawDeg += delta;
        previousHeading = heading;
    }

    string BuildTurnCountLine()
    {
        float turns = accumulatedAbsYawDeg / 360f;
        float signedTurns = accumulatedSignedYawDeg / 360f;
        string color = "";

        if (turns >= twistCriticalRevolutions)
            color = twistCriticalColor;
        else if (turns >= twistWarningRevolutions)
            color = twistWarningColor;

        string line;
        if (!showTotalTurnCount)
            line = $"{turnCountLabel}: {signedTurns:+0.00;-0.00;0.00} rev";
        else
            line = $"{turnCountLabel}: {signedTurns:+0.00;-0.00;0.00} rev  Total: {turns:0.00}";

        return string.IsNullOrEmpty(color) ? line : $"<color={color}>{line}</color>";
    }

    string BuildAttitudeLine()
    {
        Vector3 euler = target.eulerAngles;
        float pitch = NormalizeSignedAngle(euler.x);
        float roll = NormalizeSignedAngle(euler.z);
        return $"{attitudeLabel}: P {pitch:+0.0;-0.0;0.0} deg  R {roll:+0.0;-0.0;0.0} deg";
    }

    static float NormalizeSignedAngle(float deg)
    {
        while (deg > 180f) deg -= 360f;
        while (deg < -180f) deg += 360f;
        return deg;
    }

    string BuildCollisionCountLine()
    {
        int count = sourceController != null ? sourceController.CollisionCount : 0;
        return $"{collisionCountLabel}: {count}";
    }

    string BuildLightLevelLine()
    {
        if (sourceController == null)
            return $"{lightLevelLabel}: ---";

        float percent = sourceController.LightLevel01 * 100f;
        return $"{lightLevelLabel}: {percent:0}% ({sourceController.LightIntensity:0})";
    }

    string BuildControlGainLine()
    {
        if (sourceController == null)
            return $"{controlGainLabel}: ---";

        return $"{controlGainLabel}: x{sourceController.ControlResponseGain:0.00}";
    }

    string BuildHeadingLockLine()
    {
        if (sourceController == null)
            return $"{headingLockLabel}: ---";

        if (!sourceController.HeadingLockEnabled)
            return $"{headingLockLabel}: OFF";

        string line = $"{headingLockLabel}: ON {sourceController.HeadingLockTargetDeg:0} deg";
        return $"<color={headingLockOnColor}>{line}</color>";
    }

    string BuildAltitudeHoldLine()
    {
        if (sourceController == null)
            return $"{altitudeHoldLabel}: ---";

        string measured = sourceController.AltitudeHoldHasGround
            ? $"{sourceController.AltitudeHoldMeasuredMeters:0.00} m"
            : "---";

        if (!sourceController.AltitudeHoldEnabled)
            return $"{altitudeHoldLabel}: {measured}  OFF";

        string line = $"{altitudeHoldLabel}: ON {measured} / {sourceController.AltitudeHoldTargetMeters:0.00} m";
        return $"<color={altitudeHoldOnColor}>{line}</color>";
    }

    string BuildAutoTetherPayLine()
    {
        if (sourceController == null)
            return $"{autoTetherPayLabel}: ---";

        if (!sourceController.AutoTetherPayEnabled)
            return $"{autoTetherPayLabel}: OFF";

        string state = sourceController.AutoTetherPayIsPaying ? "PAY" : "ARMED";
        string color = sourceController.AutoTetherPayIsPaying ? autoTetherPayPayingColor : autoTetherPayOnColor;
        string line = $"{autoTetherPayLabel}: {state} {sourceController.AutoTetherPayTensionNewton:0} N";
        return $"<color={color}>{line}</color>";
    }

    string BuildTensionLine(float tensionN, float stretchMeters)
    {
        string value = tensionInNewton
            ? $"{tensionN:0} N"
            : $"{tensionN / 1000f:0.00} kN";

        string color = tensionNormalColor;
        string state = "";

        if (tensionN >= tensionCriticalNewton)
        {
            color = "#FF0000";
            state = "  HIGH";
        }
        else if (tensionN >= tensionWarningNewton || stretchMeters >= tensionWarningStretchMeters)
        {
            color = "#FF0000";
            state = "  TENSION";
        }

        return $"<color={color}>{tensionLabel}: {value}{state}</color>";
    }
}
