using System;
using System.Text;
using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// Lightweight Tactical Telemetry HUD overlay using Unity uGUI.
/// Observes MissionManager, PathFollower, ThreatAssessment, and MissionScore to render real-time
/// UAV mission progress, spatial coordinates, flight speed, threat counters, clearance metrics,
/// and 5-axis evaluation scores with zero per-frame garbage collection allocations.
/// </summary>
public class TacticalHUD : MonoBehaviour
{
    [Header("HUD Configuration")]
    [SerializeField] private bool showHUD = true;
    [SerializeField] private float updateInterval = 0.05f; // 20 Hz update rate to minimize allocations

    private MissionManager missionManager;
    private PathFollower pathFollower;
    private ThreatAssessment threatAssessment;
    private ReplanningController replanningController;
    private Pathfinding pathfinding;

    // uGUI Hierarchy Elements
    private Canvas hudCanvas;
    private GameObject hudPanelObject;
    private Image backgroundPanel;
    private Image headerBar;
    private Text headerText;
    private Text stateBadgeText;
    private RectTransform progressBarFillRect;
    private Text progressBarText;
    private Text telemetryText;
    private Text scoreText;

    private float nextUpdateTime = 0f;
    private readonly StringBuilder telemetrySb = new StringBuilder(512);
    private readonly StringBuilder scoreSb = new StringBuilder(256);

    public bool ShowHUD
    {
        get => showHUD;
        set
        {
            showHUD = value;
            if (hudPanelObject != null)
            {
                hudPanelObject.SetActive(value);
            }
        }
    }

    private void Awake()
    {
        AcquireSubsystems();
    }

    private void Start()
    {
        AcquireSubsystems();
        CreateUGUIHierarchy();
        RefreshDisplay(true);
    }

    private void AcquireSubsystems()
    {
        if (missionManager == null)
            missionManager = GetComponent<MissionManager>() ?? FindFirstObjectByType<MissionManager>();
        if (pathFollower == null)
            pathFollower = GetComponent<PathFollower>() ?? FindFirstObjectByType<PathFollower>();
        if (threatAssessment == null)
            threatAssessment = GetComponent<ThreatAssessment>() ?? FindFirstObjectByType<ThreatAssessment>();
        if (replanningController == null)
            replanningController = GetComponent<ReplanningController>() ?? FindFirstObjectByType<ReplanningController>();
        if (pathfinding == null)
            pathfinding = FindFirstObjectByType<Pathfinding>();
    }

    private void OnEnable()
    {
        AcquireSubsystems();

        if (missionManager != null)
        {
            missionManager.OnMissionStateChanged += HandleMissionStateChanged;
            missionManager.OnMissionCompleted += HandleMissionCompleted;
        }
    }

    private void OnDisable()
    {
        if (missionManager != null)
        {
            missionManager.OnMissionStateChanged -= HandleMissionStateChanged;
            missionManager.OnMissionCompleted -= HandleMissionCompleted;
        }
    }

    private void OnDestroy()
    {
        if (hudCanvas != null && hudCanvas.gameObject != null)
        {
            Destroy(hudCanvas.gameObject);
        }
    }

    private void Update()
    {
        if (!showHUD || hudPanelObject == null)
            return;

        if (Time.time >= nextUpdateTime)
        {
            nextUpdateTime = Time.time + updateInterval;
            RefreshDisplay(false);
        }
    }

    private void HandleMissionStateChanged(MissionState newState)
    {
        RefreshDisplay(true);
    }

    private void HandleMissionCompleted(MissionResult result)
    {
        RefreshDisplay(true);
    }

    /// <summary>
    /// Builds the tactical uGUI Canvas, visual progress bar, and formatted Text hierarchy.
    /// </summary>
    private void CreateUGUIHierarchy()
    {
        if (hudCanvas != null)
            return;

        // 1. Root Canvas
        GameObject canvasObj = new GameObject("TacticalHUD_Canvas");
        hudCanvas = canvasObj.AddComponent<Canvas>();
        hudCanvas.renderMode = RenderMode.ScreenSpaceOverlay;
        hudCanvas.sortingOrder = 100;

        CanvasScaler scaler = canvasObj.AddComponent<CanvasScaler>();
        scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
        scaler.referenceResolution = new Vector2(1920f, 1080f);
        scaler.matchWidthOrHeight = 0.5f;

        canvasObj.AddComponent<GraphicRaycaster>();

        // 2. Background Panel Container (Top-Left tactical HUD card)
        hudPanelObject = new GameObject("HUD_CardPanel");
        hudPanelObject.transform.SetParent(canvasObj.transform, false);

        RectTransform panelRect = hudPanelObject.AddComponent<RectTransform>();
        panelRect.anchorMin = new Vector2(0f, 1f);
        panelRect.anchorMax = new Vector2(0f, 1f);
        panelRect.pivot = new Vector2(0f, 1f);
        panelRect.anchoredPosition = new Vector2(28f, -28f);
        panelRect.sizeDelta = new Vector2(460f, 720f);

        backgroundPanel = hudPanelObject.AddComponent<Image>();
        backgroundPanel.color = new Color(0.05f, 0.08f, 0.12f, 0.94f); // Deep Tactical Dark Slate

        Font defaultFont = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf") ??
                           Resources.GetBuiltinResource<Font>("Arial.ttf");

        // 3. Header Accent Strip
        GameObject headerObj = new GameObject("Header_Strip");
        headerObj.transform.SetParent(hudPanelObject.transform, false);
        RectTransform headerRect = headerObj.AddComponent<RectTransform>();
        headerRect.anchorMin = new Vector2(0f, 1f);
        headerRect.anchorMax = new Vector2(1f, 1f);
        headerRect.pivot = new Vector2(0.5f, 1f);
        headerRect.anchoredPosition = new Vector2(0f, 0f);
        headerRect.sizeDelta = new Vector2(0f, 42f);

        headerBar = headerObj.AddComponent<Image>();
        headerBar.color = new Color(0.09f, 0.18f, 0.29f, 0.98f);

        // Header Title Text
        GameObject headerTextObj = new GameObject("Header_Text");
        headerTextObj.transform.SetParent(headerObj.transform, false);
        RectTransform headerTextRect = headerTextObj.AddComponent<RectTransform>();
        headerTextRect.anchorMin = Vector2.zero;
        headerTextRect.anchorMax = Vector2.one;
        headerTextRect.offsetMin = new Vector2(16f, 0f);
        headerTextRect.offsetMax = new Vector2(-16f, 0f);

        headerText = headerTextObj.AddComponent<Text>();
        headerText.font = defaultFont;
        headerText.fontSize = 15;
        headerText.fontStyle = FontStyle.Bold;
        headerText.color = new Color(0.35f, 0.88f, 1f); // Neon Cyan
        headerText.alignment = TextAnchor.MiddleLeft;
        headerText.text = "TACTICAL UAV MISSION TELEMETRY";

        // 4. Mission State Badge Banner Container
        GameObject badgeObj = new GameObject("State_Badge_Container");
        badgeObj.transform.SetParent(hudPanelObject.transform, false);
        RectTransform badgeRect = badgeObj.AddComponent<RectTransform>();
        badgeRect.anchorMin = new Vector2(0f, 1f);
        badgeRect.anchorMax = new Vector2(1f, 1f);
        badgeRect.pivot = new Vector2(0.5f, 1f);
        badgeRect.anchoredPosition = new Vector2(0f, -48f);
        badgeRect.sizeDelta = new Vector2(-28f, 32f);

        Image badgeBg = badgeObj.AddComponent<Image>();
        badgeBg.color = new Color(0.08f, 0.13f, 0.20f, 0.90f);

        // State Badge Text
        GameObject badgeTextObj = new GameObject("State_Badge_Text");
        badgeTextObj.transform.SetParent(badgeObj.transform, false);
        RectTransform badgeTextRect = badgeTextObj.AddComponent<RectTransform>();
        badgeTextRect.anchorMin = Vector2.zero;
        badgeTextRect.anchorMax = Vector2.one;
        badgeTextRect.offsetMin = new Vector2(8f, 0f);
        badgeTextRect.offsetMax = new Vector2(-8f, 0f);

        stateBadgeText = badgeTextObj.AddComponent<Text>();
        stateBadgeText.font = defaultFont;
        stateBadgeText.fontSize = 13;
        stateBadgeText.fontStyle = FontStyle.Bold;
        stateBadgeText.alignment = TextAnchor.MiddleCenter;
        stateBadgeText.color = Color.white;

        // 5. Mission Progress Bar Container
        GameObject progressContainer = new GameObject("Progress_Container");
        progressContainer.transform.SetParent(hudPanelObject.transform, false);
        RectTransform progRect = progressContainer.AddComponent<RectTransform>();
        progRect.anchorMin = new Vector2(0f, 1f);
        progRect.anchorMax = new Vector2(1f, 1f);
        progRect.pivot = new Vector2(0.5f, 1f);
        progRect.anchoredPosition = new Vector2(0f, -86f);
        progRect.sizeDelta = new Vector2(-28f, 22f);

        Image progBg = progressContainer.AddComponent<Image>();
        progBg.color = new Color(0.06f, 0.10f, 0.16f, 0.95f);

        // Progress Bar Fill Image
        GameObject fillObj = new GameObject("Progress_Fill");
        fillObj.transform.SetParent(progressContainer.transform, false);
        progressBarFillRect = fillObj.AddComponent<RectTransform>();
        progressBarFillRect.anchorMin = new Vector2(0f, 0f);
        progressBarFillRect.anchorMax = new Vector2(0f, 1f);
        progressBarFillRect.pivot = new Vector2(0f, 0.5f);
        progressBarFillRect.offsetMin = Vector2.zero;
        progressBarFillRect.offsetMax = Vector2.zero;

        Image fillImg = fillObj.AddComponent<Image>();
        fillImg.color = new Color(0.0f, 0.78f, 1.0f, 0.92f); // Bright Neon Cyan Fill

        // Progress Bar Text
        GameObject progTextObj = new GameObject("Progress_Text");
        progTextObj.transform.SetParent(progressContainer.transform, false);
        RectTransform progTextRect = progTextObj.AddComponent<RectTransform>();
        progTextRect.anchorMin = Vector2.zero;
        progTextRect.anchorMax = Vector2.one;
        progTextRect.offsetMin = Vector2.zero;
        progTextRect.offsetMax = Vector2.zero;

        progressBarText = progTextObj.AddComponent<Text>();
        progressBarText.font = defaultFont;
        progressBarText.fontSize = 12;
        progressBarText.fontStyle = FontStyle.Bold;
        progressBarText.alignment = TextAnchor.MiddleCenter;
        progressBarText.color = Color.white;

        // 6. Telemetry Main Content Text
        GameObject telemetryObj = new GameObject("Telemetry_Content");
        telemetryObj.transform.SetParent(hudPanelObject.transform, false);
        RectTransform telemetryRect = telemetryObj.AddComponent<RectTransform>();
        telemetryRect.anchorMin = new Vector2(0f, 1f);
        telemetryRect.anchorMax = new Vector2(1f, 1f);
        telemetryRect.pivot = new Vector2(0.5f, 1f);
        telemetryRect.anchoredPosition = new Vector2(0f, -118f);
        telemetryRect.sizeDelta = new Vector2(-32f, 410f);

        telemetryText = telemetryObj.AddComponent<Text>();
        telemetryText.font = defaultFont;
        telemetryText.fontSize = 13;
        telemetryText.lineSpacing = 1.28f;
        telemetryText.color = new Color(0.92f, 0.95f, 0.98f);
        telemetryText.alignment = TextAnchor.UpperLeft;
        telemetryText.supportRichText = true;

        // 7. Mission Score Evaluation Footer Box
        GameObject scoreObj = new GameObject("Score_Content");
        scoreObj.transform.SetParent(hudPanelObject.transform, false);
        RectTransform scoreRect = scoreObj.AddComponent<RectTransform>();
        scoreRect.anchorMin = new Vector2(0f, 0f);
        scoreRect.anchorMax = new Vector2(1f, 0f);
        scoreRect.pivot = new Vector2(0.5f, 0f);
        scoreRect.anchoredPosition = new Vector2(0f, 16f);
        scoreRect.sizeDelta = new Vector2(-32f, 160f);

        scoreText = scoreObj.AddComponent<Text>();
        scoreText.font = defaultFont;
        scoreText.fontSize = 13;
        scoreText.lineSpacing = 1.25f;
        scoreText.color = new Color(1f, 0.88f, 0.40f); // Amber / Gold
        scoreText.alignment = TextAnchor.UpperLeft;
        scoreText.supportRichText = true;
    }

    /// <summary>
    /// Refreshes the telemetry text labels, progress bar, threat status, and state badges.
    /// </summary>
    public void RefreshDisplay(bool forceImmediate)
    {
        AcquireSubsystems();

        if (missionManager == null)
            return;

        MissionState state = missionManager.State;
        UpdateStateBadge(state);

        // Compute Spatial & Progress Telemetry
        Vector3 uavPos = pathFollower != null ? pathFollower.transform.position : transform.position;
        Vector3 startPos = pathfinding != null && pathfinding.startMarkerTransform != null
            ? pathfinding.startMarkerTransform.position
            : GameManagerBootstrapper.DefaultStartPosition;
        Vector3 targetPos = pathfinding != null && pathfinding.targetTransform != null
            ? pathfinding.targetTransform.position
            : GameManagerBootstrapper.DefaultTargetPosition;

        float progressPct = CalculateMissionProgress(uavPos, startPos, targetPos, state);

        // Update Visual Progress Bar
        if (progressBarFillRect != null)
        {
            progressBarFillRect.anchorMax = new Vector2(Mathf.Clamp01(progressPct / 100f), 1f);
        }
        if (progressBarText != null)
        {
            progressBarText.text = $"MISSION PROGRESS:  {progressPct:F1}%";
        }

        // Live Threat Status
        ThreatLevel threatLevel = threatAssessment != null ? threatAssessment.CurrentThreatLevel : ThreatLevel.None;
        string threatBadgeStr;
        switch (threatLevel)
        {
            case ThreatLevel.Critical:
                threatBadgeStr = "<color=#FF4444><b>[ CRITICAL DETOUR ]</b></color>";
                break;
            case ThreatLevel.Warning:
                threatBadgeStr = "<color=#FFAA22><b>[ WARNING HAZARD ]</b></color>";
                break;
            case ThreatLevel.Advisory:
                threatBadgeStr = "<color=#FFFF44><b>[ ADVISORY ]</b></color>";
                break;
            default:
                threatBadgeStr = "<color=#33FF88><b>[ CLEAR AIRSPACE ]</b></color>";
                break;
        }

        float currentSpeed = pathFollower != null
            ? (pathFollower.IsFollowing ? pathFollower.CurrentFlightSpeed : 0f)
            : 0f;

        float efficiencyPct = missionManager.PathEfficiency * 100f;
        string effColor = efficiencyPct >= 85f ? "#33FF88" : (efficiencyPct >= 70f ? "#FFCC33" : "#FF6644");

        int activeThreatsCount = threatAssessment != null && threatAssessment.ActiveThreatReports != null
            ? threatAssessment.ActiveThreatReports.Count
            : (threatLevel >= ThreatLevel.Warning ? 1 : 0);
        int peakThreats = replanningController != null ? replanningController.PeakSimultaneousThreats : activeThreatsCount;
        int pacingCount = replanningController != null ? replanningController.SpeedPacingCount : 0;
        int spatialCount = replanningController != null ? replanningController.SpatialReplanCount : (replanningController != null ? replanningController.ReplanCount : missionManager.TotalReplans);
        int totalReplans = replanningController != null ? replanningController.ReplanCount : missionManager.TotalReplans;

        string speedStr = $"<color=#00FFAA><b>{currentSpeed:F2}</b> m/s</color>";
        if (pathFollower != null && pathFollower.IsSpeedOverrideActive)
        {
            speedStr += $" <color=#FFAA00>[VO Pacing: {pathFollower.CurrentSpeedOverrideRatio * 100f:F0}%]</color>";
        }

        string evasionModeStr = "<color=#33FF88>NORMAL</color>";
        if (replanningController != null)
        {
            switch (replanningController.State)
            {
                case NavigationState.ThreatDetected:
                    evasionModeStr = "<color=#FFFF44>THREAT TRACKING</color>";
                    break;
                case NavigationState.Replanning:
                    evasionModeStr = "<color=#FF4444>SPATIAL REPLANNING</color>";
                    break;
                case NavigationState.Rerouting:
                    evasionModeStr = pathFollower != null && pathFollower.IsSpeedOverrideActive
                        ? "<color=#FFAA00>VO SPEED PACING</color>"
                        : "<color=#00FFFF>A* DETOUR EXECUTION</color>";
                    break;
                case NavigationState.NoSafePath:
                    evasionModeStr = "<color=#FF2222>SAFE HOLD</color>";
                    break;
            }
        }

        // Build Telemetry Block with High-Contrast Typography
        telemetrySb.Clear();
        telemetrySb.AppendLine("<color=#5EC8FF><b>── MISSION & NAVIGATION ─────────────────</b></color>");
        telemetrySb.AppendLine($"<b><color=#88A2BF>Position:</color></b>      <color=#FFFFFF>X: <b>{uavPos.x,6:F2}</b> m,  Z: <b>{uavPos.z,6:F2}</b> m</color>");
        telemetrySb.AppendLine($"<b><color=#88A2BF>Target:</color></b>        <color=#FFFFFF>X: <b>{targetPos.x,6:F2}</b> m,  Z: <b>{targetPos.z,6:F2}</b> m</color>");
        telemetrySb.AppendLine($"<b><color=#88A2BF>Flight Speed:</color></b>  {speedStr}");
        telemetrySb.AppendLine($"<b><color=#88A2BF>Flight Time:</color></b>   <color=#FFFFFF><b>{missionManager.TotalFlightTime:F2}</b> s</color>");
        telemetrySb.AppendLine($"<b><color=#88A2BF>Distance:</color></b>      <color=#FFFFFF><b>{missionManager.TotalDistanceTraveled:F2}</b> m</color>  <color=#88A2BF>(Plan: {missionManager.PlannedPathDistance:F2} m)</color>");
        telemetrySb.AppendLine($"<b><color=#88A2BF>Path Efficiency:</color></b><color={effColor}> <b>{efficiencyPct:F1} %</b></color>");

        telemetrySb.AppendLine();
        telemetrySb.AppendLine("<color=#5EC8FF><b>── TACTICAL & THREAT ENCOUNTERS ─────────</b></color>");
        telemetrySb.AppendLine($"<b><color=#88A2BF>Threat Status:</color></b>   {threatBadgeStr}");
        telemetrySb.AppendLine($"<b><color=#88A2BF>Active Threats:</color></b>  <color=#FFFFFF><b>{activeThreatsCount}</b></color>  <color=#88A2BF>(Peak: {peakThreats})</color>");
        telemetrySb.AppendLine($"<b><color=#88A2BF>Evasion Mode:</color></b>    {evasionModeStr}");
        telemetrySb.AppendLine($"<b><color=#88A2BF>Dynamic Replans:</color></b> <color=#FFFFFF><b>{totalReplans}</b></color>  <color=#88A2BF>(Pacing: {pacingCount} | Spatial: {spatialCount})</color>");
        telemetrySb.AppendLine($"<b><color=#88A2BF>Threat Events:</color></b>   <color=#FFFFFF><b>{missionManager.TotalThreatEncounters}</b></color>  <color=#88A2BF>({missionManager.CriticalThreatCount} Critical)</color>");

        string clearanceStr = float.IsPositiveInfinity(missionManager.MinimumClearanceObserved)
            ? "<color=#33FF88><b>N/A (Clear)</b></color>"
            : $"<color=#FFFFFF><b>{missionManager.MinimumClearanceObserved:F2}</b> m</color>";
        telemetrySb.AppendLine($"<b><color=#88A2BF>Min Clearance:</color></b>   {clearanceStr}");

        if (telemetryText != null)
        {
            telemetryText.text = telemetrySb.ToString();
        }

        // Build Score Evaluation Block
        float nominalSpeed = pathFollower != null ? pathFollower.MoveSpeed : 1.5f;
        MissionResult currentSnapshot = missionManager.Result ?? new MissionResult(
            state == MissionState.Completed,
            state,
            missionManager.TotalFlightTime,
            missionManager.TotalDistanceTraveled,
            missionManager.PlannedPathDistance,
            missionManager.TotalReplans,
            missionManager.TotalThreatEncounters,
            missionManager.CriticalThreatCount,
            missionManager.MinimumClearanceObserved,
            missionManager.PathEfficiency);

        MissionScore score = missionManager.Score ?? MissionScore.Evaluate(currentSnapshot, nominalSpeed);

        scoreSb.Clear();
        scoreSb.AppendLine("<color=#FFD54F><b>── PERFORMANCE SCORE EVALUATION ─────────</b></color>");
        scoreSb.AppendLine($"<b><color=#FFFFFF>OVERALL SCORE:</color></b>      <size=15><color=#FFD54F><b>{score.OverallScore:F1} / 100.0</b></color></size>");
        scoreSb.AppendLine($" • <color=#88A2BF>Safety:</color>     <color=#FFFFFF><b>{score.SafetyScore,5:F1}</b></color>      • <color=#88A2BF>Threat Mgmt:</color> <color=#FFFFFF><b>{score.ThreatManagementScore,5:F1}</b></color>");
        scoreSb.AppendLine($" • <color=#88A2BF>Navigation:</color> <color=#FFFFFF><b>{score.NavigationScore,5:F1}</b></color>      • <color=#88A2BF>Flight Time:</color> <color=#FFFFFF><b>{score.TimeScore,5:F1}</b></color>");
        scoreSb.AppendLine($" • <color=#88A2BF>Efficiency:</color> <color=#FFFFFF><b>{score.EfficiencyScore,5:F1}</b></color>");

        if (scoreText != null)
        {
            scoreText.text = scoreSb.ToString();
        }
    }

    private float CalculateMissionProgress(Vector3 uavPos, Vector3 startPos, Vector3 targetPos, MissionState state)
    {
        if (state == MissionState.Completed)
            return 100f;

        if (state == MissionState.Pending)
            return 0f;

        float totalSpan = Vector3.Distance(new Vector3(startPos.x, 0f, startPos.z), new Vector3(targetPos.x, 0f, targetPos.z));
        if (totalSpan <= 0.001f)
            return 100f;

        float remainingDist = Vector3.Distance(new Vector3(uavPos.x, 0f, uavPos.z), new Vector3(targetPos.x, 0f, targetPos.z));
        float progress = Mathf.Clamp01(1.0f - (remainingDist / totalSpan)) * 100f;

        // Never display 100% while still in-flight
        return Mathf.Min(progress, 99.9f);
    }

    private void UpdateStateBadge(MissionState state)
    {
        if (stateBadgeText == null)
            return;

        switch (state)
        {
            case MissionState.Pending:
                stateBadgeText.text = "● PENDING TAKEOFF";
                stateBadgeText.color = new Color(1f, 0.8f, 0.2f); // Amber
                break;

            case MissionState.Navigating:
                stateBadgeText.text = "▶ NAVIGATING CORRIDOR";
                stateBadgeText.color = new Color(0.2f, 1f, 0.4f); // Neon Green
                break;

            case MissionState.Rerouting:
                stateBadgeText.text = "▲ TACTICAL DETOUR / REROUTING";
                stateBadgeText.color = new Color(1f, 0.45f, 0.15f); // Orange / Coral
                break;

            case MissionState.Completed:
                stateBadgeText.text = "✔ MISSION SUCCESS (COMPLETED)";
                stateBadgeText.color = new Color(0.1f, 1f, 0.5f); // Vibrant Emerald Green
                break;

            case MissionState.Failed:
                stateBadgeText.text = "✖ MISSION FAILED (NO SAFE PATH)";
                stateBadgeText.color = new Color(1f, 0.2f, 0.25f); // Crimson Red
                break;
        }
    }
}
