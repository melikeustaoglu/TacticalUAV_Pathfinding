using System;
using System.Text;
using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// Lightweight Tactical Telemetry HUD overlay using Unity uGUI.
/// Observes MissionManager and MissionScore to render real-time UAV mission state,
/// flight telemetry, threat counters, clearance metrics, and evaluation scores.
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

    // uGUI Hierarchy Elements
    private Canvas hudCanvas;
    private GameObject hudPanelObject;
    private Image backgroundPanel;
    private Image headerBar;
    private Text headerText;
    private Text stateBadgeText;
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
        missionManager = GetComponent<MissionManager>() ?? FindFirstObjectByType<MissionManager>();
        pathFollower = GetComponent<PathFollower>() ?? FindFirstObjectByType<PathFollower>();
        threatAssessment = GetComponent<ThreatAssessment>() ?? FindFirstObjectByType<ThreatAssessment>();
        replanningController = GetComponent<ReplanningController>() ?? FindFirstObjectByType<ReplanningController>();
    }

    private void Start()
    {
        CreateUGUIHierarchy();
        RefreshDisplay(true);
    }

    private void OnEnable()
    {
        if (missionManager == null)
        {
            missionManager = GetComponent<MissionManager>() ?? FindFirstObjectByType<MissionManager>();
        }

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
    /// Builds the tactical uGUI Canvas and formatted Text hierarchy programmatically.
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
        panelRect.anchoredPosition = new Vector2(24f, -24f);
        panelRect.sizeDelta = new Vector2(400f, 540f);

        backgroundPanel = hudPanelObject.AddComponent<Image>();
        backgroundPanel.color = new Color(0.06f, 0.09f, 0.14f, 0.88f); // Tactical Dark Slate

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
        headerRect.sizeDelta = new Vector2(0f, 40f);

        headerBar = headerObj.AddComponent<Image>();
        headerBar.color = new Color(0.12f, 0.22f, 0.35f, 0.95f);

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
        headerText.fontSize = 16;
        headerText.fontStyle = FontStyle.Bold;
        headerText.color = new Color(0.35f, 0.85f, 1f); // Neon Cyan
        headerText.alignment = TextAnchor.MiddleLeft;
        headerText.text = "TACTICAL UAV MISSION TELEMETRY";

        // 4. Mission State Badge Banner
        GameObject badgeObj = new GameObject("State_Badge");
        badgeObj.transform.SetParent(hudPanelObject.transform, false);
        RectTransform badgeRect = badgeObj.AddComponent<RectTransform>();
        badgeRect.anchorMin = new Vector2(0f, 1f);
        badgeRect.anchorMax = new Vector2(1f, 1f);
        badgeRect.pivot = new Vector2(0.5f, 1f);
        badgeRect.anchoredPosition = new Vector2(0f, -44f);
        badgeRect.sizeDelta = new Vector2(-24f, 36f);

        stateBadgeText = badgeObj.AddComponent<Text>();
        stateBadgeText.font = defaultFont;
        stateBadgeText.fontSize = 15;
        stateBadgeText.fontStyle = FontStyle.Bold;
        stateBadgeText.alignment = TextAnchor.MiddleCenter;
        stateBadgeText.color = Color.white;

        // 5. Telemetry Main Content Text
        GameObject telemetryObj = new GameObject("Telemetry_Content");
        telemetryObj.transform.SetParent(hudPanelObject.transform, false);
        RectTransform telemetryRect = telemetryObj.AddComponent<RectTransform>();
        telemetryRect.anchorMin = new Vector2(0f, 1f);
        telemetryRect.anchorMax = new Vector2(1f, 1f);
        telemetryRect.pivot = new Vector2(0.5f, 1f);
        telemetryRect.anchoredPosition = new Vector2(0f, -86f);
        telemetryRect.sizeDelta = new Vector2(-32f, 260f);

        telemetryText = telemetryObj.AddComponent<Text>();
        telemetryText.font = defaultFont;
        telemetryText.fontSize = 13;
        telemetryText.lineSpacing = 1.25f;
        telemetryText.color = new Color(0.9f, 0.93f, 0.96f);
        telemetryText.alignment = TextAnchor.UpperLeft;

        // 6. Mission Score Evaluation Footer Box
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
        scoreText.lineSpacing = 1.2f;
        scoreText.color = new Color(1f, 0.85f, 0.35f); // Amber / Gold
        scoreText.alignment = TextAnchor.UpperLeft;
    }

    /// <summary>
    /// Refreshes the telemetry text labels and state badges.
    /// </summary>
    public void RefreshDisplay(bool forceImmediate)
    {
        if (missionManager == null)
        {
            missionManager = GetComponent<MissionManager>() ?? FindFirstObjectByType<MissionManager>();
            if (missionManager == null)
                return;
        }

        MissionState state = missionManager.State;
        UpdateStateBadge(state);

        // Build Telemetry Block
        telemetrySb.Clear();
        telemetrySb.AppendLine("── FLIGHT TELEMETRY ───────────────");
        telemetrySb.AppendLine($"Flight Time:        {missionManager.TotalFlightTime:F2} s");
        telemetrySb.AppendLine($"Actual Distance:    {missionManager.TotalDistanceTraveled:F2} m");
        telemetrySb.AppendLine($"Planned Distance:   {missionManager.PlannedPathDistance:F2} m");

        float efficiencyPct = missionManager.PathEfficiency * 100f;
        telemetrySb.AppendLine($"Path Efficiency:    {efficiencyPct:F1} %");

        telemetrySb.AppendLine();
        telemetrySb.AppendLine("── TACTICAL & THREAT ENCOUNTERS ───");
        telemetrySb.AppendLine($"Dynamic Replans:    {missionManager.TotalReplans}");
        telemetrySb.AppendLine($"Threat Encounters:  {missionManager.TotalThreatEncounters}");
        telemetrySb.AppendLine($"Critical Threats:   {missionManager.CriticalThreatCount}");

        string clearanceStr = float.IsPositiveInfinity(missionManager.MinimumClearanceObserved)
            ? "N/A (Clear)"
            : $"{missionManager.MinimumClearanceObserved:F2} m";
        telemetrySb.AppendLine($"Min Clearance:      {clearanceStr}");

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
        scoreSb.AppendLine("── MISSION SCORE EVALUATION ───────");
        scoreSb.AppendLine($"OVERALL SCORE:      <b>{score.OverallScore:F1} / 100</b>");
        scoreSb.AppendLine($" • Safety:          {score.SafetyScore:F1}");
        scoreSb.AppendLine($" • Navigation:      {score.NavigationScore:F1}");
        scoreSb.AppendLine($" • Efficiency:      {score.EfficiencyScore:F1}");
        scoreSb.AppendLine($" • Threat Mgmt:     {score.ThreatManagementScore:F1}");
        scoreSb.AppendLine($" • Flight Time:     {score.TimeScore:F1}");

        if (scoreText != null)
        {
            scoreText.text = scoreSb.ToString();
        }
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
