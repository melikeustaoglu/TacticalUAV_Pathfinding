using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

/// <summary>
/// Editor utility that automatically configures Assets/Scenes/Main.unity as the designated
/// Play Mode start scene. This guarantees that entering Play Mode always boots into the authoritative
/// Main scene (with its configured UAVScenarioConfig) without requiring manual scene opening.
/// </summary>
[InitializeOnLoad]
public static class ScenePlayModeSetup
{
    private const string MainScenePath = "Assets/Scenes/Main.unity";

    static ScenePlayModeSetup()
    {
        EditorApplication.delayCall += ConfigurePlayModeStartScene;
    }

    private static void ConfigurePlayModeStartScene()
    {
        SceneAsset mainScene = AssetDatabase.LoadAssetAtPath<SceneAsset>(MainScenePath);
        if (mainScene != null)
        {
            if (EditorSceneManager.playModeStartScene != mainScene)
            {
                EditorSceneManager.playModeStartScene = mainScene;
                Debug.Log($"[ScenePlayModeSetup] Successfully set Play Mode start scene to: {MainScenePath}");
            }
        }
        else
        {
            Debug.LogWarning($"[ScenePlayModeSetup] Could not locate SceneAsset at: {MainScenePath}");
        }
    }
}
