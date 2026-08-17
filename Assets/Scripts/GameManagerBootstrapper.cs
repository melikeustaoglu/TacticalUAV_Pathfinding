using UnityEngine;

public static class GameManagerBootstrapper
{
    private const string SystemObjectName = "PathfindingSystem";
    public static readonly Vector3 DefaultStartPosition = new Vector3(-10f, 1f, -10f);
    public static readonly Vector3 DefaultTargetPosition = new Vector3(10f, 1f, 10f);
    public static readonly Color UavColor = new Color(1f, 0.1f, 0.1f);

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
    private static void Bootstrap()
    {
        if (GameObject.Find(SystemObjectName) != null)
            return;

        GameObject systemObject = CreatePathfindingSystem();
        Transform startTransform = CreateStartTransform();
        Transform targetTransform = CreateTargetTransform();

        WireReferences(systemObject.GetComponent<Pathfinding>(), startTransform, targetTransform);
    }

    public static GameObject CreateUav(Vector3 spawnPosition)
    {
        GameObject uavObject = GameObject.CreatePrimitive(PrimitiveType.Cube);
        uavObject.name = "UAV";
        uavObject.transform.position = spawnPosition;
        uavObject.transform.localScale = Vector3.one * 1.2f;
        uavObject.layer = LayerMask.NameToLayer("Default");

        ApplyBrightMaterial(uavObject);
        uavObject.AddComponent<PathFollower>();
        uavObject.AddComponent<UAVPerception>();
        uavObject.AddComponent<ThreatAssessment>();
        uavObject.AddComponent<ReplanningController>();
        uavObject.AddComponent<MissionManager>();

        return uavObject;
    }

    private static void ApplyBrightMaterial(GameObject uavObject)
    {
        Renderer renderer = uavObject.GetComponent<Renderer>();
        if (renderer == null)
            return;

        Shader shader = Shader.Find("Unlit/Color");
        if (shader == null)
        {
            shader = Shader.Find("Standard");
        }

        Material uavMaterial = new Material(shader);
        uavMaterial.color = UavColor;
        renderer.material = uavMaterial;
    }

    private static GameObject CreatePathfindingSystem()
    {
        GameObject systemObject = new GameObject(SystemObjectName);

        GridManager gridManager = systemObject.AddComponent<GridManager>();
        gridManager.gridWorldSize = new Vector2(50f, 50f);
        gridManager.nodeRadius = 0.5f;
        gridManager.obstacleMask = ProceduralObstacleGenerator.GetObstacleMask();

        PathfindingRuntimeSetup runtimeSetup = systemObject.AddComponent<PathfindingRuntimeSetup>();
        runtimeSetup.startPosition = DefaultStartPosition;
        runtimeSetup.targetPosition = DefaultTargetPosition;

        systemObject.AddComponent<Pathfinding>();

        return systemObject;
    }

    private static Transform CreateStartTransform()
    {
        GameObject startObject = new GameObject("Start");
        startObject.transform.position = DefaultStartPosition;
        return startObject.transform;
    }

    private static Transform CreateTargetTransform()
    {
        GameObject targetObject = new GameObject("Target");
        targetObject.transform.position = DefaultTargetPosition;
        return targetObject.transform;
    }

    private static void WireReferences(Pathfinding pathfinding, Transform startTransform, Transform targetTransform)
    {
        pathfinding.startMarkerTransform = startTransform;
        pathfinding.targetTransform = targetTransform;
        pathfinding.agentSpawnPosition = DefaultStartPosition;
    }
}
