using UnityEngine;

public class Node : IHeapItem<Node>
{
    // Bu nokta uculabilir mi? (Uzerinde dag/radar var mi?)
    public bool isWalkable; 
    
    // Bu noktanin Unity'nin 3D dunyasindaki gercek (X, Y, Z) koordinati nedir?
    public Vector3 worldPosition; 

    // Grid (Matris) dizisindeki satir (X) ve sutun (Y) indexleri
    public int gridX;
    public int gridY;

    public int gCost;
    public int hCost;
    public int fCost => gCost + hCost;
    public Node parent;

    // Airspace Clearance Potential Field parameters
    public float clearanceDistance; // Approximate distance to nearest obstacle in meters
    public int clearancePenalty;    // Additive cost penalty based on obstacle proximity

    private int heapIndex = -1;
    public int HeapIndex
    {
        get => heapIndex;
        set => heapIndex = value;
    }

    // Node sinifinin yapici (Constructor) metodu
    public Node(bool _isWalkable, Vector3 _worldPos, int _gridX, int _gridY)
    {
        isWalkable = _isWalkable;
        worldPosition = _worldPos;
        gridX = _gridX;
        gridY = _gridY;
    }

    /// <summary>
    /// Min-Heap comparator preserving exact A* ordering:
    /// Lowest fCost has highest priority; ties broken by lowest hCost.
    /// </summary>
    public int CompareTo(Node nodeToCompare)
    {
        int compare = fCost.CompareTo(nodeToCompare.fCost);
        if (compare == 0)
        {
            compare = hCost.CompareTo(nodeToCompare.hCost);
        }
        return -compare;
    }
}
