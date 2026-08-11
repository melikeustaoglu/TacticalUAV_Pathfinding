using UnityEngine;

public class Node
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

    // Node sinifinin yapici (Constructor) metodu
    public Node(bool _isWalkable, Vector3 _worldPos, int _gridX, int _gridY)
    {
        isWalkable = _isWalkable;
        worldPosition = _worldPos;
        gridX = _gridX;
        gridY = _gridY;
    }
}
