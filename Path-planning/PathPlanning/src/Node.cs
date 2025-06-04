public class Node
{
    public bool Walkable;
    public int GridX, GridY;
    public int GCost, HCost;
    public float Cost = 1;
    public Node Parent;
    public Vector3 WorldPosition;

    public int FCost => GCost + HCost;

    public Node(bool walkable, int gridX, int gridY, Vector3 worldPosition)
    {
        Walkable = walkable;
        WorldPosition = worldPosition;
        GridX = gridX;
        GridY = gridY;
    }
}
