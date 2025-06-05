using System;

public class Grid : MonoBehaviour
{
    // Serialized fields for Unity Inspector (showing a private variable's value)
    [SerializeField] private LayerMask obstacleMask;
    [SerializeField] private Vector2 gridWorldSize = new Vector2(50, 50);
    [SerializeField] private float nodeRadius = 1f;

    private Node[,] nodes;
    private float nodeDiameter;
    private int gridSizeX, gridSizeY;

    void Awake()
    {
        nodeDiameter = nodeRadius * 2;
        gridSizeX = Mathf.RoundToInt(gridWorldSize.x / nodeDiameter);
        gridSizeY = Mathf.RoundToInt(gridWorldSize.y / nodeDiameter);
        CreateGrid();
    }

    // Core grid creation method
    public void CreateGrid()
    {
        nodes = new Node[gridSizeX, gridSizeY];
        Vector3 worldBottomLeft = transform.position -
            Vector3.right * gridWorldSize.x / 2 -
            Vector3.forward * gridWorldSize.y / 2;

        for (int x = 0; x < gridSizeX; x++)
        {
            for (int y = 0; y < gridSizeY; y++)
            {
                Vector3 worldPoint = worldBottomLeft +
                    Vector3.right * (x * nodeDiameter + nodeRadius) +
                    Vector3.forward * (y * nodeDiameter + nodeRadius);

                bool walkable = !Physics.CheckSphere(worldPoint, nodeRadius, obstacleMask);
                nodes[x, y] = new Node(walkable, worldPoint, x, y);
            }
        }
    }

    // Critical conversion method
    public Node NodeFromWorldPoint(Vector3 worldPosition)
    {
        float percentX = (worldPosition.x + gridWorldSize.x / 2) / gridWorldSize.x;
        float percentY = (worldPosition.z + gridWorldSize.y / 2) / gridWorldSize.y;

        percentX = Mathf.Clamp01(percentX);
        percentY = Mathf.Clamp01(percentY);

        int x = Mathf.FloorToInt(percentX * (gridSizeX - 1));
        int y = Mathf.FloorToInt(percentY * (gridSizeY - 1));

        return nodes[x, y];
    }

    // Costmap integration method
    public void ApplyCostmap(float[,] costs)
    {
        for (int x = 0; x < gridSizeX; x++)
        {
            for (int y = 0; y < gridSizeY; y++)
            {
                if (nodes[x, y] != null)
                {
                    nodes[x, y].Cost = costs[x, y];
                }
            }
        }
    }

    // Visual debugging
    public void OnDrawGizmos()
    {
        Gizmos.DrawWireCube(transform.position, new Vector3(gridWorldSize.x, 1, gridWorldSize.y));
        if (nodes == null) return;

        foreach (Node node in nodes)
        {
            Gizmos.color = node.Walkable ? Color.white : Color.red;
            Gizmos.DrawCube(node.WorldPosition, Vector3.one * (nodeDiameter - 0.1f));
        }
    }
}