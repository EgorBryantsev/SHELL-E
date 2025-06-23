using UnityEngine;

[RequireComponent(typeof(Collider))]
public class ShapeDetector : MonoBehaviour
{
    [Header("Wall Heuristic Thresholds")]
    public float minWallWidth = 1.8f;
    public float minWallHeight = 1f;
    public float maxWallThickness = 0.05f;

    [Header("Normal Box Heuristic Thresholds (width/depth vs height)")]
    public float minBoxWidth = 0.4f;
    public float maxBoxWidth = 0.5f;
    public float minBoxHeight = 0.4f;
    public float maxBoxHeight = 1.0f;
    [Range(0f, 1f)]
    public float widthDepthTolerance = 0.1f;

    [Header("Trash Heuristic Thresholds (small width/depth vs height)")]
    public float minTrashWidth = 0.05f;
    public float maxTrashWidth = 0.15f;
    public float minTrashHeight = 0.7f;
    public float maxTrashHeight = 1.0f;
    [Range(0f, 1f)]
    public float trashWidthDepthTolerance = 0.1f;

    private void OnTriggerEnter(Collider other)
    {
        BoxCollider[] boxColliders = other.GetComponentsInChildren<BoxCollider>();
        foreach (BoxCollider bc in boxColliders)
        {
            Vector3 size = Vector3.Scale(bc.size, bc.transform.lossyScale);

            if (IsWallSize(size))
            {
                // Optionally register wall if needed; here we ignore
                return;
            }
            if (IsTrashSizeUpright(size))
            {
                HandleTrash(other.gameObject, size);
                return;
            }
            if (IsBoxSizeUpright(size))
            {
                HandleBox(other.gameObject, size);
                return;
            }
        }
        // If none matched, do nothing
    }

    private bool IsWallSize(Vector3 size)
    {
        float a = size.x, b = size.y, c = size.z;
        if (a <= maxWallThickness && b >= minWallWidth && c >= minWallHeight) return true;
        if (b <= maxWallThickness && a >= minWallWidth && c >= minWallHeight) return true;
        if (c <= maxWallThickness && a >= minWallWidth && b >= minWallHeight) return true;
        return false;
    }

    private bool IsBoxSizeUpright(Vector3 size)
    {
        float w = size.x, h = size.y, d = size.z;
        if (w < minBoxWidth || w > maxBoxWidth) return false;
        if (d < minBoxWidth || d > maxBoxWidth) return false;
        if (h < minBoxHeight || h > maxBoxHeight) return false;
        if (!ApproximatelyEqual(w, d, widthDepthTolerance)) return false;
        return true;
    }

    private bool IsTrashSizeUpright(Vector3 size)
    {
        float w = size.x, h = size.y, d = size.z;
        if (w < minTrashWidth || w > maxTrashWidth) return false;
        if (d < minTrashWidth || d > maxTrashWidth) return false;
        if (h < minTrashHeight || h > maxTrashHeight) return false;
        if (!ApproximatelyEqual(w, d, trashWidthDepthTolerance)) return false;
        return true;
    }

    private bool ApproximatelyEqual(float a, float b, float tolFraction)
    {
        float max = Mathf.Max(Mathf.Abs(a), Mathf.Abs(b));
        if (max < 1e-6f) return true;
        return Mathf.Abs(a - b) / max <= tolFraction;
    }

    private void HandleBox(GameObject obj, Vector3 size)
    {
        Debug.Log($"Detected BIN: {obj.name}");
        // Register in manager
        if (DetectionManager.Instance != null)
            DetectionManager.Instance.RegisterBox(obj);
        // Your other reactions...
    }

    private void HandleTrash(GameObject obj, Vector3 size)
    {
        Debug.Log($"Detected TRASH: {obj.name}");
        if (DetectionManager.Instance != null)
            DetectionManager.Instance.RegisterTrash(obj);
        // Your other reactions...
    }
}

