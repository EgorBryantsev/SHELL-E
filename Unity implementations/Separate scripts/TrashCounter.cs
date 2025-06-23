using UnityEngine;
using System.Collections.Generic;

public class TrashCounter : MonoBehaviour
{
    [Tooltip("Range (in Unity units) within which trash counts toward a box.")]
    public float range = 0.5f;

    void Update()
    {
        // Example: press 'C' to compute and log counts
        if (Input.GetKeyDown(KeyCode.C))
        {
            if (DetectionManager.Instance == null)
            {
                Debug.LogWarning("DetectionManager instance not found.");
                return;
            }
            Dictionary<GameObject, int> results = DetectionManager.Instance.ComputeTrashCounts(range);
            foreach (var kv in results)
            {
                var box = kv.Key;
                int count = kv.Value;
                if (box != null)
                    Debug.Log($"Box '{box.name}' has {count} trash objects within {range} units.");
            }
        }
    }
}

