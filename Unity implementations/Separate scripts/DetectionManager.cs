using System.Collections.Generic;
using UnityEngine;

public class DetectionManager : MonoBehaviour
{
    public static DetectionManager Instance { get; private set; }

    private HashSet<GameObject> boxSet = new HashSet<GameObject>();
    private HashSet<GameObject> trashSet = new HashSet<GameObject>();

    private void Awake()
    {
        if (Instance != null && Instance != this)
        {
            Destroy(this);
            return;
        }
        Instance = this;
    }

    public void RegisterBox(GameObject box)
    {
        if (box != null)
            boxSet.Add(box);
    }

    public void RegisterTrash(GameObject trash)
    {
        if (trash != null)
            trashSet.Add(trash);
    }

    public Dictionary<GameObject, int> ComputeTrashCounts(float range)
    {
        var result = new Dictionary<GameObject, int>();
        var trashList = new List<GameObject>(trashSet);

        foreach (var box in boxSet)
        {
            if (box == null) continue;
            Vector3 boxPos = box.transform.position;
            int count = 0;
            foreach (var t in trashList)
            {
                if (t == null) continue;
                if (Vector3.Distance(boxPos, t.transform.position) <= range)
                    count++;
            }
            result[box] = count;
        }
        return result;
    }
}

