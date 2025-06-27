using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class BinNavigator : MonoBehaviour
{

    [Tooltip("Minimum distance to consider 'arrived' at a bin.")]
    public float arriveThreshold = 0.5f;

    [Tooltip("If true, after visiting all bins, start over.")]
    public bool loop = false;

    private NavMeshAgent agent;
    private List<Transform> bins;  // remaining unvisited bins
    private Transform currentTarget;
    private bool finished = false;

    void Start()
    {
        agent = GetComponent<NavMeshAgent>();
        if (agent == null)
        {
            Debug.LogError("BinNavigator requires a NavMeshAgent on the same GameObject.");
            enabled = false;
            return;
        }
        
        if (!agent.isOnNavMesh)
        {
            Debug.LogError("NavMeshAgent is not on a NavMesh. Ensure the GameObject starts on a baked NavMesh.");
            enabled = false;
            return;
        }
	
	GameObject[] binObjects = GameObject.FindGameObjectsWithTag("Bin");
        bins = new List<Transform>();
        foreach (GameObject go in binObjects) bins.Add(go.transform);
        if (bins.Count == 0) Debug.LogWarning("No bins found.");
        PickNextTarget();
    }


    void Update()
    {   
        if(finished) return;
        if (!agent.pathPending && agent.remainingDistance <= arriveThreshold)
        {
            OnArrivedAtBin();
        }
    }


    private void OnArrivedAtBin()
    {
        // Remove visited bin from list
        if (bins.Contains(currentTarget))
            bins.Remove(currentTarget);

        if (bins.Count == 0)
        {
            Debug.Log("All bins visited.");
            finished=true;
        }
        else
        {
            PickNextTarget();
        }
    }

    private void PickNextTarget()
    {
        if (bins == null || bins.Count == 0)
        {
            currentTarget = null;
            return;
        }

        // Find nearest bin from current position
        Transform nearest = null;
        float minDistSqr = float.MaxValue;
        Vector3 pos = transform.position;
        foreach (Transform t in bins)
        {
            float d = (t.position - pos).sqrMagnitude;
            if (d < minDistSqr)
            {
                minDistSqr = d;
                nearest = t;
            }
        }

        if (nearest != null)
        {
            currentTarget = nearest;
            agent.SetDestination(currentTarget.position);
            Debug.Log($"Navigating to next bin at {currentTarget.position}");
        }
    }


}

