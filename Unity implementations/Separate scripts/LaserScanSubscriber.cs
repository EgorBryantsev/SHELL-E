using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Sensor;

public class LaserScanSubscriber : MonoBehaviour
{
    ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<LaserScanMsg>("/scan", OnScanReceived);
    }

    void OnScanReceived(LaserScanMsg msg)
    {
        float minDistance = Mathf.Min(msg.ranges);
        Debug.Log($"[LIDAR] Min distance: {minDistance:F2} m");
    }
}
