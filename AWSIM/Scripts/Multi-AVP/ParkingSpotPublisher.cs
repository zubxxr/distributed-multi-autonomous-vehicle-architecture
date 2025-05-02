using System;
using System.Collections.Generic;
using UnityEngine;
using ROS2;

namespace AWSIM
{
    /// <summary>
    /// Publish the list of empty parking spots to multiple ROS2 topics as a comma-separated string.
    /// </summary>
    public class ParkingSpotRos2Publisher2 : MonoBehaviour
    {
        [Tooltip("List of ROS 2 topic names to publish to.")]
        public List<string> topicNames = new List<string> { "/parking_spots/empty" };

        public QoSSettings qosSettings = new QoSSettings()
        {
            ReliabilityPolicy = ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE,
            DurabilityPolicy = DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
            HistoryPolicy = HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
            Depth = 10,
        };

        private List<IPublisher<std_msgs.msg.String>> _publishers = new List<IPublisher<std_msgs.msg.String>>();
        private YoloIntegration _yoloIntegration;

        private void Awake()
        {
            CreatePublishers();
        }

        private void Start()
        {
            ConnectYoloIntegration();
        }

        private void CreatePublishers()
        {
            var qos = qosSettings.GetQoSProfile();

            foreach (var topic in topicNames)
            {
                var pub = SimulatorROS2Node.CreatePublisher<std_msgs.msg.String>(topic, qos);
                _publishers.Add(pub);
                Debug.Log($"Created publisher for topic: {topic}");
            }
        }

        private void ConnectYoloIntegration()
        {
            _yoloIntegration = GetComponent<YoloIntegration>();
            if (_yoloIntegration == null)
            {
                Debug.LogError("YoloIntegration component not found. Please add it to the GameObject.");
                return;
            }

            _yoloIntegration.OnParkingSpotsUpdated += Publish;
        }

        private void Publish(string emptySpots)
        {
            string timestamp = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss");
            string messageData = $"{timestamp}: {emptySpots}";

            var msg = new std_msgs.msg.String { Data = messageData };

            foreach (var pub in _publishers)
            {
                pub.Publish(msg);
            }
        }

        private void OnDestroy()
        {
            if (_yoloIntegration != null)
                _yoloIntegration.OnParkingSpotsUpdated -= Publish;

            foreach (var pub in _publishers)
            {
                SimulatorROS2Node.RemovePublisher<std_msgs.msg.String>(pub);
            }

            _publishers.Clear();
            GC.Collect();
        }
    }
}
