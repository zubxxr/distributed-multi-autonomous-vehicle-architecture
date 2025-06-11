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
        [Tooltip("List of ROS 2 topic names to publish empty spots to.")]
        public List<string> topicNames = new List<string> { "/parking_spots/empty" };

        [Tooltip("List of ROS 2 topic names to subscribe to for reserved spots.")]
        public List<string> reservedTopicNames = new List<string> { "/parking_spots/reserved" };

        public QoSSettings qosSettings = new QoSSettings()
        {
            ReliabilityPolicy = ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE,
            DurabilityPolicy = DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
            HistoryPolicy = HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
            Depth = 10,
        };

        private List<IPublisher<std_msgs.msg.String>> _publishers = new List<IPublisher<std_msgs.msg.String>>();
        private List<ISubscription<std_msgs.msg.String>> _reservedSubs = new List<ISubscription<std_msgs.msg.String>>();
        private HashSet<int> reservedSpotIds = new HashSet<int>();
        private YoloIntegration _yoloIntegration;

        private void Awake()
        {
            CreatePublishers();
        }

        private void Start()
        {
            ConnectYoloIntegration();
            CreateSubscribers();
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

        private void CreateSubscribers()
        {
            foreach (var reservedTopic in reservedTopicNames)
            {
                var sub = SimulatorROS2Node.CreateSubscription<std_msgs.msg.String>(
                    reservedTopic,
                    msg =>
                    {
                        foreach (var id in ParseReservedIds(msg.Data))
                            reservedSpotIds.Add(id);

                        Debug.Log("Updated reserved list from: " + reservedTopic);
                    });

                _reservedSubs.Add(sub);
                Debug.Log($"Created subscriber for topic: {reservedTopic}");
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

        private HashSet<int> ParseReservedIds(string data)
        {
            HashSet<int> result = new HashSet<int>();
            try
            {
                // Extract only what's after the final colon (to handle timestamps with colons)
                int lastColonIndex = data.LastIndexOf(':');
                if (lastColonIndex != -1 && lastColonIndex + 1 < data.Length)
                {
                    string idSection = data.Substring(lastColonIndex + 1).Trim();
                    string[] ids = idSection.Split(',');

                    foreach (var idStr in ids)
                    {
                        if (int.TryParse(idStr.Trim(), out int id))
                            result.Add(id);
                    }
                }
            }
            catch (Exception e)
            {
                Debug.LogError("Failed to parse reserved IDs: " + e.Message);
            }

            return result;
        }

        private void Publish(string emptySpots)
        {
            string timestamp = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss");

            List<string> filtered = new List<string>();
            foreach (var s in emptySpots.Split(','))
            {   
                if (int.TryParse(s.Trim(), out int id) && !reservedSpotIds.Contains(id))
                    filtered.Add(id.ToString());
            }

            string filteredSpots = string.Join(",", filtered);
            string messageData = $"{timestamp}: {filteredSpots}";

            var msg = new std_msgs.msg.String { Data = messageData };

            Debug.Log("Publishing filtered spots: " + filteredSpots + " | Reserved: " + string.Join(",", reservedSpotIds));

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
                SimulatorROS2Node.RemovePublisher<std_msgs.msg.String>(pub);

            foreach (var sub in _reservedSubs)
                SimulatorROS2Node.RemoveSubscription<std_msgs.msg.String>(sub);

            _publishers.Clear();
            _reservedSubs.Clear();
            GC.Collect();
        }
    }
}
