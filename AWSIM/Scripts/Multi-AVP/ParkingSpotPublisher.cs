using System;
using UnityEngine;
using ROS2;

namespace AWSIM
{
    /// <summary>
    /// Publish the list of empty parking spots to a ROS2 topic as a comma-separated string.
    /// </summary>
    public class ParkingSpotRos2Publisher2 : MonoBehaviour
    {
        /// <summary>
        /// ROS 2 topic name for publishing empty parking spots.
        /// </summary>
        public string topicName = "/parking_spots/empty";

        /// <summary>
        /// QoS settings for the publisher.
        /// </summary>
        public QoSSettings qosSettings = new QoSSettings()
        {
            ReliabilityPolicy = ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE,
            DurabilityPolicy = DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
            HistoryPolicy = HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
            Depth = 10,
        };

        private IPublisher<std_msgs.msg.String> _publisher;

        private YoloIntegration _yoloIntegration;

        private void Awake()
        {
            CreatePublisher();
        }

        private void Start()
        {
            ConnectYoloIntegration();
        }

        private void CreatePublisher()
        {
            // Create the ROS2 publisher
            var qos = qosSettings.GetQoSProfile();
            _publisher = SimulatorROS2Node.CreatePublisher<std_msgs.msg.String>(topicName, qos);
        }

        private void ConnectYoloIntegration()
        {
            // Get the YoloIntegration component
            _yoloIntegration = GetComponent<YoloIntegration>();
	
            if (_yoloIntegration == null)
            {
                Debug.LogError("YoloIntegration component not found. Please add it to the GameObject.");
                return;
            }

            // Subscribe to parking spot updates
            _yoloIntegration.OnParkingSpotsUpdated += Publish;

        }

        private void Publish(string emptySpots)
        {   
            string timestamp = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss");

            string messageData = $"{timestamp}: {emptySpots}";


            // Prepare the message
            var message = new std_msgs.msg.String
            {
                Data = messageData // Pass the comma-separated string directly
            };
            
            // Publish the message
            _publisher.Publish(message);
        }

        private void OnDestroy()
        {
            if (_yoloIntegration != null)
                _yoloIntegration.OnParkingSpotsUpdated -= Publish;

            SimulatorROS2Node.RemovePublisher<std_msgs.msg.String>(_publisher);
            GC.Collect();
        }
    }
}

