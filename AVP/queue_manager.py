import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from shapely.geometry import Point, Polygon

DROP_OFF_ZONE_POLYGON = [
    (-57.1, -28.6),
    (-55.2, -35.8),
    (-70.7, -31.9),
    (-68.3, -39.2)
]

class QueueManager(Node):
    def __init__(self):
        super().__init__('queue_manager')

        # Setup drop-off zone
        self.dropoff_polygon = Polygon(DROP_OFF_ZONE_POLYGON)

        # Track vehicle queue
        self.queue = []

        # ROS interfaces
        self.request_sub = self.create_subscription(
            String,
            '/queue_manager/request',
            self.queue_request_callback,
            10
        )

        self.queue_pub = self.create_publisher(String, '/avp/dropoff_queue', 10)

        # Periodic publishing
        self.timer = self.create_timer(1.0, self.publish_queue)

        self.get_logger().info("✅ Queue Manager Initialized")

    def is_in_dropoff_zone(self, x, y):
        return self.dropoff_polygon.contains(Point(x, y))

    def queue_request_callback(self, msg):
        try:
            vehicle_id = msg.data.strip()
            if not vehicle_id:
                self.get_logger().warn("⚠️ Received empty vehicle ID.")
                return

            if vehicle_id not in self.queue:
                self.queue.append(vehicle_id)
                self.get_logger().info(f"🚗 Added vehicle '{vehicle_id}' to queue.")
            else:
                self.get_logger().info(f"ℹ️ Vehicle '{vehicle_id}' is already in the queue.")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to process request: {e}")

    def publish_queue(self):
        msg = String()
        msg.data = '[' + ', '.join(self.queue) + ']'
        self.queue_pub.publish(msg)
        self.get_logger().debug(f"📤 Published queue: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = QueueManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
