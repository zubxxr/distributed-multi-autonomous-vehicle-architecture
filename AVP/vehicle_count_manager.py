import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
import ast
import sys
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

if '--help' in sys.argv or '-h' in sys.argv:
    print("""
🚗 Vehicle Count Manager Help

This node listens for vehicle count requests and publishes vehicle counts per namespace.

✅ Example usage:
    ros2 run multi_avp vehicle_count_manager.py --ros-args -p namespaces:='["vehicle1", "vehicle2"]'

📌 Parameters:
    - namespaces: List of namespaces (e.g., ["vehicle1", "vehicle2"])
    - Use "main" as namespace to publish directly to root-level topics (e.g., /vehicle_count)

💡 Tip: You must bridge vehicle_count topics via Zenoh across hosts.
    """)
    sys.exit(0)

def get_topic(namespace, topic):
    return f"/{topic}" if namespace == "main" else f"/{namespace}/{topic}"
    
class VehicleCountManager(Node):
    def __init__(self):
        super().__init__('vehicle_count_manager')

        self.valid = False  # Prevent spin if setup fails

        # Declare parameter with correct descriptor
        self.declare_parameter(
            'namespaces',
            ['main'],  # ← default must be a list of strings
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)
        )

        # Retrieve and parse namespaces
        try:
            self.namespaces = self.get_parameter('namespaces').value
            if not isinstance(self.namespaces, list) or not all(isinstance(ns, str) for ns in self.namespaces):
                raise ValueError("Invalid type inside namespaces")
        except Exception as e:
            self.get_logger().error(f"❌ Invalid 'namespaces' parameter. Must be a list of strings. Error: {e}")
            print("\n✅ Example usage:")
            print("    ros2 run multi_avp vehicle_count_manager --ros-args -p namespaces:='[\"vehicle1\", \"vehicle2\"]'")
            print("💡 Tip: Run with --help to see more options.\n")
            return

        self.get_logger().info(f"✅ Managing vehicle_count for: {self.namespaces}")

        self.vehicle_counts = {}
        self.count_publishers = {}

        for ns in self.namespaces:
            self.vehicle_counts[ns] = 0
            count_topic = get_topic(ns, "vehicle_count")
            request_topic = get_topic(ns, "vehicle_count_request")

            self.count_publishers[ns] = self.create_publisher(Int32, count_topic, 10)
            self.create_subscription(String, request_topic, self.generate_callback(ns), 10)

        self.timer = self.create_timer(1.0, self.publish_all_counts)
        self.valid = True  # Node setup complete

        
    def generate_callback(self, triggering_namespace):
        def callback(msg):
            if msg.data == "add_me":
                self.vehicle_counts[triggering_namespace] += 1
                new_count = self.vehicle_counts[triggering_namespace]
                self.get_logger().info(f"[{triggering_namespace}] Vehicle joined → Count: {new_count}")

                # Forward this new count to ALL namespaces
                msg_out = Int32()
                msg_out.data = new_count
                for ns in self.namespaces:
                    self.count_publishers[ns].publish(msg_out)
                    self.get_logger().info(f"🔁 Forwarded count {new_count} to /{ns}/vehicle_count")
        return callback

    def publish_all_counts(self):
        for ns in self.namespaces:
            msg = Int32()
            msg.data = self.vehicle_counts[ns]
            self.count_publishers[ns].publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VehicleCountManager()

    if not getattr(node, 'valid', False):
        # No need to spin or shutdown again
        node.destroy_node()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
