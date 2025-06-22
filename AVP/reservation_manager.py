import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ReservationManager(Node):
    def __init__(self):
        super().__init__('reservation_manager')

        # Canonical reservation list
        self.reserved_spots = []

        # Subscriber for vehicle reservation requests
        self.create_subscription(String, '/parking_spots/reserved/request', self.request_callback, 10)

        # Subscriber for removal requests
        self.create_subscription(String, '/parking_spots/reserved/remove', self.remove_callback, 10)

        # Publisher for the master reservation list
        self.reserved_pub = self.create_publisher(String, '/parking_spots/reserved', 10)

        # Timer to republish reservation list every 2 seconds
        self.create_timer(2.0, self.publish_reserved_list)

    def request_callback(self, msg):
        try:
            spot = int(msg.data.strip())
            if spot not in self.reserved_spots:
                self.reserved_spots.append(spot)
                self.get_logger().info(f"✅ Reserved spot {spot}")
            else:
                self.get_logger().info(f"ℹ️ Spot {spot} already reserved")
        except ValueError:
            self.get_logger().warn(f"⚠️ Invalid spot request: '{msg.data}'")

    def remove_callback(self, msg):
        try:
            spot = int(msg.data.strip())
            if spot in self.reserved_spots:
                self.reserved_spots.remove(spot)
                self.get_logger().info(f"🗑️ Removed spot {spot}")
            else:
                self.get_logger().info(f"ℹ️ Spot {spot} was not in reservation list")
        except ValueError:
            self.get_logger().warn(f"⚠️ Invalid spot removal: '{msg.data}'")

    def publish_reserved_list(self):
        msg = String()
        msg.data = f"Reserved Spots: {self.reserved_spots}"
        self.reserved_pub.publish(msg)
        self.get_logger().info(f"🛰 Published: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = ReservationManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

