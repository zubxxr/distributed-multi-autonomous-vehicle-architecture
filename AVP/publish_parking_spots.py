import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime

class ParkingSpotPublisher(Node):
    def __init__(self):
        super().__init__('parking_spot_publisher')
        self.publisher_ = self.create_publisher(String, '/parking_spots/empty', 10)
        self.subscription = self.create_subscription(
            String,
            '/parking_spots/reserved',
            self.reserved_callback,
            10
        )

        self.timer = self.create_timer(1.0, self.publish_parking_spots)

        # Initial state
        self.empty_spots = {17, 18, 19, 28}
        self.reserved_spots = set()

    def reserved_callback(self, msg):
        try:
            spot_str = msg.data.split(':')[-1].strip()
            self.reserved_spots = {int(s) for s in spot_str.split(',') if s.strip().isdigit()}
        except Exception as e:
            self.get_logger().warn(f"Failed to parse reserved spots: {e}")

    def publish_parking_spots(self):
        available_spots = sorted(self.empty_spots - self.reserved_spots)
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        msg = String()
        msg.data = f"{timestamp}: {','.join(str(s) for s in available_spots)}"
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = ParkingSpotPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
