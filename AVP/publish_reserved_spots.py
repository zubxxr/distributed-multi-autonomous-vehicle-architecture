import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ReservedSpotPublisher(Node):
    def __init__(self):
        super().__init__('parking_spot_publisher')
        self.publisher_ = self.create_publisher(String, '/parking_spots/reserved', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_reserved_spots)

    def publish_reserved_spots(self):
        msg = String()
        msg.data = '2025-05-04 23:27:41: 17'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = ReservedSpotPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
