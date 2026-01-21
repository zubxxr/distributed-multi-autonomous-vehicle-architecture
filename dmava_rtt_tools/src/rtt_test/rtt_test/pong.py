import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

class PongNode(Node):
    def __init__(self):
        super().__init__('pong_node')

        self.declare_parameter('namespace', '')
        ns = self.get_parameter('namespace').get_parameter_value().string_value

        ping_topic = f'/{ns}/rtt_ping' if ns else '/rtt_ping'
        pong_topic = f'/{ns}/rtt_pong' if ns else '/rtt_pong'

        self.subscriber = self.create_subscription(
            Header, ping_topic, self.ping_callback, 10
        )
        self.publisher = self.create_publisher(Header, pong_topic, 10)

        self.get_logger().info(f'RTT Pong using topics:')
        self.get_logger().info(f'  {ping_topic}')
        self.get_logger().info(f'  {pong_topic}')

    def ping_callback(self, msg):
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = PongNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

