import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
import time
import csv


class PingNode(Node):
    def __init__(self):
        super().__init__('ping_node')

        # Parameters
        self.declare_parameter('namespace', '')
        self.declare_parameter('output_file', 'rtt_log.csv')

        ns = self.get_parameter('namespace').get_parameter_value().string_value
        output_file = self.get_parameter('output_file').get_parameter_value().string_value

        # CSV logging
        self.log_file = open(output_file, 'w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow(['timestamp', 'seq', 'rtt_ms'])

        self.get_logger().info(f'Logging RTT data to: {output_file}')

        # Topic names
        ping_topic = f'/{ns}/rtt_ping' if ns else '/rtt_ping'
        pong_topic = f'/{ns}/rtt_pong' if ns else '/rtt_pong'

        self.publisher = self.create_publisher(Header, ping_topic, 10)
        self.subscriber = self.create_subscription(
            Header, pong_topic, self.pong_callback, 10
        )

        self.seq = 0
        self.sent_times = {}
        self.timer = self.create_timer(0.1, self.send_ping)

        self.get_logger().info('RTT Ping using topics:')
        self.get_logger().info(f'  {ping_topic}')
        self.get_logger().info(f'  {pong_topic}')

    def send_ping(self):
        msg = Header()
        msg.stamp = self.get_clock().now().to_msg()
        msg.frame_id = str(self.seq)

        self.sent_times[self.seq] = time.time()
        self.publisher.publish(msg)
        self.seq += 1

    def pong_callback(self, msg):
        seq = int(msg.frame_id)

        if seq in self.sent_times:
            rtt = (time.time() - self.sent_times[seq]) * 1000.0
            now = time.time()

            self.csv_writer.writerow([now, seq, rtt])
            self.log_file.flush()

            self.get_logger().info(f'RTT seq={seq} : {rtt:.2f} ms')

            del self.sent_times[seq]

    def destroy_node(self):
        self.log_file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = PingNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

