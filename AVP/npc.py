import rclpy
from rclpy.node import Node
from tier4_simulation_msgs.msg import DummyObject
from autoware_perception_msgs.msg import Shape, ObjectClassification
from geometry_msgs.msg import Pose, Quaternion, Vector3, Point
from unique_identifier_msgs.msg import UUID
from std_msgs.msg import Header, String
import uuid
import tf_transformations
import sys
import threading
import time
from datetime import datetime



def generate_uuid():
    return UUID(uuid=list(uuid.uuid4().bytes))


def make_quaternion(yaw_rad):
    q = tf_transformations.quaternion_from_euler(0, 0, yaw_rad)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

class ReservedSpotPublisher(Node):
    def __init__(self):
        super().__init__('reserved_spot_publisher')

        self.received_initial_list = False

        self.publisher_ = self.create_publisher(String, '/parking_spots/reserved', 10)
        self.sub = self.create_subscription(
            String,
            '/parking_spots/reserved',
            self.callback,
            10
        )

        self.parking_spot = []  # You can change or set this externally if needed
        self.target_spot = 17
        
        self.timer = None

    def start_periodic_publishing(self):
        # Wait up to 2 seconds for callback to populate list
        wait_time = time.time() + 2.0
        while not self.received_initial_list and time.time() < wait_time:
            rclpy.spin_once(self, timeout_sec=0.1)

        if not self.received_initial_list:
            self.get_logger().warn("⚠️ Did not receive current reserved list before starting publisher. Starting anyway...")

        if self.timer is None:
            self.timer = self.create_timer(2.0, self.publish_periodically)

    def stop_periodic_publishing(self):
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None

    def publish_periodically(self):
        if self.target_spot not in self.parking_spot:
            self.parking_spot.append(self.target_spot)

        msg = String()
        msg.data = f"Reserved Spots: {self.parking_spot}"
        self.publisher_.publish(msg)
        self.get_logger().info(f"[NPC] 🛰 Published: {msg.data}")

    def remove_spot(self, spot):
        if spot in self.parking_spot:
            self.parking_spot.remove(spot)
            msg = String()
            msg.data = f"Reserved Spots: {self.parking_spot}"
            self.publisher_.publish(msg)
            self.get_logger().info(f"🗑️ Removed spot {spot}, updated list: {self.parking_spot}")


    def callback(self, msg):
        try:
            raw = msg.data.replace("Reserved Spots:", "").strip().strip("[] ")
            if raw:
                self.parking_spot = [int(x.strip()) for x in raw.split(',') if x.strip().isdigit()]
            else:
                self.parking_spot = []
            self.received_initial_list = True
        except Exception as e:
            self.get_logger().warn(f"Error parsing reserved spots: {e}")

class NPCDummyCarPublisher(Node):

    npc_counter = 0
    
    def __init__(self, reserved_spot_publisher):
        super().__init__('npc_dummy_car_publisher')

        self.parking_spot = reserved_spot_publisher.parking_spot

        self.reserved_spot_publisher = reserved_spot_publisher

        self.reserved_spot_publisher.start_periodic_publishing()

        self.pub = self.create_publisher(
            DummyObject,
            '/simulation/dummy_perception_publisher/object_info',
            10
        )

        self.active = True

        self.trajectories = [
            {"path": [(-58.1, -32.7, 0.25)], "max_v": 0.0, "min_v": 0.0, "delay": None},
            {"path": [(-58.1, -32.7, 0.25)], "max_v": 3.0, "min_v": 3.0, "delay": 9.0},
            {"path": [(-33.0, -26.4, 0.25)], "max_v": -1.0, "min_v": -1.0, "delay": 2.0},
            {"path": [(-34.6, -28.1, -5.25)], "max_v": 0.0, "min_v": 0.0, "delay": 2.0},
            {"path": [(-34.3, -26.9, -4.45)], "max_v": 1.0, "min_v": 1.0, "delay": 8.0},
            {"path": [(-36.4, -20.6, -4.45)], "max_v": 0.0, "min_v": 0.0, "delay": None},
        ]

        self.index = 0

        # Generate unique car ID
        NPCDummyCarPublisher.npc_counter += 1
        self.car_id = f"car_{NPCDummyCarPublisher.npc_counter}"

        # Queue publishing

        self.queue_list = []
        self.queue_pub = self.create_publisher(String, '/avp/dropoff_queue', 10)


        self.queue_sub = self.create_subscription(
            String,
            '/avp/dropoff_queue',
            self.queue_callback,
            10
        )


        # Give the subscription some time to receive the current queue
        rclpy.spin_once(self, timeout_sec=0.5)

        # Add this car to the queue only if it's not already there
        if self.car_id not in self.queue_list and self.active:
            self.queue_list.append(self.car_id)
            self.publish_queue()
            # self.get_logger().info(f"✅ {self.car_id} added to queue at startup.")


        self.object_id = generate_uuid()

        time.sleep(1.0)
        self.publish_static_car()
        threading.Thread(target=self.wait_for_input, daemon=True).start()


    def queue_callback(self, msg):
        data = msg.data.replace("Drop-off Queue:", "").strip()
        data = data.strip("[] ")
        if data:
            self.queue_list = [car.strip() for car in data.split(",")]
        else:
            self.queue_list = []

        # self.get_logger().info(f"📥 Received current queue: {self.queue_list}")

        # Only add if car is active (i.e., still in drop-off zone)
        if self.active and self.car_id not in self.queue_list:
            self.queue_list.append(self.car_id)
            self.publish_queue()
            # self.get_logger().info(f"✅ Added {self.car_id} to queue and published.")




    def publish_queue(self):
        msg = String()
        msg.data = "Drop-off Queue: [" + ", ".join(self.queue_list) + "]"
        self.queue_pub.publish(msg)
        # self.get_logger().info(f"📤 Published queue: {msg.data}")



    def publish_static_car(self):
        traj = self.trajectories[0]
        self.publish_pose(traj["path"][0], traj["max_v"], traj["min_v"])
        self.get_logger().info("Published initial static car. Waiting for input...")

    def wait_for_input(self):
        self.get_logger().info("Waiting for input to start parking... (Press 'y' and Enter)")
        while True:
            user_input = sys.stdin.readline().strip().lower()
            if user_input == 'y':
                # self.get_logger().info("✅ Received input to start parking.")

                self.active = False

                # Remove ALL instances of self.car_id from the queue
                self.queue_list = [car for car in self.queue_list if car != self.car_id]
                self.publish_queue()

                self.run_trajectory_sequence()
                break


    def send_delete_all(self):
        obj = DummyObject()
        obj.header = Header()
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.header.frame_id = 'map'
        obj.action = DummyObject.DELETEALL
        self.pub.publish(obj)
        # self.get_logger().info("🗑️ Sent DELETE_ALL command.")

    def publish_pose(self, pos_tuple, max_v, min_v):
        x, y, yaw = pos_tuple

        obj = DummyObject()
        obj.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='map')
        obj.id = self.object_id

        pose = Pose()
        pose.position = Point(x=x, y=y, z=0.0)
        pose.orientation = make_quaternion(yaw)
        obj.initial_state.pose_covariance.pose = pose
        obj.initial_state.pose_covariance.covariance = [0.01] * 36
        obj.initial_state.twist_covariance.covariance = [0.0] * 36
        obj.initial_state.accel_covariance.covariance = [0.0] * 36

        classification = ObjectClassification()
        classification.label = ObjectClassification.CAR
        classification.probability = 1.0
        obj.classification = classification

        shape = Shape()
        shape.type = Shape.BOUNDING_BOX
        shape.dimensions = Vector3(x=4.0, y=1.8, z=2.0)
        obj.shape = shape

        obj.max_velocity = max_v
        obj.min_velocity = min_v
        obj.action = DummyObject.ADD

        self.pub.publish(obj)
        # self.get_logger().info(f"📍 Published dummy NPC pose at x={x:.2f}, y={y:.2f}")

    def run_trajectory_sequence(self):
        for i in range(1, len(self.trajectories)):  # skip first one (already static)
            traj = self.trajectories[i]

            self.send_delete_all()
            self.object_id = generate_uuid()
            self.publish_pose(traj["path"][0], traj["max_v"], traj["min_v"])

            delay = traj["delay"]
            if delay is not None:
                # self.get_logger().info(f"⏱️ Waiting {delay} seconds before next move...")
                time.sleep(delay)

        if self.parking_spot:
            removed = self.parking_spot[0]
            self.reserved_spot_publisher.remove_spot(removed)
            time.sleep(2)    
            self.reserved_spot_publisher.stop_periodic_publishing()


            # delay to let it remove spot and publishing that message before stopping the publishing
            # time.sleep(0.5)            
                


def main(args=None):
    rclpy.init(args=args)

    reserved_spot_publisher = ReservedSpotPublisher()
    npc_dummy_car_publisher = NPCDummyCarPublisher(reserved_spot_publisher)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(npc_dummy_car_publisher)
    executor.add_node(reserved_spot_publisher)
    executor.spin()

    npc_dummy_car_publisher.destroy_node()
    reserved_spot_publisher.destroy_node()

    rclpy.shutdown()
    


if __name__ == '__main__':
    main()
