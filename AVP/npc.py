import rclpy
from rclpy.node import Node
from tier4_simulation_msgs.msg import DummyObject
from autoware_perception_msgs.msg import Shape, ObjectClassification
from geometry_msgs.msg import Pose, Quaternion, Vector3, Point
from unique_identifier_msgs.msg import UUID
from std_msgs.msg import Header
import uuid
import tf_transformations


def generate_uuid():
    return UUID(uuid=list(uuid.uuid4().bytes))

def make_quaternion(yaw_rad):
    q = tf_transformations.quaternion_from_euler(0, 0, yaw_rad)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


class NPCDummyCarPublisher(Node):
    def __init__(self):
        super().__init__('npc_dummy_car_publisher')

        self.pub = self.create_publisher(
            DummyObject,
            '/simulation/dummy_perception_publisher/object_info',
            10
        )

        self.trajectories = [
            {"path": [(-66.0, -34.0, 0.25)], "max_v": 3.0, "min_v": 3.0, "delay": 13.0},
            {"path": [(-33.0, -26.4, 0.25)], "max_v": -2.0, "min_v": -2.0, "delay": 2.0},
            {"path": [(-34.6, -28.1, -5.25)], "max_v": 0.0, "min_v": 0.0, "delay": 2.0},
            {"path": [(-34.3, -26.9, -4.45)], "max_v": 1.0, "min_v": 1.0, "delay": 8.0},
            {"path": [(-36.4, -20.6, -4.45)], "max_v": 0.0, "min_v": 0.0, "delay": None},  # final
        ]

        self.index = 0
        self.counter = 0
        self.object_id = generate_uuid()

        self.timer = self.create_timer(1.0, lambda: self.run_trajectory(0))
        self.delete_timer = self.create_timer(self.trajectories[0]["delay"], self.send_delete_all) if self.trajectories[0]["delay"] else None

    def send_delete_all(self):
        if self.delete_timer:
            self.delete_timer.cancel()
        if self.timer:
            self.timer.cancel()

        self.pub.publish(DummyObject(
            header=Header(stamp=self.get_clock().now().to_msg(), frame_id="map"),
            action=DummyObject.DELETEALL
        ))

        self.counter += 1
        self.index = 0
        self.object_id = generate_uuid()

        if self.counter < len(self.trajectories):
            traj = self.trajectories[self.counter]
            self.timer = self.create_timer(1.0, lambda: self.run_trajectory(self.counter))
            if traj["delay"]:
                self.delete_timer = self.create_timer(traj["delay"], self.send_delete_all)



    def run_trajectory(self, traj_index):
        traj = self.trajectories[traj_index]
        path = traj["path"]

        if self.index >= len(path):
            self.get_logger().info("Final pose reached. Stopping publisher.")
            if self.timer:
                self.timer.cancel()
            return

        x, y, yaw = path[self.index]
        self.index += 1

        obj = DummyObject()
        obj.header = Header()
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.header.frame_id = 'map'
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

        obj.max_velocity = traj["max_v"]
        obj.min_velocity = traj["min_v"]
        obj.action = DummyObject.ADD

        self.pub.publish(obj)


def main(args=None):
    rclpy.init(args=args)
    node = NPCDummyCarPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
