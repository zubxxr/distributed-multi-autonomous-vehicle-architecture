import rclpy
from rclpy.node import Node
# from autoware_adapi_v1_msgs.msg import RouteState
from tier4_planning_msgs.msg import RouteState
from std_msgs.msg import String
from nav_msgs.msg import Odometry

from shapely.geometry import Point, Polygon
import datetime 
import subprocess
import time

def is_in_drop_off_zone(x, y):
    return drop_zone_polygon.contains(Point(x, y))

DROP_OFF_ZONE_POLYGON = [
    (-57.1, -28.6),
    (-55.2, -35.8),
    (-70.7, -31.9),
    (-68.3, -39.2)
]

drop_zone_polygon = Polygon(DROP_OFF_ZONE_POLYGON)

drop_off_queue = []
drop_off_counter = 1
cars_in_zone = set()

def dropoff_queue_callback(msg):
    global drop_off_queue
    try:
        data = msg.data.replace("Drop-off Queue: [", "").replace("]", "").strip()
        if data:
            drop_off_queue = [item.strip() for item in data.split(",")]
        else:
            drop_off_queue = []
    except Exception as e:
        print(f"Failed to parse drop-off queue: {e}")



def update_drop_off_queue(car_id, x, y, publisher):
    global drop_off_counter
    if is_in_drop_off_zone(x, y): 
        if car_id not in cars_in_zone:
            # Prevent duplicate car labels for the same ID
            existing_numbers = [int(car.replace("car_", "")) for car in drop_off_queue if car.startswith("car_")]
            next_car_num = max(existing_numbers) + 1 if existing_numbers else 1
            label = f"car_{next_car_num}"

            if label not in drop_off_queue:
                drop_off_queue.append(label)
                cars_in_zone.add(car_id)
                print(f"{label} entered the drop-off zone.")
                publish_queue(publisher)
    else:
        if car_id in cars_in_zone:
            cars_in_zone.remove(car_id)

def publish_queue(publisher):
    queue_str = ", ".join(drop_off_queue)
    msg = String()
    msg.data = f"Drop-off Queue: [{queue_str}]"
    publisher.publish(msg)

class AVPCommandListener(Node):
    def __init__(self, route_state_subscriber):
        super().__init__('avp_command_listener')

        self.route_state_subscriber = route_state_subscriber
        self.head_to_drop_off = False
        self.status_initialized = False
        self.initiate_parking = False
        self.state = -1
        self.publisher_ = self.create_publisher(String, '/avp/status', 10)
        self.queue_publisher = self.create_publisher(String, '/avp/dropoff_queue', 10)
        
        self.create_subscription(
            String,
            '/avp/dropoff_queue',
            dropoff_queue_callback,
            10
        )

        self.subscription = self.create_subscription(
            String,
            '/avp/command',
            self.command_callback,
            10)
        

        self.create_subscription(Odometry, '/localization/kinematic_state', self.odom_callback, 10)
        self.ego_x = None
        self.ego_y = None
        self.ego_id = "ego"

        
    
    def odom_callback(self, msg):
        self.ego_x = msg.pose.pose.position.x
        self.ego_y = msg.pose.pose.position.y
        update_drop_off_queue(self.ego_id, self.ego_x, self.ego_y, self.queue_publisher)

    
    def command_callback(self, msg):
        if msg.data == "start":
            self.head_to_drop_off = True
            self.publisher_.publish(String(data="Heading to drop-off zone"))
            
class ParkingSpotSubscriber(Node):
    def __init__(self):
        super().__init__('parking_spot_subscriber')
        self.available_parking_spots = None

        self.subscription = self.create_subscription(
            String,
            '/parking_spots/empty',
            self.available_parking_spots_callback,
            1)

    def available_parking_spots_callback(self, msg):
        self.available_parking_spots = msg.data

class RouteStateSubscriber(Node):

    def __init__(self):
        super().__init__('route_state_subscriber')
        self.subscription = self.create_subscription(
            RouteState,
            '/planning/mission_planning/route_selector/main/state',
            self.route_state_callback,
            10)
        self.flag = False
        self.state = -1

    def route_state_callback(self, msg):
        if msg.state == 6:
            print("\nCar has been parked.")
            self.state = 6
    
def run_ros2_command(command):
    try:
        subprocess.run(command, shell=True, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except subprocess.CalledProcessError as e:
        print(f"Error executing command: {e}")
    
def main(args=None):
    rclpy.init(args=args)
    
    route_state_subscriber = RouteStateSubscriber()
    avp_command_listener = AVPCommandListener(route_state_subscriber)
    parking_spot_subscriber = ParkingSpotSubscriber()
    reserved_spots_publisher = avp_command_listener.create_publisher(String, '/parking_spots/reserved', 10)

    engage_auto_mode = "ros2 topic pub --once /autoware/engage autoware_vehicle_msgs/msg/Engage '{engage: True}' -1"
    
    # Farther one
    # set_initial_pose = "ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{header: {stamp: {sec: 1749585776, nanosec: 961698513}, frame_id: 'map'}, pose: {pose: {position: {x: -61.5980224609375, y: -84.97380065917969, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.792272260736117, w: 0.6101677350270843}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467]}}'"

    # Closer one
    set_initial_pose = "ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{header: {stamp: {sec: 1749608320, nanosec: 716034807}, frame_id: 'map'}, pose: {pose: {position: {x: -71.57246398925781, y: -48.895652770996094, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.7899923832424256, w: 0.6131166564520593}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467]}}'"

    head_to_drop_off = "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1749596637, nanosec: 370052949}, frame_id: 'map'}, pose: {position: {x: -57.330997467041016, y: -32.513362884521484, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.11762553592567591, w: 0.9930580211136696}}}' --once"

    # set_goal_pose_entrance = "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1708315201, nanosec: 354400841}, frame_id: 'map'}, pose: {position: {x: 3730.5029296875, y: 73724.484375, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.9714600644596665, w: 0.2372031685286277}}}' --once"

    parking_spot_locations = {
        1: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1736234930, nanosec: 485255271}, frame_id: 'map'}, pose: {position: {x: -44.340084075927734, y: -14.542484283447266, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.6048959869666175, w: 0.79630449261051}}}' --once",
        2: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1736234964, nanosec: 268967027}, frame_id: 'map'}, pose: {position: {x: -41.737525939941406, y: -13.785665512084961, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.6017180648737738, w: 0.7987085641237115}}}' --once",
        3: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1736235015, nanosec: 500375897}, frame_id: 'map'}, pose: {position: {x: -38.7567138671875, y: -12.567403793334961, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.6019133113169981, w: 0.7985614351190562}}}' --once",
        4: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1736235477, nanosec: 950428247}, frame_id: 'map'}, pose: {position: {x: -35.9503173828125, y: -11.982339859008789, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.6007336954096015, w: 0.799449202388447}}}' --once",
        5: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1736235517, nanosec: 532171546}, frame_id: 'map'}, pose: {position: {x: -32.98591232299805, y: -11.230877876281738, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.603871811065959, w: 0.7970814486612511}}}' --once",
        6: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1736235543, nanosec: 363285134}, frame_id: 'map'}, pose: {position: {x: -30.08383560180664, y: -10.155061721801758, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.608585037220489, w: 0.7934886593211878}}}' --once",
        7: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1736235564, nanosec: 777861615}, frame_id: 'map'}, pose: {position: {x: -27.20238494873047, y: -9.49226188659668, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.6007326637076104, w: 0.7994499776438543}}}' --once",
        8: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1736235588, nanosec: 16728058}, frame_id: 'map'}, pose: {position: {x: -24.6075382232666, y: -8.993618965148926, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.6036992773480252, w: 0.797212131449009}}}' --once",
        15: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1736235678, nanosec: 86261653}, frame_id: 'map'}, pose: {position: {x: -42.034812927246094, y: -22.750713348388672, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.7969156099507567, w: 0.6040906476819629}}}' --once",
        16: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1736235705, nanosec: 118039164}, frame_id: 'map'}, pose: {position: {x: -39.109901428222656, y: -22.36135482788086, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.7978757289989354, w: 0.6028219646582376}}}' --once",
        17: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1736235729, nanosec: 260929592}, frame_id: 'map'}, pose: {position: {x: -36.33027648925781, y: -21.73630142211914, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.7999696817030807, w: 0.6000404222682599}}}' --once",
        18: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1736235765, nanosec: 496677516}, frame_id: 'map'}, pose: {position: {x: -33.3206672668457, y: -20.794353485107422, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.7897854690370711, w: 0.6133831697217438}}}' --once",
        19: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1736235786, nanosec: 558316281}, frame_id: 'map'}, pose: {position: {x: -30.4317569732666, y: -19.839237213134766, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.7983271798448894, w: 0.6022239732200185}}}' --once",
    }

    # Simulate AWSIM initial pose
    run_ros2_command(set_initial_pose)

    initiate_parking = True
    # print("Park your car? (yes/no)")

    # user_input = input().lower()

    # if user_input == "yes" or user_input == "y":
    #     initiate_parking = True

    counter = 0
    new_counter = 0

    chosen_parking_spot = None

    drop_off_flag = True

    drop_off_completed = False

    while rclpy.ok():

        if not avp_command_listener.status_initialized:
            avp_command_listener.publisher_.publish(String(data="Arrived at location."))
            avp_command_listener.status_initialized = True

        # If "Start AVP" is clicked
        if drop_off_flag and avp_command_listener.head_to_drop_off:
            run_ros2_command(head_to_drop_off)
            run_ros2_command(engage_auto_mode)
            drop_off_flag = False


        # Simulate the drop-off sequence once car arrives at destination
        # if route_state_subscriber.state == 6 and not avp_command_listener.initiate_parking:


        # print(avp_command_listener.ego_x)
        # print(avp_command_listener.ego_y)


        if (    
            not avp_command_listener.initiate_parking and
            not drop_off_completed and
            avp_command_listener.ego_x is not None and
            is_in_drop_off_zone(avp_command_listener.ego_x, avp_command_listener.ego_y)
        ):
            print("Drop-off valet destination reached.")
            avp_command_listener.publisher_.publish(String(data="Arrived at drop-off area."))
            time.sleep(2)
            avp_command_listener.publisher_.publish(String(data="Owner is exiting..."))
            time.sleep(7)
            avp_command_listener.publisher_.publish(String(data="Owner has exited. Switching to autonomous mode."))
            time.sleep(2)
            avp_command_listener.publisher_.publish(String(data="Beginning autonomous valet parking..."))
            time.sleep(2)
            avp_command_listener.initiate_parking = True
            drop_off_completed = True
            avp_command_listener.publisher_.publish(String(data="Waiting for an available parking spot..."))
            
        ############### START PARKING SPOT DETECTION ###############

        if avp_command_listener.initiate_parking and route_state_subscriber.state == 6:

            rclpy.spin_once(parking_spot_subscriber, timeout_sec=0.1)  

            parking_spots = parking_spot_subscriber.available_parking_spots

            if parking_spots is None:
                print('Waiting for available parking spots...')

            if parking_spots is not None:
                
                first_spot_in_queue = int(parking_spots.split(': ')[1].split(',')[0])

                if first_spot_in_queue != chosen_parking_spot:
                    
                    if counter == 1:
                        print('\nParking spot #%s was taken.' % chosen_parking_spot)
                        counter = 0

                    # Get the first value from the priority queue
                    first_spot_in_queue = int(parking_spots.split(': ')[1].split(',')[0])

                    # Printing the first value
                    print('\nAvailable parking spot found: %s' % first_spot_in_queue)

                    avp_command_listener.publisher_.publish(String(data="Available parking spot found."))
                    time.sleep(2)
                    avp_command_listener.publisher_.publish(String(data=f"Parking in Spot {first_spot_in_queue}."))
                    
                    parking_spot_goal_pose_command = parking_spot_locations[first_spot_in_queue]
                    run_ros2_command(parking_spot_goal_pose_command)
                    run_ros2_command(engage_auto_mode)


                    # Publish the reserved spot to Unity
                    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                    reserved_msg = String()
                    reserved_msg.data = f"{timestamp}: {first_spot_in_queue}"
                    print(reserved_msg)
                    reserved_spots_publisher.publish(reserved_msg)

                    route_state_subscriber.state = -1

                    print('Setting goal pose to parking spot:', first_spot_in_queue)

                    counter += 1
                    chosen_parking_spot = first_spot_in_queue

                
            time.sleep(1)
        ############### END PARKING SPOT DETECTION #################

        if route_state_subscriber.state == 6:
            avp_command_listener.publisher_.publish(String(data="Car has been parked."))

        # if new_counter == 0:
        #     if route_state_subscriber.state == 6:
                # print("Retrieve car? (yes/no)")

                # user_input = input().lower()

                # if user_input == "yes" or user_input == "y":
                #     print("Going to Drop Off Zone.")
                #     run_ros2_command(set_goal_pose_entrance)
                #     run_ros2_command(engage_auto_mode)
                # elif user_input == "no" or user_input == "n":
                #     print("Exiting the script.")
                #     exit()
                # else:
                #     print("Invalid input. Please enter 'yes' or 'no'.")

                # route_state_subscriber.state = -1
                # new_counter += 1                  
        
        rclpy.spin_once(route_state_subscriber, timeout_sec=1)
        rclpy.spin_once(avp_command_listener, timeout_sec=1)

    route_state_subscriber.destroy_node()
    parking_spot_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
