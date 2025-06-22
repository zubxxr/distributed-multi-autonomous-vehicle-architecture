from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    avp_file = LaunchConfiguration('avp_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'avp_file',
            default_value='avp_node_planning_simulator.py',
            description='Which AVP file to run'
        ),
        DeclareLaunchArgument(
            'vehicle_id',
            default_value='1',
            description='Vehicle ID to pass to the AVP script'
        ),

        # Queue Manager
        Node(
            package='multi_avp_managers',
            executable='queue_manager',
            name='queue_manager',
            output='screen',
            arguments=['--ros-args', '-p', 'namespaces:=[main, vehicle2]']
        ),

        # Reservation Manager
        Node(
            package='multi_avp_managers',
            executable='reservation_manager',
            name='reservation_manager',
            output='screen',
            arguments=['--ros-args', '-p', 'namespaces:=[main, vehicle2]']
        ),

        # Vehicle Count Manager
        Node(
            package='multi_avp_managers',
            executable='vehicle_count_manager',
            name='vehicle_count_manager',
            output='screen',
            arguments=['--ros-args', '-p', 'namespaces:=[main, vehicle2]']
        ),

        # AVP Script
        Node(
            package='multi_avp_nodes',
            executable=avp_file,
            name='avp_script',
            output='screen',
            arguments=['--vehicle_id', LaunchConfiguration('vehicle_id')],
        ),
    ])

