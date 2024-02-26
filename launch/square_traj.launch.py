from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    launch_args = [DeclareLaunchArgument('lapNum',default_value='2')]

    offboard_node = Node(
    name='px4_offboard_node',
    package='drone_control',
    executable='offboard_node',
    parameters=[{'lap_numbers': LaunchConfiguration('lapNum')}],
    output='screen'
    )

    ld = LaunchDescription(launch_args)
    ld.add_action(offboard_node)
    return ld
