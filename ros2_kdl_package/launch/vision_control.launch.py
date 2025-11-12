from launch import LaunchDescription 
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Declare arguments
    K_vision_arg = DeclareLaunchArgument(
        'K_vision',
        default_value='1.0',
        description='Vision control gain'
    )

    lambda_arg = DeclareLaunchArgument(
        'lambda',
        default_value='0.5',
        description='Joint limit avoidance parameter'
    )

    return LaunchDescription([
        K_vision_arg,
        lambda_arg,
        Node(
            package='ros2_kdl_package',    
            executable='ros2_kdl_vision_node',
            name='ros2_kdl_vision_node',
            output='screen',
            parameters=[{
                'cmd_interface': 'velocity',
                'K_vision': LaunchConfiguration('K_vision'),
                'lambda': LaunchConfiguration('lambda')
            }],
            emulate_tty=True,
        )
    ])