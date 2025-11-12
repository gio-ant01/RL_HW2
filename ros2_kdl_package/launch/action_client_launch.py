from launch import LaunchDescription 
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('ros2_kdl_package')  
    config_file = os.path.join(pkg_share, 'config', 'action_client_params.yaml')

    # Declare launch argument for controller type
    ctrl_arg = DeclareLaunchArgument(
        'ctrl',
        default_value='velocity_ctrl',
        description='Controller type: velocity_ctrl or velocity_ctrl_null'
    )
    # Get the launch configuration
    ctrl_param = LaunchConfiguration('ctrl')

    return LaunchDescription([
        ctrl_arg,
        Node(
            package='ros2_kdl_package',    
            executable='ros2_kdl_action_client',
            name='linear_trajectory_action_client',
            output='screen',
            parameters=[
                config_file,
                {'ctrl': ctrl_param}  # Override ctrl parameter from command line
            ],
            emulate_tty=True,
        )
    ])

