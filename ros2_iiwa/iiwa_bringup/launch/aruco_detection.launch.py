from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aruco_ros',
            executable='single',
            name='aruco_single',
            output='screen',
            parameters=[{
                'image_is_rectified': True,
                'marker_size': 0.1,  # dimensione del tuo marker in metri
                'marker_id': 201,    # ID del tuo marker ArUco
                'reference_frame': 'camera_optical_link',  # frame di riferimento
                'camera_frame': 'camera_optical_link',
                'marker_frame': 'aruco_marker_frame',
                'corner_refinement': 'LINES',
            }],
            remappings=[
                ('/camera_info', '/iiwa/camera_info'),
                ('/image', '/iiwa/camera'),
            ]
        )
    ])