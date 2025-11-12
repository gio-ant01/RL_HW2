#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class ArucoPoseSubscriber(Node):
    def __init__(self):
        super().__init__('aruco_pose_subscriber')
        
        # Subscribe to ArUco detection
        self.subscription = self.create_subscription(
            PoseStamped,
            '/aruco_single/pose',
            self.pose_callback,
            10
        )
        
        self.get_logger().info('ArUco Pose Subscriber initialized')
        self.get_logger().info('  Listening to: /aruco_single/pose')
       
    def pose_callback(self, msg):
        """
        Callback che riceve la pose del marker ArUco e la ri-pubblica.
        msg è di tipo geometry_msgs/PoseStamped
        """
        self.get_logger().info('='*50)
        self.get_logger().info(f'ArUco marker detected!')
        self.get_logger().info(f'Frame: {msg.header.frame_id}')
        self.get_logger().info(f'Position in {msg.header.frame_id}:')
        self.get_logger().info(f'  x: {msg.pose.position.x:.3f} m')
        self.get_logger().info(f'  y: {msg.pose.position.y:.3f} m')
        self.get_logger().info(f'  z: {msg.pose.position.z:.3f} m')
        self.get_logger().info(f'Orientation (quaternion):')
        self.get_logger().info(f'  x: {msg.pose.orientation.x:.3f}')
        self.get_logger().info(f'  y: {msg.pose.orientation.y:.3f}')
        self.get_logger().info(f'  z: {msg.pose.orientation.z:.3f}')
        self.get_logger().info(f'  w: {msg.pose.orientation.w:.3f}')

        

def main(args=None):
    rclpy.init(args=args)
    node = ArucoPoseSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()