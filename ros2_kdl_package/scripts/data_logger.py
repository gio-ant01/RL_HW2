#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import csv
import os
from datetime import datetime

class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')
        
        # Declare parameter for controller type
        self.declare_parameter('ctrl_type', 'velocity_ctrl')
        self.ctrl_type = self.get_parameter('ctrl_type').value
        
        # Create filename with timestamp and controller type
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.filename = f'robot_data_{self.ctrl_type}_{timestamp}.csv'
        
        # Create CSV file and write header
        self.csv_file = open(self.filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp',
            'q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7',  # joint positions
            'dq1', 'dq2', 'dq3', 'dq4', 'dq5', 'dq6', 'dq7',  # joint velocities
            'cmd1', 'cmd2', 'cmd3', 'cmd4', 'cmd5', 'cmd6', 'cmd7'  # commanded velocities
        ])
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.cmd_sub = self.create_subscription(
            Float64MultiArray,
            '/velocity_controller/commands',
            self.cmd_callback,
            10
        )
        
        # Storage for latest data
        self.latest_joint_state = None
        self.latest_cmd = None
        
        # Timer to write data at fixed rate (100 Hz)
        self.timer = self.create_timer(0.01, self.write_data)
        
        self.get_logger().info(f'Data logger started. Saving to: {self.filename}')
        self.get_logger().info(f'Controller type: {self.ctrl_type}')
    
    def joint_state_callback(self, msg):
        self.latest_joint_state = msg
    
    def cmd_callback(self, msg):
        self.latest_cmd = msg.data
    
    def write_data(self):
        if self.latest_joint_state is not None and self.latest_cmd is not None:
            timestamp = self.get_clock().now().nanoseconds / 1e9
            
            # Get joint positions and velocities
            positions = self.latest_joint_state.position
            velocities = self.latest_joint_state.velocity
            
            # Get commanded velocities
            commands = self.latest_cmd
            
            # Write row
            row = [timestamp]
            row.extend(positions)
            row.extend(velocities)
            row.extend(commands)
            
            self.csv_writer.writerow(row)
    
    def __del__(self):
        if hasattr(self, 'csv_file'):
            self.csv_file.close()
            self.get_logger().info(f'Data saved to: {self.filename}')

def main(args=None):
    rclpy.init(args=args)
    node = DataLogger()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()