#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import yaml
import numpy as np
import time

class DeterministicFuzzer(Node):
    def __init__(self):
        super().__init__('deterministic_fuzzer')
        
        # Load configuration
        try:
            with open('/config/fuzzing_config.yaml', 'r') as f:  # Changed path
                self.config = yaml.safe_load(f)['fuzzing']
        except FileNotFoundError as e:
            self.get_logger().error(f'Config file not found: {e}')
            raise
        
        # Set random seed for reproducibility
        np.random.seed(self.config['seed'])
        
        # Initialize publisher
        self.publisher = self.create_publisher(
            JointState,
            'joint_states',
            10
        )
        
        # Create timer
        self.timer_period = 1.0 / self.config['frequency']
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.joint_names = list(self.config['joint_limits'].keys())
        self.sequence_counter = 0
        
        self.get_logger().info('Fuzzer initialized successfully')

    def generate_joint_value(self, joint_name):
        limits = self.config['joint_limits'][joint_name]
        return np.random.uniform(limits['min'], limits['max'])

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        
        msg.name = self.joint_names
        msg.position = [self.generate_joint_value(joint) for joint in self.joint_names]
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)
        
        self.publisher.publish(msg)
        self.sequence_counter += 1

def main():
    rclpy.init()
    try:
        fuzzer = DeterministicFuzzer()
        rclpy.spin(fuzzer)
    except Exception as e:
        print(f"Error initializing fuzzer: {e}")
        raise
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()