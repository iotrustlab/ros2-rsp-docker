#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import yaml
import numpy as np
import math
from dataclasses import dataclass
from typing import Dict

@dataclass
class JointLimits:
    min: float
    max: float
    velocity_limit: float = 2.0  # rad/s
    acceleration_limit: float = 1.0  # rad/s^2

class DeterministicFuzzer(Node):
    def __init__(self):
        super().__init__('deterministic_fuzzer')
        
        # Load configuration
        self.load_config()
        
        # Initialize state tracking
        self.joint_states = {joint: 0.0 for joint in self.joint_limits.keys()}
        self.joint_velocities = {joint: 0.0 for joint in self.joint_limits.keys()}
        self.last_update_time = self.get_clock().now()
        self.pattern_time = 0.0
        
        # Publisher
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        
        # Timer for publishing
        self.timer = self.create_timer(
            1.0 / self.config['frequency'],
            self.timer_callback
        )
        
        self.get_logger().info('Fuzzer initialized')

    def load_config(self):
        with open('/config/fuzzing_config.yaml', 'r') as f:
            self.config = yaml.safe_load(f)['fuzzing']
            
        # Convert joint limits to structured format
        self.joint_limits = {}
        for joint, limits in self.config['joint_limits'].items():
            self.joint_limits[joint] = JointLimits(
                min=limits['min'],
                max=limits['max']
            )
        
        np.random.seed(self.config['seed'])

    def generate_differential_drive(self, t: float) -> Dict[str, float]:
        """Generate realistic differential drive motion"""
        # Base motion parameters
        forward_velocity = 0.5  # m/s
        turning_radius = 1.0  # m
        wheel_separation = 0.235  # m
        
        # Generate smooth transitions
        linear_vel = forward_velocity * math.sin(0.2 * t)
        angular_vel = 0.5 * math.sin(0.1 * t)
        
        # Calculate wheel velocities
        left_wheel_vel = (linear_vel - angular_vel * wheel_separation/2) 
        right_wheel_vel = (linear_vel + angular_vel * wheel_separation/2)
        
        # Integrate for positions
        dt = 1.0 / self.config['frequency']
        self.joint_states['left_wheel_joint'] += left_wheel_vel * dt
        self.joint_states['right_wheel_joint'] += right_wheel_vel * dt
        
        # Apply limits
        for joint in self.joint_states:
            limits = self.joint_limits[joint]
            self.joint_states[joint] = np.clip(
                self.joint_states[joint], 
                limits.min, 
                limits.max
            )
        
        return self.joint_states

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        
        # Generate realistic motion
        joint_positions = self.generate_differential_drive(self.pattern_time)
        
        # Populate message
        msg.name = list(joint_positions.keys())
        msg.position = list(joint_positions.values())
        msg.velocity = [0.0] * len(msg.name)  # Could add actual velocities if needed
        msg.effort = [0.0] * len(msg.name)
        
        # Update time tracking
        self.pattern_time += 1.0 / self.config['frequency']
        
        # Publish
        self.publisher.publish(msg)

def main():
    rclpy.init()
    try:
        fuzzer = DeterministicFuzzer()
        rclpy.spin(fuzzer)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()