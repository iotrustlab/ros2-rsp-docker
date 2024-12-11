#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import yaml
import numpy as np
import math
from dataclasses import dataclass
from typing import Dict, Tuple, List
import os
import json
from datetime import datetime
import time

@dataclass
class JointLimits:
    min: float
    max: float
    velocity_limit: float
    acceleration_limit: float

@dataclass
class RobotParams:
    wheel_radius: float
    wheel_separation: float
    max_linear_velocity: float
    max_angular_velocity: float
    max_wheel_acceleration: float

class DeterministicFuzzer(Node):
    def __init__(self):
        super().__init__('deterministic_fuzzer')
        
        # Load configuration first
        self.load_config()
        
        # Initialize random seed for reproducibility
        np.random.seed(self.config['seed'])
        
        # Initialize state tracking
        self.joint_states = {joint: 0.0 for joint in self.joint_limits.keys()}
        self.joint_velocities = {joint: 0.0 for joint in self.joint_limits.keys()}
        self.last_velocities = self.joint_velocities.copy()
        self.pattern_time = 0.0
        self.start_time = time.time()
        self.last_save_time = self.start_time
        
        # Initialize data recording
        self.recorded_data = []
        
        # Set up publisher
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        
        # Create timer based on configured frequency
        self.dt = 1.0 / self.config['frequency']
        self.timer = self.create_timer(self.dt, self.timer_callback)
        
        self.get_logger().info('Turtlebot4 Fuzzer initialized with configuration')
        self.get_logger().info(f"Test duration: {self.config['test_sequence']['duration_seconds']} seconds")

    def load_config(self):
        try:
            with open('/config/fuzzing_config.yaml', 'r') as f:
                self.config = yaml.safe_load(f)['fuzzing']
                
            # Load robot parameters
            self.robot_params = RobotParams(
                wheel_radius=self.config['robot_params']['wheel_radius'],
                wheel_separation=self.config['robot_params']['wheel_separation'],
                max_linear_velocity=self.config['robot_params']['max_linear_velocity'],
                max_angular_velocity=self.config['robot_params']['max_angular_velocity'],
                max_wheel_acceleration=self.config['robot_params']['max_wheel_acceleration']
            )
            
            # Load joint limits
            self.joint_limits = {}
            for joint, limits in self.config['joint_limits'].items():
                self.joint_limits[joint] = JointLimits(
                    min=limits['min'],
                    max=limits['max'],
                    velocity_limit=limits['velocity_limit'],
                    acceleration_limit=limits['acceleration_limit']
                )
                
        except Exception as e:
            self.get_logger().error(f'Error loading config: {str(e)}')
            raise

    def apply_acceleration_limits(self, target_velocities: Dict[str, float]) -> Dict[str, float]:
        """Apply acceleration limits to velocity changes"""
        limited_velocities = {}
        for joint, target_vel in target_velocities.items():
            current_vel = self.last_velocities[joint]
            max_change = self.joint_limits[joint].acceleration_limit * self.dt
            
            if abs(target_vel - current_vel) > max_change:
                # Limit acceleration
                if target_vel > current_vel:
                    limited_velocities[joint] = current_vel + max_change
                else:
                    limited_velocities[joint] = current_vel - max_change
            else:
                limited_velocities[joint] = target_vel
                
        return limited_velocities

    def wheel_velocities_from_twist(self, linear: float, angular: float) -> Dict[str, float]:
        """Convert linear and angular velocities to wheel velocities with limits"""
        # Calculate basic wheel velocities
        left_velocity = (linear - angular * self.robot_params.wheel_separation / 2.0) / self.robot_params.wheel_radius
        right_velocity = (linear + angular * self.robot_params.wheel_separation / 2.0) / self.robot_params.wheel_radius
        
        # Create velocity dictionary
        velocities = {
            'left_wheel_joint': left_velocity,
            'right_wheel_joint': right_velocity
        }
        
        # Apply velocity limits
        for joint in velocities:
            velocities[joint] = np.clip(
                velocities[joint],
                -self.joint_limits[joint].velocity_limit,
                self.joint_limits[joint].velocity_limit
            )
            
        return velocities

    def get_pattern_motion(self, pattern: str, t: float) -> Tuple[float, float]:
        """Generate motion based on pattern configuration"""
        params = self.config['patterns']['parameters']
        
        if pattern == 'circle':
            radius = params['circle']['radius']
            linear = params['circle']['linear_velocity']
            angular = linear / radius
            return linear, angular
            
        elif pattern == 'figure_eight':
            linear = params['figure_eight']['linear_velocity']
            period = params['figure_eight']['period']
            angular = self.robot_params.max_angular_velocity * 0.5 * math.sin(2 * math.pi * t / period)
            return linear, angular
            
        elif pattern == 'square':
            t_mod = t % 8.0
            linear_vel = params['square']['linear_velocity']
            side_time = params['square']['side_length'] / linear_vel
            
            if t_mod % (side_time + 1.0) < side_time:
                return linear_vel, 0.0
            else:
                return 0.0, self.robot_params.max_angular_velocity * 0.5
                
        elif pattern == 'spiral':
            base_linear = params['spiral']['linear_velocity']
            expansion = params['spiral']['expansion_rate']
            radius = params['spiral']['initial_radius'] + expansion * t
            angular = base_linear / max(radius, 0.1)
            return base_linear, angular
            
        else:  # stop pattern
            return 0.0, 0.0
    
    def generate_motion(self, t: float) -> Dict[str, float]:
        """Generate realistic differential drive motion"""
        # Get current pattern
        pattern_duration = self.config['patterns']['duration']
        patterns = self.config['patterns']['sequence']
        current_pattern = patterns[int(t / pattern_duration) % len(patterns)]
        
        # Get motion commands
        linear_vel, angular_vel = self.get_pattern_motion(current_pattern, t % pattern_duration)
        
        # Convert to wheel velocities
        target_velocities = self.wheel_velocities_from_twist(linear_vel, angular_vel)
        
        # Apply acceleration limits
        limited_velocities = self.apply_acceleration_limits(target_velocities)
        
        # Update velocities and positions
        self.last_velocities = self.joint_velocities.copy()
        self.joint_velocities = limited_velocities  # These velocities should be non-zero
        
        # Integrate velocities to positions
        for joint in self.joint_states:
            # Update position
            new_position = self.joint_states[joint] + self.joint_velocities[joint] * self.dt
            
            # Apply position limits using modulo for continuous rotation
            limit_range = self.joint_limits[joint].max - self.joint_limits[joint].min
            self.joint_states[joint] = (((new_position - self.joint_limits[joint].min) % limit_range) 
                                        + self.joint_limits[joint].min)
        
        return self.joint_states

    def save_data(self):
        """Save recorded data to file"""
        if not self.recorded_data:
            return
            
        # Create output directory if it doesn't exist
        output_path = self.config['test_sequence']['data_output_path']
        os.makedirs(output_path, exist_ok=True)
        
        # Generate filename with timestamp
        filename = os.path.join(
            output_path,
            f'fuzzing_data_{datetime.now().strftime("%Y%m%d_%H%M%S")}.json'
        )
        
        # Save data
        with open(filename, 'w') as f:
            json.dump(self.recorded_data, f, indent=2)
            
        self.get_logger().info(f'Saved data to {filename}')
        self.recorded_data = []  # Clear after saving

    def timer_callback(self):
        # Check if test duration has been exceeded
        current_time = time.time()
        if current_time - self.start_time > self.config['test_sequence']['duration_seconds']:
            self.save_data()
            self.get_logger().info('Test sequence completed')
            rclpy.shutdown()
            return
            
        # Generate message
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        
        # Generate motion
        joint_positions = self.generate_motion(self.pattern_time)
        
        # Populate message
        msg.name = list(joint_positions.keys())
        msg.position = list(joint_positions.values())
        msg.velocity = list(self.joint_velocities.values())
        msg.effort = [0.0] * len(msg.name)
        
        # Record data if interval has elapsed
        if current_time - self.last_save_time >= self.config['test_sequence']['save_interval_seconds']:
            self.recorded_data.append({
                'timestamp': current_time,
                'joint_states': {
                    name: float(pos) for name, pos in zip(msg.name, msg.position)
                },
                'velocities': {
                    name: float(vel) for name, vel in zip(msg.name, msg.velocity)
                }
            })
            self.last_save_time = current_time
        
        # Update time tracking
        self.pattern_time += self.dt
        
        # Log pattern changes
        pattern_duration = self.config['patterns']['duration']
        if int(self.pattern_time / pattern_duration) != int((self.pattern_time - self.dt) / pattern_duration):
            patterns = self.config['patterns']['sequence']
            current_pattern = patterns[int(self.pattern_time / pattern_duration) % len(patterns)]
            self.get_logger().info(f'Switching to {current_pattern} pattern')
        
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