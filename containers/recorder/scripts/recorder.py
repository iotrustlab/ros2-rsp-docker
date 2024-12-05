#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from datetime import datetime
import json
import os

class StateRecorder(Node):
    def __init__(self):
        super().__init__('state_recorder')
        
        # Create results directory
        self.results_dir = '/data/results'
        os.makedirs(self.results_dir, exist_ok=True)
        
        # Initialize data storage
        self.joint_states_data = []
        self.tf_data = []
        
        # Subscribe to topics
        self.joint_states_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_states_callback,
            10
        )
        
        self.tf_sub = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10
        )
        
        self.get_logger().info('Recorder initialized')

    def joint_states_callback(self, msg):
        # Convert ROS time to string timestamp
        timestamp = datetime.fromtimestamp(
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        ).isoformat()
        
        data = {
            'timestamp': timestamp,
            'joint_names': list(msg.name),  # Convert tuple to list for JSON
            'positions': list(msg.position),
            'velocities': list(msg.velocity) if msg.velocity else [],
            'efforts': list(msg.effort) if msg.effort else []
        }
        
        self.joint_states_data.append(data)
        self._save_periodic()

    def tf_callback(self, msg):
        timestamp = datetime.fromtimestamp(
            msg.transforms[0].header.stamp.sec + 
            msg.transforms[0].header.stamp.nanosec * 1e-9
        ).isoformat()
        
        transforms = []
        for t in msg.transforms:
            transform = {
                'child_frame_id': t.child_frame_id,
                'translation': {
                    'x': float(t.transform.translation.x),
                    'y': float(t.transform.translation.y),
                    'z': float(t.transform.translation.z)
                },
                'rotation': {
                    'x': float(t.transform.rotation.x),
                    'y': float(t.transform.rotation.y),
                    'z': float(t.transform.rotation.z),
                    'w': float(t.transform.rotation.w)
                }
            }
            transforms.append(transform)
        
        data = {
            'timestamp': timestamp,
            'transforms': transforms
        }
        
        self.tf_data.append(data)
        self._save_periodic()

    def _save_periodic(self, force=False):
        # Save every 100 messages or when forced
        if len(self.joint_states_data) % 100 == 0 or force:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            
            # Save joint states
            if self.joint_states_data:
                joint_states_file = f'{self.results_dir}/joint_states_{timestamp}.json'
                with open(joint_states_file, 'w') as f:
                    json.dump(self.joint_states_data, f, indent=2)
                self.get_logger().info(f'Saved joint states to {joint_states_file}')
            
            # Save transforms
            if self.tf_data:
                tf_file = f'{self.results_dir}/transforms_{timestamp}.json'
                with open(tf_file, 'w') as f:
                    json.dump(self.tf_data, f, indent=2)
                self.get_logger().info(f'Saved transforms to {tf_file}')

def main():
    rclpy.init()
    recorder = StateRecorder()
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        # Save any remaining data
        recorder._save_periodic(force=True)
        recorder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()