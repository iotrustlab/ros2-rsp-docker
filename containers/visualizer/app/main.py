#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse
import json
import asyncio
from threading import Thread
from typing import Set
import logging
import yaml

app = FastAPI()
app.mount("/static", StaticFiles(directory="static"), name="static")

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class ConnectionManager:
    def __init__(self):
        self.active_connections: Set[WebSocket] = set()

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.add(websocket)
        logger.info(f"New client connected. Total connections: {len(self.active_connections)}")

    def disconnect(self, websocket: WebSocket):
        self.active_connections.remove(websocket)
        logger.info(f"Client disconnected. Total connections: {len(self.active_connections)}")

    async def broadcast(self, message: str):
        disconnected = set()
        for connection in self.active_connections:
            try:
                await connection.send_text(message)
            except WebSocketDisconnect:
                disconnected.add(connection)
            except Exception as e:
                logger.error(f"Error broadcasting message: {e}")
                disconnected.add(connection)
        
        for conn in disconnected:
            self.disconnect(conn)

class Visualizer(Node):
    def __init__(self, connection_manager):
        super().__init__('visualizer')
        self.connection_manager = connection_manager
        
        # Load configuration
        try:
            with open('config/visualizer_config.yaml', 'r') as f:
                self.config = yaml.safe_load(f)
        except Exception as e:
            logger.error(f"Error loading config: {e}")
            self.config = {}
        
        # Create subscriptions
        self.create_subscription(
            JointState, 
            'joint_states', 
            self.joint_states_callback, 
            10
        )
        self.create_subscription(
            TFMessage, 
            '/tf', 
            self.tf_callback, 
            10
        )

    def joint_states_callback(self, msg):
        try:
            data = {
                'type': 'joint_states',
                'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                'data': {
                    'names': list(msg.name),
                    'positions': list(map(float, msg.position)),
                    'velocities': list(map(float, msg.velocity)),
                    'efforts': list(map(float, msg.effort))
                }
            }
            asyncio.run(self.connection_manager.broadcast(json.dumps(data)))
        except Exception as e:
            logger.error(f"Error in joint_states_callback: {e}")

    def tf_callback(self, msg):
        try:
            data = {
                'type': 'transforms',
                'timestamp': msg.transforms[0].header.stamp.sec + 
                        msg.transforms[0].header.stamp.nanosec * 1e-9,
                'transforms': [
                    {
                        'child_frame_id': t.child_frame_id,  # Match expected property name
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
                    } for t in msg.transforms
                ]
            }
            asyncio.run(self.connection_manager.broadcast(json.dumps(data)))
        except Exception as e:
            logger.error(f"Error in tf_callback: {e}")

manager = ConnectionManager()

@app.get("/", response_class=HTMLResponse)
async def get():
    with open('static/index.html') as f:
        return f.read()

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        manager.disconnect(websocket)
    except Exception as e:
        logger.error(f"WebSocket error: {e}")
        manager.disconnect(websocket)

def run_ros():
    rclpy.init()
    visualizer = Visualizer(manager)
    rclpy.spin(visualizer)

if __name__ == "__main__":
    ros_thread = Thread(target=run_ros, daemon=True)
    ros_thread.start()
    
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)