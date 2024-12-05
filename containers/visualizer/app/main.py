# containers/visualizer/app/main.py
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from fastapi import FastAPI, WebSocket
from fastapi.staticfiles import StaticFiles
import json
import asyncio
from threading import Thread

app = FastAPI()
app.mount("/static", StaticFiles(directory="static"), name="static")

class Visualizer(Node):
    def __init__(self, websocket_handler):
        super().__init__('visualizer')
        self.websocket_handler = websocket_handler
        
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

    def joint_states_callback(self, msg):
        data = {
            'type': 'joint_states',
            'data': {
                'positions': list(msg.position),
                'names': list(msg.name)
            }
        }
        asyncio.run(self.websocket_handler.broadcast(json.dumps(data)))

    def tf_callback(self, msg):
        data = {
            'type': 'transforms',
            'data': [
                {
                    'child_frame': t.child_frame_id,
                    'translation': {
                        'x': t.transform.translation.x,
                        'y': t.transform.translation.y,
                        'z': t.transform.translation.z
                    },
                    'rotation': {
                        'x': t.transform.rotation.x,
                        'y': t.transform.rotation.y,
                        'z': t.transform.rotation.z,
                        'w': t.transform.rotation.w
                    }
                } for t in msg.transforms
            ]
        }
        asyncio.run(self.websocket_handler.broadcast(json.dumps(data)))

class WebSocketHandler:
    def __init__(self):
        self.connections = set()

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.connections.add(websocket)

    async def broadcast(self, message: str):
        for connection in self.connections:
            try:
                await connection.send_text(message)
            except:
                self.connections.remove(connection)

websocket_handler = WebSocketHandler()

@app.get("/")
async def get():
    return {"message": "ROS2 Visualizer"}

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket_handler.connect(websocket)
    try:
        while True:
            await websocket.receive_text()
    except:
        pass

def run_ros():
    rclpy.init()
    visualizer = Visualizer(websocket_handler)
    rclpy.spin(visualizer)

if __name__ == "__main__":
    ros_thread = Thread(target=run_ros)
    ros_thread.start()
    
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)