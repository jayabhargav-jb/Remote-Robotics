import rclpy.subscription
import socketio
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import rclpy
from sensor_msgs.msg import LaserScan 
import uvicorn
import asyncio
import json
import numpy as np
import cv2
# import cors
import base64


dot_radius = 3

# def laser_scan_old(msg):
#     print("hello")

#     # await sio.emit('stream', {"json":"hi"})
#     pass

def laser_scan(msg):
    global img_base64

    # Image dimensions
    width = 800
    height = 800

    # Create an empty image with a black background
    image = np.zeros((height, width, 3), dtype=np.uint8)  # 3 channels for BGR

    # Image center (origin) in pixel coordinates
    center_x = width // 2
    center_y = height // 2

    # Fixed scaling factor
    scale = 70  # Adjust this value as needed for visualization

    # Handle NaN values and find the max distance
    valid_ranges = [r for r in msg.ranges if not (np.isnan(r) or r == float('inf')) and r > 0]
    if valid_ranges:
        # print("Valid ranges detected")
        pass
    else:
        # print("No valid ranges detected")
        return

    # Draw the LaserScan data on the image
    dot_radius = 5  # Radius for the dots
    for i, distance in enumerate(msg.ranges):
        if np.isnan(distance) or distance <= 0:
            continue

        # Convert polar to Cartesian coordinates
        angle = msg.angle_min + i * msg.angle_increment
        x = distance * np.cos(angle)
        y = distance * np.sin(angle)

        # Convert Cartesian coordinates to image coordinates with fixed scaling
        try:
            img_x = int(center_x + x * scale)
            img_y = int(center_y - y * scale)
        except Exception as e:
            # print(f"Error converting coordinates: {e}")
            continue  # Skip this iteration if there's an error

        # Check if the coordinates are within the image boundaries
        if 0 <= img_x < width and 0 <= img_y < height:
            # Draw a filled circle for each point
            cv2.circle(image, (img_x, img_y), dot_radius, (0, 0, 255), -1)  # Red dot

    # Encode the image as JPEG
    _, buffer = cv2.imencode('.jpg', image)
    img_bytes = buffer.tobytes()

    # Encode the image to base64 for sending over Socket.IO
    img_base64 = base64.b64encode(img_bytes).decode('utf-8')



app = FastAPI()

rclpy.init()
node = rclpy.create_node('laser_scan_listener')
subscription = node.create_subscription(LaserScan, '/scan', laser_scan, 10)

req = dict()
app.add_middleware(CORSMiddleware, allow_origins=["http://172.26.153.161:3001"], allow_credentials=True, allow_methods=["*"], allow_headers=["*"])
sio = socketio.AsyncServer(async_mode='asgi', cors_allowed_origins=["http://172.26.153.16:3001"])
app.mount("/", socketio.ASGIApp(sio, other_asgi_app=app))

@sio.event
async def connect(sid, environ):
    print('Client connected:', sid)

@sio.event
async def disconnect(sid):
    print('Client disconnected:', sid)

@sio.event
async def stop(sid,data):
    global req
    req["command"] = "stop"
    print("stopped")

@sio.event
async def start(sid, data):
    global req
    if data:
        # print(type(data))
        req = data
    #     print(data)
    #     req = json.loads(data)
    #     print(req)
        while req["command"] == "start":
            rclpy.spin_once(node)
            await asyncio.sleep(1/20)
            await sio.emit('stream', img_base64)
            # if data[""]


if __name__ == '__main__':
    uvicorn.run(app, host='0.0.0.0', port=5000)
    rclpy.spin()