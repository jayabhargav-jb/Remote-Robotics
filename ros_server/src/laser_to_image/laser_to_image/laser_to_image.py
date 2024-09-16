# laser_to_image_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class LaserScanToImageNode(Node):
    def __init__(self):
        super().__init__('laser_scan_to_image_node')
        
        # Create a publisher for the image
        self.image_pub = self.create_publisher(Image, 'laser_scan_image', 10)
        
        # Create a subscription to LaserScan
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )
        
        # Create a CvBridge object to convert between ROS and OpenCV images
        self.bridge = CvBridge()

    def scan_callback(self, msg: LaserScan):
        # Image dimensions
        width = 800
        height = 800

        # Create an empty image with white background
        image = np.ones((height, width), dtype=np.uint8) * 255

        # Image center (origin) in pixel coordinates
        center_x = width // 2
        center_y = height // 2

        # Handle NaN values and find the max distance
        valid_ranges = [r for r in msg.ranges if not (np.isnan(r) or np.isinf(r)) and r > 0]
        if valid_ranges:
            max_distance = max(valid_ranges)
            print("max_d:",max_distance)
        else:
            # If all values are NaN or invalid, we can't create a meaningful image
            return

        # Draw the LaserScan data on the image
        for i, distance in enumerate(msg.ranges):
            if np.isnan(distance) or distance <= 0:
                continue
            
            # Convert polar to Cartesian coordinates
            angle = msg.angle_min + i * msg.angle_increment
            x = distance * np.cos(angle)
            y = distance * np.sin(angle)

            # Convert Cartesian coordinates to image coordinates
            try:
                img_x = int(center_x + x * (width / (2 * max_distance)))
                img_y = int(center_y - y * (height / (2 * max_distance)))
            except:
                img_x, img_y = width+1, height+1
            # Check if the coordinates are within the image boundaries
            if 0 <= img_x < width and 0 <= img_y < height:
                image[img_y, img_x] = 0  # Set pixel to black

        # Convert numpy array to ROS Image message
        ros_image = self.bridge.cv2_to_imgmsg(image, encoding='mono8')
        
        # Publish the image
        self.image_pub.publish(ros_image)

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanToImageNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
