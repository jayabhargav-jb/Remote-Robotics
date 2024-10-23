# max_distance_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
from numpy import isinf

class MaxDistanceNode(Node):
    def __init__(self):
        super().__init__('max_distance_node')

        # Subscriber to LiDAR data
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',  # Make sure this matches your LiDAR topic
            self.lidar_callback,
            10
        )

    def lidar_callback(self, msg):
        # Process the LaserScan data
        max_distance = 0.0
        max_angle = 0.0

        for i, distance in enumerate(msg.ranges):
            if distance < msg.range_min or distance > msg.range_max: # or isinf(distance)
                continue  # Skip invalid distances

            if distance > max_distance or isinf(distance):
                max_distance = distance
                max_angle = msg.angle_min + i * msg.angle_increment

        # Calculate the (x, y) position from polar coordinates
        x = max_distance * math.cos(max_angle)
        y = max_distance * math.sin(max_angle)

        self.get_logger().info(f'Max Distance: {max_distance:.2f} m at angle: {max_angle:.2f} rad')
        self.get_logger().info(f'Coordinates: (x: {x:.2f}, y: {y:.2f})')

def main(args=None):
    rclpy.init(args=args)
    max_distance_node = MaxDistanceNode()

    try:
        rclpy.spin(max_distance_node)
    except KeyboardInterrupt:
        pass
    finally:
        max_distance_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
