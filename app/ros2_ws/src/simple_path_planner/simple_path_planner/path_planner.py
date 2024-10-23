#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import tf_transformations

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        
        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)

        # Robot state
        self.position = [0.0, 0.0]
        self.orientation = 0.0
        self.current_map = None
        self.map_resolution = 0.05
        self.frontier_points = []
        self.current_frontier = None
        self.exploring = False
        
        # Exploration parameters
        self.min_distance_to_obstacle = 0.05
        self.map_coverage_threshold = 0.95
        self.unknown_cell_value = -1
        self.previous_unknown_count = float('inf')

        # Create timers for exploration and movement control
        self.exploration_timer = self.create_timer(2.0, self.exploration_loop)
        self.movement_timer = self.create_timer(0.05, self.movement_control)  # 20 Hz

        self.get_logger().info('Path Planner initialized')

    def odom_callback(self, msg):
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y
        
        orientation_q = msg.pose.pose.orientation
        _, _, self.orientation = tf_transformations.euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)

        valid_readings = np.isfinite(ranges) & (ranges > 0.1)
        filtered_ranges = ranges[valid_readings]
        filtered_angles = angles[valid_readings]

        if np.any(filtered_ranges < self.min_distance_to_obstacle):
            self.stop_robot()
            self.find_new_frontier()

    def map_callback(self, msg):
        self.current_map = msg
        
        if self.current_map is not None:
            unknown_count = sum(1 for cell in self.current_map.data if cell == self.unknown_cell_value)
            total_cells = len(self.current_map.data)
            coverage = 1 - (unknown_count / total_cells)

            if unknown_count < self.previous_unknown_count * 0.95:
                self.get_logger().info(f'Map coverage: {coverage*100:.2f}%')
                self.previous_unknown_count = unknown_count
            
            if coverage > self.map_coverage_threshold:
                self.get_logger().info('Mapping complete!')
                self.stop_robot()
                self.exploring = False

    def find_frontiers(self):
        if self.current_map is None:
            return []

        frontiers = []
        map_data = np.array(self.current_map.data).reshape(
            (self.current_map.info.height, self.current_map.info.width))

        for y in range(1, self.current_map.info.height - 1):
            for x in range(1, self.current_map.info.width - 1):
                if map_data[y, x] == self.unknown_cell_value:
                    neighbors = map_data[y-1:y+2, x-1:x+2]
                    if np.any((neighbors >= 0) & (neighbors < 50)):
                        world_point = self.map_to_world(x, y)
                        if world_point:
                            frontiers.append(world_point)

        return frontiers

    def find_new_frontier(self):
        frontiers = self.find_frontiers()
        if not frontiers:
            self.get_logger().info('No more frontiers to explore!')
            self.exploring = False
            self.stop_robot()
            return
            
        distances = [math.sqrt((f[0] - self.position[0])**2 + (f[1] - self.position[1])**2) for f in frontiers]
        self.current_frontier = frontiers[np.argmin(distances)]
        self.exploring = True

    def movement_control(self):
        if not self.exploring or self.current_frontier is None:
            return
            
        dx = self.current_frontier[0] - self.position[0]
        dy = self.current_frontier[1] - self.position[1]
        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)

        if distance < self.map_resolution * 2:
            self.current_frontier = None
            self.find_new_frontier()
            return

        angle_diff = target_angle - self.orientation
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

        cmd = Twist()

        if abs(angle_diff) > 0.1:
            cmd.angular.z = 1.0 if angle_diff > 0 else -1.0  # Faster rotation
        else:
            cmd.linear.x = min(0.5, distance)  # Faster forward speed
            cmd.angular.z = angle_diff

        self.cmd_vel_pub.publish(cmd)

    def exploration_loop(self):
        if not self.exploring and self.current_map is not None:
            self.find_new_frontier()

    def map_to_world(self, map_x, map_y):
        if self.current_map is None:
            return None
        
        x = map_x * self.map_resolution + self.current_map.info.origin.position.x
        y = map_y * self.map_resolution + self.current_map.info.origin.position.y
        
        return [x, y]

    def stop_robot(self):
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
