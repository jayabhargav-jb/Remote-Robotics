#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
import numpy as np

class SlamDiagnosticNode(Node):
    def __init__(self):
        super().__init__('slam_diagnostic_node')
        
        # Initialize TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10)
        
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
            
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        
        # Initialize diagnostic info
        self.last_map_time = None
        self.last_odom_time = None
        self.last_scan_time = None
        self.map_updates = 0
        
        # Create timer for periodic diagnostics
        self.create_timer(1.0, self.run_diagnostics)
    
    def map_callback(self, msg):
        self.last_map_time = self.get_clock().now()
        self.map_updates += 1
        
        # Check map data
        if msg.data:
            non_zero = sum(1 for cell in msg.data if cell != -1)
            self.get_logger().info(f'Map received: {len(msg.data)} cells, {non_zero} mapped cells')
        else:
            self.get_logger().warn('Empty map data received')
    
    def odom_callback(self, msg):
        self.last_odom_time = self.get_clock().now()
        
        # Check for zero-only values
        pos = msg.pose.pose.position
        if pos.x == 0.0 and pos.y == 0.0 and pos.z == 0.0:
            self.get_logger().warn('Odometry showing all zero values!')
            
        # Log odometry data
        self.get_logger().info(
            f'Odometry received - Position: x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}')
    
    def scan_callback(self, msg):
        self.last_scan_time = self.get_clock().now()
        
        # Check scan data
        valid_ranges = sum(1 for r in msg.ranges if r > msg.range_min and r < msg.range_max)
        self.get_logger().info(f'Scan received: {valid_ranges}/{len(msg.ranges)} valid readings')
    
    def check_tf_tree(self):
        required_transforms = [
            ('map', 'odom'),
            ('odom', 'base_link'),
            ('base_link', 'base_laser')  # or whatever your laser frame is named
        ]
        
        results = []
        for parent, child in required_transforms:
            try:
                transform: TransformStamped = self.tf_buffer.lookup_transform(
                    parent,
                    child,
                    rclpy.time.Time())
                results.append(f'Transform {parent} -> {child}: OK')
            except Exception as e:
                results.append(f'Transform {parent} -> {child}: MISSING ({str(e)})')
        
        return results
    
    def run_diagnostics(self):
        self.get_logger().info('\n=== SLAM Diagnostics ===')
        
        # Check topic timestamps
        now = self.get_clock().now()
        
        if self.last_map_time:
            delta = (now - self.last_map_time).nanoseconds / 1e9
            self.get_logger().info(f'Map updates: {self.map_updates}, Last update: {delta:.1f}s ago')
        else:
            self.get_logger().warn('No map messages received!')
            
        if self.last_odom_time:
            delta = (now - self.last_odom_time).nanoseconds / 1e9
            self.get_logger().info(f'Last odometry: {delta:.1f}s ago')
        else:
            self.get_logger().warn('No odometry messages received!')
            
        if self.last_scan_time:
            delta = (now - self.last_scan_time).nanoseconds / 1e9
            self.get_logger().info(f'Last scan: {delta:.1f}s ago')
        else:
            self.get_logger().warn('No laser scan messages received!')
        
        # Check TF tree
        self.get_logger().info('\n=== TF Tree Status ===')
        for result in self.check_tf_tree():
            self.get_logger().info(result)

def main(args=None):
    rclpy.init(args=args)
    node = SlamDiagnosticNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()