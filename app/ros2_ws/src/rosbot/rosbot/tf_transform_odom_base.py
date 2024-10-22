import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf2_ros
import geometry_msgs.msg
import numpy as np

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        self.timer = self.create_timer(0.1, self.publish_odometry)
        
        # Initial position and orientation
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # TF2 broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def cmd_vel_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        
        # Simple differential drive model
        if dt > 0:
            # Update position based on velocity commands
            v = msg.linear.x
            w = msg.angular.z

            # Update robot's position
            self.x += v * np.cos(self.theta) * dt
            self.y += v * np.sin(self.theta) * dt
            self.theta += w * dt
            
            self.last_time = current_time

    def publish_odometry(self):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Fill in the odometry message
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation.z = np.sin(self.theta / 2)
        odom_msg.pose.pose.orientation.w = np.cos(self.theta / 2)

        self.odom_pub.publish(odom_msg)

        # Create and publish the transform
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = odom_msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom_msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    odometry_publisher = OdometryPublisher()
    try:
        rclpy.spin(odometry_publisher)
    except:
        odometry_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
