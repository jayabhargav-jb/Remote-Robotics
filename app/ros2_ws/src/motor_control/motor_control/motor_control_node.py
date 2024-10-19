import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import minimalmodbus
import time

class MotorControl:

    def __init__(self, slave_id):
        self.instrument = minimalmodbus.Instrument('/dev/ttyUSB1', slave_id)
        self.instrument.serial.baudrate = 9600
        self.instrument.serial.timeout = 2  # seconds
        self.instrument.mode = minimalmodbus.MODE_ASCII
        self.slave_id = slave_id

    def set_motor_speed(self, speed_rpm, direction):
        # Set motor direction
        if direction == 0:
            self.instrument.write_register(2, 257, 0, functioncode=6)  # Forward
        elif direction == 1:
            self.instrument.write_register(2, 265, 0, functioncode=6)  # Reverse

        # Set motor speed (RPM)
        self.instrument.write_register(14, speed_rpm, 0, functioncode=6)  # Speed register (40015)

    def stop(self):
        # Set motor to E-stop mode
        self.instrument.write_register(self.slave_id, 1792, 0, functioncode=6)  # E-Stop mode


class Stop:
    @staticmethod
    def emergency():
        slave_ids = [1, 5, 6, 7]  # All motor IDs
        for slave_id in slave_ids:
            instrument = minimalmodbus.Instrument('/dev/ttyUSB1', slave_id)
            instrument.serial.baudrate = 9600
            instrument.serial.timeout = 2
            instrument.mode = minimalmodbus.MODE_ASCII
            instrument.write_register(2, 1792, 0, functioncode=6)
            instrument.serial.close()


class MotorControlNode(Node):

    def __init__(self):
        super().__init__('motor_control_node')

        # Initialize motor objects for each wheel
        self.motor_back_left = MotorControl(1)
        self.motor_back_right = MotorControl(5)
        self.motor_front_left = MotorControl(6)
        self.motor_front_right = MotorControl(7)

        # ROS 2 subscription to 'cmd_vel'
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

    def cmd_vel_callback(self, msg: Twist):
        linear_velocity = msg.linear.x  # Forward/backward velocity (m/s)
        angular_velocity = msg.angular.z  # Rotational velocity (rad/s)

        # Calculate individual motor speeds (in RPM)
        speed_bl = self.calculate_motor_speed(linear_velocity, angular_velocity, -1)  # Back Left
        speed_br = self.calculate_motor_speed(linear_velocity, angular_velocity, 1)   # Back Right
        speed_fl = self.calculate_motor_speed(linear_velocity, angular_velocity, -1)  # Front Left
        speed_fr = self.calculate_motor_speed(linear_velocity, angular_velocity, 1)   # Front Right

        # Set motor directions
        direction_bl = 0 if speed_bl >= 0 else 1
        direction_br = 0 if speed_br >= 0 else 1
        direction_fl = 0 if speed_fl >= 0 else 1
        direction_fr = 0 if speed_fr >= 0 else 1

        # Apply absolute values since speed cannot be negative
        self.motor_back_left.set_motor_speed(abs(speed_bl), direction_bl)
        self.motor_back_right.set_motor_speed(abs(speed_br), direction_br)
        self.motor_front_left.set_motor_speed(abs(speed_fl), direction_fl)
        self.motor_front_right.set_motor_speed(abs(speed_fr), direction_fr)

        # Log motor commands
        self.get_logger().info(f"Back Left: Speed={abs(speed_bl)}, Direction={direction_bl}")
        self.get_logger().info(f"Back Right: Speed={abs(speed_br)}, Direction={direction_br}")
        self.get_logger().info(f"Front Left: Speed={abs(speed_fl)}, Direction={direction_fl}")
        self.get_logger().info(f"Front Right: Speed={abs(speed_fr)}, Direction={direction_fr}")

    def calculate_motor_speed(self, linear_vel, angular_vel, side):
        """
        Calculate motor speed based on linear and angular velocity.

        Args:
            linear_vel: Linear velocity from cmd_vel (m/s)
            angular_vel: Angular velocity from cmd_vel (rad/s)
            side: 1 for right motors, -1 for left motors (helps in turning)

        Returns:
            Motor speed in RPM (bounded between 0 and 9750 RPM)
        """
        # Scaling constants (example values; adjust as needed)
        linear_scale = 5000  # Conversion from m/s to motor RPM
        angular_scale = 1000  # Conversion from rad/s to differential motor RPM

        # Calculate motor speed
        speed_rpm = (linear_vel * linear_scale) + (side * angular_vel * angular_scale)
        return max(0, min(9750, int(speed_rpm)))  # Bound speed within [0, 9750] RPM


def main(args=None):
    rclpy.init(args=args)
    motor_control_node = MotorControlNode()

    try:
        rclpy.spin(motor_control_node)
    except KeyboardInterrupt:
        motor_control_node.get_logger().info('Shutting down motor control node...')
        Stop.emergency()
    finally:
        motor_control_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
