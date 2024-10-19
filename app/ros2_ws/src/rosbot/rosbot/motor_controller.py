#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import minimalmodbus
import math

class MinimalModbusNode(Node):
	def __init__(self):
		super().__init__('minimal_modbus_node')
		self.subscription = self.create_subscription(
		Twist,
		'cmd_vel',
		self.listener_callback,
		10)
		self.subscription  # prevent unused variable warning

		self.rmcs2303_fright = minimalmodbus.Instrument('/dev/ttyUSB0', 7, minimalmodbus.MODE_ASCII)
		self.rmcs2303_fleft = minimalmodbus.Instrument('/dev/ttyUSB0', 6, minimalmodbus.MODE_ASCII)
		self.rmcs2303_rright = minimalmodbus.Instrument('/dev/ttyUSB0', 5, minimalmodbus.MODE_ASCII)
		self.rmcs2303_rleft = minimalmodbus.Instrument('/dev/ttyUSB0', 4, minimalmodbus.MODE_ASCII)

		self.rmcs2303_fright.serial.baudrate = 9600
		self.rmcs2303_fleft.serial.baudrate = 9600
		self.rmcs2303_rright.serial.baudrate = 9600
		self.rmcs2303_rleft.serial.baudrate = 9600


	def listener_callback(self, msg):
		gear_ratio = 90
		wheel_radius = 0.1 #meters
		lin_velocity = msg.linear.x # m/s
		ang_velocity = msg.angular.z
		right_velocity = lin_velocity + 0.05*ang_velocity
		left_velocity = lin_velocity - 0.05*ang_velocity
		rpm_right = abs((right_velocity / (2*math.pi*wheel_radius))*60*gear_ratio)
		rpm_left = abs((left_velocity / (2*math.pi*wheel_radius))*60*gear_ratio)

		try:
			self.rmcs2303_fright.write_register(2, 2048, number_of_decimals=0, functioncode=6, signed=False) # set encoder count to 0
			self.rmcs2303_fleft.write_register(2, 2048, number_of_decimals=0, functioncode=6, signed=False) # set encoder count to 0
			self.rmcs2303_rright.write_register(2, 2048, number_of_decimals=0, functioncode=6, signed=False) # set encoder count to 0
			self.rmcs2303_rleft.write_register(2, 2048, number_of_decimals=0, functioncode=6, signed=False) # set encoder count to 0
			
			if (ang_velocity<0): #turn right
				rpm_left= rpm_left + 4000
				self.rmcs2303_fright.write_register(14, 0 , number_of_decimals=0, functioncode=6, signed=False)  # Speed command for base motor in rpm
				self.rmcs2303_fleft.write_register(14, rpm_left , number_of_decimals=0, functioncode=6, signed=False)  # Speed command for base motor in rpm
				self.rmcs2303_rright.write_register(14,0 , number_of_decimals=0, functioncode=6, signed=False)  # Speed command for base motor in rpm
				self.rmcs2303_rleft.write_register(14, rpm_left , number_of_decimals=0, functioncode=6, signed=False)  # Speed command for base motor in rpm
				
			elif (ang_velocity>0): #turn left
				rpm_right= rpm_right + 4000
				self.rmcs2303_fright.write_register(14, rpm_right , number_of_decimals=0, functioncode=6, signed=False)  # Speed command for base motor in rpm
				self.rmcs2303_fleft.write_register(14,0 , number_of_decimals=0, functioncode=6, signed=False)  # Speed command for base motor in rpm
				self.rmcs2303_rright.write_register(14, rpm_right , number_of_decimals=0, functioncode=6, signed=False)  # Speed command for base motor in rpm
				self.rmcs2303_rleft.write_register(14, 0 , number_of_decimals=0, functioncode=6, signed=False)  # Speed command for base motor in rpm
				
			else:
				self.rmcs2303_fright.write_register(14, rpm_right , number_of_decimals=0, functioncode=6, signed=False)  # Speed command for base motor in rpm
				self.rmcs2303_fleft.write_register(14, rpm_left , number_of_decimals=0, functioncode=6, signed=False)  # Speed command for base motor in rpm
				self.rmcs2303_rright.write_register(14, rpm_right , number_of_decimals=0, functioncode=6, signed=False)  # Speed command for base motor in rpm
				self.rmcs2303_rleft.write_register(14, rpm_left , number_of_decimals=0, functioncode=6, signed=False)  # Speed command for base motor in rpm
			

			if (lin_velocity>=0):
				self.rmcs2303_fright.write_register(2, 257, number_of_decimals=0, functioncode=6, signed=False) # Enable fright motor in CW
				self.rmcs2303_fleft.write_register(2, 265, number_of_decimals=0, functioncode=6, signed=False) # Enable fleft motor in CCW
				self.rmcs2303_rright.write_register(2, 257, number_of_decimals=0, functioncode=6, signed=False) # Enable rright motor in CW
				self.rmcs2303_rleft.write_register(2, 265, number_of_decimals=0, functioncode=6, signed=False) # Enable rleft motor in CCW
			else:
				self.rmcs2303_fright.write_register(2, 265, number_of_decimals=0, functioncode=6, signed=False) # Enable fright motor in CCW
				self.rmcs2303_fleft.write_register(2, 257, number_of_decimals=0, functioncode=6, signed=False) # Enable fleft motor in CW
				self.rmcs2303_rright.write_register(2, 265, number_of_decimals=0, functioncode=6, signed=False) # Enable rright motor in CCW
				self.rmcs2303_rleft.write_register(2, 257, number_of_decimals=0, functioncode=6, signed=False) # Enable rleft motor in CW


			fright_speed_feedback = self.rmcs2303_fright.read_register(24) #current speed feedback
			fleft_speed_feedback = self.rmcs2303_fleft.read_register(24) #current speed feedback
			rright_speed_feedback = self.rmcs2303_rright.read_register(24) #current speed feedback
			rleft_speed_feedback = self.rmcs2303_rleft.read_register(24) #current speed feedback
			#print("Speed feedback : ", right_speed_feedback)
			#print("Velocity : ", right_velocity, left_velocity)
			#print("RPM : ", rpm_right, rpm_left)

		except KeyboardInterrupt:
			if (lin_velocity>=0):
				self.rmcs2303_fright.write_register(2, 256, number_of_decimals=0, functioncode=6, signed=False) # disable fright motor in CW
				self.rmcs2303_fleft.write_register(2, 264, number_of_decimals=0, functioncode=6, signed=False) # disable fleft motor in CCW
				self.rmcs2303_rright.write_register(2, 256, number_of_decimals=0, functioncode=6, signed=False) # disable rrightmotor in CW
				self.rmcs2303_rleft.write_register(2, 264, number_of_decimals=0, functioncode=6, signed=False) # disable rleft motor in CCW
			else:
				self.rmcs2303_fright.write_register(2, 264, number_of_decimals=0, functioncode=6, signed=False) # disable motor in CCW
				self.rmcs2303_fleft.write_register(2, 256, number_of_decimals=0, functioncode=6, signed=False) # disable motor in CW
				self.rmcs2303_rright.write_register(2, 264, number_of_decimals=0, functioncode=6, signed=False) # disable motor in CCW
				self.rmcs2303_rleft.write_register(2, 256, number_of_decimals=0, functioncode=6, signed=False) # disable motor in CW

			self.rmcs2303_fright.write_register(2, 2304, number_of_decimals=0, functioncode=6, signed=False) #restarts the drive
			self.rmcs2303_fright.write_register(2, 2304, number_of_decimals=0, functioncode=6, signed=False) #restarts the drive
			
			self.rmcs2303_fleft.write_register(2, 2304, number_of_decimals=0, functioncode=6, signed=False) #restarts the drive
			self.rmcs2303_fleft.write_register(2, 2304, number_of_decimals=0, functioncode=6, signed=False) #restarts the drive
			
			self.rmcs2303_rright.write_register(2, 2304, number_of_decimals=0, functioncode=6, signed=False) #restarts the drive
			self.rmcs2303_rright.write_register(2, 2304, number_of_decimals=0, functioncode=6, signed=False) #restarts the drive
			
			self.rmcs2303_rleft.write_register(2, 2304, number_of_decimals=0, functioncode=6, signed=False) #restarts the drive
			self.rmcs2303_rleft.write_register(2, 2304, number_of_decimals=0, functioncode=6, signed=False) #restarts the drive

def main(args=None):
	rclpy.init(args=args)

	minimal_modbus_node = MinimalModbusNode()

	rclpy.spin(minimal_modbus_node)

	minimal_modbus_node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
