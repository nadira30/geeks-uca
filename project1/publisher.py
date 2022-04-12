#This code determines the actual linear and angular speed using the encoder

from time import sleep
import numpy as np
import rclpy
import serial
from rclpy.node import Node
from geometry_msgs.msg import Twist


wheel_separation= 0.41
radius= 0.04
# init serial comm
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
ser.reset_input_buffer()
cpr= 2240

class botListener(Node):
	def __init__(self):
		super().__init__('bot_listener')

def main(args=None):
	
	"""
	Write code to make your own main function.
	"""
	print('Hi from bot_listener.')
	rclpy.init(args=args)
	node= rclpy.create_node('bot_listerner')
	publisher= node.create_publisher(Twist, 'cmd_vel',1)
	msg= Twist()
	bot_listener= botListener()

	node = rclpy.create_node('minimal_client')

	#initialize the other to zero

	angular = 0.0
	
	while True:
		if ser.in_waiting > 0:
			line = ser.readline()
			#TODO: Strip out every hex/binary elements
			if line == b'\xfe\n' or line == b'\xff\n' or line == b'\off\n' or line =='0' or line =='0\n':

				continue

			#            print(line)
			speeds = line.decode('utf-8').strip()
			#print(speeds)
			s = speeds.split(",")
			#print(type(s[0]))
			left_count= s[0]
			left_count = float(left_count)
			if(len(s) > 1):
				right_count= s[1]
				right_count = float(right_count)
			#left motor speed
			l_speed= left_count*(2*np.pi*radius)/cpr
			#right motor speed
			r_speed= left_count*(2*np.pi*radius)/cpr
			#robot speed
			linear= (l_speed+r_speed)/2
			#robot angular speed
			angular= linear/wheel_separation
			#print the linear and angular speeds of the robot
			print(f'roboot linear speed: {linear}', f'robot angular speed: {angular}')


	rclpy.spin(bot_listener)
	bot_listener.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
