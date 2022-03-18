#Subscriber to the geometry twist msg for the remote controlling of the robot
from gpiozero import LED, PhaseEnableRobot
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from time import time
import numpy as np


# init motor
#en_left = LED(22)
#en_right = LED(23)
bot = PhaseEnableRobot(left=(24,12),right=(23,13))

def chatter_callback(msg):
	
	global g_node
	g_node.get_logger().info('I heard linear x = : "%s"' % msg.linear.x)
	g_node.get_logger().info('I heard angular z = : "%s"' % msg.angular.z) 
	t=time()
	
	if abs(msg.linear.x)>1:
		msg.linear.x= 1.0
	if abs(msg.angular.z)>1:
		msg.angular.z=1.0
		
	if msg.linear.x>0:
		bot.forward(abs(msg.linear.x))
	if msg.linear.x<0:
		bot.backward(abs(msg.linear.x))
	if msg.angular.z>0:
		bot.right(abs(msg.angular.z))
	if msg.angular.z<0:
		bot.left(abs(msg.angular.z))
	if msg.angular.z==0 and msg.linear.x==0:
		bot.stop()

	
    
def main(args=None):
	print('Hi from robot_driver')
	global g_node
	rclpy.init(args=args)

	g_node = rclpy.create_node('minimal_subscriber')

	subscription = g_node.create_subscription(Twist,'cmd_vel',chatter_callback,1)
	subscription  # prevent unused variable warning

	while rclpy.ok():
		rclpy.spin_once(g_node)
	

	g_node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()

