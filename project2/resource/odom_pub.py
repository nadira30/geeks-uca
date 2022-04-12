
import rclpy
from rclpy.node import Node
import tf_transformations
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry

import numpy as np


class MinimalPublisher(Node): 
	def __init__(self):
		super.__init__('odom_publisher')
		#tf broadcaster
		self.odom_broadcaster = TransformBroadcaster(self)
		#robt velocity subscriber
		self.vel_sub = self.create_subscription(Twist, '/cmd_vel', self.sub_callback,1)
		self.vel_sub
		self.odom_pub = self.create_publisher((Odometry), '/odom', 1)
		#timer for publishing tfs and topics
		self.timer_callback = self.create_timer(0.1, self.timer_callback)
		
		# can define variables
		self.x = 0.0
		self.y = 0.0
		self.th = 0.0
		self.linearX = 0.0
		self.angularZ = 0.0
		self.curr_time = self.get_clock().now() #convert from nano seconds to seconds 
		self.prev_time = self.get_clock().now()
		
	def sub_callback(self,msg):
		
		#Callback function for subscriber
		self.linearX = msg.linear.x
		self.angularZ = msg.angular.z
		
	def timer_callback(self):
		"""
		call publisher
		use motor direction 
		"""
		self.curr_time = self.get_clock().now()
		dt = (self.curr_time - self.prev_time)*10**(-9)

		#Set the position 
		delta_x= (self.linearX * np.cos(self.th)) * dt
		delta_y= (self.linearX * np.sin(self.th)) * dt
		delta_th = self.angularZ * dt
		#increment x, y, th 
		self.x += delta_x 
		self.y += delta_y 
		self.th += delta_th
		# quaternion 
		q = tf_transformations.quaternion_about_axis(self.th, (0,0,1))

		#Odom transformation
		trans = TransformStamped()
		trans.header.stamp = self.curr_time.to_msg()
		trans.header.frame_id = "odom"
		trans.child_frame_id = "base_link"
		trans.transform.translation.x = self.x 
		trans.transform.translation.y = self.y
		trans.transform.translation.z = 0.0
		trans.transform.rotation.x = q[0]
		trans.transform.rotation.y = q[1]
		trans.transform.rotation.z = q[2]
		trans.transform.rotation.w = q[3]
		self.odom_broadcaster.sendTransform(trans)

		# Odom topic
		msg = Odometry()
		msg.header.stamp = self.curr_time.to_msg()
		msg.header.frame_id = "odom"
		msg.child_frame_id = "base_link"
		msg.pose.pose.position.x = self.x 
		msg.pose.pose.position.y = self.y
		msg.pose.pose.orientation.x = q[0]
		msg.pose.pose.orientation.y = q[1]
		msg.pose.pose.orientation.z = q[2]
		msg.pose.pose.orientation.w = q[3]
		msg.twist.linear.x= self.linearX
		msg.twist.angular.z = self.angularZ
		self.odom_pub.publish(msg)
		self.get_logger().debug('fPublishing : {msg}')
		self.pre_time = self.curr_time
		
def main(args = None):
	rclpy.init(args = args)
	
	odom_publisher = MinimalPublisher()

	rclpy.spin(odom_publisher)

	# destroy node
	odom_publisher.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
