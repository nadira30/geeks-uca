
# reading from the encoder the speed and including PID control
from gpiozero import LED, PhaseEnableRobot
import serial
import rclpy
from rclpy.node import Node
import tf_transformations
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import numpy as np

# setup motors
bot = PhaseEnableRobot((24, 12), (23, 13))

# setup serial port
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
ser.reset_input_buffer()

# setup constants
radius = 0.04
K_P = 0.2
# init variables
counter = 0
linear = 0  # robot linear velocity
left_dutycycle = 0
right_dutycycle = 0
cpr=8960
wheel_sep = 0.41


class MinimalPublisher(Node): 
	def __init__(self):
		super().__init__('odom_publisher')
		#tf broadcaster
		self.odom_broadcaster = TransformBroadcaster(self)
		#robt velocity subscriber
		self.vel_sub = self.create_subscription(Twist, '/cmd_vel', self.sub_callback,1)
		self.vel_sub
		self.odom_pub = self.create_publisher((Odometry), '/odom', 1)
		#timer for publishing tfs and topics
		self.timer_callback = self.create_timer(0.01, self.timer_callback_cb)
		self.odom_callback = self.create_timer(0.01, self.odom_callback_cb)
		#self.actual_speed = self.create_publisher((Twist), 'actual_speed',1)

                # can define variables
		self.x = 0.0
		self.y = 0.0
		self.th = 0.0
		self.linearX = 0.0
		self.angularZ = 0.0
		self.targetX= 0.0
		self.targetLX= 0.0
		self.targetRX= 0.0
		self.targetZ= 0.0
		self.Rspeed = 0.0
		self.Lspeed = 0.0
		self.left_dutycycle = 0.0
		self.right_dutycycle= 0.0
		self.curr_time = self.get_clock().now() #convert from nano seconds to seconds 
		self.prev_time = self.get_clock().now()
		
	def sub_callback(self,msg):
		self.targetX= msg.linear.x
		self.targetZ= msg.angular.z
		
	def odom_callback_cb(self):
		
		if ser.in_waiting > 0:
			line = ser.readline()
			#if b'\xff' in line or b'\xfe' in line:
				#continue
			if not b'\off' in line or  not b'\xfe' in line:
				speeds = line.decode('utf-8').rstrip().split(',')
				right_count = float(speeds[0])
				if(len(speeds) > 1):
					left_count = float(speeds[1])
				
				self.Lspeed = left_count*2*np.pi*radius/cpr
				self.Rspeed = right_count*2*np.pi*radius/cpr
				# set target velocity
				self.linearX = (self.Rspeed+self.Lspeed)/2
				self.angularZ=(self.Lspeed+self.Rspeed)/wheel_sep
		msg = Twist()
		# compute error
		self.targetLX= self.targetX - ((self.targetZ*wheel_sep)/2)
		self.targetRX= self.targetX + ((self.targetZ*wheel_sep)/2)
		left_error = self.targetLX - self.Lspeed
		right_error = self.targetRX - self.Rspeed
		print(left_error, right_error)
		# compute dutycycle increment
		
		left_dutycycle_inc = K_P * left_error 
		self.left_dutycycle += left_dutycycle_inc
		if self.left_dutycycle > 1:
			self.left_dutycycle = 1
		elif self.left_dutycycle < 0:
			self.left_dutycycle = 0
		right_dutycycle_inc = K_P * right_error
		self.right_dutycycle += right_dutycycle_inc
		if self.right_dutycycle > 1:
			self.right_dutycycle = 1
		elif self.right_dutycycle < 0:
			self.right_dutycycle = 0
			
		# drive motors
		bot.left_motor.forward(self.left_dutycycle)
		bot.right_motor.forward(self.right_dutycycle)
		
		#print(left_dutycycle)
		#print(right_dutycycle)
		print(
			"---\n",
			f"Left Target Speed: {self.targetLX}, Left Speed: {self.Lspeed} \n", \
			f"Right Target Speed: {self.targetRX}, Right Speed: {self.Rspeed}"
		)
	def timer_callback_cb(self):
		"""
		call publisher
		use motor direction 
		"""
				
		self.curr_time = self.get_clock().now()
		
		dt =(self.curr_time - self.prev_time).nanoseconds*10**(-9)

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
		msg1 = Twist()
		msg.header.stamp = self.curr_time.to_msg()
		msg.header.frame_id = "odom"
		msg.child_frame_id = "base_link"
		msg.pose.pose.position.x = self.x 
		msg.pose.pose.position.y = self.y
		msg.pose.pose.orientation.x = q[0]
		msg.pose.pose.orientation.y = q[1]
		msg.pose.pose.orientation.z = q[2]
		msg.pose.pose.orientation.w = q[3]
		msg.twist.twist.linear.x= msg1.linear.x
		msg.twist.twist.angular.z = msg1.angular.z
		self.odom_pub.publish(msg)
		#self.odom_pub.publish(msg1)
		#self.get_logger().debug('fPublishing : {msg1}')
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
