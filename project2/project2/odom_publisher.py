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
ser = serial.Serial("/dev/ttyACM0", 9600, timeout=1)
ser.reset_input_buffer()

# setup constants
RADIUS = 0.04
CPR = 8960
WHEEL_SEP = 0.41
K_P = 0.2
# init variables
counter = 0
linear = 0  # robot linear velocity
left_dutycycle = 0
right_dutycycle = 0


class OdomPublisher(Node):
    def __init__(self):
        super().__init__("odom_publisher")
        # tf broadcaster
        self.odom_broadcaster = TransformBroadcaster(self)
        # robt velocity subscriber
        self.vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self.velsub_callback, 1
        )
        self.vel_sub
        self.odom_pub = self.create_publisher(Odometry, "/odom", 1)
        # timer for publishing tfs and topics
        self.timer1 = self.create_timer(0.01, self.actvel_cb)
        self.timer2 = self.create_timer(0.01, self.odom_cb)
        # define variables
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.linearX = 0.0
        self.angularZ = 0.0
        self.targetX = 0.0
        self.targetLX = 0.0
        self.targetRX = 0.0
        self.targetZ = 0.0
        self.Rspeed = 0.0
        self.Lspeed = 0.0
        self.left_dutycycle = 0.0
        self.right_dutycycle = 0.0
        self.curr_time = self.get_clock().now()
        self.prev_time = self.get_clock().now()

    def velsub_callback(self, msg):
        self.targetX = msg.linear.x
        self.targetZ = msg.angular.z

    def actvel_cb(self):

        if ser.in_waiting > 0:
            line = ser.readline()
            # if b'\xff' in line or b'\xfe' in line:
            # continue
            if b"\xff" not in line or b"\xfe" not in line:
                speeds = line.decode("utf-8").rstrip().split(",")
                if len(speeds) == 2:
                    right_count = float(speeds[0])
                    left_count = float(speeds[1])

        self.Lspeed = left_count / CPR * 2 * np.pi * RADIUS
        self.Rspeed = right_count / CPR * 2 * np.pi * RADIUS
        # set target velocity
        self.linearX = (self.Rspeed + self.Lspeed) / 2
        self.angularZ = (self.Rspeed - self.Lspeed) / WHEEL_SEP
        print(f"target vel: {(self.targetX, self.targetZ)}, actual vel: {(self.linearX, self.angularZ)}")
        # compute error
        self.targetLX = self.targetX - ((self.targetZ * WHEEL_SEP) / 2)
        self.targetRX = self.targetX + ((self.targetZ * WHEEL_SEP) / 2)
        left_error = self.targetLX - self.Lspeed
        right_error = self.targetRX - self.Rspeed
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

    def odom_cb(self):
        """
        pub odom
        """
        self.curr_time = self.get_clock().now()
        dt = (self.curr_time - self.prev_time).nanoseconds * 1e-9

        # Set the position
        delta_x = (self.linearX * np.cos(self.th)) * dt
        delta_y = (self.linearX * np.sin(self.th)) * dt
        delta_th = self.angularZ * dt
        # increment x, y, th
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        # quaternion
        q = tf_transformations.quaternion_about_axis(self.th, (0, 0, 1))

        # Odom transformation
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
        msg.twist.twist.linear.x = self.linearX
        msg.twist.twist.angular.z = self.angularZ
        self.odom_pub.publish(msg)
        # self.odom_pub.publish(msg1)
        # self.get_logger().debug('fPublishing : {msg1}')
        self.get_logger().debug(f"Publish: {msg}")
        self.pre_time = self.curr_time


def main(args=None):
    rclpy.init(args=args)

    odom_publisher = OdomPublisher()

    rclpy.spin(odom_publisher)

    # destroy node
    odom_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
