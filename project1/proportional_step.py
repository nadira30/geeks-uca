# Read encoder and publish robot speed in ros2 topic with geometry twist 
"""
Test robot's velocity control using PI controller.
This test cathes motors' resonses toward a step signal.
"""
import time
from gpiozero import PhaseEnableRobot, Robot, LED
import serial
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# setup motors
bot = PhaseEnableRobot((24, 12), (23, 13))

# setup serial port
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
ser.reset_input_buffer()

# setup constants
wheel_separation = 0.41
radius = 0.04
K_P = 0.3
# init variables
counter = 0
linear = 0  # robot linear velocity
r_speed = 0
l_speed = 0
left_dutycycle = 0
right_dutycycle = 0
l_list = []
r_list = []
target_speed = 0
cpr=8960

#main loop
class botlistener(Node):
   def __init__(self):
       super().__init__('bot_listen')   

print('Hi from bot')
rclpy.init(args=None)
node= rclpy.create_node('bot_listen')
publisher= node.create_publisher(Twist, 'cmd_vel', 1)
msg= Twist()
bot_listen= botlistener()
node= rclpy.create_node('minimal_client')

# main loop
while counter < 400:  # 4 seconds
    if ser.in_waiting > 0:
        line = ser.readline()
        if b'\xff' in line or b'\xfe' in line:
            continue
        speeds = line.decode('utf-8').rstrip().split(',')
        right_count = float(speeds[0])
        left_count = float(speeds[1])
        l_speed = left_count*2*np.pi*radius/cpr
        r_speed = right_count*2*np.pi*radius/cpr
        l_list.append(l_speed)
        r_list.append(r_speed)
        # set target velocity
        if counter >= 100:  # at 1 sec
            target_speed = msg.linear.x
        # compute error
        left_error = target_speed - l_speed
        right_error = target_speed - r_speed
       
        # compute dutycycle increment
        left_dutycycle_inc = K_P * left_error 
        left_dutycycle += left_dutycycle_inc
        if left_dutycycle > 1:
            left_dutycycle = 1
        elif left_dutycycle < 0:
            left_dutycycle = 0
        right_dutycycle_inc = K_P * right_error
        right_dutycycle += right_dutycycle_inc
        if right_dutycycle > 1:
            right_dutycycle = 1
        elif right_dutycycle < 0:
            right_dutycycle = 0
        # drive motors
        bot.left_motor.forward(left_dutycycle)
        bot.right_motor.forward(right_dutycycle)
        time.sleep(.01)
        #print(left_dutycycle)
        print(right_dutycycle)
        print(
            "---\n",
            f"Left Target Speed: {target_speed}, Left Speed: {l_speed} \n", \
            f"Right Target Speed: {target_speed}, Right Speed: {r_speed}"
        )
        counter += 1

bot.stop()

# plot
step_list = [0]*100 + [target_speed]*300
x = list(range(400))
plt.figure()
plt.subplot(211)
plt.plot(x, step_list, color='tab:orange', linestyle='--')
plt.plot(x, l_list, color='lime')
plt.subplot(212)
plt.plot(x, step_list, color='tab:orange', linestyle='--')
plt.plot(x, r_list, color='red')
plt.savefig('new_kp_1.png')
