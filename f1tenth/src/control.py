#!/usr/bin/env python

import rospy
import math
from race.msg import drive_param
from race.msg import pid_input
from sensor_msgs.msg import Imu
from quant import Quaternion

kp = 20
kp2 = 0.1667
kd = 0.09
prev_error = 0.0 
vel_input = 25.0

pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)

def control(data):
	global prev_error
	global vel_input
	global kp
	global kp2
	global kd
	
def gyro(data):
	pose = Quaternion(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
	msg = drive_param()
	#msg.velocity = vel_input - (data.pid_error * kp2)
	msg.velocity = 0
	msg.angle = 15
	pub.publish(msg)

if __name__ == '__main__':
	rospy.init_node('pid_controller', anonymous=True)
	rospy.Subscriber("error", pid_input, control)
	rospy.Subscriber("imu", Imu, gyro)
	rospy.spin()
