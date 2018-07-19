#!/usr/bin/env python

import rospy
import math
from race.msg import drive_param
from race.msg import pid_input
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from quant import Quaternion

wheelbase = 0.25

pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)


def recieveCommand(data):
	steering_angle = 0
	omega = data.angular.z
	v = data.linear.x
	if v == 0:
		steering_angle = 0
	elif omega == 0:
		steering_angle = 0
	else:
		radius = v/omega
		steering_angle = math.atan(wheelbase/radius)
		if data.linear.x < 0:
			steering_angle *= -1
	print(math.degrees(steering_angle))
	msg = drive_param()
	msg.velocity = data.linear.x
	msg.angle = steering_angle
	pub.publish(msg)

if __name__ == '__main__':
	rospy.init_node('pid_controller', anonymous=True)
	rospy.Subscriber("cmd_vel", Twist, recieveCommand)
	rospy.spin()
