#!/usr/bin/env python

import rospy
import math
from race.msg import drive_param
import rospy, math
from geometry_msgs.msg import Twist

wheelbase = 0;

pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)


def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
    if omega == 0 or v == 0:
        return 0

    radius = v / omega
    return math.atan(wheelbase / radius)


def cmd_callback(data):
    global pub

    v = data.linear.x
    steering = convert_trans_rot_vel_to_steering_angle(v, data.angular.z, wheelbase)

    msg = drive_param()
    msg.velocity = v
    msg.angle = steering
    pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('Control', anonymous=True)
    rospy.get_param('wheelbase', 0.32385)
    rospy.Subscriber("cmd_vel", Twist, cmd_callback)
    rospy.spin()
