#!/usr/bin/env python

import rospy
import math, time
from race.msg import drive_param
import rospy, math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

wheelbase = 0.3
kp = 0.1
ki = 0.0
kd = 1.0
compVel = 0.0
prevErr = 0.0
prevtime = 0.0
pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)


def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
    if omega == 0 or v == 0:
        return 0

    radius = v / omega
    return math.atan(wheelbase / radius)


def quaternion_to_euler_angle(w, x, y, z):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    X = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.degrees(math.asin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    Z = math.degrees(math.atan2(t3, t4))

    return X, Y, Z


def cmd_callback(data):
    global pub
    global prevtime
    global prevErr
    v = data.linear.x
    steering = convert_trans_rot_vel_to_steering_angle(v, data.angular.z, wheelbase)





def updateOdom(data):
    try:
        thet = math.atan(data.twist.twist.linear.y / data.twist.twist.linear.x)
    except ZeroDivisionError:
        thet = 0
    q = euler_from_quaternion(
        [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
    diff = thet - q[2]
    vel = math.sqrt(math.pow(data.twist.twist.linear.x, 2) + math.pow(data.twist.twist.linear.y, 2))
    compVel = vel / math.cos(diff)
    currentTime = time.clock()
    err = v - compVel
    output = (err * kp) + ((err - prevErr) / (currentTime - prevtime))
    prevtime = currentTime
    prevErr = err
    msg = drive_param()
    msg.velocity = output
    msg.angle = steering
    pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('Control', anonymous=True)
    rospy.get_param('wheelbase', 0.32385)
    rospy.Subscriber("cmd_vel", Twist, cmd_callback)
    rospy.Subscriber("odometry/filtered", Odometry, updateOdom)
    rospy.spin()
