#!/usr/bin/env python

import rospy
import math
import rospy, math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
r = 2.5;
1.5
pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)

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
    q = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
    z1 = q[2]
    x = (5 * math.cos(z1)) + data.pose.pose.position.x
    y = (5 * math.sin(z1)) + data.pose.pose.position.y

    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'map'
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = 0
    msg.pose.orientation = data.pose.pose.orientation

    pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('setpointgen', anonymous=True)
    rospy.Subscriber("odometry/filtered", Odometry, cmd_callback)
    rospy.spin()
