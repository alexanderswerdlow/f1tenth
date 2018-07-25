#!/usr/bin/env python

import rospy
import math
from obstacle_detector.msg import Obstacles
from obstacle_detector.msg import SegmentObstacle
from obstacle_detector.msg import CircleObstacle
from geometry_msgs.msg import Point
from race.msg import pid_input

pub = rospy.Publisher('error', pid_input, queue_size=10)



def callback(data):
    dist1 = 0
    dist2 = 0
    slope1 = 0
    slope2 = 0
    c1 = 0
    c2 = 0
    for x in data.segments:
        dist = math.sqrt(math.pow(x.first_point.x - x.last_point.x,2) + math.pow(x.first_point.y - x.last_point.y,2))
        if dist > dist1:
            dist1 = dist
            slope1 = (x.first_point.y - x.last_point.y) / (x.first_point.x - x.last_point.x)
            c1 = x.first_point.y + (slope1 * x.first_point.x)
        elif dist > dist2:
            dist2 = dist
            slope2 = (x.first_point.y - x.last_point.y) / (x.first_point.x - x.last_point.x)
            c2 = x.first_point.y + (slope2 * x.first_point.x)
    msg = pid_input()
    msg.pid_error = error
    msg.pid_vel = vel
    pub.publish(msg)


if __name__ == '__main__':
    print("Started Finding Goal")
    rospy.init_node('dist_finder',anonymous = True)
    rospy.Subscriber("raw_obstacles",Obstacles,callback)
    rospy.spin()