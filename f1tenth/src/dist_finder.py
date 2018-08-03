#!/usr/bin/env python

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from obstacle_detector.msg import Obstacles
from obstacle_detector.msg import SegmentObstacle
from obstacle_detector.msg import CircleObstacle
from geometry_msgs.msg import Point
from race.msg import pid_input
import matplotlib.pyplot as plt
import time
from operator import add

desired_trajectory = 1
vel = 30

pub = rospy.Publisher('error', pid_input, queue_size=10)


def callback(data):
    dist1 = 0
    dist2 = 0
    slope1 = 0
    slope2 = 0
    c1 = 0
    c2 = 0
    for x in data.segments:
        dist = math.sqrt(math.pow(x.first_point.x - x.last_point.x, 2) + math.pow(x.first_point.y - x.last_point.y, 2))
        if dist > dist1:
            dist1 = dist
            slope1 = (x.first_point.y - x.last_point.y) / (x.first_point.x - x.last_point.x)
            c1 = x.first_point.y + (slope1 * x.first_point.x)
        elif dist > dist2:
            dist2 = dist
            slope2 = (x.first_point.y - x.last_point.y) / (x.first_point.x - x.last_point.x)
            c2 = x.first_point.y + (slope2 * x.first_point.x)
    x = np.linspace(-16, 16, 1000)

    y = slope1 * x + c1
    ya = slope2 * x + c2
    global counter
    if counter % 10 == 0:
        plt.clf()
        plt.plot(x, y)
        plt.plot(x, ya)
        plt.axis([-16, 16, -16, 16])
        plt.draw()
        plt.pause(0.00000000001)
    counter += 1
    msg = pid_input()
    msg.pid_error = 0
    msg.pid_vel = 0
    pub.publish(msg)


if __name__ == '__main__':
    counter = 0
    ocount = 1000
    print("Started Finding Goal")
    rospy.init_node('dist_finder', anonymous=True)
    rospy.Subscriber("raw_obstacles", Obstacles, callback)
    plt.ion()
    plt.show()
    rospy.spin()
