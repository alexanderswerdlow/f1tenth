#!/usr/bin/env python

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from race.msg import pid_input
import matplotlib.pyplot as plt
import time
from operator import add
desired_trajectory = 1
vel = 30

pub = rospy.Publisher('error', pid_input, queue_size=10)
def rect(r, theta):
    x = r * math.cos(theta)
    y = r * math.sin(theta)
    return x, y

def callback(data):
    new_data = data.ranges
    half = len(new_data)/2
    side1 = new_data[:half]
    side2 = new_data[half:]
    wid = list(map(add, side1, side2))
    wall = wid.index(min(wid))

    #print(wall, wid[wall], (wall, side1[wall]), (wall + 180, side2[wall]))

    xarr = []
    yarr = []
    thet = 0
    for i in data.ranges:
        thet += ((math.pi * 2) / 360)
        if i < data.range_max and i > data.range_min:
            point = rect(i, thet)
            xarr.append(point[0])
            yarr.append(point[1])

    p1 = rect(side1[wall], ((math.pi * 2) / 360) * wall)
    p2 = rect(side2[wall], (((math.pi * 2) / 360) * wall) + math.pi)
    try:
        m1 = 1/(-(p1[1]/p1[0]))
        m2 = 1/(-(p2[1]/p2[0]))
        c1 = p1[1] - m1 * p1[0]
        c2 = p2[1] - m2 * p2[0]
        x = np.linspace(-16, 16, 1000)

        y = m1 * x + c1
        ya = m2 * x + c2
        """ global counter
        if counter % 10 == 0:
            plt.clf()
            plt.plot(x,y)
            plt.plot(x,ya)
            plt.scatter(xarr, yarr)
            plt.scatter(0,0)
            plt.scatter(p1[0],p1[1],marker="x",s=1000)
            plt.scatter(p2[0],p2[1],marker="x",s=1000)
            plt.axis([-16, 16,-16,16])
            plt.draw()
            plt.pause(0.00000000001)
        counter += 1 """
        eq = (math.sqrt(((p1[0] + p2[0])**2) + ((p2[1] + p1[1])**2))) / 2.0
        er = eq - math.sqrt(((p1[0])**2) + ((p1[1])**2))
        msg = pid_input()
        if math.isnan(float(er)):
            er = 0
        msg.pid_error = float(er)
        msg.pid_vel = float(10)
        pub.publish(msg)
    except ZeroDivisionError:
        pass

if __name__ == '__main__':
    counter = 0
    ocount = 1000
    print("Wall Finder Started")
    rospy.init_node('dist_finder', anonymous=True)
    rospy.Subscriber("scan", LaserScan, callback)
    plt.ion()
    plt.show()
    rospy.spin()
