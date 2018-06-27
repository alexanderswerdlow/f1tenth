#!/usr/bin/env python

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
from race.msg import pid_input
import time
from operator import add

def linearReg(x, y):
    A = np.vstack([x, np.ones(len(x))]).T
    m, c = np.linalg.lstsq(A, y)[0]
    return(m, c)


def rect(r, theta):
    x = r * math.cos(theta)
    y = r * math.sin(theta)
    return x, y


def callback(data):
    a = np.split(data.ranges, 3)
    side1 = a[0]
    side2 = a[1]
    wid = list(map(add, side1, side2))
    wall = wid.index(min(wid))
    print(wid[wall], wall, wall + 180)

    """ xarr = []
    yarr = []
    thet = 0
    for i in data.ranges:
        thet += ((math.pi * 2) / 360)
        if i < data.range_max and i > data.range_min:
            point = rect(i, thet)
            xarr.append(point[0])
            yarr.append(point[1])

    
	m, c = linearReg(xarr, yarr)
	a = 0.95 / math.cos(math.atan(c))

	x = np.linspace(-5, 5, 1000)

	y = m * x + (c - a)
	ya = m * x + (c + a)

    plt.plot(x,y)
    plt.plot(x,ya)
    plt.scatter(xarr,yarr)
    plt.show() """




if __name__ == '__main__':
	print("Laser node started")
	rospy.init_node('dist_finder', anonymous=True)
	rospy.Subscriber("scan", LaserScan, callback)
	rospy.spin()
