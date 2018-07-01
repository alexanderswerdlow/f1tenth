#!/usr/bin/env python

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from race.msg import pid_input
import matplotlib.pyplot as plt
import time
from operator import add

def rect(r, theta):
    x = r * math.cos(theta)
    y = r * math.sin(theta)
    return x, y

def callback(data):
    """ global ocount
    ocount += 1000
    new_data = []
    for i in range(0,360):
        if data.ranges[i] < data.range_max and data.ranges[i] > data.range_min:
            new_data.append(ocount)
        else:
            new_data.append(data.ranges[i]) """
    new_data = data.ranges
    half = len(new_data)/2
    side1 = new_data[:half]
    side2 = new_data[half:]
    wid = list(map(add, side1, side2))
    wall = wid.index(min(wid))

    print(wall, wid[wall], (wall, side1[wall]), (wall + 180, side2[wall]))

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
        global counter
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
        counter += 1
    except ZeroDivisionError:
        pass
    

    
    #print((side1[wall], ((math.pi * 2) / 360) * wall), (side2[wall], (((math.pi * 2) / 360) * wall) + math.pi))
    #print((side1[wall], ((math.pi * 2) / 360) * wall), (side2[wall], ((math.pi * 2) / 360) * wall * 2))

    #print(m1, m2)
    #print(p1, p2)

    

    

    #plt.scatter(p1[0],p1[1],marker='+')
    #plt.scatter(p2[0],p2[1],marker='+')
    
    

if __name__ == '__main__':
    counter = 0
    ocount = 1000
    print("Robot")
    rospy.init_node('dist_finder', anonymous=True)
    rospy.Subscriber("scan", LaserScan, callback)
    plt.ion()
    plt.show()
    rospy.spin()
