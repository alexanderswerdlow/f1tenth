#!/usr/bin/env python

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
from race.msg import pid_input
import time
from operator import add

def rect(r, theta):
    x = r * math.cos(theta)
    y = r * math.sin(theta)
    return x, y


def callback(data):
	half = len(data.ranges)/2
	side1 = data.ranges[:half]
	side2 = data.ranges[half:]
	wid = list(map(add, side1, side2))
	wall = wid.index(min(wid))

	#print(wid[wall], (wall, side1[wall]), (wall + 180, side2[wall]))

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

	m1 = 1/(-(p1[1]/p1[0]))

	m2 = 1/(-(p2[1]/p2[0]))

	c1 = side1[wall] / math.cos(((math.pi * 2) / 360) * wall)

	c2 = side2[wall] / math.cos((((math.pi * 2) / 360) * wall) + math.pi)
	#print((side1[wall], ((math.pi * 2) / 360) * wall), (side2[wall], (((math.pi * 2) / 360) * wall) + math.pi))
	#print((side1[wall], ((math.pi * 2) / 360) * wall), (side2[wall], ((math.pi * 2) / 360) * wall * 2))

	print(m1, m2)
	print(c1, c2)

	x = np.linspace(-5, 5, 1000)

	y = m1 * x + c1
	ya = m2* x + c2

	#plt.plot(x,y)
	#plt.plot(x,ya)
	#plt.scatter(xarr,yarr)

	plt.plot([1,2,3,4])
	plt.ylabel('some numbers')
	plt.show()




if __name__ == '__main__':
	print("Laser node started")
	rospy.init_node('dist_finder', anonymous=True)
	rospy.Subscriber("scan", LaserScan, callback)
	rospy.spin()
