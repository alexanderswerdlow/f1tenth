#!/usr/bin/env python

import rospy
from race.msg import drive_flags
import curses


stdscr = curses.initscr()
curses.cbreak()
stdscr.keypad(1)
rospy.init_node('keyboard_talker', anonymous=True)
em_pub = rospy.Publisher('driveFlags', drive_flags, queue_size=100)
stdscr.refresh()

key = ''
while key != ord('q'):
    key = stdscr.getch()
    stdscr.refresh()
    if key == curses.KEY_UP:
        temp = drive_flags()
        em_pub.publish(temp)
temp = drive_flags()
temp.controlOverride = True
em_pub.publish(temp)
stdscr.addstr(5, 20, "STOP!!!!!")
curses.endwin()