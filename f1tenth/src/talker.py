#!/usr/bin/env python

import rospy
from race.msg import drive_values
from race.msg import drive_param
from std_msgs.msg import Bool

pub = rospy.Publisher('drive_pwm', drive_values, queue_size=10)
em_pub = rospy.Publisher('eStop', Bool, queue_size=10)
control_pub = rospy.Publisher('controlOverride', Bool, queue_size=10)

# function to map from one range to another, similar to arduino
def arduino_map(x, in_min, in_max, out_min, out_max):
    val = x
    if (val < in_min):
        val = in_min
    elif (val > in_max):
        val = in_max
    return (val - in_min) * (out_max - out_min) // (in_max - in_min) + out_min


# callback function on occurance of drive parameters(angle & velocity)
def callback(data):
    velocity = data.velocity
    angle = data.angle

    pwm1 = arduino_map(velocity, -1.7, 1.7, 6554, 13108);
    pwm2 = arduino_map(-angle, -25, 25, 6554, 12241);
    vel = pwm1
    if pwm1 > 9830:
        if(pwm1 - 9830 < 305 and pwm1 - 9830 > 75):
            vel = 9830 + 305
    else:
        if(9830 - pwm1 < 305 and 9830 - pwm1 > 75):
            vel = 9830 - 305
    msg = drive_values()
    msg.pwm_drive = vel
    msg.pwm_angle = pwm2
    pub.publish(msg)


def talker():
    rospy.init_node('serial_talker', anonymous=True)
    em_pub.publish(False)
    control_pub.publish(False)
    rospy.Subscriber("drive_parameters", drive_param, callback)
    rospy.spin()


if __name__ == '__main__':
    rospy.loginfo("Serial Talker Initialized")
    talker()
    em_pub.publish(True)
