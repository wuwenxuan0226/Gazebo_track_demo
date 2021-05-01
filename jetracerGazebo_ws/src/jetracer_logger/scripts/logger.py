#!/usr/bin/env python
import rospy
import csv
import time
from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

steering = 0
throttle = 0
x = 0
y = 0
z = 0
a = 0
b = 0
c = 0

def callback1(data):
    global throttle
    throttle = data.data

def callback2(data):
    global steering
    steering = data.data

def callback3(data):
    global x, y, z, a, b, c
    x = data.linear.x
    y = data.linear.y
    z = data.linear.z
    a = data.angular.x
    b = data.angular.y
    c = data.angular.z



def logger():

    global throttle, steering, x, y, z, a, b, c

    rospy.init_node('logger')
    rate = rospy.Rate(100)

    rospy.Subscriber("/control_throttle", Float64, callback1)
    rospy.Subscriber("/control_steering", Float64, callback2)
    rospy.Subscriber("/arduino", Twist, callback3)

    with open('measurement.csv', 'w', newline='') as file:
        writer = csv.writer(file)

        while not rospy.is_shutdown():
#        str_msg = (throttle, steering, x, y, z, a, b, c)
#        rospy.loginfo(str_msg)
            writer.writerow([throttle, steering, x, y, z, a, b, c])
            time.sleep(0.04) #hope this is millisec

        rate.sleep()


if __name__ == '__main__':
    logger()
