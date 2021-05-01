#!/usr/bin/env python3

import rospy
import sys
from std_msgs.msg import Float64
import numpy as np
from jetracer.nvidia_racecar import NvidiaRacecar
import time

### Car parameter definition ###
car = NvidiaRacecar()

car.steering_gain = 0.9 #Limits steering range
car.steering_offset = 0.01

car.throttle_gain = 1

def steer_callback(data):
    car.steering = data.data

def throttle_callback(data):
    car.throttle = -data.data
    #car.throttle = 0
    

rospy.init_node('command_listener_node', anonymous=True)
steer_sub = rospy.Subscriber('steering_control_action', Float64, steer_callback)
throt_sub = rospy.Subscriber('throttle_control_action', Float64, throttle_callback)
rate = rospy.Rate(10)


print('Ready')
def main():
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    car.steering = 0
    car.throttle = 0
    print('Exiting')
    sys.exit()

if __name__=='__main__':
    main()