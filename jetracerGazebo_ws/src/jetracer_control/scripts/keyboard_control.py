#!/usr/bin/env python
import getch
import rospy
import math
from std_msgs.msg import Float64

def keys():
    pub_throttle_left = rospy.Publisher('jetracer/joint_back_left_wheel_velocity_controller/command', Float64, queue_size = 10)
    pub_throttle_right = rospy.Publisher('jetracer/joint_back_right_wheel_velocity_controller/command', Float64, queue_size = 10)
    pub_steering_left = rospy.Publisher('jetracer/joint_front_left_axle_steering_controller/command', Float64, queue_size = 10)
    pub_steering_right = rospy.Publisher('jetracer/joint_front_right_axle_steering_controller/command', Float64, queue_size = 10)
    rospy.init_node('gazebo_control')
    #rate = rospy.Rate(100)

    lat = 0
    lon = 0
    scale = 50 #rad/s 1.7m/s r=0.034m
    L = 15 #wheelbase (in cm. all of them)
    R = 50 #turning radius
    W = 15 #track width

    while not rospy.is_shutdown():

        k=ord(getch.getch())# this is used to convert the keypress event in the keyboard or joypad , joystick to a ord value

        if k == 119:
            if lon < 0.9:
                lon += 1
            else:
                lon = 1

        elif k == 115:
            if lon > -0.9:
                lon -= 1
            else:
                lon = -1

        elif k == 100:
            if lat < 0.349:
                lat += 0.08725
            else:
                lat = 0.349

        elif k == 97:
            if lat > -0.349:
                lat -= 0.08725
            else:
                lat = -0.349

        elif k == 32:
            lon = 0
            lat = 0

        lon = round(lon, 5)
        lat = round(lat, 5)
        str = (lon, lat)

        lon_c = lon * scale

        #ackerman
        lat_r = lat
        alpha_left = math.atan((2*L*math.sin(lat_r))/((2*L*math.cos(lat_r))-(W*math.sin(lat_r))))
        alpha_right = math.atan((2*L*math.sin(lat_r))/((2*L*math.cos(lat_r))+(W*math.sin(lat_r))))

        rospy.loginfo(str)

        pub_throttle_left.publish(lon_c)
        pub_throttle_right.publish(lon_c)

        pub_steering_left.publish(-alpha_left)
        pub_steering_right.publish(alpha_right)
        #rate.sleep()

if __name__=='__main__':
    try:
        keys()
    except rospy.ROSInterruptException:
        pass
