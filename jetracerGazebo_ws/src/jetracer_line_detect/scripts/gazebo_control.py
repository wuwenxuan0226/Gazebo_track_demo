#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float64
from std_msgs.msg import String

longitudinal = 0
lateral = 0

scale =  56.176 #scaling for acceleration
L = 15 #wheelbase (in cm. all of them)
R = 50 #turning radius
W = 15 #track width


def callback1(data):
    global longitudinal
    longitudinal = data.data


def callback2(data):
    global lateral
    lateral =  data.data


def gazebo_motor_control():

    global longitudinal, lateral

    throttle = 0.0
    steering = 0.0

    pub_throttle_left = rospy.Publisher('/jetracer/joint_back_left_wheel_velocity_controller/command', Float64, queue_size = 4)
    pub_throttle_right = rospy.Publisher('/jetracer/joint_back_right_wheel_velocity_controller/command', Float64, queue_size = 4)
    pub_steering_left = rospy.Publisher('/jetracer/joint_front_left_axle_steering_controller/command', Float64, queue_size = 4)
    pub_steering_right = rospy.Publisher('/jetracer/joint_front_right_axle_steering_controller/command', Float64, queue_size = 4)

    rospy.Subscriber("/throttle_control_action", Float64, callback1)
    rospy.Subscriber("/steering_control_action", Float64, callback2)



    rospy.init_node('gazebo_motor_control')
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

#throttle
        throttle = longitudinal * scale

        pub_throttle_left.publish(throttle)
        pub_throttle_right.publish(throttle)

#steering
        lat_r = lateral * 0.261799
        alpha_left = math.atan((2*L*math.sin(lat_r))/((2*L*math.cos(lat_r))-(W*math.sin(lat_r))))
        alpha_right = math.atan((2*L*math.sin(lat_r))/((2*L*math.cos(lat_r))+(W*math.sin(lat_r))))
        pub_steering_left.publish(-alpha_left)
        pub_steering_right.publish(alpha_right)

#        str = (alpha_left, throttle, alpha_right)
#        rospy.loginfo(str)

        rate.sleep()

if __name__ == '__main__':
    try:
        gazebo_motor_control()
    except rospy.ROSInterruptException:
        pass
