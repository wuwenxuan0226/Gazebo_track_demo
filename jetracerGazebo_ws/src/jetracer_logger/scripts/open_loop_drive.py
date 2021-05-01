#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Float64

def talker():
    pub_throttle = rospy.Publisher('throttle_control_action', Float64, queue_size=10)
    pub_steering = rospy.Publisher('steering_control_action', Float64, queue_size=10)
    rospy.init_node('Open_loop_driver')
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        time.sleep(1) # Delays for 5 seconds. You can also use a float value
        rospy.loginfo('Starting Loop!')

        pub_throttle.publish(0)
        pub_steering.publish(0)
        rospy.loginfo('start')
        time.sleep(0.99)

        pub_throttle.publish(0)
        pub_steering.publish(0)
        rospy.loginfo('start(end)')
        time.sleep(0.01)

        pub_throttle.publish(0)
        pub_steering.publish(1.0)
        rospy.loginfo('steering right')
        time.sleep(0.49)

        pub_throttle.publish(0)
        pub_steering.publish(1.0)
        rospy.loginfo('steering right(end)')
        time.sleep(0.01)

        pub_throttle.publish(1.0)
        pub_steering.publish(1.0)
        rospy.loginfo('full throttle')
        time.sleep(5.99)

        pub_throttle.publish(1.0)
        pub_steering.publish(1.0)
        rospy.loginfo('full throttle(end)')
        time.sleep(0.01)

        pub_throttle.publish(0.0)
        pub_steering.publish(1.0)
        rospy.loginfo('throttle 0')
        time.sleep(0.99)

        pub_throttle.publish(0.0)
        pub_steering.publish(1.0)
        rospy.loginfo('throttle 0(end)')
        time.sleep(0.01)

        pub_throttle.publish(0)
        pub_steering.publish(0)
        rospy.loginfo('steering 0')
        time.sleep(13.99)

        pub_throttle.publish(0)
        pub_steering.publish(0)
        rospy.loginfo('steering 0(end)')
        time.sleep(0.01)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
