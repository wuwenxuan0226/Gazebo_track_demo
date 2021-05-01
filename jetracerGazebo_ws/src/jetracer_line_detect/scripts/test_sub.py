#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

steering = 0
throttle = 0

def callback1(data):
    global steering
    steering=data.data
    rospy.loginfo(rospy.get_caller_id() + " Received steering control is %s", steering)

def callback2(data):
    global steering
    throttle=data.data
    rospy.loginfo(rospy.get_caller_id() + " Received throttle control is %s", throttle)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('receive_control', anonymous=True)

    rospy.Subscriber("control_steering", Float64, callback1)
    rospy.Subscriber("control_throttle", Float64, callback2)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()