#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

def talker():
    #Pulisher (topic) name
    pub = rospy.Publisher('control', Float64, queue_size=5) 
    #Initial a node name       
    rospy.init_node('image_jetracer_control',anonymous = True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        send_str = rospy.get_time()
        rospy.loginfo("Send control is: %s", send_str)
        pub.publish(send_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass