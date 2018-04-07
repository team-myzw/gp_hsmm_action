#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Empty

if __name__=="__main__":  
    rospy.init_node("tf_pulse_publisher")
    pub = rospy.Publisher("tf_pulse",Empty,queue_size=10)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        e = Empty()
        pub.publish(e)
        r.sleep()
