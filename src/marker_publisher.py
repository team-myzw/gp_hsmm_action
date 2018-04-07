#!/usr/bin/env python
# -*- coding: sjis -*-

#import os
import rospy
import numpy as np
from geometry_msgs.msg import TransformStamped
from multiprocessing import Process
import os
#import matplotlib.pyplot as plt
import tf2_ros

NAME= "exp"
BASE= "odom"
TIME = 3.0
XDIS = 1.5
XMIN = 0.1
XD=1.0
YDIS= 0.5
OBJECTS=["apple_01","banana_01","doll_bear_01","doll_dog_01","doll_rabbit_01","cannedjuice_350ml_01","petbottle_2l_empty_c01_01","petbottle_500ml_full_c01_01"]
OBJIDS=["exp_fruit_0","exp_fruit_1","exp_doll_2","exp_doll_3","exp_doll_4","exp_drink_5","exp_drink_6","exp_drink_7"]
EQUIPS= ["open_button_01","table_01","table_02"]
EQUIPIDS=["exp_button_8","exp_table_9","exp_table_10"]
PUB = ["table_01","table_02"]
COM = "rosrun tf2_ros static_transform_publisher {0:f} {1:f} {2:f} {3:f} {4:f} {5:f} {6:f} odom {7:s}"
POSES=[[6.31032297185,-2.10130905101,0.781365848114,0.724969557077,-0.0580177775819,0.683192470186,0.0655829816023],
       [-1.57300008779,-6.02897084933,0.255140638422,0.435133483748,0.51351328672,0.563307097678,-0.479216098842]]
class MarkerPublisher(object):
    def __init__(self):
        self.buf = tf2_ros.Buffer()
        self.lis = tf2_ros.TransformListener(self.buf)
        self._broadcaster = tf2_ros.TransformBroadcaster()       
        self.stamps = []
        for p in range(len(PUB)):
            cm = POSES[p]
            command = COM.format(cm[0],cm[1],cm[2],cm[3],cm[4],cm[5],cm[6],PUB[p])
            rb = Process(target=os.system, args=[command])
            rb.start()
        rospy.sleep(5.0)
        rospy.loginfo("ok")            
        self.rate = rospy.Rate(10)
        
        
    def broadcast(self,ts,name):
        """
        tf_stampを発行する関数
        param TransformStamped tf_stamp: tfのRosMessage        
        """    
        ts.child_frame_id = name
        ts.header.stamp = rospy.Time.now()
        self._broadcaster.sendTransform(ts)

    def set_stamp(self, pose,name="target_object"):
        ts = TransformStamped()
        ts.header.frame_id = "odom"
        ts.child_frame_id = name
        ts.transform.translation.x = pose[0]
        ts.transform.translation.y = pose[1]
        ts.transform.translation.z = pose[2]
        ts.transform.rotation.x = pose[3]
        ts.transform.rotation.y = pose[4]
        ts.transform.rotation.z = pose[5]
        ts.transform.rotation.w = pose[6]
        ts.header.stamp = rospy.Time.now()
        return ts
        
    def run(self):
        while not rospy.is_shutdown():            
            ts = []
            ds = []
            for o in OBJECTS:
                t = self.buf.lookup_transform("odom",o,rospy.Time(0),rospy.Duration(3.0))
                ts.append(t)
                h = self.buf.lookup_transform("hand_palm_link",o,rospy.Time(0),rospy.Duration(3.0))
                d = np.power(h.transform.translation.x**2+h.transform.translation.y**2+h.transform.translation.z**2,0.5)
                ds.append(d)
            if len(ds) >0:
                i = np.argmin(ds)
                dmin = ds[i]
                if dmin >= XMIN and dmin <= XDIS:
                    t = ts[i]
                    self.broadcast(t,OBJIDS[i])
            for i in range(len(EQUIPS)):
                e = EQUIPS[i]
                t = self.buf.lookup_transform("odom",e,rospy.Time(0),rospy.Duration(3.0))
                h = self.buf.lookup_transform("base_footprint",e,rospy.Time(0),rospy.Duration(3.0))
                if h.transform.translation.x >0. and h.transform.translation.x < XD:
                    if np.abs(h.transform.translation.y) < YDIS:
                        self.broadcast(t,EQUIPIDS[i])
                
                    
            self.rate.sleep()
            
        
if __name__ == "__main__":
    rospy.init_node("marker_publisher")
    a = MarkerPublisher()
    a.run()
