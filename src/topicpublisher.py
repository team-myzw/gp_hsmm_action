#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
import numpy as np
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import actionlib

JOINT = "arm_flex_joint_arm_lift_joint_arm_roll_joint_base_l_drive_wheel_joint_base_r_drive_wheel_joint_base_roll_joint_hand_l_spring_proximal_joint_hand_motor_joint_hand_r_spring_proximal_joint_head_pan_joint_head_tilt_joint_wrist_flex_joint_wrist_roll_joint.csv"
BASE = "base.csv"
class TopicPublisher(object):
    def __init__(self,):
#        self._action = actionlib.SimpleActionClient('/hsrb/arm_trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)        
        self.pub_joint = rospy.Publisher("/hsrb/arm_trajectory_controller/command",JointTrajectory,queue_size=10)
        self.pub_base = rospy.Publisher("/hsrb/opt_command_velocity",Twist,queue_size=10)
        self.pub_sig = rospy.Publisher('/hsrb/gripper_trajectory_controller/command', JointTrajectory, queue_size=10)        
        self.traj = JointTrajectory()
        self.traj.joint_names = ["arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        self.p = JointTrajectoryPoint()
        self.p.positions = [0.,0.,0.,0.,0.]
        self.p.velocities = []
        self.p.effort = []
        for i in range(5):
            self.p.velocities += [0.0]
            self.p.effort += [0.0]
        self.p.time_from_start = rospy.Time(1.0)
        self.traj.points = [self.p]
        
#        self.goal = FollowJointTrajectoryGoal()
        
    def hand_close(self,):
        hand_sig = JointTrajectory()
        hand_sig.joint_names = ["hand_l_proximal_joint","hand_r_proximal_joint"]
        pp = JointTrajectoryPoint()
        pp.positions = [-0.05,-0.05]
        pp.velocities = []
        pp.effort = []
        hand_sig.points = [pp]
        self.pub_sig.publish(hand_sig)            

    def hand_open(self,):
        hand_sig = JointTrajectory()
        hand_sig.joint_names = ["hand_l_proximal_joint","hand_r_proximal_joint"]
        pp = JointTrajectoryPoint()
        pp.positions = [0.61,-0.61]
        pp.velocities = []
        pp.effort = []
        hand_sig.points = [pp]
        self.pub_sig.publish(hand_sig)            
        
        
    def run(self,):
        b = np.loadtxt(BASE,delimiter=",")
        j = np.loadtxt(JOINT,delimiter=",")
        bs = b[0][0]
        js = j[0][0]
        be = b[-1][0]
        je = j[-1][0]
        ji = 1
        jn = len(j)
        bi = 1
        bn = len(b)
        t = Twist()
        t.linear.x = b[0][1]
        t.linear.y = b[0][2]
        t.angular.z = b[0][3]
        p = JointTrajectoryPoint()
        p.positions = [j[0][2],
                       j[0][1],
                       j[0][3],
                       j[0][12],
                       j[0][13]]
        p.velocities =[]
        p.effort = []
        p.time_from_start = rospy.Time((je-js)/float(jn))
        self.traj.points =[p]
        self.pub_base.publish(t)
        self.pub_joint.publish(self.traj)
        jf = False
        bf = False
        s = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            if jf and bf:
                break
            if not bf:
                if  (rospy.Time.now().to_sec()-s) > b[bi][0]-bs:
                    t = Twist()
                    t.linear.x = b[bi][1]
                    t.linear.y = b[bi][2]
                    t.angular.z = b[bi][3]
                    self.pub_base.publish(t)
                    bi += 1
                    if bi == bn:
                        bf = True
            if not jf:
                if  (rospy.Time.now().to_sec()-s) > j[ji][0]-js:
                    if j[ji][8] > 0.6:
                        self.hand_open()
                    else:
                        self.hand_close()
                    p = JointTrajectoryPoint()
                    p.positions = [j[ji][2],
                                   j[ji][1],
                                   j[ji][3],
                                   j[ji][12],
                                   j[ji][13]]
                    p.velocities =[]
                    p.effort = []
    #                p.time_from_start = rospy.Time((je-js)/float(jn))
                    p.time_from_start = rospy.Time(1.0)
                    self.traj.points =[p]
                   # self.goal.trajectory = self.traj
                   # self._action.send_goal(self.goal)
                    self.pub_joint.publish(self.traj)
                    ji += 1
                    if ji == jn:
                        jf = True
        return True
                
            
        

if __name__=="__main__":  
    rospy.init_node("make_trajector")
    r = TopicPublisher()
    r.run()
