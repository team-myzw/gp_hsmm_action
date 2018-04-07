#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import actionlib

from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryResult, FollowJointTrajectoryActionFeedback
from tmc_control_msgs.msg import GripperApplyEffortActionGoal
TIME=0.0



class HandGrip(object):
    def __init__(self):
        rospy.Subscriber("/hsrb/joint_states", JointState, self.cb_joint, queue_size=10)
        rospy.set_param('/hsrb/gripper_effort/joints', ["hand_motor_joint"])
        self.action_server = actionlib.SimpleActionServer("/hsrb/gripper_effort/follow_joint_trajectory", 
                                                    FollowJointTrajectoryAction, execute_cb=self.cb,
                                                    auto_start=False)
        self.grip_act = rospy.Publisher("/hsrb/gripper_controller/grasp/goal",GripperApplyEffortActionGoal, queue_size=10)
        
        self.rate = rospy.Rate(100)
        self.hand_state = 0.0
        self.action_server.start()

    def cb_joint(self, msg):
        self.hand_state = msg.position[7]
        
    def hand_open(self,):            
        grip = GripperApplyEffortActionGoal()
        grip.goal.effort = 0.02
        rospy.loginfo("open")
        rospy.sleep(3.)
        self.grip_act.publish(grip)
    
    def hand_close(self,):
        grip = GripperApplyEffortActionGoal()
        grip.goal.effort = -0.05
        rospy.loginfo("grasp")
        rospy.sleep(3.)
        self.grip_act.publish(grip)
        

    def cb(self, data):
        """
        指定時間の間に閾値以上の力が力各センサに加わったかどうかを返す。
        現在の値からの変位を見ているため、開始のタイミングに注意

        Args:
            ntime  int32: 指定時間
            nforce float64: 閾値の力
        Result:
            success bool:
        Feedback:
            force float64: 現在加えられている力
        """
        num = 1
        N = len(data.trajectory.points)

        success =FollowJointTrajectoryResult()
        success.SUCCESSFUL
        end_sec = data.trajectory.points[-1].time_from_start.to_sec()
        start_sec = rospy.Time.now().to_sec()
        hand = self.hand_state
        _fl = False
        hp = data.trajectory.points[0].positions[0]
        if hand > 1.0 and hp <= 0.5:
            self.hand_close()
        elif hand < 0.1 and hp >= 0.5:
            self.hand_open()
        if hp >= 0.5:
            _fl=True
        while (rospy.Time.now().to_sec()-start_sec) <= (end_sec + 1.0) and not rospy.is_shutdown():
            hand = self.hand_state
            n = num -1
            if n <0:
                n = 0
            if hp <= 0.5 and _fl:
                self.hand_close()
            if (rospy.Time.now().to_sec()-start_sec) >= data.trajectory.points[n].time_from_start.to_sec()-TIME:
                hp = data.trajectory.points[num].positions[0]
                if hand > 1.0 and hp <= 0.5:
                    self.hand_close()
                    self.hand_close()
                elif hand < 0.1 and hp >= 0.5:
                    self.hand_open()
                num += 1
            if num == N:
                break
#            self.action_server.publish_feedback(feedback)
            self.rate.sleep()
        self.action_server.set_succeeded(success)


if __name__ == "__main__":
    rospy.init_node("hand_state_controller")
    wrist_wrench = HandGrip()
    rospy.spin()    
