#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import planning_program_sig as planning_program
from geometry_msgs.msg import TransformStamped
import planner
import tf2_ros
from tf_utils import Tf
import tf
import math
import numpy as np
try:
    from rosbag_database.srv import RosbagRecord, RosbagRecordRequest
    from rosbag_database.srv import RosbagStop, RosbagStopRequest
    from rosbag_database.srv import RosbagPlayRequest
except:
    pass
from joint_sampler import JointBagReader
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

TEST_FOLDA = "new_act/action"
#CAT = ["box","door","fridge","pettbotle"]
CAT = ["button","table","doll","fruit","drink"]

CAT_OBJ={"pettbotle":[0,1,2,3,4,5,6],
         "box":[7],
         "door":"recognized_object/exp_door1/15",
         "fridge":"recognized_object/exp_friedge/19"
         }
MARK = {"recognized_object/exp_door1/15": "ar_marker/15",
        "recognized_object/exp_friedge/19": "ar_marker/19"}
CAL=0
TOPICS=["/hsrb/odom",
        "/hsrb/joint_states",
        ]
ENV="environment"        



def quat2quat(quat1, quat2):
    """
    quat1 --> quat2 = quat
    """
    key = False
    qua1 = -quat1[0:3]
    quv1 = quat1[3]
    if len(quat2.shape) ==1:
        qua2 = quat2[0:3]        
        quv2 = quat2[3]
    else:
        qua2 = quat2[:,0:3]        
        quv2 = quat2[:,3]
        key = True
    if not key:
        qua = quv1 * qua2 + quv2 * qua1 + np.cross(qua1, qua2)
        quv = quv1 * quv2 - np.dot(qua1, qua2)
        if quv < 0:
            quv = quv * -1
            qua = qua * -1
    else:
        qua = quv1 * qua2 + np.c_[qua1[0]*quv2,qua1[1]*quv2,qua1[2]*quv2] + np.c_[qua1[1]*qua2[:,2]+qua1[2]*qua2[:,1], qua1[0]*qua2[:,2]+qua1[2]*qua2[:,0], qua1[1]*qua2[:,0]+qua1[0]*qua2[:,1]]
        quv = quv1 * quv2 - (qua1[0] * qua2[:,0] + qua1[1] * qua2[:,1] + qua1[2] * qua2[:,2])
    nq = np.linalg.norm(qua)
    if nq != 0.0 and quv < 1.0:
        p = np.sqrt(1.0 - np.power(quv,2)) / nq
        qua_p = np.array(qua)*p
        quat = np.r_[qua_p, np.array([quv])]
    else:
        quat = np.array([0.,0.,0.,1.])
    return quat


class ObjectGetter(object):
    def __init__(self, robot):
        self.body = planning_program.JointGroup()
        self.body.angular_weight = 0.5
        self.body.linear_weight = 2.0
        self.joint_group = robot.get('whole_body', robot.Items.JOINT_GROUP)
        self.buf = tf2_ros.Buffer()
        self._lis = tf2_ros.TransformListener(self.buf)
        self._broadcaster = tf2_ros.TransformBroadcaster()        
        self.planner = planner.Planner(TEST_FOLDA,CAT)
        self._tf = Tf()
        
        try:
            self.srv_record_start = rospy.ServiceProxy("rosbag_record", RosbagRecord)
            self.srv_record_stop = rospy.ServiceProxy("rosbag_record_stop", RosbagStop)
            self.srv_marker = rospy.ServiceProxy("marker_data_saver", Empty)
            self.jbr = JointBagReader()
        except:
            pass
        self.rosbag_number = 0
        self.record_flag = False
        self.rosbag_time = rospy.Time(0)
        self.record_number = 0
        self.number = 0
        self.date = "act_bag_{}"
        self.image_number = 0
        self.image_time = 0
            

    def _calc_lookup_head(self, x, y, z,base_tf):
        """
        HSRがそのTf_nameに頭を向けるようにする
        関節角度制御による位置指定のため
        向き続けるわけではないため注意
        param float x: 視点の目標位置x
        param float y: 視点の目標位置y
        param float z: 視点の目標位置z
        param float base_tf: 視点の基準ｔｆ

        return float: pan joint        
        return float: tilt joint
        """
        headxyz = []
        pan = 0.0
        tilt = 0.0
        try:
            headxyz = self._tf.calcpos_base2child( x, y, z, base_tf, "torso_lift_link")
        except:
            pass
        if headxyz == []:
            return 0., 0.
        x = headxyz[0]
        y = headxyz[1]
        z = headxyz[2]
        pan = np.arctan2(y,x)
        tilt = np.arctan2(z,np.sqrt(x**2+y**2))
        mv_pan = pan
        mv_tilt = tilt

        if mv_pan > 1.75:
            mv_pan = 1.75
        elif mv_pan < -3.5:
            mv_pan = -3.5
        if mv_tilt > 0.5:
            mv_tilt = 0.5
        elif mv_tilt < -1.5:
            mv_tilt = -1.5
        return mv_pan,mv_tilt
        
    def action(self,obj_pose, object_category):
        back_ = None
        otype = None
        _fl = True
        if len(obj_pose) ==3:
            otype="object"
        elif len(obj_pose) ==7:
            otype="equipment"
        action_count = 0
        s = rospy.Time.now()
        while not rospy.is_shutdown():
            hand = self.buf.lookup_transform("map","hand_palm_link",rospy.Time.now(),rospy.Duration(3.0))
            h = hand.transform
            end_effect = [h.translation.x, h.translation.y, h.translation.z, h.rotation.x, h.rotation.y, h.rotation.z,h.rotation.w]
            if otype == "object":
                xyz,qxyzw,exyz = self._tf.lookup("base_footprint", "map")
                p = obj_pose
                p2 = self._tf.calcpos_base2child(p[0],p[1],p[2],"map", "base_footprint")
                x = -p2[0]
                y = -p2[1]
                zaw = math.atan2(y, x)
                yaw = -1.57
                qxyzw1_e = tf.transformations.quaternion_from_euler(0.0,yaw,zaw)
                qxyzw2_w = tf.transformations.quaternion_from_euler(0.0,0.0,zaw)
                qxyzw1 = quat2quat(np.array(qxyzw),np.array(qxyzw1_e))
                qxyzw2 = quat2quat(np.array(qxyzw),np.array(qxyzw2_w))
                if obj_pose[2] < 0.05:
                    q = qxyzw1
                else:
                    q = qxyzw2
                obj_pose.extend(q)
            elif otype=="equipment":
                oname = CAT_OBJ[object_category]
                marker = MARK[oname]
                try:
                    m = self.buf.lookup_transform("map", marker, rospy.Time.now(),rospy.Duration(3.0))
                    m_pos = [m.transform.translation.x, m.transform.translation.y, m.transform.translation.z]
                except:
                    m_pos = obj_pose[:3]
                    print "cant look mark"
                    pass
            now_hand = self.body.joint_positions["hand_motor_joint"]
            if now_hand > 0.8:
                now_hand = 1.0
                self.body.angular_weight = 0.5
                self.body.linear_weight = 1.0
            else:
                now_hand = 0.0
                self.body.angular_weight = 0.5
                self.body.linear_weight = 50.0
            c = self.planner.choose_action(end_effect,now_hand,obj_pose,object_category,CAL,back_)
#            np.savetxt("{0:d}_{1:d}_{2:d}_{3:s}.csv".format(action_count,c[0],c[1],c[4]),c[2],delimiter=(","))
            action_count += 1
#            self.body.impedance_config = c[4]
#            self.body.impedance_config = "SSS"
            
            rospy.loginfo(c[-1])
            rospy.loginfo(c[4])
            pk = 0
            while not rospy.is_shutdown():
                try:
                    self.body.move_cartesian_path(c[2],c[3],ref_frame_id="map")
                    break
                except:
                    pk+=1
                    print "miss{}".format(pk)
                    continue
            if c[5]:
                break
            back_ = c[0]
            now_hand = self.body.joint_positions["hand_motor_joint"]
            if now_hand > 0.8:
                now_hand = 1.0
            else:
                now_hand = 0.0
#                _fl = False
#            
#            if otype == "equipment":
#                if _fl:
#                    pan,tilt = self._calc_lookup_head(m_pos[0], m_pos[1], m_pos[2], "map")
#                    goal = {}
#                    goal["head_pan_joint"] = pan
#                    goal["head_tilt_joint"] = tilt
#                    self.joint_group.move_to_joint_positions(goal)
#                    try:
#                        o = self.buf.lookup_transform("map", oname, rospy.Time.now(),rospy.Duration(3.0))
#                        oo = o.transform
#                        obj_pose = [oo.translation.x,oo.translation.y, oo.translation.z,oo.rotation.x,oo.rotation.y,oo.rotation.z,oo.rotation.w]
#                    except:
#                        print "pass"
#                        pass
#            if otype == "object":
#                if _fl:
#                    pan,tilt = self._calc_lookup_head(obj_pose[0], obj_pose[1], obj_pose[2]-0.15, "map")
#                    goal = {}
#                    goal["head_pan_joint"] = pan
#                    goal["head_tilt_joint"] = tilt
#                    self.joint_group.move_to_joint_positions(goal)
#                    try:
#                        o = self.obj.get_object_pos_filterd_time_and_specific_score(10,0,0.75)
#                        obj_pose = o[0][0]
#                    except:
#                        print "pass"
#                        pass
        hand = self.body.joint_positions["hand_motor_joint"]
        rospy.loginfo(rospy.Time.now().to_sec()-s.to_sec())
        if hand >= 1.18:
            return False
        elif hand <= -0.83:
            return False
        else:
            return True

    def test(self,obj_pose, object_category,cl_len=[None,None]):
        back_ = None
        otype = None
        if len(obj_pose) ==3:
            otype="object"
        elif len(obj_pose) ==7:
            otype="equipment"
        action_count = 0
        while not rospy.is_shutdown():
            hand = self.buf.lookup_transform("map","hand_palm_link",rospy.Time.now(),rospy.Duration(3.0))
            h = hand.transform
            end_effect = [h.translation.x, h.translation.y, h.translation.z, h.rotation.x, h.rotation.y, h.rotation.z,h.rotation.w]
            if otype == "object":
                xyz,qxyzw,exyz = self._tf.lookup("base_footprint", "map")
                p = obj_pose
                p2 = self._tf.calcpos_base2child(p[0],p[1],p[2],"map", "base_footprint")
                x = -p2[0]
                y = -p2[1]
                zaw = math.atan2(y, x)
                yaw = -1.57
                qxyzw1_e = tf.transformations.quaternion_from_euler(0.0,yaw,zaw)
                qxyzw2_w = tf.transformations.quaternion_from_euler(0.0,0.0,zaw)
                qxyzw1 = quat2quat(np.array(qxyzw),np.array(qxyzw1_e))
                qxyzw2 = quat2quat(np.array(qxyzw),np.array(qxyzw2_w))
                print np.sin(zaw)
                print np.cos(zaw)
                p3 = self._tf.calcpos_base2child(p2[0],p2[1],p2[2], "base_footprint","map")                
                if obj_pose[2] < 0.05:
                    q = qxyzw1
                else:
                    q = qxyzw2
                obj_pose.extend(q)
            elif otype=="equipment":
                oname = CAT_OBJ[object_category]
                marker = MARK[oname]
                try:
                    m = self.buf.lookup_transform("map", marker, rospy.Time.now(),rospy.Duration(3.0))
                    m_pos = [m.transform.translation.x, m.transform.translation.y, m.transform.translation.z]
                except:
                    m_pos = obj_pose[:3]
                    print "cant look mark"
                    pass
            now_hand = self.body.joint_positions["hand_motor_joint"]
            if now_hand > 0.8:
                now_hand = 1.0
            else:
                now_hand = 0.0
            c = self.planner.choose_action(end_effect,now_hand,obj_pose,object_category,CAL,back_,top_command_act_cls_len=cl_len)
            np.savetxt("{0:d}_{1:d}_{2:d}_{3:s}.csv".format(action_count,c[0],c[1],c[4]),c[6],delimiter=(","))
            action_count += 1
            self.body.impedance_config = c[4]
#            self.body.impedance_config = "SSS"
            
            rospy.loginfo(c[-1])
            rospy.loginfo(c[4])
            while not rospy.is_shutdown():
                try:
                    self.body.move_cartesian_path(c[2],c[3],ref_frame_id="map")
                    break
                except:
                    continue
            if True:
                break
            back_ = c[0]
            
            if otype == "equipment":
                pan,tilt = self._calc_lookup_head(m_pos[0], m_pos[1], m_pos[2], "map")
                goal = {}
                goal["head_pan_joint"] = pan
                goal["head_tilt_joint"] = tilt
                self.joint_group.move_to_joint_positions(goal)
#                try:
#                    o = self.buf.lookup_transform("map", oname, rospy.Time.now(),rospy.Duration(3.0))
#                    oo = o.transform
#                    obj_pose = [oo.translation.x,oo.translation.y, oo.translation.z,oo.rotation.x,oo.rotation.y,oo.rotation.z,oo.rotation.w]
#                except:
#                    print "pass"
#                    pass
        hand = self.body.joint_positions["hand_motor_joint"]
        if hand >= 1.18:
            return False
        elif hand <= -0.83:
            return False
        else:
            return True        

    def set_stamp(self, xyz, qxyzw,name="target_object"):
        ts = TransformStamped()
        ts.header.frame_id = "odom"
        ts.child_frame_id = name
        ts.transform.translation.x = xyz[0]
        ts.transform.translation.y = xyz[1]
        ts.transform.translation.z = xyz[2]
        ts.transform.rotation.x = qxyzw[0]
        ts.transform.rotation.y = qxyzw[1]
        ts.transform.rotation.z = qxyzw[2]
        ts.transform.rotation.w = qxyzw[3]
        ts.header.stamp = rospy.Time.now()
        return ts
        
    def broadcast(self,ts):
        """
        tf_stampを発行する関数
        param TransformStamped tf_stamp: tfのRosMessage        
        """
        
        ts.header.stamp = rospy.Time.now()
        self._broadcaster.sendTransform(ts)


    def action_crest(self,obj_pose, act_list,object_category):
        back_ = None
        otype = None
        _fl = True
        if len(obj_pose) ==3:
            otype="object"
        elif len(obj_pose) ==7:
            otype="equipment"
        action_count = 0
        
        s = rospy.Time.now()
        bb_req = RosbagRecordRequest()
        bb_req.node_name= "joint_teleope_bag"
        bb_req.save_name= "act"
        bb_req.split_duration_str = "600"
        bb_req.record_topic_list = TOPICS
        self.rosbag_time = rospy.Time.now()
        res = self.srv_record_start.call(bb_req)        
        record_number = res.record_number
        
        actor = self.planner.set_motion(act_list,object_category)
        for i in range(len(actor)):
            hand = self.buf.lookup_transform("odom","hand_palm_link",rospy.Time.now(),rospy.Duration(3.0))
            h = hand.transform
            end_effect = [h.translation.x, h.translation.y, h.translation.z, h.rotation.x, h.rotation.y, h.rotation.z,h.rotation.w]
            if otype == "object":
                xyz,qxyzw,exyz = self._tf.lookup("base_footprint", "odom")
                p = obj_pose
                p2 = self._tf.calcpos_base2child(p[0],p[1],p[2],"odom", "base_footprint")
                x = -p2[0]
                y = -p2[1]
                zaw = math.atan2(y, x)
                yaw = -1.57
                qxyzw1_e = tf.transformations.quaternion_from_euler(0.0,yaw,zaw)
                qxyzw2_w = tf.transformations.quaternion_from_euler(0.0,0.0,zaw)
                qxyzw1 = quat2quat(np.array(qxyzw),np.array(qxyzw1_e))
                qxyzw2 = quat2quat(np.array(qxyzw),np.array(qxyzw2_w))
                if obj_pose[2] < 0.05:
                    q = qxyzw1
                else:
                    q = qxyzw2
                obj_pose.extend(q)
            self.body.angular_weight = 50.
            self.body.linear_weight = 50.
            c = self.planner.make_trajector(actor[i], end_effect, 1., obj_pose, CAT[object_category],back_)
            for j in range(len(c[0])):
                st = self.set_stamp(c[0][j][0],c[0][j][1],"act_{}".format(j))
                self.broadcast(st)
            back_ = actor[i]
#            np.savetxt("{0:d}_{1:d}_{2:d}_{3:s}.csv".format(action_count,c[0],c[1],c[4]),c[2],delimiter=(","))
            action_count += 1
            act =1
            while not rospy.is_shutdown():
                if act == len(c[0]):
                    break
                try:
                    self.body.move_cartesian_path(c[0][::act],c[1][::act],ref_frame_id="odom")
                    break
                except:
                    act+=1
                    print act
                    pass
        bs_req = RosbagStopRequest()
        bs_req.rosbag_number = record_number
        self.srv_record_stop.call(bs_req)
        ##########            
        req = RosbagPlayRequest()
        req.name = "act"
        req.topics = TOPICS[0]
        print req
        j = self.jbr.data_get(req, Odometry())
        csvs = []
        for jj in j:
            csv = [jj.header.stamp.to_sec(), jj.twist.twist.linear.x,jj.twist.twist.linear.y,jj.twist.twist.angular.z]
            csvs.append(csv)
        np.savetxt("base.csv",csvs,delimiter=",")            
        ############
        req = RosbagPlayRequest()
        req.name = "act"
        req.start_time = s
        req.end_time = rospy.Time.now()
        req.count_number = 0
        req.duration = 0
        req.topics = TOPICS[1]
        print req
        j = self.jbr.data_get(req, JointState())
        csvs = []
        for jj in j:
            csv = [jj.header.stamp.to_sec()]
            csv.extend(jj.position)
            csvs.append(csv)
        np.savetxt("_".join(jj.name)+".csv",csvs,delimiter=",")
        

        

        
        return True



if __name__=="__main__":  
    pass
