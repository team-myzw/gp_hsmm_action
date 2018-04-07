#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np

import rospy
import tf2_ros
import tf
import os
import threading

from multiprocessing import Process




from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped

STATIC_TF = "rosrun tf2_ros static_transform_publisher {} {} {} {} {} {} {} {} {} __name:=hsr7_tf_static_{}"


def rotM(p):
    # 回転行列を計算する
    px = p[0]
    py = p[1]
    pz = p[2]
     #物体座標系の 3->2->1 軸で回転させる
    Rx = np.array([[1, 0, 0],
                    [0, np.cos(px), np.sin(px)],
                    [0, -np.sin(px), np.cos(px)]])
    Ry = np.array([[np.cos(py), 0, -np.sin(py)],
                    [0, 1, 0],
                    [np.sin(py), 0, np.cos(py)]])
    Rz = np.array([[np.cos(pz), np.sin(pz), 0],
                    [-np.sin(pz), np.cos(pz), 0],
                    [0, 0, 1]])
    R = Rx.dot(Ry).dot(Rz)
    return R

def calctransform(p,B,sP_B):
#    p = np.array([np.pi,np.pi/2, np.pi/3])
    R = rotM(p)
    A = R.T
    
    O = np.array([0, 0, 0])
    sP_O = np.dot(A, sP_B)
    rB_O = B - O
    rP_O = rB_O + sP_O
    return rP_O



class Tf(object):
    def __init__(self):
        #
        self._buffer = tf2_ros.Buffer()
        rospy.loginfo("class create success")
        self.lis = tf2_ros.TransformListener(self._buffer)
#        self._listener = tf2_ros.TransformListener(self._buffer)
        self._broadcaster = tf2_ros.TransformBroadcaster()
        self._rate = rospy.Rate(10)
        self._publish_flag = False
        self._tf_flag = True
        self._tf_list = []
        self._thread = threading.Thread(target=self.run, name="publish")
        self._thread.start()

    def _name_detect(self, name):
        flag = False        
        
        for i,tf_name in enumerate(self._tf_list):
            if tf_name.child_frame_id != name:
                continue
            else:
                flag = True
                del(self._tf_list[i])
                return flag
        return flag

    def _transformbase2child(self,x, y, z, xaw, yaw, zaw, child_tf, base_tf="map"):
        """
        child_tfから見てx y z の位置にあり
        xaw yaw zawだけ各軸に回転させた
        base_tf基準の姿勢を返す
        param float x: tfの位置x
        param float y: tfの位置y
        param float z: tfの位置z
        param flaot xaw: tfの姿勢x オイラー角
        param flaot yaw: tfの姿勢y オイラー角
        param flaot zaw: tfの姿勢z オイラー角
        param string base_tf: 基準と成るtfの名前
        param string publish_tf: 発行基準と成るtfの名前
        return list: xyzの座標
        return list: xyzwの姿勢 クォータニオン
        return list: xyzの姿勢　オイラー
        """
        odomxyz, odomxyzw, odomexyz = self.lookup(base_tf, child_tf)
        if odomxyz==[]:
            rospy.logwarn("no tf {} or {}".format(child_tf, base_tf))
            return [], [], []
        obj_pos_from_base_frame = np.array([x, y, z])
        rot = odomexyz
        rot = np.array(rot)
        pos = np.array(odomxyz)
        base_pos = calctransform(rot,pos,obj_pos_from_base_frame)
        child_rot = np.array([xaw, yaw, zaw])
        rot = rot + child_rot
        qua = tf.transformations.quaternion_from_euler(rot[0],rot[1],rot[2])
        
        return base_pos, qua, rot


    def _param2transformstamp(self, tf_name, x, y, z, xaw, yaw, zaw, base_tf, publish_tf="map"):
        """
        base_tfから見てx y z の位置にあり
        xaw yaw zawだけ各軸に回転させた姿勢で
        publish_frame からのTfの型に
        tf_nameという名前で設定する関数
        param string tf_name: 発行するtfの名前
        param float x: tfの位置x
        param float y: tfの位置y
        param float z: tfの位置z
        param flaot xaw: tfの姿勢x オイラー角
        param flaot yaw: tfの姿勢y オイラー角
        param flaot zaw: tfの姿勢z オイラー角
        param string base_tf: 基準と成るtfの名前
        param string publish_tf: 発行基準と成るtfの名前
        return TransformStamped: tfのRosMessage
        """
        mes = TransformStamped()
        
        xyz, qxyzw, exyz = self._transformbase2child(x, y, z, xaw, yaw, zaw, base_tf, publish_tf)
        if xyz==[]:
            return mes

        mes.header = Header(frame_id=publish_tf)
        mes.child_frame_id = "{0}".format(tf_name)

        mes.transform.translation.x = xyz[0]
        mes.transform.translation.y = xyz[1]
        mes.transform.translation.z = xyz[2]

        mes.transform.rotation.x = qxyzw[0]
        mes.transform.rotation.y = qxyzw[1]
        mes.transform.rotation.z = qxyzw[2]
        mes.transform.rotation.w = qxyzw[3]

        return mes

    def _publish(self, tf_stamp):
        """
        tf_stampを発行する関数
        param TransformStamped tf_stamp: tfのRosMessage        
        """
        tf_stamp.header.stamp = rospy.Time.now()
        self._broadcaster.sendTransform(tf_stamp)

    def calcpos_base2child(self, x, y, z, child_tf, base_tf):
        """
        child_tfから見てx y z の位置にあるものを
        base_tfから見てx,y,zの位置を返す関数
        param float x: tfの位置x
        param float y: tfの位置y
        param float z: tfの位置z
        param string base_tf: 変換前の基準tfの名前
        param string publish_tf: 変換後の基準tfの名前
        return list: xyzの座標
        """
        odomxyz, odomxyzw, odomexyz = self.lookup(base_tf, child_tf)
        if odomxyz==[]:
            rospy.logwarn("no tf {} or {}".format(child_tf, base_tf))
            return [], [], []
        obj_pos_from_base_frame = np.array([x, y, z])
        rot = odomexyz
        rot = np.array(rot)
        pos = np.array(odomxyz)
        base_pos = calctransform(rot,pos,obj_pos_from_base_frame)
        return base_pos

        
    def lookup(self, base_tf, child_tf):
        """
        base_tfから見たchild_tfの姿勢を返す関数
        param string base_tf: 基準と成るtfの名前
        param string _tf: 基準と成るtfの名前
        return list: xyzの座標
        return list: xyzwの姿勢 クォータニオン
        return list: xyzの姿勢　オイラー
        """
        try:
            base2child = self._buffer.lookup_transform(base_tf, child_tf, rospy.Time(0), rospy.Duration(3.0))
        except:
            print("not exist {} {}".format(base_tf, child_tf))
            return [], [], []
        xyz = [base2child.transform.translation.x, base2child.transform.translation.y, base2child.transform.translation.z]
        xyzw = [base2child.transform.rotation.x, base2child.transform.rotation.y, base2child.transform.rotation.z, base2child.transform.rotation.w] 
        exyz = tf.transformations.euler_from_quaternion(xyzw)
        return xyz, xyzw, exyz


    def publish_start(self,):
        """
        tfの発行を開始する関数
        """
        self._publish_flag=True
        self._tf_flag = False
        while not rospy.is_shutdown():
            if self._tf_flag:
                break

    def publish_stop(self,):
        """
        tfの発行を停止する関数
        """
        self._publish_flag=False

    def run(self):
        """
        tfを発行する関数
        別スレッドで常時実行している
        publish_flagがTrueの時は発行する
        """
        while not rospy.is_shutdown():
            flag = False
            if not self._tf_flag:
                flag = True                
            if not self._publish_flag:
                self._rate.sleep()        
                continue
            for tf_lis in self._tf_list:
                self._publish(tf_lis)
            if flag and not self._tf_flag:
                self._tf_flag = True
            self._rate.sleep()
        print "wait for stop"
        self._thread.join()
        print "stop"

    def tf_deleate(self, tf_name):
        """
        tf_listに登録されたtf_nameを削除する
        param string tf_name: 削除するtfの名前
        return bool: 関数の成否
        """
        flag = False        
        
        for i,tf_lis in enumerate(self._tf_list):
            if tf_lis.child_frame_id != tf_name:
                continue
            else:
                flag = True
                del(self._tf_list[i])
                rospy.loginfo("deleate {}".format(tf_name))
                return flag
        rospy.loginfo("not exist {}".format(tf_name))
        return flag

    def tf_deleate_all(self,):
        """
        tf_listに登録されたtf_nameをすべて削除する
        """
        self._tf_list = []
        
        
    def tf_set(self,tf_name, x, y, z, xaw, yaw, zaw, base_tf, publish_tf="map"):
        """
        発行したいtfの情報を入力する
        入力した情報を発行するTf_listに登録する関数
        この関数は登録のみで発行はしない
        param string tf_name: 発行するtfの名前
        param float x: tfの位置x
        param float y: tfの位置y
        param float z: tfの位置z
        param flaot xaw: tfの姿勢x オイラー角
        param flaot yaw: tfの姿勢y オイラー角
        param flaot zaw: tfの姿勢z オイラー角
        param string base_tf: 基準と成るtfの名前
        param string publish_tf: 発行基準と成るtfの名前
        """
        self.tf_deleate(tf_name)
        mes = self._param2transformstamp(tf_name, x, y, z, xaw, yaw, zaw, base_tf, publish_tf)
        self._tf_list.append(mes)


    def tf_set_static(self,tf_name, x, y, z, xaw, yaw, zaw, base_tf, publish_tf="map"):
        """
        staticに発行したいtfの情報を入力する
        入力した関数は別スレッドにて実行される
        この関数は発行まで行う
        本当に消さない、常に残し続けるTfの登録のみ推奨する
        param string tf_name: 発行するtfの名前
        param float x: tfの位置x
        param float y: tfの位置y
        param float z: tfの位置z
        param flaot xaw: tfの姿勢x オイラー角
        param flaot yaw: tfの姿勢y オイラー角
        param flaot zaw: tfの姿勢z オイラー角
        param string base_tf: 基準と成るtfの名前
        param string publish_tf: 発行基準と成るtfの名前
        """
        xyz, qxyzw, exyz = self._transformbase2child(x, y, z, xaw, yaw, zaw, base_tf, publish_tf)

        message = STATIC_TF.format(xyz[0], xyz[1], xyz[2], qxyzw[0], qxyzw[1], qxyzw[2], qxyzw[3], publish_tf, tf_name, tf_name)
        pr = Process(target=os.system, args=[message])

        pr.start()

    def tf_set_from_db(self,name):
        """
        発行したいtfの名前を入力する
        data_baseに記録されているその名前のtfを発行する
        未実装
        param storing name: データベースに登録されている名称
        return bool: 発行の成否
        """
        return False
        
        
        
if __name__ =="__main__":
    rospy.init_node("example")
    tt = Tf() 
##  map base_footprintから見て, x=0.5, y=0.0, z=0.2 の位置を mapから見た位置に変換する
    xyz = tt.calcpos_base2child(0.5, 0.0, 0.2, "base_footprint", "map")
    print "calcpos_base2child"
    print xyz
    
##  map から見た　hand_palm_link　の位置　xyz, クォータニオン　qxyzw, オイラー角 xyz を返す
    xyz, qxyzw, exyz = tt.lookup("map", "hand_palm_link")    
    print "lookup"
    print xyz
    print qxyzw
    print exyz

##  static_なtfを発行する　これは消せない    
    tt.tf_set_static("test_static2", 0.1, 0.1, 0.1, 0., 0., 1.57, "base_footprint", "map")
    rospy.sleep(1.0)
    print "tf_set_static"
    
##  base_footprintから見て、 0,1 0,1 0,1 の位置に　x軸に０　y軸に０　ｚ軸に１．５７回転させたｔｆを　map 基準で登録する
    tt.tf_set("test", 0.1, 0.1, 0.1, 0., 0., 1.57, "base_footprint", "map")
    print "tf_set　test"
        
##  登録したTfを発行する    
    tt.publish_start()
    print "publish_start"
    
    rospy.sleep(5.0)
    
##  登録したTf指定して削除する
    tt.tf_deleate("test")
    print "tf_deleate"

##  発行を止めなければ途中で追加すると随時発行される
    tt.tf_set("test2", 0.1, 0.1, 0.1, 0., 0., 1.57, "base_footprint", "map")
    print "tf_set test2"

    rospy.sleep(5.0)
            
##  tfの発行を止める
    tt.publish_stop()
    print "publish_stop"
    rospy.spin()
    print "all stop"