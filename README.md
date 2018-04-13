# gp_hsmm_action   
学習動作実行方法  
1. localにmasterを持つコンソールを立ち上げる.  
2. localのコンソールで,roslaunch hsrb_rviz_simulator hsrb_rviz_simulator.launch を実行する  
3. localのコンソールで,roslaunch gp_hsmm_action action.launch を実行する.  
4. localのコンソールの,gp_hsmm_action/srcにおいてipythonを実行する.  
5. localのipythonコンソールで,以下のコードを実行する.  
import action_done  
from hsrb_interface import Robot  
robot = Robot()  
action = action_done.ObjectGetter(robot)  
6. sigverseをmasterに持つコンソールを立ち上げる  
7. sigverseをmasterに持つコンソールで,python marker_publisher.pyを実行する  
8. sigverse上の対象物体の位置を取得する:以降この物体位置をpos=[x,y,z]とする  
9. 対象物のカテゴリ番号(action_doneのプログラム内参照: ["button","table","doll","fruit","drink"])を選択:id=0  
10. localのシミュレータのHSRが初期位置にいることを確認し,sigverse上のHSRと同様の姿勢に変更する.(初期姿勢が望ましい)  
11. 対象物のカテゴリに対応した動作Foldaより動作起動を選択する.対応したクラスとそのクラスの実行回数を選択する.  
例クラス０を０回,クラス1を1回,クラス2を０回選択する場合/traj=[0,1,0]  
12. localのipythonコンソールで,５の続きとして以下のコードを実行する.
action.action_crest(pos,traj,id)  
13. 動作ファイルが生成されていることを確認:(base.csvと関節の名称がそのまま付けられたcsvファイル)  
14. sigverseをmasterを持つコンソールで、python topicpublisher.py を実行する.：これで動作する.  

hsrの初期姿勢への戻し方  
1. ihsrbを起動する  
2. whole_body.move_to_neutral() を実行  
3. omni_base.go(0.,0.,0.,ralative=False) を実行  

