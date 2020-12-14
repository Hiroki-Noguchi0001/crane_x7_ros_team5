#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
import math
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String  # メッセージ型
from std_msgs.msg import Int32  # メッセージ型

from move_def import arm_move   # 指定座標に手先を動かす関数

flag = True # 動作フラグ

arm = moveit_commander.MoveGroupCommander("arm")

arm.set_max_velocity_scaling_factor(0.1) # 実行速度


def check(data):
    global flag, arm

    if data.data == "Check" and flag :
        rospy.loginfo("Start check")
        flag = False
        # -------------------
        pub = rospy.Publisher("report", Int32, queue_size = 1) # 動作報告パブリッシャ
        # --------------------
        while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
            rospy.sleep(1.0)
        rospy.sleep(1.0)
        # --------------------
        arm_initial_pose = arm.get_current_pose().pose # アーム初期ポーズを表示
        # --------------------
        # 文書確認の動作
        
        # SRDFに定義されている"home"の姿勢にする
        arm.set_named_target("home")
        arm.go()
        arm_move(0.24, 0.10, 0.2)

        arm_move(0.24, 0.0, 0.2) 
        # --------------------
        # 動作終了報告
        pub.publish(True)
        rospy.loginfo("Finish Check")
        # --------------------


def main():
    sub = rospy.Subscriber("name", String, check) # 動作指示サブスクライバ
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node("check", anonymous=True)
    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass