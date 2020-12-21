#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
import math
from tf.transformations import quaternion_from_euler

from std_msgs.msg import String
from std_msgs.msg import Int32

from move_def import arm_move   # 指定座標に手先を動かす関数
from move_def import joint_move # 指定関節の角度[deg]を指定し動かす関数

flag = True # 動作フラグ

arm = moveit_commander.MoveGroupCommander("arm")

arm.set_max_velocity_scaling_factor(0.1) # 実行速度


def cd(data):
    global flag, arm

    inkpad_x = 0.20         # x座標[m]
    inkpad_y = -0.15        # y座標[m]
    inkpad_before_z = 0.30  # 押す前  z座標[m]
    inkpad_z = 0.13         # 押す    z座標[m]
    inkpad_after_z = 0.30   # 押す後  z座標[m]

    inkpad_up_z = 0.01      # ポンポンする高さ[m]
    check_deg = 50          # 確認時回転角度[deg]

    if data.data == "PushCheck" and flag :
        rospy.loginfo("Start Push and Check")
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
        # 朱肉につけ確認する
        for i in range(2):
            arm_move(inkpad_x, inkpad_y, inkpad_before_z)
            # --------------------
            # 朱肉にはんこを数回押し付ける
            arm.set_max_velocity_scaling_factor(1.0)
            arm_move(inkpad_x, inkpad_y, inkpad_z)
            for j in range(2):
                arm_move(inkpad_x, inkpad_y, inkpad_z + inkpad_up_z)
                arm_move(inkpad_x, inkpad_y, inkpad_z)
            # --------------------
            arm.set_max_velocity_scaling_factor(0.1)
            # 持ち上げる
            arm_move(inkpad_x, inkpad_y, inkpad_after_z)

            # 確認する
            joint_move(4,check_deg)
        # --------------------
        # 動作終了報告
        pub.publish(True)
        rospy.loginfo("Finish Push and Check")
        # --------------------


def main():
    sub = rospy.Subscriber("name", String, cd) # 動作指示サブスクライバ
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node("PushCheck", anonymous=True)
    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass