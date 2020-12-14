#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
import math
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Int32  # メッセージ型

turn = 4    # 動作実行順序
flag = True # 動作フラグ

arm = moveit_commander.MoveGroupCommander("arm")

arm.set_max_velocity_scaling_factor(0.1) # 実行速度


# 指定座標に手先を動かす関数
def arm_move(x,y,z):
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = x
    target_pose.position.y = y
    target_pose.position.z = z
    q = quaternion_from_euler(- math.pi,0.0,- math.pi)
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)
    arm.go()


# 指定関節の角度[deg]を指定し動かす関数
def joint_move(joint_value,deg):
    target_joint_values = arm.get_current_joint_values() # 現在角度をベースに、目標角度を作成する
    target_joint_values[joint_value] = arm.get_current_joint_values()[joint_value] + math.radians(deg)
    arm.set_joint_value_target(target_joint_values)
    arm.go()


def Push_Check(data):
    global flag, trun, arm

    inkpad_x = 0.20         # x座標[m]
    inkpad_y = -0.15        # y座標[m]
    inkpad_before_z = 0.30  # 押す前  z座標[m]
    inkpad_z = 0.13         # 押す    z座標[m]
    inkpad_after_z = 0.30   # 押す後  z座標[m]

    inkpad_up_z = 0.01      # ポンポンする高さ[m]
    check_deg = 50          # 確認時回転角度[deg]

    if data.data == turn and flag :
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
        pub.publish(turn)
        rospy.loginfo("Finish Push and Check")
        # --------------------


def main():
    sub = rospy.Subscriber("number", Int32, Push_Check) # 動作指示サブスクライバ
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node("Push_Check", anonymous=True)
    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass