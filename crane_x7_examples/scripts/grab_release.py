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
from move_def import hand_move  # ハンドの角度[rad]を指定し動かす関数

flag0 = True # 動作フラグ0
flag1 = True # 動作フラグ1
flag2 = True # 動作フラグ2

arm = moveit_commander.MoveGroupCommander("arm")
gripper = moveit_commander.MoveGroupCommander("gripper")

arm.set_max_velocity_scaling_factor(0.1) # 実行速度


def grab_release(data):
    global flag0, flag1, flag2, arm

    seal_x = 0.30           # x座標[m]
    seal_y = -0.15          # y座標[m]
    seal_before_z = 0.30    # 掴む前  Z座標[m]
    seal_z = 0.135          # 掴む    Z座標[m]
    seal_after_z = 0.30     # 掴む後  Z座標[m]
    seal_close = 0.10       # 掴む角度[rad]

    hand_open = math.pi/4   # ハンド 開く角度[rad]

    pub = rospy.Publisher("report", Int32, queue_size = 1) # 動作報告パブリッシャ

    if data.data == "Exclusion" and flag0 :
        rospy.loginfo("Start Exclusion")
        flag0 = False
        # --------------------
        while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
            rospy.sleep(1.0)
        rospy.sleep(1.0)
        # --------------------
        arm_initial_pose = arm.get_current_pose().pose # アーム初期ポーズを表示
        # --------------------
        # 跳ね除ける
        push_x1 = 0
        push_x2 = 0.10
        push_x3 = 0.20
        push_y1 = -0.25
        push_y2 = -0.20
        push_before_z = 0.20
        push_after_z = 0.12
        hand_open = math.pi/4   # ハンド 開く角度[rad]

        arm.set_named_target("home")
        arm.go()
        arm_move(push_x1, push_y1, push_before_z)
        hand_move(hand_open)
        arm_move(push_x1, push_y1, push_after_z)
        rospy.sleep(0.5)
        arm_move(push_x1, push_y1, push_before_z)
        for n in range(4):
            if n % 2 == 0:
                arm_move(push_x2, push_y1, push_before_z)
                arm_move(push_x2, push_y1, push_after_z)
                rospy.sleep(0.5)
                arm_move(push_x2, push_y1, push_before_z)
            else:
                arm_move(push_x3, push_y1, push_before_z)
                rospy.sleep(0.5)
        hand_move(0.1)
        arm_move(push_x3, push_y2, push_before_z)
        for i in range(10):
            arm_move(push_x3, push_y2, push_after_z)
            push_y2 = push_y2 - 0.01
        # --------------------
        # 動作終了報告
        pub.publish(True)
        rospy.loginfo("Finish Exclusion")
        # --------------------

    if data.data == "Grab" and flag1 :
        rospy.loginfo("Start Grab")
        flag1 = False
        # --------------------
        while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
            rospy.sleep(1.0)
        rospy.sleep(1.0)
        # --------------------
        arm_initial_pose = arm.get_current_pose().pose # アーム初期ポーズを表示
        # --------------------
        # 掴むの動作
        hand_move(hand_open)    # ハンドを開く

        # arm_move(seal_x, seal_y, seal_before_z)   # はんこ上まで移動
        arm.set_named_target("pick_seal_position") 
        arm.go()

        arm_move(seal_x, seal_y, seal_z)    # はんこを掴む位置まで移動

        hand_move(seal_close)   # はんこを掴む

        arm_move(seal_x, seal_y, seal_after_z)  # はんこを持ち上げる
        # --------------------
        # 動作終了報告
        pub.publish(True)
        rospy.loginfo("Finish Grab")
        # --------------------

    if data.data == "Release" and flag2 :
        rospy.loginfo("Start Release")
        flag2 = False
        # --------------------
        while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
            rospy.sleep(1.0)
        rospy.sleep(1.0)
        # --------------------
        arm_initial_pose = arm.get_current_pose().pose # アーム初期ポーズを表示
        # --------------------
        # 掴むの動作

        # arm_move(seal_x, seal_y, seal_before_z)
        arm.set_named_target("pick_seal_position") # はんこ上まで移動
        arm.go()

        arm_move(seal_x, seal_y, seal_z) # はんこをはなす位置まで移動

        hand_move(hand_open) # はんこをはなす
        # --------------------
        # 動作終了報告
        pub.publish(True)
        rospy.loginfo("Finish Release")
        # --------------------


def main():
    sub = rospy.Subscriber("name", String, grab_release) # 動作指示サブスクライバ
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node("Grab_Release", anonymous=True)
    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass