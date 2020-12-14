#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
import math
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Int32  # メッセージ型

turn1 = 3    # 動作実行順序1
turn2 = 7    # 動作実行順序2
flag1 = True # 動作フラグ1
flag2 = True # 動作フラグ2

arm = moveit_commander.MoveGroupCommander("arm")
gripper = moveit_commander.MoveGroupCommander("gripper")

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


# ハンドの角度[rad]を指定し動かす関数
def hand_move(rad):
    gripper.set_joint_value_target([rad, rad])
    gripper.go()


def grab_release(data):
    global flag1, trun1, flag2, trun2, arm

    seal_x = 0.30           # x座標[m]
    seal_y = -0.15          # y座標[m]
    seal_before_z = 0.30    # 掴む前  Z座標[m]
    seal_z = 0.135          # 掴む    Z座標[m]
    seal_after_z = 0.30     # 掴む後  Z座標[m]
    seal_close = 0.10       # 掴む角度[rad]

    hand_open = math.pi/4   # ハンド 開く角度[rad]

    pub = rospy.Publisher("report", Int32, queue_size = 1) # 動作報告パブリッシャ

    if data.data == turn1 and flag1 :
        rospy.loginfo("Start Grab")
        flag1 = False
        # -------------------
        # 動作開始報告
        report_num = turn1 - 1
        pub.publish(report_num)
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
        pub.publish(turn1)
        rospy.loginfo("Finish Grab")
        # --------------------

    if data.data == turn2 and flag2 :
        rospy.loginfo("Start Release")
        flag2 = False
        # -------------------
        # 動作開始報告
        report_num = turn2 - 1
        pub.publish(report_num)
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
        pub.publish(turn2)
        rospy.loginfo("Finish Release")
        # --------------------


def main():
    sub = rospy.Subscriber("number", Int32, grab_release) # 動作指示サブスクライバ
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node("Grab_Release", anonymous=True)
    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass