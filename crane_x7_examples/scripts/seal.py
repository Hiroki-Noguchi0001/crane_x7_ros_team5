#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
import math
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Int32  # メッセージ型

turn = 5    # 動作実行順序
flag1 = True # 動作フラグ

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


def seal(data):
    global flag1, trun, arm

    put_x = 0.20            # x座標[m]
    put_y = 0.0             # y座標[m]
    put_before_z = 0.20     # 押す前  z座標[m]
    put_z = 0.12            # 押す    z座標[m]
    push_z = 0.008          # 押し込みz座標[m]
    put_after_z = 0.20      # 押す後  z座標[m]

    if data.data == turn and flag1 :
        rospy.loginfo("Start Seal")
        flag1 = False
        # -------------------
        pub = rospy.Publisher("report", Int32, queue_size = 1) # 動作報告パブリッシャ
        # -------------------
        # 動作開始報告
        report_num = turn - 1
        pub.publish(report_num)
        # --------------------
        while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
            rospy.sleep(1.0)
        rospy.sleep(1.0)
        # --------------------
        arm_initial_pose = arm.get_current_pose().pose # アーム初期ポーズを表示
        # --------------------
        # 捺印
        arm.set_named_target("before_stamping_position") # 姿勢を指定
        arm.go()

        arm_move(put_x, put_y, put_z)
        arm_move(put_x, put_y, put_z - push_z)
        arm_move(put_x, put_y, put_z)
        rospy.sleep(0.5)
        
        arm_move(put_x, put_y, put_after_z)
        # --------------------
        # 動作終了報告
        pub.publish(turn)
        rospy.loginfo("Finish Seal")
        # --------------------

    if data.data == turn :
        flag1 = True


def main():
    sub = rospy.Subscriber("number", Int32, seal) # 動作指示サブスクライバ
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node("Seal", anonymous=True)
    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass