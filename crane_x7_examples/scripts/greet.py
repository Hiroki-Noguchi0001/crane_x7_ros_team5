#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
import math
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Int32  # メッセージ型

turn = 1    # 動作実行順序
flag = True # 動作フラグ

arm = moveit_commander.MoveGroupCommander("arm")

arm.set_max_velocity_scaling_factor(0.1) # 実行速度


# 指定関節の角度[deg]を指定し動かす関数
def joint_move(joint_value,deg):
    global arm
    target_joint_values = arm.get_current_joint_values() # 現在角度をベースに、目標角度を作成する
    target_joint_values[joint_value] = arm.get_current_joint_values()[joint_value] + math.radians(deg)
    arm.set_joint_value_target(target_joint_values)
    arm.go()


def greet(data):
    global flag, trun, arm

    if data.data == turn and flag :
        rospy.loginfo("Start Greet")
        flag = False
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
        # 挨拶の動作
        arm.set_named_target("vertical")
        arm.go()

        joint_move(3,-45)
        rospy.sleep(1.0)

        arm.set_named_target("vertical")
        arm.go()
        rospy.sleep(1.0)

        joint_move(2,-45)
        joint_move(3,-45)
        rospy.sleep(1.0)

        arm.set_named_target("vertical")
        arm.go()
        rospy.sleep(1.0)

        joint_move(2,45)
        joint_move(3,-45)
        rospy.sleep(1.0)

        arm.set_named_target("vertical")
        arm.go()
        rospy.sleep(1.0)
        # --------------------
        # 動作終了報告
        pub.publish(turn)
        rospy.loginfo("Finish Greet")
        # --------------------


def main():
    sub = rospy.Subscriber("number", Int32, greet) # 動作指示サブスクライバ
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node("greet", anonymous=True)
    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass