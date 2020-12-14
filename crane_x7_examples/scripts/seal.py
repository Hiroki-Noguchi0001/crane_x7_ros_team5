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
from move_def import joint_move # 指定関節の角度[deg]を指定し動かす関数

flag = True # 動作フラグ

arm = moveit_commander.MoveGroupCommander("arm")

arm.set_max_velocity_scaling_factor(0.1) # 実行速度


def seal(data):
    global flag, arm

    put_x = 0.20            # x座標[m]
    put_y = 0.0             # y座標[m]
    put_before_z = 0.20     # 押す前  z座標[m]
    put_z = 0.12            # 押す    z座標[m]
    push_z = 0.008          # 押し込みz座標[m]
    put_after_z = 0.20      # 押す後  z座標[m]

    if data.data == "Seal" and flag :
        rospy.loginfo("Start Seal")
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
        pub.publish(True)
        rospy.loginfo("Finish Seal")
        # --------------------


def main():
    sub = rospy.Subscriber("name", String, seal) # 動作指示サブスクライバ
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node("Seal", anonymous=True)
    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass