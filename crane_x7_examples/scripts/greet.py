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

from move_def import joint_move # 指定関節の角度[deg]を指定し動かす関数

flag = True # 動作フラグ

arm = moveit_commander.MoveGroupCommander("arm")

arm.set_max_velocity_scaling_factor(0.1) # 実行速度


def cd(data):
    global flag, arm

    joint3_deg = -45        # 3番目の関節角度[deg]
    joint2_deg = -45        # 2番目の関節角度[deg]

    if data.data == "Greet" and flag :
        rospy.loginfo("Start Greet")
        flag = False
        # -------------------
        pub = rospy.Publisher("report", Int32, queue_size = 1) # 動作報告パブリッシャ
        # --------------------
        while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
            rospy.sleep(1.0)
        rospy.sleep(1.0)
        # --------------------
        arm_initial_pose = arm.get_current_pose().pose # アーム初期ポーズを表示
        # -------------------
        # お辞儀
        arm.set_named_target("vertical")
        arm.go()
        joint_move(3,joint3_deg)
        rospy.sleep(1.0)
        for i in range(2):
            arm.set_named_target("vertical")
            arm.go()
            joint_move(2,joint2_deg)
            joint_move(3,joint3_deg)
            rospy.sleep(1.0)
            joint2_deg = joint2_deg + 90
        arm.set_named_target("vertical")
        arm.go()
        # --------------------
        # 動作終了報告
        pub.publish(True)
        rospy.loginfo("Finish Greet")
        # --------------------


def main():
    sub = rospy.Subscriber("name", String, cd) # 動作指示サブスクライバ
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node("Greet", anonymous=True)
    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
