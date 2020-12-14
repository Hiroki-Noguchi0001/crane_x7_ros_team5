#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
import math
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Int32  # メッセージ型

turn = 6    # 動作実行順序
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


def wipe(data):
    global flag, trun, arm

    if data.data == turn and flag :
        rospy.loginfo("Start Wipe")
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
        # -------------------
        # 拭く
        wipe_x = 0.20           # x座標[m]
        wipe_y = 0.30           # y座標[m]
        wipe_before_z = 0.2     # 拭く前　z座標[m]　
        wipe_z = 0.12           # z座標[m]
        wipe_after_z = 0.2      # 拭いた後　z座標[m]
        # --------------------
        # 拭く
        arm_move(wipe_x, wipe_y, wipe_before_z)
        for i in range(6):
            arm_move(wipe_x, wipe_y, wipe_z)
            rospy.sleep(0.5)
            if i % 2 == 0:
                wipe_x = wipe_x + 0.02
            else:
                wipe_x = wipe_x - 0.02
        arm_move(wipe_x, wipe_y, wipe_after_z)
        # --------------------
        # 動作終了報告
        pub.publish(turn)
        rospy.loginfo("Finish Wipe")
        # --------------------


def main():
    sub = rospy.Subscriber("number", Int32, wipe) # 動作指示サブスクライバ
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node("Wipe", anonymous=True)
    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
