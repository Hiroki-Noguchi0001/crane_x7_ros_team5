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


def wipe(data):
    global flag, arm

    wipe_x = 0.20           # x座標[m]
    wipe_y = 0.30           # y座標[m]
    wipe_before_z = 0.2     # 拭く前　z座標[m]　
    wipe_z = 0.12           # z座標[m]
    wipe_after_z = 0.2      # 拭いた後　z座標[m]

    if data.data == "Wipe" and flag :
        rospy.loginfo("Start Wipe")
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
        pub.publish(True)
        rospy.loginfo("Finish Wipe")
        # --------------------


def main():
    sub = rospy.Subscriber("name", String, wipe) # 動作指示サブスクライバ
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node("Wipe", anonymous=True)
    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
