#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
import math
from tf.transformations import quaternion_from_euler

import rospy
from std_msgs.msg import String  # メッセージ型
from std_msgs.msg import Int32  # メッセージ型

from move_def import arm_move   # 指定座標に手先を動かす関数
from move_def import hand_move  # ハンドの角度[rad]を指定し動かす関数
from move_def import joint_move # 指定関節の角度[deg]を指定し動かす関数

FinishFlag = False

def order(data):
    global FinishFlag
    FinishFlag = data.data  

def main():
    global FinishFlag 
    # --------------------
    rospy.init_node("crane_x7_pick_and_place_controller", anonymous=True)
    pub = rospy.Publisher("name", String, queue_size=1)
    sub = rospy.Subscriber("report", Int32, order)
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
    # --------------------
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.1)
    gripper = moveit_commander.MoveGroupCommander("gripper")
    # --------------------
    while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
        rospy.sleep(1.0)
    rospy.sleep(1.0)
    # --------------------
    print("Group names:")
    print(robot.get_group_names())

    print("Current state:")
    print(robot.get_current_state())
    # --------------------
    # アーム初期ポーズを表示
    arm_initial_pose = arm.get_current_pose().pose
    print("Arm initial pose:")
    print(arm_initial_pose)
    # --------------------
    # 挨拶
    pub.publish("Greet")
    while FinishFlag != True :
        pass
    # --------------------
    # 悪さ
    pub.publish("Artifice")
    FinishFlag = False
    while FinishFlag != True :
        pass    
    # --------------------
    # もう一度文書確認
    pub.publish("Check")
    FinishFlag = False
    while FinishFlag != True :
        pass
    # --------------------
    # 跳ね除ける
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
    # はんこを掴む
    pub.publish("Grab")
    FinishFlag = False
    while FinishFlag != True :
        pass
    # --------------------
    # 朱肉につけ確認する
    pub.publish("PushCheck")
    FinishFlag = False
    while FinishFlag != True :
        pass
    # --------------------
    # 捺印
    pub.publish("Seal")
    FinishFlag = False
    while FinishFlag != True :
        pass
    # --------------------
    # 拭く
    pub.publish("Wipe")
    FinishFlag = False
    while FinishFlag != True :
        pass
    # --------------------
    # はんこを戻す
    pub.publish("Release")
    FinishFlag = False
    while FinishFlag != True :
        pass
    # --------------------
    # ガッツポーズ
    pub.publish("GutsPose")
    FinishFlag = False
    while FinishFlag != True :
        pass
    # --------------------



if __name__ == '__main__':

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
