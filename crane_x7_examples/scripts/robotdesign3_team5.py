#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
import math
from tf.transformations import quaternion_from_euler

import rospy
from std_msgs.msg import Int32

Moveflag = 0

def order(data):
    global Moveflag
    Moveflag = data.data  

def main():
    # --------------------
    rospy.init_node("crane_x7_pick_and_place_controller", anonymous=True)
    pub = rospy.Publisher("number", Int32, queue_size=1)
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
    # --------------------
    # ハンドの角度[rad]を指定し動かす関数
    def hand_move(rad):
        gripper.set_joint_value_target([rad, rad])
        gripper.go()
    # --------------------
    # 指定関節の角度[deg]を指定し動かす関数
    def joint_move(joint_value,deg):
        target_joint_values = arm.get_current_joint_values() # 現在角度をベースに、目標角度を作成する
        target_joint_values[joint_value] = arm.get_current_joint_values()[joint_value] + math.radians(deg)
        arm.set_joint_value_target(target_joint_values)
        arm.go()
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
    pub.publish(1)
    while Moveflag != 1 :
        pass
    
    # --------------------
    
    #紙を見つける
    arm.set_named_target("home")
    arm.go()
    arm_move(0.2 , 0.05, 0.3)
    
    #周囲に誰もいないか確認
    arm.set_named_target("home")
    arm.go()
    
    joint_move(0,-80)
    joint_move(0,160)
    joint_move(0,-160)
    
    # --------------------
    # もう一度文書確認
    pub.publish(2)
    while Moveflag != 2 :
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
    pub.publish(3)
    while Moveflag != 3 :
        pass
    # --------------------
    # 朱肉につけ確認する
    pub.publish(4)
    while Moveflag != 4 :
        pass
    # --------------------
    # 捺印
    pub.publish(5)
    while Moveflag != 5 :
        pass
    # --------------------
    # 拭く
    pub.publish(6)
    while Moveflag != 6 :
        pass
    # --------------------
    # はんこを戻す
    pub.publish(7)
    while Moveflag != 7 :
        pass
    # --------------------
    # ガッツポーズ
    pub.publish(8)
    while Moveflag != 8 :
        pass
    # --------------------



if __name__ == '__main__':

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
