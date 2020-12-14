#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
import math
from tf.transformations import quaternion_from_euler

arm = moveit_commander.MoveGroupCommander("arm")
gripper = moveit_commander.MoveGroupCommander("gripper")

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


# 指定関節の角度[deg]を指定し動かす関数
def joint_move(joint_value,deg):
    target_joint_values = arm.get_current_joint_values() # 現在角度をベースに、目標角度を作成する
    target_joint_values[joint_value] = arm.get_current_joint_values()[joint_value] + math.radians(deg)
    arm.set_joint_value_target(target_joint_values)
    arm.go()