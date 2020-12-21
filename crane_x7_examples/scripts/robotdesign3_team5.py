#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
from tf.transformations import quaternion_from_euler

import rospy
from std_msgs.msg import String  # メッセージ型
from std_msgs.msg import Int32  # メッセージ型

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
    #no-realsense-ver
    #ActionNames = ["Greet", "Artifice", "Check", "Exclusion", "PushCheck", "Seal", "Wipe", "Release", "GutsPose"]

    rospy.sleep(5)
    #realsense-ver
    ActionNames = ["Greet", "Artifice", "Check", "Exclusion", "Detect", "PushCheck", "Seal", "Wipe", "Release", "GutsPose"]
    
    for i in range(len(ActionNames)):
        pub.publish(ActionNames[i])

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
