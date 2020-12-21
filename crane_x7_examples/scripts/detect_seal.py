#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
from geometry_msgs.msg import Pose2D
import rosnode
import math
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Int32
from std_msgs.msg import String

from move_def import arm_move
from move_def import hand_move

robot = moveit_commander.RobotCommander()
arm = moveit_commander.MoveGroupCommander("arm")
arm.set_max_velocity_scaling_factor(0.1)
#gripper = moveit_commander.MoveGroupCommander("gripper")

while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
    rospy.sleep(1.0)
rospy.sleep(1.0)

flag = True
mode = True

see_x = 0.10
see_y = -0.25
see_z = 0.3
pick_z = 0.135
x_max = 0.3
y_max = 0.3
close = 0.10


def main2():
    print("Group names:")
    print(robot.get_group_names())

    print("Current state:")
    print(robot.get_current_state())

    # アーム初期ポーズを表示
    arm_initial_pose = arm.get_current_pose().pose
    print("Arm initial pose:")
    print(arm_initial_pose)

    hand_move(0.7)

    #ハンコを探すための位置
    print("serch")
    arm_move(see_x, see_y, see_z)
    rospy.sleep(0.5)

def pick_seal():
    print("持つ")
    global see_x, see_y, pick_z, close, flag
    pub = rospy.Publisher("report", Int32, queue_size = 1)
    see_y += 0.025
    arm_move(see_x, see_y, pick_z)
    hand_move(close)
    flag = False
    pub.publish(True)
    rospy.loginfo("Finish Detect seal")


def pickcallback(pose):
    limit = pose.theta
    pose_x = pose.x
    pose_y = pose.y
    global see_x , see_y, mode, x_max, y_max

    if(mode == True):
        hand_move(0.7)
        print("zahyou", pose_x, pose_y)
        if(limit == 0):
            print("探す")
            arm_move(see_x, see_y, see_z)
            see_x += 0.01
            if(see_x >= x_max):
                see_y += 0.01
                see_x = 0.10
            if(see_y >= y_max):
                see_y = -0.25

        elif(pose_x < 20):
            print(1)
            see_x -= 0.01
        elif(pose_x > 50):
            print(2)
            see_x += 0.01
        elif(pose_y < -80):
            print(3)
            see_y -= 0.01
        elif(pose_y > -50):
            print(4)
            see_y += 0.01
        else:
            print(5)
            mode = False
            pick_seal()

        print(see_x, see_y, see_z)

        arm_move(see_x, see_y, see_z)
        rospy.sleep(1.0)
        print("pickcallback,end")

    else:
        pass

def main(data):
    if(data.data == "Detect" and flag):
        rospy.loginfo("Start Detect seal")
        sub = rospy.Subscriber("pose", Pose2D, pickcallback, queue_size=1)
    else:
        pass

if __name__ == "__main__":
    rospy.init_node("detect_seal")
    main2()

    try:
        if not rospy.is_shutdown():
            sub = rospy.Subscriber("name", String, main)
            rospy.spin()
    except rospy.ROSInterruptException:
        pass
