import rospy
import moveit_commander
import geometry_msgs.msg
import sys
import math

import subprocess

command_off = [
    'rostopic', 'pub', '-1', '/rh_p12_rn_position/command',
    'std_msgs/Float64', 'data: 0'
]


def arm_grasp(move_group):
    # moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    # group_name = "ur3_manipulator" 
    # move_group = moveit_commander.MoveGroupCommander(group_name)

    joint_goal = move_group.get_current_joint_values()

    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = math.radians(-90)
    joint_goal[1] = math.radians(-96)
    joint_goal[2] = math.radians(-35)
    joint_goal[3] = math.radians(-132)
    joint_goal[4] = math.radians(98)
    joint_goal[5] = math.radians(180) 

    plan = move_group.go(joint_goal, wait=True)
    move_group.stop()
    if plan:
        result = subprocess.run(command_off, capture_output=True, text=True)
        print("Place success")
    moveit_commander.roscpp_shutdown()

def arm_place(move_group):
    # robot = moveit_commander.RobotCommander()
    # scene = moveit_commander.PlanningSceneInterface()

    # joint_goal = move_group.get_current_joint_values()
    # joint_goal[0] = math.radians(83)   
    # joint_goal[1] = math.radians(-85)
    # joint_goal[2] = math.radians(85)   
    # joint_goal[3] = math.radians(-90)
    # joint_goal[4] = math.radians(-90)
    # joint_goal[5] = math.radians(180)

    # plan = move_group.go(joint_goal, wait=True)
    # move_group.stop()
    # if plan:
    #     result = subprocess.run(command_off, capture_output=True, text=True)
    #     print("Place success")
    # moveit_commander.roscpp_shutdown()

    result = subprocess.run(command_off, capture_output=True, text=True)
    print("Gripper on")

def grasp(pose_goal, move_group):
    execution_success = False
    print("Current pose:", pose_goal)

    move_group.set_pose_target(pose_goal)
    move_group.set_planning_time(60.0)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    if plan:
        result = subprocess.run(command_off, capture_output=True, text=True)
        print("Grasp success")
        execution_success = True
        return execution_success

    else:
        print("!!!!Plan failed!!!!")
