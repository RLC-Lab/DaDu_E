import rospy
import moveit_commander
import math
import time
import sys  


def initialize_moveit():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_to_goal', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "ur3_manipulator"  # Change to your planning group name
    move_group = moveit_commander.MoveGroupCommander(group_name)
    return move_group

def move_to_joint_goal(move_group, joint_goals):
    move_group.go(joint_goals, wait=True)
    move_group.stop()

def shutdown_moveit():
    moveit_commander.roscpp_shutdown()

def arm_detect():
    move_group = initialize_moveit()
    joint_goal = move_group.get_current_joint_values()
    
    rospy.loginfo(f"Current joint values: {joint_goal}")
    rospy.loginfo(f"Number of joints: {len(joint_goal)}")

    if len(joint_goal) >= 6:
        joint_goal[0] = math.radians(83)   # Set target for joint 1
        joint_goal[1] = math.radians(-90)  # Set target for joint 2
        joint_goal[2] = math.radians(45)   # Set target for joint 3
        joint_goal[3] = math.radians(-90)  # Set target for joint 4
        joint_goal[4] = math.radians(-90)  # Set target for joint 5
        joint_goal[5] = math.radians(180)  # Set target for joint 6

        move_to_joint_goal(move_group, joint_goal)
    else:
        rospy.logerr("Error: The joint goal list does not have enough elements.")
    
    shutdown_moveit()



if __name__ == "__main__":
    try:
        arm_detect()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
