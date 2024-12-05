import numpy as np
from scipy.spatial.transform import Rotation as R
import rospy
import tf
import geometry_msgs.msg

from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal
import actionlib
import moveit_commander

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion


from geometry_msgs.msg import Pose
import geometry_msgs.msg
import sys
import math

from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool

import subprocess

# Global define of two data type
current_pose = None
current_orientation = None
tag = False

"""
    We need Tbe = Tbc*Tco/Tet

    Tbo = Tbc*Tco
    Tbo = Tbt
    Tbt = Tbe*Tet

    Tbe*Tet = Tbc*Tco
    Tbe = Tbc*Tco/Tet

    Tco: from Anygrasp
    Tbc: tf (current pose)
    Tet: tf (static)
    
"""

command_on = [
    'rostopic', 'pub', '-1', '/rh_p12_rn_position/command',
    'std_msgs/Float64', 'data: 0.0'
]
command_off = [
    'rostopic', 'pub', '-1', '/rh_p12_rn_position/command',
    'std_msgs/Float64', 'data: 0.75'
]

def callback_pose(msg):
    global current_pose
    global current_orientation
    global tag
    current_pose = msg.position
    current_orientation = msg.orientation
    if current_pose is not None and current_orientation is not None:
        tag = True
        # print(msg.position, msg.orientation)


def send_tf(translation_vector, quaternion_vector,frame_id,child_frame_id):
    br = tf.TransformBroadcaster()
    br.sendTransform(
            (translation_vector[0], translation_vector[1], translation_vector[2]), 
            (quaternion_vector[0], quaternion_vector[1], quaternion_vector[2], quaternion_vector[3]),
                
        rospy.Time.now(), 
        child_frame_id,  
        frame_id
        )

def transform_to_pose(transform_listener):
    global current_pose
    global current_orientation

    temp_pose = current_pose
    temp_orientation =current_orientation
    

    (trans_bb, rot_bb) = transform_listener.lookupTransform('/base_link', '/ur3_base_link', rospy.Time(0))
    rotation_matrix_bb = R.from_quat(rot_bb).as_matrix()
    Tbb = np.eye(4)
    Tbb[:3, :3] = rotation_matrix_bb
    Tbb[:3, 3] = trans_bb
    print("Tbb:\n", Tbb)

    # base_link->top_plate_link
    (trans_bc, rot_bc) = transform_listener.lookupTransform('/ur3_base_link', '/g_camera_color_optical_frame', rospy.Time(0))
    # (trans_bc, rot_bc) = transform_listener.lookupTransform('/ur3_base_link', '/g_camera_depth_optical_frame', rospy.Time(0))
    rotation_matrix_bc = R.from_quat(rot_bc).as_matrix()
    Tbc = np.eye(4)
    Tbc[:3, :3] = rotation_matrix_bc
    Tbc[:3, 3] = trans_bc
    print("Tbc:\n", Tbc)

    (trans_be, rot_be) = transform_listener.lookupTransform('/ur3_base_link', '/ee_link', rospy.Time(0))
    rotation_matrix_be = R.from_quat(rot_be).as_matrix()
    Tbe = np.eye(4)
    Tbe[:3, :3] = rotation_matrix_be
    Tbe[:3, 3] = trans_be
    print("Tbe:\n", Tbe)       


    (trans_et, rot_et) = transform_listener.lookupTransform('/ee_link', '/rh_p12_rn_r2', rospy.Time(0))


    Tet = np.eye(4)
    # Tet[:3, 3] = (7.83546234e-02,0 ,0)
    Tet[:3, 3] = (0.098,0 ,0)
    Tet1 = np.linalg.inv(Tet)


    

    # Get Tco
    q_x, q_y, q_z, q_w = temp_orientation.x, temp_orientation.y, temp_orientation.z, temp_orientation.w
    p_x, p_y, p_z = temp_pose.x, temp_pose.y, temp_pose.z
    Rotate = np.array([
    [1 - 2*q_y**2 - 2*q_z**2, 2*q_x*q_y - 2*q_z*q_w, 2*q_x*q_z + 2*q_y*q_w],
    [2*q_x*q_y + 2*q_z*q_w, 1 - 2*q_x**2 - 2*q_z**2, 2*q_y*q_z - 2*q_x*q_w],
    [2*q_x*q_z - 2*q_y*q_w, 2*q_y*q_z + 2*q_x*q_w, 1 - 2*q_x**2 - 2*q_y**2]])

    Tco = np.zeros((4, 4))
    Tco[:3, :3] = Rotate
    Tco[:3, 3] = [p_x, p_y, p_z]
    Tco[3, 3] = 1
    print("Tco:\n", Tco)

    rotation_matrix_y = np.array([ # turn 90 degree
            [0, 0, 1],
            [0, 1, 0],
            [-1, 0, 0]
        ])
    matrix_orientation_y =  np.eye(4)
    matrix_orientation_y[:3, :3] = rotation_matrix_y 

    rotation_matrix_z = np.array([
        [-1, 0, 0],
        [0, -1, 0],
        [0, 0, 1]
        ])
    matrix_orientation_z =  np.eye(4)
    matrix_orientation_z[:3, :3] = rotation_matrix_z

    rotation_matrix_x = np.array([
        [-1, 0, 0],
        [0, -1, 0],
        [0, 0, 1]
        ])
    matrix_orientation_x =  np.eye(4)
    matrix_orientation_x[:3, :3] = rotation_matrix_x

    Tbo = Tbc @ Tco 
    print("Tbo:\n", Tbo)

    Tbe_new = Tbb @ Tbo @ Tet1  @ matrix_orientation_y
    print("Tbe_New:\n", Tbe_new)

    translation_vector = Tbe_new[:3, 3]
    print("translation_vector:\n", translation_vector)
    rotation_matrix = Tbe_new[:3, :3]

    rotation_Tbe_new = R.from_matrix(rotation_matrix)
    quaternion_Tbe_new = rotation_Tbe_new.as_quat()  # 返回格式为 (x, y, z, w)
    print("quaternion_Tbe_new:\n", quaternion_Tbe_new)
    print("Tbe:\n", Tbe)
    send_tf(translation_vector, quaternion_Tbe_new,"base_link","Tbe_new")

    Tbe = Tbb @ Tbe
    trans_vector = Tbe[:3, 3]
    rot_matrix = Tbe[:3, :3]
    rot_Tbe = R.from_matrix(rot_matrix)
    quat_Tbe = rot_Tbe.as_quat()  # 返回格式为 (x, y, z, w)
    send_tf(trans_vector, quat_Tbe,"base_link","Tbe")
    print("quat_Tbe:\n", quat_Tbe)


    return translation_vector, quaternion_Tbe_new

def init_grasp_arm():

    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "ur3_manipulator"  
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # TODO: Init arm old grasp pose:
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = math.radians(-90)
    joint_goal[1] = math.radians(-96)
    joint_goal[2] = math.radians(-35)
    joint_goal[3] = math.radians(-132) 
    joint_goal[4] = math.radians(90) 
    joint_goal[5] = math.radians(180)

    move_group.go(joint_goal, wait=True)
    move_group.stop()


def main():
    global current_pose
    global current_orientation
    global tag
    rospy.init_node('arm_controller')
    print('Start grasp listener...')
    tf_listener = tf.TransformListener()

    # get grasp init data
    anygrasp = rospy.Subscriber('/detect_grasps/pose_grasps', Pose, callback_pose)

    # publish plan success singal
    plan_pub = rospy.Publisher('/plan_success', Bool, queue_size=10)

    rate = rospy.Rate(10)
    
    print("Got grasping init data...")

    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "ur3_manipulator"  
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # init_grasp_arm()
  
    while anygrasp:
        # input('press any key for grasp .........')
        if tag == True:
            tag = False
            execution_success = False

            translation_vector, quaternion_Tbe_new = transform_to_pose(tf_listener)
            x_transformed, y_transformed, z_transformed = translation_vector[:3]
            print("transformed postion:\n", x_transformed, y_transformed, z_transformed)
            x0_transformed, y0_transformed, z0_transformed, w0_transformed = quaternion_Tbe_new[:4]
            rospy.sleep(1)

           
            end_effector_pose = move_group.get_current_pose()
            print("Current pose is:", end_effector_pose)
            
            pose_goal = geometry_msgs.msg.Pose()
            pose_goal = end_effector_pose.pose

            t_vec = (pose_goal.position.x,pose_goal.position.y,pose_goal.position.z)
            r_quat = (pose_goal.orientation.x,pose_goal.orientation.y,pose_goal.orientation.z,pose_goal.orientation.w)
            send_tf(t_vec, r_quat,"base_link","ee_pose")


            pose_goal.orientation.x = x0_transformed
            pose_goal.orientation.y = y0_transformed
            pose_goal.orientation.z = z0_transformed
            pose_goal.orientation.w = w0_transformed
            pose_goal.position.x = x_transformed
            pose_goal.position.y = y_transformed
            pose_goal.position.z = z_transformed

            move_group.set_pose_target(pose_goal)
            move_group.set_planning_time(30.0)
            # Strat plannning
            plan = move_group.go(wait=True)
            move_group.stop()
            move_group.clear_pose_targets()

            if plan:
                subprocess.run(command_off, capture_output=True, text=True)
                print("Grasp success")
                execution_success = True

                plan_pub.publish(True)
                init_grasp_arm()
                print("Arm init success")

            else:
                print("!!!!Plan failed!!!!")
            
        rate.sleep()
    
    return execution_success


if __name__ == '__main__':
    main()