# grasping.py
import random
import time
from geometry_msgs.msg import Twist
import rospy
import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool

import sys
import os
import subprocess
import threading

import grasp_utils
import moveit_commander

from arm_eval_pose import arm_grasp, arm_place, grasp


command_on = [
    'rostopic', 'pub', '-1', '/rh_p12_rn_position/command',
    'std_msgs/Float64', 'data: 0.0'
]
command_off = [
    'rostopic', 'pub', '-1', '/rh_p12_rn_position/command',
    'std_msgs/Float64', 'data: 0.7'
]


import camera_demo
import test_camera_convert

parser = camera_demo.argparse.ArgumentParser()
parser.add_argument(
    "--checkpoint_path",
    default="./checkpoints/checkpoint_detection.tar",
    help="Model checkpoint path",
)
parser.add_argument(
    "--max_gripper_width",
    type=float,
    default=0.085,
    help="Maximum gripper width (<=0.1m)",
)
parser.add_argument("--gripper_height", type=float, default=0.03, help="Gripper height")
parser.add_argument("--port", type=int, default=5556, help="port")
# parser.add_argument(
#     "--top_down_grasp", action="store_true", help="Output top-down grasps"
# )
parser.add_argument("--top_down_grasp", action="store_true", default=True, help="Output top-down grasps")

parser.add_argument("--debug", action="store_true", help="Enable visualization")
parser.add_argument("--headless", action="store_true", help="Enable headless mode")
parser.add_argument(
    "--open_communication",
    action="store_true",
    help="Use image transferred from the robot",
)
parser.add_argument(
    "--max_depth", type=float, default=2.0, help="Maximum depth of point cloud"
)
parser.add_argument(
    "--min_depth", type=float, default=0, help="Maximum depth of point cloud"
)
parser.add_argument(
    "--sampling_rate", type=float, default=1.0, help="Sampling rate of points [<= 1]"
)
parser.add_argument("--environment", default="./example_data", help="Environment name")
cfgs = parser.parse_args()
cfgs.max_gripper_width = max(0, min(0.2, cfgs.max_gripper_width))


class Grasping:

    def __init__(self):
        camera_demo.check_license_folder()
        self.object_handler = camera_demo.CameraObjectHandler(cfgs)
        self.plan_success_flag = False

    def plan_callback(self, msg):
        if msg.data:
            print("Plan executed successfully, proceeding with the next steps.")
            self.plan_success_flag = True
        else:
            print("Plan execution failed, handle accordingly.")
            self.plan_success_flag = False
        
    def pick(self, target, move_group):
        # grasp pose
        print("init grasp pose ...")
        arm_grasp(move_group)

        print("start grasping ... ")
        print("target: ",target)
        self.object_handler.manipulate(target)
        
        # plan_callback = rospy.Subscriber("/detect_grasps/pose_grasps", Pose, self.plan_callback)
        # while not self.plan_success_flag:
        #     print("waiting for plan to execute")
        #     rospy.sleep(0.1)

        print("Grasping")
        time.sleep(120)
        
        print("Plan successfully executed.")
        # arm_grasp(move_group)
        grasp_utils.move_backward()
        print("pick up successful")
        return "pick up successful"
        
    
    def place(self, move_group):

        grasp_utils.move_forward()
        arm_place(move_group)
        result = subprocess.run(command_on, capture_output=True, text=True)
        grasp_utils.move_backward()
        
        return "place successful"    

if __name__ == "__main__":
    
    rospy.init_node('send_move_base_goal', anonymous=True)
    grasper = Grasping()
    grasper.pick("banana")


