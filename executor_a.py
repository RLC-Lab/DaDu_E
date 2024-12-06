# executor.py

from navigation import Navigation
from grasp.grasping import Grasping
from image import ImageSaver
import time
import subprocess
from std_srvs.srv import Empty,EmptyResponse
# from openai import OpenAI
import json
import base64
import json
import requests
import rospy
import moveit_commander


import sys
import os
folder1_dir = os.path.join('/home/hs/disk1/sunwenhao/grasp/src')
sys.path.append(folder1_dir)

from adjust_base import adjust_main

class Executor:

    def __init__(self):

        rospy.init_node('embody', anonymous=True)

        self.is_hold = False
        self.return_pic = False

        self.navigation = Navigation()
        self.grasping = Grasping()
        self.image_saver = ImageSaver()

    def format_custom_input(self,role,custom_text):
        formatted_output = {
            "role": "user" if role==1 else "assistant",
            "content": [
                {
                    "type": "text",
                    "text": custom_text
                },
            ]
        }

        return formatted_output
    

    def execute(self):

        # moveit config
        moveit_commander.roscpp_initialize(sys.argv)
        group_name = "ur3_manipulator" 
        move_group = moveit_commander.MoveGroupCommander(group_name)
       
        while True:
            type = input("action_type(1_navi 2_pick 3_place 4_done): ")
            param = input("param: ")
            
            if type == '1':
                action = f"goto[table][{param}]"
            elif type == '2':
                action = f"pick_up[{param}]"
            elif type == '3':
                action = f"place[{param}]"
            else:   
                break

            action_type = action[:2]

            print(action)

            # call api

            if action_type =='do':
                break

            elif action_type == 'go':
                print('navigatoin')
                action = action.replace(",", " ")
                param = action.split('[')
                
                print(action)
                coordinates = param[2].strip(']').split()
                x = float(coordinates[0])
                y = float(coordinates[1])
                print(x,y)

                api_feedback = self.navigation.navi_xy_goto(coordinates)

                if not self.is_hold:
                    print('return current view')
                    # ImageSaver.run()


                    # utilise vision
                    # self.return_pic = True

            elif action_type =='pi':
            
                print('grasping')
                param = action.split('[')
                object = param[1].strip(']')
                print(object)

                # refinement 1
                gpt_direction = int(input('gpt4o return ideal direction:'))
                print(self.navigation.last_direction)
                print('adjust base for better grasping side')
                if gpt_direction!=4:
                    refine_direction = (gpt_direction+self.navigation.last_direction)%4
                    print(refine_direction)
                    self.navigation.navi_xy_goto(self.navigation.last_location,refine_direction)

                else:
                    print("No need to adjust side")

                # CAP baselline doesn' t need following adjust method
                # adjust 2
                print('adjust base for better grasping distance')
                adjust_main(object, move_group)
                print("finish adjust base for grasping distance")


                # grasping
                print("grasping execing...")
                api_feedback = self.grasping.pick(object, move_group)

                if True:
                    self.is_hold = True

            else:
                print('placing')
                api_feedback = self.grasping.place(move_group)


if __name__ == "__main__":
    executor = Executor()
    executor.execute()