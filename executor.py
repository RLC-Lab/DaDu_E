# executor.py

from navigation import Navigation
from grasping import Grasping
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
    
    def format_custom_input_vision(self,base64_image,custom_text):
        formatted_output =  {
            "role": "user",
            "content": [
                {
                "type": "text",
                "text": custom_text
                },
                {
                "type": "image_url",
                "image_url": {
                    "url": f"data:image/jpeg;base64,{base64_image}"
                }
                }
            ]
            }

        return formatted_output

    def execute(self, instruction):
        


        api_key='YOUR API KEY HERE'

        # prompt
        messages = [
                {
                "role": "system",
                "content": [
                    {
                    "type": "text",
                    "text": "#CONTEXT# You are highly skilled in robotic task planning, breaking down intricate and long-term tasks into distinct primitive actions. The robot has a mobile base and one arm, and here is the semantic map of the environment, for each line, it represents an object and its corresponding location: —MAP— [drink table] [6.96548 -4.40075] [toy rack] [-6.83587 -4.10468] [storage rack] [7.65881 5.69058] [book rack] [-4.66216 3.40295] [fruit table] [0.610712 5.894910] —MAP— When given a language instruction, you are required to break them into sub-tasks, for each subtask, you should list a set of skills to meet the goal. —SKILL— go_to[table][coordinate] pick_up[obj] place[obj] done —SKILL— you must strictly obey these rules with the exact output form above, you can only output [coordinate] in go to subtasks. #OBECTIVE# When given a language instruction, you are required to break them into subtasks, for each subtask, you should list a set of skills to meet the goal. You should try all possible orders/combinations of the subtasks, and choose the one of which the total path is minimum. You must strictly obey these rules with the exact output form above. First, you should output the reasoning part and output the best order to execute the instruction. In the reasoning part, you should think step by step, and try all possible orders/combinations of the subtasks, choose the one of which the total path is minimum. The initial location is [0,0]. Then, you need to out put the first skill to execute, and output one corresponding skill according to the user’ feedback. You can adjust the skill according to the feedback. #OUTPUT# All of your output should be in json format. At the first time, you should output\nreasoning 2.action list of the best order 3.the first action.\nThen, according to the user’s feed back, you should output 1.reasoning 2.next action to execute.\nExcept for the reasoning part, the output should be compose of skills."
                    }
                ]
                },
                
            ]
        messages.append(self.format_custom_input(1,instruction))

        while True:
            
            # LLM planning
            headers = {
            "Content-Type": "application/json",
            "Authorization": f"Bearer {api_key}"
            }

            payload = {
            "model": "gpt-3.5-turbo",
            "messages": messages,
            "max_tokens": 300,
            "temperature":1
            }
            json_response = requests.post("https://api.openai.com/v1/chat/completions", headers=headers, json=payload)

            # parse response
            data = json_response.json()
            gpt_response = data['choices'][0]['message']['content']
            gpt_content = json.loads(gpt_response)
            action_field = None
            if "first_action" in gpt_content:
                action_field = "first_action"
            elif "next_action" in gpt_content:
                action_field = "next_action"
            action = gpt_content[action_field]
            print(action)
            action_type = action[:2]

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
                gpt_direction = int(input('# gpt4o return ideal direction:'))

                # 1 left 2 opposite 3 right
                print(self.navigation.last_direction)
                if gpt_direction!=4: # return 1,2,3
                    
                    refine_direction = (gpt_direction+self.navigation.last_direction)%4
                    print(refine_direction)
                    self.navigation.navi_xy_goto(self.navigation.last_location,refine_direction)


                # adjust 2

                print('adjust')
                adjust_main(object)


                # grasping
                api_feedback = self.grasping.pick(object)

                

                if True:
                    self.is_hold = True

            else:

                print('placing')

                api_feedback = self.grasping.place()

                if True:
                    self.is_hold = False

            # update history
            messages.append(self.format_custom_input(0,gpt_response))
            

            if self.return_pic == True:
                base64_image = self.encode_image(self.save_dir+self.file_name)
                messages.append(self.format_custom_input_vision(base64_image,api_feedback))


            else:
                messages.append(self.format_custom_input(1,api_feedback))


            print(messages)

            flag = input('next loop')
            if flag=='q':
                break






# {
#   "reasoning": "The task involves moving to the fruit table to pick up an apple and then returning to the initial position. The optimal path is to first go to the fruit table, pick up the apple, and then return to the initial position.",
#   "action_list": [
#     "go_to[fruit table][0.610712 5.894910]",
#     "pick_up[apple]",
#     "go_to[initial position][0 0]",
#     "done"
#   ],
#   "first_action": "go_to[fruit table][0.610712 5.894910]"
# }