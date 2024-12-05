# navigation.py

import random
import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import math
from nav_msgs.msg import Odometry
import rospy
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf
from nav_msgs.msg import OccupancyGrid

import time

class Navigation:
    def __init__(self):
    

        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.map_data = None
        self.last_direction = None
        self.last_location = None
        self.direct_mapping = {
            (1, 0): 0,
            (0, -1): 1,
            (-1, 0): 2,
            (0, 1): 3
        }
        pass
   

    def navi_xy_goto(self, location,direction=4):
        
        self.last_location = location
        print('move_base: ',location)

        x = float(location[0])
        y = float(location[1])

        target_x = x
        target_y = y

        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        x,y = self.get_nearest_reachable_point_by_direction(x,y,direction)

        print('# directiond:',direction)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y


        orientation = self.calculate_orientation(x, y, target_x, target_y)
        goal.target_pose.pose.orientation.x = orientation[0]
        goal.target_pose.pose.orientation.y = orientation[1]
        goal.target_pose.pose.orientation.z = orientation[2]
        goal.target_pose.pose.orientation.w = orientation[3]


        rospy.loginfo("Sending goal: x={}, y={}".format(x, y))
        client.send_goal(goal)
        start_time = time.time()

        
        client.wait_for_result()

        if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            end_time = time.time()
            elapsed_time = end_time - start_time
            rospy.loginfo(f"Goal reached in {elapsed_time:.2f} seconds")
            return "navigation successful"
        

        rospy.loginfo("Goal failed with error code: " + str(client.get_state()))
        return "navigation false"
    
    def get_nearest_reachable_point_by_direction(self,input_x,input_y,direction):
        if self.map_data is None:
            rospy.logwarn("Map data is not available yet.")
            return

        # Example input: World coordinates
        # input_x = float(input("Enter X coordinate (world): "))
        # input_y = float(input("Enter Y coordinate (world): "))
        # -7.91081 -1.33447
        

        # Convert world coordinates to map coordinates
        map_x = round((input_x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
        map_y = round((input_y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)

        # Define search range
        search_range = 60  # 1*20(resolution)/2
        # min_dis = float('inf')
        # Search while expanding the search range
         # Define the search directions based on the input direction

        print('d:',direction)
        if direction == 0:# right
            directions = [(1, 0)]
        elif direction == 1:# down
            directions = [(0, -1)]
        elif direction == 2:# left
            directions = [(-1, 0)]
        elif direction == 3:# up
            directions = [(0,1)]
        elif direction == 4:# all
            directions = [(1, 0), (0, -1), (-1, 0), (0,1)]
        else:
            raise ValueError("Invalid direction. Choose from 'up', 'down', 'left', 'right', or 'all'.")
        
        print('ds:',directions)

        # Loop over increasing search ranges
        flag = False
        
        for radius in range(1, search_range + 1):
            if flag:
                break
            for dx, dy in directions:
                # Calculate the new coordinates
                x = map_x + dx * radius
                y = map_y + dy * radius

                # Check if the new coordinates are within the map boundaries
                if 0 <= x < self.map_data.info.width and 0 <= y < self.map_data.info.height:
                    index = y * self.map_data.info.width + x
                    occupancy_value = self.map_data.data[index]
                    if occupancy_value == 0:  # Check if the cell is free space
                    # Check if the point satisfies the distance constraint to obstacles
                        if self.check_distance_to_obstacles(x, y):
                            self.last_direction = self.direct_mapping[(dx,dy)]
                            print('# direction:', self.last_direction)
                            # Convert map coordinates to world coordinates
                            final_x = round(self.map_data.info.origin.position.x + x * self.map_data.info.resolution, 2)
                            final_y = round(self.map_data.info.origin.position.y + y * self.map_data.info.resolution, 2)
                            # Calculate the distance to the target point
                            min_dis = self.calculate_distance(final_x, final_y, input_x, input_y)
                            # Update the minimum distance and final coordinates if a closer point is found
                            # if dis < min_dis:
                            #     min_dis = dis
                            #     final_x = raw_x
                            #     final_y = raw_y
                            flag= True
                            break
                
        
        rospy.loginfo("Nearest reachable point: (%.3f, %.3f)" % (final_x, final_y))
        print(min_dis/20)
        return final_x,final_y
    def calculate_distance(self,x1, y1, x2, y2):
        distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        return distance

    def check_distance_to_obstacles(self, x, y):
        search_range = 15  # 1*20(resolution)/2
        
        
        for b in range(y - search_range, y + search_range + 1):
            for a in range(x - search_range, x + search_range + 1):
        
                if 0 <= a < self.map_data.info.width and 0 <= b < self.map_data.info.height:
                    index = b * self.map_data.info.width + a
                    occupancy_value = self.map_data.data[index]
                    if occupancy_value != 0:  # Check if the cell is occupied
                        return False
        return True  # Distance constraint satisfied
    
    def map_callback(self, data):
        rospy.loginfo("Received map data")
        self.map_data = data

    def calculate_orientation(self,x, y, a, b):
        # 计算角度
        angle = math.atan2(b - y, a - x)
    
        # 创建四元数
        quaternion = tf.transformations.quaternion_from_euler(0, 0, angle)
        
        return quaternion

    # 调用函数


    def get_nearest_reachable_point(self,input_x,input_y):
        if self.map_data is None:
            rospy.logwarn("Map data is not available yet.")
            return
        

        # Convert world coordinates to map coordinates
        map_x = round((input_x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
        map_y = round((input_y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)

        # Define search range
        search_range = 40  # 1*20(resolution)/2
        min_dis = float('inf')
        # Search while expanding the search range
        
        for y in range(map_y - search_range, map_y + search_range + 1):
            for x in range(map_x - search_range, map_x + search_range + 1):
                # Check if the current cell is within the map boundaries
                if 0 <= x < self.map_data.info.width and 0 <= y < self.map_data.info.height:
                    index = y * self.map_data.info.width + x
                    occupancy_value = self.map_data.data[index]
                    if occupancy_value == 0:  # Check if the cell is free space
                        # Check if the cell satisfies the distance constraint to obstacles
                        if self.check_distance_to_obstacles(x,y):
                        # Convert map coordinates to world coordinates
                            raw_x =  round(self.map_data.info.origin.position.x + x * self.map_data.info.resolution,2)
                            raw_y = round(self.map_data.info.origin.position.y + y  * self.map_data.info.resolution,2)
                            dis = self.calculate_distance(raw_x,raw_y,input_x,input_y)
                            if dis<min_dis:
                                min_dis = dis
                                final_x = raw_x
                                final_y = raw_y
        rospy.loginfo("Nearest reachable point: (%.3f, %.3f)" % (final_x, final_y))
        print(min_dis/20)
        return final_x,final_y
        # Expand the search range
            

        rospy.loginfo("No reachable point found within the search range.")

if __name__ == "__main__":
    
    
    rospy.init_node('send_move_base_goal', anonymous=True)

    coordinates = [-4.66216,3.40295]
    coordinates = [0.610712,5.894910]
    navigation = Navigation()
    import grasp_utils

    grasp_utils.move_forward()
    grasp_utils.move_backward()


    # navigation.navi_xy_goto(coordinates,)


    # while True:
    #     d = int(input())
    #     if d == 10:
    #         break

    #     refine_direction = (d+navigation.last_direction)%4
    #     print(refine_direction)
    #     navigation.navi_xy_goto(coordinates,refine_direction)