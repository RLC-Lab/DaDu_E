#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import math

class DistanceCalculator:
    def __init__(self):
        self.prev_x = None
        self.prev_y = None
        self.total_distance = 0.0

        rospy.init_node('distance_calculator', anonymous=True)
        self.odom_sub = rospy.Subscriber('/odometry/filtered1', Odometry, self.odom_callback)
        rospy.spin()

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        print("current x and y is:", x, y)

        if self.prev_x is not None and self.prev_y is not None:
            distance = math.sqrt((x - self.prev_x)**2 + (y - self.prev_y)**2)
            self.total_distance += distance

        self.prev_x = x
        self.prev_y = y

        rospy.loginfo("Total Distance: %f", self.total_distance)

if __name__ == '__main__':
    try:
        print('hello')
        DistanceCalculator()
    except rospy.ROSInterruptException:
        pass


# import rospy
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import PoseWithCovarianceStamped

# class DistanceCalculator:
#     def __init__(self):
#         # rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, path_callback)
#         rospy.Subscriber("/odometry/filtered1", Odometry, self.odom_callback)
#         self.start_pose = None
#         self.total_distance = 0.0

#     def odom_callback(self, data):
#         current_pose = data.pose.pose
#         if self.start_pose is None:
#             self.start_pose = current_pose
#         else:
#             self.update_distance(current_pose)

#     def update_distance(self, current_pose):
#         dx = current_pose.position.x - self.start_pose.position.x
#         dy = current_pose.position.y - self.start_pose.position.y
#         distance = (dx**2 + dy**2) ** 0.5
#         self.total_distance += distance
#         self.start_pose = current_pose

# rospy.init_node('distance_calculator')
# dc = DistanceCalculator()
# rospy.spin()
