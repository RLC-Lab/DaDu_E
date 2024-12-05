import rospy
import time
from geometry_msgs.msg import Twist
import rospy



def move_to_grasp_pose(self):
    pass
    

def move_to_detect_pose(self):
    pass


def move_forward():

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(10) 
    time.sleep(1)
    
    print("forward...")
    move_cmd = Twist()

    move_cmd.linear.x = 0.15

    duration = 20
    start_time = time.time()

    while time.time() - start_time < duration:
        pub.publish(move_cmd)
        rate.sleep()  
    print("forword finished")
    move_cmd.linear.x = 0
    pub.publish(move_cmd)

def move_backward():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(10) 
    time.sleep(1)


    move_cmd = Twist()
    move_cmd.linear.x = -0.12
    duration = 20


    print("backward...")
    

    start_time = time.time()
    while time.time() - start_time < duration+2:
        pub.publish(move_cmd)
        rate.sleep() 

    move_cmd.linear.x = 0
    pub.publish(move_cmd)
    
