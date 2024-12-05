import rospy
from nav_msgs.msg import Path
from math import sqrt

# 计算路径长度
def calculate_path_length(path):
    length = 0.0
    for i in range(1, len(path.poses)):
        p1 = path.poses[i-1].pose.position
        p2 = path.poses[i].pose.position
        dist = sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)
        length += dist
    return length

def path_callback(msg):
    path_length = calculate_path_length(msg)
    rospy.loginfo(f"Global Path Length: {path_length} meters")

def main():
    rospy.init_node('path_length_calculator')
    rospy.loginfo(f"start to get global path")

    rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, path_callback)

    rospy.spin()

if __name__ == '__main__':
    main()