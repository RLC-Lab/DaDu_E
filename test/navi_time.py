import rospy
from nav_msgs.msg import Path

def plan_callback(msg):
    # 获取路径规划时间戳
    plan_time = msg.header.stamp
    # 当前时间
    current_time = rospy.Time.now()
    # 计算规划时间
    planning_duration = current_time - plan_time
    print("Global Planner path computed in: %.3f seconds", planning_duration.to_sec())

if __name__ == "__main__":
    rospy.init_node('global_planner_timing')
    rospy.Subscriber('/move_base/NavfnROS/plan', Path, plan_callback)
    rospy.spin()
