#li!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray

goal_pub = None
goal_poses = []
nxt = 0

def statusCallback(msg):
    global nxt
    if msg.status_list[-1].status == 3 and nxt < len(goal_poses):
        goal_pub.publish(goal_poses[nxt])
        nxt += 1

def init():
    global goal_pub, goal_poses
    rospy.init_node("irta_demo")
    goal_pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=1)

    rospy.sleep(2)

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "odom"
    goal_pose.header.stamp = rospy.Time.now()
    goal_pose.pose.position.x = 12.1
    goal_pose.pose.position.y = -0.3
    goal_pose.pose.orientation.w = 1.0
    #  goal_pose.pose.position.y = 11.5
    #  goal_pose.pose.position.x = 11.0

    #  goal_pose.pose.orientation.w = 0.707
    #  goal_pose.pose.orientation.z = 0.707


    goal_pub.publish(goal_pose)

    move_base_status_sub = rospy.Subscriber("/move_base/status", GoalStatusArray, statusCallback)

    #  goal_pose.pose.position.x = 14.028
    #  goal_pose.pose.position.y = 8.58
    #  goal_pose.pose.orientation.z = 0.814
    #  goal_pose.pose.orientation.w = 0.581

    #  goal_poses.append(goal_pose)

    #  goal_pose.pose.position.x = 18.139
    #  goal_pose.pose.position.y = 1.245
    #  goal_pose.pose.orientation.z = 0.179
    #  goal_pose.pose.orientation.w = 0.983

    #  goal_poses.append(goal_pose)

    #  goal_pose.pose.position.x = 24.139
    #  goal_pose.pose.position.y = 11.03
    #  goal_pose.pose.orientation.z = 0.953
    #  goal_pose.pose.orientation.w = 0.303

    #  goal_poses.append(goal_pose)

    rospy.spin()

init()


