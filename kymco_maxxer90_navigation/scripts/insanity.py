#!/usr/bin/env python
import tf
import math
import rospy
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

path_sub = None
robot_goal = None
cmd_vel_pub = None

prev_odom = Twist()
prev_twist = Odometry()

stairway_to_heaven = None

xy_tolerance = 0.2
max_trans_vel = 0.7
min_trans_vel = 0.35

def cmdVelFor(tw, tm):
    for i in range(0, int(tm*1000), 100):
        cmd_vel_pub.publish(tw)
        rospy.sleep(0.1)

def lookAt(robotx, roboty, goalx, goaly):
    dx = goalx - robotx
    dy = goaly - roboty
    return math.atan2(dy,dx)

def distance(x1, y1, x2, y2):
    return math.sqrt( (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2))

def path_callback(path):
    global robot_goal, stairway_to_heaven
    global path_sub
    path_sub.unregister()
    stairway_to_heaven = path.poses
    #  if len(path.poses) > 10:
        #  robot_goal = path.poses[10].pose
    #  else:
        #  robot_goal = None
    prev = stairway_to_heaven[0]
    prev_yaw = lookAt(prev.pose.position.x, prev.pose.position.y, stairway_to_heaven[-1].pose.position.x, stairway_to_heaven[-1].pose.position.y)
    twist = Twist()
    # Stop everything (reset)
    print("Resetting steering and throttle actuators")
    cmdVelFor(twist, 4)
    print("Trying to achieve nominal throttle for actual movement...")
    twist.linear.x = 0.35
    cmdVelFor(twist, 3)
    for i in range(0, len(stairway_to_heaven), 10):
        velx = 0.3
        angz = 0.0
        tm = 0.1
        p = stairway_to_heaven[i]
        curr_yaw = lookAt(p.pose.position.x, p.pose.position.y, stairway_to_heaven[-1].pose.position.x, stairway_to_heaven[-1].pose.position.y)
        yaw_diff = prev_yaw - curr_yaw
        print(yaw_diff)
        angz = -yaw_diff * 50

        tm += abs(angz) * 2
        if abs(yaw_diff) > 0.1:
            print("Detected the need for a sharp turn, deccelerating fully")
            velx = 0
            angz = 0.5 * yaw_diff / abs(yaw_diff)



        twist.linear.x = velx
        twist.angular.z = angz
        cmdVelFor(twist, tm)


        prev = p
        prev_yaw = curr_yaw
    twist = Twist()
    cmdVelFor(twist, 100)
    path_sub = rospy.Subscriber("/move_base/SBPLLatticePlanner/plan", Path, path_callback)




def odom_callback(odom):
    global robot_goal, prev_odom, prev_twist
    global stairway_to_heaven
    twist = Twist()
    curr_pos = odom.pose.pose.position
    curr_or = odom.pose.pose.orientation
    robot_goal = None
    print(len(stairway_to_heaven))
    if stairway_to_heaven:
        robot_goal = stairway_to_heaven[0].pose
    if robot_goal is not None:
        d = distance(curr_pos.x, curr_pos.y, robot_goal.position.x, robot_goal.position.y)
        if d > xy_tolerance :
            velx = min_trans_vel
            d = distance(curr_pos.x,curr_pos.y,prev_odom.pose.pose.position.x,prev_odom.pose.pose.position.y) 
            #  if d < 0.3:
                #  velx += 0.1
            if velx > max_trans_vel:
                velx = max_trans_vel
            (rr, rp, ry) = tf.transformations.euler_from_quaternion([curr_or.x, curr_or.y, curr_or.z, curr_or.w])

            straight_yaw = lookAt(curr_pos.x, curr_pos.y, robot_goal.position.x, robot_goal.position.y)
            yaw_diff = straight_yaw - ry

            #  x_diff = 2 * d * math.cos(yaw_diff2)
            #  y_diff = 2 * d * math.sin(yaw_diff2)
            #  z_diff = 2 * (robot_goal.position.z - (curr_pos.z - 0.1)) # up is positive

            #  x_diff = max(min_trans_vel, min(x_diff,max_trans_vel))
            #  y_diff = max(min_trans_vel, min(y_diff,max_trans_vel))
            if yaw_diff > math.pi:
                yaw_diff -= math.pi
            print(yaw_diff)
            twist.linear.x = velx
            twist.angular.z = min(1.0, max(yaw_diff, -1.0))
        else:
            if (stairway_to_heaven) > 30:
                stairway_to_heaven = stairway_to_heaven[30:]
            else:
                stairway_to_heaven = None
            stairway_to_heaven.pop(0)# = stairway_to_heaven[1:]
    cmd_vel_pub.publish(twist)
    prev_odom = odom
    prev_twist = twist

def init():
    global cmd_vel_pub, path_sub
    global max_trans_vel, min_trans_vel
    global  xy_tolerance
    rospy.init_node("insanity_local_planner")

    #  min_trans_vel = rospy.get_param("~min_trans_vel", 0.35)
    min_trans_vel = rospy.get_param("~min_trans_vel", 0.4)
    #  max_trans_vel = rospy.get_param("~max_trans_vel", 0.7)
    max_trans_vel = rospy.get_param("~max_trans_vel", 0.55)
    xy_tolerance = rospy.get_param("~xy_tolerance", 0.4)

    cmd_vel_pub = rospy.Publisher("insanity/cmd_vel", Twist, queue_size=1)
    path_sub = rospy.Subscriber("/move_base/SBPLLatticePlanner/plan", Path, path_callback)
    #  rospy.Subscriber("/odometry/filtered", Odometry, odom_callback)

    while not rospy.is_shutdown():
        rospy.spin()

init()

