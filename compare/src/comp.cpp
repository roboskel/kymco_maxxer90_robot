// Major comment : gazebo plugin gives twist at world frame, so does robot_localization due to _differential parameter at odometry inputs
// 2D, z containts the eucleidian distance, all messages (should) have pose.z == 0

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Vector3.h"
#include <ros/console.h>
#include <math.h>

double twist_t[3] = {0.0, 0.0, 0.0}, twist_est[3] = {0.0, 0.0, 0.0};
geometry_msgs::Vector3 diff_vel_pub;
double diff_vel[3] = {0.0, 0.0, 0.0};

double pose_t[3] = {0.0, 0.0, 0.0}, pose_est[3] = {0.0, 0.0, 0.0};
geometry_msgs::Vector3 diff_pose_pub;
double diff_pose[3] = {0.0, 0.0, 0.0};

int i,j;

void truth(const nav_msgs::Odometry::ConstPtr& msg) {
    //ROS_INFO("Hello %s", "truth");
    twist_t[0] = msg->twist.twist.linear.x;
    twist_t[1] = msg->twist.twist.linear.y;
    twist_t[2] = msg->twist.twist.angular.z;

    pose_t[0] = msg->pose.pose.position.x;
    pose_t[1] = msg->pose.pose.position.y;
    //pose_t[2] = msg->pose.pose.position.z;
    double temp = 0.0;

    for (i=0; i<3; i++) {
        diff_vel[i] = (twist_t[i] - twist_est[i]);
        diff_pose[i] = pose_t[i] - pose_est[i];
        temp += (pose_t[i] - pose_est[i])*(pose_t[i] - pose_est[i]);
    }
    diff_pose[2] = sqrt(temp);
}

void estimate(const nav_msgs::Odometry::ConstPtr& msg) {
    //ROS_INFO("Hello %s", "estimate");
    twist_est[0] = msg->twist.twist.linear.x;
    twist_est[1] = msg->twist.twist.linear.y;
    twist_est[2] = msg->twist.twist.angular.z;

    pose_est[0] = msg->pose.pose.position.x;
    pose_est[1] = msg->pose.pose.position.y;
    //pose_est[2] = msg->pose.pose.position.z;
    double temp = 0.0;

    for (j=0; j<3; j++) {
        diff_vel[j] = (twist_t[j] - twist_est[j]);
        diff_pose[j] = pose_t[j] - pose_est[j];
        temp += (pose_t[j] - pose_est[j])*(pose_t[j] - pose_est[j]);
    }
    diff_pose[2] = sqrt(temp);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "compare_vel");
    ros::NodeHandle n;
    ros::Subscriber sub1 = n.subscribe("/odom_perfect", 1000, truth);
    ros::Subscriber sub2 = n.subscribe("/odometry/filtered", 1000, estimate);
    ros::Publisher pub1 = n.advertise<geometry_msgs::Vector3>("/vel_diff", 1000);
    ros::Publisher pub2 = n.advertise<geometry_msgs::Vector3>("/pose_diff", 1000);
    while (ros::ok()){
        //ROS_INFO("Hello %s", "publisher");
        diff_vel_pub.x = diff_vel[0];
        diff_vel_pub.y = diff_vel[1];
        diff_vel_pub.z = diff_vel[2];

        diff_pose_pub.x = diff_pose[0];
        diff_pose_pub.y = diff_pose[1];
        diff_pose_pub.z = diff_pose[2];

        pub1.publish(diff_vel_pub);
        pub2.publish(diff_pose_pub);
        ros::spinOnce();
    }

    return 0;
}
