#pragma once

#include <math.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

namespace kymco_maxxer90_ackermann_steering_controller {

// NEW PROTOCOL
static const std::string THROTTLE_START_N = "CS";
static const std::string THROTTLE_END_N = "";
static const std::string THROTTLE_INC = "I";
static const std::string THROTTLE_DEC = "D";
static const std::string STEERING_START_N = "CS";
static const std::string STEERING_END_N = "";
static const std::string STEERING_RIGHT = "R";
static const std::string STEERING_LEFT = "L";
static const int MS_FROM_ZERO_TO_MAX_THROTTLE = 9500; // 9.5 secs
static const int MS_FROM_MIN_TO_MAX_STEERING = 9000; // 9 secs
// ---

// JOINTS
static const std::string THROTTLE_JOINT = "RWD_joint";
static const std::string STEERING_JOINT = "steering_wheel_joint";
static const std::string FRONT_LEFT_WHEEL_JOINT = "front_left_wheel";
static const std::string FRONT_RIGHT_WHEEL_JOINT = "front_right_wheel";
static const std::string REAR_LEFT_WHEEL_JOINT = "rear_left_wheel";
static const std::string REAR_RIGHT_WHEEL_JOINT = "rear_right_wheel";
static const std::string REAR_SUSPENSION_JOINT = "rear_suspension_joint";
static const std::string FRONT_LEFT_SUSPENSION_JOINT = "front_left_suspension_joint";
static const std::string FRONT_RIGHT_SUSPENSION_JOINT = "front_right_suspension_joint";
static const std::string FRONT_LEFT_STEERING_JOINT = "front_left_steering_wheel_joint";
static const std::string FRONT_RIGHT_STEERING_JOINT = "front_right_steering_wheel_joint";
// ---

static const int  MIN_VELX = 0;
static const int  MAX_VELX = 9;
static const int MIN_ANGZ = 0;
static const int MAX_ANGZ = 49;
static const double MIN_VELX_MS = 0;
//static const double MAX_VELX_MS = 6.1;
static const double MAX_VELX_MS = 4.3;
static const double WHEEL_DIAMETER = 0.4572; //m
static const double METERS_PER_ENCODER_TICK = 1.f/39.f; // 39 ticks per meter
static const double DEG_PER_SEC = (MAX_ANGZ-MIN_ANGZ+1)/2.f;
static const double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * M_PI;
static const double RAD_PER_SEC = DEG_PER_SEC*M_PI/180.f;
static const double CENTERED_RAD_STEERING = int((MAX_ANGZ - MIN_ANGZ) / 2.f) * M_PI / 180.f;

}
