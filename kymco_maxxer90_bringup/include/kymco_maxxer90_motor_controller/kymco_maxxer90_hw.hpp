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

static const std::string THROTTLE_START = "$SPEED,";
static const std::string THROTTLE_END = "*11\r\n";
static const std::string STEERING_START = "$ANGLE,";
static const std::string STEERING_END = "*11\r\n";
static const std::string THROTTLE_JOINT = "RWD_joint";
static const std::string STEERING_JOINT = "steering_wheel_joint";
static const int  MIN_VELX = 0;
static const int  MAX_VELX = 9;
static const int MIN_ANGZ = 0;
static const int MAX_ANGZ = 49;
static const double WHEEL_DIAMETER = 0.4572; //m
static const double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * M_PI;
static const double METERS_PER_ENCODER_TICK = 1.f/39.f; // 39 ticks per meter
static const double RAD_PER_SEC = (MAX_ANGZ-MIN_ANGZ+1)*M_PI/180.f/2.f;
static const double DEG_PER_SEC = (MAX_ANGZ-MIN_ANGZ+1)/2.f;

class KymcoMaxxer90AckermannSteeringController : public hardware_interface::RobotHW {

    public:
        KymcoMaxxer90AckermannSteeringController(const ros::NodeHandle&, const std::string& = "/dev/ttyUSB1", const std::string& = "/dev/ttyUSB2", const std::string& = "/dev/ttyUSB0", const int& = 4800, const int& = 4800, const int& = 19200, const std::string& = "motor_driver", const std::string& = "motor_driver", const std::string& = "arduino");
        ~KymcoMaxxer90AckermannSteeringController();

    private:
        void serialClose();
        void centerSteeringWheel();
        void update(const ros::TimerEvent&);
        void writeThrottleSerial(const double&);
        void writeSteeringSerial(const double&);
        void readEncoders(const ros::TimerEvent&);
        void sprayingCallback(const std_msgs::Bool::ConstPtr&);
        void cmdVelCallback(const geometry_msgs::Twist::ConstPtr&);
        int norm(const double&, const double&, const double&, const double&, const double&);
        void serialInit(serial::Serial*&, const std::string&, const int&, const std::string&);
        void connectToMotorDriver(const std::string&, const std::string&, const int&, const int&, const std::string&, const std::string&);

        ros::NodeHandle nh;
        ros::Subscriber spray_sub;
        ros::Time last_encoder_reading;
        sensor_msgs::JointState state;
        serial::Serial *srl1, *srl2, *srl3;
        ros::Timer update_timer, encoders_timer;
        ros::Duration elapsed_time, elapsed_time2;
        int target_steering_angle, encoder_reading;
        hardware_interface::JointStateInterface jstate_interface;
        hardware_interface::VelocityJointInterface jvel_interface;
        hardware_interface::PositionJointInterface jpos_interface;
        hardware_interface::EffortJointInterface jeff_interface;
        std::shared_ptr<controller_manager::ControllerManager> controller_manager;
        double curr_steering_angle, linear_speed, angular_speed, linear_velocity, angular_velocity;

};

}
