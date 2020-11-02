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

namespace kymco_maxxer90_motor_controller {

static const std::string THROTTLE_START = "$SPEED,";
static const std::string THROTTLE_END = "*1\r\n";
static const std::string STEERING_START = "$ANGLE,";
static const std::string STEERING_END = "*1\r\n";
static const std::string THROTTLE_JOINT = "RWD_joint";
static const std::string STEERING_JOINT = "steering_wheel_joint";
static const double RAD_PER_SEC = 50*M_PI/180/2;
static const double DEG_PER_SEC = 50/2;
static const int  MIN_VELX = 0;
static const int  MAX_VELX = 9;
static const int MIN_ANGZ = 0;
static const int MAX_ANGZ = 49;

class KymcoMaxxer90MotorController : public hardware_interface::RobotHW {

    public:
        KymcoMaxxer90MotorController(const ros::NodeHandle&, const std::string& = "/dev/ttyS0", const std::string& = "/dev/ttyS1", const std::string& = "/dev/ttyACM0", const int& = 4800, const int& = 4800, const int& = 19200, const std::string& = "motor_driver", const std::string& = "motor_driver", const std::string& = "arduino");
        ~KymcoMaxxer90MotorController();

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
        void serialInit(serial::Serial*, const std::string&, const int&, const std::string&);
        void connectToMotorDriver(const std::string&, const std::string&, const int&, const int&, const std::string&, const std::string&);

        ros::NodeHandle nh;
        ros::Publisher odom_pub;
        int target_steering_angle;
        double curr_steering_angle;
        ros::Duration elapsed_time;
        geometry_msgs::Twist cmd_vel;
        sensor_msgs::JointState state;
        serial::Serial *srl1, *srl2, *srl3;
        ros::Subscriber cmd_vel_sub, spray_sub;
        ros::Timer update_timer, encoders_timer;
        hardware_interface::JointStateInterface jstate_interface;
        hardware_interface::VelocityJointInterface jvel_interface;
        hardware_interface::PositionJointInterface jpos_interface;
        hardware_interface::EffortJointInterface jeff_interface;
        std::shared_ptr<controller_manager::ControllerManager> controller_manager;

};

}
