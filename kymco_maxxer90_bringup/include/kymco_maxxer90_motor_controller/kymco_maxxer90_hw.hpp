#pragma once

#include <ros/ros.h>
#include <serial/serial.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

namespace kymco_maxxer90_motor_controller {

class KymcoMaxxer90MotorController : public hardware_interface::RobotHW {

    public:
        KymcoMaxxer90MotorController(const ros::NodeHandle&, const std::string& = "/dev/ttyS0", const std::string& = "/dev/ttyS1", const int& = 4800, const int& = 4800, const std::string& = "motor_driver", const std::string& = "motor_driver");
        ~KymcoMaxxer90MotorController();

    private:
        void serialInit(serial::Serial*, const std::string&, const int&, const std::string&);
        void connectToMotorDriver(const std::string&, const std::string&, const int&, const int&, const std::string&, const std::string&);
        void centerSteeringWheel();
        void serialClose();

        serial::Serial *srl1, *srl2;
        ros::NodeHandle nh;
        sensor_msgs::JointState state;
};

}
