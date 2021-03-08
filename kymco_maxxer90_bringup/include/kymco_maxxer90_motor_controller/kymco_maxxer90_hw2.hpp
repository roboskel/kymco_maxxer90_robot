#pragma once

#include <kymco_maxxer90_motor_controller/common.hpp>

namespace kymco_maxxer90_ackermann_steering_controller {

class KymcoMaxxer90AckermannSteeringController : public hardware_interface::RobotHW {

    public:
        KymcoMaxxer90AckermannSteeringController(const ros::NodeHandle&, const std::string& = "/dev/ttyUSB1", const std::string& = "/dev/ttyUSB2", const std::string& = "/dev/ttyUSB0", const int& = 57600, const std::string& = "motor_driver", const std::string& = "motor_driver", const std::string& = "arduino");
        ~KymcoMaxxer90AckermannSteeringController();

    private:
        void control();
        void serialClose();
        void actuatorsReset();
        void writeThrottleSerial();
        void writeSteeringSerial();
        void update(const ros::TimerEvent&);
        void readEncoders(const ros::TimerEvent&);
        void throttleFeedback(const ros::TimerEvent&);
        void steeringFeedback(const ros::TimerEvent&);
        void sprayingCallback(const std_msgs::Bool::ConstPtr&);
        void discoverDevices(std::vector<std::string>, const int);
        void serialInit(serial::Serial*&, const std::string&, const int&, const std::string&);
        double norm(const double&, const double&, const double&, const double&, const double&);

        ros::NodeHandle nh;
        ros::Subscriber spray_sub;
        sensor_msgs::JointState state;
        ros::Time last_encoder_reading;
        serial::Serial *srl1, *srl2, *srl3;
        ros::Duration elapsed_time, elapsed_time2;
        int encoder_reading, throttle_reading, steering_reading;
        hardware_interface::EffortJointInterface jeff_interface;
        hardware_interface::JointStateInterface jstate_interface;
        hardware_interface::VelocityJointInterface jvel_interface;
        hardware_interface::PositionJointInterface jpos_interface;
        int throttle_m, steering_m, throttle_command, steering_command;
        std::shared_ptr<controller_manager::ControllerManager> controller_manager;
        float throttle_actuator_pos, steering_actuator_pos, target_tap, target_sap;
        ros::Timer update_timer, encoders_timer, throttle_feedback_timer, steering_feedback_timer;
        double curr_steering_angle, target_steering_angle, linear_velocity, angular_velocity, prev_l, prev_a;

};

}
