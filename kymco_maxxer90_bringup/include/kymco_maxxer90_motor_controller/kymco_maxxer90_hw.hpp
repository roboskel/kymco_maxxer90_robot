#pragma once

#include <kymco_maxxer90_motor_controller/common.hpp>

namespace kymco_maxxer90_ackermann_steering_controller {

// OLD PROTOCOL
static const std::string THROTTLE_START = "$SPEED,";
static const std::string THROTTLE_END = "*11\r\n";
static const std::string STEERING_START = "$ANGLE,";
static const std::string STEERING_END = "*11\r\n";
// ---

class KymcoMaxxer90AckermannSteeringController : public hardware_interface::RobotHW {

    public:
        KymcoMaxxer90AckermannSteeringController(const ros::NodeHandle&, const std::string& = "/dev/ttyUSB1", const std::string& = "/dev/ttyUSB2", const std::string& = "/dev/ttyUSB0", const int& = 4800, const int& = 4800, const int& = 57600, const std::string& = "motor_driver", const std::string& = "motor_driver", const std::string& = "arduino");
        ~KymcoMaxxer90AckermannSteeringController();

    private:
        void control();
        void serialClose();
        void centerSteeringWheel();
        void writeThrottleSerial();
        void writeSteeringSerial();
        void newProtocolActuatorsReset();
        void update(const ros::TimerEvent&);
        void readEncoders(const ros::TimerEvent&);
        void sprayingCallback(const std_msgs::Bool::ConstPtr&);
        double norm(const double&, const double&, const double&, const double&, const double&);
        void serialInit(serial::Serial*&, const std::string&, const int&, const std::string&);
        void connectToMotorDriver(const std::string&, const std::string&, const int&, const int&, const std::string&, const std::string&);

        ros::NodeHandle nh;
        int encoder_reading;
        ros::Subscriber spray_sub;
        int throttle_m, steering_m;
        sensor_msgs::JointState state;
        ros::Time last_encoder_reading;
        serial::Serial *srl1, *srl2, *srl3;
        ros::Timer update_timer, encoders_timer;
        ros::Duration elapsed_time, elapsed_time2;
        hardware_interface::EffortJointInterface jeff_interface;
        hardware_interface::JointStateInterface jstate_interface;
        hardware_interface::VelocityJointInterface jvel_interface;
        hardware_interface::PositionJointInterface jpos_interface;
        std::shared_ptr<controller_manager::ControllerManager> controller_manager;
        float throttle_actuator_pos, steering_actuator_pos, target_tap, target_sap;
        double curr_steering_angle, target_steering_angle, linear_velocity, angular_velocity, prev_l, prev_a;

};

}
