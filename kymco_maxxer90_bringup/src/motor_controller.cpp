#include <kymco_maxxer90_motor_controller/kymco_maxxer90_hw.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "kymco_maxxer90_ackermann_steering_controller");
    ros::NodeHandle nh;

    std::string port1, port2, port3;
    int baudrate1, baudrate2, baudrate3;

    nh.param("kymco_maxxer90_ackermann_steering_controller/port1", port1, std::string("/dev/ttyUSB1"));
    nh.param("kymco_maxxer90_ackermann_steering_controller/port2", port2, std::string("/dev/ttyUSB2"));
    nh.param("kymco_maxxer90_ackermann_steering_controller/port3", port3, std::string("/dev/ttyUSB0"));
    nh.param("kymco_maxxer90_ackermann_steering_controller/baudrate1", baudrate1, 4800);
    nh.param("kymco_maxxer90_ackermann_steering_controller/baudrate2", baudrate2, 4800);
    nh.param("kymco_maxxer90_ackermann_steering_controller/baudrate3", baudrate3, 19200);

    ros::MultiThreadedSpinner spinner(3);

    kymco_maxxer90_ackermann_steering_controller::KymcoMaxxer90AckermannSteeringController km90(nh);

    spinner.spin();

    return 0;

}
