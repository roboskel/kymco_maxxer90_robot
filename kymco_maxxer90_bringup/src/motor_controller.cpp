#include <kymco_maxxer90_motor_controller/kymco_maxxer90_hw.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "kymco_maxxer90_motor_controller");
    ros::NodeHandle nh;

    std::string port1, port2;
    int baudrate1, baudrate2;

    nh.param("kymco_maxxer90_motor_controller/port1", port1, std::string("/dev/ttyUSB0"));
    nh.param("kymco_maxxer90_motor_controller/port2", port2, std::string("/dev/ttyUSB1"));
    nh.param("kymco_maxxer90_motor_controller/baudrate1", baudrate1, 4800);
    nh.param("kymco_maxxer90_motor_controller/baudrate2", baudrate2, 4800);

    ros::MultiThreadedSpinner spinner(2);

    kymco_maxxer90_motor_controller::KymcoMaxxer90MotorController km90(nh);

    spinner.spin();

    return 0;

}
