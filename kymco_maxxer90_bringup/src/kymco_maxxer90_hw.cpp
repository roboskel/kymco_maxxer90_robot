#include <kymco_maxxer90_motor_controller/kymco_maxxer90_hw.hpp>

using namespace kymco_maxxer90_motor_controller;

KymcoMaxxer90MotorController::KymcoMaxxer90MotorController(const ros::NodeHandle& n, const std::string& port1, const std::string& port2, const int& baudrate1, const int& baudrate2, const std::string& device_name1, const std::string& device_name2) {

    connectToMotorDriver(port1, port2, baudrate1, baudrate2, device_name1, device_name2);

    nh = n;
    connectToMotorDriver(port1, port2, baudrate1, baudrate2, device_name1, device_name2);
    state.position.resize(2);
    state.velocity.resize(2);
    state.effort.resize(2);
    state.name.resize(2);

}

KymcoMaxxer90MotorController::~KymcoMaxxer90MotorController() {
    serialClose();
}

void KymcoMaxxer90MotorController::connectToMotorDriver(const std::string& port1, const std::string& port2, const int& baudrate1, const int& baudrate2, const std::string& device_name1, const std::string& device_name2) {

    serialInit(srl1, port1, baudrate1, device_name1);
    serialInit(srl2, port2, baudrate2, device_name2);

    centerSteeringWheel();
    std::string response = srl1->read(9);
    if (response.find("$ANG") >= response.length()) {
        std::swap(srl1,srl2);
        centerSteeringWheel();
    }

}

void KymcoMaxxer90MotorController::serialInit(serial::Serial *srl, const std::string& port, const int& baudrate, const std::string& device_name) {
    while(ros::ok()) {
        try {
            ROS_INFO_STREAM("Looking for the " << device_name << " at port " << port);
            srl = new serial::Serial(port, baudrate, serial::Timeout::simpleTimeout(100));
            if (srl->isOpen()) {
                break;
            }
            throw std::runtime_error("Connection error");
        }
        catch(...) {
                ROS_ERROR_STREAM("Could not connect to the " << device_name << " ... Will retry every 3 seconds...");
                ros::Duration(3).sleep();
        }
    }
    ROS_INFO_STREAM("Connected to the " << device_name << " at port " << port);
}

void KymcoMaxxer90MotorController::centerSteeringWheel() {
    srl1->write("$ANGLE,25*1\r\n");
}

void KymcoMaxxer90MotorController::serialClose() {
    if (srl1) {
        srl1->close();
    }
    if (srl2) {
        srl2->close();
    }
}


