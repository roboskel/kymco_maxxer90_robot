#include <ros/ros.h>
#include <ros/package.h>
#include "serial/serial.h"

// Serial 1 is for the steering board
serial::Serial *srl1;
// Serial 2 is for the throttle board
serial::Serial *srl2;

void centerSteeringWheel() {
    srl1->write("$ANGLE,25*1\r\n");
}

std::string serialInit(serial::Serial *srl, const std::string& port, const int& baudrate, const std::string& device_name) {
    while(true) {
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

void connectToMotorDriver(const std::string& port1="/dev/ttyUSB0", const std::string& port2="/dev/ttyUSB1", const int& baudrate1=9600, const int& baudrate2=9600) {

    serialInit(srl1, port1, baudrate1, "motor driver");
    serialInit(srl2, port2, baudrate2, "motor driver");

    centerSteeringWheel();
    std::string response = srl1->read(9);
    if (response.find("$ANG") >= response.length()) {
        std::swap(srl1,srl2);
        centerSteeringWheel();
    }

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "kymco_maxxer90_motor_controller");
    ros::NodeHandle nh;

    std::string port1, port2;
    int baudrate1, baudrate2;

    nh.param("kymco_maxxer90_motor_controller/port1", port1, std::string("/dev/ttyUSB0"));
    nh.param("kymco_maxxer90_motor_controller/port2", port2, std::string("/dev/ttyUSB1"));
    nh.param("kymco_maxxer90_motor_controller/baudrate1", baudrate1, 4800);
    nh.param("kymco_maxxer90_motor_controller/baudrate2", baudrate2, 4800);

    connectToMotorDriver(port1, port2, baudrate1, baudrate2);

    ros::spin();

    srl1->close();
    srl2->close();

    return 0;

}
