#include <kymco_maxxer90_motor_controller/kymco_maxxer90_hw.hpp>

using namespace kymco_maxxer90_motor_controller;

KymcoMaxxer90MotorController::KymcoMaxxer90MotorController(const ros::NodeHandle& n, const std::string& port1, const std::string& port2, const std::string& port3, const int& baudrate1, const int& baudrate2, const int& baudrate3, const std::string& device_name1, const std::string& device_name2, const std::string& device_name3) {

    connectToMotorDriver(port1, port2, baudrate1, baudrate2, device_name1, device_name2);
    serialInit(srl3, port3, baudrate3, device_name3);

    nh = n;

    state.position.push_back(0.0);
    state.position.push_back(0.0);
    state.velocity.push_back(0.0);
    state.velocity.push_back(0.0);
    state.effort.push_back(0.0);
    state.effort.push_back(0.0);
    state.name.push_back(THROTTLE_JOINT);
    state.name.push_back(STEERING_JOINT);

    curr_steering_angle = 25;
    target_steering_angle = 25.0;

    hardware_interface::JointStateHandle state_linear_x(THROTTLE_JOINT, &state.position[0], &state.velocity[0], &state.effort[0]);
    hardware_interface::JointStateHandle state_angular_z(STEERING_JOINT, &state.position[1], &state.velocity[1], &state.effort[1]);
    jstate_interface.registerHandle(state_linear_x);
    jstate_interface.registerHandle(state_angular_z);

    hardware_interface::JointHandle vel_handle_linx(jstate_interface.getHandle(THROTTLE_JOINT), &state.velocity[0]);
    hardware_interface::JointHandle vel_handle_angz(jstate_interface.getHandle(STEERING_JOINT), &state.velocity[1]);
    jvel_interface.registerHandle(vel_handle_linx);
    jvel_interface.registerHandle(vel_handle_angz);

    hardware_interface::JointHandle pos_handle_linx(jstate_interface.getHandle(THROTTLE_JOINT), &state.position[0]);
    hardware_interface::JointHandle pos_handle_angz(jstate_interface.getHandle(STEERING_JOINT), &state.position[1]);
    jpos_interface.registerHandle(pos_handle_linx);
    jpos_interface.registerHandle(pos_handle_angz);

    hardware_interface::JointHandle eff_handle_linx(jstate_interface.getHandle(THROTTLE_JOINT), &state.effort[0]);
    hardware_interface::JointHandle eff_handle_angz(jstate_interface.getHandle(STEERING_JOINT), &state.effort[1]);
    jeff_interface.registerHandle(eff_handle_linx);
    jeff_interface.registerHandle(eff_handle_angz);

    registerInterface(&jstate_interface);
    registerInterface(&jvel_interface);
    registerInterface(&jpos_interface);
    registerInterface(&jeff_interface);

    controller_manager.reset(new controller_manager::ControllerManager(this, nh));

    update_timer = nh.createTimer(ros::Duration(0.1), &KymcoMaxxer90MotorController::update, this);

    encoders_timer = nh.createTimer(ros::Duration(0.5), &KymcoMaxxer90MotorController::readEncoders, this);

    odom_pub = nh.advertise<nav_msgs::Odometry>("/kymco_maxxer90_motor_controller/odom", 1);

    cmd_vel_sub = nh.subscribe("/kymco_maxxer90_motor_controller/cmd_vel", 1, &KymcoMaxxer90MotorController::cmdVelCallback, this);
    spray_sub = nh.subscribe("/kymco_maxxer90_motor_controller/spray_valve", 1, &KymcoMaxxer90MotorController::sprayingCallback, this);

}

KymcoMaxxer90MotorController::~KymcoMaxxer90MotorController() {
    serialClose();
}

void KymcoMaxxer90MotorController::update(const ros::TimerEvent& e) {
    elapsed_time = ros::Duration(e.current_real - e.last_real);

    double delta_steering = std::min(abs(target_steering_angle - curr_steering_angle), elapsed_time.toSec() * DEG_PER_SEC);
    curr_steering_angle = target_steering_angle > curr_steering_angle ? curr_steering_angle + delta_steering : curr_steering_angle - delta_steering;
    double centered_rad_steering = (MAX_ANGZ - MIN_ANGZ) / 2 * M_PI / 180;

    state.position[1] = norm(curr_steering_angle, MIN_ANGZ, MAX_ANGZ, -centered_rad_steering, centered_rad_steering);

    // TODO update joint states (only for linear, angular is now implemented)

    controller_manager->update(ros::Time::now(), elapsed_time);

    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    // TODO update positions xy and velocities xyz
    odom.pose.pose.position.x = 0;
    odom.pose.pose.position.y = 0;
    odom.pose.pose.position.z = 0;

    odom.twist.twist.linear.x = 0;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = 0;

    odom_pub.publish(odom);
}

void KymcoMaxxer90MotorController::readEncoders(const ros::TimerEvent& e) {
    std::string response = srl3->read(9);
    if (not response.empty()) {
        try {
            int encoder_reading = std::stoi(response);
            // TODO update encoder value
        }
        catch (...) {

        }
    }
}

void KymcoMaxxer90MotorController::sprayingCallback(const std_msgs::Bool::ConstPtr& msg) {
    srl3->write("CV"+std::to_string(int(msg->data)));
}

void KymcoMaxxer90MotorController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel) {
    writeThrottleSerial(cmd_vel->linear.x);
    writeSteeringSerial(cmd_vel->angular.z);
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
    srl1->write(STEERING_START + "25" + STEERING_END);
}

void KymcoMaxxer90MotorController::serialClose() {
    if (srl1) {
        srl1->close();
    }
    if (srl2) {
        srl2->close();
    }
}

void KymcoMaxxer90MotorController::writeThrottleSerial(const double& x) {
    if (x >= 0) {
        int v = norm(x, MIN_VELX, MAX_VELX, 0, 1.0);
        srl2->write(THROTTLE_START + std::to_string(v) + THROTTLE_END);
    }
    else {
        ROS_WARN("Received negative X velocity value. Our robot cannot move backwards. Ignoring.");
    }
}

void KymcoMaxxer90MotorController::writeSteeringSerial(const double& z) {
    target_steering_angle = norm(z, 1.0, -1.0, MIN_ANGZ, MAX_ANGZ);
    srl1->write(STEERING_START + std::to_string(target_steering_angle) + STEERING_END);
}

int KymcoMaxxer90MotorController::norm(const double& v1, const double& s1, const double& e1, const double& s2, const double& e2) {
    return (((v1 - s1) * (e2 - s2)) / (e1 - s1)) + s2;
}

