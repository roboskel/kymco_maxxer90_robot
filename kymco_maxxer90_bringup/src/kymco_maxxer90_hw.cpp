#include <kymco_maxxer90_motor_controller/kymco_maxxer90_hw.hpp>

using namespace kymco_maxxer90_ackermann_steering_controller;

KymcoMaxxer90AckermannSteeringController::KymcoMaxxer90AckermannSteeringController(const ros::NodeHandle& n, const std::string& port1, const std::string& port2, const std::string& port3, const int& baudrate1, const int& baudrate2, const int& baudrate3, const std::string& device_name1, const std::string& device_name2, const std::string& device_name3) {

    // NOTE:
    // srl1 -> Steering
    // srl2 -> Throttle
    // srl3 -> Arduino (spraying, encoders, etc)

    connectToMotorDriver(port1, port2, baudrate1, baudrate2, device_name1, device_name2);
    serialInit(srl3, port3, baudrate3, device_name3);

    nh = n;

    state.name.push_back(THROTTLE_JOINT);
    state.name.push_back(STEERING_JOINT);
    state.name.push_back(FRONT_RIGHT_WHEEL_JOINT);
    state.name.push_back(FRONT_LEFT_WHEEL_JOINT);
    state.name.push_back(REAR_RIGHT_WHEEL_JOINT);
    state.name.push_back(REAR_LEFT_WHEEL_JOINT);
    state.name.push_back(FRONT_RIGHT_STEERING_JOINT);
    state.name.push_back(FRONT_LEFT_STEERING_JOINT);
    state.name.push_back(FRONT_RIGHT_SUSPENSION_JOINT);
    state.name.push_back(FRONT_LEFT_SUSPENSION_JOINT);
    state.name.push_back(REAR_SUSPENSION_JOINT);
    for(unsigned i=0; i<11; i++) {
        state.position.push_back(0.0);
        state.velocity.push_back(0.0);
        state.effort.push_back(0.0);
    }
    // Handlers do not work when initialised inside the for loop
    // and I am too lazy to create a struct/class
    hardware_interface::JointStateHandle state_handler0(state.name.at(0), &state.position[0], &state.velocity[0], &state.effort[0]);
    jstate_interface.registerHandle(state_handler0);
    hardware_interface::JointStateHandle state_handler1(state.name.at(1), &state.position[1], &state.velocity[1], &state.effort[1]);
    jstate_interface.registerHandle(state_handler1);
    hardware_interface::JointStateHandle state_handler2(state.name.at(2), &state.position[2], &state.velocity[2], &state.effort[2]);
    jstate_interface.registerHandle(state_handler2);
    hardware_interface::JointStateHandle state_handler3(state.name.at(3), &state.position[3], &state.velocity[3], &state.effort[3]);
    jstate_interface.registerHandle(state_handler3);
    hardware_interface::JointStateHandle state_handler4(state.name.at(4), &state.position[4], &state.velocity[4], &state.effort[4]);
    jstate_interface.registerHandle(state_handler4);
    hardware_interface::JointStateHandle state_handler5(state.name.at(5), &state.position[5], &state.velocity[5], &state.effort[5]);
    jstate_interface.registerHandle(state_handler5);
    hardware_interface::JointStateHandle state_handler6(state.name.at(6), &state.position[6], &state.velocity[6], &state.effort[6]);
    jstate_interface.registerHandle(state_handler6);
    hardware_interface::JointStateHandle state_handler7(state.name.at(7), &state.position[7], &state.velocity[7], &state.effort[7]);
    jstate_interface.registerHandle(state_handler7);
    hardware_interface::JointStateHandle state_handler8(state.name.at(8), &state.position[8], &state.velocity[8], &state.effort[8]);
    jstate_interface.registerHandle(state_handler8);
    hardware_interface::JointStateHandle state_handler9(state.name.at(9), &state.position[9], &state.velocity[9], &state.effort[9]);
    jstate_interface.registerHandle(state_handler9);
    hardware_interface::JointStateHandle state_handler10(state.name.at(10), &state.position[10], &state.velocity[10], &state.effort[10]);
    jstate_interface.registerHandle(state_handler10);

    hardware_interface::JointHandle vel_handle_linx(jstate_interface.getHandle(THROTTLE_JOINT), &linear_velocity);
    jvel_interface.registerHandle(vel_handle_linx);

    hardware_interface::JointHandle pos_handle_angz(jstate_interface.getHandle(STEERING_JOINT), &angular_velocity);
    jpos_interface.registerHandle(pos_handle_angz);

    registerInterface(&jstate_interface);
    registerInterface(&jvel_interface);
    registerInterface(&jpos_interface);

    controller_manager.reset(new controller_manager::ControllerManager(this, nh));

    update_timer = nh.createTimer(ros::Duration(0.02), &KymcoMaxxer90AckermannSteeringController::update, this);

    encoders_timer = nh.createTimer(ros::Duration(0.05), &KymcoMaxxer90AckermannSteeringController::readEncoders, this);

    spray_sub = nh.subscribe("/kymco_maxxer90_ackermann_steering_controller/spray_valve", 1, &KymcoMaxxer90AckermannSteeringController::sprayingCallback, this);

}

KymcoMaxxer90AckermannSteeringController::~KymcoMaxxer90AckermannSteeringController() {
    serialClose();
}

void KymcoMaxxer90AckermannSteeringController::update(const ros::TimerEvent& e) {
    target_steering_angle = norm(angular_velocity, 1.0, -1.0, MIN_ANGZ, MAX_ANGZ);
    elapsed_time = ros::Duration(e.current_real - e.last_real);

    double delta_steering = std::min(abs(target_steering_angle - curr_steering_angle), elapsed_time.toSec() * DEG_PER_SEC);
    curr_steering_angle = target_steering_angle > curr_steering_angle ? curr_steering_angle + delta_steering : curr_steering_angle - delta_steering;

    double new_steering_pos = norm(curr_steering_angle, MIN_ANGZ, MAX_ANGZ, -CENTERED_RAD_STEERING, CENTERED_RAD_STEERING);

    double dx = encoder_reading * METERS_PER_ENCODER_TICK; // m

    // double new_throttle_pos = dx != 0 ? (2 * M_PI) / (dx / WHEEL_CIRCUMFERENCE) : 0; // rad

    // Throttle joints
    // state.velocity[0] = elapsed_time2.toSec() != 0 ? new_throttle_pos / elapsed_time2.toSec() : 0; // rad/s;
    // state.position[0] += new_throttle_pos; //rad

    // NOTE: Even though joints are documented to be measured in rad and rad/s for position and velocity
    // respectively, ackermann_steering_controller expects m and m/s (!)
    state.velocity[0] = elapsed_time2.toSec() != 0 ? dx / elapsed_time2.toSec() : 0; // m/s;
    state.position[0] += dx; // m
    // All wheels blindly mimic the RWD joint
    state.position[2] = -state.position[0];
    state.velocity[2] = -state.velocity[0];
    state.position[3] = state.position[0];
    state.velocity[3] = state.velocity[0];
    state.position[4] = -state.position[0];
    state.velocity[4] = -state.velocity[0];
    state.position[5] = state.position[0];
    state.velocity[5] = state.velocity[0];

    // Steering joints
    state.velocity[1] = (new_steering_pos - state.position[1]) / elapsed_time.toSec();
    state.position[1] = new_steering_pos;
    // Front wheels blindly mimic the steering wheel
    state.position[6] = state.position[1];
    state.position[7] = state.position[1];

    controller_manager->update(ros::Time::now(), elapsed_time);

    control();
}

void KymcoMaxxer90AckermannSteeringController::readEncoders(const ros::TimerEvent& e) {
    try {
        std::string response = srl3->read(9);
        if (not response.empty()) {
            encoder_reading = std::stoi(response);
            ros::Time t = ros::Time::now();
            elapsed_time2 = t - last_encoder_reading;
            last_encoder_reading = t;
        }
    }
    catch (...) {}
}

void KymcoMaxxer90AckermannSteeringController::sprayingCallback(const std_msgs::Bool::ConstPtr& msg) {
    srl3->write("CV"+std::to_string(int(msg->data)));
}

void KymcoMaxxer90AckermannSteeringController::control() {
    writeThrottleSerial();
    writeSteeringSerial();
}

void KymcoMaxxer90AckermannSteeringController::connectToMotorDriver(const std::string& port1, const std::string& port2, const int& baudrate1, const int& baudrate2, const std::string& device_name1, const std::string& device_name2) {

    serialInit(srl1, port1, baudrate1, device_name1);
    serialInit(srl2, port2, baudrate2, device_name2);

    centerSteeringWheel();
    std::string response = srl1->read(9);
    if (response.find("$ANG") >= response.length()) {
        std::swap(srl1,srl2);
        centerSteeringWheel();
    }

}

void KymcoMaxxer90AckermannSteeringController::serialInit(serial::Serial *&srl, const std::string& port, const int& baudrate, const std::string& device_name) {
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

void KymcoMaxxer90AckermannSteeringController::centerSteeringWheel() {
    srl1->write(STEERING_START + std::to_string(int((MAX_ANGZ-MIN_ANGZ)/2)) + STEERING_END);
}

void KymcoMaxxer90AckermannSteeringController::serialClose() {
    if (srl1) {
        srl1->close();
    }
    if (srl2) {
        srl2->close();
    }
    if (srl3) {
        srl3->close();
    }
}

void KymcoMaxxer90AckermannSteeringController::writeThrottleSerial() {
    if (linear_velocity >= 0) {
        int v = norm(linear_velocity, MIN_VELX, MAX_VELX, 0, 1.0);
        srl2->write(THROTTLE_START + std::to_string(v) + THROTTLE_END);
    }
    else {
        ROS_WARN("Received negative X velocity value. Our robot cannot move backwards. Ignoring.");
    }
}

void KymcoMaxxer90AckermannSteeringController::writeSteeringSerial() {
    srl1->write(STEERING_START + std::to_string((int)target_steering_angle) + STEERING_END);
}

double KymcoMaxxer90AckermannSteeringController::norm(const double& v1, const double& s1, const double& e1, const double& s2, const double& e2) {
    return (((v1 - s1) * (e2 - s2)) / (e1 - s1)) + s2;
}

