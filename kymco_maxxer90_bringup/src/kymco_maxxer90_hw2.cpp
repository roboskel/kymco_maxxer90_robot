#include <kymco_maxxer90_motor_controller/kymco_maxxer90_hw2.hpp>

using namespace kymco_maxxer90_ackermann_steering_controller;

KymcoMaxxer90AckermannSteeringController::KymcoMaxxer90AckermannSteeringController(const ros::NodeHandle& n, const std::string& port1, const std::string& port2, const std::string& port3, const int& baudrate1, const std::string& device_name1, const std::string& device_name2, const std::string& device_name3) {

    // NOTE:
    // srl1 -> Steering
    // srl2 -> Throttle
    // srl3 -> spraying/encoders

    std::vector<std::string> ports = {port1, port2, port3};

    discoverDevices(ports, baudrate1);
    actuatorsReset();

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

    encoders_timer = nh.createTimer(ros::Duration(0.1), &KymcoMaxxer90AckermannSteeringController::readEncoders, this);

    throttle_feedback_timer = nh.createTimer(ros::Duration(0.1), &KymcoMaxxer90AckermannSteeringController::throttleFeedback, this);

    steering_feedback_timer = nh.createTimer(ros::Duration(0.1), &KymcoMaxxer90AckermannSteeringController::steeringFeedback, this);

    spray_sub = nh.subscribe("/kymco_maxxer90_ackermann_steering_controller/spray_valve", 1, &KymcoMaxxer90AckermannSteeringController::sprayingCallback, this);

}

KymcoMaxxer90AckermannSteeringController::~KymcoMaxxer90AckermannSteeringController() {
    serialClose();
}

void KymcoMaxxer90AckermannSteeringController::update(const ros::TimerEvent& e) {
    elapsed_time = ros::Duration(e.current_real - e.last_real);

    steering_actuator_pos = steering_actuator_pos != target_sap ? std::max(0.0, std::min(double(target_sap - steering_reading * steering_m) / MS_FROM_MIN_TO_MAX_STEERING, 1.0)) : steering_actuator_pos;
    throttle_actuator_pos = throttle_actuator_pos != target_tap ? std::max(0.0, std::min(double(target_tap - throttle_reading * throttle_m) / MS_FROM_ZERO_TO_MAX_THROTTLE, 1.0)) : throttle_actuator_pos;

    target_steering_angle = norm(angular_velocity, 1.0, -1.0, MIN_ANGZ, MAX_ANGZ);

    double delta_steering = std::min(abs(target_steering_angle - curr_steering_angle), elapsed_time.toSec() * DEG_PER_SEC);
    curr_steering_angle = target_steering_angle > curr_steering_angle ? curr_steering_angle + delta_steering : curr_steering_angle - delta_steering;

    double new_steering_pos = norm(curr_steering_angle, MIN_ANGZ, MAX_ANGZ, -CENTERED_RAD_STEERING, CENTERED_RAD_STEERING);

    double dx = encoder_reading * METERS_PER_ENCODER_TICK; // m

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
        std::string response = srl3->read(256);
        if (not response.empty()) {
            encoder_reading = std::stoi(response);
            ros::Time t = ros::Time::now();
            elapsed_time2 = t - last_encoder_reading;
            last_encoder_reading = t;
        }
    }
    catch (...) {}
}

void KymcoMaxxer90AckermannSteeringController::throttleFeedback(const ros::TimerEvent& e) {
    try {
        std::string response = srl2->read(256);
        if (not response.empty()) {
            throttle_reading = std::stoi(response);
        }
    }
    catch (...) {}
}

void KymcoMaxxer90AckermannSteeringController::steeringFeedback(const ros::TimerEvent& e) {
    try {
        std::string response = srl1->read(256);
        if (not response.empty()) {
            steering_reading = std::stoi(response);
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

void KymcoMaxxer90AckermannSteeringController::actuatorsReset() {
    target_tap = 0.0;
    target_sap = 0.5;
    throttle_actuator_pos = 0.0;
    steering_actuator_pos = 0.5;
    std::string command = THROTTLE_START_N + THROTTLE_DEC + std::to_string(MS_FROM_ZERO_TO_MAX_THROTTLE) + THROTTLE_END_N;
    ROS_WARN_STREAM(command);
    srl2->write(command);
    command = STEERING_START_N + STEERING_LEFT + std::to_string(MS_FROM_MIN_TO_MAX_STEERING) + STEERING_END_N;
    ROS_WARN_STREAM(command);
    srl1->write(command);
    command = STEERING_START_N + STEERING_RIGHT + std::to_string(MS_FROM_MIN_TO_MAX_STEERING/2) + STEERING_END_N;
    srl1->write(command);
    ROS_WARN_STREAM(command);
    ros::Duration(std::max(MS_FROM_MIN_TO_MAX_STEERING, MS_FROM_ZERO_TO_MAX_THROTTLE)/1000.f).sleep();
}

void KymcoMaxxer90AckermannSteeringController::discoverDevices(std::vector<std::string> ports,  const int baudrate) {
    ROS_INFO("Searching for devices...");
    while (not ports.empty()) {
        for (int i=0; i<ports.size(); i++) {
            // Find the steering controller first
            if (ports.size() == 3) {
                serialInit(srl1, ports[i], baudrate, "steering controller");
                srl1->write(STEERING_START_N + STEERING_LEFT + "0" + STEERING_END_N);
                std::string response = srl1->read(256);
		ROS_ERROR_STREAM(response);
                if (response.find("OK") < response.length()) {
                    ROS_INFO_STREAM("Found steering controller in port " + ports[i]);
                    ports.erase(ports.begin() + i);
                    break;
                }
            }
            // Then the throttle controller
            else if (ports.size() == 2) {
                serialInit(srl2, ports[i], baudrate, "throttle controller");
                srl2->write(THROTTLE_START_N + THROTTLE_DEC + "0" + THROTTLE_END_N);
                std::string response = srl2->read(256);
		ROS_ERROR_STREAM(response);
                if (response.find("OK") < response.length()) {
                    ROS_INFO_STREAM("Found throttle controller in port " + ports[i]);
                    ports.erase(ports.begin() + i);
                    break;
                }
            }
            // And assign the remaining port to the misc. controller
            else {
                serialInit(srl3, ports[0], baudrate, "misc. controller");
                ROS_INFO_STREAM("Found misc. controller in port " + ports[0]);
                ports.clear();
            }
        }
    }
}

void KymcoMaxxer90AckermannSteeringController::serialInit(serial::Serial *&srl, const std::string& port, const int& baudrate, const std::string& device_name) {
    while(ros::ok()) {
        try {
            ROS_INFO_STREAM("Looking for the " << device_name << " at port " << port);
            srl = new serial::Serial(port, baudrate, serial::Timeout::simpleTimeout(150));
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
    if (prev_l != linear_velocity) {
        if (linear_velocity >= 0) {
            target_tap = norm(linear_velocity, MIN_VELX_MS, MAX_VELX_MS, 0.0, 1.0);
            int c = (target_tap - throttle_actuator_pos) * MS_FROM_ZERO_TO_MAX_THROTTLE;
            std::string command = c > 0 ? THROTTLE_START_N + THROTTLE_INC : THROTTLE_START_N + THROTTLE_DEC;
            throttle_m = c > 0 ? 1 : -1;
            throttle_command = abs(c);
            command += std::to_string(throttle_command) + THROTTLE_END_N;
            srl3->write(command);
            prev_l = linear_velocity;
        }
        else {
            ROS_WARN("Received negative X velocity value. Our robot cannot move backwards. Ignoring.");
        }
    }
}

void KymcoMaxxer90AckermannSteeringController::writeSteeringSerial() {
    if (prev_a != target_steering_angle) {
        target_sap = norm(target_steering_angle, 1.0, -1.0, 0.0, 1.0);
        int c = (target_sap - steering_actuator_pos) / MS_FROM_MIN_TO_MAX_STEERING;
        std::string command = c > 0 ? STEERING_START_N + STEERING_RIGHT : STEERING_START_N + STEERING_LEFT;
        steering_m = c > 0 ? 1 : -1;
        steering_command = abs(c);
        command += std::to_string(steering_command) + STEERING_END_N;
        srl3->write(command);
        prev_a = target_steering_angle;
    }
}

double KymcoMaxxer90AckermannSteeringController::norm(const double& v1, const double& s1, const double& e1, const double& s2, const double& e2) {
    return (((v1 - s1) * (e2 - s2)) / (e1 - s1)) + s2;
}

