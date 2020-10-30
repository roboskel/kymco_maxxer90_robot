cmake_minimum_required(VERSION 3.0.2)
project(kymco_maxxer90_bringup)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    serial
    nav_msgs
    sensor_msgs
    geometry_msgs
    controller_manager
    hardware_interface
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp serial hardware_interface 
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(motor_controller src/motor_controller.cpp src/kymco_maxxer90_hw.cpp)
target_link_libraries(motor_controller ${catkin_LIBRARIES})

