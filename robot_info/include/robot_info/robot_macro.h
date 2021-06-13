#ifndef ROBOT_MACRO_H_
#define ROBOT_MACRO_H_

#include <iostream>
#include <string>

// robot arm degree of freedom
#define ARM_DOF 6

// They are used to generate key values for shared memory and semaphore.
#define ARM_SHM_FILE  "/usr/local/robot_files/robot_arm_shm" 
#define ARM_SEM_FILE  "/usr/local/robot_files/robot_arm_sem"

// robot arm joint names, it must be same as the sdf, "robot_hw_interface/config/robot_hardware.yaml"
// and "robot_hw_interface/config/robot_controllers.yaml".
const std::string ROBOT_ARM_JOINT_NAMES[ARM_DOF] = 
    {"shoulder_pan","shoulder_lift","elbow","wrist_1","wrist_2","wrist_3"};

// robot control mode
#define ROBOT_POSITION_MODE  1<<0
#define ROBOT_VELOCITY_MODE  1<<1
#define ROBOT_EFFORT_MODE    1<<2

// This corresponds to robot hardware interface in "robot_hw_interface/config/robot_hardware.yaml".
#define ROBOT_ARM_JOINTS  "/robot/arm/joints"

// This corresponds to robot hardware interface in "robot_hw_interface/config/robot_controllers.yaml".
#define ROBOT_ARM_POSITION_CONTROLLER  "/robot/arm/position_controller"
#define ROBOT_ARM_VELOCITY_CONTROLLER  "/robot/arm/velocity_controller"
#define ROBOT_ARM_EFFORT_CONTROLLER    "/robot/arm/effort_controller"

#endif