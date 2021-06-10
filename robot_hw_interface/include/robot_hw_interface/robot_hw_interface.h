#ifndef ROBOT_HW_INTERFACE_H_
#define ROBOT_HW_INTERFACE_H_

#include <iostream>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <pluginlib/class_list_macros.hpp>
#include <ros/ros.h>
#include <vector>
#include <string>
#include <robot_info/robot_info.h>
#include <robot_sh_memory/arm_shm.h>
#include <robot_sh_memory/sem_common.h>
#include <robot_sh_memory/shm_common.h>

namespace robot_hw_interface
{
class RobotHwInterface: public hardware_interface::RobotHW
{
public:
    RobotHwInterface();
    ~RobotHwInterface();

    bool init(const std::string ns);
    void read(const ros::Time& time, const ros::Duration& period);
    void write(const ros::Time& time, const ros::Duration& period);

private:    
    // joint states interfaces
    hardware_interface::JointStateInterface joint_state_interface_;

    // joint position commands interfaces
    hardware_interface::PositionJointInterface joint_position_interface_;

    // joint velocity commands interfaces
    hardware_interface::VelocityJointInterface joint_velocity_interface_;

    // joint effort commands interfaces
    hardware_interface::EffortJointInterface joint_effort_interface_;

    // store robot information
    std::shared_ptr<robot_info::Robot> robot_ = 
        std::make_shared<robot_info::Robot>();

    arm_shm::Arm *arm_shm_;
    int arm_shm_id_;
    int arm_sem_id_;

};

}

#endif