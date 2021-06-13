#ifndef ROBOT_SDK_H_
#define ROBOT_SDK_H_

#include <ros/ros.h>
#include <controller_manager_msgs/SwitchController.h>
#include <iostream>
#include <string.h>
#include <robot_info/robot_info.h>
#include <robot_sh_memory/arm_shm.h>
#include <robot_sh_memory/sem_common.h>
#include <robot_sh_memory/shm_common.h>

namespace robot_sdk
{
class Robot
{
public:
    Robot();
    ~Robot();

    int switch_controller(const std::string &start_controller);

private:
    ros::NodeHandle nh_;

    ros::ServiceClient switch_controller_cli_;

    // store robot information
    std::shared_ptr<robot_info::Robot> robot_ = 
        std::make_shared<robot_info::Robot>();

    arm_shm::Arm *arm_shm_;
    int arm_shm_id_;
    int arm_sem_id_;

};

}

#endif