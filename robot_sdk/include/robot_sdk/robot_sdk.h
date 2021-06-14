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
#include <vector>
#include <sensor_msgs/JointState.h>
#include <mutex>
#include <std_msgs/Float64MultiArray.h>
#include <robot_info/robot_macro.h>

namespace robot_sdk
{
class Robot
{
public:
    unsigned int dof_;
    
    Robot();
    ~Robot();

    void wait_for_ready(int times);

    int switch_controller(const std::string &start_controller);

    int get_joint_positions(std::vector<double> &positions);
    int get_joint_velocities(std::vector<double> &velocities);
    int get_joint_efforts(std::vector<double> &efforts);

    int set_joint_positions(std::vector<double> positions);

private:
    void arm_joint_states_cb_(const sensor_msgs::JointState::ConstPtr &msg);

    ros::NodeHandle nh_;

    ros::ServiceClient switch_controller_cli_;

    ros::Subscriber arm_joint_states_sub_;

    ros::Publisher joint_position_cmds_pub_;

    // store robot information
    std::shared_ptr<robot_info::Robot> robot_ = 
        std::make_shared<robot_info::Robot>();

    arm_shm::Arm *arm_shm_;
    int arm_shm_id_;
    int arm_sem_id_;

    std::vector<double> cur_positions_;
    std::vector<double> cur_velocities_;
    std::vector<double> cur_efforts_;

    std::mutex mtx_;

};

}

#endif