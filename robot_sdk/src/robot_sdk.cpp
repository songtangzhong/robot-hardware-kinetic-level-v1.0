#include <robot_sdk/robot_sdk.h>
#include <robot_info/robot_macro.h>
#include <vector>

namespace robot_sdk
{
Robot::Robot()
{
    switch_controller_cli_ = nh_.serviceClient<controller_manager_msgs::SwitchController>(
        "/controller_manager/switch_controller");

    // arm shared memory
    arm_shm_id_ = shm_common::create_shm(robot_->arm_->shm_key_, &arm_shm_);
    if (arm_shm_id_ == SHM_STATE_NO)
    {
        ROS_ERROR("Create arm shared memory failed.");
    }

    arm_sem_id_ = sem_common::create_semaphore(robot_->arm_->sem_key_);
    if (arm_sem_id_ == SEM_STATE_NO)
    {
        ROS_ERROR("Create arm semaphore failed.");
    }
    // end arm shared memory
}

Robot::~Robot(){}

void Robot::wait_for_ready(int seconds)
{
    int times = seconds;
    ros::Rate rate(1);
    while (ros::ok())
    {
        ROS_INFO("wait for ready...%d s.", times);
        rate.sleep();
        if (times-- == 0)
        {
            break;
        }
    }
}

int Robot::switch_controller(const std::string &start_controller)
{
    if ((start_controller!=ROBOT_ARM_POSITION_CONTROLLER) 
        && (start_controller!=ROBOT_ARM_VELOCITY_CONTROLLER) 
        && (start_controller!=ROBOT_ARM_EFFORT_CONTROLLER))
    {
        ROS_WARN("No such controller, please confirm.");
        return 1;
    }

    std::string cur_controller;

    sem_common::semaphore_p(arm_sem_id_);
    if (arm_shm_->control_modes_[0] == robot_->position_mode_)
    {
        cur_controller = ROBOT_ARM_POSITION_CONTROLLER;
    }
    else if (arm_shm_->control_modes_[0] == robot_->velocity_mode_)
    {
        cur_controller = ROBOT_ARM_VELOCITY_CONTROLLER;
    }
    else if (arm_shm_->control_modes_[0] == robot_->effort_mode_)
    {
        cur_controller = ROBOT_ARM_EFFORT_CONTROLLER;
    }
    sem_common::semaphore_v(arm_sem_id_);

    if (cur_controller == start_controller)
    {
        ROS_INFO("Current controller is [%s], don't need to switch.", start_controller.c_str());
        return 1;
    }

    std::vector<std::string> start_controller_ = {start_controller};
    std::vector<std::string> stop_controller_ = {cur_controller};
    controller_manager_msgs::SwitchController req;
    req.request.start_controllers = start_controller_;
    req.request.stop_controllers = stop_controller_;
    req.request.strictness = 1;

    if (switch_controller_cli_.call(req))
    {
        ROS_INFO("Switch to controller [%s] successfully.", start_controller.c_str());
    }
    else
    {
        ROS_WARN("Switch to controller [%s] failed.", start_controller.c_str());
        return -1;
    }

    sem_common::semaphore_p(arm_sem_id_);
    if (start_controller == ROBOT_ARM_POSITION_CONTROLLER)
    {
        for (unsigned int j=0; j< robot_->arm_->dof_; j++)
        {
            arm_shm_->control_modes_[j] = robot_->position_mode_;
        }
    }
    else if (start_controller == ROBOT_ARM_VELOCITY_CONTROLLER)
    {
        for (unsigned int j=0; j< robot_->arm_->dof_; j++)
        {
            arm_shm_->control_modes_[j] = robot_->velocity_mode_;
        }
    }
    else if (start_controller == ROBOT_ARM_EFFORT_CONTROLLER)
    {
        for (unsigned int j=0; j< robot_->arm_->dof_; j++)
        {
            arm_shm_->control_modes_[j] = robot_->effort_mode_;
        }
    }
    sem_common::semaphore_v(arm_sem_id_);

    return 1;
}

}
