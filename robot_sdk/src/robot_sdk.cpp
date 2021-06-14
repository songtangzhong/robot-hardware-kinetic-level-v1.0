#include <robot_sdk/robot_sdk.h>
#include <robot_info/robot_macro.h>
#include <vector>

namespace robot_sdk
{
Robot::Robot()
{
    switch_controller_cli_ = nh_.serviceClient<controller_manager_msgs::SwitchController>(
        "/controller_manager/switch_controller");
    arm_joint_states_sub_ = nh_.subscribe("/joint_states", 100, &Robot::arm_joint_states_cb_, this);

    joint_position_cmds_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(
            ROBOT_ARM_POSITION_CONTROLLER_TOPIC, 10);

    dof_ = robot_->arm_->dof_;

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

    cur_positions_.resize(robot_->arm_->dof_);
    cur_velocities_.resize(robot_->arm_->dof_);
    cur_efforts_.resize(robot_->arm_->dof_);
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

void Robot::arm_joint_states_cb_(const sensor_msgs::JointState::ConstPtr &msg)
{
    std::lock_guard<std::mutex> guard(mtx_);

    for (unsigned int j=0; j< robot_->arm_->dof_; j++)
    {
        cur_positions_[j] = msg->position[j];
        cur_velocities_[j] = msg->velocity[j];
        cur_efforts_[j] = msg->effort[j];
    }
}

int Robot::get_joint_positions(std::vector<double> &positions)
{
    if (positions.size()!=robot_->arm_->dof_)
    {
        ROS_ERROR("positions size is not matched.");
        return -1;
    }

    std::lock_guard<std::mutex> guard(mtx_);

    for (unsigned int j=0; j< robot_->arm_->dof_; j++)
    {
        positions[j] = cur_positions_[j];
    }

    return 1;
}

int Robot::get_joint_velocities(std::vector<double> &velocities)
{
    if (velocities.size()!=robot_->arm_->dof_)
    {
        ROS_ERROR("velocities size is not matched.");
        return -1;
    }

    std::lock_guard<std::mutex> guard(mtx_);

    for (unsigned int j=0; j< robot_->arm_->dof_; j++)
    {
        velocities[j] = cur_velocities_[j];
    }

    return 1;
}

int Robot::get_joint_efforts(std::vector<double> &efforts)
{
    if (efforts.size()!=robot_->arm_->dof_)
    {
        ROS_ERROR("efforts size is not matched.");
        return -1;
    }

    std::lock_guard<std::mutex> guard(mtx_);

    for (unsigned int j=0; j< robot_->arm_->dof_; j++)
    {
        efforts[j] = cur_efforts_[j];
    }

    return 1;
}

int Robot::set_joint_positions(std::vector<double> positions)
{
    if (positions.size()!=robot_->arm_->dof_)
    {
        ROS_ERROR("command positions size is not matched.");
        return -1;
    }

    std_msgs::Float64MultiArray cmd;
    cmd.data.resize(robot_->arm_->dof_);
    for (unsigned int j=0; j< robot_->arm_->dof_; j++)
    {
        cmd.data[j] = positions[j];
    }
    joint_position_cmds_pub_.publish(cmd);

    return 1;
}

}
