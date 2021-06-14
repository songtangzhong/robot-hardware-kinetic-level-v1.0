#include <robot_hw_interface/robot_hw_interface.h>

namespace robot_hw_interface
{
RobotHwInterface::RobotHwInterface(){}

RobotHwInterface::~RobotHwInterface(){}

bool RobotHwInterface::init(const std::string &ns)
{
    // arm shared memory
    arm_shm_id_ = shm_common::create_shm(robot_->arm_->shm_key_, &arm_shm_);
    if (arm_shm_id_ == SHM_STATE_NO)
    {
        ROS_ERROR("Create arm shared memory failed.");
        return false;
    }

    arm_sem_id_ = sem_common::create_semaphore(robot_->arm_->sem_key_);
    if (arm_sem_id_ == SEM_STATE_NO)
    {
        ROS_ERROR("Create arm semaphore failed.");
        return false;
    }
    // end arm shared memory

    // Set current real robot positions to controller variables.
    sem_common::semaphore_p(arm_sem_id_);    
    for (unsigned int j=0; j< robot_->arm_->dof_; j++)
    {
        robot_->arm_->cmd_positions_[j] = arm_shm_->cur_positions_[j];
        robot_->arm_->cmd_velocities_[j] = 0;
        robot_->arm_->cmd_efforts_[j] = 0;
    }
    sem_common::semaphore_v(arm_sem_id_);

    // get joint names and joint nums
    ros::param::get(ns, robot_->arm_->joint_names_);

    // register handles
    for (unsigned int i=0; i< robot_->arm_->dof_; i++)
    {
        // register state controller
        joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(
            robot_->arm_->joint_names_[i], &robot_->arm_->cur_positions_[i], 
            &robot_->arm_->cur_velocities_[i], &robot_->arm_->cur_efforts_[i]));

        // register position controller
        joint_position_interface_.registerHandle(hardware_interface::JointHandle(
            joint_state_interface_.getHandle(robot_->arm_->joint_names_[i]), 
            &robot_->arm_->cmd_positions_[i]));

        // register velocity controller
        joint_velocity_interface_.registerHandle(hardware_interface::JointHandle(
            joint_state_interface_.getHandle(robot_->arm_->joint_names_[i]), 
            &robot_->arm_->cmd_velocities_[i]));
        
        // register effort controller
        joint_effort_interface_.registerHandle(hardware_interface::JointHandle(
            joint_state_interface_.getHandle(robot_->arm_->joint_names_[i]), 
            &robot_->arm_->cmd_efforts_[i]));
    }

    // register interfaces
    registerInterface(&joint_state_interface_);
    registerInterface(&joint_position_interface_);
    registerInterface(&joint_velocity_interface_);
    registerInterface(&joint_effort_interface_);

    // return true for successful init or RobotHwInterface initialisation will fail
    return true;
}

void RobotHwInterface::read(const ros::Time& time, const ros::Duration& period)
{
    sem_common::semaphore_p(arm_sem_id_);
    for (unsigned int i=0; i< robot_->arm_->dof_; i++)
    {
        robot_->arm_->cur_positions_[i] = arm_shm_->cur_positions_[i];
        robot_->arm_->cur_velocities_[i] = arm_shm_->cur_velocities_[i];
        robot_->arm_->cur_efforts_[i] = arm_shm_->cur_efforts_[i];

        // test code
        /*if (arm_shm_->control_modes_[0] == robot_->position_mode_)
        {
            std::cout << "arm_shm_->cur_positions_[" << i << "] = "
                << arm_shm_->cur_positions_[i] << std::endl;
        }
        else if (arm_shm_->control_modes_[0] == robot_->velocity_mode_)
        {
            std::cout << "arm_shm_->cur_velocities_[" << i << "] = "
                << arm_shm_->cur_velocities_[i] << std::endl;
        }
        else if (arm_shm_->control_modes_[0] == robot_->effort_mode_)
        {
            std::cout << "arm_shm_->cur_efforts_[" << i << "] = "
                << arm_shm_->cur_efforts_[i] << std::endl;
        }*/
        // end test
    }
    sem_common::semaphore_v(arm_sem_id_);
}

void RobotHwInterface::write(const ros::Time& time, const ros::Duration& period)
{
    sem_common::semaphore_p(arm_sem_id_);
    if (arm_shm_->control_modes_[0] == robot_->position_mode_)
    {
        for (unsigned int i=0; i< robot_->arm_->dof_; i++)
        {
            arm_shm_->cmd_positions_[i] = robot_->arm_->cmd_positions_[i];
            /*std::cout << "robot_->arm_->cmd_positions_[" << i << "] = "
                << robot_->arm_->cmd_positions_[i] << std::endl;*/
        }
    }
    else if (arm_shm_->control_modes_[0] == robot_->velocity_mode_)
    {
        for (unsigned int i=0; i< robot_->arm_->dof_; i++)
        {
            arm_shm_->cmd_velocities_[i] = robot_->arm_->cmd_velocities_[i];
            /*std::cout << "robot_->arm_->cmd_velocities_[" << i << "] = "
                << robot_->arm_->cmd_velocities_[i] << std::endl;*/
        }
    }
    else if (arm_shm_->control_modes_[0] == robot_->effort_mode_)
    {
        for (unsigned int i=0; i< robot_->arm_->dof_; i++)
        {
            arm_shm_->cmd_efforts_[i] = robot_->arm_->cmd_efforts_[i];
            /*std::cout << "robot_->arm_->cmd_efforts_[" << i << "] = "
                << robot_->arm_->cmd_efforts_[i] << std::endl;*/
        }
    }
    sem_common::semaphore_v(arm_sem_id_);
}

}

PLUGINLIB_EXPORT_CLASS(robot_hw_interface::RobotHwInterface, hardware_interface::RobotHW)

