#include <ros/ros.h>
#include <signal.h>
#include <robot_info/robot_info.h>
#include <robot_sh_memory/arm_shm.h>
#include <robot_sh_memory/sem_common.h>
#include <robot_sh_memory/shm_common.h>
#include <iostream>
#include <math.h>

void toExit(int sig)
{
    std::cout << "do some things before ros shutdown..." << std::endl;
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "virtual_robot", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    signal(SIGINT, toExit);

    // store robot information
    std::shared_ptr<robot_info::Robot> robot = 
        std::make_shared<robot_info::Robot>();

    arm_shm::Arm *arm_shm;
    int arm_shm_id;
    int arm_sem_id;

    // arm shared memory
    arm_shm_id = shm_common::create_shm(robot->arm_->shm_key_, &arm_shm);
    if (arm_shm_id == SHM_STATE_NO)
    {
        ROS_ERROR("Create arm shared memory failed.");
        return false;
    }

    arm_sem_id = sem_common::create_semaphore(robot->arm_->sem_key_);
    if (arm_sem_id == SEM_STATE_NO)
    {
        ROS_ERROR("Create arm semaphore failed.");
        return false;
    }

    // Firstly, set the default control mode,
    // read data from real robot, 
    // then write them into current and command shared memory.
    sem_common::semaphore_p(arm_sem_id);
    for (unsigned int j=0; j< robot->arm_->dof_; j++)
    {
        arm_shm->control_modes_[j] = robot->position_mode_;

        arm_shm->cmd_positions_[j] = 1; // data is from real robot
        arm_shm->cmd_velocities_[j] = 0; // zeros
        arm_shm->cmd_efforts_[j] = 0; // zeros
    }
    sem_common::semaphore_v(arm_sem_id);
    
    double rate = 1000; // 1000 Hz
    ros::Rate loop_rate(rate);
    double t = -1/rate;
    while (ros::ok())
    {
        t += 1/rate;

        sem_common::semaphore_p(arm_sem_id);
        for (unsigned int j=0; j< robot->arm_->dof_; j++)
        {
            // data is from real robot
            arm_shm->cur_positions_[j] = 11; 
            arm_shm->cur_velocities_[j] = 12;
            arm_shm->cur_efforts_[j] = 13;
        }

        if (arm_shm->control_modes_[0] == robot->position_mode_)
        {
            for (unsigned int j=0; j< robot->arm_->dof_; j++)
            {
                // write data to real robot
                std::cout << "cmd_position[" << j << "] = " 
                    << arm_shm->cmd_positions_[j] << std::endl;
            }
        }
        else if (arm_shm->control_modes_[0] == robot->velocity_mode_)
        {
            for (unsigned int j=0; j< robot->arm_->dof_; j++)
            {
                std::cout << "cmd_velocity[" << j << "] = " 
                    << arm_shm->cmd_velocities_[j] << std::endl;
            }
        }
        else if (arm_shm->control_modes_[0] == robot->effort_mode_)
        {
            for (unsigned int j=0; j< robot->arm_->dof_; j++)
            {
                std::cout << "cmd_effort[" << j << "] = " 
                    << arm_shm->cmd_efforts_[j] << std::endl;
            }
        }
        sem_common::semaphore_v(arm_sem_id);
    }

    return 0;
}
