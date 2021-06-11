#include <robot_info/arm_info.h>
#include <robot_info/robot_macro.h>
#include <sys/sem.h>

namespace arm_info
{
Arm::Arm()
{
    dof_ = ARM_DOF;

    joint_names_.resize(dof_);

    control_modes_.resize(dof_);
    
    cur_positions_.resize(dof_);
    cur_velocities_.resize(dof_);
    cur_efforts_.resize(dof_);

    cmd_positions_.resize(dof_);
    cmd_velocities_.resize(dof_);
    cmd_efforts_.resize(dof_);

    for (unsigned int j=0; j<dof_; j++)
    {
        control_modes_[j] = ROBOT_POSITION_MODE; // default: position mode
        // control_modes_[j] = ROBOT_VELOCITY_MODE; 
        // control_modes_[j] = ROBOT_EFFORT_MODE; 

        cur_positions_[j] = cmd_positions_[j] = 0;
        cur_velocities_[j] = cmd_velocities_[j] = 0;
        cur_efforts_[j] = cmd_efforts_[j] = 0;
    }

    shm_key_ = ftok(ARM_SHM_FILE, 1);
    if (shm_key_ == -1)
    {
        ROS_ERROR("Generate key value of arm shared memory failed.");
    }

    sem_key_ = ftok(ARM_SEM_FILE, 1);
    if (sem_key_ == -1)
    {
        ROS_ERROR("Generate key value of arm semaphore failed.");
    }
}

Arm::~Arm(){}

}
