#ifndef ARM_SHM_H_
#define ARM_SHM_H_

#include <robot_info/robot_macro.h>

namespace arm_shm
{
typedef struct
{
    unsigned int control_modes_[ARM_DOF];
    
    double cur_positions_[ARM_DOF];
    double cur_velocities_[ARM_DOF];
    double cur_efforts_[ARM_DOF];

    double cmd_positions_[ARM_DOF];
    double cmd_velocities_[ARM_DOF];
    double cmd_efforts_[ARM_DOF];
} Arm;

}

#endif
