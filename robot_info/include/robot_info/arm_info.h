#ifndef ARM_INFO_H_
#define ARM_INFO_H_

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <string>

namespace arm_info
{
class Arm
{
public:
    Arm();
    ~Arm();

    unsigned int dof_;

    std::vector<std::string> joint_names_;

    std::vector<unsigned int> control_modes_;

    std::vector<double> cur_positions_;
    std::vector<double> cur_velocities_;
    std::vector<double> cur_efforts_;

    std::vector<double> cmd_positions_;
    std::vector<double> cmd_velocities_;
    std::vector<double> cmd_efforts_;

    key_t shm_key_;
    key_t sem_key_;

private:

};

}

#endif