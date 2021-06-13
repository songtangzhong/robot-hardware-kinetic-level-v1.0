#ifndef GAZEBO_CONTROL_PLUGIN_H_
#define GAZEBO_CONTROL_PLUGIN_H_

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/Joint.hh>
#include <robot_info/robot_info.h>
#include <robot_sh_memory/arm_shm.h>
#include <robot_sh_memory/sem_common.h>
#include <robot_sh_memory/shm_common.h>
#include <iostream>
#include <vector>

namespace gazebo_control_plugin
{
class ControlPlugin : public gazebo::ModelPlugin
{
public:
    ControlPlugin();
    ~ControlPlugin();

    void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf);

    void Update();

private:
    gazebo::physics::ModelPtr parent_model_;

    std::vector<gazebo::physics::JointPtr> arm_joints_;

    gazebo::event::ConnectionPtr update_connection_;

    // store robot information
    std::shared_ptr<robot_info::Robot> robot_ = 
        std::make_shared<robot_info::Robot>();

    arm_shm::Arm *arm_shm_;
    int arm_shm_id_;
    int arm_sem_id_;
}; 

}

#endif
