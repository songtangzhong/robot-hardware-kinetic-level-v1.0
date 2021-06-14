#include <gazebo_control_plugin/gazebo_control_plugin.h>
#include <ros/ros.h>
#include <robot_info/robot_macro.h>

namespace gazebo_control_plugin
{
ControlPlugin::ControlPlugin()
{
    ROS_INFO("Start Robot Simulation ...");

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

    for (unsigned int j=0; j< robot_->arm_->dof_; j++)
    {
        robot_->arm_->joint_names_[j] = ROBOT_ARM_JOINT_NAMES[j];
    }
}

ControlPlugin::~ControlPlugin()
{ 
    ROS_INFO("Simulation has been shutdown.");
}

void ControlPlugin::Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf)
{
    ROS_INFO("Load control plugin ...");

    parent_model_ = parent;

    for (unsigned int j=0; j< robot_->arm_->dof_; j++)
    {
        gazebo::physics::JointPtr arm_joint = parent_model_->GetJoint(robot_->arm_->joint_names_[j]);
        if (!arm_joint)
        {
            ROS_ERROR_STREAM("This robot has a joint named \"" << robot_->arm_->joint_names_[j]
                << "\" which is not defined in the gazebo model.");
        }
        arm_joints_.push_back(arm_joint);
    }
    
    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        boost::bind(&ControlPlugin::Update, this));

    /* Set initial positions of gazebo model.
       This is used to simulate the real positions of real robot
       when system on power first time.
    */
    std::vector<double> initial_positions;
    initial_positions.resize(robot_->arm_->dof_);
    initial_positions[0] = 0;
    initial_positions[1] = -1.57;
    initial_positions[2] = 1.57;
    initial_positions[3] = 0;
    initial_positions[4] = 0;
    initial_positions[5] = 0;
    for (unsigned int j=0; j< robot_->arm_->dof_; j++)
    {
        arm_joints_[j]->SetPosition(0, initial_positions[j]);
    }
    
    // Set initial values of commands shared memory.
    for (unsigned int j=0; j< robot_->arm_->dof_; j++)
    {
        arm_shm_->control_modes_[j] = robot_->position_mode_;

        arm_shm_->cmd_positions_[j] = arm_joints_[j]->GetAngle(0).Radian(); // data from real robot
        arm_shm_->cmd_velocities_[j] = 0;
        arm_shm_->cmd_efforts_[j] = 0;
    }

    ROS_INFO("Load control plugin successfully.");
}

void ControlPlugin::Update()
{
    sem_common::semaphore_p(arm_sem_id_);
    if (arm_shm_->control_modes_[0] == robot_->position_mode_)
    {
        for (unsigned int j=0; j< robot_->arm_->dof_; j++)
        {
            arm_joints_[j]->SetPosition(0, arm_shm_->cmd_positions_[j]);
        }
    }
    else if (arm_shm_->control_modes_[0] == robot_->velocity_mode_)
    {
        for (unsigned int j=0; j< robot_->arm_->dof_; j++)
        {
            arm_joints_[j]->SetVelocity(0, arm_shm_->cmd_velocities_[j]);
        }
    }
    else if (arm_shm_->control_modes_[0] == robot_->effort_mode_)
    {
        for (unsigned int j=0; j< robot_->arm_->dof_; j++)
        {
            arm_joints_[j]->SetForce(0, arm_shm_->cmd_efforts_[j]);
        }
    }

    for (unsigned int j=0; j< robot_->arm_->dof_; j++)
    {
        // arm_joints_[j]->GetVelocity(0) and arm_joints_[j]->GetForce((unsigned int)(0))
        // can't work in current gazebo version, but this part can work in real robot rightly.
        arm_shm_->cur_positions_[j] = arm_joints_[j]->GetAngle(0).Radian();
        arm_shm_->cur_velocities_[j] = arm_joints_[j]->GetVelocity(0);
        arm_shm_->cur_efforts_[j] = arm_joints_[j]->GetForce((unsigned int)(0));

        // test code
        /*arm_shm_->cur_velocities_[j] = j;
        arm_shm_->cur_efforts_[j] = j*2;*/
        /*if (arm_shm_->control_modes_[0] == robot_->position_mode_)
        {
            std::cout << "arm_joints_[" << j << "]->GetAngle = "
                << arm_joints_[j]->GetAngle(0).Radian() << std::endl;
        }
        else if (arm_shm_->control_modes_[0] == robot_->velocity_mode_)
        {
            std::cout << "arm_joints_[" << j << "]->GetVelocity = "
                << arm_joints_[j]->GetVelocity(0) << std::endl;
        }
        else if (arm_shm_->control_modes_[0] == robot_->effort_mode_)
        {
            std::cout << "arm_joints_[" << j << "]->GetForce = "
                << arm_joints_[j]->GetForce((unsigned int)(0)) << std::endl;
        }*/
        // end test
    }
    sem_common::semaphore_v(arm_sem_id_);
}

GZ_REGISTER_MODEL_PLUGIN(ControlPlugin)

}
