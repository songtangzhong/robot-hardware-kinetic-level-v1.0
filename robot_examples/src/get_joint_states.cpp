#include <ros/ros.h>
#include <robot_sdk/robot_sdk.h>
#include <iostream>
#include <vector>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_joint_states");
    ros::NodeHandle nh;

    std::shared_ptr<robot_sdk::Robot> robot =
        std::make_shared<robot_sdk::Robot>();
    robot->wait_for_ready(3);

    std::vector<double> cur_positions;
    std::vector<double> cur_velocities;
    std::vector<double> cur_efforts;
    cur_positions.resize(robot->dof_);
    cur_velocities.resize(robot->dof_);
    cur_efforts.resize(robot->dof_);
    
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        ros::spinOnce();
        robot->get_joint_positions(cur_positions);
        robot->get_joint_velocities(cur_velocities);
        robot->get_joint_efforts(cur_efforts);
        for (unsigned int j=0; j< robot->dof_; j++)
        {
            std::cout << "cur_positions[" << j << "] = " << cur_positions[j] << std::endl;
        }
        for (unsigned int j=0; j< robot->dof_; j++)
        {
            std::cout << "cur_velocities[" << j << "] = " << cur_velocities[j] << std::endl;
        }
        for (unsigned int j=0; j< robot->dof_; j++)
        {
            std::cout << "cur_efforts[" << j << "] = " << cur_efforts[j] << std::endl;
        }
        
        loop_rate.sleep();
    }

    ros::shutdown();
    return 0;
}
