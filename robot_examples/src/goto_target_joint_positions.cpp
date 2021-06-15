#include <ros/ros.h>
#include <robot_sdk/robot_sdk.h>
#include <iostream>
#include <vector>
#include <stdlib.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "goto_target_joint_positions");
    ros::NodeHandle nh;

    std::shared_ptr<robot_sdk::Robot> robot =
        std::make_shared<robot_sdk::Robot>();
    
    if (argc!=(1+robot->dof_))
    {
        ROS_WARN("target joint positions size is not matched.");
        return 0;
    }

    robot->wait_for_ready(3);

    std::vector<double> target_positions;
    target_positions.resize(robot->dof_);
    for (unsigned int j=0; j< robot->dof_; j++)
    {
        target_positions[j] = strtod(argv[j+1],NULL);
    }

    robot->move_to_target_joint_positions(target_positions, 5);

    ros::shutdown();
    return 0;
}
