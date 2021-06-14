#include <ros/ros.h>
#include <robot_sdk/robot_sdk.h>
#include <iostream>
#include <vector>
#include <math.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "track_trajectory");
    ros::NodeHandle nh;

    std::shared_ptr<robot_sdk::Robot> robot =
        std::make_shared<robot_sdk::Robot>();
    robot->wait_for_ready(3);

    std::vector<double> cmd_positions;
    cmd_positions.resize(robot->dof_);

    while (ros::ok())
    {
        ros::spinOnce();
        robot->get_joint_positions(cmd_positions);
        break;
    }

    double cur_p2 = cmd_positions[2];
    
    double rate = 1000;
    double t = -1.0/rate;
    ros::Rate loop_rate(rate);
    while (ros::ok())
    {
        t += 1.0/rate;

        cmd_positions[2] = cur_p2+0.5*sin(t);

        robot->set_joint_positions(cmd_positions);
        
        loop_rate.sleep();
    }

    ros::shutdown();
    return 0;
}
