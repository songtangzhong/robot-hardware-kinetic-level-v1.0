#include <ros/ros.h>
#include <robot_sdk/robot_sdk.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <traj_plan/traj_plan.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "track_trajectory");
    ros::NodeHandle nh;

    std::shared_ptr<robot_sdk::Robot> robot =
        std::make_shared<robot_sdk::Robot>();
    robot->wait_for_ready(3);

    std::vector<double> cur_positions;
    std::vector<double> cur_velocities;
    std::vector<double> cur_accelerations;
    std::vector<double> target_positions;
    std::vector<double> target_velocities;
    std::vector<double> target_accelerations;
    cur_positions.resize(robot->dof_);
    cur_velocities.resize(robot->dof_, 0);
    cur_accelerations.resize(robot->dof_, 0);
    target_positions.resize(robot->dof_, 0);
    target_velocities.resize(robot->dof_, 0);
    target_accelerations.resize(robot->dof_, 0);

    while (ros::ok())
    {
        ros::spinOnce();
        robot->get_joint_positions(cur_positions);
        break;
    }
    for (unsigned int j=0; j< robot->dof_; j++)
    {
        // target_positions[j] = cur_positions[j]+0.5;
    }
    // target_positions[2] += (-1.57);

    std::shared_ptr<traj_plan::MultiJointPlanner> planner =
        std::make_shared<traj_plan::MultiJointPlanner>();
    planner->init(cur_positions, cur_velocities, cur_accelerations,
        target_positions, target_velocities, target_accelerations,
        5, 0.001);
    std::cout << "planning points length: " << planner->length_ << std::endl;
    
    double rate = 1000;
    double t = -1.0/rate;
    ros::Rate loop_rate(rate);

    std::vector<double> p;
    std::vector<double> v;
    std::vector<double> a;
    p.resize(robot->dof_);
    v.resize(robot->dof_);
    a.resize(robot->dof_);

    while (ros::ok())
    {
        t += 1.0/rate;

        planner->rt_plan(t, p, v, a);
        for (unsigned int j=0; j< robot->dof_; j++)
        {
            std::cout << "p[" << j << "] = " << p[j] << std::endl;
        }
        robot->set_joint_positions(p);
        loop_rate.sleep();
    }

    ros::shutdown();
    return 0;
}
