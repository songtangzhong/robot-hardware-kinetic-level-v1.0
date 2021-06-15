#include <ros/ros.h>
#include <robot_sdk/robot_sdk.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <traj_plan/traj_plan.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "multi_joint_traj_plan_pre");
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
    cur_positions.resize(robot->dof_, 0);
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

    std::shared_ptr<traj_plan::MultiJointPlanner> planner =
        std::make_shared<traj_plan::MultiJointPlanner>();
    double tf = 5;
    double step = 0.001;
    planner->init(cur_positions, cur_velocities, cur_accelerations,
        target_positions, target_velocities, target_accelerations,
        tf, step);
    std::cout << "planning points length: " << planner->length_ << std::endl;

    std::vector<std::vector<double>> p;
    std::vector<std::vector<double>> v;
    std::vector<std::vector<double>> a;
    p.resize(planner->length_);
    v.resize(planner->length_);
    a.resize(planner->length_);
    for (unsigned int j=0; j<planner->length_; j++)
    {
        p[j].resize(robot->dof_);
        v[j].resize(robot->dof_);
        a[j].resize(robot->dof_);
    }
    planner->pre_plan(p, v, a);
    
    double rate = 1000;
    ros::Rate loop_rate(rate);

    while (ros::ok())
    {
        for (unsigned int j=0; j< planner->length_; j++)
        {
            robot->set_joint_positions(p[j]);
            loop_rate.sleep();
        }

        for (unsigned int j=0; j< robot->dof_; j++)
        {
            cur_positions[j] = target_positions[j];
            target_positions[j] = -0.5;
        }
        planner->init(cur_positions, cur_velocities, cur_accelerations,
        target_positions, target_velocities, target_accelerations,
        tf, step);
        planner->pre_plan(p, v, a);

        for (unsigned int j=0; j< planner->length_; j++)
        {
            robot->set_joint_positions(p[j]);
            loop_rate.sleep();
        }
        
        break;
    }

    ros::shutdown();
    return 0;
}
