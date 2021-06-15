#ifndef TRAJ_PLAN_H_
#define TRAJ_PLAN_H_

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <robot_info/robot_macro.h>

namespace traj_plan
{
class SingleJointPlanner
{
public:
    double length_;

    SingleJointPlanner();
    ~SingleJointPlanner();

    void init(const double start_p, const double start_v, const double start_a,
        const double target_p, const double target_v, const double target_a,
        const double tf, const double step);

    void rt_plan(const double t, double &p, double &v, double &a);
    void pre_plan(std::vector<double> &p, std::vector<double> &v, std::vector<double> &a);
    
private:
    double theta0_;
    double theta0Dot_;
    double theta0DDot_;

    double thetaf_;
    double thetafDot_;
    double thetafDDot_;

    double tf_;
    double step_;

    double a0_;
    double a1_;
    double a2_;
    double a3_;
    double a4_;
    double a5_;

};

class MultiJointPlanner
{
public:
    double length_;

    MultiJointPlanner();
    ~MultiJointPlanner();

    void init(std::vector<double> start_p, std::vector<double> start_v, std::vector<double> start_a,
        std::vector<double> target_p, std::vector<double> target_v, std::vector<double> target_a,
        const double tf, const double step);

    void rt_plan(const double t, 
        std::vector<double> &p, std::vector<double> &v, std::vector<double> &a);
    void pre_plan(std::vector<std::vector<double>> &p, 
        std::vector<std::vector<double>> &v, std::vector<std::vector<double>> &a);

private:
    SingleJointPlanner sjp[ARM_DOF];

    double step_;

};

}

#endif