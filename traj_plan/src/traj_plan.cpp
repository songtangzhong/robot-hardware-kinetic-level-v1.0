#include <traj_plan/traj_plan.h>
#include <math.h>

namespace traj_plan
{
SingleJointPlanner::SingleJointPlanner(){}

SingleJointPlanner::~SingleJointPlanner(){}

void SingleJointPlanner::init(const double start_p, const double start_v, const double start_a,
        const double target_p, const double target_v, const double target_a,
        const double tf, const double step)
{
    theta0_ = start_p;
    theta0Dot_ = start_v;
    theta0DDot_ = start_a;

    thetaf_ = target_p;
    thetafDot_ = target_v;
    thetafDDot_ = target_a;

    tf_ = tf;
    step_ = step;
    length_ = (unsigned int)tf_/step_;

    a0_ = theta0_;
    a1_ = theta0Dot_;
    a2_ = theta0DDot_/2;
    a3_ = (20*thetaf_-20*theta0_-(8*thetafDot_+12*theta0Dot_)*tf_-(3*theta0DDot_-thetafDDot_)*pow(tf_,2))/(2*pow(tf_,3));
    a4_ = (30*theta0_-30*thetaf_+(14*thetafDot_+16*theta0Dot_)*tf_+(3*theta0DDot_-2*thetafDDot_)*pow(tf_,2))/(2*pow(tf_,4));
    a5_ = (12*thetaf_-12*theta0_-(6*thetafDot_+6*theta0Dot_)*tf_-(theta0DDot_-thetafDDot_)*pow(tf_,2))/(2*pow(tf_,5));
}

void SingleJointPlanner::rt_plan(const double t, double &p, double &v, double &a)
{
    if (t<tf_)
    {
        p = a0_+a1_*t+a2_*pow(t,2)+a3_*pow(t,3)+a4_*pow(t,4)+a5_*pow(t,5);
        v = a1_+2*a2_*t+3*a3_*pow(t,2)+4*a4_*pow(t,3)+5*a5_*pow(t,4);
        a = 2*a2_+6*a3_*t+12*a4_*pow(t,2)+20*a5_*pow(t,3);
    }
    else
    {
        p = a0_+a1_*tf_+a2_*pow(tf_,2)+a3_*pow(tf_,3)+a4_*pow(tf_,4)+a5_*pow(tf_,5);
        v = a1_+2*a2_*tf_+3*a3_*pow(tf_,2)+4*a4_*pow(tf_,3)+5*a5_*pow(tf_,4);
        a = 2*a2_+6*a3_*tf_+12*a4_*pow(tf_,2)+20*a5_*pow(tf_,3);
    }
}

void SingleJointPlanner::pre_plan(std::vector<double> &p, std::vector<double> &v, std::vector<double> &a)
{
    double t = -step_;

    for (unsigned int j=0; j<length_; j++)
    {
        t += step_;

        double p_;
        double v_;
        double a_;
        SingleJointPlanner::rt_plan(t, p_, v_, a_);

        p[j] = p_;
        v[j] = v_;
        a[j] = a_;
    }
}

MultiJointPlanner::MultiJointPlanner(){}

MultiJointPlanner::~MultiJointPlanner(){}

void MultiJointPlanner::init(
    std::vector<double> start_p, std::vector<double> start_v, std::vector<double> start_a,
    std::vector<double> target_p, std::vector<double> target_v, std::vector<double> target_a,
    const double tf, const double step)
{
    for (unsigned int j=0; j<ARM_DOF; j++)
    {
        sjp[j].init(start_p[j], start_v[j], start_a[j],
            target_p[j], target_v[j], target_a[j],
            tf, step);
    }

    length_ = sjp[0].length_;
    step_ = step;
}

void MultiJointPlanner::rt_plan(const double t, 
    std::vector<double> &p, std::vector<double> &v, std::vector<double> &a)
{
    for (unsigned int j=0; j<ARM_DOF; j++)
    {
        double p_;
        double v_;
        double a_;
        sjp[j].rt_plan(t, p_, v_, a_);

        p[j] = p_;
        v[j] = v_;
        a[j] = a_;
    }
}

void MultiJointPlanner::pre_plan(std::vector<std::vector<double>> &p, 
    std::vector<std::vector<double>> &v, std::vector<std::vector<double>> &a)
{
    double t = -step_;

    std::vector<double> p_;
    std::vector<double> v_;
    std::vector<double> a_;
    p_.resize(ARM_DOF);
    v_.resize(ARM_DOF);
    a_.resize(ARM_DOF);

    for (unsigned int j=0; j<length_; j++)
    {
        t += step_;

        MultiJointPlanner::rt_plan(t, p_, v_, a_);

        for (unsigned int i=0; i<ARM_DOF; i++)
        {
            p[j][i] = p_[i];
            v[j][i] = v_[i];
            a[j][i] = a_[i];
        }
    }
}

}
