#include <iostream>
#include <ros/ros.h>
#include <robot_hw_interface/robot_hw_interface.h>
#include <controller_manager/controller_manager.h>
#include <robot_info/robot_macro.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_hw_control");
   
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh;
    robot_hw_interface::RobotHwInterface hw;
    bool init_success = hw.init(ROBOT_ARM_JOINTS);

    // Read hw.arm_shm_->cur_positions_[j],
    // and publish them to /ROBOT/ARM/POSITION_CONTROLLER
    // This will set the values of real robot position to position command buffer area.

    // Publish zeros to /ROBOT/ARM/VELOCITY_CONTROLLER and /ROBOT/ARM/EFFORT_CONTROLLER
    // This will clear velocity and effort command buffer area.

    controller_manager::ControllerManager cm(&hw,nh);

    double update_rate = 1000; // 100Hz update rate
    ros::Rate rate(update_rate); 
    ROS_INFO("start robot hardware interface node...");
    ROS_INFO("update rate is %.3f Hz...", update_rate);

    while (ros::ok())
    {
        hw.read(ros::Time::now(), rate.expectedCycleTime());
        cm.update(ros::Time::now(), rate.expectedCycleTime());
        hw.write(ros::Time::now(), rate.expectedCycleTime());
        rate.sleep();
    }

    spinner.stop();
    return 0;
}
