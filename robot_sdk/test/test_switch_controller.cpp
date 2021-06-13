#include <robot_sdk/robot_sdk.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        ROS_INFO("usage: rosrun ros_sdk test_switch_controller start_controller\nreplace start_controller with the controller you want to start.");
        return 0;
    }

    ros::init(argc, argv, "test_switch_controller");

    std::shared_ptr<robot_sdk::Robot> robot = 
        std::make_shared<robot_sdk::Robot>();

    robot->switch_controller(argv[1]);

    ros::shutdown();
    return 0;
}
