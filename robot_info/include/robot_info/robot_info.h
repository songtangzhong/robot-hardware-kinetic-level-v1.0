#ifndef ROBOT_INFO_H_
#define ROBOT_INFO_H_

#include <robot_info/arm_info.h>
#include <robot_info/robot_macro.h>

namespace robot_info
{
class Robot
{
public:
    Robot(){};
    ~Robot(){};

    std::shared_ptr<arm_info::Arm> arm_ = 
        std::make_shared<arm_info::Arm>();

    const unsigned int position_mode_ = ROBOT_POSITION_MODE;
    const unsigned int velocity_mode_ = ROBOT_VELOCITY_MODE;
    const unsigned int effort_mode_ = ROBOT_EFFORT_MODE;

private:

};

}

#endif