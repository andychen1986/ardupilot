#pragma once

#include <AP_Common/AP_Common.h>
#include <AE_RobotArmWP/AE_RobotArmWP.h>

class AE_Navigation_Arm {
public:
    bool set_origin_dest(RobotArmLocation& origin, RobotArmLocation& dest);

    // find next _insertion_point 
    // return: false: has reached the _destination 
    virtual bool advance_next_point(RobotArmLocation& origin, RobotArmLocation& dest) = 0;

protected:
    RobotArmLocation _origin;
    RobotArmLocation _destination;
    RobotArmLocation _insertion_point;
};