#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include "AE_Navigation_Arm.h"

class AE_Linear_Nav_Arm :public AE_Navigation_Arm
{
public:
    // find next _insertion_point 
    // return: false: has reached the _destination 
    bool advance_next_point(RobotArmLocation& origin, RobotArmLocation& dest) override;

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

private:
    // parameters
    AP_Float _insert_distance;    // The minimum insert distance between two points
};