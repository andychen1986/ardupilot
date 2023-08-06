#include "AE_Navigation_Arm.h"

bool AE_Navigation_Arm::set_origin_dest(RobotArmLocation& origin, RobotArmLocation& dest)
{
    bool ret = false;

    _origin = origin;
    _destination = dest;
    _insertion_point = origin;

    ret = advance_next_point(origin, dest);

    return ret;
}