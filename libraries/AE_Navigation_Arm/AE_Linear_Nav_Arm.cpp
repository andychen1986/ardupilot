#include "AE_Linear_Nav_Arm.h"

#define AE_Nav_Insert_Distance 50

const AP_Param::GroupInfo AE_Linear_Nav_Arm::var_info[] = {

    // @Param: InsertDist
    // @DisplayName: Insert Distance
    // @Description: The minimum insert distance between two waypoints
    // @Units: mm
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("InsertDist", 1, AE_Linear_Nav_Arm, _insert_distance, AE_Nav_Insert_Distance),

    AP_GROUPEND
};

// find next _insertion_point 
// return: false: has reached the _destination 
bool AE_Linear_Nav_Arm::advance_next_point(RobotArmLocation& origin, RobotArmLocation& dest)
{
    Vector2f od = _destination.xy() - _insertion_point.xy();

    origin = _insertion_point;

    float length = od.length();

    if(length >= _insert_distance * 2)
    {
        od.normalize();
        _insertion_point.set_xy(_insertion_point.xy() + od * _insert_distance.get());
        
        dest = _insertion_point;

        return true;
    }
    else
    {
        dest = _insertion_point = _destination;

        return false;
    }
}
