/// @file    AE_RobotArmWP.h
/// @brief   Handles robotic arm way point storage, retrieval and lookup.
#include "AE_RobotArmWP.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>
#include <StorageManager/StorageManager.h>

#if HAL_ROBOTARMWP_ENABLED
// storage object
StorageAccess AE_RobotArmWP::_storage(StorageManager::StorageRobotArmWP);

assert_storage_size<RobotArmLocation, 16> _assert_storage_size_RobotArmLocation;

const AP_Param::GroupInfo AE_RobotArmWP::var_info[] = {
    // @Param: TOTAL
    // @DisplayName: Robotic Arm Way Points Total
    // @Description: Number of robotic arm way points currently loaded
    // @User: Advanced
    AP_GROUPINFO("TOTAL", 0, AE_RobotArmWP, _rbtarm_waypoint_total_count, 0),

    AP_GROUPEND
};

// constructor
AE_RobotArmWP::AE_RobotArmWP()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("robotic arm way points must be singleton");
    }
#endif
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

// get a robotic arm way point from EEPROM
bool AE_RobotArmWP::get_rbtarm_waypoint_with_index(uint8_t i, RobotArmLocation &ret) const
{
    if (i >= (uint8_t) _rbtarm_waypoint_total_count) {
        return false;
    }

    _storage.read_block(&ret, i * sizeof(RobotArmLocation), sizeof(RobotArmLocation));

    if (abs(ret.xhorizontal) > 1 && abs(ret.yvertical) > 1) {
        return false; // sanity check,for the range of RobotArmLocation is 0-100
    }

    return true;
}

void AE_RobotArmWP::truncate(uint8_t num)
{
    if (num > _rbtarm_waypoint_total_count) {
        // we never make the space larger this way
        return;
    }
    _rbtarm_waypoint_total_count.set_and_save_ifchanged(num);
}

bool AE_RobotArmWP::append(const RobotArmLocation &loc)
{
    const uint8_t current_total = get_rbtarm_waypoint_total();
    _rbtarm_waypoint_total_count.set_and_save_ifchanged(current_total + 1);
    if (!set_rbtarm_waypoint_with_index(current_total, loc)) {
        _rbtarm_waypoint_total_count.set_and_save_ifchanged(current_total);
        return false;
    }
    return true;
}

// save a robotic arm way point to EEPROM - this assumes that the AE_RobotArmWP_TOTAL param has been incremented beforehand, which is the case in Mission Planner
bool AE_RobotArmWP::set_rbtarm_waypoint_with_index(uint8_t i, const RobotArmLocation &rbtArmLoc)
{
    if (i >= (uint8_t) _rbtarm_waypoint_total_count) {
        return false;
    }

    if (i >= get_rbtarm_waypoint_max()) {
        return false;
    }

    _storage.write_block(i * sizeof(RobotArmLocation), &rbtArmLoc, sizeof(RobotArmLocation));

    _last_change_time_ms = AP_HAL::millis();

    AP::logger().Write_RobotArmWayPoint(_rbtarm_waypoint_total_count, i, rbtArmLoc);

    return true;
}

// return bearing in radians from loc1 to loc2, return is 0 to 2*Pi
ftype AE_RobotArmWP::get_bearing(const RobotArmLocation &loc1, const RobotArmLocation &loc2)
{
    const int32_t off_x = loc2.xhorizontal - loc1.xhorizontal;
    const int32_t off_y = loc2.yvertical - loc1.yvertical;
    ftype bearing = (M_PI*0.5) + atan2F(-off_y, off_x);
    if (bearing < 0) {
        bearing += 2*M_PI;
    }
    return bearing;
}

ftype AE_RobotArmWP::get_distance(const RobotArmLocation &loc1, const RobotArmLocation &loc2)
{
    ftype dx = (ftype)(loc2.xhorizontal - loc1.xhorizontal);
    ftype dy = (ftype)(loc2.yvertical - loc1.yvertical);
    return norm(dx, dy);
}

Vector3f AE_RobotArmWP::get_diff_angle(const RobotArmJointAngle &angle1, const RobotArmJointAngle &angle2)
{
    Vector3f angle_diff;
    angle_diff.x = (ftype)(angle1.theta_boom - angle2.theta_boom);
    angle_diff.y = (ftype)(angle1.theta_forearm - angle2.theta_forearm);
    angle_diff.z = (ftype)(angle1.theta_bucket - angle2.theta_bucket);
    return angle_diff;
}

// see if location is past a line perpendicular to
// the line between point1 and point2 and passing through point2.
// If point1 is our previous waypoint and point2 is our target waypoint
// then this function returns true if we have flown past
// the target waypoint
bool AE_RobotArmWP::past_interval_finish_line(const RobotArmLocation &current_loc, const RobotArmLocation &point1, const RobotArmLocation &point2)
{
    return line_path_proportion(current_loc, point1, point2) >= 1.0f;
}

/*
  return the proportion we are along the path from point1 to
  point2, along a line parallel to point1<->point2.

  This will be more than 1 if we have passed point2
 */
float AE_RobotArmWP::line_path_proportion(const RobotArmLocation &current_loc, const RobotArmLocation &point1, const RobotArmLocation &point2)
{
    const Vector2f vec1 = AE_RobotArmWP::get_distance_NE(point1, point2);
    const Vector2f vec2 = AE_RobotArmWP::get_distance_NE(point1, current_loc);
    const ftype dsquared = sq(vec1.x) + sq(vec1.y);
    if (dsquared < 0.001f) {
        // the two points are very close together
        return 1.0f;
    }
    return (vec1 * vec2) / dsquared;
}

Vector2f AE_RobotArmWP::get_distance_NE(const RobotArmLocation &loc1, const RobotArmLocation &loc2)
{
    return Vector2f(loc2.yvertical - loc1.yvertical,
                    loc2.xhorizontal - loc1.xhorizontal);
}

// singleton instance
AE_RobotArmWP *AE_RobotArmWP::_singleton;

namespace AE
{

AE_RobotArmWP *robotarmwp()
{
    return AE_RobotArmWP::get_singleton();
}

}
#endif //HAL_ROBOTARMWP_ENABLED
