/// @file    AE_RobotArmWP.h
/// @brief   Handles robotic arm way point storage, retrieval and lookup

/*
 * The AE_RobotArmWP library:
 *
 * Initial implementation: andychen, May 2023
 *
 * - responsible for managing a list of robotic arm way points
 * - reads and writes the robotic arm way points to storage
 * - provides access to the robotic arm way points
 *
 */
#pragma once

#include <AP_HAL/AP_HAL.h>

#ifndef HAL_ROBOTARMWP_ENABLED
#define HAL_ROBOTARMWP_ENABLED 1
#endif

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/vectorN.h>

struct PACKED RobotArmLocation {
    float xhorizontal;          //Horizontal position of cutting_head/bucket_tip in rear view (relatively in the range 0-1)
    float yvertical;          //Vertical position of cutting_head/bucket_tip in rear view (relatively in the range 0-1)
    float zalt;        //The altitude of the cutting_head/bucket_tip in meters (relatively);
    float flags;      //reserve

    void zero() {
        xhorizontal = 0;
        yvertical = 0;
        zalt = 0;
        flags = 0;
    }

    Vector2f xy() {
        return Vector2f(xhorizontal, yvertical);
    }

    void set_xy(Vector2f xy) {
        xhorizontal = xy.x;
        yvertical = xy.y;
    }

    RobotArmLocation& operator =(const VectorN<float, 4>& v) {
        xhorizontal = v[0];
        yvertical = v[1];
        zalt = v[2];

        return *this;
    }
};

struct PACKED RobotArmJointAngle {
    float theta_boom;               //The parameter theta2 in the standard DH coordinate method:The angle between the coordinate axis x1 and the coordinate axis x2
    float theta_forearm;            //The parameter theta3 in the standard DH coordinate method:The angle between the coordinate axis x2 and the coordinate axis x3
    float theta_bucket;             //The parameter theta4 in the standard DH coordinate method:The angle between the coordinate axis x3 and the coordinate axis x4
    float flags;                    //reserve

    void zero() {
        theta_boom = 0;
        theta_forearm = 0;
        theta_bucket = 0;
        flags = 0;
    }

    Vector3f jointAngle() {
        return Vector3f(theta_boom, theta_forearm, theta_bucket);
    }

    void set_jointAngle(Vector3f jointAngle) {
        theta_boom = jointAngle.x;
        theta_forearm = jointAngle.y;
        theta_bucket = jointAngle.z;
    }

    RobotArmJointAngle& operator =(const VectorN<float, 4>& v) {
        theta_boom = v[0];
        theta_forearm = v[1];
        theta_bucket = v[2];

        return *this;
    }
};

/// @class    AE_RobotArmWP
/// @brief    Object managing robotic arm way points
class AE_RobotArmWP
{
public:
    AE_RobotArmWP();

    /* Do not allow copies */
    AE_RobotArmWP(const AE_RobotArmWP &other) = delete;
    AE_RobotArmWP &operator=(const AE_RobotArmWP&) = delete;

    // data handling
    bool get_rbtarm_waypoint_with_index(uint8_t i, RobotArmLocation &ret) const;
    bool set_rbtarm_waypoint_with_index(uint8_t i, const RobotArmLocation &rbtArmLoc);
    uint8_t get_rbtarm_waypoint_total() const
    {
        return (uint8_t)_rbtarm_waypoint_total_count;
    }
    uint8_t get_rbtarm_waypoint_max(void) const
    {
        const uint16_t ret = _storage.size() / uint16_t(sizeof(RobotArmLocation));
        if (ret > 255) {
            return 255;
        }
        return (uint8_t)ret;
    }
    // reduce point count:
    void truncate(uint8_t num);
    // append a robotic arm way point to the list
    bool append(const RobotArmLocation &loc) WARN_IF_UNUSED;

    // last time robotic arm way points changed
    uint32_t last_change_time_ms(void) const
    {
        return _last_change_time_ms;
    }

    // parameter block
    static const struct AP_Param::GroupInfo var_info[];

    // get singleton instance
    static AE_RobotArmWP *get_singleton()
    {
        return _singleton;
    }

    // return bearing in radians from loc1 to loc2, return is 0 to 2*Pi
    static ftype get_bearing(const RobotArmLocation &loc1, const RobotArmLocation &loc2);

    // return bearing in centi-degrees from location to loc2, return is 0 to 35999
    static int32_t get_bearing_to(const RobotArmLocation &loc1, const RobotArmLocation &loc2) {
        return int32_t(get_bearing(loc1, loc2) * DEGX100 + 0.5);
    }
    // return distance in meters between two locations
    static ftype get_distance(const RobotArmLocation &loc1, const RobotArmLocation &loc2);

    // return difference between current and desired angles in radians
    static Vector3f get_diff_angle(const RobotArmJointAngle &angle1, const RobotArmJointAngle &angle2);
    
    // see if location is past a line perpendicular to
    // the line between point1 and point2 and passing through point2.
    // If point1 is our previous waypoint and point2 is our target waypoint
    // then this function returns true if we have flown past
    // the target waypoint
    static bool past_interval_finish_line(const RobotArmLocation &current_loc, const RobotArmLocation &point1, const RobotArmLocation &point2);

    /*
    return the proportion we are along the path from point1 to
    point2, along a line parallel to point1<->point2.

    This will be more than 1 if we have passed point2
    */
    static float line_path_proportion(const RobotArmLocation &current_loc, const RobotArmLocation &point1, const RobotArmLocation &point2);

    static Vector2f get_distance_NE(const RobotArmLocation &loc1, const RobotArmLocation &loc2);

private:
    static AE_RobotArmWP *_singleton;

    virtual bool is_valid(const RobotArmLocation &robot_arm_waypoint) const
    {
        return true;
    }

    static StorageAccess _storage;

    // parameters
    AP_Int8  _rbtarm_waypoint_total_count;


    uint32_t _last_change_time_ms = 0xFFFFFFFF;
};

namespace AE
{
AE_RobotArmWP *robotarmwp();
};
