#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AE_Navigation_Arm/AE_Navigation_Arm.h>
#include <AE_RobotArmWP/AE_RobotArmWP.h>
#include <AE_Control_Arm/AE_PosControl_Arm.h>
#include <AE_RobotArmInfo/AE_RobotArmInfo.h>
#include <AE_RobotArmInfo/AE_RobotArmInfo_Backend.h>
#include <AE_RobotArmInfo/AE_RobotArmInfo_TBM.h>

const float AE_WPNAV_ARM_HEADING_UNKNOWN = 99999.0f; // used to indicate to set_desired_location method that next leg's heading is unknown

class AE_WPNav_Arm
{
public:
    // constructor
    AE_WPNav_Arm(AE_PosControl_Arm& pos_controller, AE_Navigation_Arm& nav_controller);

    // return desired velocity
    Vector2f get_desired_vel() const { return _desired_vel; }

    // return desired acceleration
    Vector2f get_desired_accel() const { return _desired_accel; }

    // set desired velocity in m/s
    void set_desired_vel(Vector2f speed) { _desired_vel = speed.length() > 0 ? speed : Vector2f(0, 0); }

    // restore desired velocity to default from parameter value
    void set_desired_vel_to_default() { _desired_vel = Vector2f(_speed_min, _speed_min); }

    // set desired location
    // next_leg_bearing_cd should be heading to the following waypoint (used to slow the vehicle in order to make the turn)
    bool set_desired_location(const RobotArmLocation& destination, float next_leg_bearing_cd = AE_WPNAV_ARM_HEADING_UNKNOWN);

    // true if vehicle has reached desired location. defaults to true because this is normally used by missions and we do not want the mission to become stuck
    bool reached_destination() const { return _reached_destination; }

    // return distance (in meters) to destination
    float get_distance_to_destination() const { return _distance_to_destination; }

    // return true if destination is valid
    bool is_destination_valid() const { return _orig_and_dest_valid; }

    // get current destination. Note: this is not guaranteed to be valid (i.e. _orig_and_dest_valid is not checked)
    const RobotArmLocation &get_destination() { return _destination; }

    bool is_active() const;

    // update navigation
    void update(float dt);

    // accessors for parameter values
    float get_max_speed() const { return _speed_max; }
    float get_radius() const { return _radius; }

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

private:
    bool get_position_xy_mm(RobotArmLocation& loc);

    // parameters
    AP_Float _speed_max;            // target speed between waypoints in m/s
    AP_Float _speed_min;            // target speed minimum in m/s.  Vehicle will not slow below this speed for corners
    AP_Float _radius;               // distance in meters from a waypoint when we consider the waypoint has been reached

    // references
    AE_PosControl_Arm& _pos_controller;
    AE_Navigation_Arm& _nav_controller;
    
    // variables for navigation
    uint32_t _last_update_ms;       // system time of last call to update
    RobotArmLocation _origin;               // origin Location (vehicle will travel from the origin to the destination)
    RobotArmLocation _destination;          // destination Location when in Guided_WP
    bool _orig_and_dest_valid;      // true if the origin and destination have been set

    // main outputs from navigation library
    Vector2f _desired_vel;           // desired speed in mm/s
    Vector2f _desired_accel;           // desired speed in mm/s^2

    // variables for reporting
    float _distance_to_destination; // distance from vehicle to final destination in meters
    bool _reached_destination;      // true once the vehicle has reached the destination

    bool _insertF;

    AE_RobotArmInfo *_armInfo;
    AE_RobotArmInfo_Backend* _armInfo_backend;
};
