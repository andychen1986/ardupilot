#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include "AE_WPNav_Arm.h"

extern const AP_HAL::HAL& hal;

#define AE_WPNAV_TIMEOUT_MS             100
#define AE_WPNAV_SPEED_MAX              80.0f
#define AE_WPNAV_SPEED_MIN              40.0f
#define AE_WPNAV_RADIUS_DEFAULT         20.0f

const AP_Param::GroupInfo AE_WPNav_Arm::var_info[] = {

    // @Param: SPEED
    // @DisplayName: Waypoint speed default
    // @Description: Waypoint speed default
    // @Units: mm/s
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SPEED", 1, AE_WPNav_Arm, _speed_max, AE_WPNAV_SPEED_MAX),

    // @Param: SPEED_MIN
    // @DisplayName: Waypoint speed minimum
    // @Description: arm will not slow below this speed for corners.
    // @Units: mm/s
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SPEED_MIN", 3, AE_WPNav_Arm, _speed_min, AE_WPNAV_SPEED_MIN),

    // @Param: RADIUS
    // @DisplayName: Waypoint radius
    // @Description: The distance in meters from a waypoint when we consider the waypoint has been reached. This determines when the arm will turn toward the next waypoint.
    // @Units: mm
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RADIUS", 2, AE_WPNav_Arm, _radius, AE_WPNAV_RADIUS_DEFAULT),

    AP_GROUPEND
};

// constructor
AE_WPNav_Arm::AE_WPNav_Arm(AE_PosControl_Arm& pos_controller, AE_Navigation_Arm& nav_controller):
    _pos_controller(pos_controller),
    _nav_controller(nav_controller),
    _insertF(false),
    _armInfo(nullptr),
    _armInfo_backend(nullptr)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// set desired location
bool AE_WPNav_Arm::set_desired_location(const RobotArmLocation& destination, float next_leg_bearing_cd)
{
    // set origin to last destination if waypoint controller active
    if (is_active() && _orig_and_dest_valid && _reached_destination) {
        _origin = _destination;       
    } else {
        RobotArmLocation current_loc;
        if (!get_position_xy_mm(current_loc)) {
            return false;
        }

        _origin = current_loc;
    }

    // initialise some variables
    _destination = destination;
    _insertF = _nav_controller.set_origin_dest(_origin, _destination);

    _distance_to_destination = AE_RobotArmWP::get_distance(_origin, _destination);
    _orig_and_dest_valid = true;
    _reached_destination = false;

    return true;
}

// true if update has been called recently
bool AE_WPNav_Arm::is_active() const
{
    return ((AP_HAL::millis() - _last_update_ms) < AE_WPNAV_TIMEOUT_MS);
}

// update navigation
void AE_WPNav_Arm::update(float dt)
{
    // exit immediately if no current location, origin or destination
    RobotArmLocation current_loc;

    if (!hal.util->get_soft_armed() || !_orig_and_dest_valid  \
        || !get_position_xy_mm(current_loc)) {
        _desired_vel = Vector2f(0, 0);
        return;
    }

    _last_update_ms = AP_HAL::millis();

    _distance_to_destination = AE_RobotArmWP::get_distance(current_loc, _destination);

    // check if vehicle has reached the destination
    const bool near_wp = _distance_to_destination <= _radius;
    const bool past_wp = AE_RobotArmWP::past_interval_finish_line(current_loc, _origin, _destination);
    
    if (!_reached_destination && (near_wp || past_wp)) {
        if(!_insertF) {
            _reached_destination = true;
        }
        else {
            gcs().send_text(MAV_SEVERITY_INFO, "reach insert x:%f, y:%f, z:%f\n", \
                            _destination.xhorizontal, _destination.yvertical, _destination.zalt);
            
            _insertF = _nav_controller.advance_next_point(_origin, _destination);
            _distance_to_destination = AE_RobotArmWP::get_distance(current_loc, _destination);
        }
    }

    Vector2p targetPos(_destination.xhorizontal, _destination.yvertical);
    _pos_controller.update_xy_controller(targetPos, _speed_min, _speed_max, dt, _insertF);
    _desired_vel = _pos_controller.get_desired_vel();
    _desired_accel = _pos_controller.get_desired_accel();
}

bool AE_WPNav_Arm::get_position_xy_mm(RobotArmLocation& loc)
{
    if(_armInfo == nullptr) {
        _armInfo = AE_RobotArmInfo::get_singleton();
        
        if(_armInfo == nullptr) {
            return false;
        }
    }
    
    if (_armInfo_backend == nullptr) {
        _armInfo_backend = _armInfo->backend();

        if(_armInfo_backend == nullptr) {
            return false;
        }
    }

    AE_RobotArmInfo_TBM *arminfo_backend = (AE_RobotArmInfo_TBM*)_armInfo_backend;

    if(!arminfo_backend->get_healthy()) {
        return false;
    }

    AE_RobotArmInfo_TBM::TBM_Cutting_Header_State state_tbm = arminfo_backend->get_TBM_cutting_header_state();

    loc.xhorizontal = state_tbm.cutheader_horizon_pos;
    loc.yvertical = state_tbm.cutheader_height;

    return true;
}
