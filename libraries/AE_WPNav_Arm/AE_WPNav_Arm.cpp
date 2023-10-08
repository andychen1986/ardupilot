#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include "AE_WPNav_Arm.h"

extern const AP_HAL::HAL& hal;

int count_pid = 0;
int count_angle = 0;

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
AE_WPNav_Arm::AE_WPNav_Arm(AE_PosControl_Arm& pos_controller, AE_Navigation_Arm& nav_controller, AE_AngleControl& angle_controller):
    _pos_controller(pos_controller),
    _nav_controller(nav_controller),
    _angle_controller(angle_controller),
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

void AE_WPNav_Arm::update_excavator(float dt)
{
    RobotArmJointAngle current_angle;

    if (!hal.util->get_soft_armed() || !_orig_and_dest_valid  
        || !get_jointAngle_excavator(current_angle)) {
        _desired_vel_excavator = Vector3f(0, 0, 0);
        return;
    }

    _last_update_ms = AP_HAL::millis();

    //PID控制
    Vector3f targetAngle(_destination_Angle.theta_boom, _destination_Angle.theta_forearm, _destination_Angle.theta_bucket);
    _angle_controller.update_angle_controller(targetAngle, dt);
    _desired_vel_excavator = _angle_controller.get_desired_vel();

    count_pid++;
    if(count_pid > 2)
    {
        gcs().send_text(MAV_SEVERITY_INFO, "pid output boom:%f, forearm:%f, bucket:%f\n", _desired_vel_excavator.x, _desired_vel_excavator.y, _desired_vel_excavator.z);
        count_pid = 0;
    }

    _angle_diff = AE_RobotArmWP::get_diff_angle(_destination_Angle, current_angle);

    if(angle_diff_check())
    {
        _reached_destination = true;
    }
    
}

bool AE_WPNav_Arm::angle_diff_check()
{
    bool ret = false;
    
    //角度误差在1度以内，视作到达目标点 1deg = 0.01745rad
    if((_angle_diff.x > -0.06 && _angle_diff.x < 0.06) 
        && (_angle_diff.y > -0.2 && _angle_diff.y < 0.2)
        && (_angle_diff.z > -0.2 && _angle_diff.z < 0.2))
    {
        ret = true;
    }

    gcs().send_text(MAV_SEVERITY_INFO, "angele diff boom:%f, forearm:%f, bucket:%f", _angle_diff.x, _angle_diff.y, _angle_diff.z);

    return ret;
}

bool AE_WPNav_Arm::get_jointAngle_excavator(RobotArmJointAngle& jointAngle)
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

    AE_RobotArmInfo_Excavator *arminfo_backend = (AE_RobotArmInfo_Excavator*)_armInfo_backend;

    AE_RobotArmInfo_Excavator::Excavator_DH_Anger dh_Angles = arminfo_backend->get_DH_Angles();

    jointAngle.theta_boom = dh_Angles.boom_to_slewing;
    jointAngle.theta_forearm = dh_Angles.forearm_to_boom;
    jointAngle.theta_bucket = dh_Angles.bucket_to_forearm;
    
    count_angle++;
    if(count_angle > 2)
    {
        gcs().send_text(MAV_SEVERITY_INFO, "Current jointAngle: boom: %f, forearm: %f, bucket: %f", jointAngle.theta_boom, jointAngle.theta_forearm, jointAngle.theta_bucket);
        count_angle = 0;
    }
    

    return true;
}

bool AE_WPNav_Arm::kinematic_inverse(const RobotArmLocation loc, RobotArmJointAngle& desired_angle, float l_bo, float l_f, float l_bu)
{
    float k1,k2;
    k1 = loc.xhorizontal - l_bu * cosf(azimuth[index]);
    k2 = loc.yvertical - l_bu * sinf(azimuth[index]);

    //cos(theta2) = i
    float i = (powf(k1, 2) + powf(k2, 2) - powf(l_bo, 2) - powf(l_f, 2)) / (2 * l_f * l_bo);
    if(i > 1 || i < -1)
    {
        //无解，达不到该区域
        gcs().send_text(MAV_SEVERITY_INFO, "No solution, cannot reach this position");
        index++;
        return false;
    }
    //sin(theta2) = j
    float j = -sqrtf(1 - powf(i, 2));

    desired_angle.theta_forearm = atan2f(j, i); 
    
    float m = l_f * i + l_bo;
    float n = l_f * j;

    desired_angle.theta_boom = atan2f(k2, k1) - atan2f(n, m);

    //让角度在-180 ~ 180 之间
    if(desired_angle.theta_boom > M_PI){
        desired_angle.theta_boom -= 2 * M_PI;
        desired_angle.theta_bucket = azimuth[index] - desired_angle.theta_forearm - desired_angle.theta_boom + 2 * M_PI;
        //调整theta_bucket角度
        if(desired_angle.theta_bucket > 2 * M_PI){
            while((desired_angle.theta_bucket -= 2 * M_PI) > 2 * M_PI);    //先将其调整为0-360
            
            if(desired_angle.theta_bucket > M_PI){
                desired_angle.theta_bucket -= 2 * M_PI;
            }
        }  
        else if(desired_angle.theta_bucket < 2 * -M_PI){
            while((desired_angle.theta_bucket += 2 * M_PI) < 2 * -M_PI);    //先将其调整为0-360

            if(desired_angle.theta_bucket < -M_PI){
                desired_angle.theta_bucket += 2 * M_PI;
            }
        }  
    } 
    else if(desired_angle.theta_boom < -M_PI){
        desired_angle.theta_boom += 2 * M_PI;
        desired_angle.theta_bucket = azimuth[index] - desired_angle.theta_forearm - desired_angle.theta_boom - 2 * M_PI;
        //调整theta_bucket角度
        if(desired_angle.theta_bucket > 2 * M_PI){
            while((desired_angle.theta_bucket -= 2 * M_PI) > 2 * M_PI);    //先将其调整为0-360
            
            if(desired_angle.theta_bucket > M_PI){
                desired_angle.theta_bucket -= 2 * M_PI;
            }
        }  
        else if(desired_angle.theta_bucket < 2 * -M_PI){
            while((desired_angle.theta_bucket += 2 * M_PI) < 2 * -M_PI);    //先将其调整为0-360

            if(desired_angle.theta_bucket < -M_PI){
                desired_angle.theta_bucket += 2 * M_PI;
            }
        } 
    }
    else
    {
        desired_angle.theta_bucket = azimuth[index] - desired_angle.theta_forearm - desired_angle.theta_boom;
        //调整theta_bucket角度
        if(desired_angle.theta_bucket > 2 * M_PI){
            while((desired_angle.theta_bucket -= 2 * M_PI) > 2 * M_PI);    //先将其调整为0-360
            
            if(desired_angle.theta_bucket > M_PI){
                desired_angle.theta_bucket -= 2 * M_PI;
            }
        }  
        else if(desired_angle.theta_bucket < 2 * -M_PI){
            while((desired_angle.theta_bucket += 2 * M_PI) < 2 * -M_PI);    //先将其调整为0-360

            if(desired_angle.theta_bucket < -M_PI){
                desired_angle.theta_bucket += 2 * M_PI;
            }
        } 
    }
    index++;
    return true;
}

bool AE_WPNav_Arm::set_desired_jointAngle(const RobotArmJointAngle& dest_jointAngle)
{
    _destination_Angle = dest_jointAngle;
    gcs().send_text(MAV_SEVERITY_INFO, "num--:%d,_destination_Angle theta1:%f, theta2:%f, theta3:%f\n", \
                index, _destination_Angle.theta_boom, _destination_Angle.theta_forearm, _destination_Angle.theta_bucket);

    _orig_and_dest_valid = true;
    _reached_destination = false;
    if(index >= 5)
    {
        index = 0;
    }
    return true;
}   