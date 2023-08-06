#include "AE_PosControl_Arm.h"
#include <AP_HAL/AP_HAL.h>

 # define POSCONTROL_POS_XY_P                  0.8f    // position controller P gain default

 # define POSCONTROL_VEL_X_P                   0.06f    // horizontal velocity controller P gain default
 # define POSCONTROL_VEL_X_I                   0.3f    // horizontal velocity controller I gain default
 # define POSCONTROL_VEL_X_D                   0.0f    // horizontal velocity controller D gain default
 # define POSCONTROL_VEL_X_IMAX                28.0f    // horizontal velocity controller IMAX gain default
 # define POSCONTROL_VEL_X_FILT_T_HZ           5.0f    // horizontal velocity controller PID target filter frequency in Hz
 # define POSCONTROL_VEL_X_FILT_E_HZ           5.0f    // horizontal velocity controller PID error filter frequency in Hz
 # define POSCONTROL_VEL_X_FILT_D_HZ           5.0f    // horizontal velocity controller PID derivative filter frequency in Hz

 # define POSCONTROL_VEL_Y_P                   1.5f    // vertical velocity controller P gain default
 # define POSCONTROL_VEL_Y_I                   1.5f    // vertical velocity controller I gain default
 # define POSCONTROL_VEL_Y_D                   0.0f    // vertical velocity controller D gain default
 # define POSCONTROL_VEL_Y_IMAX                40.0f    // vertical velocity controller IMAX gain default
 # define POSCONTROL_VEL_Y_FILT_T_HZ           5.0f    // vertical velocity controller PID target filter frequency in Hz
 # define POSCONTROL_VEL_Y_FILT_E_HZ           5.0f    // vertical velocity controller PID error filter frequency in Hz
 # define POSCONTROL_VEL_Y_FILT_D_HZ           5.0f    // vertical velocity controller PID derivative filter frequency in Hz


const AP_Param::GroupInfo AE_PosControl_Arm::var_info[] = {
    
    // @Param: _POSXY_P
    // @DisplayName: Position (horizontal) controller P gain
    // @Description: Position controller P gain.  Converts the distance (in the latitude direction) to the target location into a desired speed which is then passed to the loiter latitude rate controller
    // @Range: 0.500 2.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_pos_xy, "_POSXY_", 1, AE_PosControl_Arm, AC_P_2D),

    // @Param: _VELY_
    // @DisplayName: Velocity (horizontal) feed forward gain
    // @Description: Velocity (horizontal) feed forward gain.  Converts the difference between desired velocity to a target acceleration
    // @Range: 0 6
    // @Increment: 0.01
    // @User: Advanced
    AP_SUBGROUPINFO(_pid_vel_x, "_VELX_", 2, AE_PosControl_Arm, AC_PID),

    // @Param: _VELY_
    // @DisplayName: Velocity (vertical) feed forward gain
    // @Description: Velocity (vertical) feed forward gain.  Converts the difference between desired velocity to a target acceleration
    // @Range: 0 6
    // @Increment: 0.01
    // @User: Advanced
    AP_SUBGROUPINFO(_pid_vel_y, "_VELY_", 3, AE_PosControl_Arm, AC_PID),

    AP_GROUPEND
};

AE_PosControl_Arm::AE_PosControl_Arm(float dt) :
    _p_pos_xy(POSCONTROL_POS_XY_P, dt),
    _pid_vel_x(POSCONTROL_VEL_X_P, POSCONTROL_VEL_X_I, POSCONTROL_VEL_X_D, 0.0f, POSCONTROL_VEL_X_IMAX, \
               POSCONTROL_VEL_X_FILT_T_HZ, POSCONTROL_VEL_X_FILT_E_HZ, POSCONTROL_VEL_X_FILT_D_HZ, dt),
    _pid_vel_y(POSCONTROL_VEL_Y_P, POSCONTROL_VEL_Y_I, POSCONTROL_VEL_Y_D, 0.0f, POSCONTROL_VEL_Y_IMAX, \
               POSCONTROL_VEL_Y_FILT_T_HZ, POSCONTROL_VEL_Y_FILT_E_HZ, POSCONTROL_VEL_Y_FILT_D_HZ, dt),
    _dt(dt),
    _armInfo(nullptr),
    _armInfo_backend(nullptr)
{
    _pid_vel_x.reset_filter();
    _pid_vel_x.reset_I();
    _pid_vel_y.reset_filter();
    _pid_vel_y.reset_I();

    AP_Param::setup_object_defaults(this, var_info);
}

// is_active_xy - returns true if the xy position controller has been run in the previous 5 loop times
bool AE_PosControl_Arm::is_active_xy() const
{
    return ((AP_HAL::micros64() - _last_update_xy_us) <= _dt * 5000000.0);
}

void AE_PosControl_Arm::update_xy_controller(Vector2p& targetPos, float speed_min, float speed_max, float dt, bool insertF)
{
    Vector2f curr_pos;
    if(!get_position_xy_mm(curr_pos)) {
        _vel_target.zero();
        return;
    }

    Vector2f curr_vel;
    if(!get_vel_xy_mmpers(curr_vel)) {
        _accel_target.zero();
        return;
    }

    // Check for control time out
    if (!is_active_xy()) {
        _pid_vel_x.reset_filter();
        _pid_vel_x.reset_I();
        _pid_vel_y.reset_filter();
        _pid_vel_y.reset_I();
        _vel_target = curr_vel;
    }
    _last_update_xy_us = AP_HAL::micros64();
    _dt = dt;

    // Position Controller
    _vel_target = _p_pos_xy.update_all(targetPos.x, targetPos.y, curr_pos);

    float speed = _vel_target.length();
    if(speed > speed_max) {
        _vel_target.normalize();
        _vel_target *= speed_max;
    }

    if(insertF && (speed < speed_min)) {
        _vel_target.normalize();
        _vel_target *= speed_min;
    }

    // Velocity Controller
    _pid_vel_x.set_dt(_dt);
    _pid_vel_y.set_dt(_dt);
    float target_x = _pid_vel_x.update_all(_vel_target.x, curr_vel.x, false);
    float target_y = _pid_vel_y.update_all(_vel_target.y, curr_vel.y, false);

    _accel_target.x = target_x;
    _accel_target.y = target_y;

    // debug_luo
    static int count = 0;
    count++;
    if(count >= 60) {
        count = 0;

        gcs().send_text(MAV_SEVERITY_INFO,"_vel_target x:%f y:%f", _vel_target.x, _vel_target.y);
    }
}

bool AE_PosControl_Arm::get_position_xy_mm(Vector2f& pos)
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

    AE_RobotArmInfo_TBM::TBM_Cutting_Header_State state_tbm = arminfo_backend->get_TBM_cutting_header_state();

    pos.x = state_tbm.cutheader_horizon_pos;
    pos.y = state_tbm.cutheader_height;

    return true;
}

bool AE_PosControl_Arm::get_vel_xy_mmpers(Vector2f& vel)
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

    AE_RobotArmInfo_TBM::TBM_Cutting_Header_State state_tbm = arminfo_backend->get_TBM_cutting_header_state();

    vel.x = state_tbm.cutheader_horizon_vel;
    vel.y = state_tbm.cutheader_vertical_vel;

    return true;
}
