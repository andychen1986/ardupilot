#pragma once

#include <AP_Common/AP_Common.h>
#include <AC_PID/AC_P_2D.h>         // P library (2-axis)
#include <AC_PID/AC_PID.h>
#include <AE_RobotArmInfo/AE_RobotArmInfo.h>
#include <AE_RobotArmInfo/AE_RobotArmInfo_Backend.h>
#include <AE_RobotArmInfo/AE_RobotArmInfo_TBM.h>

class AE_PosControl_Arm
{
public:
    AE_PosControl_Arm(float dt);

    // is_active_xy - returns true if the xy position controller has been run in the previous 5 loop times
    bool is_active_xy() const;
    void update_xy_controller(Vector2p& targetPos, float speed_min,  float speed_max, float dt, bool insertF = false);

    Vector2f get_desired_vel() const { return _vel_target; };
    Vector2f get_desired_accel() const { return _accel_target; };

    static const struct AP_Param::GroupInfo var_info[];

private:
    bool get_position_xy_mm(Vector2f& pos);
    bool get_vel_xy_mmpers(Vector2f& vel);

    uint64_t    _last_update_xy_us;     // system time (in microseconds) since last update_xy_controller call
    float       _dt;                    // time difference (in seconds) between calls from the main program

    AC_P_2D     _p_pos_xy;          // XY axis position controller to convert distance error to desired velocity
    AC_PID      _pid_vel_x;         // X axis velocity controller to convert velocity error to desired acceleration
    AC_PID      _pid_vel_y;         // Y axis velocity controller to convert velocity error to desired acceleration

    Vector2f    _vel_target;
    Vector2f    _accel_target;

    AE_RobotArmInfo *_armInfo;
    AE_RobotArmInfo_Backend* _armInfo_backend;
};