#pragma once

#include <AP_Common/AP_Common.h>
#include <AC_PID/AC_P_1D.h>         // P library (1-axis)
#include <AC_PID/AC_PID.h>
#include <AE_RobotArmInfo/AE_RobotArmInfo.h>
#include <AE_RobotArmInfo/AE_RobotArmInfo_Backend.h>
#include <AE_RobotArmInfo/AE_RobotArmInfo_Excavator.h>

class AE_AngleControl
{
public:
    AE_AngleControl(float dt);

    // is_active_xy - returns true if the xy position controller has been run in the previous 5 loop times
    bool is_active_xy() const;

    //更新角度控制器输出
    void update_angle_controller(Vector3f& targetAngle, float dt);

    Vector3f pi_control(Vector3f target, Vector3f messurement, float dt);          //PI控制器输出

    Vector3f get_desired_vel() const { return _vel_target; };
    Vector3f get_desired_accel() const { return _accel_target; };

    static const struct AP_Param::GroupInfo var_info[];

private:
    float _kp_boom = 800;   // 比例增益
    float _kp_forearm = 300;
    float _kp_bucket = 300;  
    float _ki_boom = 0.5;  // 积分增益
    float _ki_forearm = 0.01;
    float _ki_bucket = 0.01;
    Vector3f _integral;     // 积分项，用于消除静态误差
    
    bool getCurrentAngle(Vector3f& currentAngle);       //获取当前角度

    uint64_t    _last_update_xy_us;     // system time (in microseconds) since last update_xy_controller call
    float       _dt;                    // time difference (in seconds) between calls from the main program

    AC_P_1D     _p_angle_ctl;          

    Vector3f    _vel_target;
    Vector3f    _accel_target;

    AE_RobotArmInfo *_armInfo;
    AE_RobotArmInfo_Backend* _armInfo_backend;
};