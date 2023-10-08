#include "AE_AngleControl.h"
#include <AP_HAL/AP_HAL.h>

# define POSCONTROL_POS_XY_P                  0.8f    // position controller P gain default



const AP_Param::GroupInfo AE_AngleControl::var_info[] = {

    AP_GROUPEND
};

AE_AngleControl::AE_AngleControl(float dt): 
            _p_angle_ctl(POSCONTROL_POS_XY_P, dt),
            _dt(dt),
            _armInfo(nullptr),
            _armInfo_backend(nullptr)
{
    _integral.zero();
    _vel_target.zero();
}

// is_active_xy - returns true if the xy position controller has been run in the previous 5 loop times
bool AE_AngleControl::is_active_xy() const
{
    return ((AP_HAL::micros64() - _last_update_xy_us) <= _dt * 5000000.0);
}

void AE_AngleControl::update_angle_controller(Vector3f& targetAngle, float dt)
{
    Vector3f currentAngle;
    if(!getCurrentAngle(currentAngle))
    {
        _vel_target.zero();
        return;
    }

    if(!is_active_xy())
    {
        
    }
    _last_update_xy_us = AP_HAL::micros64();
    _dt = dt;

    // if(currentAngle.x > 0 && currentAngle.x < M_PI_2){                              //初始角在第一象限
    //     if(targetAngle.x < - M_PI_2 && targetAngle.x > -M_PI)                       //终止角在第三象限
    //         targetAngle.x += 2 * M_PI;
    // }
    // else if(currentAngle.x > M_PI_2 && currentAngle.x < M_PI){                      //初始角在第二象限
    //     if(targetAngle.x < -M_PI_2 && targetAngle.x > -M_PI){                       //终止角在第三象限
    //         targetAngle.x += 2 * M_PI;
    //     }       
    //     else if(targetAngle.x < 0 && targetAngle.x > -M_PI_2){                      //终止角在第四象限
    //         if(targetAngle.x - currentAngle.x < -M_PI)
    //             targetAngle.x += 2 * M_PI;
    //     }
    // }
    // else if(currentAngle.x < -M_PI_2 && currentAngle.x > -M_PI){                    //初始角在第三象限
    //     if(targetAngle.x > 0 && targetAngle.x < M_PI_2){                            //终止角在第一象限
    //         if(targetAngle.x - currentAngle.x > M_PI)
    //             targetAngle.x -= 2 * M_PI;
    //     }
    //     else if(targetAngle.x > M_PI_2 && targetAngle.x < M_PI){                    //终止角在第二象限
    //         targetAngle.x -= 2 * M_PI;
    //     }
    // }
    // else if(currentAngle.x > -M_PI_2 && currentAngle.x < 0){                        //初始角在第四象限
    //         if(targetAngle.x > M_PI_2 && targetAngle.x < M_PI){
    //             if(targetAngle.x - currentAngle.x > M_PI)
    //                 targetAngle.x -= 2 * M_PI;
    //         }
    // }

    // if(currentAngle.y > 0 && currentAngle.y < M_PI_2){                              //初始角在第一象限
    //     if(targetAngle.y < - M_PI_2 && targetAngle.y > -M_PI)                       //终止角在第三象限
    //         targetAngle.y += 2 * M_PI;
    // }
    // else if(currentAngle.y > M_PI_2 && currentAngle.y < M_PI){                      //初始角在第二象限
    //     if(targetAngle.y < -M_PI_2 && targetAngle.y > -M_PI){                       //终止角在第三象限
    //         targetAngle.y += 2 * M_PI;
    //     }       
    //     else if(targetAngle.y < 0 && targetAngle.y > -M_PI_2){                      //终止角在第四象限
    //         if(targetAngle.y - currentAngle.y < -M_PI)
    //             targetAngle.y += 2 * M_PI;
    //     }
    // }
    // else if(currentAngle.y < -M_PI_2 && currentAngle.y > -M_PI){                    //初始角在第三象限
    //     if(targetAngle.y > 0 && targetAngle.y < M_PI_2){                            //终止角在第一象限
    //         if(targetAngle.y - currentAngle.y > M_PI)
    //             targetAngle.y -= 2 * M_PI;
    //     }
    //     else if(targetAngle.y > M_PI_2 && targetAngle.y < M_PI){                    //终止角在第二象限
    //         targetAngle.y -= 2 * M_PI;
    //     }
    // }
    // else if(currentAngle.y > -M_PI_2 && currentAngle.y < 0){                        //初始角在第四象限
    //         if(targetAngle.y > M_PI_2 && targetAngle.y < M_PI){
    //             if(targetAngle.y - currentAngle.y > M_PI)
    //                 targetAngle.y -= 2 * M_PI;
    //         }
    // }

    // if(currentAngle.z > 0 && currentAngle.z < M_PI_2){                              //初始角在第一象限
    //     if(targetAngle.z < - M_PI_2 && targetAngle.z > -M_PI)                       //终止角在第三象限
    //         targetAngle.z += 2 * M_PI;
    // }
    // else if(currentAngle.z > M_PI_2 && currentAngle.z < M_PI){                      //初始角在第二象限
    //     if(targetAngle.z < -M_PI_2 && targetAngle.z > -M_PI){                       //终止角在第三象限
    //         targetAngle.z += 2 * M_PI;
    //     }       
    //     else if(targetAngle.z < 0 && targetAngle.z > -M_PI_2){                      //终止角在第四象限
    //         if(targetAngle.z - currentAngle.z < -M_PI)
    //             targetAngle.z += 2 * M_PI;
    //     }
    // }
    // else if(currentAngle.z < -M_PI_2 && currentAngle.z > -M_PI){                    //初始角在第三象限
    //     if(targetAngle.z > 0 && targetAngle.z < M_PI_2){                            //终止角在第一象限
    //         if(targetAngle.z - currentAngle.z > M_PI)
    //             targetAngle.z -= 2 * M_PI;
    //     }
    //     else if(targetAngle.z > M_PI_2 && targetAngle.z < M_PI){                    //终止角在第二象限
    //         targetAngle.z -= 2 * M_PI;
    //     }
    // }
    // else if(currentAngle.z > -M_PI_2 && currentAngle.z < 0){                        //初始角在第四象限
    //         if(targetAngle.z > M_PI_2 && targetAngle.z < M_PI){
    //             if(targetAngle.z - currentAngle.z > M_PI)
    //                 targetAngle.z -= 2 * M_PI;
    //         }
    // }
    

    // _vel_target.x = pi_control(targetAngle.x, currentAngle.x, dt);
    // _vel_target.y = pi_control(targetAngle.y, currentAngle.y, dt);
    // _vel_target.z = pi_control(targetAngle.z, currentAngle.z, dt);

    _vel_target = pi_control(targetAngle, currentAngle, dt);

}

Vector3f AE_AngleControl::pi_control(Vector3f target, Vector3f messurement, float dt)
{
    Vector3f error, output;

    //大臂pi输出
    error.x = target.x - messurement.x;
    _integral.x += error.x * dt;
    //积分限幅
    if(_integral.x > 20){
        _integral.x = 20;
    }
    else if(_integral.x < -20){
        _integral.x = -20;
    }
    output.x = _kp_boom * error.x + _ki_boom * _integral.x;

    //小臂pi输出
    error.y = target.y - messurement.y;

    _integral.y += error.y * dt;

    output.y = _kp_forearm * error.y + _ki_forearm * _integral.y;

    //铲斗pi输出
    error.z = target.z - messurement.z;

    _integral.z += error.z * dt;

    output.z = _kp_bucket * error.z + _ki_bucket * _integral.z;

    return output;
}

bool AE_AngleControl::getCurrentAngle(Vector3f& currentAngle)
{
    if(nullptr == _armInfo)
    {
        _armInfo = AE_RobotArmInfo::get_singleton();

        if(nullptr == _armInfo)
        {
            return false;
        }
    }

    if(nullptr == _armInfo_backend)
    {
        _armInfo_backend = _armInfo->backend();

        if(nullptr == _armInfo_backend)
        {
            return false;
        }
    }

    AE_RobotArmInfo_Excavator *arminfo_backend = (AE_RobotArmInfo_Excavator*)_armInfo_backend;

    AE_RobotArmInfo_Excavator::Excavator_DH_Anger dh_angle = arminfo_backend->get_DH_Angles();

    currentAngle.x = dh_angle.boom_to_slewing;
    currentAngle.y = dh_angle.forearm_to_boom;
    currentAngle.z = dh_angle.bucket_to_forearm;
    
    return true;
}