#include "mode.h"
#include "Rover.h"
#include <AP_Math/matrixN.h>
#include <AP_Math/vectorN.h>
#include <AE_RobotArmInfo/AE_RobotArmInfo.h>
#include <AE_RobotArmInfo/AE_RobotArmInfo_Backend.h>
#include <AE_RobotArmInfo/AE_RobotArmInfo_TBM.h>
#include <AE_RobotArmInfo/AE_RobotArmInfo_Excavator.h>

bool ModeExcavator::__enter()
{
    // do some config (Excavator)
    gcs().send_text(MAV_SEVERITY_INFO, "ModeExcavator::__enter()");

    //获取挖掘机的大臂，小臂，挖斗长度
    if(!get_ExcavatorParam())
    {
        gcs().send_text(MAV_SEVERITY_INFO, "can't get EXCAVATOR lenth param.");
        return false;
    }

    return true;
}

bool ModeExcavator::start_command(const RobotArmLocation& cmd)
{ 
    return true;
}

bool ModeExcavator::verify_command_callback(const RobotArmLocation& cmd)
{
    // for test
    RobotArmLocation real_cmd(cmd);
    convert_wp(real_cmd);

    // reached_destination ? (tbm)
    bool ret = false;

    ret |= g2.wp_nav_arm.reached_destination();

    if(ret) {
        gcs().send_text(MAV_SEVERITY_INFO, "reach x:%f, y:%f, z:%f\n", \
                        real_cmd.xhorizontal, real_cmd.yvertical, real_cmd.zalt);
    }

    return ret;
}

void ModeExcavator::set_submode()
{
    _dig_submode = Dig_Excavator;
}

bool ModeExcavator::do_nav_wp(const RobotArmLocation& cmd)
{
    // for test
    RobotArmLocation real_cmd(cmd);
    convert_wp(real_cmd);

    gcs().send_text(MAV_SEVERITY_INFO, "deatination x:%f, y:%f, z:%f\n", \
                    real_cmd.xhorizontal, real_cmd.yvertical, real_cmd.zalt);

    _is_last = mission.is_last_wp();

    //在任务开始时将坐标转换为目标的dh角度
    RobotArmJointAngle dest_angle;
    if (!g2.wp_nav_arm.kinematic_inverse(real_cmd, dest_angle, l_boom, l_forearm, l_bucket)) {
        return false;
    }

    //设置角度为期望到达角度
    if (!g2.wp_nav_arm.set_desired_jointAngle(dest_angle)) {
        return false;
    }

    _reached_destination = false;

    return true;
}

void ModeExcavator::exit_mission()
{
    
}

void ModeExcavator::__exit()
{

}

// high level call to navigate to waypoint
void ModeExcavator::navigate_to_arm_waypoint()
{
    // update navigation controller
    g2.wp_nav_arm.update_excavator(rover.get_dt());

    Vector3f desired_output = g2.wp_nav_arm.get_desiredVel_excavator();

    desired_output.x = constrain_float(desired_output.x, -100, +100);
    desired_output.y = constrain_float(desired_output.y, -100, +100);
    desired_output.z = constrain_float(desired_output.z, -100, +100);

    // static int count = 0;
    // count++;
    // if(count >= 60) {
    //     count = 0;
    //     gcs().send_text(MAV_SEVERITY_INFO,"desired_output boom:%f forearm:%f, bucket:%f", desired_output.x, desired_output.y, desired_output.z);
    // }

    g2.arm_motors.set_boom(-desired_output.x);
    g2.arm_motors.set_forearm(desired_output.y);
    g2.arm_motors.set_bucket(desired_output.z);
}

// performs a controlled stop
void ModeExcavator::stop_arm()
{
    g2.arm_motors.set_boom(0);
    g2.arm_motors.set_forearm(0);
    g2.arm_motors.set_bucket(0);
}

#define xscale 800
#define yscale 650
// 坐标转换，变换为在gcs中以左边的中间为原点，向右为x轴，向上为y轴
void ModeExcavator::convert_wp(RobotArmLocation& cmd)
{
    float tmp_loc[4] = {cmd.xhorizontal*xscale, cmd.yvertical*yscale, cmd.zalt, 1.0f};
    VectorN<float, 4> loc(tmp_loc);

    float tmp_tran[4][4] = {
        1,  0,  0,         0,
        0, -1,  0,  yscale/2,
        0,  0, -1,         0,
        0,  0,  0,         1
    };

    // float tmp_tran[4][4] = {
    //     1,  0, 0, -xscale/2,
    //     0, -1, 0,    yscale,
    //     0,  0, 1,         0,
    //     0,  0, 0,         1
    // };
    MatrixN<float, 4> transformation(tmp_tran);

    loc = transformation*loc;

    cmd = loc;
}

bool ModeExcavator::get_ExcavatorParam()
{
    AE_RobotArmInfo *_armInfo;
    AE_RobotArmInfo_Backend* _armInfo_backend;

    _armInfo = AE_RobotArmInfo::get_singleton();

    if(_armInfo == nullptr) {
        return false;
    }
    
    _armInfo_backend = _armInfo->backend();

    if(_armInfo_backend == nullptr) {
        return false;
    }

    AE_RobotArmInfo_Excavator *arminfo_backend = (AE_RobotArmInfo_Excavator*)_armInfo_backend;

    l_boom = arminfo_backend->get_ex_param()._mm_CF;
    l_forearm = arminfo_backend->get_ex_param()._mm_FQ;
    l_bucket = arminfo_backend->get_ex_param()._mm_QV;

    gcs().send_text(MAV_SEVERITY_INFO, "excavator_param l1:%f, l1:%f, l3:%f\n", \
                        l_boom, l_forearm, l_bucket);
    return true;
}