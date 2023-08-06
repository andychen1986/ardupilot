#include "mode.h"
#include "Rover.h"
#include <AP_Math/matrixN.h>
#include <AP_Math/vectorN.h>

bool ModeTBM::__enter()
{
    // do some config (tbm)
    gcs().send_text(MAV_SEVERITY_INFO, "ModeTBM::__enter()");

    return true;
}

bool ModeTBM::start_command(const RobotArmLocation& cmd)
{ 
    return true;
}

bool ModeTBM::verify_command_callback(const RobotArmLocation& cmd)
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

void ModeTBM::set_submode()
{
    _dig_submode = Dig_TBM;
}

bool ModeTBM::do_nav_wp(const RobotArmLocation& cmd)
{
    // for test
    RobotArmLocation real_cmd(cmd);
    convert_wp(real_cmd);

    _is_last = mission.is_last_wp();

    if (!g2.wp_nav_arm.set_desired_location(real_cmd)) {
        return false;
    }

    _reached_destination = false;

    return true;
}

void ModeTBM::exit_mission()
{
    
}

void ModeTBM::__exit()
{

}

// high level call to navigate to waypoint
void ModeTBM::navigate_to_arm_waypoint()
{
    // update navigation controller
    g2.wp_nav_arm.update(rover.get_dt());
    Vector2f desired_output = g2.wp_nav_arm.get_desired_accel();

    desired_output.x = constrain_float(desired_output.x, -100, +100);
    desired_output.y = constrain_float(desired_output.y, -100, +100);

    // debug_luo
    static int count = 0;
    count++;
    if(count >= 60) {
        count = 0;
        gcs().send_text(MAV_SEVERITY_INFO,"desired_output x:%f y:%f", desired_output.x, desired_output.y);
    }

    g2.arm_motors.set_boom(desired_output.y);
    g2.arm_motors.set_rotation(desired_output.x);
}

// performs a controlled stop
void ModeTBM::stop_arm()
{
    g2.arm_motors.set_boom(0);
    g2.arm_motors.set_rotation(0);
}

#define xscale 450
#define yscale 350
// 坐标转换，变换为在gcs中以底边的中间为原点，向右为x轴，向上为y轴
void ModeTBM::convert_wp(RobotArmLocation& cmd)
{
    float tmp_loc[4] = {cmd.xhorizontal*xscale, cmd.yvertical*yscale, cmd.zalt, 1.0f};
    VectorN<float, 4> loc(tmp_loc);
    
    float tmp_tran[4][4] = {
        1,  0, 0, -xscale/2,
        0, -1, 0,    yscale,
        0,  0, 1,         0,
        0,  0, 0,         1
    };
    MatrixN<float, 4> transformation(tmp_tran);

    loc = transformation*loc;

    cmd = loc;
}
