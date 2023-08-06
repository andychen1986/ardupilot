#include "mode.h"
#include "Rover.h"

bool ModeDig::_enter()
{
    stop_vehicle();

    // fail to enter tbm if no mission commands
    if (g2.rbtarmwp.get_rbtarm_waypoint_total() < 1) {
        gcs().send_text(MAV_SEVERITY_INFO, "No Mission. Can't set %s.", name4());
        return false;
    }

    if (!__enter()) {
        gcs().send_text(MAV_SEVERITY_INFO, "fail to enter mode %s", name4());
        return false;
    }

    // initialise waypoint speed
    g2.wp_nav_arm.set_desired_vel_to_default();

    mission.set_cmd_total(g2.rbtarmwp.get_rbtarm_waypoint_total());

    // set flag to start mission
    waiting_to_start = true;

    _is_last = false;

    return true;
}

void ModeDig::update()
{
    // start or update mission
    if (waiting_to_start) {
        mission.start_or_resume();

        waiting_to_start = false;
    } else {
        mission.update();
    }

    switch (_dig_submode) {
    case Dig_TBM: {
        // nav to target loc (tbm)
        if (!g2.wp_nav_arm.reached_destination()) {
            // update navigation controller
            navigate_to_arm_waypoint();
        } else {
            if(_is_last) {
                stop_arm();      
            }
        }

        break;
    }
    case Dig_Excavator: {
        // nav to target loc (excavator)
        // ...
        break;
    }
    default:
        break;
    }
}

bool ModeDig::_start_command(const RobotArmLocation& cmd)
{
    bool ret = false;

    ret |= do_nav_wp(cmd);
    
    ret |= start_command(cmd);

    set_submode();

    gcs().send_text(MAV_SEVERITY_INFO, "Mode%s::start_command()\n", name4());

    return ret;
}

void ModeDig::_exit_mission()
{
    exit_mission();

    gcs().send_text(MAV_SEVERITY_INFO, "Mode%s::exit_mission()\n", name4());
}

bool ModeDig::_verify_command_callback(const RobotArmLocation& cmd)
{
    bool ret = false;

    ret |= verify_command_callback(cmd);

    // if(_is_last && ret)
    // {
    //     rover.set_mode(rover.mode_auto, ModeReason::GCS_COMMAND);
    // }

    return ret;
}

void ModeDig::_exit()
{
    // stop running the mission
    if (mission.state() == AE_Mission_Arm::MISSION_RUNNING) {
        mission.stop();
    }

    __exit();
}
