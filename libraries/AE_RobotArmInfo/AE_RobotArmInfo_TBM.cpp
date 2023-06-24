#define ALLOW_DOUBLE_MATH_FUNCTIONS

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>
#include "AE_RobotArmInfo_TBM.h"


// perform any required initialisation of backend
void AE_RobotArmInfo_TBM::init()
{
    // update health
    _state.flags.healthy = true;

    // assign the initial value of Robot_Arm_State::_backend_state at the front end
    _state.type = (int8_t)AE_RobotArmInfo::Type::TBM;

    // initialize according to different backend types
    _cuthead_state.cutheader_height = 0;
    _cuthead_state.cutheader_horizon_pos = 0;
    _cuthead_state.cutheader_horizon_vel = 0;
    _cuthead_state.cutheader_vertical_vel = 0;
    for (uint8_t i=0; i<CUTTING_HEADER_CYLINDER_NUM; i++) {
        _cuthead_state.cylinder_status[i].cylinder_name = (AE_RobotArmInfo::TBM_CH_OC_Name)i;
        _cuthead_state.cylinder_status[i].length_max_mm = get_tbm_param()._cylinder_max[i];
        _cuthead_state.cylinder_status[i].length_mm = 0;
        _cuthead_state.cylinder_status[i].velocity_mms = 0;
    }
}

// retrieve updates from inclination and ahrs sensors
void AE_RobotArmInfo_TBM::update()
{
    Inclination *inclination = AP::inclination();
    if (inclination == nullptr) {
        AP_HAL::panic("AE_RobotArmInfo_TBM should be got inclination data before calc");
        return;
    }
    const AP_AHRS &ahrs = AP::ahrs();

    if (!calc_TBM_info(ahrs, inclination)) {
        // update health
        _state.flags.healthy = false;
    }

    if (check_if_cutting_head_info_valid(_cuthead_state)) {
        _state.flags.healthy = true;
    }

    Write_TBM_CutheadInfo();

    return;
}

bool AE_RobotArmInfo_TBM::calc_TBM_info(const AP_AHRS &_ahrs, const Inclination *_inclination)
{
    if (!update_TBM_cutting_header_state(_ahrs, _inclination)) {
        return false;
    }
    return true;
}

// update the cutting header state at base's body frame
// return true if update successed
bool AE_RobotArmInfo_TBM::update_TBM_cutting_header_state(const AP_AHRS &_ahrs, const Inclination *_inclination)
{
    Matrix3f apm_matrix;
    Matrix3f boom_matrix;
    Vector3f _euler_boom_e2b;
    float boom_to_body;
    float slew_to_body;
    float angle_ACB;

    apm_matrix.from_euler(_ahrs.get_roll(),_ahrs.get_pitch(),radians(_inclination->yaw_deg_location(Boom)));
    if (!apm_matrix.invert()) {
        return false;
    }

    _euler_boom_e2b = _inclination->get_deg_location(Boom);
    boom_matrix.from_euler(radians(_euler_boom_e2b.x),radians(_euler_boom_e2b.y),radians(_euler_boom_e2b.z));
    boom_matrix = apm_matrix*boom_matrix;
    boom_matrix.to_euler(&_euler_boom_e2b.x,&_euler_boom_e2b.y,&_euler_boom_e2b.z);

    boom_to_body = _euler_boom_e2b.y + get_tbm_param()._deg_BFC;
    slew_to_body = radians(_inclination->yaw_deg_location(Boom));
    _cuthead_state.cutheader_height =  get_tbm_param()._mm_CF*sinf(boom_to_body) + get_tbm_param()._mm_JL;
    _cuthead_state.cutheader_horizon_pos = sinf(slew_to_body)*(get_tbm_param()._mm_CF*cosf(boom_to_body) + get_tbm_param()._mm_JC);

    angle_ACB = get_tbm_param()._deg_TCA + get_tbm_param()._deg_BCF + boom_to_body;
    _cuthead_state.cylinder_status[(int8_t)AE_RobotArmInfo::TBM_CH_OC_Name::OC_TBM_CUTHEADER_VERTIC].length_mm = sqrt(get_tbm_param()._mm_AC
            *get_tbm_param()._mm_AC + get_tbm_param()._mm_BC*get_tbm_param()._mm_BC - 2*get_tbm_param()._mm_AC*get_tbm_param()._mm_BC*cos(angle_ACB));
    return true;
}

// return true if cutting header info is valid
bool AE_RobotArmInfo_TBM::check_if_cutting_head_info_valid(struct TBM_Cutting_Header_State& cuthead_state)
{
    if (cuthead_state.cylinder_status[0].length_mm > cuthead_state.cylinder_status[0].length_max_mm) {
        return false;
    }
    return true;
}

void AE_RobotArmInfo_TBM::Write_TBM_CutheadInfo()
{
    AP::logger().Write("ARMP", "TimeUS,height,horiz,boomLength",
                       "sm", // units: seconds, meters
                       "FB", // mult: 1e-6, 1e-2
                       "Qfff", // format: uint64_t, float, float, float
                       AP_HAL::micros64(),
                       (float)_cuthead_state.cutheader_height,
                       (float)_cuthead_state.cutheader_horizon_pos,
                       (float)_cuthead_state.cylinder_status[0].length_mm);
}



