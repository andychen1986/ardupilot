#include "SIM_TBM.h"
#include <AP_Math/matrix3.h>

namespace SITL {

#define    Rotation_Channel     9
#define    Boom_Channel         11
#define Debug(fmt, args ...)  do {::fprintf(stderr, "%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)

void SimTBM::init()
{
    if(!get_tbm_info()) {
        return;
    }

    AE_RobotArmInfo::TBM_PARAM tbm_params = _armInfo_backend->get_tbm_param();
    Vector3f _euler_boom_e2b_from_sensor;
    Matrix3f dbodyrbody;
    Matrix3f dboomrboom;

    dbodyrbody.from_euler(ahrs.get_roll(), ahrs.get_pitch(), ahrs.get_yaw());
    Debug("n_ahrs_roll:%f,n_ahrs_pitch:%f,n_ahrs_yaw:%f", degrees(ahrs.get_roll()), degrees(ahrs.get_pitch()), degrees(ahrs.get_yaw()));
    
    Matrix3f dboomrdbody;
    dboomrdbody.from_euler(0, 0, radians(-90));
    Matrix3f dboomrbody  = dboomrdbody*dbodyrbody;
    
    // bodyrdboom
    if(!dboomrbody.invert())
    {
        return;
    }
    _euler_boom_e2b_from_sensor.x = sitl->state.inclination_state.roll_deg.x;
    _euler_boom_e2b_from_sensor.y = sitl->state.inclination_state.pitch_deg.x;
    _euler_boom_e2b_from_sensor.z = sitl->state.inclination_state.yaw_deg.x;

    dboomrboom.from_euler(radians(_euler_boom_e2b_from_sensor.x),radians(_euler_boom_e2b_from_sensor.y),radians(_euler_boom_e2b_from_sensor.z));
    // bodyrboom = bodyrdboom * dboomrboom;
    dboomrboom = dboomrbody*dboomrboom;
    // bodyrboom
    dboomrboom.to_euler(&_euler_boom_e2b_from_sensor.x,&_euler_boom_e2b_from_sensor.y,&_euler_boom_e2b_from_sensor.z);
    Debug("n_b_roll:%f,n_b_pitch:%f,n_b_yaw:%f", degrees(_euler_boom_e2b_from_sensor.x), degrees(_euler_boom_e2b_from_sensor.y),degrees(_euler_boom_e2b_from_sensor.z));

    boom_cylinder_length = sqrt(tbm_params._mm_AC * tbm_params._mm_AC + tbm_params._mm_BC * tbm_params._mm_BC -  \
                            2*tbm_params._mm_AC*tbm_params._mm_BC * cos(radians(tbm_params._deg_TCA) + \
                            radians(tbm_params._deg_BCF) + radians(tbm_params._deg_BFC) + _euler_boom_e2b_from_sensor.x));

    rotation_rad = _euler_boom_e2b_from_sensor.z;

    is_init = true;
}

/* update model by one time step */
void SimTBM::update(const struct sitl_input &input)
{
    if(!get_tbm_info()) {
        return;
    }

    if(!is_init) {
        init();
    }

    calc_speed(input.servos[Rotation_Channel-1], input.servos[Boom_Channel-1]);

    // how much time has passed?
    float delta_time = frame_time_us * 1.0e-6f;

    AE_RobotArmInfo::TBM_PARAM tbm_params = _armInfo_backend->get_tbm_param();

    Debug("boom:%f, rotation:%f, pwm_boom:%d, pwm_rotation:%d", boom_speed, rotation_speed, input.servos[Boom_Channel-1], input.servos[Rotation_Channel-1]);

    boom_cylinder_length += boom_speed * delta_time;

    rotation_rad += rotation_speed * delta_time;
    Debug("boom_cylinder_length:%f", boom_cylinder_length);

    float theta2 = acosf( (powf(tbm_params._mm_AC, 2) + powf(tbm_params._mm_BC, 2) - powf(boom_cylinder_length, 2)) \
                        / (2*tbm_params._mm_AC*tbm_params._mm_BC)) - radians(tbm_params._deg_BCF) - radians(tbm_params._deg_TCA);
    // Debug("tbm_params._mm_AC:%f, tbm_params._mm_BC:%f, boom_cylinder_length:%f",powf(tbm_params._mm_AC, 2), powf(tbm_params._mm_BC.get(), 2), 2*tbm_params._mm_AC*tbm_params._mm_BC);

    float roll_to_body = theta2 - radians(tbm_params._deg_BFC);
    // Debug("roll_to_body:%f rotation_rad:%f", roll_to_body, rotation_rad);

    Matrix3f bodyrboom;

    bodyrboom.from_euler(roll_to_body, 0, rotation_rad);

    Debug("b_roll:%f,b_pitch:%f,b_yaw:%f", degrees(roll_to_body), 0.0, degrees(rotation_rad));

    Matrix3f dbodyrbody;
    dbodyrbody.from_euler(ahrs.get_roll(), ahrs.get_pitch(), ahrs.get_yaw());
    Debug("ahrs_roll:%f,ahrs_pitch:%f,ahrs_yaw:%f", degrees(ahrs.get_roll()), degrees(ahrs.get_pitch()), degrees(ahrs.get_yaw()));

    Matrix3f dboomrdbody;
    dboomrdbody.from_euler(0, 0, radians(-90));

    Matrix3f dboomrboom;
    dboomrboom = dboomrdbody*dbodyrbody*bodyrboom;

    dboomrboom.to_euler(&sitl->state.inclination_state.roll_deg.x, &sitl->state.inclination_state.pitch_deg.x, \
                        &sitl->state.inclination_state.yaw_deg.x);
    
    sitl->state.inclination_state.roll_deg.x = degrees(sitl->state.inclination_state.roll_deg.x);
    sitl->state.inclination_state.pitch_deg.x = degrees(sitl->state.inclination_state.pitch_deg.x);
    sitl->state.inclination_state.yaw_deg.x = degrees(sitl->state.inclination_state.yaw_deg.x);

    Debug("roll:%f,pitch:%f,yaw:%f",sitl->state.inclination_state.roll_deg.x, sitl->state.inclination_state.pitch_deg.x, sitl->state.inclination_state.yaw_deg.x);
    Debug("                                              ");

    SimRover::update(input);
}

// Motor model
void SimTBM::calc_speed(uint16_t pwm_rotation, uint16_t pwm_boom)
{
    if(pwm_boom < 1100  || pwm_boom > 1900 || pwm_rotation < 1100 || pwm_rotation > 1900)
    {
        boom_speed = rotation_speed = 0;

        return;
    }

    boom_speed = (pwm_boom - 1500) / 500.0 * 15;

    rotation_speed = (pwm_rotation - 1500) / 500.0 * radians(16);
}

bool SimTBM::get_tbm_info()
{
    if(_armInfo == nullptr) {
        _armInfo = AE_RobotArmInfo::get_singleton();
        
        if(_armInfo == nullptr) {
            return false;
        }
    }
    
    if (_armInfo_backend == nullptr) {
        _armInfo_backend = (AE_RobotArmInfo_TBM*)_armInfo->backend();

        if(_armInfo_backend == nullptr) {
            return false;
        }
    }

    if(!_armInfo_backend->get_healthy()) {
        return false;
    }

    return true;
}

} // namespace SITL
