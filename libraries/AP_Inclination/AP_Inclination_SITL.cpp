#include "AP_Inclination_SITL.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#define INCLINATION_ROLL_MAX_DEGREE 180
#define INCLINATION_PITCH_MAX_DEGREE 90
#define INCLINATION_YAW_MAX_DEGREE 180

bool AP_Inclination_SITL::get_reading(Vector3f &reading_roll_deg, Vector3f &reading_pitch_deg, Vector3f &reading_yaw_deg, InstallLocation location)
{
    if (sitl == nullptr) {
        return false;
    }

    reading_roll_deg = sitl->state.inclination_state.roll_deg.limit_xyz(-INCLINATION_ROLL_MAX_DEGREE, INCLINATION_ROLL_MAX_DEGREE);
    reading_pitch_deg = sitl->state.inclination_state.pitch_deg.limit_xyz(-INCLINATION_PITCH_MAX_DEGREE, INCLINATION_PITCH_MAX_DEGREE);
    reading_yaw_deg = sitl->state.inclination_state.yaw_deg.limit_xyz(-INCLINATION_YAW_MAX_DEGREE, INCLINATION_YAW_MAX_DEGREE);

    state.last_reading_ms = AP_HAL::millis();

    return true;
}

#endif
