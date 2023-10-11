#pragma once

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "AP_Inclination.h"
#include "AP_Inclination_Backend_Serial.h"
#include <SITL/SITL.h>


class AP_Inclination_SITL : public AP_Inclination_Backend_Serial
{

public:

    AP_Inclination_SITL(Inclination::Inclination_State &_state, 
                        AP_Inclination_Params &_params) : 
    AP_Inclination_Backend_Serial(_state, _params),
    sitl(AP::sitl()) {}

private:
    SITL::SIM *sitl;

    // get a reading,Vector3f's first value is boom, second is forearm, third is bucket.
    bool get_reading(Vector3f &reading_roll_deg, Vector3f &reading_pitch_deg, Vector3f &reading_yaw_deg, InstallLocation location) override;
};

#endif
