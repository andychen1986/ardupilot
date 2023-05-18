#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

class AP_Inclination_Params
{
public:
    static const struct AP_Param::GroupInfo var_info[];

    AP_Inclination_Params(void);

    /* Do not allow copies */
    AP_Inclination_Params(const AP_Inclination_Params &other) = delete;
    AP_Inclination_Params &operator=(const AP_Inclination_Params&) = delete;

    AP_Vector3f inclination;
    AP_Float roll_deg;
    AP_Float pitch_deg;
    AP_Float yaw_deg;

    AP_Int8 type;

    AP_Float min_roll_deg;
    AP_Float max_roll_deg;

    AP_Float min_yaw_deg;
    AP_Float max_yaw_deg;

    AP_Float temperature;
    AP_Int8  location;

};
