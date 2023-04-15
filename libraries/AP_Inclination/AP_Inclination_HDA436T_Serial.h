#pragma once

#include "AP_Inclination.h"
#include "AP_Inclination_Backend_Serial.h"

class AP_Inclination_HDA436T_Serial : public AP_Inclination_Backend_Serial
{

public:

    using AP_Inclination_Backend_Serial::AP_Inclination_Backend_Serial;

private:

    // get a reading
    bool get_reading(float &reading_roll_m, float &reading_yaw_m) override;

    uint8_t  linebuf[20];
    uint8_t  linebuf_len;
};
