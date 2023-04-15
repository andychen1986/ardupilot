/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_Inclination_HDA436T_Serial.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <ctype.h>

#define HDA436T_HDR 0x03   // Header Byte from USD1_Serial
#define HDA436T_DATA_LENGTH 0x0E // length of Data for Byte of HDA436T_Serial

extern const AP_HAL::HAL& hal;

#define BENEWAKE_FRAME_HEADER 0x03
#define BENEWAKE_FRAME_LENGTH 19
#define ROLL_YAW_OFFSET 180000
#define PITCH_OFFSET 90000
#define INCLINATION_ROLL_MAX_DEGREE 180 
#define INCLINATION_YAW_MAX_DEGREE 180 

// format of serial packets received from inclination sensor HDA436T
//
// Data Bit             Definition      Description
// ------------------------------------------------
// byte 0               Frame header    0x03
// byte 1               Frame header    0x03
// byte 2               DATA_LENGTH     roll pitch yaw temperature these 4 data totle length of byte, default is 0x0E
// byte 3               ROLL1_L         roll raw data 1 low 8 bits
// byte 4               ROLL1_H         roll raw data 1 high 8 bits
// byte 5               ROLL2_L         roll raw data 2 low 8 bits
// bute 6               ROLL2_H         roll raw data 2 high 8 bits
// byte 7               PITCH1_L        pitch raw data 1 low 8 bits
// bute 8               PITCH1_H        pitch raw data 1 high 8 bits
// byte 9               PITCH2_L        pitch raw data 2 low 8 bits
// byte 10              PITCH2_H        pitch raw data 2 high 8 bits
// byte 11              YAW1_L          yaw raw data 1 low 8 bits
// byte 12              YAW1_H          yaw raw data 1 high 8 bits
// byte 13              YAW2_L          yaw raw data 1 low 8 bits
// byte 14              YAW2_H          yaw raw data 1 high 8 bits
// byte 15              TEMPERATURE1    high 4 bits is positive or negtive?  0:P   1:N ;  low 4 bits is tens bit of tempterature 
// byte 16              TEMPERATURE2    high 4 bits is ones bit of tempterature;   low 4 bits is decimal bit of tempterature 
// byte 17               Checksum       high 8 bits of Checksum byte, sum of bytes 0 to bytes 16
// byte 18               Checksum       low  8 bits of Checksum byte, sum of bytes 0 to bytes 16


// read - return last value measured by sensor
bool AP_Inclination_HDA436T_Serial::get_reading(float &reading_roll_deg, float &reading_yaw_deg)
{
    if (uart == nullptr) {
        return false;
    }

    float sum_roll_deg = 0;
    float sum_yaw_deg = 0;
    uint16_t count = 0;
    uint16_t count_out_of_positive_range = 0;
    uint16_t count_out_of_negtive_range = 0;

    // read any available lines from the inclination
    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        int16_t r = uart->read();
        if (r < 0) {
            continue;
        }

        uint8_t c = (uint8_t)r;
        // if buffer is empty and this byte is 0x03, add to buffer
        if (linebuf_len == 0) {
            if (c == BENEWAKE_FRAME_HEADER) {
                linebuf[linebuf_len++] = c;
            }
        } else if (linebuf_len == 1) {
            // if buffer has 1 element and this byte is 0x03, add it to buffer
            // if not clear the buffer
            if (c == BENEWAKE_FRAME_HEADER) {
                linebuf[linebuf_len++] = c;
            } else {
                linebuf_len = 0;
            }
        } else {
             // add character to buffer
            linebuf[linebuf_len++] = c;
            // if buffer now has 19 items try to decode it
            if (linebuf_len == BENEWAKE_FRAME_LENGTH) {
                // calculate checksum
                uint16_t crc = (linebuf[18]<<8) | linebuf[17];
                if (crc == calc_crc_modbus(linebuf, 17)) {
                    // calculate roll angle
                    int32_t roll_raw = ((uint32_t)linebuf[6] << 24) | ((uint32_t)linebuf[5] << 16) | ((uint16_t)linebuf[4] << 8) | linebuf[3];
                    // int32_t pitch_raw = ((uint32_t)linebuf[10] << 24) | ((uint32_t)linebuf[9] << 16) | ((uint16_t)linebuf[8] << 8) | linebuf[7];
                    int32_t yaw_raw = ((uint32_t)linebuf[14] << 24) | ((uint32_t)linebuf[13] << 16) | ((uint16_t)linebuf[12] << 8) | linebuf[11];
                    float roll = (float)((roll_raw - ROLL_YAW_OFFSET)*0.001); 
                    // float pitch = (float)((pitch_raw - PITCH_OFFSET)*0.001);  
                    float yaw = (float)((yaw_raw - ROLL_YAW_OFFSET)*0.001); 
                    if (roll > INCLINATION_ROLL_MAX_DEGREE || yaw > INCLINATION_YAW_MAX_DEGREE) {
                        // this reading is out of positive range
                        count_out_of_positive_range++;
                    } else if((roll < - INCLINATION_ROLL_MAX_DEGREE) || (yaw < - INCLINATION_YAW_MAX_DEGREE)){
                        // this reading is out of negtive range
                        count_out_of_negtive_range++;
                    } else {
                        // add degree to sum
                        //hal.console->printf("555inclination tilt sensor uart: %f\t, %lu\t,  %lu\r\n", roll, roll_raw, (roll_raw - ROLL_YAW_OFFSET));
                        sum_roll_deg += roll;
                        sum_yaw_deg += yaw;
                        count++;
                    }
                }                

                // clear buffer
                linebuf_len = 0;
            }
        }
    }

    if (count > 0) {
        // return average distance of readings
        reading_roll_deg = sum_roll_deg / count;   
        reading_yaw_deg = sum_yaw_deg / count;  
        return true;
    }

    if (count_out_of_positive_range > 0) {
        // if out of range readings return maximum range for the positive angle
        reading_roll_deg = INCLINATION_ROLL_MAX_DEGREE;
        reading_yaw_deg = INCLINATION_YAW_MAX_DEGREE;
        return true;
    }

    if (count_out_of_negtive_range > 0) {
        // if out of range readings return maximum range for the negtive angle
        reading_roll_deg = - INCLINATION_ROLL_MAX_DEGREE;
        reading_yaw_deg = - INCLINATION_YAW_MAX_DEGREE;
        return true;
    }

    // no readings so return false
    return false;
}
