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

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_Inclination.h"
#include "AP_Inclination_Backend.h"

extern const AP_HAL::HAL& hal;

/*
  base class constructor.
  This incorporates initialisation as well.
*/
AP_Inclination_Backend::AP_Inclination_Backend(Inclination::Inclination_State &_state, AP_Inclination_Params &_params) :
    state(_state),
    params(_params)
{
    _backend_type = type();
}

Inclination::Status AP_Inclination_Backend::status() const
{
    if (type() == Inclination::Type::NONE) {
        // turned off at runtime?
        return Inclination::Status::NotConnected;
    }
    return state.status;
}

// true if sensor is returning data
bool AP_Inclination_Backend::has_data() const
{
    return ((state.status != Inclination::Status::NotConnected) &&
            (state.status != Inclination::Status::NoData));
}

// update status based on roll angle measurement
void AP_Inclination_Backend::update_status(enum InstallLocation loc)
{
    // check roll angle
    if (get_roll_deg_from_location(loc) > get_max_roll_deg()) {
        set_status(Inclination::Status::OutOfRangeHigh);
    } else if (get_roll_deg_from_location(loc) < get_min_roll_deg()) {
        set_status(Inclination::Status::OutOfRangeLow);
    } else {
        set_status(Inclination::Status::Good);
    }
}

// set status and update valid count
void AP_Inclination_Backend::set_status(Inclination::Status _status)
{
    state.status = _status;

    // update valid count
    if (_status == Inclination::Status::Good) {
        if (state.range_valid_count < 10) {
            state.range_valid_count++;
        }
    } else {
        state.range_valid_count = 0;
    }
}

