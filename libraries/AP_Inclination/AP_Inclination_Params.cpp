#include "AP_Inclination_Params.h"
#include "AP_Inclination.h"

// table of user settable parameters
const AP_Param::GroupInfo AP_Inclination_Params::var_info[] = {
    // @Param: TYPE
    // @DisplayName: inclination type
    // @Description: Type of connected inclination
    // @Values: 0:None,1:HDA436T_Serial,2:three_HDA436Ts_Serial,100:SIM
    // @User: Standard
    AP_GROUPINFO_FLAGS("TYPE", 1, AP_Inclination_Params, type, 2, AP_PARAM_FLAG_ENABLE),

    // @Param: LOCATION
    // @DisplayName: Inclination location
    // @Description: Install location of inclination
    // @Values: 0:LOCATION_NONE, 1:Boom, 2:Forearm, 3:bucket
    // @User: Advanced
    AP_GROUPINFO("LOCATION", 4, AP_Inclination_Params, location, Boom),

    // @Param: MIN_DEG
    // @DisplayName: Inclination minimum roll angle
    // @Description: Minimum roll angle in degree that Inclination can reliably read
    // @Units: degree
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("MIN_DEG",  6, AP_Inclination_Params, min_roll_deg, -180),

    // @Param: MAX_DEG
    // @DisplayName: Inclination maximum roll angle
    // @Description: Maximum roll angle in degree that Inclination can reliably read
    // @Units: degree
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("MAX_DEG",  7, AP_Inclination_Params, max_roll_deg, 180),

    AP_GROUPEND
};

AP_Inclination_Params::AP_Inclination_Params(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}
