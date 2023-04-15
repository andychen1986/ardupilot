#include "AP_Inclination_Params.h"
#include "AP_Inclination.h"

// table of user settable parameters
const AP_Param::GroupInfo AP_Inclination_Params::var_info[] = {
    // @Param: TYPE
    // @DisplayName: inclination type
    // @Description: Type of connected inclination
    // @Values: 0:None,1:HDA436T_Serial
    // @User: Standard
    AP_GROUPINFO_FLAGS("TYPE", 1, AP_Inclination_Params, type, 1, AP_PARAM_FLAG_ENABLE),

    // @Param: LOCATION
    // @DisplayName: Inclination location
    // @Description: Install location of inclination
    // @Values: 0:LOCATION_NONE, 1:Boom, 2:Forearm, 3:bucket
    // @User: Advanced
    AP_GROUPINFO("LOCATION", 4, AP_Inclination_Params, location, Boom),

    AP_GROUPEND
};

AP_Inclination_Params::AP_Inclination_Params(void) {
    AP_Param::setup_object_defaults(this, var_info);
}
