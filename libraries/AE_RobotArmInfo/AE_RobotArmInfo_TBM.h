
#pragma once
#include "AE_RobotArmInfo_Backend.h"
#include "AE_RobotArmInfo.h"

// Number of Cylinders for Cutter Head Assembly. One cylinder controls the vertical, the other one controls the herizontal.
#ifndef CUTTING_HEADER_CYLINDER_NUM
#define CUTTING_HEADER_CYLINDER_NUM 2
#endif

#ifndef BACK_SUPPORT_LEG_CYLINDER_NUM
#define BACK_SUPPORT_LEG_CYLINDER_NUM 2
#endif

#ifndef TBM_OIL_CYLINDER_NUM_MAX
#define TBM_OIL_CYLINDER_NUM_MAX 2
#endif

class AE_RobotArmInfo_TBM : public AE_RobotArmInfo_Backend
{

public:
    // Constructor
    using AE_RobotArmInfo_Backend::AE_RobotArmInfo_Backend;

    // perform any required initialisation of backend
    void init() override;

    // retrieve updates from sensor
    void update() override;

    struct TBM_Cutting_Header_State {
        float cutting_header_height;
        float cutting_header_yaw;
        struct AE_RobotArmInfo::TBM_CH_Cylinder_State cylinder_status[TBM_OIL_CYLINDER_NUM_MAX];
    };

    struct Back_Support_Leg_State {
        // float height;
        // float yaw;
    };

    //get the cutting header state at base's body frame
    TBM_Cutting_Header_State get_TBM_cutting_header_state() const { return _cutting_header_state; }

    //update the cutting header state at base's body frame
    // void  AP_TBM_Info::update_TBM_cutting_header_state(const Vector3f &boom_atti_deg);    

private:

    TBM_Cutting_Header_State _cutting_header_state;


};
