
#pragma once
#include "AE_RobotArmInfo_Backend.h"

#ifndef EXCAVATOR_ARM_CYLINDER_NUM
#define EXCAVATOR_ARM_CYLINDER_NUM 3
#endif

class AE_RobotArmInfo_Excavator : public AE_RobotArmInfo_Backend
{

public:
    // Constructor
    using AE_RobotArmInfo_Backend::AE_RobotArmInfo_Backend;

    // perform any required initialisation of backend
    void init() override;

    // retrieve updates excavator robot arm info
    void update() override;

    //get the cutting header state at base's body frame
    // AP_TBM_Info::TBM_Cutting_Header_State get_TBM_cutting_header_state() const { return _cutting_header_state; }

    //update the cutting header state at base's body frame
    // void  AP_TBM_Info::update_TBM_cutting_header_state(const Vector3f &boom_atti_deg);    

private:

    // backend state
    struct Excavator_Robot_Arm_State {
        float rbt_arm_yaw_body;
        enum AE_RobotArmInfo::Component_name component; 
        Vector3f boom_tip_pos_body;
        Vector3f forearm_tip_pos_body;
        Vector3f bucket_tip_pos_body;
        struct AE_RobotArmInfo::Ex_Cylinder_State cylinder_status[OIL_CYLINDER_NUM_MAX];
    }ex_info;

    // struct Excavator_Arm_State {
    //     float height;
    //     float yaw;
    //     struct AE_RobotArmInfo::Cylinder_State status[EXCAVATOR_ARM_CYLINDER_NUM];
    // };
    bool calc_excavator_info(void);
    bool calc_bucket_position(void);
    bool calc_oil_sylinder_length(void);
    bool update_excavator_struct(void);
    bool check_if_info_valid(struct Excavator_Robot_Arm_State& ex_state);

};
