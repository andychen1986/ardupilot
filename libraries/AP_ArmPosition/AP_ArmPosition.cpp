#define ALLOW_DOUBLE_MATH_FUNCTIONS

#include <cmath>

#include "AP_ArmPosition.h"
#include <cmath>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Inclination/AP_Inclination.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

bool AP_ArmLocation_Excavator::check_if_outrange(AP_ArmLocation_Excavator::Oil_Cylinder_STROKE oil_cylinder_stroke)const{
    switch(oil_cylinder_stroke){
    case Oil_Cylinder_STROKE::OC_STROKE_BOOM_MAX: return state.boom_out_range_up;
    case Oil_Cylinder_STROKE::OC_STROKE_BOOM_MIN: return state.boom_out_range_down;
    case Oil_Cylinder_STROKE::OC_STROKE_FOREARM_MAX: return state.forearm_out_range_up;
    case Oil_Cylinder_STROKE::OC_STROKE_FOREARM_MIN: return state.forearm_out_range_down;
    case Oil_Cylinder_STROKE::OC_STROKE_BUCKET_MAX: return state.bucket_out_range_up;
    case Oil_Cylinder_STROKE::OC_STROKE_BUCKET_MIN: return state.bucket_out_range_down;
    default: return true;
    }
    return true;
}


void AP_ArmLocation_Excavator::update_TBM_cutting_head()
{
    _TBM_cutting_head.x = _distance_CF * sinf(_angle_boom_to_body_safe)+_distance_slewing_to_ground;
    _TBM_cutting_head.y = _angle_boom_to_body_safe*180/M_PI;
    _TBM_cutting_head.z = _euler_boom_e2b_from_sensor.z;
}

void AP_ArmLocation_Excavator::update_arm_position()
{
    Inclination *inclination = AP::inclination();
    if(inclination == nullptr){
        return;
    }
    AP_AHRS &ahrs = AP::ahrs();
    //get angle data from sensor and transform to body coordinate
    _euler_boom_e2b_from_sensor = ahrs.earth_to_body(inclination->get_deg_location(Boom));
    _euler_forearm_e2b_from_sensor = ahrs.earth_to_body(inclination->get_deg_location(Forearm));
    _euler_bucket_e2b_from_sensor = ahrs.earth_to_body(-inclination->get_deg_location(Bucket));
    
    //transform angle data from sensor to arm
    get_3angle_to_body(_euler_boom_e2b_from_sensor,_euler_forearm_e2b_from_sensor,_euler_bucket_e2b_from_sensor,0);//ahrs.get_yaw()
    
    //checkout if arm outrange and update boom2slewing,forearm2boom,bucket2forearm angle to cal cyl's length
    _angle_boom_to_slewing = _angle_boom_to_body;
    update_boom_state();
    _angle_forearm_to_boom = _angle_forearm_to_body - _angle_boom_to_slewing;
    update_forearm_state();
    _angle_bucket_to_forearm = _angle_bucket_to_body - _angle_forearm_to_boom - _angle_boom_to_slewing;
    update_bucket_state();
    
    //after checkout give a safe angle for cal tip position
    _angle_boom_to_body_safe = _angle_boom_to_slewing;
    _angle_forearm_to_body_safe = _angle_boom_to_body_safe + _angle_forearm_to_boom;
    _angle_bucket_to_body_safe = _angle_bucket_to_forearm + _angle_forearm_to_boom + _angle_boom_to_slewing;
    
    update_TBM_cutting_head();
    cal_bucket_position_body();
    cal_cyl_length();
    AP::logger().Write("ARMP", "TimeUS,tip_x,tip_y,tip_z,boom_cyl,forearm_cyl,bucket_cyl,boom,s,b",
                   "sm", // units: seconds, meters
                   "FB", // mult: 1e-6, 1e-2
                   "Qfffffffff", // format: uint64_t, float, float, float
                   AP_HAL::micros64(),
                   (float)_position_tip_to_body.x,
                   (float)_position_tip_to_body.y,
                   (float)_position_tip_to_body.z,
                   (float)_cyl_boom_length,
                   (float)_cyl_forearm_length,
                   (float)_cyl_bucket_length,
                   (float)_euler_boom_e2b_from_sensor.x,
                   (float)_euler_forearm_e2b_from_sensor.x,
                   (float)_euler_bucket_e2b_from_sensor.x);
    AP::logger().Write("APP2", "TimeUS,B2SL,S2B,B2ST,boom2b,forearm2b,bucket2b",
                   "sm", // units: seconds, meters
                   "FB", // mult: 1e-6, 1e-2
                   "Qffffff", // format: uint64_t, float, float, float
                   AP_HAL::micros64(),
                   (float)(_angle_boom_to_slewing*180/M_PI),
                   (float)(_angle_forearm_to_boom*180/M_PI),
                   (float)(_angle_bucket_to_forearm*180/M_PI),
                   (float)(_angle_boom_to_body*180/M_PI),
                   (float)(_angle_forearm_to_body*180/M_PI),
                   (float)(_angle_bucket_to_body*180/M_PI));
    Location current_loc;
    if(ahrs.get_position(current_loc)){
    if(current_loc.get_vector_from_origin_NEU(_body_position)){
        _position_tip_to_ned.x = cosf(ahrs.get_yaw()) * _position_tip_to_body.x - sinf(ahrs.get_yaw()) * _position_tip_to_body.y + _body_position.x;
        _position_tip_to_ned.y = sinf(ahrs.get_yaw()) * _position_tip_to_body.x + cosf(ahrs.get_yaw()) * _position_tip_to_body.y + _body_position.y;
        _position_tip_to_ned.z = _position_tip_to_body.z +_body_position.y;
    } 
    }
}


//Determine whether the arm exceeds the limit
void AP_ArmLocation_Excavator::update_boom_state()
{
    state.boom_out_range_up = false;
    state.boom_out_range_down = false;
    if(_angle_boom_to_slewing >= ARM_BOOM_DEGREE_MAX){
        _angle_boom_to_slewing = ARM_BOOM_DEGREE_MAX;
        state.boom_out_range_up = true;
    }else if(_angle_boom_to_slewing <= ARM_BOOM_DEGREE_MIN){
        _angle_boom_to_slewing = ARM_BOOM_DEGREE_MIN;
        state.boom_out_range_down = true;
    }else{
        return;
    }
}

void AP_ArmLocation_Excavator::update_forearm_state()
{
    state.forearm_out_range_up = false;
    state.forearm_out_range_down = false;
    if(_angle_forearm_to_boom >= ARM_FOREARM_DEGREE_MAX){
        _angle_forearm_to_boom = ARM_FOREARM_DEGREE_MAX;
        state.forearm_out_range_up = true;
    }else if(_angle_forearm_to_boom <= ARM_FOREARM_DEGREE_MIN){
        _angle_forearm_to_boom = ARM_FOREARM_DEGREE_MIN;
        state.forearm_out_range_down = true;
    }else{
        return;
    }
}
void AP_ArmLocation_Excavator::update_bucket_state()
{
    state.bucket_out_range_up = false;
    state.bucket_out_range_down = false;
    if(_angle_bucket_to_forearm <= ARM_BUCKET_DEGREE_MAX){
        _angle_bucket_to_forearm = ARM_BUCKET_DEGREE_MAX;
        state.bucket_out_range_up = true;
    }else if(_angle_bucket_to_forearm >= ARM_BUCKET_DEGREE_MIN){
        _angle_bucket_to_forearm = ARM_BUCKET_DEGREE_MIN;
        state.bucket_out_range_down = true;
    }else{
        return;
    }
}


//Calculate the angle of the boom, forearm, bucket relative to the fuselage
void AP_ArmLocation_Excavator::get_3angle_to_body(Vector3f euler_boom, Vector3f euler_forearm, Vector3f euler_bucket, float angle_phi)
{
    _angle_boom_to_body = radians(euler_boom.x) + _angle_boom_up;
    _angle_forearm_to_body = radians(euler_forearm.x);
    float angle_GNM = M_PI + _angle_forearm_to_body - _angle_GNF - radians(euler_bucket.x);
    float angle_QNM = M_2PI - _angle_FNQ -
        _angle_GNF - angle_GNM;
    float distance_QM = sqrt(_distance_QN * _distance_QN
        + _distance_MN * _distance_MN
        - 2 * _distance_QN * _distance_MN * cosf(angle_QNM));
    float angle_MQN = acosf((distance_QM * distance_QM
        + _distance_QN * _distance_QN
        - _distance_MN * _distance_MN) /
        (2 * distance_QM * _distance_QN));
    float angle_MQK = acosf((_distance_QK * _distance_QK
        + distance_QM * distance_QM
        - _distance_MK * _distance_MK) /
        (2 * _distance_QK * _distance_MK));
    float angle_DQV = M_2PI - _angle_NQF - angle_MQN
        - angle_MQK - _angle_KQV;
    _angle_bucket_to_body = _angle_forearm_to_body + angle_DQV - M_PI;
    if(_angle_bucket_to_body<(-2*M_PI)){
        _angle_bucket_to_body += M_PI;
    }
    if(_angle_bucket_to_body>2*M_PI){
        _angle_bucket_to_body -= M_PI;
    }
    _angle_slewing_to_body = euler_boom.z - angle_phi;
}
//Calculate the three-dimensional coordinates of the tooth tip relative to the body
void AP_ArmLocation_Excavator::cal_bucket_position_body()
{
        ///位置计算
        _position_tip_to_body.x = cosf(_angle_slewing_to_body) *
            (_distance_QV * cosf(_angle_bucket_to_body_safe) + _distance_QF * cosf(_angle_forearm_to_body_safe)
                + _distance_CF * cosf(_angle_boom_to_body_safe) + _distance_boom_to_slewing);
        _position_tip_to_body.y = sinf(_angle_slewing_to_body) *
            (_distance_QV * cosf(_angle_bucket_to_body_safe) + _distance_QF * cosf(_angle_forearm_to_body_safe)
                + _distance_CF * cosf(_angle_boom_to_body_safe) + _distance_boom_to_slewing);
        _position_tip_to_body.z = _distance_QV * sinf(_angle_bucket_to_body_safe) + _distance_QF * sinf(_angle_forearm_to_body_safe)
            + _distance_CF * sinf(_angle_boom_to_body_safe) + _distance_slewing_to_ground;
}

void AP_ArmLocation_Excavator::cal_cyl_length()
{
    float angle_ACB = _angle_TCA + _angle_BCF + _angle_boom_to_slewing;
    _cyl_boom_length = sqrt(_distance_AC * _distance_AC + _distance_BC * _distance_BC - 2*_distance_AC*_distance_BC*cos(angle_ACB));
    float phi = (M_PI - _angle_DFC -_angle_QFG -_angle_GFE - _angle_forearm_to_boom);
    _cyl_forearm_length = sqrt(_distance_DF * _distance_DF + _distance_EF*_distance_EF -2*_distance_EF*_distance_DF*cos(phi));
    float angle_KQN = M_PI - _angle_NQF -_angle_bucket_to_forearm -_angle_KQV;
    float distance_NK = sqrt(_distance_QN*_distance_QN + _distance_QK*_distance_QK
    - 2*_distance_QK*_distance_QN*cos(angle_KQN));
    float angle_MNK = acos((_distance_MK*_distance_MK + distance_NK*distance_NK - 
    _distance_MK*_distance_MK)/(2*_distance_MK*distance_NK));
    float angle_QNK = asin(_distance_QK*sin(angle_KQN)/distance_NK);
    float angle_GNM = M_2_PI - _angle_GNF - _angle_GNQ - angle_MNK - angle_QNK;
    _cyl_bucket_length = sqrt(_distance_GN*_distance_GN + _distance_MN*_distance_MN 
    - 2*_distance_MN*_distance_GN*cos(angle_GNM));
}

void AP_ArmLocation_Excavator::cal_angle_from_cyl()
{
    _angle_boom_to_slewing_target = acos((_distance_AC*_distance_AC + _distance_BC*_distance_BC - _cyl_boom_length_way*_cyl_boom_length_way)
    / (2 *_distance_AC *_distance_BC)) -_angle_BCF -_angle_TCA;
    _angle_forearm_to_boom_target = M_PI -_angle_DFC -_angle_QFG -_angle_GFE -
    acos((_distance_DF*_distance_DF + _distance_EF*_distance_EF - _cyl_forearm_length_way*_cyl_forearm_length_way)/(2*_distance_DF*_distance_DF*_distance_EF));
    float angle_GNM = acos((_distance_GN*_distance_GN + _distance_MN*_distance_MN - _cyl_bucket_length_way*_cyl_bucket_length_way)
    /(2*_distance_MN*_distance_GN));
    float angle_QNM = M_2_PI - _angle_FNQ - _angle_GNF - angle_GNM;
    float distance_QM = sqrt(_distance_QN*_distance_QN +_distance_MN*_distance_MN
    - 2*_distance_MN*_distance_QN*cos(angle_QNM));
    float angle_MQN = acos((_distance_QN*_distance_QN +distance_QM*distance_QM-_distance_MN*_distance_MN)
    /(2*distance_QM*_distance_QN));
    float angle_MQK = acos((_distance_QK*_distance_QK +distance_QM*distance_QM-_distance_MK*_distance_MK)
    /(2*_distance_QK*distance_QM));
    _angle_bucket_to_forearm_target = M_PI - _angle_NQF - angle_MQK - angle_MQN - _angle_KQV;
}

void AP_ArmLocation_Excavator::cal_angle_to_body_from_cyl()
{
    _angle_boom_body_sensor_target = _angle_boom_to_slewing_target -_angle_boom_up;
    _angle_forearm_body_sensor_target = _angle_forearm_to_boom_target + _angle_boom_to_slewing_target;
    _angle_bucket_body_sensor_target = M_PI_2 -acos((_cyl_bucket_length_way*_cyl_bucket_length_way + _distance_MN*_distance_MN
    - _distance_GN*_distance_GN)/(2*_cyl_bucket_length_way*_distance_MN));
}

AP_ArmLocation_Excavator *AP_ArmLocation_Excavator::_singleton;

namespace AP
{

AP_ArmLocation_Excavator *excavator_arm()
{
    return AP_ArmLocation_Excavator::get_singleton();
}

}
