#pragma once

#define ALLOW_DOUBLE_MATH_FUNCTIONS

#include <AP_Math/AP_Math.h>
#include <stdlib.h>
#include <cmath>

#define ARM_BOOM_DEGREE_MIN   -18*M_PI/180                //挖掘机大臂最小角度  radians
#define ARM_BOOM_DEGREE_MAX   42.6*M_PI/180               //挖掘机大臂最大角度  radians
#define ARM_FOREARM_DEGREE_MIN   -151.12*M_PI/180           //挖掘机斗杆最小角度  radians
#define ARM_FOREARM_DEGREE_MAX   -50.3*M_PI/180             //挖掘机斗杆最大角度  radians
#define ARM_BUCKET_DEGREE_MIN   23.35*M_PI/180            //挖掘机铲斗最小角度  radians
#define ARM_BUCKET_DEGREE_MAX   -157*M_PI/180             //挖掘机铲斗最大角度  radians

#define DISTANCE_BOOM_TO_SLEWING    45
#define DISTANCE_CF                 420.56
#define DISTANCE_QF                 199.55
#define DISTANCE_QV                 150.2
#define DISTANCE_SLEWING_TO_GROUND  120
#define ANGLE_BOOM_UP               24*M_PI/180
#define ANGLE_FNQ                   174*M_PI/180
#define ANGLE_GNF                   17*M_PI/180
#define ANGLE_NQF                   5*M_PI/180
#define ANGLE_KQV                   107*M_PI/180
#define DISTANCE_QN                 33
#define DISTANCE_QK                 43
#define DISTANCE_MK                 53.45
#define DISTANCE_MN                 53.45
#define ANGLE_BCF                   32*M_PI/180
#define ANGLE_TCA                   51*M_PI/180
#define DISTANCE_AC                 68.25
#define DISTANCE_BC                 203.6
#define DISTANCE_DF                 235.5
#define DISTANCE_EF                 71
#define ANGLE_DFC                   32*M_PI/180
#define ANGLE_QFG                   107*M_PI/180
#define ANGLE_GFE                   53*M_PI/180
#define DISTANCE_GN                 189
#define ANGLE_GNQ                   169*M_PI/180
class AP_ArmLocation_Excavator
{
public:
    AP_ArmLocation_Excavator() :
        _distance_boom_to_slewing(DISTANCE_BOOM_TO_SLEWING),
        _distance_CF(DISTANCE_CF),
        _distance_QF(DISTANCE_QF),
        _distance_QV(DISTANCE_QV),
        _distance_slewing_to_ground(DISTANCE_SLEWING_TO_GROUND),
        _angle_boom_up(ANGLE_BOOM_UP),
        _angle_FNQ(ANGLE_FNQ),
        _angle_GNF(ANGLE_GNF),
        _angle_NQF(ANGLE_NQF),
        _angle_KQV(ANGLE_KQV),
        _distance_QN(DISTANCE_QN),
        _distance_QK(DISTANCE_QK),
        _distance_MK(DISTANCE_MK),
        _distance_MN(DISTANCE_MN),
        _angle_BCF(ANGLE_BCF),
        _angle_TCA(ANGLE_TCA),
        _distance_AC(DISTANCE_AC),
        _distance_BC(DISTANCE_BC),
        _distance_DF(DISTANCE_DF),
        _distance_EF(DISTANCE_EF),
        _angle_DFC(ANGLE_DFC),
        _angle_QFG(ANGLE_QFG),
        _angle_GFE(ANGLE_GFE),
        _distance_GN(DISTANCE_GN),
        _angle_GNQ(ANGLE_GNQ)
    {
        _singleton = this;
    }
    
    /* Do not allow copies */
    AP_ArmLocation_Excavator(const AP_ArmLocation_Excavator &other) = delete;
    AP_ArmLocation_Excavator &operator=(const AP_ArmLocation_Excavator&) = delete;

    typedef enum{
        OC_STROKE_BOOM_MAX,
        OC_STROKE_BOOM_MIN,
        OC_STROKE_FOREARM_MAX,
        OC_STROKE_FOREARM_MIN,
        OC_STROKE_BUCKET_MAX,
        OC_STROKE_BUCKET_MIN
    }Oil_Cylinder_STROKE;

    //return tip to body position vector
    Vector3f bucket_position_body() const { return _position_tip_to_body; }
    Vector3f TBM_cutting_head_info() const { return _TBM_cutting_head; }
    //return tip position in ned to vector position_vec if location exist
    Vector3f bucket_position_ned() const { return _position_tip_to_ned; }

    //update main function
    void update_arm_position();
/*
Determine whether each cylinder is within the stroke range
*/
    bool check_if_outrange(AP_ArmLocation_Excavator::Oil_Cylinder_STROKE oil_cylinder_stroke)const;

    float get_boom_cyl_length() const {return _cyl_boom_length; }
    float get_forearm_cyl_length() const {return _cyl_forearm_length; }
    float get_bucket_cyl_length() const {return _cyl_bucket_length; }

    static AP_ArmLocation_Excavator *get_singleton(void)
    {
        return _singleton;
    }


private:


    void update_boom_state();
    void update_forearm_state();
    void update_bucket_state();

        //Calculate the three-dimensional coordinates of the tooth tip relative to the body
    void cal_bucket_position_body();
    void update_TBM_cutting_head();

    void cal_cyl_length();
    void cal_angle_from_cyl();
    void cal_angle_to_body_from_cyl();

    //Calculate the angle of the boom, forearm, bucket relative to the fuselage
    void get_3angle_to_body(Vector3f euler_boom, Vector3f euler_forearm, Vector3f euler_bucket, float angle_yaw);

    Vector3f  _position_tip_to_body; ///The position coordinates of the tooth tip relative to the car body

    Vector3f  _position_tip_to_ned;//The coordinates of the tooth tip in the global coordinate system

/*
The following specifies some parameters for calculating the position of the excavator working device, 
which are only relevant to the model of the excavator
*/
    float    _distance_boom_to_slewing;///大臂转动原点到回转台中心的距离
    float    _distance_CF;///斗杆转动原点到大臂转动原点的距离
    float    _distance_QF;//铲斗转动原点到斗杆转动原点的距离
    float    _distance_QV;///齿尖到铲斗转动原点的距离
    float    _distance_slewing_to_ground;///回转中心到地面的距离
    float    _angle_boom_up;///大臂三角形上小角角度
    float    _angle_FNQ;///斗杆旋转中心、活塞杆固定点以及铲斗旋转中心构成的夹角
    float    _angle_GNF;///斗杆活塞杆起始点、活塞杆固定点以及斗杆旋转中心构成的夹角
    float    _angle_NQF;///活塞杆固定点、铲斗旋转中心以及斗杆旋转中心构成的角度
    float    _angle_KQV; //铲斗与活塞杆的连接点、铲斗旋转中心以及铲斗齿尖构成角度
    float    _distance_QN;///铲斗旋转中心、活塞杆固定点间的距离
    float    _distance_QK;///铲斗旋转中心、铲斗与活塞杆的连接点间距离
    float    _distance_MK;///活塞杆顶点、铲斗与活塞杆的连接点间距离
    float    _distance_MN;///活塞杆顶点、活塞杆固定点间的距离
    /*not all parameter use to cal cyl'length include below,some also on top*/
    //boom cyl parameter
    float    _angle_BCF;
    float    _angle_TCA;
    float    _distance_AC;
    float    _distance_BC;
    //forearm cyl parameter
    float    _distance_DF;
    float    _distance_EF;
    float    _angle_DFC;
    float    _angle_QFG;
    float    _angle_GFE;
    //bucket cyl parameter
    float    _distance_GN;
    float    _angle_GNQ;

/*
Some intermediate variables that are required for calculations
*/
    float   _angle_boom_to_body;///The angle of the boom relative to the fuselage      //α1
    float   _angle_forearm_to_body;///The angle of the forearm relative to the fuselage    //α2
    float   _angle_bucket_to_body;///The angle of the bucket relative to the fuselage  //α3
    float   _angle_slewing_to_body;///The angle of the swing relative to the fuselage

    float   _angle_boom_to_body_safe;///The angle of the boom relative to the fuselage      //α1
    float   _angle_forearm_to_body_safe;///The angle of the forearm relative to the fuselage    //α2
    float   _angle_bucket_to_body_safe;///The angle of the bucket relative to the fuselage  //α3
    float   _angle_slewing_to_body_safe;///The angle of the swing relative to the fuselage

    float  _angle_boom_to_slewing;                  //θ1
    float  _angle_forearm_to_boom;                    //θ2
    float  _angle_bucket_to_forearm;                  //θ3

    //Converted into sensor data in the body's coordinate system
    Vector3f _euler_boom_e2b_from_sensor;
    Vector3f _euler_forearm_e2b_from_sensor;
    Vector3f _euler_bucket_e2b_from_sensor;

    Vector3f _body_position;
    Vector3f _TBM_cutting_head; //x:high,y:roll,z,yaw
    
    //now length for excavator's cylinder
    float _cyl_boom_length;
    float _cyl_forearm_length;
    float _cyl_bucket_length;

    //cyl length get from waypoint
    float _cyl_boom_length_way;
    float _cyl_forearm_length_way;
    float _cyl_bucket_length_way;

    //target 3 angle from cal cyl length
    float _angle_boom_to_slewing_target;
    float _angle_forearm_to_boom_target;
    float _angle_bucket_to_forearm_target;

    //target 3 sensor angle cal by cyl length
    float _angle_boom_body_sensor_target;
    float _angle_forearm_body_sensor_target;
    float _angle_bucket_body_sensor_target;

    //Structures related to overrun and location data
    struct State
    {
        bool boom_out_range_up;
        bool boom_out_range_down;
        bool forearm_out_range_up;
        bool forearm_out_range_down;
        bool bucket_out_range_up;
        bool bucket_out_range_down;
    }state;

    struct pos
    {
        bool max;
        bool min;
    };
    
    struct arm
    {
        struct pos boom;
        struct pos stick;
        struct pos bucket;
    };
    

    static AP_ArmLocation_Excavator *_singleton;  

};

namespace AP
{
AP_ArmLocation_Excavator *excavator_arm();
};