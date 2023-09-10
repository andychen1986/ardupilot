#pragma once
/*
This library does not support any ESCs and motors other than normal_pwm signals!!!
*/
//#include <AP_Arming/AP_Arming.h>
#include <AP_ServoRelayEvents/AP_ServoRelayEvents.h>
#include <AR_Motors/AP_MotorsUGV.h>
#include <AE_RobotArmInfo/AE_RobotArmInfo_Excavator.h>

class AE_Motors {
public:
    // Constructor
    AE_Motors(AE_RobotArmInfo &rbt_arm_info);

    // singleton support
    static AE_Motors   *get_singleton(void) { return _singleton; }
    
    // initialise motors
    void init(uint8_t contype);

    // return true if motors are active
    bool active() const;

    // setup output in case of main CPU failure
    void setup_safety_output();

    // setup servo output ranges
    void setup_servo_output();

    // get or set boom as a value from -100 to 100
    float get_boom() const { return _boom; }
    void set_boom(float boom);

    // get or set forearm as a value from -100 to 100
    float get_forearm() const { return _forearm; }
    void set_forearm(float forearm);

    // get or set bucket as a value from -100 to 100
    float get_bucket() const { return _bucket; }
    void set_bucket(float bucket);

    // set or get mast rotation input as a value from -100 to 100
    void set_rotation(float _rotation);
    float get_rotation() const { return _rotation; }

    // set or get cutting_header input as a value from -100 to 100
    void set_cutting_header(float _cutting_header);
    float get_cutting_header() const { return _cutting_header; }

    // set or get mast rotation input as a value from -100 to 100
    void set_support_leg(float _support_leg);
    float get_support_leg() const { return _support_leg; }

    // true if vehicle is capable of excavator
    bool have_excavator() const;

    // true if vehicle is capable of TBM
    bool have_TBM() const;

    // output to motors and steering servos
    // ground_speed should be the vehicle's speed over the surface in m/s
    // dt should be expected time between calls to this function
    void output(bool armed, float dt);

    //  returns true if checks pass, false if they fail.  display_failure argument should be true to send text messages to GCS
    bool pre_arm_check(bool report) const;

    // return the motor mask
    uint16_t get_motor_mask() const { return _motor_mask; }

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

    // return the motor mask
    uint8_t get_construction_type() const{return _con_type;};

    //construction type
    enum CON_TYPE{
        UNDEFINED   = 0,
        EXCAVATOR   = 1,
        TBM         = 2
    };

private:

    // sanity check parameters
    void sanity_check_parameters();

    // output to excavator's boom , forearm , bucket rotation channels
    void output_excavator(bool armed, float boom, float forearm, float bucket, float rotation);

    // output to excavator's boom , forearm and bucket channels
    void output_TBM(bool armed, float boom, float rotation, float cutting_header, float support_leg);

    // arm_output (-100 ~ +100) to a arm channel.  Sets relays if required
    // dt is the main loop time interval and is required when rate control is required
    void arm_output(SRV_Channel::Aux_servo_function_t function, float arm_output, float dt = 0.0f);

    // prevent the boom from exceeding the limit position
    float prevent_exceeding_position(SRV_Channel::Aux_servo_function_t function, float arm_output);

    // scale an arm output using the _thrust_curve_expo parameter.  arm output should be in the range -100 to +100
    float get_scaled_arm_output(float arm_output) const;

    // use rate controller to achieve desired throttle
    float get_rate_controlled_throttle(SRV_Channel::Aux_servo_function_t function, float throttle, float dt); 

    // external references
    AE_RobotArmInfo &_rbt_arm_info;

    // parameters
    AP_Int8 _disarm_disable_pwm;    // disable PWM output while disarmed
    AP_Float _thrust_curve_expo; // thrust curve exponent from -1 to +1 with 0 being linear

    // internal variables
    float   _boom;  // requested boom as a value from -100 to 100
    float   _forearm;   // requested forearm as a value from -100 to 100
    float   _bucket;    // requested bucket as a value from -100 to 100
    float   _throttle_prev; // throttle input from previous iteration
    float   _rotation;  // requested rotation input as a value from -100 to 100
    float   _support_leg;  // requested support_leg input as a value from -100 to 100
    float   _cutting_header;  // requested cutting_header input as a value from -100 to 100
    AP_Int8 _output_min; // throttle minimum percentage
    uint16_t _motor_mask;   // mask of motors configured with pwm_type
    CON_TYPE _con_type; // frame type requested at initialisation

    static AE_Motors *_singleton;
};
