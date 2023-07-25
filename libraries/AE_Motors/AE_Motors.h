#pragma once

//#include <AP_Arming/AP_Arming.h>
#include <AP_ServoRelayEvents/AP_ServoRelayEvents.h>

class AE_Motors {
public:
    // Constructor
    AE_Motors(AP_ServoRelayEvents &relayEvents);

    // singleton support
    static AE_Motors   *get_singleton(void) { return _singleton; }

    // supported omni motor configurations
    enum AE_type {
        FRAME_TYPE_UNDEFINED = 0,
        FRAME_TYPE_EXCAVATOR = 1,
        FRAME_TYPE_TBM = 2,
    };
    
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

    // structure for holding motor limit flags
    struct AP_MotorsUGV_limit {
        uint8_t boom_lower  : 1; // we have reached boom's lower limit
        uint8_t boom_upper  : 1; // we have reached boom's upper limit
        uint8_t forearm_lower  : 1; // we have reached forearm's lower limit
        uint8_t forearm_upper  : 1; // we have reached forearm's upper limit
        uint8_t bucket_lower  : 1; // we have reached bucket's lower limit
        uint8_t bucket_upper  : 1; // we have reached bucket's upper limit
    } limit;

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

private:

    enum pwm_type {
        PWM_TYPE_NORMAL = 0,
        PWM_TYPE_ONESHOT = 1,
        PWM_TYPE_ONESHOT125 = 2,
        PWM_TYPE_BRUSHED_WITH_RELAY = 3,
        PWM_TYPE_BRUSHED_BIPOLAR = 4,
        PWM_TYPE_DSHOT150 = 5,
        PWM_TYPE_DSHOT300 = 6,
        PWM_TYPE_DSHOT600 = 7,
        PWM_TYPE_DSHOT1200 = 8
    };

    // sanity check parameters
    void sanity_check_parameters();

    // setup pwm output type
    void setup_pwm_type();

    // output to excavator's boom , forearm , bucket rotation channels
    void output_excavator(bool armed, float boom, float forearm, float bucket, float rotation);

    // output to excavator's boom , forearm and bucket channels
    void output_TBM(bool armed, float boom, float rotation);

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
    AP_ServoRelayEvents &_relayEvents;

    // parameters
    AP_Int8 _pwm_type;  // PWM output type
    AP_Int8 _pwm_freq;  // PWM output freq for brushed motors
    AP_Int8 _disarm_disable_pwm;    // disable PWM output while disarmed
    AP_Float _thrust_curve_expo; // thrust curve exponent from -1 to +1 with 0 being linear

    // internal variables
    float   _boom;  // requested boom as a value from -100 to 100
    float   _forearm;   // requested forearm as a value from -100 to 100
    float   _bucket;    // requested bucket as a value from -100 to 100
    float   _throttle_prev; // throttle input from previous iteration
    float   _rotation;  // requested rotation input as a value in the range +- 100
    uint16_t _motor_mask;   // mask of motors configured with pwm_type
    AE_type _AE_type; // frame type requested at initialisation

    static AE_Motors *_singleton;
};
