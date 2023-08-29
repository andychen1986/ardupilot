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

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include "AE_Motors.h"

extern const AP_HAL::HAL& hal;

// singleton instance
AE_Motors *AE_Motors::_singleton;

// parameters for the motor class 
// first part have 6 letters!!!!!!!!! can only add other 10
const AP_Param::GroupInfo AE_Motors::var_info[] = {
    // @Param: PWM_TYPE
    // @DisplayName: Motor Output PWM type
    // @Description: This selects the output PWM type as regular PWM, OneShot, Brushed motor support using PWM (duty cycle) with separated direction signal, Brushed motor support with separate throttle and direction PWM (duty cyle)
    // @Values: 0:Normal,1:OneShot,2:OneShot125,3:BrushedWithRelay,4:BrushedBiPolar,5:DShot150,6:DShot300,7:DShot600,8:DShot1200
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("PWM_TYPE", 1, AE_Motors, _pwm_type, PWM_TYPE_NORMAL),

    // @Param: PWM_FREQ
    // @DisplayName: Motor Output PWM freq for brushed motors
    // @Description: Motor Output PWM freq for brushed motors
    // @Units: kHz
    // @Range: 1 20
    // @Increment: 1
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("PWM_FREQ", 2, AE_Motors, _pwm_freq, 16),

    // @Param: SAFE_DISARM
    // @DisplayName: Motor PWM output disabled when disarmed
    // @Description: Disables motor PWM output when disarmed
    // @Values: 0:PWM enabled while disarmed, 1:PWM disabled while disarmed
    // @User: Advanced
    AP_GROUPINFO("SAFE_DIS", 3, AE_Motors, _disarm_disable_pwm, 0),

    // @Param: THST_EXPO
    // @DisplayName: Thrust Curve Expo
    // @Description: Thrust curve exponent (-1 to +1 with 0 being linear)
    // @Range: -1.0 1.0
    // @User: Advanced
    AP_GROUPINFO("THST_EXPO", 9, AE_Motors, _thrust_curve_expo, 0.0f),

    // @Param: OUT_MIN
    // @DisplayName: Output minimum
    // @Description: Output minimum percentage the autopilot will apply. This is useful for handling a deadzone around low throttle and for preventing internal combustion motors cutting out during missions.
    // @Units: %
    // @Range: 0 20
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("OUT_MIN", 10, AE_Motors, _output_min, 0),


    AP_GROUPEND
};


AE_Motors::AE_Motors(AP_ServoRelayEvents &relayEvents, AE_RobotArmInfo &rbt_arm_info) :
    _relayEvents(relayEvents),
    _rbt_arm_info(rbt_arm_info)
{
    AP_Param::setup_object_defaults(this, var_info);
    _singleton = this;
}

void AE_Motors::init(uint8_t frtype)
{
    _frame_type = AP_MotorsUGV::frame_type(frtype);

    // setup servo output
    setup_servo_output();

    // setup pwm type
    setup_pwm_type();

    // set safety output
    setup_safety_output();

}

// setup output in case of main CPU failure
void AE_Motors::setup_safety_output()
{
    if (_pwm_type == PWM_TYPE_BRUSHED_WITH_RELAY) {
        SRV_Channels::set_trim_to_min_for(SRV_Channel::k_boom, true);
        SRV_Channels::set_trim_to_min_for(SRV_Channel::k_forearm, true);
        SRV_Channels::set_trim_to_min_for(SRV_Channel::k_bucket, true);
        SRV_Channels::set_trim_to_min_for(SRV_Channel::k_rotation, true);
    }

    // stop sending pwm if main CPU fails
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_boom, SRV_Channel::Limit::ZERO_PWM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_forearm, SRV_Channel::Limit::ZERO_PWM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_bucket, SRV_Channel::Limit::ZERO_PWM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_rotation, SRV_Channel::Limit::ZERO_PWM);
}

// setup servo output ranges
void AE_Motors::setup_servo_output()
{

    //excavator boom/forearm/bucket as -1000 to 1000 values
    SRV_Channels::set_angle(SRV_Channel::k_boom,        100);
    SRV_Channels::set_angle(SRV_Channel::k_forearm,     100);
    SRV_Channels::set_angle(SRV_Channel::k_bucket,      100);
    SRV_Channels::set_angle(SRV_Channel::k_rotation,    100);
}

// set boom as a value from -100 to 100
void AE_Motors::set_boom(float boom)
{
    // only allow setting boom if armed
    if (!hal.util->get_soft_armed()) {
        return;
    }

    _boom = constrain_float(boom, -100.0f, 100.0f);
}

// set forearm as a value from -100 to 100
void AE_Motors::set_forearm(float forearm)
{
    // only allow setting forearm if armed
    if (!hal.util->get_soft_armed()) {
        return;
    }

    _forearm = constrain_float(forearm, -100.0f, 100.0f);
}

// set bucket as a value from -100 to 100
void AE_Motors::set_bucket(float bucket)
{
    // only allow setting bucket if armed
    if (!hal.util->get_soft_armed()) {
        return;
    }
    _bucket = constrain_float(bucket, -100.0f, 100.0f);
}

// set mast rotation input as a value from -100 to 100
void AE_Motors::set_rotation(float rotation)
{
    // only allow setting rotation if armed
    if (!hal.util->get_soft_armed()) {
        return;
    }
    _rotation = constrain_float(rotation, -100.0f, 100.0f);
}

// true if vehicle is capable of excavator
bool AE_Motors::have_excavator() const
{
    if (SRV_Channels::function_assigned(SRV_Channel::k_boom) &&
        SRV_Channels::function_assigned(SRV_Channel::k_forearm) &&
        SRV_Channels::function_assigned(SRV_Channel::k_bucket) &&
        SRV_Channels::function_assigned(SRV_Channel::k_rotation)) {
        return true;
    }
    return false;
}

// true if vehicle is capable of TBM
bool AE_Motors::have_TBM() const
{
    if (SRV_Channels::function_assigned(SRV_Channel::k_boom) &&
        SRV_Channels::function_assigned(SRV_Channel::k_rotation)) {
        return true;
    }
    return false;
}

void AE_Motors::output(bool armed,  float dt)
{
    // soft-armed overrides passed in armed status
    if (!hal.util->get_soft_armed()) {
        armed = false;
        _boom = 0.0f;
        _forearm = 0.0f;
        _bucket = 0.0f;
        _rotation = 0.0f;
    }

    // sanity check parameters
    sanity_check_parameters();

    if(have_excavator()){
        // output to excavator's boom , forearm , bucket and rotation channels
        output_excavator(armed, _boom, _forearm, _bucket, _rotation);
    }
    else if(have_TBM()){
        // output to excavator's boom , forearm , bucket and rotation channels
        output_TBM(armed, _boom, _rotation);
    }
}

//  returns true if checks pass, false if they fail.  report should be true to send text messages to GCS
bool AE_Motors::pre_arm_check(bool report) const
{
    if ((SRV_Channels::function_assigned(SRV_Channel::k_boom) ||
         SRV_Channels::function_assigned(SRV_Channel::k_forearm) ||
         SRV_Channels::function_assigned(SRV_Channel::k_bucket) ||
         SRV_Channels::function_assigned(SRV_Channel::k_rotation)) &&
        (SRV_Channels::function_assigned(SRV_Channel::k_motor1) ||
         SRV_Channels::function_assigned(SRV_Channel::k_motor2) ||
         SRV_Channels::function_assigned(SRV_Channel::k_motor3) ||
         SRV_Channels::function_assigned(SRV_Channel::k_motor4))) {
        if (report) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: AE_Construction AND omni configured");
        }
        return false;
    }
    return true;
}

// sanity check parameters
void AE_Motors::sanity_check_parameters()
{
    _output_min = constrain_int16(_output_min, 0, 20);
}

// setup pwm output type
void AE_Motors::setup_pwm_type()
{
    _motor_mask = 0;

    _motor_mask |= SRV_Channels::get_output_channel_mask(SRV_Channel::k_boom);
    _motor_mask |= SRV_Channels::get_output_channel_mask(SRV_Channel::k_forearm);
    _motor_mask |= SRV_Channels::get_output_channel_mask(SRV_Channel::k_bucket);
    _motor_mask |= SRV_Channels::get_output_channel_mask(SRV_Channel::k_rotation);

    switch (_pwm_type) {
    case PWM_TYPE_ONESHOT:
        hal.rcout->set_output_mode(_motor_mask, AP_HAL::RCOutput::MODE_PWM_ONESHOT);
        break;
    case PWM_TYPE_ONESHOT125:
        hal.rcout->set_output_mode(_motor_mask, AP_HAL::RCOutput::MODE_PWM_ONESHOT125);
        break;
    case PWM_TYPE_BRUSHED_WITH_RELAY:
    case PWM_TYPE_BRUSHED_BIPOLAR:
        hal.rcout->set_output_mode(_motor_mask, AP_HAL::RCOutput::MODE_PWM_BRUSHED);
        hal.rcout->set_freq(_motor_mask, uint16_t(_pwm_freq * 1000));
        break;
    case PWM_TYPE_DSHOT150:
        hal.rcout->set_output_mode(_motor_mask, AP_HAL::RCOutput::MODE_PWM_DSHOT150);
        break;
    case PWM_TYPE_DSHOT300:
        hal.rcout->set_output_mode(_motor_mask, AP_HAL::RCOutput::MODE_PWM_DSHOT300);
        break;
    case PWM_TYPE_DSHOT600:
        hal.rcout->set_output_mode(_motor_mask, AP_HAL::RCOutput::MODE_PWM_DSHOT600);
        break;
    case PWM_TYPE_DSHOT1200:
        hal.rcout->set_output_mode(_motor_mask, AP_HAL::RCOutput::MODE_PWM_DSHOT1200);
        break;
    default:
        // do nothing
        break;
    }
}

// output to excavator's boom , forearm and bucket channels
void AE_Motors::output_excavator(bool armed, float boom, float forearm, float bucket, float rotation)
{
    if (!have_excavator()) {
        return;
    }

    // handle simpler disarmed case
    if (!armed) {
        if (_disarm_disable_pwm) {
            SRV_Channels::set_output_limit(SRV_Channel::k_boom, SRV_Channel::Limit::ZERO_PWM);
            SRV_Channels::set_output_limit(SRV_Channel::k_forearm, SRV_Channel::Limit::ZERO_PWM);
            SRV_Channels::set_output_limit(SRV_Channel::k_bucket, SRV_Channel::Limit::ZERO_PWM);
            SRV_Channels::set_output_limit(SRV_Channel::k_rotation, SRV_Channel::Limit::ZERO_PWM);
        } else {
            SRV_Channels::set_output_limit(SRV_Channel::k_boom, SRV_Channel::Limit::TRIM);
            SRV_Channels::set_output_limit(SRV_Channel::k_forearm, SRV_Channel::Limit::TRIM);
            SRV_Channels::set_output_limit(SRV_Channel::k_bucket, SRV_Channel::Limit::TRIM);
            SRV_Channels::set_output_limit(SRV_Channel::k_rotation, SRV_Channel::Limit::TRIM);
        }
        return;
    }

    arm_output(SRV_Channel::k_boom, boom);
    arm_output(SRV_Channel::k_forearm, forearm);
    arm_output(SRV_Channel::k_bucket, bucket);
    arm_output(SRV_Channel::k_rotation, rotation);

}

// output to excavator's boom , forearm and bucket channels
void AE_Motors::output_TBM(bool armed, float boom, float rotation)
{
    if (!have_TBM()) {
        return;
    }

    // handle simpler disarmed case
    if (!armed) {
        if (_disarm_disable_pwm) {
            SRV_Channels::set_output_limit(SRV_Channel::k_boom, SRV_Channel::Limit::ZERO_PWM);
            SRV_Channels::set_output_limit(SRV_Channel::k_rotation, SRV_Channel::Limit::ZERO_PWM);
        } else {
            SRV_Channels::set_output_limit(SRV_Channel::k_boom, SRV_Channel::Limit::TRIM);
            SRV_Channels::set_output_limit(SRV_Channel::k_rotation, SRV_Channel::Limit::TRIM);
        }
        return;
    }

    arm_output(SRV_Channel::k_boom, boom);
    arm_output(SRV_Channel::k_rotation, rotation);

}

// arm_output (-100 ~ +100) to a arm channel.  Sets relays if required dt is the main loop time interval and is required when rate control is required
void AE_Motors::arm_output(SRV_Channel::Aux_servo_function_t function, float arm_output, float dt)
{
    // sanity check servo function
    if (function != SRV_Channel::k_boom && function != SRV_Channel::k_forearm && function != SRV_Channel::k_bucket && function!= SRV_Channel::k_rotation) {
        return;
    }

    // constrain and scale output
    arm_output = get_scaled_arm_output(arm_output);

    // apply rate control
    arm_output = get_rate_controlled_throttle(function, arm_output, dt);

    // set relay if necessary
    if (_pwm_type == PWM_TYPE_BRUSHED_WITH_RELAY) {
        // find the output channel, if not found return
        const SRV_Channel *out_chan = SRV_Channels::get_channel_for(function);
        if (out_chan == nullptr) {
            return;
        }
        const int8_t reverse_multiplier = out_chan->get_reversed() ? -1 : 1;
        bool relay_high = is_negative(reverse_multiplier * arm_output);

        switch (function) {
        case SRV_Channel::k_boom:
            _relayEvents.do_set_relay(2, relay_high);
            break;
        case SRV_Channel::k_forearm:
            _relayEvents.do_set_relay(3, relay_high);
            break;
        case SRV_Channel::k_bucket:
            _relayEvents.do_set_relay(4, relay_high);
            break;
        case SRV_Channel::k_rotation:
            _relayEvents.do_set_relay(5, relay_high);
            break;
        default:
            // do nothing
            break;
        }
        // invert the output to always have positive value calculated by calc_pwm
        arm_output = reverse_multiplier * fabsf(arm_output);
    }

    arm_output = prevent_exceeding_position(function, arm_output);

    // output to servo channel
    SRV_Channels::set_output_scaled(function,  arm_output);
}

// prevent the boom from exceeding the limit position
float AE_Motors::prevent_exceeding_position(SRV_Channel::Aux_servo_function_t function, float arm_output)
{
    if (function == SRV_Channel::k_rotation){
        return arm_output;
    }

    int8_t joint = (int8_t)(function - SRV_Channel::k_boom);
    int8_t status = _rbt_arm_info.get_cylinder_length_state(joint);

    switch (status) {
        case AE_RobotArmInfo::Robot_Arm_Safe_State::SAFETY:
            break;
        
        case AE_RobotArmInfo::Robot_Arm_Safe_State::DOWN_ALERT:
            arm_output = constrain_float(arm_output, -100.0f, 0.0f);
            break;
        
        case AE_RobotArmInfo::Robot_Arm_Safe_State::UP_ALERT:
            arm_output = constrain_float(arm_output, 0.0f, 100.0f);
            break;

        case AE_RobotArmInfo::Robot_Arm_Safe_State::EMERG:
            return 0.0;
    }
        
    return arm_output;
}

// scale an arm output using the _thrust_curve_expo parameter.  arm output should be in the range -100 to +100
float AE_Motors::get_scaled_arm_output(float arm_output) const
{
    // exit immediately if arm_output is zero
    if (is_zero(arm_output)) {
        return arm_output;
    }

    // scale using output_min
    if (_output_min > 0) {
        if (is_negative(arm_output)) {
            arm_output = -_output_min + (arm_output * ((100.0f - _output_min) / 100.0f));
        } else {
            arm_output = _output_min + (arm_output * ((100.0f - _output_min) / 100.0f));
        }
    }

    // skip further scaling if thrust curve disabled or invalid
    if (is_zero(_thrust_curve_expo) || (_thrust_curve_expo > 1.0f) || (_thrust_curve_expo < -1.0f)) {
        return arm_output;
    }

    // calculate scaler
    const float sign = (arm_output < 0.0f) ? -1.0f : 1.0f;
    const float arm_output_pct = constrain_float(arm_output, -100.0f, 100.0f) / 100.0f;
    return 100.0f * sign * ((_thrust_curve_expo - 1.0f) + safe_sqrt((1.0f - _thrust_curve_expo) * (1.0f - _thrust_curve_expo) + 4.0f * _thrust_curve_expo * fabsf(arm_output_pct))) / (2.0f * _thrust_curve_expo);
}

// use rate controller to achieve desired throttle
float AE_Motors::get_rate_controlled_throttle(SRV_Channel::Aux_servo_function_t function, float throttle, float dt)
{
    // require non-zero dt
    if (!is_positive(dt)) {
        return throttle;
    }

    // attempt to rate control boom throttle
    if ((function == SRV_Channel::k_boom) /* && _rate_controller.enabled(0)*/) {
        //return _rate_controller.get_rate_controlled_throttle(0, throttle, dt);
    }

    // rate control forearm throttle
    if ((function == SRV_Channel::k_forearm) /*&& _rate_controller.enabled(1)*/) {
        //return _rate_controller.get_rate_controlled_throttle(1, throttle, dt);
    }

    // rate control bucket throttle
    if ((function == SRV_Channel::k_bucket) /*&& _rate_controller.enabled(2)*/) {
        //return _rate_controller.get_rate_controlled_throttle(1, throttle, dt);
    }

    // rate control rotation throttle
    if ((function == SRV_Channel::k_rotation) /*&& _rate_controller.enabled(2)*/) {
        //return _rate_controller.get_rate_controlled_throttle(1, throttle, dt);
    }

    // return throttle unchanged
    return throttle;
}

// return true if motors are moving
bool AE_Motors::active() const
{
    // if soft disarmed, motors not active
    if (!hal.util->get_soft_armed()) {
        return false;
    }

    // check throttle is active
    if (!is_zero(get_boom())||!is_zero(get_forearm())||!is_zero(get_bucket())||!is_zero(get_rotation())) {
        return true;
    }

    return false;
}

