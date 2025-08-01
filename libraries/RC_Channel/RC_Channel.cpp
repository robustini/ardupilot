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

/*
 *       RC_Channel.cpp - class for one RC channel input
 */

#include "RC_Channel_config.h"

#if AP_RC_CHANNEL_ENABLED

#include <stdlib.h>
#include <cmath>

#include <AP_HAL/AP_HAL.h>
extern const AP_HAL::HAL& hal;

#include <AP_Math/AP_Math.h>

#include "RC_Channel.h"
#include <GCS_MAVLink/GCS.h>

#include <AC_Avoidance/AC_Avoid.h>
#include <AC_Sprayer/AC_Sprayer.h>
#include <AP_Camera/AP_Camera.h>
#include <AP_Camera/AP_RunCam.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Generator/AP_Generator.h>
#include <AP_Gripper/AP_Gripper.h>
#include <AP_GyroFFT/AP_GyroFFT.h>
#include <AP_ADSB/AP_ADSB.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_LandingGear/AP_LandingGear.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_ServoRelayEvents/AP_ServoRelayEvents.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Arming/AP_Arming.h>
#include <AP_Avoidance/AP_Avoidance.h>
#include <AP_GPS/AP_GPS.h>
#include <AC_Fence/AC_Fence.h>
#include <AP_OpticalFlow/AP_OpticalFlow.h>
#include <AP_VisualOdom/AP_VisualOdom.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Mount/AP_Mount.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_VideoTX/AP_VideoTX.h>
#include <AP_Torqeedo/AP_Torqeedo.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_Parachute/AP_Parachute_config.h>
#include <AP_Scripting/AP_Scripting.h>
#define SWITCH_DEBOUNCE_TIME_MS  200

const AP_Param::GroupInfo RC_Channel::var_info[] = {
    // @Param: MIN
    // @DisplayName: RC min PWM
    // @Description: RC minimum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
    // @Units: PWM
    // @Range: 800 2200
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MIN",  1, RC_Channel, radio_min, 1100),

    // @Param: TRIM
    // @DisplayName: RC trim PWM
    // @Description: RC trim (neutral) PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
    // @Units: PWM
    // @Range: 800 2200
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("TRIM", 2, RC_Channel, radio_trim, 1500),

    // @Param: MAX
    // @DisplayName: RC max PWM
    // @Description: RC maximum PWM pulse width in microseconds. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
    // @Units: PWM
    // @Range: 800 2200
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MAX",  3, RC_Channel, radio_max, 1900),

    // @Param: REVERSED
    // @DisplayName: RC reversed
    // @Description: Reverse channel input. Set to 0 for normal operation. Set to 1 to reverse this input channel.
    // @Values: 0:Normal,1:Reversed
    // @User: Advanced
    AP_GROUPINFO("REVERSED",  4, RC_Channel, reversed, 0),

    // @Param: DZ
    // @DisplayName: RC dead-zone
    // @Description: PWM dead zone in microseconds around trim or bottom
    // @Units: PWM
    // @Range: 0 200
    // @User: Advanced
    AP_GROUPINFO("DZ",   5, RC_Channel, dead_zone, 0),

    // @Param: OPTION
    // @DisplayName: RC input option
    // @Description: Function assigned to this RC channel
    // @SortValues: AlphabeticalZeroAtTop
    // @Values{Copter, Rover, Plane, Blimp, Sub}: 0:Do Nothing
    // @Values{Copter}: 2:FLIP Mode
    // @Values{Copter}: 3:Simple Mode
    // @Values{Copter, Rover, Plane}: 4:RTL
    // @Values{Copter}: 5:Save Trim
    // @Values{Rover}: 5:Save Trim (4.1 and lower)
    // @Values{Copter, Rover}: 7:Save WP
    // @Values{Copter, Rover, Plane, Sub}: 9:Camera Trigger
    // @Values{Copter}: 10:RangeFinder Enable
    // @Values{Copter, Rover, Plane, Sub}: 11:Fence Enable
    // @Values{Copter}: 13:Super Simple Mode
    // @Values{Copter}: 14:Acro Trainer
    // @Values{Copter}: 15:Sprayer Enable
    // @Values{Copter, Rover, Plane}: 16:AUTO Mode
    // @Values{Copter}: 17:AUTOTUNE Mode
    // @Values{Copter, Blimp}: 18:LAND Mode
    // @Values{Copter, Rover}: 19:Gripper Release
    // @Values{Copter}: 21:Parachute Enable
    // @Values{Copter, Plane}: 22:Parachute Release
    // @Values{Copter}: 23:Parachute 3pos
    // @Values{Copter, Rover, Plane, Sub}: 24:Auto Mission Reset
    // @Values{Copter}: 25:AttCon Feed Forward
    // @Values{Copter}: 26:AttCon Accel Limits
    // @Values{Copter, Rover, Plane, Sub}: 27:Retract Mount1
    // @Values{Copter, Rover, Plane, Sub}: 28:Relay1 On/Off
    // @Values{Copter, Plane}: 29:Landing Gear
    // @Values{Copter}: 30:Lost Copter Sound
    // @Values{Rover}: 30:Lost Rover Sound
    // @Values{Plane}: 30:Lost Plane Sound
    // @Values{Copter, Rover, Plane, Sub}: 31:Motor Emergency Stop
    // @Values{Copter}: 32:Motor Interlock
    // @Values{Copter}: 33:BRAKE Mode
    // @Values{Copter, Rover, Plane, Sub}: 34:Relay2 On/Off, 35:Relay3 On/Off, 36:Relay4 On/Off
    // @Values{Copter}: 37:THROW Mode
    // @Values{Copter, Plane}: 38:ADSB Avoidance Enable
    // @Values{Copter}: 39:PrecLoiter Enable
    // @Values{Copter, Rover}: 40:Proximity Avoidance Enable
    // @Values{Copter, Rover, Plane}: 41:ArmDisarm (4.1 and lower)
    // @Values{Copter, Rover}: 42:SMARTRTL Mode
    // @Values{Copter, Plane}: 43:InvertedFlight Enable
    // @Values{Copter}: 44:Winch Enable, 45:Winch Control
    // @Values{Copter, Rover, Plane, Blimp, Sub}: 46:RC Override Enable
    // @Values{Copter}: 47:User Function 1, 48:User Function 2, 49:User Function 3
    // @Values{Rover}: 50:LearnCruise Speed
    // @Values{Rover, Plane}: 51:MANUAL Mode
    // @Values{Copter, Rover, Plane}: 52:ACRO Mode
    // @Values{Rover}: 53:STEERING Mode
    // @Values{Rover}: 54:HOLD Mode
    // @Values{Copter, Rover, Plane}: 55:GUIDED Mode
    // @Values{Copter, Rover, Plane}: 56:LOITER Mode
    // @Values{Copter, Rover}: 57:FOLLOW Mode
    // @Values{Copter, Rover, Plane, Sub}: 58:Clear Waypoints
    // @Values{Rover}: 59:Simple Mode
    // @Values{Copter}: 60:ZigZag Mode
    // @Values{Copter}: 61:ZigZag SaveWP
    // @Values{Copter, Rover, Plane, Sub}: 62:Compass Learn
    // @Values{Rover}: 63:Sailboat Tack
    // @Values{Plane}: 64:Reverse Throttle
    // @Values{Copter, Rover, Plane, Blimp, Sub}: 65:GPS Disable
    // @Values{Copter, Rover, Plane, Sub}: 66:Relay5 On/Off, 67:Relay6 On/Off
    // @Values{Copter}: 68:STABILIZE Mode
    // @Values{Copter}: 69:POSHOLD Mode
    // @Values{Copter}: 70:ALTHOLD Mode
    // @Values{Copter}: 71:FLOWHOLD Mode
    // @Values{Copter,Rover,Plane}: 72:CIRCLE Mode
    // @Values{Copter}: 73:DRIFT Mode
    // @Values{Rover}: 74:Sailboat motoring 3pos
    // @Values{Copter}: 75:SurfaceTrackingUpDown
    // @Values{Copter}: 76:STANDBY Mode
    // @Values{Plane}: 77:TAKEOFF Mode
    // @Values{Copter, Rover, Plane, Sub}: 78:RunCam Control
    // @Values{Copter, Rover, Plane, Sub}: 79:RunCam OSD Control
    // @Values{Copter}: 80:VisOdom Align
    // @Values{Rover}: 80:VisoOdom Align
    // @Values{Copter, Rover, Plane, Blimp, Sub}: 81:Disarm
    // @Values{Plane}: 82:QAssist 3pos
    // @Values{Copter}: 83:ZigZag Auto
    // @Values{Copter, Plane}: 84:AirMode
    // @Values{Copter, Plane}: 85:Generator
    // @Values{Plane}: 86:Non Auto Terrain Follow Disable
    // @Values{Plane}: 87:Crow Select
    // @Values{Plane}: 88:Soaring Enable
    // @Values{Plane}: 89:Landing Flare
    // @Values{Copter, Rover, Plane, Blimp, Sub}: 90:EKF Source Set
    // @Values{Plane}: 91:Airspeed Ratio Calibration
    // @Values{Plane}: 92:FBWA Mode
    // @Values{Copter, Rover, Plane, Sub}: 94:VTX Power
    // @Values{Plane}: 95:FBWA taildragger takeoff mode
    // @Values{Plane}: 96:Trigger re-reading of mode switch
    // @Values{Rover}: 97:Windvane home heading direction offset
    // @Values{Plane}: 98:TRAINING Mode
    // @Values{Copter}: 99:AUTO RTL
    // @Values{Copter, Rover, Plane, Blimp, Sub}: 100:KillIMU1, 101:KillIMU2
    // @Values{Copter, Rover, Plane, Sub}: 102:Camera Mode Toggle
    // @Values{Copter, Rover, Plane, Blimp, Sub, Tracker}: 103: EKF lane switch attempt
    // @Values{Copter, Rover, Plane, Blimp, Sub, Tracker}: 104: EKF yaw reset
    // @Values{Copter, Rover, Plane, Sub}: 105:GPS Disable Yaw
    // @Values{Rover, Plane}: 106:Disable Airspeed Use
    // @Values{Plane}: 107:Enable FW Autotune
    // @Values{Plane}: 108:QRTL Mode
    // @Values{Copter}: 109:use Custom Controller
    // @Values{Copter, Rover, Plane, Blimp, Sub}:  110:KillIMU3
    // @Values{Copter, Rover, Plane, Blimp, Sub}:  111:Loweheiser starter
    // @Values{Copter,Plane,Rover,Blimp,Sub,Tracker}: 112:SwitchExternalAHRS
    // @Values{Copter, Rover, Plane, Sub}: 113:Retract Mount2
    // @Values{Plane}: 150:CRUISE Mode
    // @Values{Copter}: 151:TURTLE Mode
    // @Values{Copter}: 152:SIMPLE heading reset
    // @Values{Copter, Rover, Plane, Sub}: 153:ArmDisarm (4.2 and higher)
    // @Values{Blimp}: 153:ArmDisarm
    // @Values{Copter}: 154:ArmDisarm with AirMode  (4.2 and higher)
    // @Values{Plane}: 154:ArmDisarm with Quadplane AirMode (4.2 and higher)
    // @Values{Rover}: 155:Set steering trim to current servo and RC
    // @Values{Plane}: 155:Set roll pitch and yaw trim to current servo and RC
    // @Values{Rover}: 156:Torqeedo Clear Err
    // @Values{Plane}: 157:Force FS Action to FBWA
    // @Values{Copter, Plane}: 158:Optflow Calibration
    // @Values{Copter}: 159:Force IS_Flying
    // @Values{Plane}: 160:Weathervane Enable
    // @Values{Copter}: 161:Turbine Start(heli)
    // @Values{Copter, Rover, Plane}: 162:FFT Tune
    // @Values{Copter, Rover, Plane, Sub}: 163:Mount Lock
    // @Values{Copter, Rover, Plane, Blimp, Sub}: 164:Pause Stream Logging
    // @Values{Copter, Rover, Plane, Sub}: 165:Arm/Emergency Motor Stop
    // @Values{Copter, Rover, Plane, Blimp, Sub}: 166:Camera Record Video, 167:Camera Zoom, 168:Camera Manual Focus, 169:Camera Auto Focus
    // @Values{Plane}: 170:QSTABILIZE Mode
    // @Values{Copter, Rover, Plane, Blimp}: 171:Calibrate Compasses
    // @Values{Copter, Rover, Plane, Blimp}: 172:Battery MPPT Enable
    // @Values{Plane}: 173:Plane AUTO Mode Landing Abort
    // @Values{Copter, Rover, Plane, Blimp, Sub}: 174:Camera Image Tracking
    // @Values{Copter, Rover, Plane, Blimp, Sub}: 175:Camera Lens
    // @Values{Plane}: 176:Quadplane Fwd Throttle Override enable
    // @Values{Copter, Rover, Plane, Blimp, Sub}: 177:Mount LRF enable
    // @Values{Copter}: 178:FlightMode Pause/Resume
    // @Values{Plane}: 179:ICEngine start / stop
    // @Values{Copter, Plane}: 180:Test autotuned gains after tune is complete
    // @Values{Plane}: 181: QuickTune
    // @Values{Copter}: 182: AHRS AutoTrim
    // @Values{Plane}: 183: AUTOLAND mode
    // @Values{Plane}: 184: System ID Chirp (Quadplane only)
    // @Values{Rover}: 201:Roll
    // @Values{Rover}: 202:Pitch
    // @Values{Rover}: 207:MainSail
    // @Values{Rover, Plane}:  208:Flap
    // @Values{Plane}: 209:VTOL Forward Throttle
    // @Values{Plane}: 210:Airbrakes
    // @Values{Rover}: 211:Walking Height
    // @Values{Copter, Rover, Plane, Sub}: 212:Mount1 Roll, 213:Mount1 Pitch, 214:Mount1 Yaw, 215:Mount2 Roll, 216:Mount2 Pitch, 217:Mount2 Yaw
    // @Values{Copter, Rover, Plane, Blimp, Sub}:  218:Loweheiser throttle
    // @Values{Copter}: 219:Transmitter Tuning
    // @Values{All-Vehicles}: 300:Scripting1, 301:Scripting2, 302:Scripting3, 303:Scripting4, 304:Scripting5, 305:Scripting6, 306:Scripting7, 307:Scripting8, 308:Scripting9, 309:Scripting10, 310:Scripting11, 311:Scripting12, 312:Scripting13, 313:Scripting14, 314:Scripting15, 315:Scripting16
    // @Values{All-Vehicles}: 316:Stop-Restart Scripting
    // @User: Standard
    AP_GROUPINFO("OPTION",  6, RC_Channel, option, 0),

    AP_GROUPEND
};


// constructor
RC_Channel::RC_Channel(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void RC_Channel::set_range(uint16_t high)
{
    type_in = ControlType::RANGE;
    high_in = high;
}

void RC_Channel::set_angle(uint16_t angle)
{
    type_in = ControlType::ANGLE;
    high_in = angle;
}

void RC_Channel::set_default_dead_zone(int16_t dzone)
{
    dead_zone.set_default(abs(dzone));
}

bool RC_Channel::get_reverse(void) const
{
    return bool(reversed.get());
}

// read input from hal.rcin or overrides
bool RC_Channel::update(void)
{
    if (has_override() && !rc().option_is_enabled(RC_Channels::Option::IGNORE_OVERRIDES)) {
        radio_in = override_value;
    } else if (rc().has_had_rc_receiver() && !rc().option_is_enabled(RC_Channels::Option::IGNORE_RECEIVER)) {
        radio_in = hal.rcin->read(ch_in);
    } else {
        return false;
    }

    if (type_in == ControlType::RANGE) {
        control_in = pwm_to_range();
    } else {
        // ControlType::ANGLE
        control_in = pwm_to_angle();
    }

    return true;
}

/*
  return the center stick position expressed as a control_in value
  used for thr_mid in copter
 */
int16_t RC_Channel::get_control_mid() const
{
    if (type_in == ControlType::RANGE) {
        int16_t r_in = (radio_min.get() + radio_max.get())/2;

        int16_t radio_trim_low  = radio_min + dead_zone;

        return (((int32_t)(high_in) * (int32_t)(r_in - radio_trim_low)) / (int32_t)(radio_max - radio_trim_low));
    } else {
        return 0;
    }
}

/*
  return an "angle in centidegrees" (normally -4500 to 4500) from
  the current radio_in value using the specified dead_zone
 */
float RC_Channel::pwm_to_angle_dz_trim(uint16_t _dead_zone, uint16_t _trim) const
{
    int16_t radio_trim_high = _trim + _dead_zone;
    int16_t radio_trim_low  = _trim - _dead_zone;

    float reverse_mul = (reversed?-1:1);

    // don't allow out of range values
    int16_t r_in = constrain_int16(radio_in, radio_min.get(), radio_max.get());

    if (r_in > radio_trim_high && radio_max != radio_trim_high) {
        return reverse_mul * ((float)high_in * (float)(r_in - radio_trim_high)) / (float)(radio_max  - radio_trim_high);
    } else if (r_in < radio_trim_low && radio_trim_low != radio_min) {
        return reverse_mul * ((float)high_in * (float)(r_in - radio_trim_low)) / (float)(radio_trim_low - radio_min);
    } else {
        return 0;
    }
}

/*
  return an "angle in centidegrees" (normally -4500 to 4500) from
  the current radio_in value using the specified dead_zone
 */
float RC_Channel::pwm_to_angle_dz(uint16_t _dead_zone) const
{
    return pwm_to_angle_dz_trim(_dead_zone, radio_trim);
}

/*
  return an "angle in centidegrees" (normally -4500 to 4500) from
  the current radio_in value
 */
float RC_Channel::pwm_to_angle() const
{
    return pwm_to_angle_dz(dead_zone);
}


/*
  convert a pulse width modulation value to a value in the configured
  range, using the specified deadzone
 */
float RC_Channel::pwm_to_range_dz(uint16_t _dead_zone) const
{
    int16_t r_in = constrain_int16(radio_in, radio_min.get(), radio_max.get());

    if (reversed) {
        r_in = radio_max.get() - (r_in - radio_min.get());
    }

    int16_t radio_trim_low  = radio_min + _dead_zone;

    if (r_in > radio_trim_low) {
        return (((float)(high_in) * (float)(r_in - radio_trim_low)) / (float)(radio_max - radio_trim_low));
    }
    return 0;
}

/*
  convert a pulse width modulation value to a value in the configured
  range
 */
float RC_Channel::pwm_to_range() const
{
    return pwm_to_range_dz(dead_zone);
}


float RC_Channel::get_control_in_zero_dz(void) const
{
    if (type_in == ControlType::RANGE) {
        return pwm_to_range_dz(0);
    }
    return pwm_to_angle_dz(0);
}

// ------------------------------------------

float RC_Channel::norm_input() const
{
    float ret;
    int16_t reverse_mul = (reversed?-1:1);
    if (radio_in < radio_trim) {
        if (radio_min >= radio_trim) {
            return 0.0f;
        }
        ret = reverse_mul * (float)(radio_in - radio_trim) / (float)(radio_trim - radio_min);
    } else {
        if (radio_max <= radio_trim) {
            return 0.0f;
        }
        ret = reverse_mul * (float)(radio_in - radio_trim) / (float)(radio_max  - radio_trim);
    }
    return constrain_float(ret, -1.0f, 1.0f);
}

float RC_Channel::norm_input_dz() const
{
    int16_t dz_min = radio_trim - dead_zone;
    int16_t dz_max = radio_trim + dead_zone;
    float ret;
    int16_t reverse_mul = (reversed?-1:1);
    if (radio_in < dz_min && dz_min > radio_min) {
        ret = reverse_mul * (float)(radio_in - dz_min) / (float)(dz_min - radio_min);
    } else if (radio_in > dz_max && radio_max > dz_max) {
        ret = reverse_mul * (float)(radio_in - dz_max) / (float)(radio_max  - dz_max);
    } else {
        ret = 0;
    }
    return constrain_float(ret, -1.0f, 1.0f);
}

// return a normalised input for a channel, in range -1 to 1,
// ignores trim and deadzone
float RC_Channel::norm_input_ignore_trim() const
{
    // sanity check min and max to avoid divide by zero
    if (radio_max <= radio_min) {
        return 0.0f;
    }
    const float ret = (reversed ? -2.0f : 2.0f) * (((float)(radio_in - radio_min) / (float)(radio_max - radio_min)) - 0.5f);
    return constrain_float(ret, -1.0f, 1.0f);
}


bool RC_Channel::norm_input_ignore_trim(float &norm_in) const
{
    if (!rc().has_valid_input()) {
        return false;
    }
    if (radio_in == 0) {
        return false;
    }
    if (radio_max <= radio_min) {
        // sanity check min and max to avoid divide by zero
        return false;
    }
    norm_in = norm_input_ignore_trim();
    return true;
}

/*
  get percentage input from 0 to 100. This ignores the trim value.
 */
uint8_t RC_Channel::percent_input() const
{
    if (radio_in <= radio_min) {
        return reversed?100:0;
    }
    if (radio_in >= radio_max) {
        return reversed?0:100;
    }
    uint8_t ret = 100.0f * (radio_in - radio_min) / (float)(radio_max - radio_min);
    if (reversed) {
        ret = 100 - ret;
    }
    return ret;
}

/*
  return true if input is within deadzone of trim
*/
bool RC_Channel::in_trim_dz() const
{
    return is_bounded_int32(radio_in, radio_trim - dead_zone, radio_trim + dead_zone);
}


/*
   return trues if input is within deadzone of min
*/
bool RC_Channel::in_min_dz() const
{
    return radio_in < radio_min + dead_zone;
}

void RC_Channel::set_override(const uint16_t v, const uint32_t timestamp_ms)
{
    if (!rc().gcs_overrides_enabled()) {
        return;
    }

    last_override_time = timestamp_ms != 0 ? timestamp_ms : AP_HAL::millis();
    override_value = v;
    rc().new_override_received();
}

void RC_Channel::clear_override()
{
    last_override_time = 0;
    override_value = 0;
}

bool RC_Channel::has_override() const
{
    if (override_value == 0) {
        return false;
    }

    uint32_t override_timeout_ms;
    if (!rc().get_override_timeout_ms(override_timeout_ms)) {
        // timeouts are disabled
        return true;
    }

    if (override_timeout_ms == 0) {
        // overrides are explicitly disabled by a zero value
        return false;
    }

    return (AP_HAL::millis() - last_override_time < override_timeout_ms);
}

/*
  perform stick mixing on one channel
  This type of stick mixing reduces the influence of the auto
  controller as it increases the influence of the users stick input,
  allowing the user full deflection if needed
 */
float RC_Channel::stick_mixing(const float servo_in)
{
    float ch_inf = (float)(radio_in - radio_trim);
    ch_inf = fabsf(ch_inf);
    ch_inf = MIN(ch_inf, 400.0f);
    ch_inf = ((400.0f - ch_inf) / 400.0f);

    float servo_out = servo_in;
    servo_out *= ch_inf;
    servo_out += control_in;

    return servo_out;
}

//
// support for auxiliary switches:
//

void RC_Channel::reset_mode_switch()
{
    switch_state.current_position = -1;
    switch_state.debounce_position = -1;
    read_mode_switch();
}

// read a 6 position switch
bool RC_Channel::read_6pos_switch(int8_t& position)
{
    // calculate position of 6 pos switch
    const uint16_t pulsewidth = get_radio_in();
    if (pulsewidth <= RC_MIN_LIMIT_PWM || pulsewidth >= RC_MAX_LIMIT_PWM) {
        return false;  // This is an error condition
    }

    if (pulsewidth < 1231) {
        position = 0;
    } else if (pulsewidth < 1361) {
        position = 1;
    } else if (pulsewidth < 1491) {
        position = 2;
    } else if (pulsewidth < 1621) {
        position = 3;
    } else if (pulsewidth < 1750) {
        position = 4;
    } else {
        position = 5;
    }

    if (!debounce_completed(position)) {
        return false;
    }

    return true;
}

void RC_Channel::read_mode_switch()
{
    int8_t position;
    if (read_6pos_switch(position)) {
        // set flight mode and simple mode setting
        mode_switch_changed(modeswitch_pos_t(position));
    }
}

bool RC_Channel::debounce_completed(int8_t position)
{
    // switch change not detected
    if (switch_state.current_position == position) {
        // reset debouncing
        switch_state.debounce_position = position;
    } else {
        // switch change detected
        const uint32_t tnow_ms = AP_HAL::millis();

        // position not established yet
        if (switch_state.debounce_position != position) {
            switch_state.debounce_position = position;
            switch_state.last_edge_time_ms = tnow_ms;
        } else if (tnow_ms - switch_state.last_edge_time_ms >= SWITCH_DEBOUNCE_TIME_MS) {
            // position estabilished; debounce completed
            switch_state.current_position = position;
            return true;
        }
    }

    return false;
}

//
// support for auxiliary switches:
//

// init_aux_switch_function - initialize aux functions
void RC_Channel::init_aux_function(const AUX_FUNC ch_option, const AuxSwitchPos ch_flag)
{
    // init channel options
    switch (ch_option) {
    // the following functions do not need to be initialised:
#if AP_ARMING_ENABLED
    case AUX_FUNC::ARMDISARM:
    case AUX_FUNC::ARMDISARM_AIRMODE:
#endif
#if AP_BATTERY_ENABLED
    case AUX_FUNC::BATTERY_MPPT_ENABLE:
#endif
#if AP_CAMERA_ENABLED
    case AUX_FUNC::CAMERA_TRIGGER:
#endif
#if AP_MISSION_ENABLED
    case AUX_FUNC::CLEAR_WP:
#endif
    case AUX_FUNC::COMPASS_LEARN:
#if AP_ARMING_ENABLED
    case AUX_FUNC::DISARM:
#endif
    case AUX_FUNC::DO_NOTHING:
#if AP_LANDINGGEAR_ENABLED
    case AUX_FUNC::LANDING_GEAR:
#endif
    case AUX_FUNC::LOST_VEHICLE_SOUND:
#if AP_SERVORELAYEVENTS_ENABLED && AP_RELAY_ENABLED
    case AUX_FUNC::RELAY:
    case AUX_FUNC::RELAY2:
    case AUX_FUNC::RELAY3:
    case AUX_FUNC::RELAY4:
    case AUX_FUNC::RELAY5:
    case AUX_FUNC::RELAY6:
#endif
#if HAL_VISUALODOM_ENABLED
    case AUX_FUNC::VISODOM_ALIGN:
#endif
#if AP_AHRS_ENABLED
    case AUX_FUNC::EKF_LANE_SWITCH:
    case AUX_FUNC::EKF_YAW_RESET:
#endif
#if HAL_GENERATOR_ENABLED
    case AUX_FUNC::GENERATOR: // don't turn generator on or off initially
#endif
#if AP_AHRS_ENABLED
    case AUX_FUNC::EKF_SOURCE_SET:
#endif
#if HAL_TORQEEDO_ENABLED
    case AUX_FUNC::TORQEEDO_CLEAR_ERR:
#endif
#if AP_SCRIPTING_ENABLED
    case AUX_FUNC::SCRIPTING_1:
    case AUX_FUNC::SCRIPTING_2:
    case AUX_FUNC::SCRIPTING_3:
    case AUX_FUNC::SCRIPTING_4:
    case AUX_FUNC::SCRIPTING_5:
    case AUX_FUNC::SCRIPTING_6:
    case AUX_FUNC::SCRIPTING_7:
    case AUX_FUNC::SCRIPTING_8:
    case AUX_FUNC::SCRIPTING_9:
    case AUX_FUNC::SCRIPTING_10:
    case AUX_FUNC::SCRIPTING_11:
    case AUX_FUNC::SCRIPTING_12:
    case AUX_FUNC::SCRIPTING_13:
    case AUX_FUNC::SCRIPTING_14:
    case AUX_FUNC::SCRIPTING_15:
    case AUX_FUNC::SCRIPTING_16:
    case AUX_FUNC::STOP_RESTART_SCRIPTING:
#endif
#if AP_VIDEOTX_ENABLED
    case AUX_FUNC::VTX_POWER:
#endif
#if AP_OPTICALFLOW_CALIBRATOR_ENABLED
    case AUX_FUNC::OPTFLOW_CAL:
#endif
    case AUX_FUNC::TURBINE_START:
#if HAL_MOUNT_ENABLED
    case AUX_FUNC::MOUNT1_ROLL:
    case AUX_FUNC::MOUNT1_PITCH:
    case AUX_FUNC::MOUNT1_YAW:
    case AUX_FUNC::MOUNT2_ROLL:
    case AUX_FUNC::MOUNT2_PITCH:
    case AUX_FUNC::MOUNT2_YAW:
#endif
#if HAL_GENERATOR_ENABLED
    case AUX_FUNC::LOWEHEISER_STARTER:
#endif
#if COMPASS_CAL_ENABLED
    case AUX_FUNC::MAG_CAL:
#endif
#if AP_CAMERA_ENABLED
    case AUX_FUNC::CAMERA_IMAGE_TRACKING:
#endif
#if HAL_MOUNT_ENABLED
    case AUX_FUNC::MOUNT_LRF_ENABLE:
#endif
#if HAL_GENERATOR_ENABLED
    case AUX_FUNC::LOWEHEISER_THROTTLE:
#endif
        break;

    // these functions require explicit initialization
#if HAL_ADSB_ENABLED
    case AUX_FUNC::AVOID_ADSB:
#endif
    case AUX_FUNC::AVOID_PROXIMITY:
#if AP_FENCE_ENABLED
    case AUX_FUNC::FENCE:
#endif
#if AP_GPS_ENABLED
    case AUX_FUNC::GPS_DISABLE:
    case AUX_FUNC::GPS_DISABLE_YAW:
#endif
#if AP_GRIPPER_ENABLED
    case AUX_FUNC::GRIPPER:
#endif
#if AP_INERTIALSENSOR_KILL_IMU_ENABLED
    case AUX_FUNC::KILL_IMU1:
    case AUX_FUNC::KILL_IMU2:
    case AUX_FUNC::KILL_IMU3:
#endif
#if AP_MISSION_ENABLED
    case AUX_FUNC::MISSION_RESET:
#endif
    case AUX_FUNC::MOTOR_ESTOP:
    case AUX_FUNC::RC_OVERRIDE_ENABLE:
#if AP_CAMERA_RUNCAM_ENABLED
    case AUX_FUNC::RUNCAM_CONTROL:
    case AUX_FUNC::RUNCAM_OSD_CONTROL:
#endif
#if HAL_SPRAYER_ENABLED
    case AUX_FUNC::SPRAYER:
#endif
#if AP_AIRSPEED_ENABLED
    case AUX_FUNC::DISABLE_AIRSPEED_USE:
#endif
    case AUX_FUNC::FFT_NOTCH_TUNE:
#if HAL_MOUNT_ENABLED
    case AUX_FUNC::RETRACT_MOUNT1:
    case AUX_FUNC::RETRACT_MOUNT2:
    case AUX_FUNC::MOUNT_LOCK:
#endif
#if HAL_LOGGING_ENABLED
    case AUX_FUNC::LOG_PAUSE:
#endif
    case AUX_FUNC::ARM_EMERGENCY_STOP:
#if AP_CAMERA_ENABLED
    case AUX_FUNC::CAMERA_REC_VIDEO:
    case AUX_FUNC::CAMERA_ZOOM:
    case AUX_FUNC::CAMERA_MANUAL_FOCUS:
    case AUX_FUNC::CAMERA_AUTO_FOCUS:
    case AUX_FUNC::CAMERA_LENS:
#endif
#if AP_AHRS_ENABLED
    case AUX_FUNC::AHRS_TYPE:
#endif
        run_aux_function(ch_option, ch_flag, AuxFuncTrigger::Source::INIT, ch_in);
        break;
    default:
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Failed to init: RC%u_OPTION: %u",
                        (unsigned)(this->ch_in+1), (unsigned)ch_option);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_BoardConfig::config_error("Failed to init: RC%u_OPTION: %u",
                                     (unsigned)(this->ch_in+1), (unsigned)ch_option);
#endif
        break;
    }
}

#if AP_RC_CHANNEL_AUX_FUNCTION_STRINGS_ENABLED

const RC_Channel::LookupTable RC_Channel::lookuptable[] = {
#if AP_MISSION_ENABLED
    { AUX_FUNC::SAVE_WP,"SaveWaypoint"},
#endif
#if AP_CAMERA_ENABLED
    { AUX_FUNC::CAMERA_TRIGGER,"CameraTrigger"},
#endif
#if AP_RANGEFINDER_ENABLED
    { AUX_FUNC::RANGEFINDER,"Rangefinder"},
#endif
#if AP_FENCE_ENABLED
    { AUX_FUNC::FENCE,"Fence"},
#endif
#if HAL_SPRAYER_ENABLED
    { AUX_FUNC::SPRAYER,"Sprayer"},
#endif
#if HAL_PARACHUTE_ENABLED
    { AUX_FUNC::PARACHUTE_ENABLE,"ParachuteEnable"},
    { AUX_FUNC::PARACHUTE_RELEASE,"ParachuteRelease"},
    { AUX_FUNC::PARACHUTE_3POS,"Parachute3Position"},
#endif
#if AP_MISSION_ENABLED
    { AUX_FUNC::MISSION_RESET,"MissionReset"},
#endif
#if HAL_MOUNT_ENABLED
    { AUX_FUNC::RETRACT_MOUNT1,"RetractMount1"},
    { AUX_FUNC::RETRACT_MOUNT2,"RetractMount2"},
#endif
#if AP_SERVORELAYEVENTS_ENABLED && AP_RELAY_ENABLED
    { AUX_FUNC::RELAY,"Relay1"},
#endif
    { AUX_FUNC::MOTOR_ESTOP,"MotorEStop"},
    { AUX_FUNC::MOTOR_INTERLOCK,"MotorInterlock"},
#if AP_SERVORELAYEVENTS_ENABLED && AP_RELAY_ENABLED
    { AUX_FUNC::RELAY2,"Relay2"},
    { AUX_FUNC::RELAY3,"Relay3"},
    { AUX_FUNC::RELAY4,"Relay4"},
#endif
    { AUX_FUNC::PRECISION_LOITER,"PrecisionLoiter"},
    { AUX_FUNC::AVOID_PROXIMITY,"AvoidProximity"},
#if AP_WINCH_ENABLED
    { AUX_FUNC::WINCH_ENABLE,"WinchEnable"},
    { AUX_FUNC::WINCH_CONTROL,"WinchControl"},
#endif
#if AP_MISSION_ENABLED
    { AUX_FUNC::CLEAR_WP,"ClearWaypoint"},
#endif
    { AUX_FUNC::COMPASS_LEARN,"CompassLearn"},
    { AUX_FUNC::SAILBOAT_TACK,"SailboatTack"},
#if AP_GPS_ENABLED
    { AUX_FUNC::GPS_DISABLE,"GPSDisable"},
    { AUX_FUNC::GPS_DISABLE_YAW,"GPSDisableYaw"},
#endif
#if AP_AIRSPEED_ENABLED
    { AUX_FUNC::DISABLE_AIRSPEED_USE,"DisableAirspeedUse"},
#endif
#if AP_SERVORELAYEVENTS_ENABLED && AP_RELAY_ENABLED
    { AUX_FUNC::RELAY5,"Relay5"},
    { AUX_FUNC::RELAY6,"Relay6"},
#endif
    { AUX_FUNC::SAILBOAT_MOTOR_3POS,"SailboatMotor"},
    { AUX_FUNC::SURFACE_TRACKING,"SurfaceTracking"},
#if AP_CAMERA_RUNCAM_ENABLED
    { AUX_FUNC::RUNCAM_CONTROL,"RunCamControl"},
    { AUX_FUNC::RUNCAM_OSD_CONTROL,"RunCamOSDControl"},
#endif
#if HAL_VISUALODOM_ENABLED
    { AUX_FUNC::VISODOM_ALIGN,"VisOdomAlign"},
#endif
    { AUX_FUNC::AIRMODE, "AirMode"},
#if AP_CAMERA_ENABLED
    { AUX_FUNC::CAM_MODE_TOGGLE,"CamModeToggle"},
#endif
#if HAL_GENERATOR_ENABLED
    { AUX_FUNC::GENERATOR,"Generator"},
#endif
#if AP_BATTERY_ENABLED
    { AUX_FUNC::BATTERY_MPPT_ENABLE,"Battery MPPT Enable"},
#endif
#if AP_AIRSPEED_AUTOCAL_ENABLE
    { AUX_FUNC::ARSPD_CALIBRATE,"Calibrate Airspeed"},
#endif
#if HAL_TORQEEDO_ENABLED
    { AUX_FUNC::TORQEEDO_CLEAR_ERR, "Torqeedo Clear Err"},
#endif
    { AUX_FUNC::EMERGENCY_LANDING_EN, "Emergency Landing"},
    { AUX_FUNC::WEATHER_VANE_ENABLE, "Weathervane"},
    { AUX_FUNC::TURBINE_START, "Turbine Start"},
    { AUX_FUNC::FFT_NOTCH_TUNE, "FFT Notch Tuning"},
#if HAL_MOUNT_ENABLED
    { AUX_FUNC::MOUNT_LOCK, "MountLock"},
#endif
#if HAL_LOGGING_ENABLED
    { AUX_FUNC::LOG_PAUSE, "Pause Stream Logging"},
#endif
#if AP_CAMERA_ENABLED
    { AUX_FUNC::CAMERA_REC_VIDEO, "Camera Record Video"},
    { AUX_FUNC::CAMERA_ZOOM, "Camera Zoom"},
    { AUX_FUNC::CAMERA_MANUAL_FOCUS, "Camera Manual Focus"},
    { AUX_FUNC::CAMERA_AUTO_FOCUS, "Camera Auto Focus"},
    { AUX_FUNC::CAMERA_IMAGE_TRACKING, "Camera Image Tracking"},
    { AUX_FUNC::CAMERA_LENS, "Camera Lens"},
#endif
#if HAL_MOUNT_ENABLED
    { AUX_FUNC::MOUNT_LRF_ENABLE, "Mount LRF Enable"},
#endif
};

/* lookup the announcement for switch change */
const char *RC_Channel::string_for_aux_function(AUX_FUNC function) const
{
    for (const struct LookupTable &entry : lookuptable) {
        if (entry.option == function) {
            return entry.announcement;
        }
    }
    return nullptr;
}

/* find string for postion */
const char *RC_Channel::string_for_aux_pos(AuxSwitchPos pos) const
{
    switch (pos) {
    case AuxSwitchPos::HIGH:
        return "HIGH";
    case AuxSwitchPos::MIDDLE:
        return "MIDDLE";
    case AuxSwitchPos::LOW:
        return "LOW";
    }
    return "";
}

#endif // AP_RC_CHANNEL_AUX_FUNCTION_STRINGS_ENABLED

/*
  read an aux channel. Return true if a switch has changed
 */
bool RC_Channel::read_aux()
{
    const AUX_FUNC _option = (AUX_FUNC)option.get();
    if (_option == AUX_FUNC::DO_NOTHING) {
        // may wish to add special cases for other "AUXSW" things
        // here e.g. RCMAP_ROLL etc once they become options
        return false;
#if AP_VIDEOTX_ENABLED
    } else if (_option == AUX_FUNC::VTX_POWER) {
        int8_t position;
        if (read_6pos_switch(position)) {
            AP::vtx().change_power(position);
            return true;
        }
        return false;
#endif  // AP_VIDEOTX_ENABLED
    }

    AuxSwitchPos new_position;
    if (!read_3pos_switch(new_position)) {
        return false;
    }

    if (!switch_state.initialised) {
        switch_state.initialised = true;
        if (init_position_on_first_radio_read((AUX_FUNC)option.get())) {
            switch_state.current_position = (int8_t)new_position;
            switch_state.debounce_position = (int8_t)new_position;
        }
    }

    if (!debounce_completed((int8_t)new_position)) {
        return false;
    }

#if AP_RC_CHANNEL_AUX_FUNCTION_STRINGS_ENABLED
    // announce the change to the GCS:
    const char *aux_string = string_for_aux_function(_option);
    if (aux_string != nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RC%i: %s %s", ch_in+1, aux_string, string_for_aux_pos(new_position));
    }
#endif

    // debounced; undertake the action:
    run_aux_function(_option, new_position, AuxFuncTrigger::Source::RC, ch_in);
    return true;
}

// returns true if the first time we successfully read the
// channel's three-position-switch position we should record that
// position as the current position *without* executing the
// associated auxiliary function.  e.g. do not attempt to arm a
// vehicle when the user turns on their transmitter with the arm
// switch high!
bool RC_Channel::init_position_on_first_radio_read(AUX_FUNC func) const
{
    switch (func) {
#if AP_ARMING_ENABLED
    case AUX_FUNC::ARMDISARM_AIRMODE:
    case AUX_FUNC::ARMDISARM:
    case AUX_FUNC::ARM_EMERGENCY_STOP:
#endif
#if HAL_PARACHUTE_ENABLED
    case AUX_FUNC::PARACHUTE_RELEASE:
#endif

        // we do not want to process 
        return true;
    default:
        return false;
    }
}

void RC_Channel::do_aux_function_armdisarm(const AuxSwitchPos ch_flag)
{
    // arm or disarm the vehicle
    switch (ch_flag) {
    case AuxSwitchPos::HIGH:
        AP::arming().arm(AP_Arming::Method::AUXSWITCH, true);
        break;
    case AuxSwitchPos::MIDDLE:
        // nothing
        break;
    case AuxSwitchPos::LOW:
        AP::arming().disarm(AP_Arming::Method::AUXSWITCH);
        break;
    }
}

#if AP_ADSB_AVOIDANCE_ENABLED
void RC_Channel::do_aux_function_avoid_adsb(const AuxSwitchPos ch_flag)
{
    AP_Avoidance *avoidance = AP::ap_avoidance();
    if (avoidance == nullptr) {
        return;
    }
    if (ch_flag == AuxSwitchPos::HIGH) {
        AP_ADSB *adsb = AP::ADSB();
        if (adsb == nullptr) {
            return;
        }
        // try to enable AP_Avoidance
        if (!adsb->enabled() || !adsb->healthy()) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "ADSB not available");
            return;
        }
        avoidance->enable();
        LOGGER_WRITE_EVENT(LogEvent::AVOIDANCE_ADSB_ENABLE);
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "ADSB Avoidance Enabled");
        return;
    }

    // disable AP_Avoidance
    avoidance->disable();
    LOGGER_WRITE_EVENT(LogEvent::AVOIDANCE_ADSB_DISABLE);
    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "ADSB Avoidance Disabled");
}
#endif  // AP_ADSB_AVOIDANCE_ENABLED

void RC_Channel::do_aux_function_avoid_proximity(const AuxSwitchPos ch_flag)
{
#if AP_AVOIDANCE_ENABLED && !APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    AC_Avoid *avoid = AP::ac_avoid();
    if (avoid == nullptr) {
        return;
    }

    switch (ch_flag) {
    case AuxSwitchPos::HIGH:
        avoid->proximity_avoidance_enable(true);
        break;
    case AuxSwitchPos::MIDDLE:
        // nothing
        break;
    case AuxSwitchPos::LOW:
        avoid->proximity_avoidance_enable(false);
        break;
    }
#endif // !APM_BUILD_ArduPlane
}

#if AP_CAMERA_ENABLED
void RC_Channel::do_aux_function_camera_trigger(const AuxSwitchPos ch_flag)
{
    if (ch_flag == AuxSwitchPos::HIGH) {
        AP_Camera *camera = AP::camera();
        if (camera == nullptr) {
            return;
        }
        camera->take_picture();
    }
}

bool RC_Channel::do_aux_function_record_video(const AuxSwitchPos ch_flag)
{
    AP_Camera *camera = AP::camera();
    if (camera == nullptr) {
        return false;
    }
    return camera->record_video(ch_flag == AuxSwitchPos::HIGH);
}

bool RC_Channel::do_aux_function_camera_zoom(const AuxSwitchPos ch_flag)
{
    AP_Camera *camera = AP::camera();
    if (camera == nullptr) {
        return false;
    }
    int8_t zoom_step = 0;   // zoom out = -1, hold = 0, zoom in = 1
    switch (ch_flag) {
    case AuxSwitchPos::HIGH:
        zoom_step = 1;  // zoom in
        break;
    case AuxSwitchPos::MIDDLE:
        zoom_step = 0;  // zoom hold
        break;
    case AuxSwitchPos::LOW:
        zoom_step = -1; // zoom out
        break;
    }
    return camera->set_zoom(ZoomType::RATE, zoom_step);
}

bool RC_Channel::do_aux_function_camera_manual_focus(const AuxSwitchPos ch_flag)
{
    AP_Camera *camera = AP::camera();
    if (camera == nullptr) {
        return false;
    }
    int8_t focus_step = 0;  // focus in = -1, focus hold = 0, focus out = 1
    switch (ch_flag) {
    case AuxSwitchPos::HIGH:
        // wide shot, focus out
        focus_step = 1;
        break;
    case AuxSwitchPos::MIDDLE:
        focus_step = 0;
        break;
    case AuxSwitchPos::LOW:
        // close shot, focus in
        focus_step = -1;
        break;
    }
    return camera->set_focus(FocusType::RATE, focus_step) == SetFocusResult::ACCEPTED;
}

bool RC_Channel::do_aux_function_camera_auto_focus(const AuxSwitchPos ch_flag)
{
    if (ch_flag == AuxSwitchPos::HIGH) {
        AP_Camera *camera = AP::camera();
        if (camera == nullptr) {
            return false;
        }
        return camera->set_focus(FocusType::AUTO, 0) == SetFocusResult::ACCEPTED;
    }
    return false;
}

bool RC_Channel::do_aux_function_camera_image_tracking(const AuxSwitchPos ch_flag)
{
    AP_Camera *camera = AP::camera();
    if (camera == nullptr) {
        return false;
    }
    // High position enables tracking a POINT in middle of image
    // Low or Mediums disables tracking.  (0.5,0.5) is still passed in but ignored
    return camera->set_tracking(ch_flag == AuxSwitchPos::HIGH ? TrackingType::TRK_POINT : TrackingType::TRK_NONE, Vector2f{0.5, 0.5}, Vector2f{});
}

bool RC_Channel::do_aux_function_camera_lens(const AuxSwitchPos ch_flag)
{
#if AP_CAMERA_SET_CAMERA_SOURCE_ENABLED
    AP_Camera *camera = AP::camera();
    if (camera == nullptr) {
        return false;
    }
    // Low selects lens 0 (default), Mediums selects lens1, High selects lens2
    return camera->set_lens((uint8_t)ch_flag);
#else
    return false;
#endif // AP_CAMERA_SET_CAMERA_SOURCE_ENABLED
}
#endif // AP_CAMERA_ENABLED

#if AP_CAMERA_RUNCAM_ENABLED
void RC_Channel::do_aux_function_runcam_control(const AuxSwitchPos ch_flag)
{
    AP_RunCam *runcam = AP::runcam();
    if (runcam == nullptr) {
        return;
    }

    switch (ch_flag) {
    case AuxSwitchPos::HIGH:
        runcam->start_recording();
        break;
    case AuxSwitchPos::MIDDLE:
        runcam->osd_option();
        break;
    case AuxSwitchPos::LOW:
        runcam->stop_recording();
        break;
    }
}

void RC_Channel::do_aux_function_runcam_osd_control(const AuxSwitchPos ch_flag)
{
    AP_RunCam *runcam = AP::runcam();
    if (runcam == nullptr) {
        return;
    }

    switch (ch_flag) {
    case AuxSwitchPos::HIGH:
        runcam->enter_osd();
        break;
    case AuxSwitchPos::MIDDLE:
    case AuxSwitchPos::LOW:
        runcam->exit_osd();
        break;
    }
}
#endif

#if AP_FENCE_ENABLED
// enable or disable the fence
void RC_Channel::do_aux_function_fence(const AuxSwitchPos ch_flag)
{
    AC_Fence *fence = AP::fence();
    if (fence == nullptr) {
        return;
    }

    fence->enable_configured(ch_flag == AuxSwitchPos::HIGH);
}
#endif

#if AP_MISSION_ENABLED
void RC_Channel::do_aux_function_clear_wp(const AuxSwitchPos ch_flag)
{
    if (ch_flag == AuxSwitchPos::HIGH) {
        AP_Mission *mission = AP::mission();
        if (mission == nullptr) {
            return;
        }
        mission->clear();
    }
}
#endif  // AP_MISSION_ENABLED

#if AP_SERVORELAYEVENTS_ENABLED && AP_RELAY_ENABLED
void RC_Channel::do_aux_function_relay(const uint8_t relay, bool val)
{
    AP_ServoRelayEvents *servorelayevents = AP::servorelayevents();
    if (servorelayevents == nullptr) {
        return;
    }
    servorelayevents->do_set_relay(relay, val);
}
#endif

#if HAL_GENERATOR_ENABLED
void RC_Channel::do_aux_function_generator(const AuxSwitchPos ch_flag)
{
    AP_Generator *generator = AP::generator();
    if (generator == nullptr) {
        return;
    }

    switch (ch_flag) {
    case AuxSwitchPos::LOW:
        generator->stop();
        break;
    case AuxSwitchPos::MIDDLE:
        generator->idle();
        break;
    case AuxSwitchPos::HIGH:
        generator->run();
        break;
    }
}
#endif

#if HAL_SPRAYER_ENABLED
void RC_Channel::do_aux_function_sprayer(const AuxSwitchPos ch_flag)
{
    AC_Sprayer *sprayer = AP::sprayer();
    if (sprayer == nullptr) {
        return;
    }
    sprayer->run(ch_flag == AuxSwitchPos::HIGH);
    // if we are disarmed the pilot must want to test the pump
    sprayer->test_pump((ch_flag == AuxSwitchPos::HIGH) && !hal.util->get_soft_armed());
}
#endif // HAL_SPRAYER_ENABLED

#if AP_GRIPPER_ENABLED
void RC_Channel::do_aux_function_gripper(const AuxSwitchPos ch_flag)
{
    AP_Gripper &gripper = AP::gripper();

    switch (ch_flag) {
    case AuxSwitchPos::LOW:
        gripper.release();
        break;
    case AuxSwitchPos::MIDDLE:
        // nothing
        break;
    case AuxSwitchPos::HIGH:
        gripper.grab();
        break;
    }
}
#endif  // AP_GRIPPER_ENABLED

void RC_Channel::do_aux_function_lost_vehicle_sound(const AuxSwitchPos ch_flag)
{
    switch (ch_flag) {
    case AuxSwitchPos::HIGH:
        AP_Notify::flags.vehicle_lost = true;
        break;
    case AuxSwitchPos::MIDDLE:
        // nothing
        break;
    case AuxSwitchPos::LOW:
        AP_Notify::flags.vehicle_lost = false;
        break;
    }
}

void RC_Channel::do_aux_function_rc_override_enable(const AuxSwitchPos ch_flag)
{
    switch (ch_flag) {
    case AuxSwitchPos::HIGH: {
        rc().set_gcs_overrides_enabled(true);
        break;
    }
    case AuxSwitchPos::MIDDLE:
        // nothing
        break;
    case AuxSwitchPos::LOW: {
        rc().set_gcs_overrides_enabled(false);
        break;
    }
    }
}

#if AP_MISSION_ENABLED
void RC_Channel::do_aux_function_mission_reset(const AuxSwitchPos ch_flag)
{
    if (ch_flag != AuxSwitchPos::HIGH) {
        return;
    }
    AP_Mission *mission = AP::mission();
    if (mission == nullptr) {
        return;
    }
    mission->reset();
}
#endif

void RC_Channel::do_aux_function_fft_notch_tune(const AuxSwitchPos ch_flag)
{
#if HAL_GYROFFT_ENABLED
    AP_GyroFFT *fft = AP::fft();
    if (fft == nullptr) {
        return;
    }

    switch (ch_flag) {
    case AuxSwitchPos::HIGH:
        fft->start_notch_tune();
        break;
    case AuxSwitchPos::MIDDLE:
    case AuxSwitchPos::LOW:
        fft->stop_notch_tune();
        break;
    }
#endif
}

/**
 * Perform the RETRACT_MOUNT 1/2 process.
 * 
 * @param [in] ch_flag  Position of the switch. HIGH, MIDDLE and LOW.
 * @param [in] instance 0: RETRACT MOUNT 1 <br>
 *                      1: RETRACT MOUNT 2
*/
#if HAL_MOUNT_ENABLED
void RC_Channel::do_aux_function_retract_mount(const AuxSwitchPos ch_flag, const uint8_t instance)
{
    AP_Mount *mount = AP::mount();
    if (mount == nullptr) {
        return;
    }
    switch (ch_flag) {
    case AuxSwitchPos::HIGH:
        mount->set_mode(instance,MAV_MOUNT_MODE_RETRACT);
        break;
    case AuxSwitchPos::MIDDLE:
        // nothing
        break;
    case AuxSwitchPos::LOW:
        mount->set_mode_to_default(instance);
        break;
    }
}
#endif  // HAL_MOUNT_ENABLED

bool RC_Channel::run_aux_function(AUX_FUNC ch_option, AuxSwitchPos pos, AuxFuncTrigger::Source source, uint16_t source_index)
{
#if AP_SCRIPTING_ENABLED
    rc().set_aux_cached(ch_option, pos);
#endif

    const AuxFuncTrigger trigger {
        func: ch_option,
        pos: pos,
        source: source,
        source_index: source_index,
    };

    const bool ret = do_aux_function(trigger);

#if HAL_LOGGING_ENABLED
    // @LoggerMessage: AUXF
    // @Description: Auxiliary function invocation information
    // @Field: TimeUS: Time since system startup
    // @Field: function: ID of triggered function
    // @FieldValueEnum: function: RC_Channel::AUX_FUNC
    // @Field: pos: switch position when function triggered
    // @FieldValueEnum: pos: RC_Channel::AuxSwitchPos
    // @Field: source: source of auxiliary function invocation
    // @FieldValueEnum: source: RC_Channel::AuxFuncTrigger::Source
    // @Field: index: index within source. 0 indexed. Invalid for scripting.
    // @Field: result: true if function was successful
    AP::logger().Write(
        "AUXF",
        "TimeUS,function,pos,source,index,result",
        "s#----",
        "F-----",
        "QHBBHB",
        AP_HAL::micros64(),
        uint16_t(ch_option),
        uint8_t(pos),
        uint8_t(source),
        source_index,
        uint8_t(ret)
    );
#endif

    return ret;
}

bool RC_Channel::do_aux_function(const AuxFuncTrigger &trigger)
{
    const AUX_FUNC &ch_option = trigger.func;
    const AuxSwitchPos &ch_flag = trigger.pos;

    switch (ch_option) {
#if AP_FENCE_ENABLED
    case AUX_FUNC::FENCE:
        do_aux_function_fence(ch_flag);
        break;
#endif

#if AP_GRIPPER_ENABLED
    case AUX_FUNC::GRIPPER:
        do_aux_function_gripper(ch_flag);
        break;
#endif

    case AUX_FUNC::RC_OVERRIDE_ENABLE:
        // Allow or disallow RC_Override
        do_aux_function_rc_override_enable(ch_flag);
        break;

    case AUX_FUNC::AVOID_PROXIMITY:
        do_aux_function_avoid_proximity(ch_flag);
        break;

#if AP_SERVORELAYEVENTS_ENABLED && AP_RELAY_ENABLED
    case AUX_FUNC::RELAY:
        do_aux_function_relay(0, ch_flag == AuxSwitchPos::HIGH);
        break;
    case AUX_FUNC::RELAY2:
        do_aux_function_relay(1, ch_flag == AuxSwitchPos::HIGH);
        break;
    case AUX_FUNC::RELAY3:
        do_aux_function_relay(2, ch_flag == AuxSwitchPos::HIGH);
        break;
    case AUX_FUNC::RELAY4:
        do_aux_function_relay(3, ch_flag == AuxSwitchPos::HIGH);
        break;
    case AUX_FUNC::RELAY5:
        do_aux_function_relay(4, ch_flag == AuxSwitchPos::HIGH);
        break;
    case AUX_FUNC::RELAY6:
        do_aux_function_relay(5, ch_flag == AuxSwitchPos::HIGH);
        break;
#endif  // AP_SERVORELAYEVENTS_ENABLED && AP_RELAY_ENABLED

#if AP_CAMERA_RUNCAM_ENABLED
    case AUX_FUNC::RUNCAM_CONTROL:
        do_aux_function_runcam_control(ch_flag);
        break;

    case AUX_FUNC::RUNCAM_OSD_CONTROL:
        do_aux_function_runcam_osd_control(ch_flag);
        break;
#endif

#if AP_MISSION_ENABLED
    case AUX_FUNC::CLEAR_WP:
        do_aux_function_clear_wp(ch_flag);
        break;
    case AUX_FUNC::MISSION_RESET:
        do_aux_function_mission_reset(ch_flag);
        break;
#endif

#if AP_ADSB_AVOIDANCE_ENABLED
    case AUX_FUNC::AVOID_ADSB:
        do_aux_function_avoid_adsb(ch_flag);
        break;
#endif  // AP_ADSB_AVOIDANCE_ENABLED

    case AUX_FUNC::FFT_NOTCH_TUNE:
        do_aux_function_fft_notch_tune(ch_flag);
        break;

#if HAL_GENERATOR_ENABLED
    case AUX_FUNC::GENERATOR:
        do_aux_function_generator(ch_flag);
        break;
#endif

#if AP_BATTERY_ENABLED
    case AUX_FUNC::BATTERY_MPPT_ENABLE:
        if (ch_flag != AuxSwitchPos::MIDDLE) {
            AP::battery().MPPT_set_powered_state_to_all(ch_flag == AuxSwitchPos::HIGH);
        }
        break;
#endif

#if HAL_SPRAYER_ENABLED
    case AUX_FUNC::SPRAYER:
        do_aux_function_sprayer(ch_flag);
        break;
#endif

    case AUX_FUNC::LOST_VEHICLE_SOUND:
        do_aux_function_lost_vehicle_sound(ch_flag);
        break;

#if AP_ARMING_ENABLED
    case AUX_FUNC::ARMDISARM:
        do_aux_function_armdisarm(ch_flag);
        break;

    case AUX_FUNC::DISARM:
        if (ch_flag == AuxSwitchPos::HIGH) {
            AP::arming().disarm(AP_Arming::Method::AUXSWITCH);
        }
        break;
#endif

    case AUX_FUNC::COMPASS_LEARN:
        if (ch_flag == AuxSwitchPos::HIGH) {
            Compass &compass = AP::compass();
            compass.set_learn_type(Compass::LearnType::INFLIGHT, false);
        }
        break;

#if AP_LANDINGGEAR_ENABLED
    case AUX_FUNC::LANDING_GEAR: {
        AP_LandingGear *lg = AP_LandingGear::get_singleton();
        if (lg == nullptr) {
            break;
        }
        switch (ch_flag) {
        case AuxSwitchPos::LOW:
            lg->set_position(AP_LandingGear::LandingGear_Deploy);
            break;
        case AuxSwitchPos::MIDDLE:
            // nothing
            break;
        case AuxSwitchPos::HIGH:
            lg->set_position(AP_LandingGear::LandingGear_Retract);
            break;
        }
        break;
    }
#endif

#if AP_GPS_ENABLED
    case AUX_FUNC::GPS_DISABLE:
        AP::gps().force_disable(ch_flag == AuxSwitchPos::HIGH);
#if AP_EXTERNAL_AHRS_ENABLED
        AP::externalAHRS().set_gnss_disable(ch_flag == AuxSwitchPos::HIGH);
#endif
        break;

    case AUX_FUNC::GPS_DISABLE_YAW:
        AP::gps().set_force_disable_yaw(ch_flag == AuxSwitchPos::HIGH);
        break;
#endif  // AP_GPS_ENABLED

#if AP_AIRSPEED_ENABLED
    case AUX_FUNC::DISABLE_AIRSPEED_USE: {
        AP_Airspeed *airspeed = AP::airspeed();
        if (airspeed == nullptr) {
            break;
        }
        switch (ch_flag) {
        case AuxSwitchPos::HIGH:
            airspeed->force_disable_use(true);
            break;
        case AuxSwitchPos::MIDDLE:
            break;
        case AuxSwitchPos::LOW:
            airspeed->force_disable_use(false);
            break;
        }
        break;
    }
#endif

    case AUX_FUNC::MOTOR_ESTOP:
        switch (ch_flag) {
        case AuxSwitchPos::HIGH: {
            SRV_Channels::set_emergency_stop(true);
            break;
        }
        case AuxSwitchPos::MIDDLE:
            // nothing
            break;
        case AuxSwitchPos::LOW: {
            SRV_Channels::set_emergency_stop(false);
            break;
        }
        }
        break;

#if HAL_VISUALODOM_ENABLED
    case AUX_FUNC::VISODOM_ALIGN:
        if (ch_flag == AuxSwitchPos::HIGH) {
            AP_VisualOdom *visual_odom = AP::visualodom();
            if (visual_odom != nullptr) {
                visual_odom->request_align_yaw_to_ahrs();
            }
        }
        break;
#endif

#if AP_AHRS_ENABLED
    case AUX_FUNC::EKF_SOURCE_SET: {
        AP_NavEKF_Source::SourceSetSelection source_set = AP_NavEKF_Source::SourceSetSelection::PRIMARY;
        switch (ch_flag) {
        case AuxSwitchPos::LOW:
            // low switches to primary source
            source_set = AP_NavEKF_Source::SourceSetSelection::PRIMARY;
            break;
        case AuxSwitchPos::MIDDLE:
            // middle switches to secondary source
            source_set = AP_NavEKF_Source::SourceSetSelection::SECONDARY;
            break;
        case AuxSwitchPos::HIGH:
            // high switches to tertiary source
            source_set = AP_NavEKF_Source::SourceSetSelection::TERTIARY;
            break;
        }
        AP::ahrs().set_posvelyaw_source_set(source_set);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Using EKF Source Set %u", uint8_t(source_set)+1);
        break;
    }
#endif  // AP_AHRS_ENABLED

#if AP_OPTICALFLOW_CALIBRATOR_ENABLED
    case AUX_FUNC::OPTFLOW_CAL: {
        AP_OpticalFlow *optflow = AP::opticalflow();
        if (optflow == nullptr) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "OptFlow Cal: failed sensor not enabled");
            break;
        }
        if (ch_flag == AuxSwitchPos::HIGH) {
            optflow->start_calibration();
        } else {
            optflow->stop_calibration();
        }
        break;
    }
#endif

#if AP_INERTIALSENSOR_KILL_IMU_ENABLED
    case AUX_FUNC::KILL_IMU1:
        AP::ins().kill_imu(0, ch_flag == AuxSwitchPos::HIGH);
        break;

    case AUX_FUNC::KILL_IMU2:
        AP::ins().kill_imu(1, ch_flag == AuxSwitchPos::HIGH);
        break;

    case AUX_FUNC::KILL_IMU3:
        AP::ins().kill_imu(2, ch_flag == AuxSwitchPos::HIGH);
        break;
#endif  // AP_INERTIALSENSOR_KILL_IMU_ENABLED

#if AP_CAMERA_ENABLED
    case AUX_FUNC::CAMERA_TRIGGER:
        do_aux_function_camera_trigger(ch_flag);
        break;

    case AUX_FUNC::CAM_MODE_TOGGLE: {
        // Momentary switch to for cycling camera modes
        AP_Camera *camera = AP_Camera::get_singleton();
        if (camera == nullptr) {
            break;
        }
        switch (ch_flag) {
        case AuxSwitchPos::LOW:
            // nothing
            break;
        case AuxSwitchPos::MIDDLE:
            // nothing
            break;
        case AuxSwitchPos::HIGH:
            camera->cam_mode_toggle();
            break;
        }
        break;
    }
    case AUX_FUNC::CAMERA_REC_VIDEO:
        return do_aux_function_record_video(ch_flag);

    case AUX_FUNC::CAMERA_ZOOM:
        return do_aux_function_camera_zoom(ch_flag);

    case AUX_FUNC::CAMERA_MANUAL_FOCUS:
        return do_aux_function_camera_manual_focus(ch_flag);

    case AUX_FUNC::CAMERA_AUTO_FOCUS:
        return do_aux_function_camera_auto_focus(ch_flag);

    case AUX_FUNC::CAMERA_IMAGE_TRACKING:
        return do_aux_function_camera_image_tracking(ch_flag);

#if AP_CAMERA_SET_CAMERA_SOURCE_ENABLED
    case AUX_FUNC::CAMERA_LENS:
        return do_aux_function_camera_lens(ch_flag);
#endif // AP_CAMERA_SET_CAMERA_SOURCE_ENABLED
#endif // AP_CAMERA_ENABLED

#if HAL_MOUNT_ENABLED
    case AUX_FUNC::RETRACT_MOUNT1:
        do_aux_function_retract_mount(ch_flag, 0);
        break;

    case AUX_FUNC::RETRACT_MOUNT2:
        do_aux_function_retract_mount(ch_flag, 1);
        break;

    case AUX_FUNC::MOUNT_LOCK: {
        AP_Mount *mount = AP::mount();
        if (mount == nullptr) {
            break;
        }
        mount->set_yaw_lock(ch_flag == AuxSwitchPos::HIGH);
        break;
    }

    case AUX_FUNC::MOUNT_LRF_ENABLE: {
        AP_Mount *mount = AP::mount();
        if (mount == nullptr) {
            break;
        }
        mount->set_rangefinder_enable(0, ch_flag == AuxSwitchPos::HIGH);
        break;
    }
#endif

#if HAL_LOGGING_ENABLED
    case AUX_FUNC::LOG_PAUSE: {
        AP_Logger *logger = AP_Logger::get_singleton();
        switch (ch_flag) {
        case AuxSwitchPos::LOW:
            logger->log_pause(false);
            break;
        case AuxSwitchPos::MIDDLE:
            // nothing
            break;
        case AuxSwitchPos::HIGH:
            logger->log_pause(true);
            break;
        }
        break;
    }
#endif

#if COMPASS_CAL_ENABLED
    case AUX_FUNC::MAG_CAL: {
        Compass &compass = AP::compass();
        switch (ch_flag) {
        case AuxSwitchPos::LOW:
            compass.cancel_calibration_all();
            break;
        case AuxSwitchPos::MIDDLE:
            // nothing
            break;
        case AuxSwitchPos::HIGH:
            if (!hal.util->get_soft_armed()) {
                const bool retry = true;
                const bool autosave = true;
                const float delay = 5.0;
                const bool autoreboot = false;
                compass.start_calibration_all(retry, autosave, delay, autoreboot);
            } else {
                GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Disarm to allow compass calibration");
            }
            break;
        }
        break;
    }
#endif

#if AP_ARMING_ENABLED
    case AUX_FUNC::ARM_EMERGENCY_STOP: {
        switch (ch_flag) {
        case AuxSwitchPos::HIGH:
            // request arm, disable emergency motor stop
            SRV_Channels::set_emergency_stop(false);
            AP::arming().arm(AP_Arming::Method::AUXSWITCH, true);
            break;
        case AuxSwitchPos::MIDDLE:
            // disable emergency motor stop
            SRV_Channels::set_emergency_stop(false);
            break;
        case AuxSwitchPos::LOW:
            // enable emergency motor stop
            SRV_Channels::set_emergency_stop(true);
            break;
        }
        break;
    }
#endif  // AP_ARMING_ENABLED

#if AP_AHRS_ENABLED
    case AUX_FUNC::EKF_LANE_SWITCH:
        // used to test emergency lane switch
        AP::ahrs().check_lane_switch();
        break;

    case AUX_FUNC::EKF_YAW_RESET:
        // used to test emergency yaw reset
        AP::ahrs().request_yaw_reset();
        break;

    case AUX_FUNC::AHRS_TYPE: {
#if HAL_NAVEKF3_AVAILABLE && AP_EXTERNAL_AHRS_ENABLED
        AP::ahrs().set_ekf_type(ch_flag==AuxSwitchPos::HIGH? AP_AHRS::EKFType::EXTERNAL : AP_AHRS::EKFType::THREE);
#endif
        break;
    }
#endif  // AP_AHRS_ENABLED

#if HAL_TORQEEDO_ENABLED
    // clear torqeedo error
    case AUX_FUNC::TORQEEDO_CLEAR_ERR: {
        if (ch_flag == AuxSwitchPos::HIGH) {
            AP_Torqeedo *torqeedo = AP_Torqeedo::get_singleton();
            if (torqeedo != nullptr) {
                torqeedo->clear_motor_error();
            }
        }
        break;
    }
#endif

#if AP_SCRIPTING_ENABLED
    case AUX_FUNC::STOP_RESTART_SCRIPTING: {
        AP_Scripting *scr = AP::scripting();
        if (scr != nullptr) {
            switch (ch_flag) {
            case AuxSwitchPos::HIGH:
                scr->stop();
                break;
            case AuxSwitchPos::MIDDLE:
                break;
            case AuxSwitchPos::LOW:
                scr->restart_all();
                break;
            }
        }
        break;
    }
#endif

// do nothing for these functions
#if HAL_MOUNT_ENABLED
    case AUX_FUNC::MOUNT1_ROLL:
    case AUX_FUNC::MOUNT1_PITCH:
    case AUX_FUNC::MOUNT1_YAW:
    case AUX_FUNC::MOUNT2_ROLL:
    case AUX_FUNC::MOUNT2_PITCH:
    case AUX_FUNC::MOUNT2_YAW:
#endif
#if AP_SCRIPTING_ENABLED
    case AUX_FUNC::SCRIPTING_1:
    case AUX_FUNC::SCRIPTING_2:
    case AUX_FUNC::SCRIPTING_3:
    case AUX_FUNC::SCRIPTING_4:
    case AUX_FUNC::SCRIPTING_5:
    case AUX_FUNC::SCRIPTING_6:
    case AUX_FUNC::SCRIPTING_7:
    case AUX_FUNC::SCRIPTING_8:
    case AUX_FUNC::SCRIPTING_9:
    case AUX_FUNC::SCRIPTING_10:
    case AUX_FUNC::SCRIPTING_11:
    case AUX_FUNC::SCRIPTING_12:
    case AUX_FUNC::SCRIPTING_13:
    case AUX_FUNC::SCRIPTING_14:
    case AUX_FUNC::SCRIPTING_15:
    case AUX_FUNC::SCRIPTING_16:
#endif
        break;

#if HAL_GENERATOR_ENABLED
    case AUX_FUNC::LOWEHEISER_THROTTLE:
    case AUX_FUNC::LOWEHEISER_STARTER:
        // monitored by the library itself
        break;
#endif

    default:
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Invalid channel option (%u)", (unsigned int)ch_option);
        return false;
    }

    return true;
}

void RC_Channel::init_aux()
{
    AuxSwitchPos position;
    if (!read_3pos_switch(position)) {
        position = AuxSwitchPos::LOW;
    }

    init_aux_function((AUX_FUNC)option.get(), position);
}

// read_3pos_switch
bool RC_Channel::read_3pos_switch(RC_Channel::AuxSwitchPos &ret) const
{
    const uint16_t in = get_radio_in();
    if (in <= RC_MIN_LIMIT_PWM || in >= RC_MAX_LIMIT_PWM) {
        return false;
    }

    // switch is reversed if 'reversed' option set on channel and switches reverse is allowed by RC_OPTIONS
    bool switch_reversed = reversed && rc().option_is_enabled(RC_Channels::Option::ALLOW_SWITCH_REV);

    if (in < AUX_SWITCH_PWM_TRIGGER_LOW) {
        ret = switch_reversed ? AuxSwitchPos::HIGH : AuxSwitchPos::LOW;
    } else if (in > AUX_SWITCH_PWM_TRIGGER_HIGH) {
        ret = switch_reversed ? AuxSwitchPos::LOW : AuxSwitchPos::HIGH;
    } else {
        ret = AuxSwitchPos::MIDDLE;
    }
    return true;
}

// return switch position value as LOW, MIDDLE, HIGH
// if reading the switch fails then it returns LOW
RC_Channel::AuxSwitchPos RC_Channel::get_aux_switch_pos() const
{
    AuxSwitchPos position = AuxSwitchPos::LOW;
    UNUSED_RESULT(read_3pos_switch(position));

    return position;
}

// return stick gesture pos as LOW, MIDDLE, HIGH
// this function uses different threshold values to RC_Chanel::get_aux_switch_pos()
// to avoid glitching on the stick travel and also always honours channel reversal
RC_Channel::AuxSwitchPos RC_Channel::get_stick_gesture_pos() const
{
    const uint16_t in = get_radio_in();
    if (in <= 900 || in >= 2200) {
        return RC_Channel::AuxSwitchPos::LOW;
    }

    // switch is reversed if 'reversed' option set on channel and switches reverse is allowed by RC_OPTIONS
    bool switch_reversed = get_reverse();

    if (in < RC_Channel::AUX_PWM_TRIGGER_LOW) {
        return switch_reversed ? RC_Channel::AuxSwitchPos::HIGH : RC_Channel::AuxSwitchPos::LOW;
    }
    if (in > RC_Channel::AUX_PWM_TRIGGER_HIGH) {
        return switch_reversed ? RC_Channel::AuxSwitchPos::LOW : RC_Channel::AuxSwitchPos::HIGH;
    }
    return RC_Channel::AuxSwitchPos::MIDDLE;
}

RC_Channel *RC_Channels::find_channel_for_option(const RC_Channel::AUX_FUNC option)
{
    for (uint8_t i=0; i<NUM_RC_CHANNELS; i++) {
        RC_Channel *c = channel(i);
        if (c == nullptr) {
            // odd?
            continue;
        }
        if ((RC_Channel::AUX_FUNC)c->option.get() == option) {
            return c;
        }
    }
    return nullptr;
}

// duplicate_options_exist - returns true if any options are duplicated
bool RC_Channels::duplicate_options_exist()
{
    Bitmask<(uint16_t)RC_Channel::AUX_FUNC::AUX_FUNCTION_MAX> used_auxsw_options;
    for (uint8_t i=0; i<NUM_RC_CHANNELS; i++) {
        const RC_Channel *c = channel(i);
        if (c == nullptr) {
            // odd?
            continue;
        }
        const uint16_t option = c->option.get();
        if (option == (uint16_t)RC_Channel::AUX_FUNC::DO_NOTHING) {
            continue;
        }
        if (option >= used_auxsw_options.size()) {
            continue;
        }
        if (used_auxsw_options.get(option)) {
            return true;
        }
        used_auxsw_options.set(option);
    }
    return false;
}

// convert option parameter from old to new
void RC_Channels::convert_options(const RC_Channel::AUX_FUNC old_option, const RC_Channel::AUX_FUNC new_option)
{
    for (uint8_t i=0; i<NUM_RC_CHANNELS; i++) {
        RC_Channel *c = channel(i);
        if (c == nullptr) {
            // odd?
            continue;
        }
        if ((RC_Channel::AUX_FUNC)c->option.get() == old_option) {
            c->option.set_and_save((int16_t)new_option);
        }
    }
}

#endif  // AP_RC_CHANNEL_ENABLED
