#pragma once

#define AP_PARAM_VEHICLE_NAME sub

#include <AP_Common/AP_Common.h>

#include <AP_Arming/AP_Arming.h>
#include "actuators.h"
// Global parameter class.
//
class Parameters {
public:
    // The version of the layout as described by the parameter enum.
    //
    // When changing the parameter enum in an incompatible fashion, this
    // value should be incremented by one.
    //
    // The increment will prevent old parameters from being used incorrectly
    // by newer code.
    //
    static const uint16_t        k_format_version = 1;

    // Parameter identities.
    //
    // The enumeration defined here is used to ensure that every parameter
    // or parameter group has a unique ID number.   This number is used by
    // AP_Param to store and locate parameters in EEPROM.
    //
    // Note that entries without a number are assigned the next number after
    // the entry preceding them.    When adding new entries, ensure that they
    // don't overlap.
    //
    // Try to group related variables together, and assign them a set
    // range in the enumeration.    Place these groups in numerical order
    // at the end of the enumeration.
    //
    // WARNING: Care should be taken when editing this enumeration as the
    //          AP_Param load/save code depends on the values here to identify
    //          variables saved in EEPROM.
    //
    //
    enum {
        // Layout version number, always key zero.
        //
        k_param_format_version = 0,
        k_param_software_type, // unused

        k_param_g2, // 2nd block of parameters

        k_param_sitl, // Simulation
        k_param_osd, //OSD

        // Telemetry
        k_param_gcs0_unused = 10,      // unused in ArduPilot-4.7
        k_param_gcs1_unused,           // unused in ArduPilot-4.7
        k_param_gcs2_unused,           // unused in ArduPilot-4.7
        k_param_gcs3_unused,           // unused in ArduPilot-4.7
        k_param_sysid_this_mav_old,
        k_param_sysid_my_gcs_old,

        // Hardware/Software configuration
        k_param_BoardConfig = 20, // Board configuration (Pixhawk/Linux/etc)
        k_param_scheduler, // Scheduler (for debugging/perf_info)
        k_param_logger, // AP_Logger Logging
        k_param_serial_manager_old, // Serial ports, AP_SerialManager
        k_param_notify, // Notify Library, AP_Notify
        k_param_arming = 26, // Arming checks
        k_param_can_mgr,
        k_param_thr_arming_position,

        // Sensor objects
        k_param_ins = 30, // AP_InertialSensor
        k_param_compass, // Compass
        k_param_barometer, // Barometer/Depth Sensor
        k_param_battery, // AP_BattMonitor
        k_param_leak_detector, // Leak Detector
        k_param_rangefinder, // Rangefinder
        k_param_gps, // GPS
        k_param_optflow, // Optical Flow


        // Navigation libraries
        k_param_ahrs = 50, // AHRS
        k_param_NavEKF, // Extended Kalman Filter Inertial Navigation             // remove
        k_param_NavEKF2, // EKF2
        k_param_attitude_control, // Attitude Control
        k_param_pos_control, // Position Control
        k_param_wp_nav, // Waypoint navigation
        k_param_mission, // Mission library
        k_param_fence_old, // only used for conversion
        k_param_terrain, // Terrain database
        k_param_rally, // Disabled
        k_param_circle_nav, // Disabled
        k_param_avoid, // Relies on proximity and fence
        k_param_NavEKF3,
        k_param_loiter_nav,


        // Other external hardware interfaces
        k_param_motors = 65, // Motors
        k_param_relay, // Relay
        k_param_camera, // Camera
        k_param_camera_mount, // Camera gimbal


        // RC_Channel settings (deprecated)
        k_param_rc_1_old = 75,
        k_param_rc_2_old,
        k_param_rc_3_old,
        k_param_rc_4_old,
        k_param_rc_5_old,
        k_param_rc_6_old,
        k_param_rc_7_old,
        k_param_rc_8_old,
        k_param_rc_9_old,
        k_param_rc_10_old,
        k_param_rc_11_old,
        k_param_rc_12_old,
        k_param_rc_13_old,
        k_param_rc_14_old,

        // Joystick gain parameters
        k_param_gain_default,
        k_param_maxGain,
        k_param_minGain,
        k_param_numGainSettings,
        k_param_cam_tilt_step, // deprecated
        k_param_lights_step, // deprecated

        // Joystick button mapping parameters
        k_param_jbtn_0 = 95,
        k_param_jbtn_1,
        k_param_jbtn_2,
        k_param_jbtn_3,
        k_param_jbtn_4,
        k_param_jbtn_5,
        k_param_jbtn_6,
        k_param_jbtn_7,
        k_param_jbtn_8,
        k_param_jbtn_9,
        k_param_jbtn_10,
        k_param_jbtn_11,
        k_param_jbtn_12,
        k_param_jbtn_13,
        k_param_jbtn_14,
        k_param_jbtn_15,

        // 16 more for MANUAL_CONTROL extensions
        k_param_jbtn_16,
        k_param_jbtn_17,
        k_param_jbtn_18,
        k_param_jbtn_19,
        k_param_jbtn_20,
        k_param_jbtn_21,
        k_param_jbtn_22,
        k_param_jbtn_23,
        k_param_jbtn_24,
        k_param_jbtn_25,
        k_param_jbtn_26,
        k_param_jbtn_27,
        k_param_jbtn_28,
        k_param_jbtn_29,
        k_param_jbtn_30,
        k_param_jbtn_31,

        // PID Controllers
        k_param_p_pos_xy = 126, // deprecated
        k_param_p_alt_hold, // deprecated
        k_param_pi_vel_xy, // deprecated
        k_param_p_vel_z, // deprecated
        k_param_pid_accel_z, // deprecated


        // Failsafes
        k_param_failsafe_gcs = 140,
        k_param_failsafe_leak, // leak failsafe behavior
        k_param_failsafe_pressure, // internal pressure failsafe behavior
        k_param_failsafe_pressure_max, // maximum internal pressure in pascal before failsafe is triggered
        k_param_failsafe_temperature, // internal temperature failsafe behavior
        k_param_failsafe_temperature_max, // maximum internal temperature in degrees C before failsafe is triggered
        k_param_failsafe_terrain, // terrain failsafe behavior
        k_param_fs_ekf_thresh,
        k_param_fs_ekf_action,
        k_param_fs_crash_check,
        k_param_failsafe_battery_enabled, // unused - moved to AP_BattMonitor
        k_param_fs_batt_mah,              // unused - moved to AP_BattMonitor
        k_param_fs_batt_voltage,          // unused - moved to AP_BattMonitor
        k_param_failsafe_pilot_input,
        k_param_failsafe_pilot_input_timeout,
        k_param_failsafe_gcs_timeout,


        // Misc Sub settings
        k_param_log_bitmask = 165,
        k_param_angle_max = 167,
        k_param_rangefinder_gain, // deprecated
        k_param_wp_yaw_behavior = 170,
        k_param_xtrack_angle_limit, // Angle limit for crosstrack correction in Auto modes (degrees)
        k_param_pilot_speed_up,     // renamed from k_param_pilot_velocity_z_max
        k_param_pilot_accel_z,
        k_param_compass_enabled_deprecated,
        k_param_surface_depth,
        k_param_rc_speed, // Main output pwm frequency
        k_param_gcs_pid_mask = 178,
        k_param_throttle_filt,
        k_param_throttle_deadzone, // Used in auto-throttle modes
        k_param_terrain_follow = 182,   // deprecated
        k_param_rc_feel_rp,
        k_param_throttle_gain,
        k_param_cam_tilt_center, // deprecated
        k_param_frame_configuration,
        k_param_surface_max_throttle,
        k_param_surface_nobaro_thrust,
        // 200: flight modes
        k_param_flight_mode1 = 200,
        k_param_flight_mode2,
        k_param_flight_mode3,
        k_param_flight_mode4,
        k_param_flight_mode5,
        k_param_flight_mode6,
        k_param_simple_modes,
        k_param_flight_mode_chan,
#if AP_RSSI_ENABLED
        k_param_rssi,
#endif 
        
        // Acro Mode parameters
        k_param_acro_yaw_p = 220, // Used in all modes for get_pilot_desired_yaw_rate
        k_param_acro_trainer,
        k_param_acro_expo,
        k_param_acro_rp_p,
        k_param_acro_balance_roll,
        k_param_acro_balance_pitch,

        // RPM Sensor
        k_param_rpm_sensor_old = 232, // unused - moved to vehicle

        // RC_Mapper Library
        k_param_rcmap, // Disabled

        k_param_gcs4_unused,           // unused in ArduPilot-4.7
        k_param_gcs5_unused,           // unused in ArduPilot-4.7
        k_param_gcs6_unused,           // unused in ArduPilot-4.7

        k_param_cam_slew_limit = 237, // deprecated
        k_param_lights_steps,
        k_param_pilot_speed_dn,
        k_param_rangefinder_signal_min,
        k_param_surftrak_depth,
        k_param_pilot_speed,
        k_param_failsafe_throttle,
        k_param_failsafe_throttle_value,
        k_param_vehicle = 257, // vehicle common block of parameters
        k_param__gcs = 258,
    };

    AP_Int16        format_version;

    // Telemetry control
    //
    AP_Float        throttle_filt;

#if AP_RANGEFINDER_ENABLED
    AP_Int8         rangefinder_signal_min;     // minimum signal quality for good rangefinder readings
    AP_Float        surftrak_depth;             // surftrak will try to keep sub below this depth
#endif

    AP_Int8         failsafe_leak;              // leak detection failsafe behavior
    AP_Int8         failsafe_gcs;               // ground station failsafe behavior
    AP_Int8         failsafe_pressure;
    AP_Int8         failsafe_temperature;
    AP_Int32        failsafe_pressure_max;
    AP_Int8         failsafe_temperature_max;
    AP_Int8         failsafe_terrain;
    AP_Int8         failsafe_pilot_input;       // pilot input failsafe behavior
    AP_Float        failsafe_pilot_input_timeout;
    AP_Float        failsafe_gcs_timeout;       // ground station failsafe timeout (seconds)

    AP_Int8         xtrack_angle_limit;

    AP_Int8         wp_yaw_behavior;            // controls how the autopilot controls yaw during missions
    AP_Int8         rc_feel_rp;                 // controls vehicle response to user input with 0 being extremely soft and 100 begin extremely crisp

    // Waypoints
    //
    AP_Int16        pilot_speed_up;             // maximum vertical ascending velocity the pilot may request
    AP_Int16        pilot_speed_dn;             // maximum vertical descending velocity the pilot may request
    AP_Int16        pilot_speed;                // maximum horizontal (xy) velocity the pilot may request
    AP_Int16        pilot_accel_z;              // vertical acceleration the pilot may request

    // Throttle
    //
    AP_Int16        throttle_deadzone;
    AP_Int8         failsafe_throttle;
    AP_Int16        failsafe_throttle_value;
    AP_Int16        thr_arming_position;
    

    // Misc
    //
    AP_Int32        log_bitmask;

    AP_Int8         fs_ekf_action;
    AP_Int8         fs_crash_check;
    AP_Float        fs_ekf_thresh;
    AP_Int16        gcs_pid_mask;

    AP_Int16        rc_speed; // speed of fast RC Channels in Hz

    AP_Float        gain_default;
    AP_Float        maxGain;
    AP_Float        minGain;
    AP_Int8         numGainSettings;
    AP_Float        throttle_gain;

    AP_Int16        lights_steps;

    // Joystick button parameters
    JSButton        jbtn_0;
    JSButton        jbtn_1;
    JSButton        jbtn_2;
    JSButton        jbtn_3;
    JSButton        jbtn_4;
    JSButton        jbtn_5;
    JSButton        jbtn_6;
    JSButton        jbtn_7;
    JSButton        jbtn_8;
    JSButton        jbtn_9;
    JSButton        jbtn_10;
    JSButton        jbtn_11;
    JSButton        jbtn_12;
    JSButton        jbtn_13;
    JSButton        jbtn_14;
    JSButton        jbtn_15;
    // 16 - 31 from manual_control extension
    JSButton        jbtn_16;
    JSButton        jbtn_17;
    JSButton        jbtn_18;
    JSButton        jbtn_19;
    JSButton        jbtn_20;
    JSButton        jbtn_21;
    JSButton        jbtn_22;
    JSButton        jbtn_23;
    JSButton        jbtn_24;
    JSButton        jbtn_25;
    JSButton        jbtn_26;
    JSButton        jbtn_27;
    JSButton        jbtn_28;
    JSButton        jbtn_29;
    JSButton        jbtn_30;
    JSButton        jbtn_31;

    // Acro parameters
    AP_Float        acro_rp_p;
    AP_Float        acro_yaw_p;
    AP_Float        acro_balance_roll;
    AP_Float        acro_balance_pitch;
    AP_Int8         acro_trainer;
    AP_Float        acro_expo;
    
#if AP_SUB_RC_ENABLED

    // Flight modes
    //
    AP_Int8         flight_mode1;
    AP_Int8         flight_mode2;
    AP_Int8         flight_mode3;
    AP_Int8         flight_mode4;
    AP_Int8         flight_mode5;
    AP_Int8         flight_mode6;
    AP_Int8         simple_modes;
    AP_Int8         flight_mode_chan;
#endif 

    AP_Float                surface_depth;
    AP_Int8                 frame_configuration;

    AP_Float surface_max_throttle;

    // Note: keep initializers here in the same order as they are declared
    // above.
    Parameters()
    {
    }
};

/*
  2nd block of parameters, to avoid going past 256 top level keys
*/
class ParametersG2 {
public:
    ParametersG2(void);

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

#if HAL_PROXIMITY_ENABLED
    // proximity (aka object avoidance) library
    AP_Proximity proximity;
#endif

    // RC input channels
    RC_Channels_Sub rc_channels;

    // control over servo output ranges
    SRV_Channels servo_channels;

    AP_Float backup_origin_lat;
    AP_Float backup_origin_lon;
    AP_Float backup_origin_alt;
    AP_Float surface_nobaro_thrust;
    Actuators actuators;

};

extern const AP_Param::Info        var_info[];

// Sub-specific default parameters
static const struct AP_Param::defaults_table_struct defaults_table[] = {
    { "BRD_SAFETY_DEFLT",    0 },
    { "ARMING_CHECK",        uint32_t(AP_Arming::Check::RC) |
                             uint32_t(AP_Arming::Check::VOLTAGE) |
                             uint32_t(AP_Arming::Check::BATTERY)},
    { "CIRCLE_RATE",         2.0f},
    { "ATC_ACCEL_Y_MAX",     110000.0f},
    { "ATC_RATE_Y_MAX",      180.0f},
    { "RC3_TRIM",            1500},
    { "COMPASS_OFFS_MAX",    1000},
    { "INS_GYR_CAL",         0},
    { "RCMAP_ROLL",          2},
    { "RCMAP_PITCH",         1},
    { "RCMAP_FORWARD",       5},
    { "RCMAP_LATERAL",       6},
#if HAL_MOUNT_ENABLED
    { "MNT1_TYPE",           1},
    { "MNT1_DEFLT_MODE",     MAV_MOUNT_MODE_RC_TARGETING},
    { "MNT1_RC_RATE",        30},
#endif
    { "RC7_OPTION",          214},   // MOUNT1_YAW
    { "RC8_OPTION",          213},   // MOUNT1_PITCH
    { "MOT_PWM_MIN",         1100},
    { "MOT_PWM_MAX",         1900},
    { "PSC_JERK_Z",          50.0f},
    { "WPNAV_SPEED",         100.0f},
    { "PILOT_SPEED_UP",      100.0f},
    { "PSC_VELXY_P",         6.0f},
    { "EK3_SRC1_VELZ",       0},
#if AP_SUB_RC_ENABLED
    { "RC_PROTOCOLS",        0},
#endif
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIGATOR
    { "BATT_MONITOR",        4},
    { "BATT_CAPACITY",       0},
    { "LEAK1_PIN",           27},
    { "SCHED_LOOP_RATE",     200},
    { "SERVO13_FUNCTION",    181},   // k_lights1
    { "SERVO14_FUNCTION",    182},   // k_lights2
    { "SERVO16_FUNCTION",    7},     // k_mount_tilt
    { "SERVO16_REVERSED",    1},
#else
#if AP_BARO_PROBE_EXT_PARAMETER_ENABLED
    { "BARO_PROBE_EXT",      768},
#endif
    { "SERVO9_FUNCTION",     59},    // k_rcin9, lights 1
    { "SERVO10_FUNCTION",    7},     // k_mount_tilt
#endif
};
