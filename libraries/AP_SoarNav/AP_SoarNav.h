/*
  SoarNav native soaring planner
  Concept, design and implementation by Marco Robustini

  Provides an autonomous, energy-aware navigation layer for ArduPlane.
  SoarNav selects and manages GUIDED navigation targets, exploration
  strategies, thermal memory and re-engagement, terrain-aware routing
  and pilot override.

  SoarNav requires ArduPilot's native soaring support (SOAR) to be compiled,
  configured and enabled. It operates together with the SOAR controller,
  while leaving aircraft guidance, thermal centring and energy control
  to ArduPilot's native navigation, soaring and TECS systems.
*/

#pragma once

#include "AP_SoarNav_config.h"

#if HAL_SOARNAV_ENABLED

#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

class AP_SoarNav {
public:
    class Backend {
    public:
        enum class ModeNumber : uint8_t {
            UNKNOWN = 0,
            FBWB = 6,
            CRUISE = 7,
            RTL = 11,
            LOITER = 12,
            GUIDED = 15,
            THERMAL = 24,
        };

        virtual bool armed() const = 0;
        virtual bool is_flying() const = 0;
        virtual ModeNumber mode_number() const = 0;
        virtual ModeNumber previous_mode_number() const = 0;
        virtual bool set_guided_mode() = 0;
        virtual bool set_rtl_mode() = 0;
        virtual bool set_mode(ModeNumber mode) = 0;
        virtual bool set_guided_target(const Location &loc, bool terrain_evasion_target = false) = 0;
        virtual bool navigation_target(Location &loc) const = 0;
        virtual bool current_location(Location &loc) const = 0;
        virtual bool home_location(Location &loc) const = 0;
        virtual bool relative_position_ned_home(Vector3f &ned_m) const = 0;
        virtual bool terrain_height_amsl(const Location &loc, float &height_m) const = 0;
        virtual bool height_above_ground_m(float &height_m) const = 0;
        virtual bool wind_vector(Vector3f &wind_ned) const = 0;
        virtual bool velocity_ned(Vector3f &vel_ned) const = 0;
        virtual float ground_speed_mps() const = 0;
        virtual float climb_rate_mps() const = 0;
        virtual bool airspeed_estimate_mps(float &airspeed) const = 0;
        virtual float throttle_percent() const = 0;
        virtual bool motor_running() const = 0;
        virtual bool rpm_ok(float min_rpm) const = 0;
        virtual bool rpm_reading(float &rpm) const = 0;
        virtual float roll_input_norm() const = 0;
        virtual float roll_rad() const = 0;
        virtual float yaw_rad() const = 0;
        virtual float pitch_input_norm() const = 0;
        virtual float yaw_input_norm() const = 0;
        virtual bool soar_switch_active() const = 0;
        virtual bool autotune_active() const = 0;
        virtual bool soaring_active() const = 0;
        virtual bool soaring_throttle_suppressed() const = 0;
        virtual bool param_get_float(const char *name, float &value) const = 0;
        virtual bool param_set_float(const char *name, float value) = 0;
        virtual uint8_t rally_count() const = 0;
        virtual bool rally_location(uint8_t index, Location &loc) const = 0;
        virtual uint32_t rally_last_change_ms() const = 0;
        virtual void send_text(MAV_SEVERITY severity, const char *fmt, ...) const FMT_PRINTF(3, 4) = 0;
    };

    AP_SoarNav();

    static const AP_Param::GroupInfo var_info[];
    static const AP_Param::GroupInfo *var_info_visible;
    static bool set_parameter_visibility(bool visible);

    void update(Backend &backend);
    void on_enter_thermal(Backend &backend);
    void on_exit_thermal(Backend &backend);
    bool enabled() const { return _enable.get() == 1 && _radius_m.get() >= 0.0f; }
    bool has_target() const { return _target_valid; }
    bool running() const;
    bool get_target(Location &loc) const;
    void guided_override(Backend &backend);
    void stop(Backend &backend);
    void reset();

private:
    enum class State : uint8_t {
        IDLE,
        WAITING_FOR_ACTIVATION,
        NAVIGATING,
        PILOT_OVERRIDE,
        THERMAL_PAUSE,
        ERROR,
    };

    enum class EnergyState : uint8_t {
        NORMAL,
        LOW,
        CRITICAL,
    };

    enum class TerrainState : uint8_t {
        IDLE,
        MONITORING,
        EVADING,
        HOLD,
    };

    enum class TerrainDecision : uint8_t {
        CLEAR,
        MONITOR_ONLY,
        FORWARD_CORRIDOR_COMMIT,
        SIDE_ESCAPE_COMMIT,
        CRITICAL_OVERRIDE,
        EMERGENCY_BEST_OF_BAD,
        RECOVERY,
        RELEASE_TE,
    };

    enum class TerrainReplanReason : uint8_t {
        NONE,
        FORWARD_SAFE,
        MARGIN_LOW,
        IMMEDIATE_COLLISION,
        CORRIDOR_COLLAPSE,
        COMMITTED_TARGET_WORSE,
        SIDE_SWITCH_STRONG_GAIN,
        TERRAIN_DATA_INVALID,
        EMERGENCY_BEST_OF_BAD,
        PATH_SAFE_STABLE,
        DEFERRED_PRESSURE,
    };

    enum class TerrainCandidateKind : uint8_t {
        NONE,
        FORWARD_CORRIDOR,
        SIDE_ESCAPE,
        CRITICAL,
        BEST_OF_BAD,
        RECOVERY,
    };

    enum class NavGateReason : uint8_t {
        OK,
        MODE,
        SWITCH,
        AUTOTUNE,
        AREA,
        ALT_LOW,
        ALT_HIGH,
        NOT_FLYING,
    };

    struct GridCell {
        uint8_t visits;
        uint8_t terrain_fail_count;
        uint32_t terrain_block_until_ms;
        bool valid;
    };

    struct Hotspot {
        Location loc;
        uint32_t timestamp_ms;
        float avg_strength_mps;
        float max_strength_mps;
        Vector3f wind_ned;
        float entry_alt_m;
        float density;
        float consistency_score;
        uint32_t entry_time_ms;
        bool valid;
    };

    struct ThermalStats {
        bool active;
        Location start_loc;
        Location best_loc;
        bool best_loc_valid;
        uint32_t start_ms;
        uint32_t last_sample_ms;
        float accum_strength;
        float accum_weight;
        float max_strength;
        float min_strength;
        float samples[16];
        uint8_t sample_count;
    };

    struct TerrainEvasion {
        TerrainState state;
        uint32_t hold_until_ms;
        uint32_t next_check_ms;
        Location resume_target;
        Location evasion_target;
        char resume_source[32];
        bool resume_valid;
        bool evasion_target_valid;
        bool evasion_degraded;
        uint32_t evasion_target_updated_ms;
        float evasion_initial_distance_m;
        float evasion_min_agl_m;
        float last_safe_heading_deg;
        bool last_safe_heading_valid;
        int8_t last_turn_side;
        int8_t committed_turn_side;
        uint32_t commitment_started_ms;
        uint32_t last_commit_log_ms;
        uint32_t last_commit_hold_log_ms;
        float committed_bearing_deg;
        float committed_min_agl_m;
        float committed_first_threat_time_s;
        float committed_first_threat_distance_m;
        uint32_t last_debug_ms;
        uint8_t last_debug_key;
        uint32_t last_policy_log_ms;
        TerrainDecision last_policy_log_decision;
        TerrainReplanReason last_policy_log_reason;
        uint32_t last_side_reject_log_ms;
        uint8_t last_side_reject_log_key;
        uint8_t resume_safe_count;
        uint32_t post_resume_until_ms;
        float blocked_bearing_deg;
        uint32_t blocked_bearing_until_ms;
        bool blocked_bearing_valid;
        bool side_wall_active;
        int8_t wall_side;
        int8_t escape_side;
        uint32_t side_wall_hold_until_ms;
        uint32_t escape_started_ms;
        uint32_t last_wall_flip_ms;
        float escape_heading_deg;
        uint8_t side_wall_clear_count;
        int8_t pending_wall_side;
        uint8_t pending_wall_count;
        uint32_t reroute_block_until_ms;
        uint8_t hard_unsafe_count;
        uint32_t hard_unsafe_until_ms;
        uint32_t replan_not_before_ms;
        TerrainDecision last_decision;
        TerrainReplanReason last_replan_reason;
    };

    struct TerrainSideScan {
        bool valid;
        bool left_valid;
        bool right_valid;
        bool center_valid;
        float left_min_agl_m;
        float right_min_agl_m;
        float center_min_agl_m;
        float corridor_min_agl_m;
        float lateral_delta_m;
        float center_low_delta_m;
        float center_worst_frac;
        float center_first_threat_distance_m;
        float center_first_threat_time_s;
        float center_sample_distance_m;
        float center_terrain_delta_m;
        bool weak_asymmetry;
        bool uniform_low;
        int8_t wall_side;
        int8_t escape_side;
        float escape_bearing_deg;
    };

    struct TerrainProbe {
        float min_agl_m;
        float worst_frac;
        float target_distance_m;
        float sample_distance_m;
        float bearing_deg;
        float center_min_agl_m;
        float left_min_agl_m;
        float right_min_agl_m;
        float corridor_min_agl_m;
        float corridor_score;
        float speed_mps;
        float sink_mps;
        float vario_mps;
        float raw_climb_mps;
        float climb_credit_mps;
        float start_hagl_m;
        float terrain_start_m;
        float terrain_worst_m;
        float terrain_delta_m;
        float first_threat_distance_m;
        float first_threat_time_s;
    };

    struct TerrainPolicy {
        TerrainDecision decision;
        TerrainReplanReason reason;
        bool path_sampled;
        bool side_scan_valid;
        bool side_wall_detected;
        bool side_memory_active;
        bool immediate_threat;
        bool low_altitude_pressure;
        bool hard_unsafe_active;
        bool critical_override;
        bool turn_time_critical;
        bool front_clear;
        bool front_margin;
        bool side_wall_relevant;
        float buffer_m;
        float speed_mps;
        float yaw_deg;
        float target_bearing_deg;
        float target_distance_m;
        float turn_time_s;
        float threat_time_s;
        float current_min_agl_m;
        float current_hard_agl_m;
        float side_escape_bearing_deg;
        TerrainProbe probe;
        TerrainSideScan side_scan;
    };

    struct PolarEstimator {
        bool active;
        uint32_t last_fit_ms;
        uint32_t last_sample_ms;
        uint32_t last_commit_ms;
        uint32_t first_sample_ms;
        float s11;
        float s22;
        float s12;
        float y1;
        float y2;
        float n;
        float last_saved_cd0;
        float last_saved_b;
        float sink_bias;
        float err_ema;
        bool err_ema_valid;
        float v_ema;
        float v2_ema;
        float s_mag_log_ema;
        uint16_t good_samples;
        uint8_t stable_count;
        uint32_t last_debug_ms;
        bool learned;
    };

    struct RTLHState {
        bool active;
        bool engaged_guided;
        bool abort_until_next_rtl;
        Backend::ModeNumber last_mode;
        uint32_t t0_ms;
        float d0_m;
    };

    static constexpr uint8_t MAX_GRID_CELLS = 200;
    static constexpr uint8_t MAX_HOTSPOTS = 10;
    static constexpr uint8_t MAX_POLYGON_POINTS = 32;
    static constexpr uint8_t MAX_STREET_HOTSPOTS = 4;
    static constexpr float MIN_CELL_M = 50.0f;
    static constexpr uint32_t MAIN_LOOP_FAST_MS = 100;
    static constexpr uint32_t PARAM_CHECK_MS = 2000;
    static constexpr uint32_t RALLY_POLL_MS = 2000;
    static constexpr uint32_t VISITED_CELL_MS = 1500;
    static constexpr uint32_t GLIDE_CONE_MS = 20000;
    static constexpr uint32_t POLAR_LEARN_MS = 2000;
    static constexpr uint32_t TERRAIN_TARGET_LOG_MIN_MS = 60000;
    static constexpr uint32_t TERRAIN_CHECK_INTERVAL_MS = 2000;
    static constexpr uint32_t TERRAIN_DEBUG_LOG_MIN_MS = 3000;
    static constexpr uint32_t TERRAIN_POLICY_LOG_REPEAT_MS = 15000;
    static constexpr uint32_t TERRAIN_POLICY_LOG_CHANGE_MS = 5000;
    static constexpr uint32_t TERRAIN_SIDE_REJECT_LOG_MIN_MS = 20000;
    static constexpr uint8_t TERRAIN_SIDE_WALL_CLEAR_CHECKS = 3;
    static constexpr uint8_t TERRAIN_SIDE_WALL_FLIP_CONFIRM_CHECKS = 4;
    static constexpr uint8_t TERRAIN_SIDE_WALL_FLIP_HARD_CONFIRM_CHECKS = 3;
    static constexpr float TERRAIN_EMERGENCY_REVERSAL_DEG = 135.0f;
    static constexpr float TERRAIN_SIDE_WALL_HEADING_LOCK_DEG = 75.0f;
    static constexpr float TERRAIN_SIDE_WALL_ESCAPE_UPDATE_FRACTION = 0.75f;
    static constexpr float TERRAIN_MARGIN_LOW_MIN_FRACTION = 0.85f;
    static constexpr float TERRAIN_WALL_TURN_GUARD_DEG = 8.0f;
    static constexpr float TERRAIN_REROUTE_DURING_WALL_MAX_TURN_DEG = 45.0f;
    static constexpr uint8_t TERRAIN_RESUME_SAFE_CHECKS = 2;
    static constexpr uint8_t TERRAIN_RELEASE_SAFE_CHECKS = 3;
    static constexpr uint8_t TERRAIN_HARD_UNSAFE_CHECKS = 2;
    static constexpr float TERRAIN_HARD_UNSAFE_AGL_M = 0.0f;
    static constexpr uint32_t TERRAIN_BAD_CELL_BLOCK_MS = 180000;
    static constexpr uint32_t TERRAIN_BAD_TARGET_BLOCK_MS = 45000;
    static constexpr uint32_t TERRAIN_BAD_BEARING_BLOCK_MS = 120000;
    static constexpr uint8_t TERRAIN_BAD_CELL_FAIL_LIMIT = 1;
    static constexpr float TERRAIN_BAD_BEARING_HALF_WIDTH_DEG = 35.0f;
    static constexpr float TERRAIN_POST_EVASION_MAX_REVERSAL_DEG = 75.0f;
    static constexpr float TERRAIN_POST_EVASION_FAR_FACTOR = 1.8f;
    static constexpr float TERRAIN_REENTRY_NEAR_FRAC = 0.55f;
    static constexpr float TERRAIN_LOW_BUFFER_FRACTION = 0.65f;
    static constexpr float TERRAIN_CRITICAL_BUFFER_FRACTION = 0.45f;
    static constexpr float TERRAIN_EARLY_THREAT_FRACTION = 0.55f;
    static constexpr float TERRAIN_TURN_CLEARANCE_FRACTION = 0.55f;
    static constexpr uint32_t STUCK_PROGRESS_CHECK_MS = 20000;
    static constexpr uint32_t GUIDED_PROGRESS_ARM_DELAY_MS = 10000;
    static constexpr uint32_t GUIDED_TARGET_REFRESH_MS = 1000;
    static constexpr uint32_t GUIDED_MODE_REQUEST_TIMEOUT_MS = 3000;
    static constexpr uint32_t TARGET_LOG_REPEAT_MS = 60000;
    static constexpr uint32_t TARGET_SEARCH_RETRY_MS = 1000;
    static constexpr uint8_t RANDOM_FALLBACK_TRIES = 10;
    static constexpr uint32_t POST_THERMAL_TARGET_RESEND_MS = 5000;
    static constexpr uint32_t PILOT_RESUME_DELAY_MS = 5000;
    static constexpr uint32_t ACTIVATION_GRACE_MS = 2000;
    static constexpr uint32_t ACTIVATION_OVERRIDE_GRACE_MS = 3000;
    static constexpr uint32_t MOTOR_FAILURE_DELAY_MS = 5000;
    static constexpr uint32_t MOTOR_FAILURE_DEBOUNCE_MS = 1500;

    AP_Int8 _enable;
    AP_Int8 _log_level;
    AP_Int8 _auto_start;
    AP_Float _radius_m;
    AP_Float _wp_radius_m;
    AP_Int8 _thermal_memory_enable;
    AP_Int16 _thermal_memory_life_s;
    AP_Float _stuck_efficiency;
    AP_Int16 _stuck_time_s;
    AP_Int8 _reengage_dwell_s;
    AP_Float _retry_threshold;
    AP_Int16 _street_tolerance_deg;
    AP_Int8 _reroute_probability_pct;
    AP_Float _necessity_weight;
    AP_Float _thermal_memory_min_strength;
    AP_Float _focus_threshold;
    AP_Int16 _strategy_history_s;
    AP_Int16 _wp_timeout_s;
    AP_Int16 _reroute_min_deg;
    AP_Int16 _reroute_max_deg;
    AP_Int8 _dynamic_soar_alt;
    AP_Float _glide_cone_margin_m;
    AP_Float _glide_cone_pad_m;
    AP_Int16 _terrain_lookahead_s;
    AP_Int16 _terrain_buffer_min_m;

    State _state;
    State _last_announced_state;
    EnergyState _energy_state;
    Backend::ModeNumber _guided_entry_mode;
    Backend::ModeNumber _original_entry_mode;
    TerrainEvasion _terrain;
    ThermalStats _thermal;
    PolarEstimator _polar;
    RTLHState _rtlh;

    Location _target;
    Location _last_sent_target;
    Location _last_logged_target;
    Location _center;
    Location _dynamic_center;
    Location _focus_center;
    Location _last_cell_update_loc;
    Location _last_progress_loc;
    Location _reposition_resume_target;
    Location _reengage_final_target;
    Location _reengage_flyout_origin;
    bool _target_valid;
    bool _last_sent_valid;
    bool _last_logged_target_valid;
    bool _center_valid;
    bool _dynamic_center_valid;
    bool _focus_center_valid;
    bool _last_cell_update_valid;
    bool _last_progress_loc_valid;
    bool _reposition_resume_valid;
    bool _reengage_final_valid;
    bool _reengage_flyout_origin_valid;
    bool _external_guided_adopted;
    bool _external_guided_override_blocked;
    bool _has_been_activated;
    bool _activation_wait_notified;
    bool _manual_override_active;
    bool _roll_gesture_high;
    bool _pitch_gesture_high;
    int8_t _roll_gesture_dir;
    int8_t _pitch_gesture_dir;
    bool _using_polygon;
    bool _using_rally_points;
    bool _alt_gate_state;
    bool _grid_initialized;
    bool _grid_force_reinit;
    bool _force_grid_after_reset;
    bool _focus_mode;
    bool _reengage_hold_active;
    bool _reengage_flyout_active;
    bool _is_repositioning;
    bool _reroute_check_armed;
    bool _was_in_thermal_mode;
    bool _restored_on_disarm;
    bool _initial_soar_alts_valid;
    bool _motor_failure_check_active;
    bool _override_reset_done;
    bool _gcone_param_warning_sent;
    bool _polygon_is_convex;
    NavGateReason _nav_gate_reason;

    uint32_t _last_update_ms;
    uint32_t _last_param_check_ms;
    uint32_t _last_rally_poll_ms;
    uint32_t _last_visited_cell_ms;
    uint32_t _last_glide_cone_ms;
    uint32_t _gc_reset_hold_start_ms;
    uint32_t _last_gcone_log_ms;
    uint32_t _last_alt_timestamp_ms;
    uint32_t _energy_state_transition_ms;
    uint32_t _negative_trend_start_ms;
    uint32_t _last_polar_update_ms;
    uint32_t _last_target_sent_ms;
    uint32_t _last_target_log_ms;
    uint32_t _next_target_search_ms;
    uint32_t _guided_mode_requested_ms;
    uint32_t _post_thermal_resend_until_ms;
    uint32_t _progress_hold_until_ms;
    uint32_t _waypoint_start_ms;
    uint32_t _last_progress_check_ms;
    uint32_t _last_pilot_input_ms;
    uint32_t _activation_grace_start_ms;
    uint32_t _gesture_cooldown_until_ms;
    uint32_t _roll_gesture_start_ms;
    uint32_t _pitch_gesture_start_ms;
    uint32_t _focus_start_ms;
    uint32_t _reengage_hold_until_ms;
    uint32_t _reengage_flyout_start_ms;
    uint32_t _motor_on_start_ms;
    uint32_t _rpm_failure_start_ms;
    uint32_t _rally_signature_ms;
    uint32_t _last_used_hotspot_timestamp_ms;

    float _distance_to_wp_m;
    float _initial_distance_to_wp_m;
    float _last_progress_distance_m;
    float _initial_soar_alt_min_m;
    float _initial_soar_alt_max_m;
    float _initial_soar_alt_cutoff_m;
    float _altitude_at_motor_check_start_m;
    float _soar_alt_min_at_motor_check_m;
    float _filtered_alt_factor;
    mutable float _sink_best_ema;
    float _last_alt_m;
    float _last_gcone_logged_alt_m;
    float _effective_cluster_radius_m;
    float _reengage_flyout_initial_dist_m;
    float _last_area_radius_m;
    float _last_grid_wp_radius_m;
    float _grid_min_x_m;
    float _grid_min_y_m;
    float _grid_width_m;
    float _grid_height_m;
    float _last_logged_target_bearing_deg;
    float _last_logged_target_distance_m;

    uint8_t _stuck_counter;
    uint8_t _focus_wp_counter;
    uint8_t _focus_wp_timeout;
    uint8_t _lost_thermal_counter;
    uint8_t _pitch_gesture_count;
    uint8_t _roll_gesture_count;
    uint8_t _grid_rows;
    uint8_t _grid_cols;
    uint8_t _valid_cell_count;
    uint8_t _hotspot_count;
    uint8_t _polygon_count;
    int8_t _last_cell_index;
    uint32_t _rng_state;

    float _grid_cell_size_m;
    Vector2f _polygon_xy[MAX_POLYGON_POINTS];
    int32_t _polygon_lat_offsets[MAX_POLYGON_POINTS];
    int32_t _polygon_lng_offsets[MAX_POLYGON_POINTS];
    Location _polygon_points[MAX_POLYGON_POINTS];
    GridCell _grid[MAX_GRID_CELLS];
    uint8_t _valid_cells[MAX_GRID_CELLS];
    Hotspot _hotspots[MAX_HOTSPOTS];
    char _target_source[32];
    char _reposition_resume_source[32];
    char _last_logged_target_source[32];

    enum class MsgID : uint8_t {
        AREA_INFO_POLY = 1,
        PARAM_OUT_OF_RANGE = 2,
        GESTURE_ACTIVATED = 3,
        ALL_STRAT_FAILED = 4,
        AUTO_START = 5,
        AWAITING_ACTIVATION = 6,
        BASE_ALTS_INFO = 7,
        NO_THERMAL_LOC = 8,
        CELL_INDEX_FAIL = 9,
        DISARMED = 10,
        EFF_CLUSTER_RADIUS = 12,
        GRID_READY = 13,
        FATAL_NO_WP = 14,
        FOCUS_ON_DENSITY = 15,
        FORCING_NEW_TARGET = 16,
        GCONE_RST_OVERRIDE = 18,
        GCONE_OFF_NO_PARAMS = 19,
        GCONE_ALT_UPDATE = 20,
        GRID_TOO_SMALL = 21,
        GRID_INIT_FAIL_NO_CENTER = 22,
        GRID_INIT_STARTED = 23,
        GRID_VALIDATED = 24,
        GRID_SCAN_INFO = 25,
        INVALID_RC_MAPPING = 26,
        INVALID_SNAV_PARAMS = 27,
        MOTOR_FAIL_RTL = 29,
        NAV_COND_MET = 31,
        NAV_COND_NOT_MET = 32,
        NO_RPM_FALLBACK = 33,
        NO_POLY_DISABLED = 34,
        NO_PROG_WARN = 35,
        PARAM_CHANGED_REINIT = 36,
        PERSISTENT_LOC_ERROR = 37,
        PILOT_ACTIVATION = 38,
        PILOT_OVERRIDE = 39,
        RPM_SENSOR_SET = 46,
        RADIUS_MODE_ACTIVATED = 47,
        RALLY_CHANGED_REINIT = 48,
        REROUTE_SKIP_HEADING = 49,
        REPOSITIONED = 50,
        REROUTE_SKIP_NO_TARGET = 51,
        RESUMING_NAV = 52,
        SNAV_REROUTE_MINMAX = 53,
        SNAV_RADIUS_NEGATIVE = 54,
        AREA_INFO_RADIUS = 55,
        SCRIPT_INITIALIZED = 56,
        SCRIPT_DISABLED = 57,
        SET_CRUISE_FBWB = 58,
        STICK_CMD_MANUAL_OFF = 62,
        STICK_CMD_MANUAL_ON = 63,
        STICK_CMD_POLY_RECENTER = 64,
        STICK_CMD_RADIUS_RECENTER = 65,
        STICK_CMD_STUCK = 66,
        THERMAL_EXIT_NO_DATA = 67,
        THERMAL_FOUND_EXIT_FOCUS = 68,
        THERMAL_IGNORED_OOB = 69,
        POLY_FAILED_FALLBACK = 70,
        NAV_TARGET_SIMPLE = 71,
        NAV_TARGET_CELLS = 72,
        NAV_TARGET_REROUTE = 73,
        RTL_STALL_OVERRIDE = 74,
        RTL_OVERRIDE_RESUME = 75,
        NAV_TARGET_RIDGE = 76,
        POLAR_LEARN_UPDATE = 77,
        TERRAIN_AVOID_MANEUVER = 78,
        TERRAIN_AVOID_DEBUG = 79,
        TERRAIN_DATA_MISSING_WARN = 80,
        TERRAIN_AVOID_DEBUG_EXT = 81,
        TERRAIN_DATA_UNRELIABLE = 82,
        TERRAIN_STATUS = 83,
    };

    static const char *_msg_template(MsgID id);
    void _log_gcs(Backend &backend, MAV_SEVERITY severity, int8_t level, MsgID id, ...) const;
    bool _get_wind_vector(Backend &backend, Vector3f &wind) const;
    bool _generate_target_around_point(const Location &center, float radius_m, Location &target);
    float _wind_vector_to_bearing_deg(const Vector3f &wind) const;
    float _ridge_score_at_loc(Backend &backend, const Location &loc, float *ux_e = nullptr, float *uy_n = nullptr) const;
    bool _get_active_center_location(Backend &backend, Location &center) const;
    float _get_agl_m(Backend &backend, const Location &loc) const;
    void _lifecycle_coeff(float age_s, float life_s, float &distance_coeff, float &score_coeff) const;
    bool _loc_to_xy(const Location &loc, float &x, float &y) const;
    bool _xy_to_loc(float x, float y, Location &loc) const;
    static void _nearest_on_seg(float px, float py, float ax, float ay, float bx, float by, float &qx, float &qy, float &t);
    Location _clamp_inside_polygon(const Location &loc, float pad_m) const;
    bool _is_convex() const;
    void _prepare_polygon_xy_cache();
    Location _adjust_target_segment(const Location &current, const Location &target) const;
    float _sink_sample(Backend &backend) const;
    float _sink_best_now(Backend &backend) const;
    void _announce_polygon_area(Backend &backend) const;
    void _announce_radius_area(Backend &backend) const;
    float _calculate_thermal_variance(const float *samples, uint8_t count) const;
    bool _request_mode(Backend &backend, Backend::ModeNumber mode);
    float _nav_speed_mps(Backend &backend) const;
    bool _robust_hagl_and_amsl(Backend &backend, const Location &loc, float &hagl_m, float &amsl_m) const;
    bool _nav_altitude_check(Backend &backend, bool tevas_active, const Location &loc, float initial_min_m, float &hagl_m, float &amsl_m);
    bool _is_path_terrain_safe(Backend &backend, const Location &candidate) const;
    bool _te_probe_margin_low_ok(const TerrainProbe &probe, float buffer_m) const;
    bool _te_probe_immediate_threat(const TerrainProbe &probe, float buffer_m) const;
    bool _te_low_altitude_pressure(const TerrainProbe &probe, float buffer_m) const;
    float _te_roll_limit_rad(Backend &backend) const;
    float _te_track_or_yaw_deg(Backend &backend) const;
    float _te_planning_horizon_s(Backend &backend) const;
    float _te_turn_time_s(Backend &backend, float turn_deg) const;
    bool _te_probe_turn_time_critical(Backend &backend, const TerrainProbe &probe, float buffer_m, float turn_deg) const;
    float _te_turn_feasibility_penalty_m(Backend &backend, const TerrainProbe &probe, float turn_deg) const;
    float _te_turn_clearance_m(float buffer_m, float turn_deg) const;
    float _te_probe_risk_penalty_m(const TerrainProbe &probe, float buffer_m) const;
    bool _terrain_candidate_allowed(Backend &backend, const Location &from, const Location &candidate, const char *source, uint8_t grid_idx = 255);
    bool _terrain_public_nav_source(const char *source) const;
    bool _terrain_bearing_blocked(const Location &from, const Location &candidate, const char *source, uint32_t now_ms) const;
    bool _grid_cell_blocked_by_terrain(uint8_t idx, uint32_t now_ms) const;
    bool _external_guided_candidate(Backend &backend, const Location *loc = nullptr) const;
    bool _adopt_external_guided_target(Backend &backend, const Location &loc);
    void _release_external_guided_target();
    bool _restore_pilot_mode(Backend &backend);
    bool _check_tactical_reroute_conditions() const;

    void _reset_runtime(bool keep_activation);
    void _set_state(State state, Backend &backend, const char *reason = nullptr);
    bool _can_start_from_mode(Backend::ModeNumber mode) const;
    bool _is_operating_mode(Backend::ModeNumber mode) const;
    bool _owns_operating_mode() const;
    bool _has_owned_runtime() const;
    void _clear_navigation_state(bool clear_target);
    bool _runtime_gate_open(Backend &backend) const;
    const char *_nav_gate_reason_text(NavGateReason reason) const;
    bool _nav_gate_reason_is_transient(NavGateReason reason) const;
    bool _can_navigate(Backend &backend, Location &loc);
    void _validate_params(Backend &backend);
    void _update_area(Backend &backend, const Location &loc, bool force);
    bool _load_polygon(Backend &backend);
    bool _finalize_polygon_points();
    bool _point_in_area(const Location &loc) const;
    bool _point_in_polygon(const Location &loc) const;
    bool _segment_stays_inside(const Location &from, const Location &to) const;
    bool _clamp_inside_area(const Location &from, Location &candidate) const;
    void _init_grid(Backend &backend, const Location &loc, bool force);
    bool _loc_to_grid_index(const Location &loc, uint8_t &idx) const;
    bool _grid_index_to_location(uint8_t idx, Location &loc) const;
    void _update_visited_cell(const Location &loc);
    bool _select_next_target(Backend &backend, const Location &loc, Location &target, const char *&source);
    bool _try_grid_cell_target(Backend &backend, const Location &loc, uint8_t chosen, const char *source, Location &target);
    bool _select_grid_target(Backend &backend, const Location &loc, Location &target, const char *&source);
    bool _select_thermal_memory_target(Backend &backend, const Location &loc, Location &target, const char *&source);
    bool _select_focus_target(Backend &backend, const Location &loc, Location &target, const char *&source);
    bool _select_thermal_street_target(Backend &backend, const Location &loc, Location &target, const char *&source);
    bool _select_ridge_target(Backend &backend, const Location &loc, Location &target, const char *&source);
    bool _select_random_fallback(Backend &backend, const Location &loc, Location &target, const char *&source);
    bool _send_target(Backend &backend, const Location &loc, const char *source, bool force);
    void _make_guided_location(Backend &backend, Location &loc) const;
    bool _target_key_changed(const Location &loc, bool horizontal_only = false) const;
    bool _source_is_terrain_evasion(const char *source) const;
    bool _target_log_suppresses_cells(const char *source) const;
    bool _target_log_is_duplicate(const Location &loc, const char *source, uint32_t now_ms) const;
    void _log_target(Backend &backend, const Location &loc, const char *source);
    void _handle_idle(Backend &backend, const Location &loc, bool can_nav);
    void _handle_navigating(Backend &backend, const Location &loc, bool can_nav);
    void _handle_thermal(Backend &backend, const Location &loc);
    bool _terrain_evasion_update_during_thermal(Backend &backend, const Location &loc);
    void _handle_pilot_override(Backend &backend, const Location &loc);
    bool _pilot_input_active(Backend &backend) const;
    void _update_stick_gestures(Backend &backend, const Location &loc);
    void _recenter_area(Backend &backend, const Location &loc);
    void _sample_thermal(Backend &backend, const Location &loc);
    void _finish_thermal(Backend &backend, const Location &loc, bool weak_exit);
    void _add_hotspot(Backend &backend, const Location &loc, float avg_strength, float max_strength);
    void _clean_hotspots(uint32_t now_ms);
    void _update_hotspot_density();
    bool _predict_hotspot_drift(Backend &backend, const Hotspot &hotspot, Location &loc) const;
    void _begin_reengage(Backend &backend, const Location &loc, const Location &thermal_loc);
    bool _update_reengage(Backend &backend, const Location &loc, Location &target, const char *&source);
    bool _manage_waypoint_status(Backend &backend, const Location &loc);
    bool _manage_anti_stuck(Backend &backend, const Location &loc, Location &target, const char *&source);
    bool _maybe_tactical_reroute(Backend &backend, const Location &loc, Location &target, const char *&source);
    uint32_t _wp_timeout_ms(Backend &backend) const;
    void _reset_progress_monitor(uint32_t delay_ms);
    float _energy_factor(Backend &backend, const Location &loc);
    void _update_energy(Backend &backend, const Location &loc);
    void _update_dynamic_soar_alt(Backend &backend, const Location &loc);
    void _restore_initial_soar_alts(Backend &backend);
    void _store_initial_soar_alts(Backend &backend);
    bool _terrain_evasion_update(Backend &backend, const Location &loc, Location &target, const char *&source);
    TerrainPolicy _te_evaluate_policy(Backend &backend, const Location &loc, const Location &path_target, bool path_valid, const TerrainSideScan &side_scan, bool side_scan_valid, uint32_t now_ms) const;
    bool _te_select_policy_target(Backend &backend, const Location &loc, const TerrainPolicy &policy, Location &target, float &agl, float &dist, float &bearing_deg, TerrainCandidateKind &kind);
    void _te_commit_policy_target(Backend &backend, const Location &loc, const Location &selected, float selected_agl, float selected_dist, float selected_bearing, TerrainDecision decision, TerrainReplanReason reason, TerrainCandidateKind kind);
    const char *_te_decision_name(TerrainDecision decision) const;
    const char *_te_reason_name(TerrainReplanReason reason) const;
    const char *_te_candidate_kind_name(TerrainCandidateKind kind) const;
    const char *_te_context_code(const char *context) const;
    void _te_log_policy(Backend &backend, uint32_t now_ms, const TerrainPolicy &policy, TerrainCandidateKind kind, float selected_agl, float selected_dist, const char *context = nullptr);
    bool _te_force_emergency_target(Backend &backend, const Location &loc, Location &target, const char *&source, const char *reason);
    bool _te_bootstrap_target(Backend &backend, const Location &loc, Location &target, float *selected_min_agl_m = nullptr);
    bool _te_commit_hold_current_target(Backend &backend, const Location &loc, const TerrainPolicy *policy, Location &target, const char *&source, uint32_t now_ms);
    bool _te_policy_requires_immediate_replan(const TerrainPolicy &policy, float current_bearing_deg) const;
    bool _te_keep_current_evasion_target(Backend &backend, const Location &loc, const TerrainPolicy &policy, const Location &selected, float selected_agl, float selected_bearing, Location &target, const char *&source, uint32_t now_ms);
    bool _path_min_agl_probe(Backend &backend, const Location &start, const Location &target, float &min_agl, float &worst_frac, TerrainProbe *probe) const;
    float _te_sample_horizon_m(Backend &backend, float target_distance_m) const;
    float _te_lookahead_distance_m(Backend &backend) const;
    float _te_lever_min_time_s() const;
    float _te_guided_target_time_s() const;
    float _te_degraded_advance_time_s() const;
    float _te_climb_credit_min_mps(Backend &backend) const;
    float _te_climb_credit_max_mps(Backend &backend) const;
    float _te_time_scaled_distance_m(Backend &backend, float target_time_s, float min_time_s, float lookahead_multiplier) const;
    float _te_lever_min_m(Backend &backend) const;
    float _te_target_sample_limit_m(Backend &backend) const;
    float _te_candidate_min_m(Backend &backend) const;
    float _te_hold_allowed_drop_m(Backend &backend, float buffer_m) const;
    float _te_dynamic_margin_m(float buffer_m, float fraction) const;
    float _te_side_wall_flip_gain_m(float buffer_m) const;
    float _te_side_wall_trigger_margin_m(float buffer_m) const;
    float _te_side_wall_clear_margin_m(float buffer_m) const;
    float _te_margin_low_deficit_m(float buffer_m) const;
    float _te_entry_hysteresis_m(float buffer_m) const;
    float _te_entry_headroom_m(float buffer_m) const;
    float _te_flat_delta_m(float buffer_m) const;
    float _te_exit_margin_m(float buffer_m) const;
    float _te_reentry_terrain_delta_m(float buffer_m) const;
    void _te_save_resume_source();
    bool _te_side_wall_scan(Backend &backend, const Location &loc, float lever_m, TerrainSideScan &scan) const;
    bool _te_update_side_wall_memory(Backend &backend, uint32_t now_ms, const TerrainSideScan &scan);
    bool _te_wall_memory_active(uint32_t now_ms) const;
    bool _te_reroute_block_active(uint32_t now_ms) const;
    void _te_block_reroute(uint32_t now_ms, uint32_t duration_ms);
    bool _te_hard_unsafe_active(uint32_t now_ms) const;
    bool _te_note_hard_unsafe(Backend &backend, uint32_t now_ms, float min_agl_m, float buffer_m, const char *reason);
    void _te_note_hard_safe(uint32_t now_ms);
    bool _te_heading_turns_towards_wall(float heading_deg, float reference_heading_deg, uint32_t now_ms) const;
    int8_t _te_turn_side_from_bearing(Backend &backend, float bearing_deg) const;
    char _te_side_char(int8_t side) const;
    bool _terrain_candidate(Backend &backend, const Location &loc, float bearing_deg, float dist_m, Location &candidate) const;
    bool _te_make_candidate(Backend &backend, const Location &loc, float bearing_deg, float dist_m, Location &candidate) const;
    bool _te_select_corridor_best_effort(Backend &backend, const Location &loc, float reference_bearing_deg, float min_distance_m, bool allow_large_reversal, Location &candidate, float &agl, float &dist, float &bearing_deg) const;
    bool _te_select_escape_fan_target(Backend &backend, const Location &loc, const TerrainPolicy &policy, int8_t required_side, Location &candidate, float &agl, float &dist, float &bearing_deg) const;
    bool _te_select_critical_best_effort(Backend &backend, const Location &loc, const TerrainPolicy &policy, Location &candidate, float &agl, float &dist, float &bearing_deg) const;
    bool _te_select_forward_corridor(Backend &backend, const Location &loc, float min_distance_m, Location &candidate, float &agl, float &dist, float &bearing_deg) const;
    float _te_guided_target_distance(Backend &backend) const;
    float _te_degraded_target_distance(Backend &backend) const;
    bool _te_emergency_area_target(Backend &backend, const Location &loc, float target_bearing_deg, float lever_m, Location &candidate);
    bool _te_target_reached(Backend &backend, const Location &loc, const Location &target, float &distance_m) const;
    bool _te_target_usable(Backend &backend, const Location &loc, const Location &target, bool allow_degraded_hold) const;
    void _te_clear_state(bool clear_resume);
    float _terrain_buffer_m(Backend &backend) const;
    uint32_t _te_policy_base_ms() const;
    uint32_t _te_replan_backoff_ms() const;
    uint32_t _te_hold_min_ms() const;
    uint32_t _te_degraded_hold_min_ms() const;
    uint32_t _te_hold_max_ms() const;
    uint32_t _te_side_wall_hold_ms() const;
    uint32_t _te_side_wall_min_switch_ms() const;
    uint32_t _te_reentry_cooldown_ms() const;
    uint32_t _te_post_reroute_block_ms() const;
    uint32_t _te_hard_unsafe_hold_ms() const;
    void _update_polar_learning(Backend &backend, const Location &loc);
    void _update_motor_failure(Backend &backend, const Location &loc);
    void _update_rtlh(Backend &backend, const Location &loc);
    float _home_distance_m(Backend &backend, const Location &loc) const;
    float _alt_amsl_m(const Location &loc) const;
    float _alt_above_home_m(const Location &loc) const;
    uint32_t _rand();
    float _randf(float min_v, float max_v);
    static float _wrap_360(float deg);
    static float _wrap_180(float deg);
    static float _bearing_deg(const Location &from, const Location &to);
    static float _round_to(float v, float quantum);
    static const char *_compass(float bearing_deg);
};

#endif
