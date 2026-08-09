#include "AP_SoarNav.h"

#if HAL_SOARNAV_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

extern const AP_HAL::HAL &hal;

constexpr uint8_t AP_SoarNav::MAX_GRID_CELLS;
constexpr uint8_t AP_SoarNav::MAX_HOTSPOTS;
constexpr uint8_t AP_SoarNav::MAX_POLYGON_POINTS;
constexpr uint8_t AP_SoarNav::MAX_STREET_HOTSPOTS;
constexpr float AP_SoarNav::MIN_CELL_M;
constexpr uint32_t AP_SoarNav::MAIN_LOOP_FAST_MS;
constexpr uint32_t AP_SoarNav::PARAM_CHECK_MS;
constexpr uint32_t AP_SoarNav::RALLY_POLL_MS;
constexpr uint32_t AP_SoarNav::VISITED_CELL_MS;
constexpr uint32_t AP_SoarNav::GLIDE_CONE_MS;
constexpr uint32_t AP_SoarNav::POLAR_LEARN_MS;
constexpr uint32_t AP_SoarNav::TERRAIN_TARGET_LOG_MIN_MS;
constexpr uint32_t AP_SoarNav::TERRAIN_CHECK_INTERVAL_MS;
constexpr uint8_t AP_SoarNav::TERRAIN_SIDE_WALL_CLEAR_CHECKS;
constexpr uint8_t AP_SoarNav::TERRAIN_SIDE_WALL_FLIP_CONFIRM_CHECKS;
constexpr uint8_t AP_SoarNav::TERRAIN_SIDE_WALL_FLIP_HARD_CONFIRM_CHECKS;
constexpr float AP_SoarNav::TERRAIN_EMERGENCY_REVERSAL_DEG;
constexpr float AP_SoarNav::TERRAIN_SIDE_WALL_HEADING_LOCK_DEG;
constexpr float AP_SoarNav::TERRAIN_SIDE_WALL_ESCAPE_UPDATE_FRACTION;
constexpr float AP_SoarNav::TERRAIN_WALL_TURN_GUARD_DEG;
constexpr float AP_SoarNav::TERRAIN_REROUTE_DURING_WALL_MAX_TURN_DEG;
constexpr uint8_t AP_SoarNav::TERRAIN_RESUME_SAFE_CHECKS;
constexpr uint8_t AP_SoarNav::TERRAIN_HARD_UNSAFE_CHECKS;
constexpr float AP_SoarNav::TERRAIN_HARD_UNSAFE_AGL_M;
constexpr uint32_t AP_SoarNav::TERRAIN_BAD_CELL_BLOCK_MS;
constexpr uint32_t AP_SoarNav::TERRAIN_BAD_TARGET_BLOCK_MS;
constexpr uint32_t AP_SoarNav::TERRAIN_BAD_BEARING_BLOCK_MS;
constexpr uint8_t AP_SoarNav::TERRAIN_BAD_CELL_FAIL_LIMIT;
constexpr float AP_SoarNav::TERRAIN_BAD_BEARING_HALF_WIDTH_DEG;
constexpr float AP_SoarNav::TERRAIN_POST_EVASION_MAX_REVERSAL_DEG;
constexpr float AP_SoarNav::TERRAIN_POST_EVASION_FAR_FACTOR;
constexpr float AP_SoarNav::TERRAIN_REENTRY_NEAR_FRAC;
constexpr uint32_t AP_SoarNav::STUCK_PROGRESS_CHECK_MS;
constexpr uint32_t AP_SoarNav::GUIDED_PROGRESS_ARM_DELAY_MS;
constexpr uint32_t AP_SoarNav::GUIDED_TARGET_REFRESH_MS;
constexpr uint32_t AP_SoarNav::GUIDED_MODE_REQUEST_TIMEOUT_MS;
constexpr uint32_t AP_SoarNav::TARGET_LOG_REPEAT_MS;
constexpr uint32_t AP_SoarNav::TARGET_SEARCH_RETRY_MS;
constexpr uint8_t AP_SoarNav::RANDOM_FALLBACK_TRIES;
constexpr uint32_t AP_SoarNav::POST_THERMAL_TARGET_RESEND_MS;
constexpr uint32_t AP_SoarNav::PILOT_RESUME_DELAY_MS;
constexpr uint32_t AP_SoarNav::ACTIVATION_GRACE_MS;
constexpr uint32_t AP_SoarNav::ACTIVATION_OVERRIDE_GRACE_MS;
constexpr uint32_t AP_SoarNav::MOTOR_FAILURE_DELAY_MS;
constexpr uint32_t AP_SoarNav::MOTOR_FAILURE_DEBOUNCE_MS;

const AP_Param::GroupInfo AP_SoarNav::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: SoarNav native planner enable
    // @Description: Enables the native C++ SoarNav planner. The planner sends GUIDED Location targets. Values other than 0 or 1 are treated as disabled.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_SoarNav, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: LOG_LVL
    // @DisplayName: SoarNav log level
    // @Description: SoarNav GCS verbosity. 0 silent, 1 operational, 2 diagnostic.
    // @Values: 0:Silent,1:Operational,2:Debug
    // @User: Advanced
    AP_GROUPINFO("LOG_LVL", 2, AP_SoarNav, _log_level, 1),

    // @Param: RADIUS_M
    // @DisplayName: SoarNav radius
    // @Description: Circular operating radius in metres. Set to zero to use the polygon generated from Rally points stored on the flight controller.
    // @Units: m
    // @Range: 0 50000
    // @User: Advanced
    AP_GROUPINFO("RADIUS_M", 3, AP_SoarNav, _radius_m, 500),

    // @Param: WP_RADIUS
    // @DisplayName: SoarNav waypoint acceptance radius
    // @Description: Planner acceptance radius for GUIDED targets.
    // @Units: m
    // @Range: 10 300
    // @User: Advanced
    AP_GROUPINFO("WP_RADIUS", 4, AP_SoarNav, _wp_radius_m, 50),

    // @Param: TMEM_ENABLE
    // @DisplayName: SoarNav thermal memory enable
    // @Description: Enables thermal hotspot recording and revisits.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("TMEM_ENABLE", 5, AP_SoarNav, _thermal_memory_enable, 1),

    // @Param: TMEM_LIFE
    // @DisplayName: SoarNav thermal memory life
    // @Description: Hotspot lifetime.
    // @Units: s
    // @Range: 60 7200
    // @User: Advanced
    AP_GROUPINFO("TMEM_LIFE", 6, AP_SoarNav, _thermal_memory_life_s, 1200),

    // @Param: STUCK_EFF
    // @DisplayName: SoarNav stuck efficiency
    // @Description: Minimum path efficiency before anti-stuck repositioning reacts.
    // @Range: 0.1 1.0
    // @User: Advanced
    AP_GROUPINFO("STUCK_EFF", 7, AP_SoarNav, _stuck_efficiency, 0.35f),

    // @Param: STUCK_TIME
    // @DisplayName: SoarNav stuck grace time
    // @Description: Grace time after sending a new target before anti-stuck can activate.
    // @Units: s
    // @Range: 5 120
    // @User: Advanced
    AP_GROUPINFO("STUCK_TIME", 8, AP_SoarNav, _stuck_time_s, 30),

    // @Param: REENG_DWELL
    // @DisplayName: SoarNav re-engage dwell
    // @Description: Hold time for weak thermal re-engagement phases.
    // @Units: s
    // @Range: 0 120
    // @User: Advanced
    AP_GROUPINFO("REENG_DWELL", 9, AP_SoarNav, _reengage_dwell_s, 7),

    // @Param: RETRY_THR
    // @DisplayName: SoarNav weak thermal retry threshold
    // @Description: Score threshold for weak thermal retry.
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("RETRY_THR", 10, AP_SoarNav, _retry_threshold, 30),

    // @Param: STREET_TOL
    // @DisplayName: SoarNav thermal street tolerance
    // @Description: Angular tolerance for thermal-street detection.
    // @Units: deg
    // @Range: 5 90
    // @User: Advanced
    AP_GROUPINFO("STREET_TOL", 11, AP_SoarNav, _street_tolerance_deg, 30),

    // @Param: REROUTE_P
    // @DisplayName: SoarNav tactical reroute probability
    // @Description: Probability of tactical mid-route rerouting.
    // @Units: %
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("REROUTE_P", 12, AP_SoarNav, _reroute_probability_pct, 50),

    // @Param: NEC_WEIGHT
    // @DisplayName: SoarNav necessity weight
    // @Description: Low-energy necessity weight used in retry and thermal-memory selection.
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("NEC_WEIGHT", 13, AP_SoarNav, _necessity_weight, 50),

    // @Param: TMEM_MIN_S
    // @DisplayName: SoarNav minimum saved thermal strength
    // @Description: Minimum average thermal strength to save in memory.
    // @Units: m/s
    // @Range: 0 5
    // @User: Advanced
    AP_GROUPINFO("TMEM_MIN_S", 14, AP_SoarNav, _thermal_memory_min_strength, 0.2f),

    // @Param: FOCUS_THR
    // @DisplayName: SoarNav focus threshold
    // @Description: Hotspot density threshold that triggers focus mode.
    // @Range: 0 5
    // @User: Advanced
    AP_GROUPINFO("FOCUS_THR", 15, AP_SoarNav, _focus_threshold, 1.0f),

    // @Param: STRAT_HIST
    // @DisplayName: SoarNav strategy history
    // @Description: Strategy history window.
    // @Units: s
    // @Range: 60 3600
    // @User: Advanced
    AP_GROUPINFO("STRAT_HIST", 16, AP_SoarNav, _strategy_history_s, 900),

    // @Param: WP_TIMEOUT
    // @DisplayName: SoarNav target timeout
    // @Description: Base timeout for a GUIDED target.
    // @Units: s
    // @Range: 30 900
    // @User: Advanced
    AP_GROUPINFO("WP_TIMEOUT", 17, AP_SoarNav, _wp_timeout_s, 300),

    // @Param: REROUTE_MIN
    // @DisplayName: SoarNav reroute minimum angle
    // @Description: Minimum angular offset for tactical reroute candidates.
    // @Units: deg
    // @Range: 0 180
    // @User: Advanced
    AP_GROUPINFO("REROUTE_MIN", 18, AP_SoarNav, _reroute_min_deg, 80),

    // @Param: REROUTE_MAX
    // @DisplayName: SoarNav reroute maximum angle
    // @Description: Maximum angular offset for tactical reroute candidates.
    // @Units: deg
    // @Range: 0 180
    // @User: Advanced
    AP_GROUPINFO("REROUTE_MAX", 19, AP_SoarNav, _reroute_max_deg, 100),

    // @Param: DYN_SOALT
    // @DisplayName: SoarNav dynamic soaring altitude mode
    // @Description: 0 disabled, 1 linked glide cone, 2 MIN-driven glide cone with cutoff/max spacing protection, 3 target-based terrain evasion.
    // @Values: 0:Disabled,1:Linked glide-cone,2:MIN-driven glide-cone,3:Terrain evasion
    // @User: Advanced
    AP_GROUPINFO("DYN_SOALT", 20, AP_SoarNav, _dynamic_soar_alt, 0),

    // @Param: GC_MARGIN
    // @DisplayName: SoarNav glide cone margin
    // @Description: Safety margin added to calculated glide cone return altitude.
    // @Units: m
    // @Range: 0 150
    // @User: Advanced
    AP_GROUPINFO("GC_MARGIN", 21, AP_SoarNav, _glide_cone_margin_m, 25),

    // @Param: GC_PAD
    // @DisplayName: SoarNav glide cone pad
    // @Description: Extra altitude padding for return-home glide cone.
    // @Units: m
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("GC_PAD", 22, AP_SoarNav, _glide_cone_pad_m, 20),

    // @Param: TE_LOOK_S
    // @DisplayName: SoarNav terrain evasion lookahead
    // @Description: Terrain evasion lookahead time.
    // @Units: s
    // @Range: 5 60
    // @User: Advanced
    AP_GROUPINFO("TE_LOOK_S", 23, AP_SoarNav, _terrain_lookahead_s, 10),

    // @Param: TE_BUF_MIN
    // @DisplayName: SoarNav terrain evasion buffer
    // @Description: Minimum required AGL buffer for terrain evasion.
    // @Units: m
    // @Range: 40 150
    // @User: Advanced
    AP_GROUPINFO("TE_BUF_MIN", 24, AP_SoarNav, _terrain_buffer_min_m, 80),

    // @Param: AUTO_START
    // @DisplayName: SoarNav auto start
    // @Description: Starts SoarNav automatically when the SOAR switch, entry mode and area checks are valid, without requiring the initial roll gesture.
    // @Values: 0:Roll gesture required,1:Auto start
    // @User: Advanced
    AP_GROUPINFO("AUTO_START", 25, AP_SoarNav, _auto_start, 0),

    AP_GROUPEND
};

const AP_Param::GroupInfo *AP_SoarNav::var_info_visible = AP_SoarNav::var_info;

bool AP_SoarNav::set_parameter_visibility(bool visible)
{
    const AP_Param::GroupInfo *new_info = visible ? AP_SoarNav::var_info : nullptr;
    if (var_info_visible == new_info) {
        return false;
    }
    var_info_visible = new_info;
    AP_Param::invalidate_count();
    return true;
}

AP_SoarNav::AP_SoarNav()
{
    AP_Param::setup_object_defaults(this, var_info);
    reset();
}

void AP_SoarNav::reset()
{
    for (auto &cell : _grid) {
        cell = {};
    }
    memset(_valid_cells, 0, sizeof(_valid_cells));
    for (auto &hotspot : _hotspots) {
        hotspot = {};
    }
    for (auto &point : _polygon_points) {
        point = {};
    }
    memset(_polygon_lat_offsets, 0, sizeof(_polygon_lat_offsets));
    memset(_polygon_lng_offsets, 0, sizeof(_polygon_lng_offsets));
    for (auto &point_ne : _polygon_xy) {
        point_ne = {};
    }
    _thermal = {};
    _state = State::IDLE;
    _last_announced_state = State::ERROR;
    _energy_state = EnergyState::NORMAL;
    _guided_entry_mode = Backend::ModeNumber::UNKNOWN;
    _original_entry_mode = Backend::ModeNumber::UNKNOWN;
    _terrain.state = TerrainState::IDLE;
    _terrain.resume_valid = false;
    strncpy(_terrain.resume_source, "Unknown", sizeof(_terrain.resume_source));
    _terrain.resume_source[sizeof(_terrain.resume_source) - 1] = 0;
    _terrain.evasion_target_valid = false;
    _terrain.evasion_degraded = false;
    _terrain.evasion_target_updated_ms = 0;
    _terrain.evasion_initial_distance_m = 0.0f;
    _terrain.evasion_min_agl_m = 0.0f;
    _terrain.replan_not_before_ms = 0;
    _terrain.last_safe_heading_deg = 0.0f;
    _terrain.last_safe_heading_valid = false;
    _terrain.last_turn_side = 0;
    _terrain.committed_turn_side = 0;
    _terrain.commitment_started_ms = 0;
    _terrain.last_commit_log_ms = 0;
    _terrain.last_commit_hold_log_ms = 0;
    _terrain.committed_bearing_deg = 0.0f;
    _terrain.committed_min_agl_m = 0.0f;
    _terrain.committed_first_threat_time_s = -1.0f;
    _terrain.committed_first_threat_distance_m = -1.0f;
    _terrain.last_debug_ms = 0;
    _terrain.last_debug_key = 255;
    _terrain.last_policy_log_ms = 0;
    _terrain.last_policy_log_decision = TerrainDecision::CLEAR;
    _terrain.last_policy_log_reason = TerrainReplanReason::NONE;
    _terrain.last_side_reject_log_ms = 0;
    _terrain.last_side_reject_log_key = 255;
    _terrain.resume_safe_count = 0;
    _terrain.post_resume_until_ms = 0;
    _terrain.blocked_bearing_deg = 0.0f;
    _terrain.blocked_bearing_until_ms = 0;
    _terrain.blocked_bearing_valid = false;
    _terrain.side_wall_active = false;
    _terrain.wall_side = 0;
    _terrain.escape_side = 0;
    _terrain.side_wall_hold_until_ms = 0;
    _terrain.escape_started_ms = 0;
    _terrain.last_wall_flip_ms = 0;
    _terrain.escape_heading_deg = 0.0f;
    _terrain.side_wall_clear_count = 0;
    _terrain.pending_wall_side = 0;
    _terrain.pending_wall_count = 0;
    _terrain.reroute_block_until_ms = 0;
    _terrain.hard_unsafe_count = 0;
    _terrain.hard_unsafe_until_ms = 0;
    _terrain.replan_not_before_ms = 0;
    _terrain.last_decision = TerrainDecision::CLEAR;
    _terrain.last_replan_reason = TerrainReplanReason::NONE;
    _polygon_is_convex = false;
    _nav_gate_reason = NavGateReason::OK;
    _thermal.active = false;
    _thermal.best_loc_valid = false;
    _polar = {};
    _polar.active = true;
    _polar.learned = false;
    _rtlh = {};
    _target_valid = false;
    _last_sent_valid = false;
    _center_valid = false;
    _dynamic_center_valid = false;
    _focus_center_valid = false;
    _last_cell_update_valid = false;
    _last_progress_loc_valid = false;
    _reengage_final_valid = false;
    _reengage_flyout_origin_valid = false;
    _external_guided_adopted = false;
    _external_guided_override_blocked = false;
    _has_been_activated = false;
    _activation_wait_notified = false;
    _manual_override_active = false;
    _roll_gesture_high = false;
    _pitch_gesture_high = false;
    _roll_gesture_dir = 0;
    _pitch_gesture_dir = 0;
    _using_polygon = false;
    _using_rally_points = false;
    _alt_gate_state = false;
    _grid_initialized = false;
    _grid_force_reinit = true;
    _force_grid_after_reset = false;
    _focus_mode = false;
    _reengage_hold_active = false;
    _reengage_flyout_active = false;
    _is_repositioning = false;
    _reposition_resume_valid = false;
    _reposition_resume_source[0] = 0;
    _reroute_check_armed = false;
    _was_in_thermal_mode = false;
    _restored_on_disarm = false;
    _initial_soar_alts_valid = false;
    _motor_failure_check_active = false;
    _override_reset_done = false;
    _gcone_param_warning_sent = false;
    _last_update_ms = 0;
    _last_param_check_ms = 0;
    _last_rally_poll_ms = 0;
    _last_visited_cell_ms = 0;
    _last_glide_cone_ms = 0;
    _gc_reset_hold_start_ms = 0;
    _last_gcone_log_ms = 0;
    _last_alt_timestamp_ms = 0;
    _energy_state_transition_ms = 0;
    _negative_trend_start_ms = 0;
    _last_polar_update_ms = 0;
    _last_target_sent_ms = 0;
    _next_target_search_ms = 0;
    _guided_mode_requested_ms = 0;
    _post_thermal_resend_until_ms = 0;
    _progress_hold_until_ms = 0;
    _waypoint_start_ms = 0;
    _last_progress_check_ms = 0;
    _last_pilot_input_ms = 0;
    _activation_grace_start_ms = 0;
    _gesture_cooldown_until_ms = 0;
    _roll_gesture_start_ms = 0;
    _pitch_gesture_start_ms = 0;
    _focus_start_ms = 0;
    _reengage_hold_until_ms = 0;
    _reengage_flyout_start_ms = 0;
    _motor_on_start_ms = 0;
    _rpm_failure_start_ms = 0;
    _rally_signature_ms = 0;
    _last_used_hotspot_timestamp_ms = 0;
    _distance_to_wp_m = -1;
    _initial_distance_to_wp_m = -1;
    _last_progress_distance_m = -1;
    _initial_soar_alt_min_m = 0;
    _initial_soar_alt_max_m = 0;
    _initial_soar_alt_cutoff_m = 0;
    _altitude_at_motor_check_start_m = 0;
    _soar_alt_min_at_motor_check_m = 0;
    _filtered_alt_factor = 0;
    _sink_best_ema = 0.9f;
    _last_alt_m = 0;
    _last_gcone_logged_alt_m = 0;
    _effective_cluster_radius_m = 400;
    _reengage_flyout_initial_dist_m = 0.0f;
    _last_area_radius_m = -1.0f;
    _last_grid_wp_radius_m = -1.0f;
    _grid_min_x_m = 0.0f;
    _grid_min_y_m = 0.0f;
    _grid_width_m = 0.0f;
    _grid_height_m = 0.0f;
    _last_logged_target_bearing_deg = -1.0f;
    _last_logged_target_distance_m = -1.0f;
    _stuck_counter = 0;
    _focus_wp_counter = 0;
    _focus_wp_timeout = 3;
    _lost_thermal_counter = 0;
    _pitch_gesture_count = 0;
    _roll_gesture_count = 0;
    _grid_rows = 0;
    _grid_cols = 0;
    _valid_cell_count = 0;
    _hotspot_count = 0;
    _polygon_count = 0;
    _last_cell_index = -1;
    _rng_state = 0xA5A5C3D5U;
    strncpy(_target_source, "N/A", sizeof(_target_source));
    strncpy(_last_logged_target_source, "N/A", sizeof(_last_logged_target_source));
    _last_logged_target_valid = false;
    _last_target_log_ms = 0;
}

bool AP_SoarNav::get_target(Location &loc) const
{
    if (!_target_valid) {
        return false;
    }
    loc = _target;
    return true;
}

bool AP_SoarNav::running() const
{
    return _has_owned_runtime();
}

bool AP_SoarNav::_owns_operating_mode() const
{
    if (_external_guided_adopted) {
        return _terrain.state != TerrainState::IDLE || _last_sent_valid;
    }
    return _guided_entry_mode != Backend::ModeNumber::UNKNOWN ||
           _original_entry_mode != Backend::ModeNumber::UNKNOWN ||
           _target_valid ||
           _last_sent_valid ||
           _state == State::NAVIGATING ||
           _state == State::THERMAL_PAUSE ||
           _was_in_thermal_mode ||
           _thermal.active ||
           _terrain.state != TerrainState::IDLE ||
           _reengage_flyout_active ||
           _reengage_hold_active;
}

bool AP_SoarNav::_has_owned_runtime() const
{
    return _owns_operating_mode() ||
           _external_guided_adopted ||
           _state == State::WAITING_FOR_ACTIVATION ||
           _state == State::PILOT_OVERRIDE ||
           _state == State::ERROR ||
           _manual_override_active ||
           _rtlh.active ||
           _rtlh.engaged_guided;
}

void AP_SoarNav::_clear_navigation_state(bool clear_target)
{
    if (clear_target) {
        _target_valid = false;
        _last_sent_valid = false;
    }
    _external_guided_adopted = false;
    _guided_entry_mode = Backend::ModeNumber::UNKNOWN;
    _original_entry_mode = Backend::ModeNumber::UNKNOWN;
    _guided_mode_requested_ms = 0;
    _next_target_search_ms = 0;
    _post_thermal_resend_until_ms = 0;
    _progress_hold_until_ms = 0;
    _waypoint_start_ms = 0;
    _last_progress_check_ms = 0;
    _last_progress_loc_valid = false;
    _last_progress_distance_m = -1.0f;
    _initial_distance_to_wp_m = -1.0f;
    _reroute_check_armed = false;
    _stuck_counter = 0;
    _is_repositioning = false;
    _reposition_resume_valid = false;
    _reposition_resume_source[0] = 0;
    _reengage_final_valid = false;
    _reengage_hold_active = false;
    _reengage_flyout_active = false;
    _reengage_flyout_origin_valid = false;
    _reengage_flyout_initial_dist_m = 0.0f;
    _thermal.active = false;
    _was_in_thermal_mode = false;
    _rtlh.active = false;
    _rtlh.engaged_guided = false;
    _rtlh.abort_until_next_rtl = false;
    _te_clear_state(true);
}

void AP_SoarNav::guided_override(Backend &backend)
{
    const bool had_runtime = running();
    if (had_runtime) {
        _restore_initial_soar_alts(backend);
        _reset_runtime(false);
    }
    _external_guided_override_blocked = backend.mode_number() == Backend::ModeNumber::GUIDED ||
                                        backend.mode_number() == Backend::ModeNumber::THERMAL;
}

void AP_SoarNav::stop(Backend &backend)
{
    if (!running()) {
        return;
    }
    _restore_initial_soar_alts(backend);
    if (_rtlh.engaged_guided) {
        backend.set_rtl_mode();
    } else if (_owns_operating_mode() && !_external_guided_adopted) {
        _restore_pilot_mode(backend);
    }
    _reset_runtime(false);
}

const char *AP_SoarNav::_msg_template(MsgID id)
{
    switch (id) {
    case MsgID::AREA_INFO_POLY: return "%s A=%.2fkm2;RP=%u;MD=%.0fm";
    case MsgID::PARAM_OUT_OF_RANGE: return "%s out of range [%s..%s]: %.3f";
    case MsgID::GESTURE_ACTIVATED: return "Activation gesture received.";
    case MsgID::ALL_STRAT_FAILED: return "All strategies failed. Using fallback.";
    case MsgID::AUTO_START: return "Auto-start: gesture not required.";
    case MsgID::AWAITING_ACTIVATION: return "Awaiting pilot activation (ROLL gesture).";
    case MsgID::BASE_ALTS_INFO: return "Base Alts: MIN %.0f, CUTOFF %.0f, MAX %.0f";
    case MsgID::NO_THERMAL_LOC: return "Can't monitor thermal: no location.";
    case MsgID::CELL_INDEX_FAIL: return "Cell index fail, fallback.";
    case MsgID::DISARMED: return "Disarmed.";
    case MsgID::EFF_CLUSTER_RADIUS: return "Eff. Cluster Radius: %.0fm";
    case MsgID::GRID_READY: return "Exploration grid ready.";
    case MsgID::FATAL_NO_WP: return "FATAL: Could not set any valid waypoint.";
    case MsgID::FOCUS_ON_DENSITY: return "Focus Mode ON (Density)";
    case MsgID::FORCING_NEW_TARGET: return "Forcing new target search...";
    case MsgID::GCONE_RST_OVERRIDE: return "GC Rst(Override)->MIN:%.0f C:%.0f M:%.0f";
    case MsgID::GCONE_OFF_NO_PARAMS: return "GCone off: set POLAR_CD0/B & AIRSPEED.";
    case MsgID::GCONE_ALT_UPDATE: return "Glide Cone: Safe return alt now %.0fm";
    case MsgID::GRID_TOO_SMALL: return "Grid area too small. Init aborted.";
    case MsgID::GRID_INIT_FAIL_NO_CENTER: return "Grid init failed: no active center.";
    case MsgID::GRID_INIT_STARTED: return "Grid init started...";
    case MsgID::GRID_VALIDATED: return "Grid validated: %d cells.";
    case MsgID::GRID_SCAN_INFO: return "Grid: %d rows, %d cols. Scan cells...";
    case MsgID::INVALID_RC_MAPPING: return "Invalid RC channel mapping.";
    case MsgID::INVALID_SNAV_PARAMS: return "Invalid SNAV params. Disabling SoarNav.";
    case MsgID::MOTOR_FAIL_RTL: return "MOTOR FAILURE DETECTED! RTL ACTIVATED.";
    case MsgID::NAV_COND_MET: return "Navigation conditions met, starting.";
    case MsgID::NAV_COND_NOT_MET: return "Navigation conditions no longer met.";
    case MsgID::NO_RPM_FALLBACK: return "No RPM sensor, using climb-rate fallback.";
    case MsgID::NO_POLY_DISABLED: return "No Rally polygon. Add at least 3 Rally points or SoarNav cannot run.";
    case MsgID::NO_PROG_WARN: return "NoProg E%.2f | W%.1fm/s@%.0f";
    case MsgID::PARAM_CHANGED_REINIT: return "Parameter changed. Re-initializing area.";
    case MsgID::PERSISTENT_LOC_ERROR: return "Persistent location error, disabling.";
    case MsgID::PILOT_ACTIVATION: return "Pilot activation received.";
    case MsgID::PILOT_OVERRIDE: return "Pilot override detected.";
    case MsgID::RPM_SENSOR_SET: return "RPM sensor set, motor check uses RPM.";
    case MsgID::RADIUS_MODE_ACTIVATED: return "Radius Mode activated: %.0fm.";
    case MsgID::RALLY_CHANGED_REINIT: return "Rally points changed; reinit.";
    case MsgID::REROUTE_SKIP_HEADING: return "Re-route skipped: heading unavailable";
    case MsgID::REPOSITIONED: return "Repositioned. Re-engaging.";
    case MsgID::REROUTE_SKIP_NO_TARGET: return "Reroute skipped: no target.";
    case MsgID::RESUMING_NAV: return "Resuming navigation.";
    case MsgID::SNAV_REROUTE_MINMAX: return "SNAV REROUTE_MIN <= REROUTE_MAX.";
    case MsgID::SNAV_RADIUS_NEGATIVE: return "SNAV_RADIUS_M negative.";
    case MsgID::AREA_INFO_RADIUS: return "Radius A=%.2fkm2;RD=%.0fm";
    case MsgID::SCRIPT_INITIALIZED: return "Script initialized.";
    case MsgID::SCRIPT_DISABLED: return "Script disabled by user.";
    case MsgID::SET_CRUISE_FBWB: return "Set Cruise/FBWB to resume SoarNav.";
    case MsgID::STICK_CMD_MANUAL_OFF: return "Stick CMD: Manual override OFF.";
    case MsgID::STICK_CMD_MANUAL_ON: return "Stick CMD: Manual override ON.";
    case MsgID::STICK_CMD_POLY_RECENTER: return "Stick CMD: Polygon re-centered.";
    case MsgID::STICK_CMD_RADIUS_RECENTER: return "Stick CMD: Radius Area re-centered.";
    case MsgID::STICK_CMD_STUCK: return "Stuck! Repositioning %.0fm, offset %.0f";
    case MsgID::THERMAL_EXIT_NO_DATA: return "Thermal exit: no data. (Lost Thermal)";
    case MsgID::THERMAL_FOUND_EXIT_FOCUS: return "Thermal found, exiting focus.";
    case MsgID::THERMAL_IGNORED_OOB: return "Thermal ignored: out of area.";
    case MsgID::POLY_FAILED_FALLBACK: return "Rally polygon unavailable. Fallback disabled.";
    case MsgID::NAV_TARGET_SIMPLE: return "[%s] %.0fdeg%s %.0fm";
    case MsgID::NAV_TARGET_CELLS: return "[%s] %.0fdeg%s %.0fm %d/%d";
    case MsgID::NAV_TARGET_REROUTE: return "[Re-route] %.0fdeg%s %.0fm %d/%d";
    case MsgID::RTL_STALL_OVERRIDE: return "RTL Stall: Force direct Home route @%dm";
    case MsgID::RTL_OVERRIDE_RESUME: return "RTL Override: In home area. Loitering.";
    case MsgID::NAV_TARGET_RIDGE: return "[Ridge] %.0fdeg%s %.0fm s=%.2f";
    case MsgID::POLAR_LEARN_UPDATE: return "Polar Learn: CD0=%.4f B=%.4f";
    case MsgID::TERRAIN_AVOID_MANEUVER: return "TE path A%.0f<B%.0f";
    case MsgID::TERRAIN_AVOID_DEBUG: return "TE: Fa%.0f Ba%.0f B%.0f D%d L%.0f";
    case MsgID::TERRAIN_DATA_MISSING_WARN: return "TE data missing";
    case MsgID::TERRAIN_AVOID_DEBUG_EXT: return "TE: M%.0f Df%.0f R%.1f T%.0f S%d Z%d";
    case MsgID::TERRAIN_DATA_UNRELIABLE: return "TE data unreliable";
    case MsgID::TERRAIN_STATUS: return "TE %s";
    default: return "<missing SoarNav message>";
    }
}

void AP_SoarNav::_log_gcs(Backend &backend, MAV_SEVERITY severity, int8_t level, MsgID id, ...) const
{
    if (_log_level.get() < level) {
        return;
    }
    char body[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN + 1];
    va_list ap;
    va_start(ap, id);
    hal.util->vsnprintf(body, sizeof(body), _msg_template(id), ap);
    va_end(ap);
    backend.send_text(severity, "SoarNav: %s", body);
}

bool AP_SoarNav::_get_wind_vector(Backend &backend, Vector3f &wind) const
{
    if (!backend.wind_vector(wind) || wind.length() < 0.1f) {
        wind.zero();
        return false;
    }
    return true;
}

bool AP_SoarNav::_generate_target_around_point(const Location &center, float radius_m, Location &target)
{
    if (radius_m <= 0.0f || !center.initialised()) {
        return false;
    }
    target = center;
    target.offset_bearing(_randf(0.0f, 360.0f), sqrtf(_randf(0.0f, 1.0f)) * radius_m);
    return true;
}

float AP_SoarNav::_wind_to_bearing_deg(const Vector3f &wind) const
{
    if (wind.length() < 0.1f) {
        return 0.0f;
    }
    return _wrap_360(degrees(atan2f(wind.y, wind.x)));
}

float AP_SoarNav::_wind_from_bearing_deg(const Vector3f &wind) const
{
    if (wind.length() < 0.1f) {
        return 0.0f;
    }
    return _wrap_360(degrees(atan2f(-wind.y, -wind.x)));
}

float AP_SoarNav::_ridge_score_at_loc(Backend &backend, const Location &loc, float *ux_e, float *uy_n) const
{
    if (ux_e != nullptr) {
        *ux_e = 0.0f;
    }
    if (uy_n != nullptr) {
        *uy_n = 0.0f;
    }

    Vector3f wind;
    if (!_get_wind_vector(backend, wind) || wind.length() < 2.0f) {
        return 0.0f;
    }

    const float dx = constrain_float(_grid_cell_size_m > 0.0f ? _grid_cell_size_m : MIN_CELL_M, 30.0f, 120.0f);
    float h0 = 0.0f;
    float hE = 0.0f;
    float hN = 0.0f;
    Location pE = loc;
    Location pN = loc;
    pE.offset_bearing(90.0f, dx);
    pN.offset_bearing(0.0f, dx);
    if (!backend.terrain_height_amsl(loc, h0) || !backend.terrain_height_amsl(pE, hE) || !backend.terrain_height_amsl(pN, hN)) {
        return 0.0f;
    }

    const float gx_e = (hE - h0) / dx;
    const float gy_n = (hN - h0) / dx;
    const float gnorm = sqrtf(gx_e * gx_e + gy_n * gy_n);
    if (gnorm < 0.05f) {
        return 0.0f;
    }

    const float ux = gx_e / gnorm;
    const float uy = gy_n / gnorm;
    const float wnorm = MAX(1.0e-6f, sqrtf(wind.x * wind.x + wind.y * wind.y));
    const float wx = -wind.y / wnorm;
    const float wy = -wind.x / wnorm;
    const float facing = MAX(0.0f, ux * wx + uy * wy);
    const float slope = MIN(1.0f, gnorm / 0.25f);
    const float windgain = MIN(1.0f, wind.length() / 8.0f);

    if (ux_e != nullptr) {
        *ux_e = ux;
    }
    if (uy_n != nullptr) {
        *uy_n = uy;
    }

    return facing * (0.5f + 0.5f * slope) * windgain;
}

bool AP_SoarNav::_get_active_center_location(Backend &backend, Location &center) const
{
    if (_dynamic_center_valid) {
        center = _dynamic_center;
        return true;
    }
    if (_center_valid) {
        center = _center;
        return true;
    }
    return backend.home_location(center);
}

float AP_SoarNav::_get_agl_m(Backend &backend, const Location &loc) const
{
    float terr = 0.0f;
    if (backend.terrain_height_amsl(loc, terr)) {
        return MAX(1.0f, _alt_amsl_m(loc) - terr);
    }
    return MAX(1.0f, _alt_above_home_m(loc));
}

void AP_SoarNav::_lifecycle_coeff(float age_s, float life_s, float &distance_coeff, float &score_coeff) const
{
    const float life = MAX(60.0f, life_s);
    const float g = MIN(180.0f, life * 0.15f);
    const float m = MIN(600.0f, life * 0.50f);
    const float d = MAX(60.0f, life - g - m);
    if (age_s < g) {
        const float u = age_s / MAX(g, 1.0f);
        distance_coeff = 0.55f + 0.45f * u;
        score_coeff = 0.70f + 0.30f * u;
    } else if (age_s < g + m) {
        distance_coeff = 1.0f;
        score_coeff = 1.0f;
    } else {
        const float u = MIN(1.0f, (age_s - g - m) / d);
        distance_coeff = 1.0f + 0.50f * u;
        score_coeff = 1.0f + 0.60f * u;
    }
}

bool AP_SoarNav::_loc_to_xy(const Location &loc, float &x, float &y) const
{
    if (!_center_valid) {
        return false;
    }
    const Vector2f ne = _center.get_distance_NE(loc);
    x = ne.y;
    y = ne.x;
    return true;
}

bool AP_SoarNav::_xy_to_loc(float x, float y, Location &loc) const
{
    if (!_center_valid) {
        return false;
    }
    loc = _center;
    loc.offset(y, x);
    return true;
}

void AP_SoarNav::_nearest_on_seg(float px, float py, float ax, float ay, float bx, float by, float &qx, float &qy, float &t)
{
    const float abx = bx - ax;
    const float aby = by - ay;
    const float apx = px - ax;
    const float apy = py - ay;
    const float ab2 = abx * abx + aby * aby;
    t = ab2 > 0.0f ? constrain_float((apx * abx + apy * aby) / ab2, 0.0f, 1.0f) : 0.0f;
    qx = ax + t * abx;
    qy = ay + t * aby;
}

Location AP_SoarNav::_clamp_inside_polygon(const Location &loc, float pad_m) const
{
    if (!_using_polygon || _point_in_area(loc) || _polygon_count < 3) {
        return loc;
    }
    float px, py;
    if (!_loc_to_xy(loc, px, py)) {
        return loc;
    }
    float best_dx = 0.0f, best_dy = 0.0f, best_d = 1.0e12f;
    for (uint8_t i = 0; i < _polygon_count; i++) {
        const uint8_t j = (i + 1) % _polygon_count;
        float qx, qy, t;
        _nearest_on_seg(px, py, _polygon_xy[i].x, _polygon_xy[i].y, _polygon_xy[j].x, _polygon_xy[j].y, qx, qy, t);
        const float dx = px - qx;
        const float dy = py - qy;
        const float d = dx * dx + dy * dy;
        if (d < best_d) {
            best_d = d;
            best_dx = dx;
            best_dy = dy;
        }
    }
    const float len = sqrtf(best_d);
    float nx = 0.0f, ny = 0.0f;
    if (len > 1.0e-6f) {
        nx = best_dx / len;
        ny = best_dy / len;
    }
    Location out;
    if (_xy_to_loc(px - nx * (len + MAX(0.0f, pad_m) + 1.0f), py - ny * (len + MAX(0.0f, pad_m) + 1.0f), out) && _point_in_area(out)) {
        return out;
    }
    return loc;
}

bool AP_SoarNav::_is_convex() const
{
    if (_polygon_count < 4) {
        return true;
    }
    int8_t sign = 0;
    for (uint8_t i = 0; i < _polygon_count; i++) {
        const uint8_t i2 = (i + 1) % _polygon_count;
        const uint8_t i3 = (i + 2) % _polygon_count;
        const Vector2f a = _polygon_xy[i2] - _polygon_xy[i];
        const Vector2f b = _polygon_xy[i3] - _polygon_xy[i2];
        const float z = a.x * b.y - a.y * b.x;
        if (fabsf(z) > 1.0e-6f) {
            const int8_t s = z > 0.0f ? 1 : -1;
            if (sign == 0) {
                sign = s;
            } else if (s != sign) {
                return false;
            }
        }
    }
    return true;
}

void AP_SoarNav::_prepare_polygon_xy_cache()
{
    _polygon_is_convex = _is_convex();
}

Location AP_SoarNav::_adjust_target_segment(const Location &current, const Location &target) const
{
    if (_segment_stays_inside(current, target)) {
        return target;
    }
    float cx, cy, tx, ty;
    if (!_loc_to_xy(current, cx, cy) || !_loc_to_xy(target, tx, ty)) {
        return target;
    }
    const float ks[] = {0.8f, 0.6f, 0.4f, 0.2f};
    for (float k : ks) {
        Location cand;
        if (_xy_to_loc(cx + k * (tx - cx), cy + k * (ty - cy), cand) && _point_in_area(cand) && _segment_stays_inside(current, cand)) {
            return cand;
        }
    }
    const Location clamped = _clamp_inside_polygon(target, MAX(0.0f, _wp_radius_m.get()));
    return _segment_stays_inside(current, clamped) ? clamped : target;
}

float AP_SoarNav::_sink_sample(Backend &backend) const
{
    float smin = 0.3f;
    float smax = 3.0f;
    backend.param_get_float("TECS_SINK_MIN", smin);
    backend.param_get_float("TECS_SINK_MAX", smax);
    if (!isfinite(smin) || smin <= 0.0f) {
        smin = 0.3f;
    }
    if (!isfinite(smax) || smax < smin) {
        smax = 3.0f;
    }

    const float climb_rate = backend.climb_rate_mps();
    const bool still_air = climb_rate > -3.0f && climb_rate < 0.2f;
    const bool wings_level = fabsf(degrees(backend.roll_rad())) < 12.0f;
    const bool motor_off = backend.throttle_percent() <= 10.0f;
    if (still_air && wings_level && motor_off) {
        const float sample = constrain_float(-climb_rate, smin, smax);
        _sink_best_ema = 0.98f * _sink_best_ema + 0.02f * sample;
    }
    return constrain_float(_sink_best_ema, smin, smax);
}

float AP_SoarNav::_sink_best_now(Backend &backend) const
{
    return _sink_sample(backend);
}

void AP_SoarNav::_announce_polygon_area(Backend &backend) const
{
    if (_polygon_count < 3) {
        return;
    }
    float area2 = 0.0f;
    float max_dist = 0.0f;
    for (uint8_t i = 0; i < _polygon_count; i++) {
        const uint8_t j = (i + 1) % _polygon_count;
        area2 += _polygon_xy[i].x * _polygon_xy[j].y - _polygon_xy[j].x * _polygon_xy[i].y;
        for (uint8_t k = i + 1; k < _polygon_count; k++) {
            max_dist = MAX(max_dist, _polygon_points[i].get_distance(_polygon_points[k]));
        }
    }
    _log_gcs(backend, MAV_SEVERITY_INFO, 1, MsgID::AREA_INFO_POLY, _using_rally_points ? "Rally" : "Poly", fabsf(area2) * 0.5e-6f, unsigned(_polygon_count), (double)max_dist);
}

void AP_SoarNav::_announce_radius_area(Backend &backend) const
{
    const float r = MAX(0.0f, _radius_m.get());
    _log_gcs(backend, MAV_SEVERITY_INFO, 1, MsgID::AREA_INFO_RADIUS, 3.14159265358979323846f * r * r * 1.0e-6f, (double)r);
}

float AP_SoarNav::_calculate_thermal_variance(const float *samples, uint8_t count) const
{
    if (samples == nullptr || count == 0) {
        return 0.0f;
    }
    float sum = 0.0f;
    for (uint8_t i = 0; i < count; i++) {
        sum += samples[i];
    }
    const float mean = sum / count;
    float var = 0.0f;
    for (uint8_t i = 0; i < count; i++) {
        const float d = samples[i] - mean;
        var += d * d;
    }
    return var / count;
}

bool AP_SoarNav::_request_mode(Backend &backend, Backend::ModeNumber mode)
{
    switch (mode) {
    case Backend::ModeNumber::GUIDED:
        return backend.set_guided_mode();
    case Backend::ModeNumber::RTL:
        return backend.set_rtl_mode();
    case Backend::ModeNumber::FBWB:
    case Backend::ModeNumber::CRUISE:
    case Backend::ModeNumber::LOITER:
        return backend.set_mode(mode);
    default:
        return false;
    }
}

float AP_SoarNav::_nav_speed_mps(Backend &backend) const
{
    Vector3f vned;
    float ground_speed = backend.ground_speed_mps();
    if (backend.velocity_ned(vned)) {
        ground_speed = vned.xy().length();
    }

    float airspeed = 0.0f;
    float cruise = 15.0f;
    backend.param_get_float("AIRSPEED_CRUISE", cruise);

    float speed = cruise;
    if (backend.airspeed_estimate_mps(airspeed) && airspeed > 3.0f) {
        speed = airspeed;
    } else if (ground_speed > 1.0f) {
        speed = ground_speed;
    }

    if (!isfinite(cruise) || cruise <= 1.0f) {
        cruise = 15.0f;
    }

    const float min_speed = MAX(3.0f, cruise * 0.35f);
    const float max_speed = MAX(cruise * 2.0f, cruise + 15.0f);
    return constrain_float(speed, min_speed, max_speed);
}

bool AP_SoarNav::_robust_hagl_and_amsl(Backend &backend, const Location &loc, float &hagl_m, float &amsl_m) const
{
    Location home;
    Vector3f ned_home;
    if (!backend.home_location(home) || !backend.relative_position_ned_home(ned_home)) {
        return false;
    }

    const float rel_home_m = -ned_home.z;
    const float home_relative_amsl_m = home.alt * 0.01f + rel_home_m;
    const float loc_amsl_m = _alt_amsl_m(loc);
    amsl_m = isfinite(loc_amsl_m) ? loc_amsl_m : home_relative_amsl_m;
    if (!isfinite(amsl_m)) {
        amsl_m = home_relative_amsl_m;
    }

    float terr = 0.0f;
    const bool have_terrain = backend.terrain_height_amsl(loc, terr) && isfinite(terr);
    if (have_terrain) {
        const float terrain_hagl = amsl_m - terr;
        if (isfinite(terrain_hagl)) {
            hagl_m = terrain_hagl;
            return true;
        }
    }

    float hagl = 0.0f;
    const bool have_hagl = backend.height_above_ground_m(hagl) && isfinite(hagl);
    if (have_hagl && hagl >= -5.0f) {
        hagl_m = hagl;
        if (!isfinite(amsl_m) && have_terrain) {
            amsl_m = terr + hagl_m;
        }
        return true;
    }

    hagl_m = rel_home_m;
    return isfinite(hagl_m) && hagl_m > -5.0f && isfinite(amsl_m);
}

bool AP_SoarNav::_nav_altitude_check(Backend &backend, bool tevas_active, const Location &loc, float initial_min_m, float &hagl_m, float &amsl_m)
{
    if (tevas_active) {
        _alt_gate_state = true;
        _robust_hagl_and_amsl(backend, loc, hagl_m, amsl_m);
        return true;
    }

    const float min_alt = initial_min_m > 0.0f ? initial_min_m : 100.0f;
    const float dist2d = _home_distance_m(backend, loc);
    if (dist2d >= MAX(50.0f, min_alt * 4.0f)) {
        _alt_gate_state = true;
        _robust_hagl_and_amsl(backend, loc, hagl_m, amsl_m);
        return true;
    }

    const bool terrain_ok = _robust_hagl_and_amsl(backend, loc, hagl_m, amsl_m);
    const float rel_alt = _alt_above_home_m(loc);
    const float enter_thr = terrain_ok ? 40.0f : (initial_min_m > 0.0f ? initial_min_m - 2.0f : 20.0f);
    const float exit_thr = terrain_ok ? 25.0f : (initial_min_m > 0.0f ? initial_min_m - 10.0f : 10.0f);
    const float v = terrain_ok ? hagl_m : rel_alt;

    _alt_gate_state = _alt_gate_state ? (v > exit_thr) : (v > enter_thr);
    return _alt_gate_state;
}

bool AP_SoarNav::_te_probe_margin_low_ok(const TerrainProbe &probe, float buffer_m) const
{
    if (probe.min_agl_m >= buffer_m) {
        return true;
    }
    if (!isfinite(probe.min_agl_m) || probe.min_agl_m < TERRAIN_HARD_UNSAFE_AGL_M) {
        return false;
    }

    const float soft_floor_m = MAX(TERRAIN_HARD_UNSAFE_AGL_M + _te_margin_low_deficit_m(buffer_m),
                                   buffer_m * TERRAIN_MARGIN_LOW_MIN_FRACTION);
    if (probe.min_agl_m < soft_floor_m) {
        return false;
    }

    const float deficit_m = buffer_m - probe.min_agl_m;
    const float lateral_delta_m = fabsf(probe.left_min_agl_m - probe.right_min_agl_m);
    const float center_spread_m = fabsf(probe.center_min_agl_m - probe.corridor_min_agl_m);
    const float terrain_rise_m = MAX(0.0f, probe.terrain_delta_m);
    const float uniform_limit_m = MAX(_te_flat_delta_m(buffer_m) * 2.0f, _te_entry_hysteresis_m(buffer_m) * 3.0f);
    const float look_s = MAX(float(_terrain_lookahead_s.get()), 1.0f);
    const float first_threat_s = probe.first_threat_time_s;
    const bool threat_is_distant = first_threat_s < 0.0f ||
                                   first_threat_s >= MAX(look_s * 0.65f, _te_lever_min_time_s()) ||
                                   probe.worst_frac >= TERRAIN_REENTRY_NEAR_FRAC;
    const bool open_uniform = lateral_delta_m <= uniform_limit_m && center_spread_m <= uniform_limit_m;
    const bool terrain_stable = terrain_rise_m <= MAX(_te_flat_delta_m(buffer_m), buffer_m * 0.10f);
    const float allowed_deficit_m = MIN(_te_margin_low_deficit_m(buffer_m),
                                        MAX(_te_entry_hysteresis_m(buffer_m), buffer_m * 0.15f));
    const bool margin_only = deficit_m <= allowed_deficit_m;

    return open_uniform && terrain_stable && margin_only && threat_is_distant;
}

bool AP_SoarNav::_te_low_altitude_pressure(const TerrainProbe &probe, float buffer_m) const
{
    if (!isfinite(probe.min_agl_m) || !isfinite(probe.start_hagl_m)) {
        return false;
    }

    if (probe.min_agl_m < TERRAIN_HARD_UNSAFE_AGL_M) {
        return true;
    }

    const float look_s = MAX(float(_terrain_lookahead_s.get()), 1.0f);
    const bool low_now = probe.start_hagl_m < buffer_m * TERRAIN_LOW_BUFFER_FRACTION &&
                         probe.min_agl_m < buffer_m;
    const bool critical_path = probe.min_agl_m < buffer_m * TERRAIN_CRITICAL_BUFFER_FRACTION;
    const bool early_threat = probe.first_threat_time_s >= 0.0f &&
                              probe.first_threat_time_s <= MAX(look_s * TERRAIN_EARLY_THREAT_FRACTION,
                                                               _te_lever_min_time_s() * 0.75f);
    const bool rising_fast = probe.terrain_delta_m > MAX(_te_reentry_terrain_delta_m(buffer_m) * 0.50f, buffer_m * 0.20f);

    return critical_path || low_now || (early_threat && rising_fast);
}

float AP_SoarNav::_te_turn_clearance_m(float buffer_m, float turn_deg) const
{
    const float excess = constrain_float((fabsf(turn_deg) - 45.0f) / 90.0f, 0.0f, 1.0f);
    return buffer_m * TERRAIN_TURN_CLEARANCE_FRACTION * excess;
}

float AP_SoarNav::_te_probe_risk_penalty_m(const TerrainProbe &probe, float buffer_m) const
{
    float penalty = 0.0f;
    if (isfinite(probe.terrain_delta_m) && probe.terrain_delta_m > 0.0f) {
        penalty += probe.terrain_delta_m * 1.2f;
    }
    if (isfinite(probe.min_agl_m) && probe.min_agl_m < buffer_m) {
        penalty += (buffer_m - probe.min_agl_m) * 0.4f;
    }
    if (probe.first_threat_time_s >= 0.0f) {
        const float look_s = MAX(float(_terrain_lookahead_s.get()), 1.0f);
        const float early_s = MAX(look_s * 0.80f, _te_lever_min_time_s());
        if (probe.first_threat_time_s < early_s) {
            penalty += (early_s - probe.first_threat_time_s) * MAX(probe.speed_mps, 1.0f) * 0.35f;
        }
    }
    return penalty;
}

bool AP_SoarNav::_te_probe_immediate_threat(const TerrainProbe &probe, float buffer_m) const
{
    if (!isfinite(probe.min_agl_m)) {
        return false;
    }
    if (probe.min_agl_m < TERRAIN_HARD_UNSAFE_AGL_M) {
        return true;
    }
    if (probe.min_agl_m >= buffer_m && !_te_low_altitude_pressure(probe, buffer_m)) {
        return false;
    }
    if (_te_probe_margin_low_ok(probe, buffer_m)) {
        return false;
    }

    const float look_s = MAX(float(_terrain_lookahead_s.get()), 1.0f);
    const float t = probe.first_threat_time_s;
    const float deficit_m = buffer_m - probe.min_agl_m;
    return _te_low_altitude_pressure(probe, buffer_m) ||
           (t >= 0.0f &&
            t <= MAX(look_s * TERRAIN_EARLY_THREAT_FRACTION, _te_lever_min_time_s() * 0.75f) &&
            deficit_m >= _te_entry_hysteresis_m(buffer_m));
}

float AP_SoarNav::_te_roll_limit_rad(Backend &backend) const
{
    float roll_limit_cd = 0.0f;
    if (!backend.param_get_float("LIM_ROLL_CD", roll_limit_cd) || !isfinite(roll_limit_cd) || roll_limit_cd <= 0.0f) {
        roll_limit_cd = 3000.0f;
    }
    const float roll_limit_deg = constrain_float(roll_limit_cd * 0.01f, 10.0f, 60.0f);
    return radians(roll_limit_deg);
}

float AP_SoarNav::_te_track_or_yaw_deg(Backend &backend) const
{
    Vector3f vel;
    if (backend.velocity_ned(vel)) {
        const Vector2f ground = vel.xy();
        const float ground_speed = ground.length();
        const float min_track_speed = MAX(_nav_speed_mps(backend) * 0.10f, 1.0f);
        if (isfinite(ground_speed) && ground_speed > min_track_speed) {
            return _wrap_360(degrees(atan2f(ground.y, ground.x)));
        }
    }
    return _wrap_360(degrees(backend.yaw_rad()));
}

float AP_SoarNav::_te_turn_time_s(Backend &backend, float turn_deg) const
{
    const float speed = MAX(_nav_speed_mps(backend), 1.0f);
    const float bank_rad = _te_roll_limit_rad(backend);
    const float turn_rate_rad_s = GRAVITY_MSS * tanf(bank_rad) / speed;
    if (!isfinite(turn_rate_rad_s) || turn_rate_rad_s <= 0.01f) {
        return MAX(float(_terrain_lookahead_s.get()), 1.0f);
    }
    return radians(fabsf(turn_deg)) / turn_rate_rad_s;
}

float AP_SoarNav::_te_planning_horizon_s(Backend &backend) const
{
    const float configured_look_s = MAX(float(_terrain_lookahead_s.get()), 1.0f);
    const float configured_turn_deg = constrain_float(MAX(fabsf(float(_reroute_min_deg.get())),
                                                           fabsf(float(_reroute_max_deg.get()))),
                                                      TERRAIN_WALL_TURN_GUARD_DEG,
                                                      TERRAIN_EMERGENCY_REVERSAL_DEG);
    const float turn_s = _te_turn_time_s(backend, configured_turn_deg);
    if (!isfinite(turn_s) || turn_s <= 0.0f) {
        return configured_look_s;
    }
    return configured_look_s + MIN(turn_s, configured_look_s);
}

bool AP_SoarNav::_te_probe_turn_time_critical(Backend &backend, const TerrainProbe &probe, float buffer_m, float turn_deg) const
{
    if (probe.first_threat_time_s < 0.0f || !isfinite(probe.first_threat_time_s)) {
        return false;
    }
    if (!_te_low_altitude_pressure(probe, buffer_m) && !_te_probe_immediate_threat(probe, buffer_m) && probe.min_agl_m >= buffer_m) {
        return false;
    }
    const float scheduler_s = float(TERRAIN_CHECK_INTERVAL_MS) * 0.001f;
    const float reaction_s = MAX(scheduler_s, MAX(float(_terrain_lookahead_s.get()), 1.0f) * 0.15f);
    return probe.first_threat_time_s <= _te_turn_time_s(backend, turn_deg) + reaction_s;
}

float AP_SoarNav::_te_turn_feasibility_penalty_m(Backend &backend, const TerrainProbe &probe, float turn_deg) const
{
    if (probe.first_threat_time_s < 0.0f || !isfinite(probe.first_threat_time_s)) {
        return 0.0f;
    }
    const float deficit_s = _te_turn_time_s(backend, turn_deg) - probe.first_threat_time_s;
    if (deficit_s <= 0.0f) {
        return 0.0f;
    }
    return deficit_s * MAX(probe.speed_mps, 1.0f);
}

bool AP_SoarNav::_is_path_terrain_safe(Backend &backend, const Location &candidate) const
{
    if (_dynamic_soar_alt.get() != 3) {
        return true;
    }

    Location start;
    if (!backend.current_location(start)) {
        return false;
    }

    const float buffer = _terrain_buffer_m(backend);
    float hagl = 0.0f;
    float amsl = 0.0f;
    if (!_robust_hagl_and_amsl(backend, start, hagl, amsl)) {
        return false;
    }
    if (hagl < buffer * 0.75f) {
        return false;
    }

    float min_agl = 0.0f;
    float worst = 0.0f;
    TerrainProbe probe{};
    if (!_path_min_agl_probe(backend, start, candidate, min_agl, worst, &probe)) {
        return false;
    }

    if (min_agl >= buffer || _te_probe_margin_low_ok(probe, buffer)) {
        return true;
    }

    const float look_s = MAX(float(_terrain_lookahead_s.get()), 1.0f);
    const bool start_clear = probe.start_hagl_m >= buffer + _te_exit_margin_m(buffer);
    const bool distant_threat = probe.first_threat_time_s < 0.0f ||
                                 probe.first_threat_time_s > MAX(look_s * TERRAIN_EARLY_THREAT_FRACTION,
                                                                  _te_lever_min_time_s());
    const bool late_path_only = start_clear &&
                                distant_threat &&
                                probe.worst_frac >= TERRAIN_REENTRY_NEAR_FRAC &&
                                !_te_low_altitude_pressure(probe, buffer);
    return late_path_only;
}

bool AP_SoarNav::_terrain_public_nav_source(const char *source) const
{
    return source != nullptr &&
           (strcmp(source, "Pure") == 0 ||
            strcmp(source, "Guided") == 0 ||
            strcmp(source, "Random Fallback") == 0 ||
            strcmp(source, "Re-route") == 0);
}

bool AP_SoarNav::_grid_cell_blocked_by_terrain(uint8_t idx, uint32_t now_ms) const
{
    return idx < MAX_GRID_CELLS &&
           _grid[idx].valid &&
           _grid[idx].terrain_block_until_ms != 0 &&
           now_ms < _grid[idx].terrain_block_until_ms;
}

bool AP_SoarNav::_terrain_bearing_blocked(const Location &from, const Location &candidate, const char *source, uint32_t now_ms) const
{
    if (_dynamic_soar_alt.get() != 3 || !_terrain_public_nav_source(source)) {
        return false;
    }
    if (!_terrain.blocked_bearing_valid || _terrain.blocked_bearing_until_ms == 0 || now_ms >= _terrain.blocked_bearing_until_ms) {
        return false;
    }
    const float brg = _bearing_deg(from, candidate);
    return fabsf(_wrap_180(brg - _terrain.blocked_bearing_deg)) <= TERRAIN_BAD_BEARING_HALF_WIDTH_DEG;
}

bool AP_SoarNav::_terrain_candidate_allowed(Backend &backend, const Location &from, const Location &candidate, const char *source, uint8_t grid_idx)
{
    const uint32_t now = AP_HAL::millis();
    if (grid_idx < MAX_GRID_CELLS && _grid_cell_blocked_by_terrain(grid_idx, now)) {
        return false;
    }
    if (source != nullptr && strcmp(source, "Re-route") == 0 && _te_reroute_block_active(now)) {
        return false;
    }
    if (_terrain_bearing_blocked(from, candidate, source, now)) {
        return false;
    }
    if (!_is_path_terrain_safe(backend, candidate)) {
        return false;
    }
    if (_dynamic_soar_alt.get() != 3) {
        return true;
    }

    if (_terrain_public_nav_source(source) && _te_wall_memory_active(now)) {
        const float brg = _bearing_deg(from, candidate);
        const float yaw_deg = _te_track_or_yaw_deg(backend);
        const float turn = fabsf(_wrap_180(brg - yaw_deg));
        const bool reroute = source != nullptr && strcmp(source, "Re-route") == 0;
        if (_te_heading_turns_towards_wall(brg, yaw_deg, now) ||
            (reroute && turn > TERRAIN_REROUTE_DURING_WALL_MAX_TURN_DEG)) {
            return false;
        }
    }

    const float dist = from.get_distance(candidate);
    const float sample_dist = _te_sample_horizon_m(backend, dist);
    if (_terrain_public_nav_source(source) &&
        _terrain.last_safe_heading_valid &&
        _terrain.post_resume_until_ms > now &&
        dist > MAX(_te_lever_min_m(backend), sample_dist * TERRAIN_POST_EVASION_FAR_FACTOR)) {
        const float brg = _bearing_deg(from, candidate);
        if (fabsf(_wrap_180(brg - _terrain.last_safe_heading_deg)) > TERRAIN_POST_EVASION_MAX_REVERSAL_DEG) {
            return false;
        }
    }
    return true;
}

bool AP_SoarNav::_external_guided_candidate(Backend &backend, const Location *loc) const
{
    if (backend.mode_number() != Backend::ModeNumber::GUIDED) {
        return false;
    }
    if (_state == State::PILOT_OVERRIDE || _manual_override_active || _external_guided_override_blocked) {
        return false;
    }
    if (!backend.soar_switch_active() || backend.autotune_active() || !backend.soaring_active()) {
        return false;
    }
    if (_owns_operating_mode() && !_external_guided_adopted) {
        return false;
    }
    Location ext;
    if (!backend.navigation_target(ext) || !ext.initialised()) {
        return false;
    }
    Location current;
    const Location *check_loc = loc;
    if (check_loc == nullptr && backend.current_location(current)) {
        check_loc = &current;
    }
    if (check_loc != nullptr) {
        const float min_dist = MAX(80.0f, float(_wp_radius_m.get()) * 1.5f);
        if (check_loc->get_distance(ext) < min_dist) {
            return false;
        }
    }
    return true;
}

bool AP_SoarNav::_adopt_external_guided_target(Backend &backend, const Location &loc)
{
    if (!_external_guided_candidate(backend, &loc)) {
        return false;
    }

    Location ext;
    if (!backend.navigation_target(ext) || !ext.initialised()) {
        return false;
    }

    const float min_dist = MAX(80.0f, float(_wp_radius_m.get()) * 1.5f);
    if (_last_sent_valid && _last_sent_target.get_distance(ext) < min_dist) {
        return false;
    }
    if (_target_valid && _target.get_distance(ext) < min_dist) {
        return false;
    }

    _target = ext;
    _target_valid = true;
    _external_guided_adopted = true;
    strncpy(_target_source, "External Guided", sizeof(_target_source) - 1);
    _target_source[sizeof(_target_source) - 1] = 0;
    _distance_to_wp_m = loc.get_distance(_target);
    _initial_distance_to_wp_m = _distance_to_wp_m;
    _waypoint_start_ms = AP_HAL::millis();
    _last_progress_distance_m = _distance_to_wp_m;
    _last_progress_check_ms = _waypoint_start_ms;
    _last_sent_valid = false;
    return true;
}

void AP_SoarNav::_release_external_guided_target()
{
    if (!_external_guided_adopted) {
        return;
    }
    _external_guided_adopted = false;
    _target_valid = false;
    _last_sent_valid = false;
    _guided_entry_mode = Backend::ModeNumber::UNKNOWN;
    _original_entry_mode = Backend::ModeNumber::UNKNOWN;
    _guided_mode_requested_ms = 0;
    _next_target_search_ms = 0;
    _post_thermal_resend_until_ms = 0;
    _progress_hold_until_ms = 0;
    _waypoint_start_ms = 0;
    _last_progress_check_ms = 0;
    _last_progress_loc_valid = false;
    _last_progress_distance_m = -1.0f;
    _initial_distance_to_wp_m = -1.0f;
    _reroute_check_armed = false;
    _stuck_counter = 0;
    _is_repositioning = false;
    _reposition_resume_valid = false;
    _reposition_resume_source[0] = 0;
    _reengage_final_valid = false;
    _reengage_hold_active = false;
    _reengage_flyout_active = false;
    _reengage_flyout_origin_valid = false;
    _reengage_flyout_initial_dist_m = 0.0f;
    _thermal.active = false;
    _was_in_thermal_mode = false;
    _te_clear_state(true);
    _state = State::IDLE;
    _last_announced_state = State::ERROR;
}

bool AP_SoarNav::_restore_pilot_mode(Backend &backend)
{
    _restore_initial_soar_alts(backend);
    const Backend::ModeNumber mode = backend.mode_number();
    if (mode == Backend::ModeNumber::GUIDED || mode == Backend::ModeNumber::THERMAL) {
        if (!_owns_operating_mode()) {
            return true;
        }
        Backend::ModeNumber target_mode = _original_entry_mode;
        if (!_can_start_from_mode(target_mode)) {
            target_mode = _guided_entry_mode;
        }
        if (!_can_start_from_mode(target_mode)) {
            const Backend::ModeNumber previous = backend.previous_mode_number();
            target_mode = _can_start_from_mode(previous) ? previous : Backend::ModeNumber::CRUISE;
        }
        if (backend.set_mode(target_mode)) {
            _last_sent_valid = false;
            return true;
        }
        return false;
    }
    return true;
}

bool AP_SoarNav::_check_tactical_reroute_conditions() const
{
    const uint32_t now = AP_HAL::millis();
    if (_terrain.state != TerrainState::IDLE || _te_wall_memory_active(now) || _te_hard_unsafe_active(now) ||
        _te_reroute_block_active(now) ||
        (_terrain.post_resume_until_ms != 0 && now < _terrain.post_resume_until_ms)) {
        return false;
    }
    return _target_valid && !_is_repositioning && !_reengage_flyout_active && !_reengage_hold_active && !_manual_override_active && _reroute_probability_pct.get() > 0;
}

void AP_SoarNav::_reset_runtime(bool keep_activation)
{
    const bool activated = keep_activation && _has_been_activated;
    const bool manual_override = keep_activation && _manual_override_active;
    reset();
    _has_been_activated = activated;
    _manual_override_active = manual_override;
}

void AP_SoarNav::_set_state(State state, Backend &backend, const char *reason)
{
    if (_state == state) {
        return;
    }
    const State prev_state = _state;
    if (state == State::PILOT_OVERRIDE) {
        _restore_initial_soar_alts(backend);
        if (!_external_guided_adopted) {
            _restore_pilot_mode(backend);
        }
        _last_sent_valid = false;
    }
    if (state == State::IDLE && prev_state != State::THERMAL_PAUSE) {
        _restore_initial_soar_alts(backend);
        if (!_external_guided_adopted) {
            _restore_pilot_mode(backend);
        }
        _te_clear_state(true);
    }
    _state = state;
    if (_log_level.get() <= 0 || _last_announced_state == state) {
        return;
    }
    _last_announced_state = state;
    const char *name = "IDLE";
    switch (state) {
    case State::IDLE: name = "IDLE"; break;
    case State::WAITING_FOR_ACTIVATION: name = "WAITING"; break;
    case State::NAVIGATING: name = "NAV"; break;
    case State::PILOT_OVERRIDE: name = "PILOT"; break;
    case State::THERMAL_PAUSE: name = "THERMAL"; break;
    case State::ERROR: name = "ERROR"; break;
    }
    if (reason != nullptr) {
        backend.send_text(MAV_SEVERITY_INFO, "SoarNav: State %s (%s)", name, reason);
    } else {
        backend.send_text(MAV_SEVERITY_INFO, "SoarNav: State %s", name);
    }
}

bool AP_SoarNav::_can_start_from_mode(Backend::ModeNumber mode) const
{
    return mode == Backend::ModeNumber::CRUISE || mode == Backend::ModeNumber::FBWB;
}

bool AP_SoarNav::_is_operating_mode(Backend::ModeNumber mode) const
{
    return mode == Backend::ModeNumber::GUIDED || mode == Backend::ModeNumber::THERMAL;
}

bool AP_SoarNav::_runtime_gate_open(Backend &backend) const
{
    if (!backend.soar_switch_active() || backend.autotune_active()) {
        return false;
    }

    const Backend::ModeNumber mode = backend.mode_number();
    if (_can_start_from_mode(mode)) {
        return true;
    }

    if (_is_operating_mode(mode)) {
        return _owns_operating_mode() || _external_guided_candidate(backend, nullptr);
    }

    return false;
}

const char *AP_SoarNav::_nav_gate_reason_text(NavGateReason reason) const
{
    switch (reason) {
    case NavGateReason::OK: return "ok";
    case NavGateReason::MODE: return "mode";
    case NavGateReason::SWITCH: return "SOAR switch off";
    case NavGateReason::AUTOTUNE: return "autotune";
    case NavGateReason::AREA: return "area/grid";
    case NavGateReason::ALT_LOW: return "altitude gate low";
    case NavGateReason::ALT_HIGH: return "altitude above SOAR_ALT_MAX";
    case NavGateReason::NOT_FLYING: return "not flying";
    }
    return "unknown";
}

bool AP_SoarNav::_nav_gate_reason_is_transient(NavGateReason reason) const
{
    return reason == NavGateReason::ALT_LOW ||
           reason == NavGateReason::ALT_HIGH;
}

void AP_SoarNav::update(Backend &backend)
{
    const uint32_t now = AP_HAL::millis();
    if (_last_update_ms != 0 && now - _last_update_ms < MAIN_LOOP_FAST_MS) {
        return;
    }
    _last_update_ms = now;

    if (_radius_m.get() < 0.0f && _enable.get() == 1) {
        _validate_params(backend);
    }

    if (_enable.get() != 0 && _enable.get() != 1) {
        _enable.set(0);
        if (_log_level.get() > 0) {
            backend.send_text(MAV_SEVERITY_WARNING, "SoarNav: SNAV_ENABLE invalid; disabled.");
        }
    }

    if (!enabled()) {
        _external_guided_override_blocked = false;
        if (running()) {
            _restore_initial_soar_alts(backend);
            if (_owns_operating_mode() && !_external_guided_adopted) {
                _restore_pilot_mode(backend);
            }
            _reset_runtime(false);
        }
        return;
    }

    const Backend::ModeNumber mode = backend.mode_number();
    if (_external_guided_override_blocked && !_is_operating_mode(mode)) {
        _external_guided_override_blocked = false;
    }
    const bool switch_gate = backend.soar_switch_active() && !backend.autotune_active();
    if (!switch_gate) {
        if (running()) {
            _restore_initial_soar_alts(backend);
            if (_owns_operating_mode() && !_external_guided_adopted) {
                _restore_pilot_mode(backend);
            }
            _clear_navigation_state(true);
            _manual_override_active = false;
            _state = State::IDLE;
            _last_announced_state = State::ERROR;
        }
        return;
    }

    Location loc;
    const bool have_loc = backend.current_location(loc);
    if (!backend.armed()) {
        _external_guided_override_blocked = false;
        if (!_restored_on_disarm) {
            _restore_initial_soar_alts(backend);
            _restored_on_disarm = true;
        }
        _target_valid = false;
        _clear_navigation_state(true);
        _set_state(State::IDLE, backend, "disarmed");
        return;
    }
    _restored_on_disarm = false;

    if (!have_loc || !loc.initialised()) {
        _set_state(State::ERROR, backend, "no location");
        return;
    }

    if (now - _last_param_check_ms >= PARAM_CHECK_MS || _last_param_check_ms == 0) {
        _last_param_check_ms = now;
        _validate_params(backend);
        if (!enabled()) {
            if (running()) {
                _restore_initial_soar_alts(backend);
                if (_owns_operating_mode() && !_external_guided_adopted) {
                    _restore_pilot_mode(backend);
                }
                _reset_runtime(false);
            }
            return;
        }
    }

    const bool rtlh_gate = mode == Backend::ModeNumber::RTL || (_rtlh.engaged_guided && mode == Backend::ModeNumber::GUIDED);
    if (rtlh_gate) {
        _update_area(backend, loc, _grid_force_reinit);
        _grid_force_reinit = false;
        _update_rtlh(backend, loc);
    }

    if (!_runtime_gate_open(backend)) {
        if (_external_guided_adopted) {
            _release_external_guided_target();
            return;
        }
        if (_owns_operating_mode()) {
            _restore_initial_soar_alts(backend);
            _restore_pilot_mode(backend);
            _clear_navigation_state(true);
            _manual_override_active = false;
            _state = State::IDLE;
            _last_announced_state = State::ERROR;
        }
        return;
    }

    if (_guided_mode_requested_ms != 0 && mode != Backend::ModeNumber::GUIDED && mode != Backend::ModeNumber::THERMAL &&
        now - _guided_mode_requested_ms > GUIDED_MODE_REQUEST_TIMEOUT_MS) {
        _clear_navigation_state(true);
        _set_state(State::IDLE, backend, "guided timeout");
        return;
    }

    _store_initial_soar_alts(backend);
    _update_area(backend, loc, _grid_force_reinit);
    _init_grid(backend, loc, _grid_force_reinit);
    _grid_force_reinit = false;
    _update_energy(backend, loc);
    _update_stick_gestures(backend, loc);

    if (_state == State::PILOT_OVERRIDE || _manual_override_active) {
        _handle_pilot_override(backend, loc);
        return;
    }

    if (now - _last_visited_cell_ms >= VISITED_CELL_MS) {
        _last_visited_cell_ms = now;
        _update_visited_cell(loc);
    }
    _update_motor_failure(backend, loc);

    const bool in_thermal = mode == Backend::ModeNumber::THERMAL;
    const bool can_handle_thermal = !_manual_override_active &&
                                    _owns_operating_mode() &&
                                    (_state == State::NAVIGATING || _state == State::THERMAL_PAUSE || _thermal.active);
    if (in_thermal && !_was_in_thermal_mode && can_handle_thermal) {
        on_enter_thermal(backend);
    }
    if (!in_thermal && _was_in_thermal_mode) {
        on_exit_thermal(backend);
    }
    _was_in_thermal_mode = in_thermal && can_handle_thermal;

    bool can_nav = _can_navigate(backend, loc);
    if (can_nav && !_has_been_activated && _activation_grace_start_ms == 0) {
        _activation_grace_start_ms = now;
    }

    if (can_nav && _initial_soar_alts_valid && _state != State::PILOT_OVERRIDE && !_manual_override_active && backend.throttle_percent() < 10.0f && now - _last_glide_cone_ms >= GLIDE_CONE_MS) {
        _last_glide_cone_ms = now;
        _update_dynamic_soar_alt(backend, loc);
    }

    if (backend.armed() && _state == State::NAVIGATING && backend.throttle_percent() < 10.0f && mode != Backend::ModeNumber::THERMAL && now - _last_polar_update_ms >= POLAR_LEARN_MS) {
        _last_polar_update_ms = now;
        _update_polar_learning(backend, loc);
    }

    if (!can_nav && _nav_gate_reason == NavGateReason::NOT_FLYING) {
        if (_owns_operating_mode()) {
            _restore_initial_soar_alts(backend);
            _restore_pilot_mode(backend);
            _clear_navigation_state(true);
            _manual_override_active = false;
        }
        _set_state(State::IDLE, backend, _nav_gate_reason_text(_nav_gate_reason));
        return;
    }

    if (in_thermal) {
        if (_terrain_evasion_update_during_thermal(backend, loc)) {
            return;
        }
        _handle_thermal(backend, loc);
        return;
    }

    if (_state == State::THERMAL_PAUSE) {
        _set_state(State::NAVIGATING, backend, "thermal exit");
    }

    const bool activation_override_grace = _activation_grace_start_ms != 0 && now - _activation_grace_start_ms < ACTIVATION_OVERRIDE_GRACE_MS;
    const bool pilot_input_can_override = _state != State::WAITING_FOR_ACTIVATION;
    if ((_manual_override_active || (pilot_input_can_override && _pilot_input_active(backend))) && !activation_override_grace) {
        _handle_pilot_override(backend, loc);
        return;
    }

    switch (_state) {
    case State::IDLE:
    case State::WAITING_FOR_ACTIVATION:
    case State::ERROR:
        _handle_idle(backend, loc, can_nav);
        break;
    case State::NAVIGATING:
        _handle_navigating(backend, loc, can_nav);
        break;
    case State::PILOT_OVERRIDE:
        _handle_pilot_override(backend, loc);
        break;
    case State::THERMAL_PAUSE:
        break;
    }
}

bool AP_SoarNav::_can_navigate(Backend &backend, Location &loc)
{
    _nav_gate_reason = NavGateReason::OK;
    const Backend::ModeNumber mode = backend.mode_number();
    const bool operating_mode_ok = _is_operating_mode(mode) &&
                                   (_owns_operating_mode() || _external_guided_candidate(backend, &loc)) &&
                                   _state != State::PILOT_OVERRIDE;
    if (!_can_start_from_mode(mode) && !operating_mode_ok) {
        _nav_gate_reason = NavGateReason::MODE;
        return false;
    }
    if (!backend.soar_switch_active()) {
        _nav_gate_reason = NavGateReason::SWITCH;
        return false;
    }
    if (backend.autotune_active()) {
        _nav_gate_reason = NavGateReason::AUTOTUNE;
        return false;
    }
    if (!backend.is_flying()) {
        _nav_gate_reason = NavGateReason::NOT_FLYING;
        return false;
    }
    if (!_center_valid || !_grid_initialized) {
        _nav_gate_reason = NavGateReason::AREA;
        return false;
    }

    float alt_min = 0.0f;
    backend.param_get_float("SOAR_ALT_MIN", alt_min);
    const float initial_min = _initial_soar_alts_valid ? _initial_soar_alt_min_m : alt_min;
    float hagl = 0.0f;
    float amsl = 0.0f;
    const bool tevas_active = _dynamic_soar_alt.get() == 3 || _terrain.state != TerrainState::IDLE;
    if (!_nav_altitude_check(backend, tevas_active, loc, initial_min, hagl, amsl)) {
        _nav_gate_reason = NavGateReason::ALT_LOW;
        return false;
    }

    float alt_max = 0.0f;
    backend.param_get_float("SOAR_ALT_MAX", alt_max);
    const float alt_home = _alt_above_home_m(loc);
    if (!tevas_active && alt_max > 0.0f && alt_home > alt_max + 100.0f) {
        _nav_gate_reason = NavGateReason::ALT_HIGH;
        return false;
    }
    return true;
}

void AP_SoarNav::_handle_idle(Backend &backend, const Location &loc, bool can_nav)
{
    if (!can_nav) {
        _set_state(State::IDLE, backend, _nav_gate_reason_text(_nav_gate_reason));
        return;
    }

    const uint32_t now = AP_HAL::millis();
    if (_next_target_search_ms != 0 && now < _next_target_search_ms) {
        Location emergency;
        const char *emergency_source = "Terrain Evasion";
        if (_te_force_emergency_target(backend, loc, emergency, emergency_source, "target_retry")) {
            if (_send_target(backend, emergency, emergency_source, true)) {
                _next_target_search_ms = 0;
                _set_state(State::NAVIGATING, backend, "terrain evasion");
            }
            return;
        }
        _set_state(State::IDLE, backend, "target retry");
        return;
    }

    if (_adopt_external_guided_target(backend, loc)) {
        _next_target_search_ms = 0;
        _set_state(State::NAVIGATING, backend, "external guided");
        return;
    }

    if (!_has_been_activated) {
        if (_auto_start.get() == 0) {
            if (!_activation_wait_notified) {
                _log_gcs(backend, MAV_SEVERITY_NOTICE, 1, MsgID::AWAITING_ACTIVATION);
                _activation_wait_notified = true;
            }
            _set_state(State::WAITING_FOR_ACTIVATION, backend, "await activation");
            return;
        }
        if (_activation_grace_start_ms == 0) {
            _activation_grace_start_ms = AP_HAL::millis();
        }
        if (AP_HAL::millis() - _activation_grace_start_ms < ACTIVATION_GRACE_MS) {
            _set_state(State::WAITING_FOR_ACTIVATION, backend, "auto start grace");
            return;
        }
        _has_been_activated = true;
        if (!_activation_wait_notified) {
            _log_gcs(backend, MAV_SEVERITY_NOTICE, 1, MsgID::AUTO_START);
            _activation_wait_notified = true;
        }
    }

    Location target;
    const char *source = "Unknown";
    if (_dynamic_soar_alt.get() == 3) {
        float hagl = 0.0f;
        float amsl = 0.0f;
        const bool terrain_pressure = _terrain.state != TerrainState::IDLE ||
                                      _te_wall_memory_active(now) ||
                                      _te_hard_unsafe_active(now) ||
                                      (_robust_hagl_and_amsl(backend, loc, hagl, amsl) && hagl < _terrain_buffer_m(backend));
        if (terrain_pressure && _terrain_evasion_update(backend, loc, target, source)) {
            if (_send_target(backend, target, source, true)) {
                _next_target_search_ms = 0;
                _set_state(State::NAVIGATING, backend, "terrain evasion");
            }
            return;
        }
    }
    const bool selected = _select_next_target(backend, loc, target, source);
    if (!selected) {
        if (_dynamic_soar_alt.get() == 3 && _terrain_evasion_update(backend, loc, target, source)) {
            if (_send_target(backend, target, source, true)) {
                _next_target_search_ms = 0;
                _set_state(State::NAVIGATING, backend, "terrain evasion");
            }
            return;
        }
        if (_te_force_emergency_target(backend, loc, target, source, "no_target")) {
            if (_send_target(backend, target, source, true)) {
                _next_target_search_ms = 0;
                _set_state(State::NAVIGATING, backend, "terrain evasion");
            }
            return;
        }
        _next_target_search_ms = now + TARGET_SEARCH_RETRY_MS;
        _set_state(State::IDLE, backend, "no target");
        return;
    }
    if (_send_target(backend, target, source, true)) {
        _next_target_search_ms = 0;
        _set_state(State::NAVIGATING, backend, "start");
    }
}

void AP_SoarNav::_handle_navigating(Backend &backend, const Location &loc, bool can_nav)
{
    if (!can_nav) {
        const NavGateReason reason = _nav_gate_reason;
        if (_nav_gate_reason_is_transient(reason)) {
            if (_target_valid && AP_HAL::millis() - _last_target_sent_ms > GUIDED_TARGET_REFRESH_MS * 10U) {
                _send_target(backend, _target, _target_source, false);
            }
            return;
        }
        if (_external_guided_adopted) {
            _release_external_guided_target();
            return;
        }
        _restore_initial_soar_alts(backend);
        _restore_pilot_mode(backend);
        _clear_navigation_state(true);
        _set_state(State::IDLE, backend, _nav_gate_reason_text(reason));
        return;
    }

    Location target;
    const char *source = "Unknown";

    _adopt_external_guided_target(backend, loc);

    if (_external_guided_adopted) {
        if (_terrain_evasion_update(backend, loc, target, source)) {
            const bool egress = strcmp(source, "TE Egress") == 0;
            if (!_send_target(backend, target, source, true)) {
                _te_clear_state(false);
            } else if (egress) {
                _release_external_guided_target();
            }
            return;
        }
        return;
    }

    if (_terrain_evasion_update(backend, loc, target, source)) {
        if (!_send_target(backend, target, source, true)) {
            _te_clear_state(false);
        }
        return;
    }

    if (_update_reengage(backend, loc, target, source)) {
        if (!_send_target(backend, target, source, true)) {
            _reengage_hold_active = false;
            _reengage_flyout_active = false;
            _reengage_final_valid = false;
        }
        return;
    }
    if (_reengage_flyout_active || _reengage_hold_active) {
        if (_target_valid && AP_HAL::millis() - _last_target_sent_ms > GUIDED_TARGET_REFRESH_MS * 5U) {
            _send_target(backend, _target, _target_source, false);
        }
        return;
    }

    if (_manage_anti_stuck(backend, loc, target, source)) {
        if (_send_target(backend, target, source, true)) {
            return;
        }
    }

    if (_maybe_tactical_reroute(backend, loc, target, source)) {
        if (_send_target(backend, target, source, true)) {
            return;
        }
    }

    const bool reposition_target = _is_repositioning && strcmp(_target_source, "Anti-Stuck") == 0;
    const bool need_new = !_target_valid || _manage_waypoint_status(backend, loc);
    const bool post_thermal_refresh = _post_thermal_resend_until_ms != 0 && AP_HAL::millis() < _post_thermal_resend_until_ms;
    if (need_new) {
        const uint32_t now = AP_HAL::millis();
        if (_next_target_search_ms != 0 && now < _next_target_search_ms) {
            return;
        }
        if (reposition_target) {
            const bool reposition_reached = _distance_to_wp_m >= 0.0f && _distance_to_wp_m <= _wp_radius_m.get();
            _is_repositioning = false;
            _stuck_counter = 0;
            if (reposition_reached && _reposition_resume_valid) {
                target = _reposition_resume_target;
                source = _reposition_resume_source[0] != 0 ? _reposition_resume_source : "Resume";
                _reposition_resume_valid = false;
                _reposition_resume_source[0] = 0;
                if (_clamp_inside_area(loc, target) && _terrain_candidate_allowed(backend, loc, target, source)) {
                    if (_send_target(backend, target, source, true)) {
                        _next_target_search_ms = 0;
                        return;
                    }
                } else if (_log_level.get() > 0) {
                    backend.send_text(MAV_SEVERITY_NOTICE, "SoarNav: Anti-Stuck resume rejected");
                }
            }
            _reposition_resume_valid = false;
            _reposition_resume_source[0] = 0;
        }
        if (!_select_next_target(backend, loc, target, source)) {
            if (_dynamic_soar_alt.get() == 3 && _terrain_evasion_update(backend, loc, target, source)) {
                if (_send_target(backend, target, source, true)) {
                    _next_target_search_ms = 0;
                    return;
                }
            }
            if (_te_force_emergency_target(backend, loc, target, source, "no_target")) {
                if (_send_target(backend, target, source, true)) {
                    _next_target_search_ms = 0;
                    return;
                }
            }
            _next_target_search_ms = now + TARGET_SEARCH_RETRY_MS;
            _set_state(State::IDLE, backend, "no target");
            return;
        }
        if (_send_target(backend, target, source, true)) {
            _next_target_search_ms = 0;
            return;
        }
        _clear_navigation_state(true);
        _set_state(State::IDLE, backend, "target rejected");
        return;
    }

    if (_target_valid && (post_thermal_refresh || AP_HAL::millis() - _last_target_sent_ms > GUIDED_TARGET_REFRESH_MS * 10U)) {
        _send_target(backend, _target, _target_source, post_thermal_refresh);
    }
}

void AP_SoarNav::_handle_thermal(Backend &backend, const Location &loc)
{
    if (!_owns_operating_mode() || _manual_override_active) {
        return;
    }
    _set_state(State::THERMAL_PAUSE, backend, "thermal");
    _sample_thermal(backend, loc);
}

bool AP_SoarNav::_terrain_evasion_update_during_thermal(Backend &backend, const Location &loc)
{
    if (_dynamic_soar_alt.get() != 3 || _manual_override_active || !_owns_operating_mode()) {
        return false;
    }

    Location target;
    const char *source = "Terrain Evasion";
    if (!_terrain_evasion_update(backend, loc, target, source)) {
        return false;
    }

    _request_mode(backend, Backend::ModeNumber::GUIDED);
    if (_send_target(backend, target, source, true)) {
        _set_state(State::NAVIGATING, backend, "thermal terrain evasion");
    }
    return true;
}


bool AP_SoarNav::_te_force_emergency_target(Backend &backend, const Location &loc, Location &target, const char *&source, const char *reason)
{
    if (_dynamic_soar_alt.get() != 3 || !backend.is_flying()) {
        return false;
    }

    const uint32_t now = AP_HAL::millis();
    const float buffer = _terrain_buffer_m(backend);
    float hagl = 0.0f;
    float amsl = 0.0f;
    const bool hagl_ok = _robust_hagl_and_amsl(backend, loc, hagl, amsl);
    const bool hard_active = _te_hard_unsafe_active(now);
    const bool pressure = _terrain.state != TerrainState::IDLE ||
                          hard_active ||
                          (hagl_ok && hagl < buffer);
    if (!pressure) {
        return false;
    }

    TerrainSideScan side_scan{};
    const float lever = MAX(_te_lookahead_distance_m(backend), _te_lever_min_m(backend));
    const bool side_scan_valid = _te_side_wall_scan(backend, loc, lever, side_scan);
    if (side_scan_valid) {
        (void)_te_update_side_wall_memory(backend, now, side_scan);
    }

    Location path_target;
    bool path_valid = false;
    if (_terrain.resume_valid) {
        path_target = _terrain.resume_target;
        path_valid = true;
    } else if (_target_valid) {
        path_target = _target;
        path_valid = true;
    }

    TerrainPolicy policy = _te_evaluate_policy(backend, loc, path_target, path_valid, side_scan, side_scan_valid, now);
    if (!path_valid) {
        policy.decision = (hagl_ok && hagl < buffer * TERRAIN_CRITICAL_BUFFER_FRACTION) || hard_active ?
                          TerrainDecision::CRITICAL_OVERRIDE : TerrainDecision::EMERGENCY_BEST_OF_BAD;
        policy.reason = TerrainReplanReason::EMERGENCY_BEST_OF_BAD;
        policy.path_sampled = false;
        policy.buffer_m = buffer;
        policy.speed_mps = _nav_speed_mps(backend);
        policy.yaw_deg = _te_track_or_yaw_deg(backend);
        policy.target_bearing_deg = policy.yaw_deg;
        policy.target_distance_m = _te_degraded_target_distance(backend);
        policy.current_min_agl_m = hagl_ok ? hagl : -1.0e9f;
        policy.current_hard_agl_m = hagl_ok ? hagl : -1.0e9f;
        policy.side_scan_valid = side_scan_valid;
        policy.side_scan = side_scan;
        policy.side_wall_detected = side_scan_valid && side_scan.wall_side != 0;
        policy.side_memory_active = _te_wall_memory_active(now);
        policy.hard_unsafe_active = hard_active;
        policy.side_escape_bearing_deg = policy.side_memory_active && _terrain.wall_side != 0 ? _terrain.escape_heading_deg :
                                         (policy.side_wall_detected ? side_scan.escape_bearing_deg : policy.yaw_deg);
    }

    if (policy.decision == TerrainDecision::CLEAR || policy.decision == TerrainDecision::MONITOR_ONLY) {
        if (hagl_ok && hagl < buffer) {
            policy.decision = hagl < buffer * TERRAIN_CRITICAL_BUFFER_FRACTION ? TerrainDecision::CRITICAL_OVERRIDE : TerrainDecision::FORWARD_CORRIDOR_COMMIT;
            policy.reason = hagl < buffer * TERRAIN_CRITICAL_BUFFER_FRACTION ? TerrainReplanReason::IMMEDIATE_COLLISION : TerrainReplanReason::MARGIN_LOW;
        } else if (_terrain.evasion_target_valid && _te_target_usable(backend, loc, _terrain.evasion_target, true)) {
            target = _terrain.evasion_target;
            source = "Terrain Evasion";
            return true;
        } else {
            return false;
        }
    }

    Location selected;
    float selected_agl = -1.0e9f;
    float selected_dist = 0.0f;
    float selected_bearing = policy.yaw_deg;
    TerrainCandidateKind kind = TerrainCandidateKind::NONE;
    if (!_te_select_policy_target(backend, loc, policy, selected, selected_agl, selected_dist, selected_bearing, kind)) {
        if (_terrain.evasion_target_valid &&
            (now < _terrain.replan_not_before_ms || now < _terrain.hold_until_ms) &&
            _te_target_usable(backend, loc, _terrain.evasion_target, true)) {
            target = _terrain.evasion_target;
            source = "Terrain Evasion";
            return true;
        }
        return false;
    }

    if (_terrain.evasion_target_valid &&
        _te_keep_current_evasion_target(backend, loc, policy, selected, selected_agl, selected_bearing, target, source, now)) {
        return true;
    }

    _te_commit_policy_target(backend, loc, selected, selected_agl, selected_dist, selected_bearing,
                             policy.decision == TerrainDecision::FORWARD_CORRIDOR_COMMIT ? TerrainDecision::EMERGENCY_BEST_OF_BAD : policy.decision,
                             policy.reason == TerrainReplanReason::NONE ? TerrainReplanReason::EMERGENCY_BEST_OF_BAD : policy.reason,
                             kind == TerrainCandidateKind::NONE ? TerrainCandidateKind::BEST_OF_BAD : kind);
    target = selected;
    source = "Terrain Evasion";
    _te_log_policy(backend, now, policy, kind, selected_agl, selected_dist, reason);
    return true;
}


void AP_SoarNav::_handle_pilot_override(Backend &backend, const Location &loc)
{
    const uint32_t now = AP_HAL::millis();
    const bool pilot_input = _pilot_input_active(backend);

    if (_manual_override_active || pilot_input) {
        if (pilot_input) {
            _last_pilot_input_ms = now;
        }
        if (_external_guided_adopted) {
            _release_external_guided_target();
            return;
        }
        _set_state(State::PILOT_OVERRIDE, backend, _manual_override_active ? "manual override" : "stick input");
        if (!_override_reset_done) {
            _restore_initial_soar_alts(backend);
            _restore_pilot_mode(backend);
            _clear_navigation_state(true);
            _override_reset_done = true;
            if (!_manual_override_active && _log_level.get() > 0) {
                backend.send_text(MAV_SEVERITY_NOTICE, "SoarNav: Pilot override detected.");
            }
        }
        return;
    }

    if (_state == State::PILOT_OVERRIDE && _last_pilot_input_ms != 0 && now - _last_pilot_input_ms > PILOT_RESUME_DELAY_MS) {
        _override_reset_done = false;
        Location nav_loc = loc;
        if (!_can_navigate(backend, nav_loc)) {
            _clear_navigation_state(true);
            _set_state(State::IDLE, backend, "pilot resume wait");
            return;
        }
        _set_state(State::NAVIGATING, backend, "resume");
        _reset_progress_monitor(GUIDED_PROGRESS_ARM_DELAY_MS);
        if (_log_level.get() > 0) {
            backend.send_text(MAV_SEVERITY_INFO, "SoarNav: Resuming navigation.");
        }
        Location target;
        const char *source = "Unknown";
        if (_target_valid) {
            target = _target;
            source = _target_source;
        } else if (!_select_next_target(backend, loc, target, source)) {
            return;
        }
        _send_target(backend, target, source, true);
    }
}

bool AP_SoarNav::_pilot_input_active(Backend &backend) const
{
    const uint32_t now = AP_HAL::millis();
    if (_gesture_cooldown_until_ms != 0 && now < _gesture_cooldown_until_ms) {
        return false;
    }
    const float pitch = fabsf(backend.pitch_input_norm());
    const float yaw = fabsf(backend.yaw_input_norm());
    const float roll = fabsf(backend.roll_input_norm());
    return pitch > 0.05f || yaw > 0.05f || roll > 0.05f;
}

void AP_SoarNav::_update_stick_gestures(Backend &backend, const Location &loc)
{
    const Backend::ModeNumber mode = backend.mode_number();
    const bool roll_state_ok = _state == State::NAVIGATING ||
                               _state == State::PILOT_OVERRIDE ||
                               (_state == State::WAITING_FOR_ACTIVATION && _can_start_from_mode(mode));
    const bool pitch_state_ok = _state == State::PILOT_OVERRIDE && backend.soar_switch_active() && _can_start_from_mode(mode);

    if (!roll_state_ok) {
        _roll_gesture_start_ms = 0;
        _roll_gesture_count = 0;
        _roll_gesture_high = false;
        _roll_gesture_dir = 0;
    }
    if (!pitch_state_ok) {
        _pitch_gesture_start_ms = 0;
        _pitch_gesture_count = 0;
        _pitch_gesture_high = false;
        _pitch_gesture_dir = 0;
    }
    if (!roll_state_ok && !pitch_state_ok) {
        return;
    }

    const uint32_t now = AP_HAL::millis();
    const float roll = backend.roll_input_norm();
    const float pitch = backend.pitch_input_norm();
    const float gesture_threshold = 0.5f;

    if (roll_state_ok) {
        if (_roll_gesture_start_ms != 0 && now - _roll_gesture_start_ms > 2000U) {
            _roll_gesture_start_ms = 0;
            _roll_gesture_count = 0;
            _roll_gesture_high = false;
            _roll_gesture_dir = 0;
        }
        if (_roll_gesture_dir == 0) {
            if (fabsf(roll) > gesture_threshold) {
                _roll_gesture_start_ms = now;
                _roll_gesture_count = 1;
                _roll_gesture_dir = roll > 0.0f ? 1 : -1;
                _roll_gesture_high = roll > 0.0f;
            }
        } else if (_roll_gesture_dir > 0 && roll < -gesture_threshold) {
            _roll_gesture_count++;
            _roll_gesture_dir = -1;
            _roll_gesture_high = false;
        } else if (_roll_gesture_dir < 0 && roll > gesture_threshold) {
            _roll_gesture_count++;
            _roll_gesture_dir = 1;
            _roll_gesture_high = true;
        }
        if (_roll_gesture_count >= 4) {
            const bool activation_gesture = _state == State::WAITING_FOR_ACTIVATION && _can_start_from_mode(mode);
            _roll_gesture_count = 0;
            _roll_gesture_start_ms = 0;
            _roll_gesture_dir = 0;
            _roll_gesture_high = false;
            _gesture_cooldown_until_ms = now + ACTIVATION_OVERRIDE_GRACE_MS;
            if (activation_gesture) {
                _has_been_activated = true;
                _activation_grace_start_ms = now;
                backend.send_text(MAV_SEVERITY_NOTICE, "SoarNav: Pilot activation received.");
            } else {
                _manual_override_active = !_manual_override_active;
                if (_manual_override_active) {
                    _set_state(State::PILOT_OVERRIDE, backend, "manual override");
                } else {
                    _last_pilot_input_ms = now;
                    _override_reset_done = false;
                }
                backend.send_text(MAV_SEVERITY_INFO, "SoarNav: Stick CMD: Manual override %s.", _manual_override_active ? "ON" : "OFF");
            }
        }
    }

    if (pitch_state_ok) {
        if (_pitch_gesture_start_ms != 0 && now - _pitch_gesture_start_ms > 2000U) {
            _pitch_gesture_start_ms = 0;
            _pitch_gesture_count = 0;
            _pitch_gesture_high = false;
            _pitch_gesture_dir = 0;
        }
        if (_pitch_gesture_dir == 0) {
            if (fabsf(pitch) > gesture_threshold) {
                _pitch_gesture_start_ms = now;
                _pitch_gesture_count = 1;
                _pitch_gesture_dir = pitch > 0.0f ? 1 : -1;
                _pitch_gesture_high = pitch > 0.0f;
            }
        } else if (_pitch_gesture_dir > 0 && pitch < -gesture_threshold) {
            _pitch_gesture_count++;
            _pitch_gesture_dir = -1;
            _pitch_gesture_high = false;
        } else if (_pitch_gesture_dir < 0 && pitch > gesture_threshold) {
            _pitch_gesture_count++;
            _pitch_gesture_dir = 1;
            _pitch_gesture_high = true;
        }
        if (_pitch_gesture_count >= 4) {
            _pitch_gesture_count = 0;
            _pitch_gesture_start_ms = 0;
            _pitch_gesture_dir = 0;
            _pitch_gesture_high = false;
            _recenter_area(backend, loc);
        }
    }
}

void AP_SoarNav::_validate_params(Backend &backend)
{
    bool changed = false;
    bool disabled_for_safety = false;
    if (_enable.get() != 0 && _enable.get() != 1) { _enable.set(0); changed = true; disabled_for_safety = true; }
    if (_auto_start < 0) { _auto_start.set(0); changed = true; }
    if (_auto_start > 1) { _auto_start.set(1); changed = true; }
    if (_log_level < 0) { _log_level.set(0); changed = true; }
    if (_log_level > 2) { _log_level.set(2); changed = true; }
    if (_radius_m < 0) {
        _enable.set(0);
        _center_valid = false;
        _using_polygon = false;
        _using_rally_points = false;
        _grid_initialized = false;
        _grid_force_reinit = true;
        _target_valid = false;
        _last_sent_valid = false;
        _valid_cell_count = 0;
        _polygon_count = 0;
        _te_clear_state(true);
        changed = true;
        disabled_for_safety = true;
        backend.send_text(MAV_SEVERITY_ERROR, "SoarNav: SNAV_RADIUS_M negative; disabled.");
    }
    if (_wp_radius_m < 10) { _wp_radius_m.set(10); changed = true; }
    if (_wp_radius_m > 300) { _wp_radius_m.set(300); changed = true; }
    if (_thermal_memory_life_s < 60) { _thermal_memory_life_s.set(60); changed = true; }
    if (_stuck_efficiency < 0.1f) { _stuck_efficiency.set(0.1f); changed = true; }
    if (_stuck_efficiency > 1.0f) { _stuck_efficiency.set(1.0f); changed = true; }
    if (_stuck_time_s < 5) { _stuck_time_s.set(5); changed = true; }
    if (_stuck_time_s > 120) { _stuck_time_s.set(120); changed = true; }
    if (_reengage_dwell_s < 0) { _reengage_dwell_s.set(0); changed = true; }
    if (_reengage_dwell_s > 120) { _reengage_dwell_s.set(120); changed = true; }
    if (_retry_threshold < 0) { _retry_threshold.set(0); changed = true; }
    if (_retry_threshold > 100) { _retry_threshold.set(100); changed = true; }
    if (_street_tolerance_deg < 5) { _street_tolerance_deg.set(5); changed = true; }
    if (_street_tolerance_deg > 90) { _street_tolerance_deg.set(90); changed = true; }
    if (_reroute_probability_pct < 0) { _reroute_probability_pct.set(0); changed = true; }
    if (_reroute_probability_pct > 100) { _reroute_probability_pct.set(100); changed = true; }
    if (_necessity_weight < 0.0f) { _necessity_weight.set(0.0f); changed = true; }
    if (_necessity_weight > 100.0f) { _necessity_weight.set(100.0f); changed = true; }
    if (_thermal_memory_min_strength < 0.0f) { _thermal_memory_min_strength.set(0.0f); changed = true; }
    if (_thermal_memory_min_strength > 5.0f) { _thermal_memory_min_strength.set(5.0f); changed = true; }
    if (_focus_threshold < 0.0f) { _focus_threshold.set(0.0f); changed = true; }
    if (_focus_threshold > 5.0f) { _focus_threshold.set(5.0f); changed = true; }
    if (_strategy_history_s < 60) { _strategy_history_s.set(60); changed = true; }
    if (_strategy_history_s > 3600) { _strategy_history_s.set(3600); changed = true; }
    if (_wp_timeout_s < 30) { _wp_timeout_s.set(30); changed = true; }
    if (_wp_timeout_s > 900) { _wp_timeout_s.set(900); changed = true; }
    if (_reroute_min_deg > _reroute_max_deg) {
        const int16_t min_v = _reroute_min_deg.get();
        _reroute_min_deg.set(_reroute_max_deg.get());
        _reroute_max_deg.set(min_v);
        changed = true;
    }
    if (_dynamic_soar_alt < 0) { _dynamic_soar_alt.set(0); changed = true; }
    if (_dynamic_soar_alt > 3) { _dynamic_soar_alt.set(3); changed = true; }
    if (_glide_cone_margin_m < 0.0f) { _glide_cone_margin_m.set(0.0f); changed = true; }
    if (_glide_cone_margin_m > 150.0f) { _glide_cone_margin_m.set(150.0f); changed = true; }
    if (_glide_cone_pad_m < 0.0f) { _glide_cone_pad_m.set(0.0f); changed = true; }
    if (_glide_cone_pad_m > 100.0f) { _glide_cone_pad_m.set(100.0f); changed = true; }
    if (_terrain_lookahead_s < 5) { _terrain_lookahead_s.set(5); changed = true; }
    if (_terrain_lookahead_s > 60) { _terrain_lookahead_s.set(60); changed = true; }
    if (_terrain_buffer_min_m < 40) { _terrain_buffer_min_m.set(40); changed = true; }
    if (_terrain_buffer_min_m > 150) { _terrain_buffer_min_m.set(150); changed = true; }
    if (changed && _log_level.get() > 0) {
        backend.send_text(disabled_for_safety ? MAV_SEVERITY_ERROR : MAV_SEVERITY_WARNING,
                          disabled_for_safety ? "SoarNav: unsafe SNAV params; disabled." : "SoarNav: SNAV params clamped.");
    }
}

void AP_SoarNav::_update_area(Backend &backend, const Location &loc, bool force)
{
    const uint32_t now = AP_HAL::millis();
    if (_radius_m.get() < 0.0f) {
        _center_valid = false;
        _using_polygon = false;
        _using_rally_points = false;
        _grid_initialized = false;
        _valid_cell_count = 0;
        _polygon_count = 0;
        if (_log_level.get() > 0 && (_last_area_radius_m >= 0.0f || force)) {
            backend.send_text(MAV_SEVERITY_ERROR, "SoarNav: invalid SNAV_RADIUS_M; area disabled.");
        }
        _last_area_radius_m = -1.0f;
        return;
    }
    const float radius = _radius_m.get();
    const float wp_radius = MAX(10.0f, _wp_radius_m.get());
    const bool radius_changed = _last_area_radius_m < 0.0f || fabsf(radius - _last_area_radius_m) > 0.1f;
    const bool wp_radius_changed = _last_grid_wp_radius_m < 0.0f || fabsf(wp_radius - _last_grid_wp_radius_m) > 0.1f;

    if (wp_radius_changed) {
        _last_grid_wp_radius_m = wp_radius;
        _grid_initialized = false;
        _grid_force_reinit = true;
        _target_valid = false;
        _last_sent_valid = false;
    }

    if (!force && !radius_changed && !wp_radius_changed && now - _last_rally_poll_ms < RALLY_POLL_MS) {
        return;
    }
    _last_rally_poll_ms = now;

    const bool want_polygon = radius <= 0.0f;
    const uint32_t rally_sig = backend.rally_last_change_ms();
    const bool rally_changed = rally_sig != _rally_signature_ms;
    if (!force && !radius_changed && !wp_radius_changed && want_polygon == _using_polygon && !rally_changed && _center_valid) {
        return;
    }
    _rally_signature_ms = rally_sig;
    _last_area_radius_m = radius;

    _grid_initialized = false;
    _grid_force_reinit = true;
    _force_grid_after_reset = false;
    _target_valid = false;
    _last_sent_valid = false;

    if (want_polygon) {
        if (_load_polygon(backend)) {
            _using_polygon = true;
            _center_valid = true;
            _announce_polygon_area(backend);
            return;
        }
        _center_valid = false;
        _using_polygon = false;
        _using_rally_points = false;
        _polygon_count = 0;
        _valid_cell_count = 0;
        if (_log_level.get() > 0) {
            backend.send_text(MAV_SEVERITY_WARNING, "SoarNav: No Rally polygon. Add at least 3 Rally points or SoarNav cannot run.");
        }
        return;
    }

    Location home;
    if (backend.home_location(home) && home.initialised()) {
        _center = _dynamic_center_valid ? _dynamic_center : home;
    } else {
        _center = loc;
    }
    _center_valid = true;
    _using_polygon = false;
    _using_rally_points = false;
    _announce_radius_area(backend);
}

bool AP_SoarNav::_finalize_polygon_points()
{
    if (_polygon_count < 3) {
        _center_valid = false;
        return false;
    }

    int64_t lat_sum = 0;
    int64_t lng_sum = 0;
    for (uint8_t i = 0; i < _polygon_count; i++) {
        lat_sum += _polygon_points[i].lat;
        lng_sum += _polygon_points[i].lng;
    }

    _center = _polygon_points[0];
    _center.lat = int32_t(lat_sum / _polygon_count);
    _center.lng = int32_t(lng_sum / _polygon_count);
    _center.copy_alt_from(_polygon_points[0]);
    _center_valid = true;

    for (uint8_t i = 0; i < _polygon_count; i++) {
        _polygon_lat_offsets[i] = _polygon_points[i].lat - _center.lat;
        _polygon_lng_offsets[i] = _polygon_points[i].lng - _center.lng;
        const Vector2f ne = _center.get_distance_NE(_polygon_points[i]);
        _polygon_xy[i] = Vector2f(ne.y, ne.x);
    }

    _polygon_is_convex = _is_convex();
    _prepare_polygon_xy_cache();
    return true;
}

bool AP_SoarNav::_load_polygon(Backend &backend)
{
    _polygon_count = 0;
    _using_rally_points = false;

    if (backend.rally_count() >= 3) {
        struct RallyPoint {
            Location loc;
            float x;
            float y;
            bool valid;
        } pts[MAX_POLYGON_POINTS];

        Location home;
        if (!backend.home_location(home)) {
            return false;
        }

        uint8_t count = MIN(backend.rally_count(), MAX_POLYGON_POINTS);
        uint8_t valid_count = 0;
        for (uint8_t i = 0; i < count; i++) {
            Location rp;
            if (!backend.rally_location(i, rp) || !rp.initialised()) {
                continue;
            }
            bool duplicate = false;
            for (uint8_t j = 0; j < valid_count; j++) {
                if (pts[j].loc.lat == rp.lat && pts[j].loc.lng == rp.lng) {
                    duplicate = true;
                    break;
                }
            }
            if (duplicate) {
                continue;
            }
            const Vector2f ne = home.get_distance_NE(rp);
            pts[valid_count].loc = rp;
            pts[valid_count].x = ne.y;
            pts[valid_count].y = ne.x;
            pts[valid_count].valid = true;
            valid_count++;
        }
        if (valid_count < 3) {
            return false;
        }

        for (uint8_t i = 1; i < valid_count; i++) {
            RallyPoint key = pts[i];
            int8_t j = int8_t(i) - 1;
            while (j >= 0 && (pts[j].x > key.x || (is_equal(pts[j].x, key.x) && pts[j].y > key.y))) {
                pts[j + 1] = pts[j];
                j--;
            }
            pts[j + 1] = key;
        }

        uint8_t lower[MAX_POLYGON_POINTS];
        uint8_t upper[MAX_POLYGON_POINTS];
        uint8_t lower_count = 0;
        uint8_t upper_count = 0;
        auto cross = [&](uint8_t oi, uint8_t ai, uint8_t bi) -> float {
            return (pts[ai].x - pts[oi].x) * (pts[bi].y - pts[oi].y) - (pts[ai].y - pts[oi].y) * (pts[bi].x - pts[oi].x);
        };
        for (uint8_t i = 0; i < valid_count; i++) {
            while (lower_count >= 2 && cross(lower[lower_count - 2], lower[lower_count - 1], i) <= 0.0f) {
                lower_count--;
            }
            lower[lower_count++] = i;
        }
        for (int8_t i = int8_t(valid_count) - 1; i >= 0; i--) {
            while (upper_count >= 2 && cross(upper[upper_count - 2], upper[upper_count - 1], uint8_t(i)) <= 0.0f) {
                upper_count--;
            }
            upper[upper_count++] = uint8_t(i);
        }

        _polygon_count = 0;
        for (uint8_t i = 0; i + 1 < lower_count && _polygon_count < MAX_POLYGON_POINTS; i++) {
            _polygon_points[_polygon_count++] = pts[lower[i]].loc;
        }
        for (uint8_t i = 0; i + 1 < upper_count && _polygon_count < MAX_POLYGON_POINTS; i++) {
            _polygon_points[_polygon_count++] = pts[upper[i]].loc;
        }

        if (_polygon_count < 3) {
            float cx = 0.0f;
            float cy = 0.0f;
            for (uint8_t i = 0; i < valid_count; i++) {
                cx += pts[i].x;
                cy += pts[i].y;
            }
            cx /= valid_count;
            cy /= valid_count;
            for (uint8_t i = 0; i < valid_count; i++) {
                for (uint8_t j = i + 1; j < valid_count; j++) {
                    const float ai = atan2f(pts[i].y - cy, pts[i].x - cx);
                    const float aj = atan2f(pts[j].y - cy, pts[j].x - cx);
                    if (aj < ai) {
                        RallyPoint tmp = pts[i];
                        pts[i] = pts[j];
                        pts[j] = tmp;
                    }
                }
            }
            _polygon_count = MIN(valid_count, MAX_POLYGON_POINTS);
            for (uint8_t i = 0; i < _polygon_count; i++) {
                _polygon_points[i] = pts[i].loc;
            }
        }
        _using_rally_points = true;
    } else {
        return false;
    }

    return _finalize_polygon_points();
}

bool AP_SoarNav::_point_in_area(const Location &loc) const
{
    if (!_center_valid) {
        return false;
    }
    if (_using_polygon) {
        return _point_in_polygon(loc);
    }
    const float radius = _radius_m.get();
    if (radius <= 0.0f) {
        return false;
    }
    return _center.get_distance(loc) <= radius;
}

bool AP_SoarNav::_point_in_polygon(const Location &loc) const
{
    if (_polygon_count < 3 || !_center_valid) {
        return false;
    }
    const Vector2f ne = _center.get_distance_NE(loc);
    const float x = ne.y;
    const float y = ne.x;
    bool inside = false;
    uint8_t j = _polygon_count - 1;
    for (uint8_t i = 0; i < _polygon_count; j = i++) {
        const Vector2f pi = _polygon_xy[i];
        const Vector2f pj = _polygon_xy[j];
        if (((pi.y > y) != (pj.y > y)) && (x < (pj.x - pi.x) * (y - pi.y) / (pj.y - pi.y + 1.0e-6f) + pi.x)) {
            inside = !inside;
        }
    }
    return inside;
}

bool AP_SoarNav::_segment_stays_inside(const Location &from, const Location &to) const
{
    if (!_using_polygon) {
        return true;
    }
    const float dist = from.get_distance(to);
    const float skip_dist = MAX(120.0f, _wp_radius_m.get() * 4.0f);
    if (dist < skip_dist) {
        return true;
    }
    if (!_point_in_area(from) || !_point_in_area(to)) {
        return false;
    }
    if (_polygon_is_convex) {
        return true;
    }
    float cx, cy, tx, ty;
    if (!_loc_to_xy(from, cx, cy) || !_loc_to_xy(to, tx, ty)) {
        return true;
    }
    for (uint8_t i = 0; i < _polygon_count; i++) {
        const uint8_t j = (i + 1) % _polygon_count;
        const float ax = _polygon_xy[i].x;
        const float ay = _polygon_xy[i].y;
        const float bx = _polygon_xy[j].x;
        const float by = _polygon_xy[j].y;
        if (MAX(cx, tx) < MIN(ax, bx) || MIN(cx, tx) > MAX(ax, bx) || MAX(cy, ty) < MIN(ay, by) || MIN(cy, ty) > MAX(ay, by)) {
            continue;
        }
        const float den = (bx - ax) * (ty - cy) - (by - ay) * (tx - cx);
        if (fabsf(den) <= 1.0e-9f) {
            continue;
        }
        const float u = ((cx - ax) * (ty - cy) - (cy - ay) * (tx - cx)) / den;
        const float v = ((cx - ax) * (by - ay) - (cy - ay) * (bx - ax)) / den;
        if (u >= 0.0f && u <= 1.0f && v >= 0.0f && v <= 1.0f) {
            return false;
        }
    }
    return true;
}

bool AP_SoarNav::_clamp_inside_area(const Location &from, Location &candidate) const
{
    if (_point_in_area(candidate) && _segment_stays_inside(from, candidate)) {
        return true;
    }
    if (!_center_valid) {
        return false;
    }
    if (!_using_polygon) {
        const float radius = _radius_m.get();
        if (radius <= 0.0f) {
            return false;
        }
        const float clamped_radius = MAX(radius - _wp_radius_m.get(), MIN(radius, 1.0f));
        const float brg = _bearing_deg(_center, candidate);
        candidate = _center;
        candidate.offset_bearing(brg, clamped_radius);
        return _point_in_area(candidate);
    }
    candidate = _clamp_inside_polygon(candidate, _wp_radius_m.get() * 0.5f);
    if (!_segment_stays_inside(from, candidate)) {
        candidate = _adjust_target_segment(from, candidate);
    }
    return _point_in_area(candidate);
}

void AP_SoarNav::_init_grid(Backend &backend, const Location &loc, bool force)
{
    if (_grid_initialized && !force) {
        return;
    }
    if (!_center_valid) {
        return;
    }
    for (auto &cell : _grid) {
        cell = {};
    }
    memset(_valid_cells, 0, sizeof(_valid_cells));
    _valid_cell_count = 0;

    if (_using_polygon && _polygon_count >= 3) {
        float min_x = 1.0e9f, max_x = -1.0e9f, min_y = 1.0e9f, max_y = -1.0e9f;
        for (uint8_t i = 0; i < _polygon_count; i++) {
            min_x = MIN(min_x, _polygon_xy[i].x);
            max_x = MAX(max_x, _polygon_xy[i].x);
            min_y = MIN(min_y, _polygon_xy[i].y);
            max_y = MAX(max_y, _polygon_xy[i].y);
        }
        _grid_min_x_m = min_x;
        _grid_min_y_m = min_y;
        _grid_width_m = MAX(max_x - min_x, 1.0f);
        _grid_height_m = MAX(max_y - min_y, 1.0f);
    } else {
        const float span_m = MAX(_radius_m.get() * 2.0f, 500.0f);
        _grid_min_x_m = -span_m * 0.5f;
        _grid_min_y_m = -span_m * 0.5f;
        _grid_width_m = span_m;
        _grid_height_m = span_m;
    }

    const float area_m2 = MAX(_grid_width_m * _grid_height_m, 1.0f);
    _grid_cell_size_m = MAX(MIN_CELL_M, sqrtf(area_m2 / float(MAX_GRID_CELLS)));
    _grid_rows = uint8_t(MAX(1, floorf(_grid_height_m / _grid_cell_size_m)));
    _grid_cols = uint8_t(MAX(1, floorf(_grid_width_m / _grid_cell_size_m)));
    while (uint16_t(_grid_rows) * uint16_t(_grid_cols) > MAX_GRID_CELLS) {
        if (_grid_cols >= _grid_rows && _grid_cols > 1) {
            _grid_cols--;
        } else if (_grid_rows > 1) {
            _grid_rows--;
        } else {
            break;
        }
    }

    for (uint8_t r = 0; r < _grid_rows; r++) {
        for (uint8_t c = 0; c < _grid_cols; c++) {
            const uint8_t idx = r * _grid_cols + c;
            _grid[idx].visits = 0;
            _grid[idx].valid = false;
            Location p;
            if (!_grid_index_to_location(idx, p)) {
                continue;
            }
            if (_point_in_area(p)) {
                _grid[idx].valid = true;
                _valid_cells[_valid_cell_count++] = idx;
            }
        }
    }
    _grid_initialized = _valid_cell_count > 0;
    if (_log_level.get() > 0 && _grid_initialized) {
        backend.send_text(MAV_SEVERITY_INFO, "SoarNav: Grid %ux%u, cell %.0fm, valid %u/%u",
                          (unsigned)_grid_rows, (unsigned)_grid_cols, (double)_grid_cell_size_m,
                          (unsigned)_valid_cell_count, (unsigned)(_grid_rows * _grid_cols));
    }
}

bool AP_SoarNav::_loc_to_grid_index(const Location &loc, uint8_t &idx) const
{
    if (!_grid_initialized || !_center_valid) {
        return false;
    }
    const Vector2f ne = _center.get_distance_NE(loc);
    const int16_t c = floorf((ne.y - _grid_min_x_m) / _grid_cell_size_m);
    const int16_t r = floorf((ne.x - _grid_min_y_m) / _grid_cell_size_m);
    if (r < 0 || c < 0 || r >= _grid_rows || c >= _grid_cols) {
        return false;
    }
    const uint8_t out = r * _grid_cols + c;
    if (out >= MAX_GRID_CELLS || !_grid[out].valid) {
        return false;
    }
    idx = out;
    return true;
}

bool AP_SoarNav::_grid_index_to_location(uint8_t idx, Location &loc) const
{
    if (!_center_valid || _grid_rows == 0 || _grid_cols == 0 || idx >= _grid_rows * _grid_cols) {
        return false;
    }
    const uint8_t r = idx / _grid_cols;
    const uint8_t c = idx % _grid_cols;
    const float east = _grid_min_x_m + (c + 0.5f) * _grid_cell_size_m;
    const float north = _grid_min_y_m + (r + 0.5f) * _grid_cell_size_m;
    loc = _center;
    loc.offset(north, east);
    return true;
}

void AP_SoarNav::_update_visited_cell(const Location &loc)
{
    if (_last_cell_update_valid && _last_cell_update_loc.get_distance(loc) < _wp_radius_m.get() * 0.5f) {
        return;
    }
    uint8_t idx;
    if (_loc_to_grid_index(loc, idx)) {
        if (_grid[idx].visits < 255) {
            _grid[idx].visits++;
        }
        _last_cell_index = idx;
        _last_cell_update_loc = loc;
        _last_cell_update_valid = true;
    }
}

bool AP_SoarNav::_select_next_target(Backend &backend, const Location &loc, Location &target, const char *&source)
{
    if (_reengage_flyout_active || _reengage_hold_active) {
        if (_update_reengage(backend, loc, target, source)) {
            return true;
        }
    }

    bool force_grid_search = false;
    float soar_alt_max = 0.0f;
    if (backend.param_get_float("SOAR_ALT_MAX", soar_alt_max) && soar_alt_max > 0.0f && _alt_above_home_m(loc) > soar_alt_max) {
        force_grid_search = true;
    }
    if (_lost_thermal_counter >= 2) {
        _force_grid_after_reset = true;
        _lost_thermal_counter = 0;
    }
    bool try_smart = !_force_grid_after_reset;
    _force_grid_after_reset = false;

    if (try_smart && !force_grid_search) {
        if (_focus_mode && _select_focus_target(backend, loc, target, source)) {
            return true;
        }
        if (_energy_state != EnergyState::NORMAL && _select_thermal_memory_target(backend, loc, target, source)) {
            return true;
        }
        if (_select_thermal_memory_target(backend, loc, target, source)) {
            return true;
        }
    }

    if (try_smart && _select_ridge_target(backend, loc, target, source)) {
        return true;
    }
    if (try_smart && _select_thermal_street_target(backend, loc, target, source)) {
        return true;
    }
    if (_select_grid_target(backend, loc, target, source)) {
        return true;
    }
    if (_dynamic_soar_alt.get() == 3) {
        float hagl = 0.0f;
        float amsl = 0.0f;
        const uint32_t now = AP_HAL::millis();
        const bool emergency_pressure = _terrain.state != TerrainState::IDLE ||
                                        _te_wall_memory_active(now) ||
                                        _te_hard_unsafe_active(now) ||
                                        (_robust_hagl_and_amsl(backend, loc, hagl, amsl) &&
                                         hagl < _terrain_buffer_m(backend));
        if (emergency_pressure && _te_bootstrap_target(backend, loc, target)) {
            source = "Terrain Evasion";
            return true;
        }
    }
    if (_dynamic_soar_alt.get() == 3 && _grid_initialized && _valid_cell_count > 0) {
        return false;
    }
    return _select_random_fallback(backend, loc, target, source);
}

bool AP_SoarNav::_try_grid_cell_target(Backend &backend, const Location &loc, uint8_t chosen, const char *source, Location &target)
{
    if (chosen >= MAX_GRID_CELLS || !_grid[chosen].valid) {
        return false;
    }

    Location center;
    if (!_grid_index_to_location(chosen, center)) {
        return false;
    }

    const float min_grid_dist = MAX(_grid_cell_size_m * 0.35f, _wp_radius_m.get() * 2.0f);
    const bool require_segment_inside = !_using_polygon || _point_in_area(loc);
    for (uint8_t tries = 0; tries < 7; tries++) {
        target = center;
        target.offset_bearing(_randf(0.0f, 360.0f), sqrtf(MAX(_randf(0.0f, 1.0f), 0.0f)) * MAX(_grid_cell_size_m / 2.5f, 1.0f));
        const bool segment_ok = !require_segment_inside || _segment_stays_inside(loc, target);
        if (_point_in_area(target) &&
            loc.get_distance(target) > min_grid_dist &&
            segment_ok &&
            _terrain_candidate_allowed(backend, loc, target, source, chosen)) {
            _grid[chosen].terrain_fail_count = 0;
            return true;
        }
    }

    target = center;
    const bool segment_ok = !require_segment_inside || _segment_stays_inside(loc, target);
    const bool ok = _point_in_area(target) &&
                    loc.get_distance(target) > min_grid_dist &&
                    segment_ok &&
                    _terrain_candidate_allowed(backend, loc, target, source, chosen);
    if (ok) {
        _grid[chosen].terrain_fail_count = 0;
    }
    return ok;
}

bool AP_SoarNav::_select_grid_target(Backend &backend, const Location &loc, Location &target, const char *&source)
{
    if (!_grid_initialized || _valid_cell_count == 0) {
        return false;
    }

    uint8_t unvisited[MAX_GRID_CELLS];
    uint8_t unvisited_count = 0;
    uint8_t least[MAX_GRID_CELLS];
    uint8_t least_count = 0;
    uint8_t min_visits = 255;
    uint8_t cur_idx = 255;
    const bool have_cur_idx = _loc_to_grid_index(loc, cur_idx);
    const uint32_t now = AP_HAL::millis();

    for (uint8_t i = 0; i < _valid_cell_count; i++) {
        const uint8_t idx = _valid_cells[i];
        if (have_cur_idx && idx == cur_idx && _valid_cell_count > 1) {
            continue;
        }
        if (_dynamic_soar_alt.get() == 3 && _grid_cell_blocked_by_terrain(idx, now)) {
            continue;
        }
        const uint8_t visits = _grid[idx].visits;
        if (visits == 0 && unvisited_count < ARRAY_SIZE(unvisited)) {
            unvisited[unvisited_count++] = idx;
        }
        if (visits < min_visits) {
            min_visits = visits;
            least_count = 0;
        }
        if (visits == min_visits && least_count < ARRAY_SIZE(least)) {
            least[least_count++] = idx;
        }
    }

    if (unvisited_count == 0 && _valid_cell_count > 0) {
        for (uint8_t i = 0; i < _valid_cell_count; i++) {
            _grid[_valid_cells[i]].visits = 0;
        }
        least_count = 0;
        min_visits = 0;
        for (uint8_t i = 0; i < _valid_cell_count; i++) {
            const uint8_t idx = _valid_cells[i];
            if (have_cur_idx && idx == cur_idx && _valid_cell_count > 1) {
                continue;
            }
            if (_dynamic_soar_alt.get() == 3 && _grid_cell_blocked_by_terrain(idx, now)) {
                continue;
            }
            if (unvisited_count < ARRAY_SIZE(unvisited)) {
                unvisited[unvisited_count++] = idx;
            }
            if (least_count < ARRAY_SIZE(least)) {
                least[least_count++] = idx;
            }
        }
        _force_grid_after_reset = true;
    }

    uint8_t recent_success_count = 0;
    const uint32_t hist_ms = MAX(uint32_t(60), uint32_t(_strategy_history_s.get())) * 1000U;
    for (uint8_t i = 0; i < MAX_HOTSPOTS; i++) {
        if (_hotspots[i].valid && _hotspots[i].entry_time_ms != 0 && now - _hotspots[i].entry_time_ms <= hist_ms) {
            recent_success_count++;
        }
    }
    const float success_rate_factor = MIN(1.0f, recent_success_count / 3.0f);
    const uint8_t guided_chance = uint8_t(25.0f + 50.0f * success_rate_factor);
    const bool guided = (_rand() % 100U) < guided_chance;
    source = guided ? "Guided" : "Pure";

    const uint8_t *pool = unvisited_count > 0 ? unvisited : least;
    const uint8_t pool_count = unvisited_count > 0 ? unvisited_count : least_count;
    if (pool_count == 0) {
        return false;
    }

    bool tried[MAX_GRID_CELLS] = {};
    uint8_t tried_count = 0;
    Vector3f wind;
    const bool use_wind_score = guided && _get_wind_vector(backend, wind) && wind.length() >= 1.0f;
    const float upwind_bearing = use_wind_score ? _wind_from_bearing_deg(wind) : 0.0f;

    while (tried_count < pool_count) {
        int16_t choice_pos = -1;
        if (use_wind_score) {
            float best_score = -1.0e9f;
            for (uint8_t i = 0; i < pool_count; i++) {
                if (tried[i]) {
                    continue;
                }
                Location p;
                if (!_grid_index_to_location(pool[i], p)) {
                    continue;
                }
                const float bearing = _bearing_deg(loc, p);
                const float angle_diff = fabsf(_wrap_180(bearing - upwind_bearing));
                const float score = cosf(radians(angle_diff));
                if (score > best_score) {
                    best_score = score;
                    choice_pos = i;
                }
            }
        } else {
            const uint8_t start = _rand() % pool_count;
            for (uint8_t offset = 0; offset < pool_count; offset++) {
                const uint8_t i = (start + offset) % pool_count;
                if (!tried[i]) {
                    choice_pos = i;
                    break;
                }
            }
        }

        if (choice_pos < 0) {
            break;
        }
        tried[choice_pos] = true;
        tried_count++;

        if (_try_grid_cell_target(backend, loc, pool[choice_pos], source, target)) {
            return true;
        }
    }

    return false;
}

bool AP_SoarNav::_select_random_fallback(Backend &backend, const Location &loc, Location &target, const char *&source)
{
    if (!_center_valid) {
        return false;
    }
    for (uint8_t i = 0; i < RANDOM_FALLBACK_TRIES; i++) {
        const float brg = _randf(0, 360);
        const float dist = _using_polygon ? _randf(100, 700) : _randf(_wp_radius_m.get() * 2.0f, MAX(_radius_m.get() * 0.85f, 100.0f));
        target = _using_polygon ? loc : _center;
        target.offset_bearing(brg, dist);
        source = "Random Fallback";
        if (_clamp_inside_area(loc, target) &&
            _terrain_candidate_allowed(backend, loc, target, source)) {
            return true;
        }
    }
    return false;
}

bool AP_SoarNav::_select_thermal_memory_target(Backend &backend, const Location &loc, Location &target, const char *&source)
{
    if (_thermal_memory_enable.get() <= 0) {
        return false;
    }
    _clean_hotspots(AP_HAL::millis());
    _update_hotspot_density();
    if (_hotspot_count == 0) {
        return false;
    }

    const float min_dist = MAX(_wp_radius_m.get() * 2.0f, 1.0f);
    const float quality_weight = _energy_state == EnergyState::CRITICAL ? 0.7f : (_energy_state == EnergyState::LOW ? 0.5f : 0.3f);
    const float strength_weight = 1.0f - quality_weight;
    int8_t best = -1;
    float best_score = -1.0e9f;
    Location best_loc;
    for (uint8_t pass = 0; pass < 2; pass++) {
        best = -1;
        best_score = -1.0e9f;
        for (uint8_t i = 0; i < MAX_HOTSPOTS; i++) {
            if (!_hotspots[i].valid || _hotspots[i].avg_strength_mps <= 0.0f) {
                continue;
            }
            if (pass == 0 && _last_used_hotspot_timestamp_ms != 0 && _hotspots[i].timestamp_ms == _last_used_hotspot_timestamp_ms) {
                continue;
            }
            Location predicted;
            if (!_predict_hotspot_drift(backend, _hotspots[i], predicted)) {
                predicted = _hotspots[i].loc;
            }
            if (loc.get_distance(predicted) <= min_dist ||
                !_point_in_area(predicted) ||
                !_segment_stays_inside(loc, predicted) ||
                !_terrain_candidate_allowed(backend, loc, predicted, "Thermal Memory")) {
                continue;
            }
            const float quality = _hotspots[i].avg_strength_mps * MAX(_hotspots[i].consistency_score, 0.5f);
            const float sorted_score = _hotspots[i].avg_strength_mps * strength_weight + quality * quality_weight;
            const float score = sorted_score * 100.0f + _hotspots[i].density * 20.0f;
            if (score > best_score) {
                best_score = score;
                best = i;
                best_loc = predicted;
            }
        }
        if (best >= 0) {
            break;
        }
    }
    if (best < 0) {
        return false;
    }

    target = best_loc;
    const float drift_dist = _hotspots[best].loc.get_distance(best_loc);
    source = drift_dist > 0.5f ? "Thrm(Drift)" : "Thrm(NoDrift)";
    _last_used_hotspot_timestamp_ms = _hotspots[best].timestamp_ms;
    return true;
}

bool AP_SoarNav::_select_focus_target(Backend &backend, const Location &loc, Location &target, const char *&source)
{
    if (_thermal_memory_enable.get() <= 0) {
        return false;
    }
    _clean_hotspots(AP_HAL::millis());
    _update_hotspot_density();

    int8_t best = -1;
    float best_density = _focus_threshold.get();
    for (uint8_t i = 0; i < MAX_HOTSPOTS; i++) {
        if (_hotspots[i].valid && _hotspots[i].density >= best_density) {
            best_density = _hotspots[i].density;
            best = i;
        }
    }
    if (!_focus_mode) {
        if (best < 0) {
            return false;
        }
        _focus_mode = true;
        _focus_start_ms = AP_HAL::millis();
        _focus_wp_counter = 0;
        _focus_wp_timeout = 3;
        _focus_center = _hotspots[best].loc;
        _focus_center_valid = true;
        if (_log_level.get() > 0) {
            backend.send_text(MAV_SEVERITY_NOTICE, "SoarNav: Focus Mode ON (Density)");
        }
    }

    if (_focus_wp_counter >= _focus_wp_timeout) {
        _focus_mode = false;
        _focus_center_valid = false;
        return false;
    }
    if (!_focus_center_valid && best >= 0) {
        _focus_center = _hotspots[best].loc;
        _focus_center_valid = true;
    }
    if (!_focus_center_valid) {
        _focus_mode = false;
        return false;
    }

    const uint32_t age_s = (AP_HAL::millis() - _focus_start_ms) / 1000U;
    Location center = _focus_center;
    Hotspot fake{};
    fake.loc = _focus_center;
    fake.timestamp_ms = AP_HAL::millis() - age_s * 1000U;
    fake.valid = true;
    backend.wind_vector(fake.wind_ned);
    _predict_hotspot_drift(backend, fake, center);

    const float focus_radius = MAX(50.0f, _effective_cluster_radius_m * 0.5f);
    for (uint8_t tries = 0; tries < 3; tries++) {
        if (!_generate_target_around_point(center, focus_radius, target)) {
            break;
        }
        source = "Focus";
        if (_clamp_inside_area(loc, target) &&
            _segment_stays_inside(loc, target) &&
            _terrain_candidate_allowed(backend, loc, target, source)) {
            _focus_wp_counter++;
            return true;
        }
    }
    _focus_mode = false;
    _focus_center_valid = false;
    return false;
}

bool AP_SoarNav::_select_thermal_street_target(Backend &backend, const Location &loc, Location &target, const char *&source)
{
    if (_hotspot_count < 2) {
        return false;
    }
    Vector3f wind;
    if (!_get_wind_vector(backend, wind) || wind.length() < 1.0f) {
        return false;
    }

    int8_t newest = -1;
    int8_t second = -1;
    for (uint8_t i = 0; i < MAX_HOTSPOTS; i++) {
        if (!_hotspots[i].valid) {
            continue;
        }
        if (newest < 0 || _hotspots[i].timestamp_ms > _hotspots[newest].timestamp_ms) {
            second = newest;
            newest = i;
        } else if (second < 0 || _hotspots[i].timestamp_ms > _hotspots[second].timestamp_ms) {
            second = i;
        }
    }
    if (newest < 0 || second < 0) {
        return false;
    }

    const Location &h1 = _hotspots[newest].loc;
    const Location &h2 = _hotspots[second].loc;
    if (h1.get_distance(h2) < 150.0f) {
        return false;
    }

    const float wind_bearing = _wind_from_bearing_deg(wind);
    const float street_bearing = _bearing_deg(h2, h1);
    const float angle_diff = fabsf(_wrap_180(street_bearing - wind_bearing));
    if (angle_diff > _street_tolerance_deg.get()) {
        return false;
    }

    float projection_dist = h2.get_distance(h1);
    projection_dist = constrain_float(projection_dist, 500.0f, 2000.0f);
    target = h1;
    target.offset_bearing(_wrap_360(wind_bearing + 180.0f), projection_dist);
    if (!_point_in_area(target) || loc.get_distance(target) <= _wp_radius_m.get() * 2.0f) {
        return false;
    }
    source = "Thermal Street";
    if (!_clamp_inside_area(loc, target) ||
        !_terrain_candidate_allowed(backend, loc, target, source)) {
        return false;
    }
    return true;
}

bool AP_SoarNav::_select_ridge_target(Backend &backend, const Location &loc, Location &target, const char *&source)
{
    if (!_grid_initialized || _grid_cell_size_m <= 0.0f) {
        return false;
    }

    Vector3f wind;
    if (!_get_wind_vector(backend, wind) || wind.length() < 2.0f) {
        return false;
    }

    const float dx = constrain_float(_grid_cell_size_m, 30.0f, 120.0f);
    float best_ux = 0.0f;
    float best_uy = 0.0f;
    float ux = 0.0f;
    float uy = 0.0f;
    float best_score = _ridge_score_at_loc(backend, loc, &ux, &uy);
    Location best = loc;
    best_ux = ux;
    best_uy = uy;

    const float bearings[] = {0.0f, 90.0f, 180.0f, 270.0f};
    for (float bearing : bearings) {
        Location cand = loc;
        cand.offset_bearing(bearing, dx);
        if (!_point_in_area(cand)) {
            continue;
        }
        ux = 0.0f;
        uy = 0.0f;
        const float score = _ridge_score_at_loc(backend, cand, &ux, &uy);
        if (score > best_score) {
            best_score = score;
            best = cand;
            best_ux = ux;
            best_uy = uy;
        }
    }

    if (best_score < 0.15f) {
        return false;
    }

    target = best;
    target.offset_bearing(_wind_from_bearing_deg(wind), _wp_radius_m.get());

    const float tangent_e = -best_uy;
    const float tangent_n = best_ux;
    float tangent_bearing = degrees(atan2f(tangent_e, tangent_n));
    if (tangent_bearing < 0.0f) {
        tangent_bearing += 360.0f;
    }
    const float along = constrain_float(_wp_radius_m.get() * 1.5f, 60.0f, 300.0f);
    target.offset_bearing(tangent_bearing, along);

    source = "Ridge";
    if (!_clamp_inside_area(loc, target) ||
        !_terrain_candidate_allowed(backend, loc, target, source)) {
        return false;
    }

    return true;
}

bool AP_SoarNav::_send_target(Backend &backend, const Location &loc_in, const char *source, bool force)
{
    Location here;
    if (!backend.current_location(here)) {
        return false;
    }

    Location loc = loc_in;
    _make_guided_location(backend, loc);
    if (!_clamp_inside_area(here, loc)) {
        return false;
    }

    const uint32_t now = AP_HAL::millis();
    const char *new_source = source != nullptr ? source : "Unknown";
    const bool source_changed = strncmp(_target_source, new_source, sizeof(_target_source)) != 0;
    const bool target_was_valid = _target_valid;
    Backend::ModeNumber mode = backend.mode_number();

    if (mode != Backend::ModeNumber::GUIDED && mode != Backend::ModeNumber::THERMAL) {
        if (!_can_start_from_mode(mode)) {
            return false;
        }
        const Backend::ModeNumber entry_mode = mode;
        if (!backend.set_guided_mode()) {
            return false;
        }
        if (_original_entry_mode == Backend::ModeNumber::UNKNOWN) {
            _original_entry_mode = entry_mode;
        }
        _guided_entry_mode = entry_mode;
        _guided_mode_requested_ms = now;
        mode = backend.mode_number();
        _last_sent_valid = false;
    }

    const bool terrain_source = _source_is_terrain_evasion(new_source);
    const bool target_changed = _target_key_changed(loc, terrain_source);
    const bool new_waypoint = !target_was_valid || target_changed || source_changed;
    const bool horizontal_target_changed = !_last_sent_valid ||
                                           labs(loc.lat - _last_sent_target.lat) > 3 ||
                                           labs(loc.lng - _last_sent_target.lng) > 3;
    const bool thermal_refresh_due = mode == Backend::ModeNumber::THERMAL && (now - _last_target_sent_ms >= GUIDED_TARGET_REFRESH_MS);
    if (!force && _last_sent_valid && !target_changed && !source_changed && !thermal_refresh_due) {
        _target = loc;
        _target_valid = true;
        return true;
    }
    if (force && mode == Backend::ModeNumber::GUIDED && _last_sent_valid && !horizontal_target_changed && !source_changed) {
        _target = loc;
        _target_valid = true;
        return true;
    }

    if (!backend.set_guided_target(loc, terrain_source)) {
        if (_guided_mode_requested_ms != 0 && now - _guided_mode_requested_ms > GUIDED_MODE_REQUEST_TIMEOUT_MS) {
            _clear_navigation_state(true);
        }
        return false;
    }

    _guided_mode_requested_ms = 0;
    _target = loc;
    _target_valid = true;
    if (terrain_source || _terrain.post_resume_until_ms == 0 || now >= _terrain.post_resume_until_ms) {
        _next_target_search_ms = 0;
    }
    if (source_changed) {
        strncpy(_target_source, new_source, sizeof(_target_source) - 1);
        _target_source[sizeof(_target_source) - 1] = 0;
    }
    _last_sent_target = loc;
    _last_sent_valid = true;
    _last_target_sent_ms = now;
    _distance_to_wp_m = here.get_distance(loc);
    if (new_waypoint) {
        _initial_distance_to_wp_m = _distance_to_wp_m;
        _reroute_check_armed = strncmp(_target_source, "Terrain Evasion", 16) != 0;
    }
    _log_target(backend, loc, _target_source);
    _reset_progress_monitor(GUIDED_PROGRESS_ARM_DELAY_MS);
    _waypoint_start_ms = now;
    return true;
}

void AP_SoarNav::_make_guided_location(Backend &backend, Location &loc) const
{
    Location cur;
    if (backend.current_location(cur)) {
        int32_t rel_cm = 0;
        if (cur.get_alt_cm(Location::AltFrame::ABOVE_HOME, rel_cm)) {
            loc.set_alt_cm(rel_cm, Location::AltFrame::ABOVE_HOME);
            return;
        }
        loc.copy_alt_from(cur);
        return;
    }
    loc.change_alt_frame(Location::AltFrame::ABOVE_HOME);
}

bool AP_SoarNav::_target_key_changed(const Location &loc, bool horizontal_only) const
{
    if (!_last_sent_valid) {
        return true;
    }
    if (labs(loc.lat - _last_sent_target.lat) > 3 || labs(loc.lng - _last_sent_target.lng) > 3) {
        return true;
    }
    if (horizontal_only) {
        return false;
    }
    return labs(loc.alt - _last_sent_target.alt) > 50;
}

bool AP_SoarNav::_source_is_terrain_evasion(const char *source) const
{
    return source != nullptr &&
           (strncmp(source, "Terrain Evasion", 16) == 0 ||
            strcmp(source, "TE Egress") == 0 ||
            strcmp(source, "TE Bootstrap") == 0);
}

bool AP_SoarNav::_target_log_suppresses_cells(const char *source) const
{
    if (source == nullptr) {
        return false;
    }
    return strstr(source, "Thrm") != nullptr ||
           strstr(source, "Focus") != nullptr ||
           strcmp(source, "Thermal Street") == 0 ||
           strcmp(source, "Random Fallback") == 0 ||
           strncmp(source, "Terrain Evasion", 16) == 0 ||
           strcmp(source, "TE Egress") == 0 ||
           strcmp(source, "TE Bootstrap") == 0 ||
           strcmp(source, "Anti-Stuck") == 0 ||
           strcmp(source, "Ridge") == 0 ||
           strcmp(source, "Re-Engage ReEntry") == 0 ||
           strcmp(source, "Re-Engage FlyOut") == 0;
}

bool AP_SoarNav::_target_log_is_duplicate(const Location &loc, const char *source, uint32_t now_ms) const
{
    if (!_last_logged_target_valid || source == nullptr) {
        return false;
    }
    const bool terrain_source = _source_is_terrain_evasion(source);
    if (terrain_source) {
        if (!_source_is_terrain_evasion(_last_logged_target_source)) {
            return false;
        }
    } else if (strncmp(_last_logged_target_source, source, sizeof(_last_logged_target_source)) != 0) {
        return false;
    }
    const bool same_horizontal_target = labs(loc.lat - _last_logged_target.lat) <= 10 &&
                                        labs(loc.lng - _last_logged_target.lng) <= 10;
    if (!same_horizontal_target) {
        return false;
    }
    const uint32_t repeat_ms = terrain_source ? TERRAIN_TARGET_LOG_MIN_MS : TARGET_LOG_REPEAT_MS;
    return now_ms - _last_target_log_ms < repeat_ms;
}

void AP_SoarNav::_log_target(Backend &backend, const Location &loc, const char *source)
{
    if (_log_level.get() <= 0) {
        return;
    }

    const uint32_t now = AP_HAL::millis();
    if (_target_log_is_duplicate(loc, source, now)) {
        return;
    }

    Location here;
    if (!backend.current_location(here)) {
        backend.send_text(MAV_SEVERITY_INFO, "SoarNav: [%s]", source != nullptr ? source : "Unknown");
        _last_logged_target = loc;
        _last_logged_target_valid = true;
        _last_target_log_ms = now;
        strncpy(_last_logged_target_source, source != nullptr ? source : "Unknown", sizeof(_last_logged_target_source) - 1);
        _last_logged_target_source[sizeof(_last_logged_target_source) - 1] = 0;
        return;
    }

    const char *safe_source = source != nullptr ? source : "Unknown";
    const float dist = here.get_distance(loc);
    const float brg = _bearing_deg(here, loc);
    if (_last_logged_target_valid && strncmp(safe_source, "Terrain Evasion", 16) == 0 &&
        strncmp(_last_logged_target_source, "Terrain Evasion", 16) == 0 &&
        now - _last_target_log_ms < TERRAIN_TARGET_LOG_MIN_MS &&
        fabsf(_wrap_180(brg - _last_logged_target_bearing_deg)) < 5.0f &&
        fabsf(dist - _last_logged_target_distance_m) < 50.0f) {
        return;
    }
    const bool show_cells = _grid_initialized && !_target_log_suppresses_cells(safe_source);

    if (show_cells) {
        uint16_t visited = 0;
        for (uint8_t i = 0; i < _valid_cell_count; i++) {
            if (_grid[_valid_cells[i]].visits > 0) {
                visited++;
            }
        }
        _log_gcs(backend, MAV_SEVERITY_INFO, 1, MsgID::NAV_TARGET_CELLS,
                 safe_source, (double)brg, _compass(brg), (double)dist,
                 int(visited), int(_valid_cell_count));
    } else {
        _log_gcs(backend, MAV_SEVERITY_INFO, 1, MsgID::NAV_TARGET_SIMPLE,
                 safe_source, (double)brg, _compass(brg), (double)dist);
    }

    _last_logged_target = loc;
    _last_logged_target_valid = true;
    _last_logged_target_bearing_deg = brg;
    _last_logged_target_distance_m = dist;
    _last_target_log_ms = now;
    strncpy(_last_logged_target_source, safe_source, sizeof(_last_logged_target_source) - 1);
    _last_logged_target_source[sizeof(_last_logged_target_source) - 1] = 0;
}

void AP_SoarNav::on_enter_thermal(Backend &backend)
{
    if (_thermal.active || _manual_override_active || !_owns_operating_mode() ||
        (_state != State::NAVIGATING && _state != State::THERMAL_PAUSE)) {
        return;
    }
    Location loc;
    if (!backend.current_location(loc)) {
        return;
    }
    _thermal = {};
    _thermal.active = true;
    _thermal.start_loc = loc;
    _thermal.best_loc = loc;
    _thermal.best_loc_valid = true;
    _thermal.start_ms = AP_HAL::millis();
    _thermal.last_sample_ms = _thermal.start_ms;
    _thermal.max_strength = -1000.0f;
    _thermal.min_strength = 1000.0f;
    _set_state(State::THERMAL_PAUSE, backend, "thermal enter");
}

void AP_SoarNav::on_exit_thermal(Backend &backend)
{
    if (_manual_override_active || (!_thermal.active && _state != State::THERMAL_PAUSE)) {
        _was_in_thermal_mode = false;
        return;
    }
    Location loc;
    if (!backend.current_location(loc)) {
        loc = _thermal.best_loc_valid ? _thermal.best_loc : _thermal.start_loc;
    }
    const bool weak = _thermal.active && (_thermal.accum_weight <= 0 || (_thermal.accum_strength / MAX(_thermal.accum_weight, 1.0f)) < _thermal_memory_min_strength.get());
    const bool terrain_escape_active = _terrain.state != TerrainState::IDLE;
    if (terrain_escape_active) {
        _thermal.active = false;
        _thermal.sample_count = 0;
    } else {
        _finish_thermal(backend, loc, weak);
    }

    if (backend.mode_number() != Backend::ModeNumber::GUIDED) {
        _post_thermal_resend_until_ms = 0;
        _last_sent_valid = false;
        _target_valid = false;
        _reengage_final_valid = false;
        _reengage_hold_active = false;
        _reengage_flyout_active = false;
        if (_state == State::NAVIGATING || _state == State::THERMAL_PAUSE) {
            _set_state(State::IDLE, backend, "thermal exit non-guided");
        }
        return;
    }

    if (_target_valid) {
        const uint32_t now = AP_HAL::millis();
        _last_sent_valid = false;
        _waypoint_start_ms = now;
        if (loc.initialised()) {
            _distance_to_wp_m = loc.get_distance(_target);
            _last_progress_distance_m = _distance_to_wp_m;
        }
        _post_thermal_resend_until_ms = now + POST_THERMAL_TARGET_RESEND_MS;
        _reset_progress_monitor(MAX(GUIDED_PROGRESS_ARM_DELAY_MS, uint32_t(_stuck_time_s.get()) * 1000U));
    }
}

void AP_SoarNav::_sample_thermal(Backend &backend, const Location &loc)
{
    if (!_thermal.active) {
        on_enter_thermal(backend);
        if (!_thermal.active) {
            return;
        }
    }
    const uint32_t now = AP_HAL::millis();
    if (_thermal.last_sample_ms != 0 && now - _thermal.last_sample_ms < 500U) {
        return;
    }
    _thermal.last_sample_ms = now;
    Vector3f vned;
    const float climb = backend.velocity_ned(vned) ? -vned.z : backend.climb_rate_mps();
    const float sample_weight = MIN(1.0f, fabsf(climb) / 3.0f);
    if (_thermal.accum_weight > 0.0f) {
        const float avg = _thermal.accum_strength / _thermal.accum_weight;
        const float new_avg = avg * (1.0f - sample_weight) + climb * sample_weight;
        _thermal.accum_strength = new_avg;
        _thermal.accum_weight = 1.0f;
    } else {
        _thermal.accum_strength = climb;
        _thermal.accum_weight = 1.0f;
    }
    _thermal.max_strength = MAX(_thermal.max_strength, climb);
    _thermal.min_strength = MIN(_thermal.min_strength, climb);
    if (_thermal.sample_count < ARRAY_SIZE(_thermal.samples)) {
        _thermal.samples[_thermal.sample_count++] = climb;
    } else {
        memmove(&_thermal.samples[0], &_thermal.samples[1], sizeof(float) * (ARRAY_SIZE(_thermal.samples) - 1));
        _thermal.samples[ARRAY_SIZE(_thermal.samples) - 1] = climb;
    }
    if (climb >= _thermal.max_strength - 0.01f) {
        _thermal.best_loc = loc;
        _thermal.best_loc_valid = true;
    }
}

void AP_SoarNav::_finish_thermal(Backend &backend, const Location &loc, bool weak_exit)
{
    if (!_thermal.active || !_thermal.best_loc_valid || _thermal.accum_weight <= 0.0f) {
        if (_log_level.get() > 0) {
            backend.send_text(MAV_SEVERITY_WARNING, "SoarNav: Thermal exit: no data. (Lost Thermal)");
        }
        _lost_thermal_counter++;
        _thermal.active = false;
        _thermal.sample_count = 0;
        return;
    }

    const float avg = _thermal.accum_strength / MAX(_thermal.accum_weight, 1.0f);
    const float max_s = _thermal.max_strength;
    const float uncertainty = max_s < 3.0f ? constrain_float((3.0f - max_s) * 25.0f, 0.0f, 50.0f) : 0.0f;
    const float necessity = (1.0f - _filtered_alt_factor) * _necessity_weight.get();
    const float retry_score = uncertainty + necessity;
    if (max_s < _thermal_memory_min_strength.get()) {
        if (weak_exit && retry_score >= _retry_threshold.get() && _point_in_area(_thermal.best_loc)) {
            _begin_reengage(backend, loc, _thermal.best_loc);
        }
        _thermal.active = false;
        _thermal.sample_count = 0;
        return;
    }
    _lost_thermal_counter = 0;

    if (_focus_mode) {
        if (_log_level.get() > 0) {
            backend.send_text(MAV_SEVERITY_NOTICE, "SoarNav: Thermal found, exiting focus.");
        }
        _focus_mode = false;
        _focus_wp_counter = 0;
    }

    Location save_loc = _thermal.best_loc;

    const float variance = _calculate_thermal_variance(_thermal.samples, _thermal.sample_count);
    const float consistency = variance < 0.5f ? 1.0f : 0.5f;

    if (!_point_in_area(save_loc)) {
        if (_log_level.get() > 0) {
            backend.send_text(MAV_SEVERITY_INFO, "SoarNav: Thermal ignored: out of area.");
        }
        _thermal.active = false;
        _thermal.sample_count = 0;
        return;
    }

    bool saved = false;
    if (_thermal_memory_enable.get() > 0) {
        uint8_t new_cell = 255;
        _loc_to_grid_index(save_loc, new_cell);
        bool duplicate_cell = false;
        for (uint8_t i = 0; i < MAX_HOTSPOTS; i++) {
            if (!_hotspots[i].valid) {
                continue;
            }
            uint8_t existing_cell = 255;
            if (new_cell != 255 && _loc_to_grid_index(_hotspots[i].loc, existing_cell) && existing_cell == new_cell) {
                duplicate_cell = true;
                break;
            }
        }
        if (!duplicate_cell) {
            _add_hotspot(backend, save_loc, avg, max_s);
            for (uint8_t i = 0; i < MAX_HOTSPOTS; i++) {
                if (_hotspots[i].valid && (_hotspots[i].loc.lat == save_loc.lat && _hotspots[i].loc.lng == save_loc.lng)) {
                    _hotspots[i].consistency_score = consistency;
                    _hotspots[i].entry_time_ms = _thermal.start_ms;
                    break;
                }
            }
            _last_used_hotspot_timestamp_ms = AP_HAL::millis();
            _update_hotspot_density();
            saved = true;
            if (_log_level.get() > 0) {
                backend.send_text(MAV_SEVERITY_INFO, "SoarNav: Thermal saved avg=%.2f max=%.2f", (double)avg, (double)max_s);
            }
        }
    }

    if (saved) {
        for (uint8_t i = 0; i < MAX_HOTSPOTS; i++) {
            if (_hotspots[i].valid && _hotspots[i].density >= _focus_threshold.get() && !_focus_mode) {
                _focus_mode = true;
                _focus_wp_counter = 0;
                _focus_start_ms = AP_HAL::millis();
                _focus_center = _hotspots[i].loc;
                _focus_center_valid = true;
                float total_strength = 0.0f;
                uint8_t count = 0;
                for (uint8_t j = 0; j < MAX_HOTSPOTS; j++) {
                    if (_hotspots[j].valid && _hotspots[j].loc.get_distance(_hotspots[i].loc) < _effective_cluster_radius_m) {
                        total_strength += MAX(_hotspots[j].avg_strength_mps, 0.0f);
                        count++;
                    }
                }
                const float avg_cluster = count > 0 ? total_strength / count : 0.0f;
                _focus_wp_timeout = constrain_int16(3 + int16_t(avg_cluster * 2.0f), 3, 10);
                if (_log_level.get() > 0) {
                    backend.send_text(MAV_SEVERITY_NOTICE, "SoarNav: Focus Mode ON (Density)");
                }
                break;
            }
        }
    }

    if (saved && retry_score >= _retry_threshold.get()) {
        _begin_reengage(backend, loc, save_loc);
    } else if (weak_exit && !saved && retry_score >= _retry_threshold.get()) {
        _begin_reengage(backend, loc, save_loc);
    }

    _thermal.active = false;
    _thermal.sample_count = 0;
}

void AP_SoarNav::_add_hotspot(Backend &backend, const Location &loc, float avg_strength, float max_strength)
{
    uint8_t slot = MAX_HOTSPOTS;
    uint32_t oldest = UINT32_MAX;
    for (uint8_t i = 0; i < MAX_HOTSPOTS; i++) {
        if (!_hotspots[i].valid) {
            slot = i;
            break;
        }
        if (_hotspots[i].timestamp_ms < oldest) {
            oldest = _hotspots[i].timestamp_ms;
            slot = i;
        }
    }
    if (slot >= MAX_HOTSPOTS) {
        return;
    }
    _hotspots[slot].loc = loc;
    _hotspots[slot].timestamp_ms = AP_HAL::millis();
    _hotspots[slot].avg_strength_mps = avg_strength;
    _hotspots[slot].max_strength_mps = max_strength;
    _hotspots[slot].entry_alt_m = _alt_above_home_m(loc);
    _hotspots[slot].density = 0;
    _hotspots[slot].valid = true;
    backend.wind_vector(_hotspots[slot].wind_ned);
    _hotspot_count = 0;
    for (uint8_t i = 0; i < MAX_HOTSPOTS; i++) {
        if (_hotspots[i].valid) {
            _hotspot_count++;
        }
    }
}

void AP_SoarNav::_clean_hotspots(uint32_t now_ms)
{
    _hotspot_count = 0;
    const uint32_t life_ms = MAX(uint32_t(60), uint32_t(_thermal_memory_life_s.get())) * 1000UL;
    for (uint8_t i = 0; i < MAX_HOTSPOTS; i++) {
        if (_hotspots[i].valid && now_ms - _hotspots[i].timestamp_ms > life_ms) {
            _hotspots[i].valid = false;
        }
        if (_hotspots[i].valid) {
            _hotspot_count++;
        }
    }
}

void AP_SoarNav::_update_hotspot_density()
{
    const uint32_t now = AP_HAL::millis();
    const float life_ms = MAX(60.0f, float(_thermal_memory_life_s.get())) * 1000.0f;
    for (uint8_t i = 0; i < MAX_HOTSPOTS; i++) {
        if (!_hotspots[i].valid) {
            continue;
        }
        float density = 0;
        for (uint8_t j = 0; j < MAX_HOTSPOTS; j++) {
            if (i == j || !_hotspots[j].valid) {
                continue;
            }
            const float d = _hotspots[i].loc.get_distance(_hotspots[j].loc);
            if (d < _effective_cluster_radius_m) {
                const float age_factor = 1.0f - MIN(1.0f, (now - _hotspots[j].timestamp_ms) / life_ms);
                density += (1.0f - d / MAX(_effective_cluster_radius_m, 1.0f)) * MAX(_hotspots[j].avg_strength_mps, 0.1f) * age_factor;
            }
        }
        _hotspots[i].density = density;
    }
}

bool AP_SoarNav::_predict_hotspot_drift(Backend &backend, const Hotspot &hotspot, Location &loc) const
{
    loc = hotspot.loc;
    Vector3f wind = hotspot.wind_ned;
    if (wind.length() < 0.5f && (!backend.wind_vector(wind) || wind.length() < 0.5f)) {
        return true;
    }

    const float age_s = constrain_float((AP_HAL::millis() - hotspot.timestamp_ms) * 0.001f, 0.0f, float(_thermal_memory_life_s.get()));
    float soar_min = 0.0f;
    float soar_max = 600.0f;
    backend.param_get_float("SOAR_ALT_MIN", soar_min);
    backend.param_get_float("SOAR_ALT_MAX", soar_max);
    const float band = soar_max - soar_min;
    const float est_bl_top = constrain_float(band > 0.0f ? band : 600.0f, 300.0f, 1200.0f);
    const float agl = MAX(_get_agl_m(backend, loc), 1.0f);
    const float alpha = 0.12f;
    float v_eff_factor = 1.0f / (1.0f + alpha);
    if (agl > est_bl_top) {
        const float r = est_bl_top / MAX(agl, 0.001f);
        v_eff_factor = (1.0f - r) + powf(r, alpha + 1.0f) / (1.0f + alpha);
    }

    float distance_coeff = 1.0f;
    float score_coeff = 1.0f;
    _lifecycle_coeff(age_s, _thermal_memory_life_s.get(), distance_coeff, score_coeff);
    (void)score_coeff;

    const float drift_speed = wind.length() * v_eff_factor * distance_coeff;
    const float drift_dist = drift_speed * age_s;
    const float veer_scale = powf(MIN(1.0f, agl / est_bl_top), 0.7f);
    const float drift_bearing = _wrap_360(_wind_to_bearing_deg(wind) + 8.0f * veer_scale);
    loc.offset_bearing(drift_bearing, drift_dist);
    return true;
}

void AP_SoarNav::_begin_reengage(Backend &backend, const Location &loc, const Location &thermal_loc)
{
    Location predicted = thermal_loc;
    if (!_point_in_area(predicted)) {
        (void)_clamp_inside_area(loc, predicted);
    }

    const float bearing_to_thermal = _bearing_deg(loc, predicted);
    float airspeed = 0.0f;
    Vector3f vned;
    float gs = 0.0f;
    if (backend.velocity_ned(vned)) {
        gs = vned.xy().length();
    }
    float cruise = 15.0f;
    backend.param_get_float("AIRSPEED_CRUISE", cruise);
    const float speed_for_calc = backend.airspeed_estimate_mps(airspeed) && airspeed > 3.0f ? airspeed : (gs > 3.0f ? gs : cruise);
    const float reengage_dist = constrain_float(speed_for_calc * 20.0f, 400.0f, 1000.0f);
    const float bearings[] = {
        _wrap_360(bearing_to_thermal + 180.0f),
        _wrap_360(bearing_to_thermal + 215.0f),
        _wrap_360(bearing_to_thermal + 145.0f),
        _wrap_360(bearing_to_thermal + 250.0f),
        _wrap_360(bearing_to_thermal + 110.0f),
    };

    Location best;
    bool best_valid = false;
    float best_score = -1.0e9f;
    for (uint8_t i = 0; i < ARRAY_SIZE(bearings); i++) {
        Location p = loc;
        p.offset_bearing(bearings[i], reengage_dist);
        if (!_clamp_inside_area(loc, p)) {
            continue;
        }
        if (!_terrain_candidate_allowed(backend, loc, p, "Re-Engage FlyOut")) {
            continue;
        }
        const float ang = fabsf(_wrap_180(bearings[i] - _wrap_360(bearing_to_thermal + 180.0f)));
        const float score = cosf(radians(ang)) * 2.0f + loc.get_distance(p) * 0.001f;
        if (score > best_score) {
            best_score = score;
            best = p;
            best_valid = true;
        }
    }

    if (!best_valid) {
        for (uint8_t attempt = 1; attempt <= 5; attempt++) {
            const float test_dist = reengage_dist * (0.5f / attempt);
            if (test_dist < 200.0f) {
                break;
            }
            Location p = loc;
            p.offset_bearing(_wrap_360(bearing_to_thermal + 180.0f), test_dist);
            if (_clamp_inside_area(loc, p) &&
                _terrain_candidate_allowed(backend, loc, p, "Re-Engage FlyOut")) {
                best = p;
                best_valid = true;
                break;
            }
        }
    }
    if (!best_valid) {
        _reengage_final_valid = false;
        _reengage_hold_active = false;
        _reengage_flyout_active = false;
        _reengage_flyout_origin_valid = false;
        _reengage_flyout_initial_dist_m = 0.0f;
        return;
    }

    Location final_target = predicted;
    const float dist_back = loc.get_distance(final_target);
    const float reentry_cap = reengage_dist * 2.0f;
    if (dist_back > reentry_cap) {
        final_target = loc;
        final_target.offset_bearing(_bearing_deg(loc, predicted), reentry_cap);
        if (!_clamp_inside_area(loc, final_target)) {
            return;
        }
    }
    if (_terrain_candidate_allowed(backend, loc, final_target, "Re-Engage ReEntry")) {
        _reengage_final_target = final_target;
        _reengage_final_valid = true;
    } else {
        _reengage_final_valid = false;
    }
    if (!_send_target(backend, best, "Re-Engage FlyOut", true)) {
        _reengage_final_valid = false;
        _reengage_hold_active = false;
        _reengage_flyout_active = false;
        _reengage_flyout_origin_valid = false;
        _reengage_flyout_initial_dist_m = 0.0f;
        return;
    }
    _reengage_hold_active = false;
    _reengage_flyout_active = true;
    _reengage_flyout_origin = loc;
    _reengage_flyout_origin_valid = true;
    _reengage_flyout_initial_dist_m = loc.get_distance(best);
    _reengage_flyout_start_ms = AP_HAL::millis();
}

bool AP_SoarNav::_update_reengage(Backend &backend, const Location &loc, Location &target, const char *&source)
{
    if (!_reengage_flyout_active && !_reengage_hold_active) {
        return false;
    }
    const uint32_t now = AP_HAL::millis();
    if (_reengage_flyout_active) {
        if (!_target_valid) {
            _reengage_flyout_active = false;
            _reengage_flyout_origin_valid = false;
            _reengage_flyout_initial_dist_m = 0.0f;
            return false;
        }
        const float wp_radius = MAX(float(_wp_radius_m.get()), 10.0f);
        const float dist_to_target = loc.get_distance(_target);
        const float initial_dist = _reengage_flyout_initial_dist_m > 0.0f ? _reengage_flyout_initial_dist_m : dist_to_target;
        const uint32_t elapsed_ms = now - _reengage_flyout_start_ms;
        const bool reached = dist_to_target <= MAX(wp_radius * 1.5f, 80.0f);
        float travelled = 0.0f;
        bool moved_enough = false;
        if (_reengage_flyout_origin_valid) {
            travelled = loc.get_distance(_reengage_flyout_origin);
            moved_enough = travelled >= MAX(wp_radius * 2.0f, MIN(300.0f, initial_dist * 0.45f));
        }
        const uint32_t dwell_ms = MAX(uint32_t(15000), uint32_t(_reengage_dwell_s.get()) * 2000U);
        const float speed = MAX(_nav_speed_mps(backend), 8.0f);
        const uint32_t max_ms = MAX(uint32_t(25000), MIN(uint32_t(45000), uint32_t((initial_dist / speed) * 1250.0f)));
        if (reached || (moved_enough && elapsed_ms >= dwell_ms) || elapsed_ms >= max_ms) {
            _reengage_flyout_active = false;
            _reengage_flyout_origin_valid = false;
            _reengage_flyout_initial_dist_m = 0.0f;
            if (_reengage_final_valid) {
                target = _reengage_final_target;
                source = "Re-Engage ReEntry";
                _reengage_final_valid = false;
                if (_clamp_inside_area(loc, target) &&
                    _terrain_candidate_allowed(backend, loc, target, source)) {
                    _reengage_hold_active = true;
                    _reengage_hold_until_ms = now + MAX(uint32_t(1000), uint32_t(_reengage_dwell_s.get()) * 1000U);
                    return true;
                }
            }
            _reengage_hold_active = false;
        }
        return false;
    }
    if (_reengage_hold_active && now >= _reengage_hold_until_ms) {
        _reengage_hold_active = false;
        _reengage_flyout_origin_valid = false;
        _reengage_flyout_initial_dist_m = 0.0f;
    }
    return false;
}

bool AP_SoarNav::_manage_waypoint_status(Backend &backend, const Location &loc)
{
    if (!_target_valid) {
        return true;
    }
    _distance_to_wp_m = loc.get_distance(_target);
    if (_distance_to_wp_m <= _wp_radius_m.get()) {
        return true;
    }
    const uint32_t now = AP_HAL::millis();
    if (_terrain.post_resume_until_ms != 0 && now < _terrain.post_resume_until_ms &&
        !_source_is_terrain_evasion(_target_source) &&
        _distance_to_wp_m > MAX(float(_wp_radius_m.get()) * 2.0f, 1.0f)) {
        return false;
    }
    if (_waypoint_start_ms != 0 && now - _waypoint_start_ms > _wp_timeout_ms(backend)) {
        if (_log_level.get() > 0) {
            backend.send_text(MAV_SEVERITY_WARNING, "SoarNav: WP Timeout -> Replan");
        }
        return true;
    }
    return false;
}

bool AP_SoarNav::_manage_anti_stuck(Backend &backend, const Location &loc, Location &target, const char *&source)
{
    const uint32_t now = AP_HAL::millis();
    if (_is_repositioning ||
        _reengage_flyout_active || _reengage_hold_active ||
        backend.mode_number() == Backend::ModeNumber::THERMAL ||
        _terrain.state != TerrainState::IDLE ||
        _source_is_terrain_evasion(_target_source) ||
        (_post_thermal_resend_until_ms != 0 && now < _post_thermal_resend_until_ms)) {
        _stuck_counter = 0;
        _reset_progress_monitor(GUIDED_PROGRESS_ARM_DELAY_MS);
        return false;
    }
    const uint32_t stuck_grace_ms = uint32_t(_stuck_time_s.get()) * 1000U;
    if (_waypoint_start_ms == 0 || now - _waypoint_start_ms < stuck_grace_ms) {
        return false;
    }
    if (!_target_valid || now < _progress_hold_until_ms) {
        return false;
    }
    if (_last_progress_check_ms != 0 && now - _last_progress_check_ms < STUCK_PROGRESS_CHECK_MS) {
        return false;
    }
    _last_progress_check_ms = now;
    const float dist_now = loc.get_distance(_target);
    if (!_last_progress_loc_valid) {
        _last_progress_loc = loc;
        _last_progress_loc_valid = true;
        _last_progress_distance_m = dist_now;
        return false;
    }
    const float moved = _last_progress_loc.get_distance(loc);
    const float closed = _last_progress_distance_m - dist_now;
    const float eff = moved > 1.0f ? closed / moved : 0.0f;
    _last_progress_loc = loc;
    _last_progress_distance_m = dist_now;
    if (closed < 10.0f || eff < _stuck_efficiency.get()) {
        _stuck_counter++;
        if (_log_level.get() > 1) {
            backend.send_text(MAV_SEVERITY_WARNING, "SoarNav: NoProg E%.2f moved %.0f closed %.0f", (double)eff, (double)moved, (double)closed);
        }
    } else {
        _stuck_counter = 0;
    }
    if (_stuck_counter < 2) {
        return false;
    }
    _stuck_counter = 0;
    const float away = _bearing_deg(_target, loc);
    target = loc;
    target.offset_bearing(_wrap_360(away + _randf(-70, 70)), constrain_float(dist_now * 0.5f + 200.0f, 200.0f, 900.0f));
    source = "Anti-Stuck";
    if (!_clamp_inside_area(loc, target) ||
        !_terrain_candidate_allowed(backend, loc, target, source)) {
        return false;
    }
    _reposition_resume_target = _target;
    _reposition_resume_valid = true;
    strncpy(_reposition_resume_source, _target_source, sizeof(_reposition_resume_source) - 1);
    _reposition_resume_source[sizeof(_reposition_resume_source) - 1] = 0;
    _is_repositioning = true;
    _reroute_check_armed = false;
    return true;
}

bool AP_SoarNav::_maybe_tactical_reroute(Backend &backend, const Location &loc, Location &target, const char *&source)
{
    const uint32_t now = AP_HAL::millis();
    if (!_reroute_check_armed || !_check_tactical_reroute_conditions()) {
        return false;
    }
    if (_terrain.hold_until_ms != 0 && now < _terrain.hold_until_ms) {
        return false;
    }
    if (_post_thermal_resend_until_ms != 0 && now < _post_thermal_resend_until_ms) {
        return false;
    }
    if (_initial_distance_to_wp_m <= 0.0f) {
        return false;
    }

    const float dist = loc.get_distance(_target);
    if (dist > _initial_distance_to_wp_m * 0.5f) {
        return false;
    }

    _reroute_check_armed = false;

    if (strncmp(_target_source, "Terrain Evasion", 16) == 0 ||
        strcmp(_target_source, "TE Egress") == 0 ||
        strcmp(_target_source, "Re-Engage ReEntry") == 0 ||
        strcmp(_target_source, "Re-Engage FlyOut") == 0 ||
        strcmp(_target_source, "Re-route") == 0) {
        return false;
    }

    if ((_rand() % 100U) >= uint32_t(_reroute_probability_pct.get())) {
        return false;
    }

    const float yaw_deg = _wrap_360(degrees(backend.yaw_rad()));
    const float sign = (_rand() & 1U) ? 1.0f : -1.0f;
    const float offset = _randf(_reroute_min_deg.get(), _reroute_max_deg.get()) * sign;
    target = loc;
    target.offset_bearing(_wrap_360(yaw_deg + offset), dist);
    source = "Re-route";
    if (!_clamp_inside_area(loc, target) ||
        !_terrain_candidate_allowed(backend, loc, target, source)) {
        return false;
    }
    return true;
}

uint32_t AP_SoarNav::_wp_timeout_ms(Backend &backend) const
{
    const uint32_t base = MAX(uint32_t(30), uint32_t(_wp_timeout_s.get())) * 1000UL;
    Vector3f wind;
    if (!backend.wind_vector(wind)) {
        return base;
    }
    const float w = wind.xy().length();
    const float mult = constrain_float(1.0f + w / 15.0f, 0.5f, 2.5f);
    return uint32_t(base * mult);
}

void AP_SoarNav::_reset_progress_monitor(uint32_t delay_ms)
{
    _last_progress_check_ms = 0;
    _last_progress_loc_valid = false;
    _last_progress_distance_m = -1;
    _progress_hold_until_ms = AP_HAL::millis() + delay_ms;
}

float AP_SoarNav::_energy_factor(Backend &backend, const Location &loc)
{
    float alt_min = 100.0f;
    float alt_max = 400.0f;
    backend.param_get_float("SOAR_ALT_MIN", alt_min);
    backend.param_get_float("SOAR_ALT_MAX", alt_max);
    const float current_alt = _alt_above_home_m(loc);
    const float alt_range = alt_max - alt_min;
    if (alt_range <= 0.0f) {
        return 0.0f;
    }
    return constrain_float((current_alt - alt_min) / alt_range, 0.0f, 1.0f);
}

void AP_SoarNav::_update_energy(Backend &backend, const Location &loc)
{
    const uint32_t now = AP_HAL::millis();
    const float current_alt = _alt_above_home_m(loc);
    float trend = 0.0f;
    if (_last_alt_timestamp_ms != 0) {
        const float dt = (now - _last_alt_timestamp_ms) * 0.001f;
        if (dt > 0.5f) {
            trend = (current_alt - _last_alt_m) / dt;
        }
    }
    _last_alt_m = current_alt;
    _last_alt_timestamp_ms = now;

    const float raw_factor = _energy_factor(backend, loc);
    if (_energy_state_transition_ms == 0) {
        _filtered_alt_factor = raw_factor;
        _energy_state_transition_ms = now;
    } else {
        const float dt_filter = (now - _energy_state_transition_ms) * 0.001f;
        const float alpha = 1.0f - expf(-dt_filter * 0.05f);
        _filtered_alt_factor += (raw_factor - _filtered_alt_factor) * constrain_float(alpha, 0.0f, 1.0f);
        _energy_state_transition_ms = now;
    }

    EnergyState new_state = EnergyState::NORMAL;
    if (_filtered_alt_factor < 0.25f) {
        new_state = EnergyState::LOW;
    }
    if (_filtered_alt_factor < 0.10f) {
        new_state = EnergyState::CRITICAL;
    }

    if (trend < -0.3f) {
        if (_negative_trend_start_ms == 0) {
            _negative_trend_start_ms = now;
        }
        if (now - _negative_trend_start_ms >= 3000U && new_state != EnergyState::CRITICAL) {
            new_state = EnergyState::CRITICAL;
        }
    } else {
        _negative_trend_start_ms = 0;
    }

    _energy_state = new_state;
}

void AP_SoarNav::_store_initial_soar_alts(Backend &backend)
{
    if (_initial_soar_alts_valid) {
        return;
    }
    if (backend.param_get_float("SOAR_ALT_MIN", _initial_soar_alt_min_m) &&
        backend.param_get_float("SOAR_ALT_MAX", _initial_soar_alt_max_m) &&
        backend.param_get_float("SOAR_ALT_CUTOFF", _initial_soar_alt_cutoff_m)) {
        _initial_soar_alts_valid = true;
    }
}

void AP_SoarNav::_restore_initial_soar_alts(Backend &backend)
{
    if (!_initial_soar_alts_valid) {
        return;
    }
    backend.param_set_float("SOAR_ALT_MIN", _initial_soar_alt_min_m);
    backend.param_set_float("SOAR_ALT_MAX", _initial_soar_alt_max_m);
    backend.param_set_float("SOAR_ALT_CUTOFF", _initial_soar_alt_cutoff_m);
}

void AP_SoarNav::_update_dynamic_soar_alt(Backend &backend, const Location &loc)
{
    const int8_t mode = _dynamic_soar_alt.get();
    if (mode <= 0 || mode == 3 || !_initial_soar_alts_valid) {
        return;
    }
    if (backend.throttle_percent() >= 10.0f || backend.motor_running()) {
        return;
    }

    if (_state == State::PILOT_OVERRIDE || _manual_override_active) {
        _gc_reset_hold_start_ms = 0;
        if (!_override_reset_done) {
            _restore_initial_soar_alts(backend);
            if (_log_level.get() > 0) {
                backend.send_text(MAV_SEVERITY_NOTICE, "SoarNav: Glide Cone reset override MIN %.0f CUT %.0f MAX %.0f",
                                  (double)_initial_soar_alt_min_m,
                                  (double)_initial_soar_alt_cutoff_m,
                                  (double)_initial_soar_alt_max_m);
            }
            _override_reset_done = true;
        }
        return;
    }
    _override_reset_done = false;

    float polar_b = 0.0f;
    float polar_cd0 = 0.0f;
    float best_glide_airspeed = 0.0f;
    if (!backend.param_get_float("SOAR_POLAR_B", polar_b) ||
        !backend.param_get_float("SOAR_POLAR_CD0", polar_cd0) ||
        !backend.param_get_float("AIRSPEED_CRUISE", best_glide_airspeed) ||
        polar_b <= 0.0f || polar_cd0 <= 0.0f || best_glide_airspeed <= 0.0f ||
        !isfinite(polar_b) || !isfinite(polar_cd0) || !isfinite(best_glide_airspeed)) {
        if (!_gcone_param_warning_sent && _log_level.get() > 0) {
            backend.send_text(MAV_SEVERITY_WARNING, "SoarNav: Glide Cone disabled, missing polar/airspeed params");
            _gcone_param_warning_sent = true;
        }
        return;
    }

    const float efficiency_max = 1.0f / sqrtf(4.0f * polar_cd0 * polar_b);
    if (!isfinite(efficiency_max) || efficiency_max <= 0.0f) {
        return;
    }

    Location home;
    if (!backend.home_location(home)) {
        return;
    }

    const float current_alt = _alt_above_home_m(loc);
    Vector2f vec_to_home = loc.get_distance_NE(home);
    float wind_comp = 0.0f;
    Vector3f wind;
    if (backend.wind_vector(wind) && vec_to_home.length() > 1.0f) {
        wind_comp = (vec_to_home.x * wind.x + vec_to_home.y * wind.y) / vec_to_home.length();
        if (!isfinite(wind_comp)) {
            wind_comp = 0.0f;
        }
    }

    float gs_to_home = best_glide_airspeed + wind_comp;
    if (!isfinite(gs_to_home) || gs_to_home <= 0.0f) {
        gs_to_home = 0.1f;
    }

    const float sink_best = best_glide_airspeed / efficiency_max;
    if (!isfinite(sink_best) || sink_best <= 0.0f) {
        return;
    }

    const float eff_ld = gs_to_home / sink_best;
    if (!isfinite(eff_ld) || eff_ld <= 0.0f) {
        return;
    }

    const float required_alt_glide = MAX(0.0f, loc.get_distance(home) / eff_ld);
    float required_terrain_min_alt = 0.0f;
    float cur_ter = 0.0f;
    float home_ter = 0.0f;
    if (backend.terrain_height_amsl(loc, cur_ter) && backend.terrain_height_amsl(home, home_ter)) {
        required_terrain_min_alt = _initial_soar_alt_min_m + cur_ter - home_ter;
    }

    const float calculated_min_alt = MAX(_initial_soar_alt_min_m,
                                         MAX(required_alt_glide + _glide_cone_pad_m.get() + _glide_cone_margin_m.get(),
                                             required_terrain_min_alt));

    const uint32_t now = AP_HAL::millis();
    float target_min = 0.0f;
    float target_cutoff = _initial_soar_alt_cutoff_m;
    float target_max = _initial_soar_alt_max_m;
    static constexpr float GCONE_RESET_SURPLUS_M = 50.0f;
    static constexpr uint32_t RESET_HOLD_MS = 20000U;

    if (current_alt > calculated_min_alt + GCONE_RESET_SURPLUS_M) {
        if (_gc_reset_hold_start_ms == 0) {
            _gc_reset_hold_start_ms = now;
            return;
        }
        if (now - _gc_reset_hold_start_ms < RESET_HOLD_MS) {
            return;
        }
        if (current_alt > _initial_soar_alt_max_m + 10.0f) {
            _gc_reset_hold_start_ms = 0;
            return;
        }
        target_min = _initial_soar_alt_min_m;
        target_cutoff = _initial_soar_alt_cutoff_m;
        target_max = _initial_soar_alt_max_m;
        _gc_reset_hold_start_ms = 0;
    } else {
        _gc_reset_hold_start_ms = 0;
        target_min = _round_to(floorf(calculated_min_alt), 10.0f);
        if (mode == 1) {
            const float link_threshold = floorf(_initial_soar_alt_cutoff_m * (2.0f / 3.0f));
            if (target_min > link_threshold) {
                const float delta = target_min - _initial_soar_alt_min_m;
                target_cutoff = _initial_soar_alt_cutoff_m + delta;
                target_max = _initial_soar_alt_max_m + delta;
            }
        } else if (mode == 2) {
            const float cap_altitude = floorf(_initial_soar_alt_cutoff_m * (2.0f / 3.0f));
            if (target_min > cap_altitude) {
                target_min = cap_altitude;
            }
        }
    }

    Vector3f vel;
    if (backend.velocity_ned(vel)) {
        const float climb_rate = -vel.z;
        float current_min = 0.0f;
        backend.param_get_float("SOAR_ALT_MIN", current_min);
        if (target_min < current_min && climb_rate > 1.5f) {
            return;
        }
    }

    const float gap_cut_min = MAX(floorf(_initial_soar_alt_cutoff_m / 3.0f + 0.5f), 30.0f);
    const float gap_max_min = MAX(_initial_soar_alt_max_m - _initial_soar_alt_cutoff_m, 50.0f);
    if (target_cutoff < target_min + gap_cut_min) {
        target_cutoff = target_min + gap_cut_min;
    }
    if (target_max < target_cutoff + gap_max_min) {
        target_max = target_cutoff + gap_max_min;
    }

    float current_min = 0.0f;
    float current_cutoff = 0.0f;
    float current_max = 0.0f;
    backend.param_get_float("SOAR_ALT_MIN", current_min);
    backend.param_get_float("SOAR_ALT_CUTOFF", current_cutoff);
    backend.param_get_float("SOAR_ALT_MAX", current_max);

    const float delta = target_min - current_min;
    const bool needs_update = (delta >= 20.0f) || (delta <= -80.0f);
    if (!needs_update) {
        return;
    }

    bool changed = false;
    if (fabsf(current_min - target_min) > 0.5f) {
        changed |= backend.param_set_float("SOAR_ALT_MIN", target_min);
    }
    if ((mode == 1 || mode == 2) && fabsf(current_cutoff - target_cutoff) > 0.5f) {
        changed |= backend.param_set_float("SOAR_ALT_CUTOFF", target_cutoff);
    }
    if ((mode == 1 || mode == 2) && fabsf(current_max - target_max) > 0.5f) {
        changed |= backend.param_set_float("SOAR_ALT_MAX", target_max);
    }

    if (changed && _log_level.get() > 0) {
        const float alt_range = _initial_soar_alt_max_m - _initial_soar_alt_min_m;
        const float warning_margin = alt_range > 0.0f ? alt_range * 0.1f : 0.0f;
        if (alt_range <= 0.0f || current_alt < target_min + warning_margin || fabsf(target_min - _last_gcone_logged_alt_m) >= 10.0f || now - _last_gcone_log_ms > 10000U) {
            backend.send_text(MAV_SEVERITY_NOTICE, "SoarNav: Glide Cone ALT_MIN %.0fm", (double)target_min);
            _last_gcone_logged_alt_m = target_min;
            _last_gcone_log_ms = now;
        }
    }
}

bool AP_SoarNav::_te_bootstrap_target(Backend &backend, const Location &loc, Location &target, float *selected_min_agl_m)
{
    if (_dynamic_soar_alt.get() != 3 || !loc.initialised()) {
        return false;
    }

    float terr = 0.0f;
    if (!backend.terrain_height_amsl(loc, terr)) {
        return false;
    }

    const uint32_t now = AP_HAL::millis();
    const float lever = MAX(_te_lookahead_distance_m(backend), _te_lever_min_m(backend));
    const float yaw_deg = _te_track_or_yaw_deg(backend);

    TerrainSideScan side_scan{};
    const bool side_scan_valid = _te_side_wall_scan(backend, loc, lever, side_scan);
    if (side_scan_valid) {
        (void)_te_update_side_wall_memory(backend, now, side_scan);
    }

    Location path_target;
    bool path_target_valid = false;
    if (_target_valid && !_source_is_terrain_evasion(_target_source)) {
        path_target = _target;
        path_target_valid = true;
    } else {
        path_target_valid = _te_make_candidate(backend, loc, yaw_deg, _te_guided_target_distance(backend), path_target);
    }

    TerrainPolicy policy = _te_evaluate_policy(backend, loc, path_target, path_target_valid, side_scan, side_scan_valid, now);
    if (policy.decision == TerrainDecision::CLEAR || policy.decision == TerrainDecision::MONITOR_ONLY) {
        _te_log_policy(backend, now, policy, TerrainCandidateKind::NONE, policy.current_min_agl_m, 0.0f);
        return false;
    }

    Location selected;
    float selected_agl = -1.0e9f;
    float selected_dist = 0.0f;
    float selected_bearing = yaw_deg;
    TerrainCandidateKind kind = TerrainCandidateKind::NONE;
    if (!_te_select_policy_target(backend, loc, policy, selected, selected_agl, selected_dist, selected_bearing, kind)) {
        return false;
    }
    if (!_point_in_area(selected) || (_using_polygon && !_segment_stays_inside(loc, selected))) {
        return false;
    }

    _te_commit_policy_target(backend, loc, selected, selected_agl, selected_dist, selected_bearing, policy.decision, policy.reason, kind);
    target = selected;
    if (selected_min_agl_m != nullptr) {
        *selected_min_agl_m = selected_agl;
    }
    _te_log_policy(backend, now, policy, kind, selected_agl, selected_dist);
    return true;
}

bool AP_SoarNav::_te_keep_current_evasion_target(Backend &backend, const Location &loc, const TerrainPolicy &policy, const Location &selected, float selected_agl, float selected_bearing, Location &target, const char *&source, uint32_t now_ms)
{
    if (!_terrain.evasion_target_valid) {
        return false;
    }
    if (!_point_in_area(_terrain.evasion_target) || (_using_polygon && !_segment_stays_inside(loc, _terrain.evasion_target))) {
        return false;
    }

    float distance_m = 0.0f;
    if (_te_target_reached(backend, loc, _terrain.evasion_target, distance_m)) {
        return false;
    }

    const float buffer = policy.buffer_m;
    const float yaw_deg = policy.yaw_deg;
    const float current_bearing = _bearing_deg(loc, _terrain.evasion_target);
    const float current_turn = fabsf(_wrap_180(current_bearing - yaw_deg));
    const float selected_turn = fabsf(_wrap_180(selected_bearing - yaw_deg));
    const bool hard_now = _te_hard_unsafe_active(now_ms);

    const bool policy_immediate_replan = _te_policy_requires_immediate_replan(policy, current_bearing);

    if (current_turn > TERRAIN_EMERGENCY_REVERSAL_DEG && !hard_now) {
        return false;
    }
    if (_te_wall_memory_active(now_ms) &&
        _terrain.wall_side != 0 &&
        _te_heading_turns_towards_wall(current_bearing, yaw_deg, now_ms)) {
        return false;
    }

    float current_min_agl = 0.0f;
    float current_worst_frac = 0.0f;
    TerrainProbe current_probe{};
    if (!_path_min_agl_probe(backend, loc, _terrain.evasion_target, current_min_agl, current_worst_frac, &current_probe)) {
        const bool emergency_replan = policy.decision == TerrainDecision::CRITICAL_OVERRIDE ||
                                      policy.decision == TerrainDecision::EMERGENCY_BEST_OF_BAD ||
                                      policy.reason == TerrainReplanReason::IMMEDIATE_COLLISION ||
                                      policy.immediate_threat ||
                                      policy.turn_time_critical ||
                                      policy.hard_unsafe_active;
        if (!emergency_replan &&
            (now_ms < _terrain.replan_not_before_ms || now_ms < _terrain.hold_until_ms ||
             policy.reason == TerrainReplanReason::TERRAIN_DATA_INVALID)) {
            target = _terrain.evasion_target;
            source = "Terrain Evasion";
            if (_log_level.get() > 1 &&
                (_terrain.last_commit_hold_log_ms == 0 || now_ms - _terrain.last_commit_hold_log_ms >= TERRAIN_DEBUG_LOG_MIN_MS)) {
                _terrain.last_commit_hold_log_ms = now_ms;
                backend.send_text(MAV_SEVERITY_INFO, "SoarNav: TEh%c DATA",
                                  _te_side_char(_terrain.committed_turn_side != 0 ? _terrain.committed_turn_side : _terrain.last_turn_side));
            }
            return true;
        }
        return false;
    }

    if ((_terrain.committed_turn_side != 0 || _terrain.last_turn_side != 0) &&
        current_probe.center_min_agl_m > current_min_agl &&
        current_probe.center_min_agl_m >= TERRAIN_HARD_UNSAFE_AGL_M) {
        current_min_agl = current_probe.center_min_agl_m;
    }

    const bool current_hard_escape = isfinite(current_probe.start_hagl_m) &&
                                     current_probe.start_hagl_m < TERRAIN_HARD_UNSAFE_AGL_M &&
                                     current_min_agl > current_probe.start_hagl_m + _te_entry_headroom_m(buffer) &&
                                     current_min_agl >= MAX(TERRAIN_HARD_UNSAFE_AGL_M + _te_entry_headroom_m(buffer),
                                                            buffer * TERRAIN_CRITICAL_BUFFER_FRACTION);
    const bool current_hard_hold = current_min_agl < TERRAIN_HARD_UNSAFE_AGL_M &&
                                   hard_now &&
                                   current_min_agl >= _terrain.evasion_min_agl_m - _te_dynamic_margin_m(buffer, 0.50f);
    const bool current_hard_failed = current_min_agl < TERRAIN_HARD_UNSAFE_AGL_M && !current_hard_escape && !current_hard_hold;

    float selected_min_agl = selected_agl;
    float selected_worst_frac = 0.0f;
    TerrainProbe selected_probe{};
    const bool selected_probe_ok = _path_min_agl_probe(backend, loc, selected, selected_min_agl, selected_worst_frac, &selected_probe);
    if (selected_probe_ok && selected_min_agl > selected_agl) {
        selected_agl = selected_min_agl;
    }

    const int8_t committed_side = _terrain.committed_turn_side != 0 ? _terrain.committed_turn_side : _terrain.last_turn_side;
    const int8_t selected_side = _te_turn_side_from_bearing(backend, selected_bearing);
    const int8_t current_side = _te_turn_side_from_bearing(backend, current_bearing);
    const int8_t roll_side = fabsf(degrees(backend.roll_rad())) > 12.0f ? (backend.roll_rad() > 0.0f ? 1 : -1) : 0;
    const bool selected_switches_committed_side = committed_side != 0 && selected_side != 0 && selected_side != committed_side;
    const bool selected_switches_current_side = current_side != 0 && selected_side != 0 && selected_side != current_side;
    const bool selected_reverses_bank = roll_side != 0 && selected_side != 0 && selected_side != roll_side;
    const bool selected_drops_to_center = committed_side != 0 && selected_side == 0 &&
                                          roll_side != 0 && roll_side == committed_side;
    const bool selected_side_flip = selected_switches_committed_side || selected_switches_current_side ||
                                    selected_reverses_bank || selected_drops_to_center;
    const float required_escape_turn = (policy.side_scan_valid && policy.side_scan.escape_side != 0) ? 55.0f : 45.0f;
    const bool current_timid_during_immediate = policy.decision == TerrainDecision::CRITICAL_OVERRIDE &&
                                                policy.reason == TerrainReplanReason::IMMEDIATE_COLLISION &&
                                                !policy.front_clear &&
                                                !policy.front_margin &&
                                                current_turn < required_escape_turn &&
                                                selected_turn >= required_escape_turn;
    if (current_timid_during_immediate) {
        if (_log_level.get() > 1 &&
            (_terrain.last_commit_hold_log_ms == 0 || now_ms - _terrain.last_commit_hold_log_ms >= TERRAIN_DEBUG_LOG_MIN_MS)) {
            _terrain.last_commit_hold_log_ms = now_ms;
            backend.send_text(MAV_SEVERITY_INFO, "SoarNav: TEforce b%.0f s%.0f",
                              (double)current_bearing,
                              (double)selected_bearing);
        }
        return false;
    }

    const float switch_turn_time_s = _te_turn_time_s(backend, fabsf(_wrap_180(selected_bearing - current_bearing)));
    const float scheduler_s = float(TERRAIN_CHECK_INTERVAL_MS) * 0.001f;
    const float switch_time_margin_s = MAX(scheduler_s, switch_turn_time_s);
    const float reversal_margin = selected_side_flip ?
                                  MAX(_te_dynamic_margin_m(buffer, 1.00f), _te_hold_allowed_drop_m(backend, buffer) * 3.0f) :
                                  MAX(_te_entry_headroom_m(buffer), _te_hold_allowed_drop_m(backend, buffer));
    const float current_score = current_min_agl -
                                _te_turn_feasibility_penalty_m(backend, current_probe, current_turn) -
                                _te_probe_risk_penalty_m(current_probe, buffer) * 0.50f;
    const float selected_score = selected_agl -
                                 (selected_probe_ok ? _te_turn_feasibility_penalty_m(backend, selected_probe, selected_turn) : 0.0f) -
                                 (selected_probe_ok ? _te_probe_risk_penalty_m(selected_probe, buffer) * 0.50f : 0.0f) -
                                 (selected_side_flip ? MAX(buffer, 0.0f) * 0.50f : 0.0f);

    const bool selected_buys_time = selected_probe_ok &&
                                    current_probe.first_threat_time_s >= 0.0f &&
                                    selected_probe.first_threat_time_s >= 0.0f &&
                                    selected_probe.first_threat_time_s > current_probe.first_threat_time_s + switch_time_margin_s;
    const bool selected_buys_agl = selected_agl > current_min_agl + reversal_margin;
    const bool current_immediate = _te_probe_immediate_threat(current_probe, buffer);
    const bool selected_clearly_better = selected_score > current_score + reversal_margin;
    const bool target_recently_updated = _terrain.evasion_target_updated_ms != 0 &&
                                         now_ms - _terrain.evasion_target_updated_ms < TERRAIN_CHECK_INTERVAL_MS;
    const bool current_side_dead = current_hard_failed ||
                                   (policy_immediate_replan && current_immediate &&
                                    current_probe.first_threat_time_s >= 0.0f &&
                                    current_probe.first_threat_time_s <= _te_turn_time_s(backend, current_turn) + scheduler_s &&
                                    current_min_agl < _terrain.evasion_min_agl_m - _te_hold_allowed_drop_m(backend, buffer));

    if (selected_side_flip) {
        const bool switch_cooldown_active = _terrain.commitment_started_ms != 0 &&
                                            now_ms - _terrain.commitment_started_ms < _te_side_wall_min_switch_ms();
        const bool selected_physically_reachable = selected_probe_ok &&
                                                   selected_probe.corridor_min_agl_m >= TERRAIN_HARD_UNSAFE_AGL_M &&
                                                   (selected_probe.first_threat_time_s < 0.0f || selected_probe.first_threat_time_s > switch_turn_time_s + scheduler_s);
        const bool selected_still_low = selected_agl < buffer;
        const bool both_paths_low = current_min_agl < buffer && selected_still_low;
        const bool selected_strong_escape = selected_agl >= buffer + _te_dynamic_margin_m(buffer, 0.25f) ||
                                            (selected_buys_agl && selected_buys_time &&
                                             selected_agl > current_min_agl + _te_side_wall_flip_gain_m(buffer) * 1.50f);
        const bool bad_to_bad_flip = both_paths_low && !selected_strong_escape;
        const bool selected_not_hard = selected_probe_ok &&
                                       selected_probe.center_min_agl_m >= TERRAIN_HARD_UNSAFE_AGL_M &&
                                       selected_probe.corridor_min_agl_m >= TERRAIN_HARD_UNSAFE_AGL_M;
        const bool selected_matches_escape_side = policy.side_scan_valid &&
                                                  policy.side_scan.escape_side != 0 &&
                                                  selected_side == policy.side_scan.escape_side;
        const bool committed_towards_wall = policy.side_scan_valid &&
                                            policy.side_scan.wall_side != 0 &&
                                            ((committed_side != 0 && committed_side == policy.side_scan.wall_side) ||
                                             (current_side != 0 && current_side == policy.side_scan.wall_side) ||
                                             (roll_side != 0 && roll_side == policy.side_scan.wall_side));
        const bool local_center_critical = policy.side_scan_valid &&
                                           policy.side_scan.center_valid &&
                                           (policy.side_scan.center_min_agl_m < buffer ||
                                            (policy.side_scan.center_first_threat_time_s >= 0.0f &&
                                             policy.side_scan.center_first_threat_time_s <= switch_turn_time_s + scheduler_s));
        const bool selected_acceptable_for_forced_flip = selected_not_hard &&
                                                         selected_agl >= current_min_agl - MAX(_te_dynamic_margin_m(buffer, 0.50f),
                                                                                              _te_hold_allowed_drop_m(backend, buffer) * 2.0f);
        const bool forced_side_escape = selected_matches_escape_side &&
                                        selected_acceptable_for_forced_flip &&
                                        local_center_critical &&
                                        (committed_towards_wall || current_immediate || policy_immediate_replan || policy.low_altitude_pressure);
        const bool normal_switch = !target_recently_updated &&
                                   !switch_cooldown_active &&
                                   current_side_dead &&
                                   selected_physically_reachable &&
                                   !bad_to_bad_flip &&
                                   (selected_buys_agl || selected_buys_time || selected_clearly_better);
        const bool allow_switch = normal_switch || forced_side_escape;
        if (!allow_switch) {
            target = _terrain.evasion_target;
            source = "Terrain Evasion";
            if (_log_level.get() > 1 &&
                (_terrain.last_commit_hold_log_ms == 0 || now_ms - _terrain.last_commit_hold_log_ms >= TERRAIN_DEBUG_LOG_MIN_MS)) {
                _terrain.last_commit_hold_log_ms = now_ms;
                backend.send_text(MAV_SEVERITY_INFO, "SoarNav: TEflip hold %c>%c a%.0f s%.0f t%.1f",
                                  _te_side_char(committed_side != 0 ? committed_side : current_side),
                                  _te_side_char(selected_side),
                                  (double)current_min_agl,
                                  (double)selected_agl,
                                  (double)current_probe.first_threat_time_s);
            }
            return true;
        }
        if (_log_level.get() > 1 &&
            (_terrain.last_commit_log_ms == 0 || now_ms - _terrain.last_commit_log_ms >= TERRAIN_DEBUG_LOG_MIN_MS)) {
            _terrain.last_commit_log_ms = now_ms;
            backend.send_text(MAV_SEVERITY_WARNING, "SoarNav: TEflip ok %c>%c a%.0f s%.0f t%.1f",
                              _te_side_char(committed_side != 0 ? committed_side : current_side),
                              _te_side_char(selected_side),
                              (double)current_min_agl,
                              (double)selected_agl,
                              (double)current_probe.first_threat_time_s);
        }
        return false;
    }

    const bool hard_replan_required = current_hard_failed || current_side_dead;
    const bool replan_window_open = now_ms >= _terrain.replan_not_before_ms;
    if (selected_clearly_better &&
        !target_recently_updated &&
        (hard_replan_required || (replan_window_open && (current_immediate || selected_buys_time)))) {
        return false;
    }

    target = _terrain.evasion_target;
    source = "Terrain Evasion";
    return true;
}

bool AP_SoarNav::_te_policy_requires_immediate_replan(const TerrainPolicy &policy, float current_bearing_deg) const
{
    const bool immediate = policy.decision == TerrainDecision::CRITICAL_OVERRIDE &&
        (policy.reason == TerrainReplanReason::IMMEDIATE_COLLISION ||
         policy.immediate_threat ||
         policy.turn_time_critical ||
         policy.hard_unsafe_active ||
         policy.current_hard_agl_m < TERRAIN_HARD_UNSAFE_AGL_M);
    if (!immediate) {
        return false;
    }

    const float signed_turn = _wrap_180(current_bearing_deg - policy.yaw_deg);
    const float turn = fabsf(signed_turn);
    const bool escape_side_known = policy.side_scan_valid && policy.side_scan.escape_side != 0;
    const float required_turn = escape_side_known ? 55.0f : 45.0f;
    const bool forward_blocked = !policy.front_clear && !policy.front_margin;

    if (forward_blocked && turn < required_turn) {
        return true;
    }

    if (escape_side_known && signed_turn * float(policy.side_scan.escape_side) <= 0.0f) {
        const bool motion_blocked = policy.side_scan.center_valid &&
            (policy.side_scan.center_min_agl_m < policy.buffer_m ||
             policy.side_scan.center_first_threat_time_s >= 0.0f ||
             policy.side_scan.center_terrain_delta_m > _te_reentry_terrain_delta_m(policy.buffer_m));
        if (motion_blocked || policy.current_hard_agl_m < TERRAIN_HARD_UNSAFE_AGL_M) {
            return true;
        }
    }

    return false;
}

bool AP_SoarNav::_te_commit_hold_current_target(Backend &backend, const Location &loc, const TerrainPolicy *policy, Location &target, const char *&source, uint32_t now_ms)
{
    if (!_terrain.evasion_target_valid || now_ms >= _terrain.hold_until_ms) {
        return false;
    }
    if (!_point_in_area(_terrain.evasion_target) || (_using_polygon && !_segment_stays_inside(loc, _terrain.evasion_target))) {
        return false;
    }
    float distance_m = 0.0f;
    if (_te_target_reached(backend, loc, _terrain.evasion_target, distance_m)) {
        return false;
    }

    const float buffer = _terrain_buffer_m(backend);
    const float yaw_deg = _te_track_or_yaw_deg(backend);
    const float bearing = _bearing_deg(loc, _terrain.evasion_target);
    const float turn = fabsf(_wrap_180(bearing - yaw_deg));
    const bool hard_now = _te_hard_unsafe_active(now_ms);
    if (policy != nullptr && _te_policy_requires_immediate_replan(*policy, bearing)) {
        return false;
    }
    if (turn > TERRAIN_EMERGENCY_REVERSAL_DEG && !hard_now) {
        return false;
    }
    if (_te_wall_memory_active(now_ms) &&
        _terrain.wall_side != 0 &&
        _te_heading_turns_towards_wall(bearing, yaw_deg, now_ms)) {
        return false;
    }

    float min_agl = 0.0f;
    float worst_frac = 0.0f;
    TerrainProbe probe{};
    if (!_path_min_agl_probe(backend, loc, _terrain.evasion_target, min_agl, worst_frac, &probe)) {
        return false;
    }
    if ((_terrain.committed_turn_side != 0 || _terrain.last_turn_side != 0) &&
        probe.center_min_agl_m > min_agl &&
        probe.center_min_agl_m >= TERRAIN_HARD_UNSAFE_AGL_M) {
        min_agl = probe.center_min_agl_m;
    }

    const bool immediate_threat = _te_probe_immediate_threat(probe, buffer);
    const float allowed_drop = _te_hold_allowed_drop_m(backend, buffer);
    const bool significant_drop = min_agl < _terrain.evasion_min_agl_m - allowed_drop;
    const bool replan_due = now_ms >= _terrain.replan_not_before_ms;
    const bool current_hard_escape = isfinite(probe.start_hagl_m) &&
                                     probe.start_hagl_m < TERRAIN_HARD_UNSAFE_AGL_M &&
                                     min_agl > probe.start_hagl_m + _te_entry_headroom_m(buffer) &&
                                     min_agl >= MAX(TERRAIN_HARD_UNSAFE_AGL_M + _te_entry_headroom_m(buffer),
                                                    buffer * TERRAIN_CRITICAL_BUFFER_FRACTION);
    const bool hard_hold = min_agl < TERRAIN_HARD_UNSAFE_AGL_M &&
                           hard_now &&
                           min_agl >= _terrain.evasion_min_agl_m - _te_dynamic_margin_m(buffer, 0.50f);
    const bool hard_failed = min_agl < TERRAIN_HARD_UNSAFE_AGL_M && !current_hard_escape && !hard_hold;
    const bool soft_ok = min_agl >= buffer || _te_probe_margin_low_ok(probe, buffer);
    const bool late_margin = !immediate_threat &&
                             probe.corridor_min_agl_m >= TERRAIN_HARD_UNSAFE_AGL_M &&
                             (probe.min_agl_m >= buffer * TERRAIN_MARGIN_LOW_MIN_FRACTION ||
                              probe.worst_frac >= TERRAIN_REENTRY_NEAR_FRAC);
    if (!replan_due && !hard_failed) {
        target = _terrain.evasion_target;
        source = "Terrain Evasion";
        if (_log_level.get() > 1 &&
            (_terrain.last_commit_hold_log_ms == 0 || now_ms - _terrain.last_commit_hold_log_ms >= TERRAIN_DEBUG_LOG_MIN_MS)) {
            _terrain.last_commit_hold_log_ms = now_ms;
            backend.send_text(MAV_SEVERITY_INFO, "SoarNav: TEh%c b%.0f a%.0f t%.1f",
                              _te_side_char(_terrain.committed_turn_side != 0 ? _terrain.committed_turn_side : _terrain.last_turn_side),
                              (double)bearing,
                              (double)min_agl,
                              (double)probe.first_threat_time_s);
        }
        return true;
    }

    if (soft_ok || current_hard_escape || hard_hold || late_margin || (!immediate_threat && !significant_drop)) {
        target = _terrain.evasion_target;
        source = "Terrain Evasion";
        return true;
    }

    return false;
}

AP_SoarNav::TerrainPolicy AP_SoarNav::_te_evaluate_policy(Backend &backend, const Location &loc, const Location &path_target, bool path_valid, const TerrainSideScan &side_scan, bool side_scan_valid, uint32_t now_ms) const
{
    TerrainPolicy policy{};
    policy.decision = TerrainDecision::CLEAR;
    policy.reason = TerrainReplanReason::NONE;
    policy.path_sampled = false;
    policy.side_scan_valid = side_scan_valid;
    policy.side_scan = side_scan;
    policy.side_wall_detected = side_scan_valid && side_scan.wall_side != 0;
    policy.side_memory_active = _te_wall_memory_active(now_ms);
    policy.hard_unsafe_active = _te_hard_unsafe_active(now_ms);
    policy.buffer_m = _terrain_buffer_m(backend);
    policy.speed_mps = _nav_speed_mps(backend);
    policy.yaw_deg = _te_track_or_yaw_deg(backend);
    policy.target_bearing_deg = path_valid ? _bearing_deg(loc, path_target) : policy.yaw_deg;
    policy.target_distance_m = _te_guided_target_distance(backend);
    policy.turn_time_critical = false;
    policy.turn_time_s = 0.0f;
    policy.threat_time_s = -1.0f;
    policy.current_min_agl_m = 1.0e9f;
    policy.current_hard_agl_m = 1.0e9f;
    policy.side_escape_bearing_deg = policy.side_memory_active && _terrain.wall_side != 0 ? _terrain.escape_heading_deg :
                                     (policy.side_wall_detected ? side_scan.escape_bearing_deg : policy.yaw_deg);

    if (!path_valid) {
        if (policy.hard_unsafe_active || policy.side_memory_active) {
            policy.decision = TerrainDecision::EMERGENCY_BEST_OF_BAD;
            policy.reason = TerrainReplanReason::EMERGENCY_BEST_OF_BAD;
        }
        return policy;
    }

    float worst = 0.0f;
    policy.path_sampled = _path_min_agl_probe(backend, loc, path_target, policy.current_min_agl_m, worst, &policy.probe);
    if (!policy.path_sampled) {
        policy.decision = (policy.hard_unsafe_active || policy.side_memory_active) ? TerrainDecision::EMERGENCY_BEST_OF_BAD : TerrainDecision::MONITOR_ONLY;
        policy.reason = TerrainReplanReason::TERRAIN_DATA_INVALID;
        return policy;
    }

    float motion_min_agl = policy.probe.center_min_agl_m;
    if (side_scan_valid) {
        motion_min_agl = side_scan.center_min_agl_m;
        policy.current_min_agl_m = MIN(policy.current_min_agl_m, motion_min_agl);
    }
    policy.current_hard_agl_m = MIN(policy.current_min_agl_m, motion_min_agl);
    const float target_turn_deg = fabsf(_wrap_180(policy.target_bearing_deg - policy.yaw_deg));
    policy.turn_time_s = _te_turn_time_s(backend, target_turn_deg);
    policy.threat_time_s = policy.probe.first_threat_time_s;
    const float configured_turn_deg = constrain_float(MAX(fabsf(float(_reroute_min_deg.get())),
                                                           fabsf(float(_reroute_max_deg.get()))),
                                                      TERRAIN_WALL_TURN_GUARD_DEG,
                                                      TERRAIN_EMERGENCY_REVERSAL_DEG);
    const float scheduler_s = float(TERRAIN_CHECK_INTERVAL_MS) * 0.001f;
    const float reaction_s = MAX(scheduler_s, MAX(float(_terrain_lookahead_s.get()), 1.0f) * 0.15f);
    const bool motion_threat_timed = side_scan_valid && side_scan.center_valid &&
                                     side_scan.center_first_threat_time_s >= 0.0f &&
                                     isfinite(side_scan.center_first_threat_time_s);
    const bool motion_turn_time_critical = motion_threat_timed &&
                                           side_scan.center_first_threat_time_s <= _te_turn_time_s(backend, configured_turn_deg) + reaction_s;
    const bool motion_rising_pressure = side_scan_valid && side_scan.center_valid &&
                                        side_scan.center_terrain_delta_m > MAX(_te_reentry_terrain_delta_m(policy.buffer_m) * 0.50f,
                                                                               policy.buffer_m * 0.20f) &&
                                        side_scan.center_min_agl_m < policy.buffer_m + _te_entry_headroom_m(policy.buffer_m);
    if (motion_threat_timed &&
        (policy.threat_time_s < 0.0f || side_scan.center_first_threat_time_s < policy.threat_time_s)) {
        policy.threat_time_s = side_scan.center_first_threat_time_s;
    }
    policy.turn_time_critical = _te_probe_turn_time_critical(backend, policy.probe, policy.buffer_m, target_turn_deg) ||
                                motion_turn_time_critical;
    const bool motion_immediate_threat = side_scan_valid && isfinite(motion_min_agl) &&
        (motion_min_agl < policy.buffer_m * TERRAIN_CRITICAL_BUFFER_FRACTION ||
         motion_turn_time_critical ||
         (side_scan.center_valid &&
          side_scan.center_min_agl_m < policy.buffer_m &&
          (side_scan.center_worst_frac < TERRAIN_REENTRY_NEAR_FRAC || motion_rising_pressure)));
    const bool motion_low_pressure = side_scan_valid && isfinite(motion_min_agl) &&
        (motion_min_agl < policy.buffer_m * TERRAIN_LOW_BUFFER_FRACTION ||
         motion_rising_pressure ||
         (side_scan.center_valid &&
          side_scan.center_min_agl_m < policy.buffer_m &&
          (side_scan.center_worst_frac < TERRAIN_REENTRY_NEAR_FRAC || motion_turn_time_critical)));
    policy.immediate_threat = _te_probe_immediate_threat(policy.probe, policy.buffer_m) ||
                              policy.turn_time_critical ||
                              motion_immediate_threat;
    policy.low_altitude_pressure = _te_low_altitude_pressure(policy.probe, policy.buffer_m) ||
                                   motion_low_pressure;

    const bool motion_center_window_clear = side_scan_valid &&
                                            side_scan.center_valid &&
                                            isfinite(policy.probe.start_hagl_m) &&
                                            policy.probe.start_hagl_m >= policy.buffer_m + _te_exit_margin_m(policy.buffer_m) &&
                                            side_scan.center_min_agl_m >= policy.buffer_m + _te_exit_margin_m(policy.buffer_m) &&
                                            side_scan.corridor_min_agl_m >= policy.buffer_m &&
                                            (!motion_threat_timed ||
                                             side_scan.center_first_threat_time_s > _te_turn_time_s(backend, configured_turn_deg) + reaction_s);
    const bool motion_window_clear = motion_center_window_clear &&
                                     !policy.side_wall_detected &&
                                     !policy.side_memory_active;
    const bool path_threat_is_deferred = policy.probe.first_threat_time_s < 0.0f ||
                                         policy.probe.first_threat_time_s > _te_turn_time_s(backend, target_turn_deg) +
                                                                            MAX(reaction_s, MAX(float(_terrain_lookahead_s.get()), 1.0f) * TERRAIN_EARLY_THREAT_FRACTION);
    const bool side_wall_deferred_ok = !policy.side_wall_detected ||
                                       (side_scan_valid &&
                                        side_scan.center_valid &&
                                        !side_scan.uniform_low &&
                                        side_scan.center_min_agl_m >= policy.buffer_m + _te_entry_headroom_m(policy.buffer_m) &&
                                        side_scan.corridor_min_agl_m >= policy.buffer_m);
    const bool deferred_pressure_only = motion_center_window_clear &&
                                        side_wall_deferred_ok &&
                                        path_threat_is_deferred &&
                                        !policy.hard_unsafe_active;
    if (deferred_pressure_only) {
        policy.current_min_agl_m = motion_min_agl;
        policy.current_hard_agl_m = motion_min_agl;
        policy.threat_time_s = side_scan.center_first_threat_time_s;
        policy.turn_time_critical = false;
        policy.immediate_threat = false;
        policy.low_altitude_pressure = false;
    }

    const bool stale_target_path_not_motion = side_scan_valid &&
                                              side_scan.center_valid &&
                                              target_turn_deg > TERRAIN_SIDE_WALL_HEADING_LOCK_DEG &&
                                              !motion_turn_time_critical &&
                                              side_scan.center_min_agl_m >= policy.buffer_m * TERRAIN_LOW_BUFFER_FRACTION &&
                                              side_scan.corridor_min_agl_m >= TERRAIN_HARD_UNSAFE_AGL_M &&
                                              (!motion_threat_timed ||
                                               side_scan.center_first_threat_time_s >
                                               _te_turn_time_s(backend, TERRAIN_SIDE_WALL_HEADING_LOCK_DEG) + reaction_s);
    if (stale_target_path_not_motion) {
        policy.current_min_agl_m = motion_min_agl;
        policy.current_hard_agl_m = motion_min_agl;
        policy.turn_time_critical = motion_turn_time_critical;
        policy.threat_time_s = side_scan.center_first_threat_time_s;
        policy.immediate_threat = motion_immediate_threat;
        policy.low_altitude_pressure = motion_low_pressure;
    }

    const float center_agl = side_scan_valid && side_scan.center_valid ? side_scan.center_min_agl_m : policy.probe.center_min_agl_m;
    const float center_worst_frac = side_scan_valid && side_scan.center_valid ? side_scan.center_worst_frac : policy.probe.worst_frac;
    const bool center_valid = side_scan_valid ? side_scan.center_valid : isfinite(policy.probe.center_min_agl_m);

    policy.front_clear = center_valid && !policy.hard_unsafe_active &&
                         center_agl >= policy.buffer_m + _te_entry_hysteresis_m(policy.buffer_m);
    policy.front_margin = center_valid && !policy.hard_unsafe_active &&
                          center_agl >= TERRAIN_HARD_UNSAFE_AGL_M &&
                          (center_agl >= policy.buffer_m - _te_margin_low_deficit_m(policy.buffer_m) ||
                           (center_agl >= policy.buffer_m * TERRAIN_MARGIN_LOW_MIN_FRACTION &&
                            center_worst_frac >= TERRAIN_REENTRY_NEAR_FRAC));

    const bool deferred_path_only = deferred_pressure_only ||
                                    (motion_window_clear && path_threat_is_deferred && !policy.hard_unsafe_active);
    policy.critical_override = !deferred_path_only &&
                               (policy.current_hard_agl_m < TERRAIN_HARD_UNSAFE_AGL_M ||
                                policy.probe.start_hagl_m < policy.buffer_m * TERRAIN_CRITICAL_BUFFER_FRACTION ||
                                policy.current_min_agl_m < policy.buffer_m * TERRAIN_CRITICAL_BUFFER_FRACTION ||
                                policy.turn_time_critical ||
                                (policy.immediate_threat && policy.low_altitude_pressure));

    policy.side_wall_relevant = (policy.side_wall_detected || policy.side_memory_active) &&
                                !policy.front_clear && !policy.front_margin &&
                                !policy.critical_override;

    if (policy.critical_override) {
        policy.decision = TerrainDecision::CRITICAL_OVERRIDE;
        policy.reason = TerrainReplanReason::IMMEDIATE_COLLISION;
        return policy;
    }

    if (deferred_pressure_only) {
        policy.decision = TerrainDecision::MONITOR_ONLY;
        policy.reason = TerrainReplanReason::DEFERRED_PRESSURE;
        return policy;
    }

    if (policy.current_min_agl_m >= policy.buffer_m && !policy.low_altitude_pressure) {
        policy.decision = policy.side_wall_relevant ? TerrainDecision::MONITOR_ONLY : TerrainDecision::CLEAR;
        policy.reason = TerrainReplanReason::PATH_SAFE_STABLE;
        return policy;
    }

    if (motion_window_clear && !policy.immediate_threat && !policy.low_altitude_pressure) {
        policy.decision = TerrainDecision::MONITOR_ONLY;
        policy.reason = TerrainReplanReason::PATH_SAFE_STABLE;
        return policy;
    }

    if ((policy.front_clear || policy.front_margin) && !policy.immediate_threat) {
        if (policy.current_min_agl_m < policy.buffer_m) {
            policy.decision = TerrainDecision::FORWARD_CORRIDOR_COMMIT;
            policy.reason = policy.front_clear ? TerrainReplanReason::FORWARD_SAFE : TerrainReplanReason::MARGIN_LOW;
        } else {
            policy.decision = TerrainDecision::MONITOR_ONLY;
            policy.reason = TerrainReplanReason::PATH_SAFE_STABLE;
        }
        return policy;
    }

    if (_te_probe_margin_low_ok(policy.probe, policy.buffer_m) && !policy.immediate_threat) {
        policy.decision = TerrainDecision::MONITOR_ONLY;
        policy.reason = TerrainReplanReason::MARGIN_LOW;
        return policy;
    }

    if (policy.side_wall_relevant && (policy.current_min_agl_m < policy.buffer_m || policy.low_altitude_pressure || policy.immediate_threat)) {
        policy.decision = TerrainDecision::SIDE_ESCAPE_COMMIT;
        policy.reason = TerrainReplanReason::SIDE_SWITCH_STRONG_GAIN;
        return policy;
    }

    if (policy.current_min_agl_m < policy.buffer_m || policy.low_altitude_pressure || policy.immediate_threat) {
        policy.decision = TerrainDecision::FORWARD_CORRIDOR_COMMIT;
        policy.reason = policy.immediate_threat ? TerrainReplanReason::CORRIDOR_COLLAPSE : TerrainReplanReason::MARGIN_LOW;
        return policy;
    }

    if (policy.hard_unsafe_active) {
        policy.decision = TerrainDecision::RECOVERY;
        policy.reason = TerrainReplanReason::COMMITTED_TARGET_WORSE;
        return policy;
    }

    policy.decision = TerrainDecision::MONITOR_ONLY;
    policy.reason = TerrainReplanReason::PATH_SAFE_STABLE;
    return policy;
}

bool AP_SoarNav::_te_select_policy_target(Backend &backend, const Location &loc, const TerrainPolicy &policy, Location &target, float &agl, float &dist, float &bearing_deg, TerrainCandidateKind &kind)
{
    target = Location{};
    agl = -1.0e9f;
    dist = 0.0f;
    bearing_deg = policy.yaw_deg;
    kind = TerrainCandidateKind::NONE;

    const bool allow_large_reversal = policy.decision == TerrainDecision::CRITICAL_OVERRIDE ||
                                      policy.decision == TerrainDecision::EMERGENCY_BEST_OF_BAD ||
                                      policy.current_hard_agl_m < TERRAIN_HARD_UNSAFE_AGL_M ||
                                      policy.hard_unsafe_active;
    const float corridor_distance_m = MAX(policy.target_distance_m, _te_degraded_target_distance(backend));

    if (policy.decision == TerrainDecision::FORWARD_CORRIDOR_COMMIT || policy.decision == TerrainDecision::RECOVERY) {
        if (_te_select_forward_corridor(backend, loc, corridor_distance_m, target, agl, dist, bearing_deg)) {
            kind = policy.decision == TerrainDecision::RECOVERY ? TerrainCandidateKind::RECOVERY : TerrainCandidateKind::FORWARD_CORRIDOR;
            return true;
        }
        if (_te_select_corridor_best_effort(backend, loc, policy.yaw_deg, policy.target_distance_m, allow_large_reversal, target, agl, dist, bearing_deg)) {
            kind = TerrainCandidateKind::BEST_OF_BAD;
            return true;
        }
    }

    if (policy.decision == TerrainDecision::SIDE_ESCAPE_COMMIT) {
        const int8_t required_side = policy.side_scan_valid ? policy.side_scan.escape_side : 0;
        if (required_side != 0 && _te_select_escape_fan_target(backend, loc, policy, required_side, target, agl, dist, bearing_deg)) {
            kind = TerrainCandidateKind::SIDE_ESCAPE;
            return true;
        }
        if (_te_select_forward_corridor(backend, loc, corridor_distance_m, target, agl, dist, bearing_deg)) {
            kind = TerrainCandidateKind::FORWARD_CORRIDOR;
            return true;
        }
        if (required_side == 0 && _te_select_escape_fan_target(backend, loc, policy, required_side, target, agl, dist, bearing_deg)) {
            kind = TerrainCandidateKind::SIDE_ESCAPE;
            return true;
        }
        Location side_target;
        if (_te_make_candidate(backend, loc, policy.side_escape_bearing_deg, policy.target_distance_m, side_target)) {
            float side_worst = 0.0f;
            TerrainProbe side_probe{};
            if (_path_min_agl_probe(backend, loc, side_target, agl, side_worst, &side_probe) &&
                side_probe.target_distance_m <= side_probe.sample_distance_m + _te_entry_hysteresis_m(policy.buffer_m) &&
                side_probe.corridor_min_agl_m >= TERRAIN_HARD_UNSAFE_AGL_M &&
                !_te_heading_turns_towards_wall(policy.side_escape_bearing_deg, policy.yaw_deg, AP_HAL::millis())) {
                target = side_target;
                dist = loc.get_distance(side_target);
                bearing_deg = policy.side_escape_bearing_deg;
                kind = TerrainCandidateKind::SIDE_ESCAPE;
                return true;
            }
        }
        if (_te_select_corridor_best_effort(backend, loc, policy.side_escape_bearing_deg, policy.target_distance_m, allow_large_reversal, target, agl, dist, bearing_deg)) {
            kind = TerrainCandidateKind::SIDE_ESCAPE;
            return true;
        }
        if (_te_select_forward_corridor(backend, loc, corridor_distance_m, target, agl, dist, bearing_deg)) {
            kind = TerrainCandidateKind::FORWARD_CORRIDOR;
            return true;
        }
    }

    if (policy.decision == TerrainDecision::CRITICAL_OVERRIDE || policy.decision == TerrainDecision::EMERGENCY_BEST_OF_BAD) {
        const int8_t required_side = policy.side_scan_valid ? policy.side_scan.escape_side : 0;
        const bool prefer_side_escape = required_side != 0 &&
                                        (policy.side_wall_detected ||
                                         policy.side_memory_active ||
                                         !policy.front_clear ||
                                         policy.turn_time_critical ||
                                         policy.current_hard_agl_m < TERRAIN_HARD_UNSAFE_AGL_M);
        if (prefer_side_escape && _te_select_escape_fan_target(backend, loc, policy, required_side, target, agl, dist, bearing_deg)) {
            kind = policy.decision == TerrainDecision::CRITICAL_OVERRIDE ? TerrainCandidateKind::CRITICAL : TerrainCandidateKind::BEST_OF_BAD;
            return true;
        }
        if (_te_select_forward_corridor(backend, loc, corridor_distance_m, target, agl, dist, bearing_deg)) {
            kind = TerrainCandidateKind::FORWARD_CORRIDOR;
            return true;
        }
        if (!prefer_side_escape && _te_select_escape_fan_target(backend, loc, policy, required_side, target, agl, dist, bearing_deg)) {
            kind = policy.decision == TerrainDecision::CRITICAL_OVERRIDE ? TerrainCandidateKind::CRITICAL : TerrainCandidateKind::BEST_OF_BAD;
            return true;
        }
        if (_te_select_critical_best_effort(backend, loc, policy, target, agl, dist, bearing_deg)) {
            kind = policy.decision == TerrainDecision::CRITICAL_OVERRIDE ? TerrainCandidateKind::CRITICAL : TerrainCandidateKind::BEST_OF_BAD;
            return true;
        }
        if (_te_select_corridor_best_effort(backend, loc, policy.yaw_deg, policy.target_distance_m, true, target, agl, dist, bearing_deg)) {
            kind = TerrainCandidateKind::BEST_OF_BAD;
            return true;
        }
        Location emergency;
        if (_te_emergency_area_target(backend, loc, policy.target_bearing_deg, _te_degraded_target_distance(backend), emergency)) {
            float worst = 0.0f;
            TerrainProbe probe{};
            if (_path_min_agl_probe(backend, loc, emergency, agl, worst, &probe)) {
                target = emergency;
                dist = loc.get_distance(emergency);
                bearing_deg = _bearing_deg(loc, emergency);
                kind = TerrainCandidateKind::BEST_OF_BAD;
                return true;
            }
        }
    }

    return false;
}

void AP_SoarNav::_te_commit_policy_target(Backend &backend, const Location &loc, const Location &selected, float selected_agl, float selected_dist, float selected_bearing, TerrainDecision decision, TerrainReplanReason reason, TerrainCandidateKind kind)
{
    const uint32_t now = AP_HAL::millis();
    const float buffer = _terrain_buffer_m(backend);
    if (!_terrain.resume_valid && _target_valid && !_source_is_terrain_evasion(_target_source)) {
        _terrain.resume_target = _target;
        _terrain.resume_valid = true;
        _te_save_resume_source();
    }
    const bool new_commit = !_terrain.evasion_target_valid || _terrain.state == TerrainState::IDLE || _terrain.commitment_started_ms == 0;
    const int8_t selected_side = _te_turn_side_from_bearing(backend, selected_bearing);
    const int8_t previous_side = _terrain.committed_turn_side != 0 ? _terrain.committed_turn_side : _terrain.last_turn_side;

    TerrainProbe selected_probe{};
    float selected_probe_agl = selected_agl;
    float selected_probe_worst = 0.0f;
    const bool selected_probe_ok = _path_min_agl_probe(backend, loc, selected, selected_probe_agl, selected_probe_worst, &selected_probe);

    const bool center_escape_commit = selected_probe_ok &&
                                      (kind == TerrainCandidateKind::SIDE_ESCAPE ||
                                       kind == TerrainCandidateKind::CRITICAL ||
                                       kind == TerrainCandidateKind::BEST_OF_BAD) &&
                                      selected_agl > selected_probe_agl &&
                                      selected_probe.center_min_agl_m >= TERRAIN_HARD_UNSAFE_AGL_M;
    const float commit_agl = center_escape_commit ? selected_agl : (selected_probe_ok ? selected_probe_agl : selected_agl);

    _terrain.resume_safe_count = 0;
    _terrain.evasion_target = selected;
    _terrain.evasion_target_valid = true;
    _terrain.evasion_degraded = commit_agl < buffer;
    _terrain.evasion_target_updated_ms = now;
    _terrain.evasion_initial_distance_m = selected_dist;
    _terrain.evasion_min_agl_m = commit_agl;
    _terrain.last_safe_heading_deg = selected_bearing;
    _terrain.last_safe_heading_valid = true;
    _terrain.last_turn_side = selected_side != 0 ? selected_side :
                              (previous_side != 0 ? previous_side :
                               (_wrap_180(selected_bearing - _te_track_or_yaw_deg(backend)) >= 0.0f ? 1 : -1));
    if (new_commit || _terrain.committed_turn_side == 0) {
        _terrain.committed_turn_side = _terrain.last_turn_side;
        _terrain.commitment_started_ms = now;
    } else if (selected_side != 0 && selected_side != _terrain.committed_turn_side) {
        _terrain.committed_turn_side = selected_side;
        _terrain.commitment_started_ms = now;
    }
    _terrain.committed_bearing_deg = selected_bearing;
    _terrain.committed_min_agl_m = _terrain.evasion_min_agl_m;
    _terrain.committed_first_threat_time_s = selected_probe_ok ? selected_probe.first_threat_time_s : -1.0f;
    _terrain.committed_first_threat_distance_m = selected_probe_ok ? selected_probe.first_threat_distance_m : -1.0f;
    _terrain.state = kind == TerrainCandidateKind::RECOVERY ? TerrainState::HOLD : TerrainState::EVADING;
    _terrain.last_decision = decision;
    _terrain.last_replan_reason = reason;
    _terrain.replan_not_before_ms = now + _te_replan_backoff_ms();
    _te_block_reroute(now, _te_post_reroute_block_ms());
    const uint32_t hold_min = _terrain.evasion_degraded ? _te_degraded_hold_min_ms() : _te_hold_min_ms();
    _terrain.hold_until_ms = now + uint32_t(constrain_float((MAX(selected_dist, 1.0f) / MAX(_nav_speed_mps(backend), 1.0f)) * 1000.0f,
                                                           float(hold_min),
                                                           float(_te_hold_max_ms())));
    _terrain.next_check_ms = now + TERRAIN_CHECK_INTERVAL_MS;
    if (_log_level.get() > 1 &&
        (new_commit || previous_side != _terrain.committed_turn_side || _terrain.last_commit_log_ms == 0 || now - _terrain.last_commit_log_ms >= TERRAIN_POLICY_LOG_CHANGE_MS)) {
        _terrain.last_commit_log_ms = now;
        backend.send_text(MAV_SEVERITY_INFO, "SoarNav: TEc%c b%.0f a%.0f d%.0f t%.1f %s/%s",
                          _te_side_char(_terrain.committed_turn_side),
                          (double)selected_bearing,
                          (double)_terrain.evasion_min_agl_m,
                          (double)selected_dist,
                          (double)_terrain.committed_first_threat_time_s,
                          _te_candidate_kind_name(kind),
                          _te_reason_name(reason));
    }
    if (_terrain.evasion_min_agl_m < TERRAIN_HARD_UNSAFE_AGL_M) {
        (void)_te_note_hard_unsafe(backend, now, _terrain.evasion_min_agl_m, buffer, _te_candidate_kind_name(kind));
    }
}

const char *AP_SoarNav::_te_decision_name(TerrainDecision decision) const
{
    switch (decision) {
    case TerrainDecision::CLEAR: return "CLR";
    case TerrainDecision::MONITOR_ONLY: return "MON";
    case TerrainDecision::FORWARD_CORRIDOR_COMMIT: return "FWD";
    case TerrainDecision::SIDE_ESCAPE_COMMIT: return "SIDE";
    case TerrainDecision::CRITICAL_OVERRIDE: return "CRIT";
    case TerrainDecision::EMERGENCY_BEST_OF_BAD: return "BAD";
    case TerrainDecision::RECOVERY: return "REC";
    case TerrainDecision::RELEASE_TE: return "REL";
    }
    return "UNK";
}

const char *AP_SoarNav::_te_reason_name(TerrainReplanReason reason) const
{
    switch (reason) {
    case TerrainReplanReason::NONE: return "-";
    case TerrainReplanReason::FORWARD_SAFE: return "SAFE";
    case TerrainReplanReason::MARGIN_LOW: return "LOW";
    case TerrainReplanReason::IMMEDIATE_COLLISION: return "IMM";
    case TerrainReplanReason::CORRIDOR_COLLAPSE: return "COR";
    case TerrainReplanReason::COMMITTED_TARGET_WORSE: return "WORSE";
    case TerrainReplanReason::SIDE_SWITCH_STRONG_GAIN: return "GAIN";
    case TerrainReplanReason::TERRAIN_DATA_INVALID: return "DATA";
    case TerrainReplanReason::EMERGENCY_BEST_OF_BAD: return "BEST";
    case TerrainReplanReason::PATH_SAFE_STABLE: return "STABLE";
    case TerrainReplanReason::DEFERRED_PRESSURE: return "DEFER";
    }
    return "UNK";
}

const char *AP_SoarNav::_te_candidate_kind_name(TerrainCandidateKind kind) const
{
    switch (kind) {
    case TerrainCandidateKind::NONE: return "-";
    case TerrainCandidateKind::FORWARD_CORRIDOR: return "FWD";
    case TerrainCandidateKind::SIDE_ESCAPE: return "SIDE";
    case TerrainCandidateKind::CRITICAL: return "CRIT";
    case TerrainCandidateKind::BEST_OF_BAD: return "BAD";
    case TerrainCandidateKind::RECOVERY: return "REC";
    }
    return "UNK";
}

const char *AP_SoarNav::_te_context_code(const char *context) const
{
    if (context == nullptr || context[0] == '\0') {
        return "-";
    }
    if (strcmp(context, "target_retry") == 0) {
        return "RETRY";
    }
    if (strcmp(context, "no_target") == 0) {
        return "NOTGT";
    }
    if (strcmp(context, "area_reject") == 0) {
        return "AREA";
    }
    return context;
}

void AP_SoarNav::_te_log_policy(Backend &backend, uint32_t now_ms, const TerrainPolicy &policy, TerrainCandidateKind kind, float selected_agl, float selected_dist, const char *context)
{
    (void)kind;
    (void)selected_agl;
    (void)selected_dist;
    if (_log_level.get() < 2) {
        return;
    }
    const bool changed = _terrain.last_policy_log_decision != policy.decision ||
                         _terrain.last_policy_log_reason != policy.reason;
    const uint32_t min_interval = changed ? TERRAIN_POLICY_LOG_CHANGE_MS : TERRAIN_POLICY_LOG_REPEAT_MS;
    if (_terrain.last_policy_log_ms != 0 && now_ms - _terrain.last_policy_log_ms < min_interval) {
        return;
    }
    _terrain.last_policy_log_ms = now_ms;
    _terrain.last_policy_log_decision = policy.decision;
    _terrain.last_policy_log_reason = policy.reason;

    const float fwd_agl = constrain_float(policy.side_scan_valid ? policy.side_scan.center_min_agl_m : policy.probe.center_min_agl_m, -999.0f, 9999.0f);
    const float left_agl = constrain_float(policy.side_scan_valid ? policy.side_scan.left_min_agl_m : policy.probe.left_min_agl_m, -999.0f, 9999.0f);
    const float right_agl = constrain_float(policy.side_scan_valid ? policy.side_scan.right_min_agl_m : policy.probe.right_min_agl_m, -999.0f, 9999.0f);
    const float cur_agl = constrain_float(policy.current_min_agl_m, -999.0f, 9999.0f);
    if (context != nullptr && context[0] != '\0') {
        backend.send_text(MAV_SEVERITY_INFO, "SoarNav: TEp %s/%s c%s F%.0f L%.0f R%.0f A%.0f B%.0f",
                          _te_decision_name(policy.decision),
                          _te_reason_name(policy.reason),
                          _te_context_code(context),
                          (double)fwd_agl,
                          (double)left_agl,
                          (double)right_agl,
                          (double)cur_agl,
                          (double)policy.buffer_m);
    } else {
        backend.send_text(MAV_SEVERITY_INFO, "SoarNav: TEp %s/%s F%.0f L%.0f R%.0f A%.0f B%.0f",
                          _te_decision_name(policy.decision),
                          _te_reason_name(policy.reason),
                          (double)fwd_agl,
                          (double)left_agl,
                          (double)right_agl,
                          (double)cur_agl,
                          (double)policy.buffer_m);
    }
}

bool AP_SoarNav::_terrain_evasion_update(Backend &backend, const Location &loc, Location &target, const char *&source)
{
    if (_dynamic_soar_alt.get() != 3) {
        _te_clear_state(true);
        return false;
    }

    const uint32_t now = AP_HAL::millis();
    const bool active = _terrain.state != TerrainState::IDLE;

    if (!_target_valid && !active) {
        const char *emergency_source = "Terrain Evasion";
        if (_te_force_emergency_target(backend, loc, target, emergency_source, "no_target")) {
            source = emergency_source;
            return true;
        }
        return false;
    }

    const float te_lever = MAX(_te_lookahead_distance_m(backend), _te_lever_min_m(backend));
    TerrainSideScan side_scan{};
    const bool scan_due = _terrain.next_check_ms == 0 || now >= _terrain.next_check_ms || !active;
    const bool side_scan_valid = scan_due && _te_side_wall_scan(backend, loc, te_lever, side_scan);
    if (side_scan_valid) {
        (void)_te_update_side_wall_memory(backend, now, side_scan);
    }
    if (scan_due) {
        _terrain.next_check_ms = now + TERRAIN_CHECK_INTERVAL_MS;
    }

    const float active_buffer = _terrain_buffer_m(backend);
    bool current_evasion_stale = false;
    if (active && _terrain.evasion_target_valid && side_scan_valid && side_scan.center_valid) {
        const float evasion_bearing = _bearing_deg(loc, _terrain.evasion_target);
        const float evasion_turn = fabsf(_wrap_180(evasion_bearing - _te_track_or_yaw_deg(backend)));
        const float scheduler_s = float(TERRAIN_CHECK_INTERVAL_MS) * 0.001f;
        const bool forward_time_ok = side_scan.center_first_threat_time_s < 0.0f ||
                                     side_scan.center_first_threat_time_s >
                                     _te_turn_time_s(backend, TERRAIN_SIDE_WALL_HEADING_LOCK_DEG) + scheduler_s;
        const bool motion_forward_clear = side_scan.center_min_agl_m >= active_buffer + _te_exit_margin_m(active_buffer) &&
                                          side_scan.corridor_min_agl_m >= TERRAIN_HARD_UNSAFE_AGL_M &&
                                          !side_scan.weak_asymmetry &&
                                          !side_scan.uniform_low &&
                                          forward_time_ok;
        current_evasion_stale = motion_forward_clear && evasion_turn > TERRAIN_SIDE_WALL_HEADING_LOCK_DEG;
    }

    Location motion_target = loc;
    motion_target.offset_bearing(_te_track_or_yaw_deg(backend), _te_guided_target_distance(backend));

    Location policy_target;
    bool policy_target_valid = false;
    if (active && _terrain.evasion_target_valid && !current_evasion_stale) {
        policy_target = _terrain.evasion_target;
        policy_target_valid = true;
    } else if (active && _point_in_area(motion_target)) {
        policy_target = motion_target;
        policy_target_valid = true;
    } else if (_target_valid && !_source_is_terrain_evasion(_target_source)) {
        policy_target = _target;
        policy_target_valid = true;
    } else if (_target_valid) {
        policy_target = _target;
        policy_target_valid = true;
    }

    TerrainPolicy policy = _te_evaluate_policy(backend, loc, policy_target, policy_target_valid, side_scan, side_scan_valid, now);
    const float buffer = policy.buffer_m;

    if (policy.path_sampled) {
        if (policy.current_hard_agl_m < TERRAIN_HARD_UNSAFE_AGL_M) {
            (void)_te_note_hard_unsafe(backend, now, policy.current_hard_agl_m, buffer, _te_reason_name(policy.reason));
        } else if (policy.current_hard_agl_m >= buffer) {
            _te_note_hard_safe(now);
        }
    }

    const bool override_replan = policy.decision == TerrainDecision::CRITICAL_OVERRIDE ||
                                 policy.decision == TerrainDecision::EMERGENCY_BEST_OF_BAD;

    if (active) {
        if (_terrain.resume_valid && policy.path_sampled && !override_replan) {
            float resume_min_agl = 0.0f;
            float resume_worst_frac = 0.0f;
            TerrainProbe resume_probe{};
            const bool resume_sampled = _path_min_agl_probe(backend, loc, _terrain.resume_target, resume_min_agl, resume_worst_frac, &resume_probe);
            const bool resume_path_clear = resume_sampled &&
                                           resume_probe.center_min_agl_m >= buffer + _te_exit_margin_m(buffer) &&
                                           resume_probe.corridor_min_agl_m >= TERRAIN_HARD_UNSAFE_AGL_M &&
                                           !_te_probe_immediate_threat(resume_probe, buffer);
            const bool release_margin = resume_path_clear &&
                                        (policy.current_min_agl_m >= buffer + _te_exit_margin_m(buffer) ||
                                         policy.front_clear ||
                                         (policy.front_margin && !_te_wall_memory_active(now)));
            if (release_margin) {
                if (_terrain.resume_safe_count < 255) {
                    _terrain.resume_safe_count++;
                }
            } else {
                _terrain.resume_safe_count = 0;
            }
            if (now >= _terrain.hold_until_ms && _terrain.resume_safe_count >= TERRAIN_RELEASE_SAFE_CHECKS) {
                target = _terrain.resume_target;
                source = _terrain.resume_source[0] != 0 ? _terrain.resume_source : "Guided";
                _terrain.last_decision = TerrainDecision::RELEASE_TE;
                _terrain.last_replan_reason = TerrainReplanReason::PATH_SAFE_STABLE;
                _terrain.post_resume_until_ms = now + _te_reentry_cooldown_ms();
                _next_target_search_ms = MAX(_next_target_search_ms, now + _te_reentry_cooldown_ms());
                _reset_progress_monitor(GUIDED_PROGRESS_ARM_DELAY_MS);
                _te_log_policy(backend, now, policy, TerrainCandidateKind::NONE, policy.current_min_agl_m, 0.0f);
                _te_clear_state(true);
                return true;
            }
        }

        if (_terrain.evasion_target_valid && !current_evasion_stale) {
            const float current_evasion_bearing = _bearing_deg(loc, _terrain.evasion_target);
            const float current_evasion_turn = fabsf(_wrap_180(current_evasion_bearing - policy.yaw_deg));
            const int8_t current_evasion_side = _te_turn_side_from_bearing(backend, current_evasion_bearing);
            const bool current_evasion_towards_wall = policy.side_scan_valid &&
                                                      policy.side_scan.wall_side != 0 &&
                                                      current_evasion_side != 0 &&
                                                      current_evasion_side == policy.side_scan.wall_side;
            const float required_escape_turn = (policy.side_scan_valid && policy.side_scan.escape_side != 0) ? 55.0f : 45.0f;
            const bool critical_timid_target = policy.decision == TerrainDecision::CRITICAL_OVERRIDE &&
                                               policy.reason == TerrainReplanReason::IMMEDIATE_COLLISION &&
                                               !policy.front_clear &&
                                               !policy.front_margin &&
                                               (current_evasion_turn < required_escape_turn || current_evasion_towards_wall) &&
                                               now >= _terrain.replan_not_before_ms;
            if (!critical_timid_target && _te_commit_hold_current_target(backend, loc, &policy, target, source, now)) {
                return true;
            }
            if (!critical_timid_target &&
                now < _terrain.replan_not_before_ms &&
                _te_target_usable(backend, loc, _terrain.evasion_target, true)) {
                target = _terrain.evasion_target;
                source = "Terrain Evasion";
                return true;
            }
        }

        if (policy.decision == TerrainDecision::CLEAR || policy.decision == TerrainDecision::MONITOR_ONLY) {
            if (current_evasion_stale) {
                _terrain.last_decision = TerrainDecision::RELEASE_TE;
                _terrain.last_replan_reason = TerrainReplanReason::PATH_SAFE_STABLE;
                _terrain.post_resume_until_ms = now + _te_reentry_cooldown_ms();
                _next_target_search_ms = MAX(_next_target_search_ms, now + _te_reentry_cooldown_ms());
                _reset_progress_monitor(GUIDED_PROGRESS_ARM_DELAY_MS);
                _te_clear_state(true);
                return false;
            }
            if (_terrain.evasion_target_valid && _te_target_usable(backend, loc, _terrain.evasion_target, true)) {
                target = _terrain.evasion_target;
                source = "Terrain Evasion";
                return true;
            }
            return false;
        }
    } else {
        if (policy.decision == TerrainDecision::CLEAR || policy.decision == TerrainDecision::MONITOR_ONLY) {
            _te_log_policy(backend, now, policy, TerrainCandidateKind::NONE, policy.current_min_agl_m, 0.0f);
            return false;
        }
    }

    Location selected;
    float selected_agl = -1.0e9f;
    float selected_dist = 0.0f;
    float selected_bearing = policy.yaw_deg;
    TerrainCandidateKind kind = TerrainCandidateKind::NONE;

    if (!_te_select_policy_target(backend, loc, policy, selected, selected_agl, selected_dist, selected_bearing, kind)) {
        if (active && _terrain.evasion_target_valid && !current_evasion_stale &&
            _te_target_usable(backend, loc, _terrain.evasion_target, true)) {
            target = _terrain.evasion_target;
            source = "Terrain Evasion";
            return true;
        }
        const char *emergency_source = "Terrain Evasion";
        if (_te_force_emergency_target(backend, loc, target, emergency_source, _te_reason_name(policy.reason))) {
            source = emergency_source;
            return true;
        }
        if (active && _terrain.evasion_target_valid && !current_evasion_stale) {
            target = _terrain.evasion_target;
            source = "Terrain Evasion";
            return true;
        }
        return false;
    }

    if (!_point_in_area(selected) || (_using_polygon && !_segment_stays_inside(loc, selected))) {
        if (active && _terrain.evasion_target_valid && !current_evasion_stale &&
            _te_target_usable(backend, loc, _terrain.evasion_target, true)) {
            target = _terrain.evasion_target;
            source = "Terrain Evasion";
            return true;
        }
        const char *emergency_source = "Terrain Evasion";
        if (_te_force_emergency_target(backend, loc, target, emergency_source, "area_reject")) {
            source = emergency_source;
            return true;
        }
        return false;
    }

    if (active && _terrain.evasion_target_valid && !current_evasion_stale &&
        _te_keep_current_evasion_target(backend, loc, policy, selected, selected_agl, selected_bearing, target, source, now)) {
        return true;
    }

    _te_commit_policy_target(backend, loc, selected, selected_agl, selected_dist, selected_bearing, policy.decision, policy.reason, kind);
    target = selected;
    source = "Terrain Evasion";
    _te_log_policy(backend, now, policy, kind, selected_agl, selected_dist);
    return true;
}

float AP_SoarNav::_te_lookahead_distance_m(Backend &backend) const
{
    return _nav_speed_mps(backend) * _te_planning_horizon_s(backend);
}

float AP_SoarNav::_te_lever_min_time_s() const
{
    return MAX(float(_terrain_lookahead_s.get()), 1.0f) * 0.70f;
}

float AP_SoarNav::_te_guided_target_time_s() const
{
    return MAX(float(_terrain_lookahead_s.get()), 1.0f) * 2.50f;
}

float AP_SoarNav::_te_degraded_advance_time_s() const
{
    return MAX(float(_terrain_lookahead_s.get()), 1.0f) * 3.50f;
}

float AP_SoarNav::_te_climb_credit_min_mps(Backend &backend) const
{
    const float sink = _sink_best_now(backend);
    return MAX(sink * 0.50f, 0.0f);
}

float AP_SoarNav::_te_climb_credit_max_mps(Backend &backend) const
{
    const float sink = _sink_best_now(backend);
    return MAX(_te_climb_credit_min_mps(backend), sink * 2.0f);
}

float AP_SoarNav::_te_time_scaled_distance_m(Backend &backend, float target_time_s, float min_time_s, float lookahead_multiplier) const
{
    const float speed = _nav_speed_mps(backend);
    const float look_s = MAX(float(_terrain_lookahead_s.get()), 1.0f);
    const float buffer = _terrain_buffer_m(backend);
    const float min_s = MAX(min_time_s, 0.1f);
    const float target_s = MAX(target_time_s, min_s);
    const float raw_m = speed * target_s;
    const float floor_m = MAX(speed * min_s, buffer * 0.75f);
    const float ceiling_s = MAX(look_s * MAX(lookahead_multiplier, 1.0f), target_s);
    const float ceiling_m = MAX(speed * ceiling_s, floor_m * 1.25f);
    return constrain_float(raw_m, floor_m, ceiling_m);
}

float AP_SoarNav::_te_lever_min_m(Backend &backend) const
{
    return _te_time_scaled_distance_m(backend, _te_lever_min_time_s(), _te_lever_min_time_s(), 1.0f);
}

float AP_SoarNav::_te_target_sample_limit_m(Backend &backend) const
{
    return MAX(MAX(_te_guided_target_distance(backend), _te_degraded_target_distance(backend)), _te_lookahead_distance_m(backend));
}

float AP_SoarNav::_te_candidate_min_m(Backend &backend) const
{
    const float min_time_s = _te_lever_min_time_s() * 0.30f;
    const float speed_floor_m = _nav_speed_mps(backend) * min_time_s;
    const float buffer_floor_m = _terrain_buffer_m(backend) * 0.50f;
    return MAX(speed_floor_m, buffer_floor_m);
}

float AP_SoarNav::_te_hold_allowed_drop_m(Backend &backend, float buffer_m) const
{
    const uint32_t hold_ms = MAX(_te_hold_min_ms(), TERRAIN_CHECK_INTERVAL_MS);
    const float decision_fraction = float(TERRAIN_CHECK_INTERVAL_MS) / float(hold_ms);
    return MAX(buffer_m, 0.0f) * decision_fraction;
}

float AP_SoarNav::_te_dynamic_margin_m(float buffer_m, float fraction) const
{
    return MAX(buffer_m, 0.0f) * MAX(fraction, 0.0f);
}

float AP_SoarNav::_te_side_wall_flip_gain_m(float buffer_m) const
{
    return _te_dynamic_margin_m(buffer_m, 0.55f);
}

float AP_SoarNav::_te_side_wall_trigger_margin_m(float buffer_m) const
{
    return _te_dynamic_margin_m(buffer_m, 0.30f);
}

float AP_SoarNav::_te_side_wall_clear_margin_m(float buffer_m) const
{
    return _te_dynamic_margin_m(buffer_m, 0.45f);
}

float AP_SoarNav::_te_margin_low_deficit_m(float buffer_m) const
{
    return _te_dynamic_margin_m(buffer_m, 1.0f - TERRAIN_MARGIN_LOW_MIN_FRACTION);
}

float AP_SoarNav::_te_entry_hysteresis_m(float buffer_m) const
{
    return _te_dynamic_margin_m(buffer_m, 0.06f);
}

float AP_SoarNav::_te_entry_headroom_m(float buffer_m) const
{
    return _te_dynamic_margin_m(buffer_m, 0.15f);
}

float AP_SoarNav::_te_flat_delta_m(float buffer_m) const
{
    return _te_dynamic_margin_m(buffer_m, 0.08f);
}

float AP_SoarNav::_te_exit_margin_m(float buffer_m) const
{
    return _te_dynamic_margin_m(buffer_m, 0.15f);
}

float AP_SoarNav::_te_reentry_terrain_delta_m(float buffer_m) const
{
    return _te_dynamic_margin_m(buffer_m, 0.60f);
}

float AP_SoarNav::_te_sample_horizon_m(Backend &backend, float target_distance_m) const
{
    const float target_dist = MAX(target_distance_m, 0.0f);
    if (target_dist <= 0.0f) {
        return 0.0f;
    }
    return MIN(target_dist, _te_target_sample_limit_m(backend));
}

void AP_SoarNav::_te_save_resume_source()
{
    const char *src = _target_source[0] != 0 ? _target_source : "Guided";
    if (strncmp(src, "Terrain Evasion", 16) == 0 || strcmp(src, "TE Egress") == 0 || strcmp(src, "TE Bootstrap") == 0) {
        src = "Guided";
    }
    strncpy(_terrain.resume_source, src, sizeof(_terrain.resume_source) - 1);
    _terrain.resume_source[sizeof(_terrain.resume_source) - 1] = 0;
}

int8_t AP_SoarNav::_te_turn_side_from_bearing(Backend &backend, float bearing_deg) const
{
    const float turn = _wrap_180(bearing_deg - _te_track_or_yaw_deg(backend));
    if (fabsf(turn) <= TERRAIN_WALL_TURN_GUARD_DEG) {
        return 0;
    }
    return turn > 0.0f ? 1 : -1;
}

char AP_SoarNav::_te_side_char(int8_t side) const
{
    if (side > 0) {
        return 'R';
    }
    if (side < 0) {
        return 'L';
    }
    return '-';
}

bool AP_SoarNav::_te_wall_memory_active(uint32_t now_ms) const
{
    if (!_terrain.side_wall_active || _terrain.wall_side == 0) {
        return false;
    }
    return _terrain.side_wall_hold_until_ms == 0 || now_ms < _terrain.side_wall_hold_until_ms ||
           _terrain.side_wall_clear_count < TERRAIN_SIDE_WALL_CLEAR_CHECKS;
}

bool AP_SoarNav::_te_reroute_block_active(uint32_t now_ms) const
{
    return _terrain.reroute_block_until_ms != 0 && now_ms < _terrain.reroute_block_until_ms;
}

void AP_SoarNav::_te_block_reroute(uint32_t now_ms, uint32_t duration_ms)
{
    _terrain.reroute_block_until_ms = MAX(_terrain.reroute_block_until_ms, now_ms + duration_ms);
    _reroute_check_armed = false;
}

bool AP_SoarNav::_te_hard_unsafe_active(uint32_t now_ms) const
{
    return _terrain.hard_unsafe_until_ms != 0 && now_ms < _terrain.hard_unsafe_until_ms;
}

bool AP_SoarNav::_te_note_hard_unsafe(Backend &backend, uint32_t now_ms, float min_agl_m, float buffer_m, const char *reason)
{
    if (!isfinite(min_agl_m) || min_agl_m >= TERRAIN_HARD_UNSAFE_AGL_M) {
        return false;
    }
    const bool was_hard_active = _te_hard_unsafe_active(now_ms);
    if (_terrain.hard_unsafe_count < TERRAIN_HARD_UNSAFE_CHECKS) {
        _terrain.hard_unsafe_count++;
    } else {
        _terrain.hard_unsafe_count = TERRAIN_HARD_UNSAFE_CHECKS;
    }
    if (_terrain.hard_unsafe_count < TERRAIN_HARD_UNSAFE_CHECKS) {
        return false;
    }
    _terrain.hard_unsafe_until_ms = MAX(_terrain.hard_unsafe_until_ms, now_ms + _te_hard_unsafe_hold_ms());
    _terrain.hold_until_ms = MAX(_terrain.hold_until_ms, now_ms + _te_hard_unsafe_hold_ms());
    _terrain.post_resume_until_ms = MAX(_terrain.post_resume_until_ms, now_ms + _te_reentry_cooldown_ms());
    _te_block_reroute(now_ms, _te_post_reroute_block_ms());
    if (_log_level.get() > 1 && !was_hard_active) {
        backend.send_text(MAV_SEVERITY_WARNING, "SoarNav: TEhard %s a%.0f b%.0f",
                          _te_context_code(reason),
                          (double)min_agl_m,
                          (double)buffer_m);
    }
    return true;
}

void AP_SoarNav::_te_note_hard_safe(uint32_t now_ms)
{
    if (_terrain.hard_unsafe_count > 0) {
        _terrain.hard_unsafe_count--;
    }
    if (_terrain.hard_unsafe_count == 0) {
        _terrain.hard_unsafe_until_ms = 0;
    } else if (!_te_hard_unsafe_active(now_ms)) {
        _terrain.hard_unsafe_count = 0;
    }
}

bool AP_SoarNav::_te_heading_turns_towards_wall(float heading_deg, float reference_heading_deg, uint32_t now_ms) const
{
    if (!_te_wall_memory_active(now_ms) || _terrain.wall_side == 0) {
        return false;
    }
    const float turn = _wrap_180(heading_deg - reference_heading_deg);
    if (fabsf(turn) < TERRAIN_WALL_TURN_GUARD_DEG) {
        return false;
    }
    return turn * float(_terrain.wall_side) > 0.0f;
}

bool AP_SoarNav::_te_side_wall_scan(Backend &backend, const Location &loc, float lever_m, TerrainSideScan &scan) const
{
    scan = {};
    scan.left_min_agl_m = 1.0e9f;
    scan.right_min_agl_m = 1.0e9f;
    scan.center_min_agl_m = 1.0e9f;
    scan.corridor_min_agl_m = 1.0e9f;
    scan.center_worst_frac = 1.0f;
    scan.center_first_threat_distance_m = -1.0f;
    scan.center_first_threat_time_s = -1.0f;
    scan.center_sample_distance_m = 0.0f;
    scan.center_terrain_delta_m = 0.0f;
    scan.escape_bearing_deg = _te_track_or_yaw_deg(backend);

    const float yaw_deg = _te_track_or_yaw_deg(backend);
    const float right_brg = _wrap_360(yaw_deg + 90.0f);
    const float left_brg = _wrap_360(yaw_deg - 90.0f);

    float hagl = 0.0f;
    float amsl_m = 0.0f;
    if (!_robust_hagl_and_amsl(backend, loc, hagl, amsl_m)) {
        return false;
    }

    const float speed = MAX(_nav_speed_mps(backend), 1.0f);
    float sink = _sink_best_now(backend);
    if (!isfinite(sink) || sink < 0.2f || sink > 5.0f) {
        sink = 0.7f;
    }

    Vector3f vel;
    float raw_climb = backend.climb_rate_mps();
    if (backend.velocity_ned(vel)) {
        raw_climb = -vel.z;
    }
    const float climb_credit_min = _te_climb_credit_min_mps(backend);
    const float climb_credit_max = _te_climb_credit_max_mps(backend);
    float vario = 0.0f;
    if (isfinite(raw_climb)) {
        if (raw_climb > climb_credit_min) {
            vario = MIN(raw_climb, climb_credit_max);
        } else if (raw_climb < 0.0f) {
            vario = raw_climb;
        }
    }

    const float buffer = _terrain_buffer_m(backend);
    const float forward_fracs[] = {0.0f, 0.20f, 0.40f, 0.60f, 0.80f, 1.0f};
    const float lateral_fracs[] = {0.35f, 0.70f, 1.15f};
    const float max_forward = MAX(MAX(lever_m, _te_lever_min_m(backend)), _te_degraded_target_distance(backend));
    const float lateral_base_m = MAX(buffer, speed * 1.5f);
    scan.center_sample_distance_m = max_forward;
    float center_terrain_start_m = 0.0f;
    float center_terrain_max_m = -1.0e9f;
    bool center_terrain_start_valid = false;

    for (uint8_t i = 0; i < ARRAY_SIZE(forward_fracs); i++) {
        const float forward_m = max_forward * forward_fracs[i];
        Location base = loc;
        base.offset_bearing(yaw_deg, forward_m);
        if (!_point_in_area(base)) {
            continue;
        }
        const float t = forward_m / speed;
        const float projected_alt_m = amsl_m + vario * t - sink * t;
        float terr = 0.0f;
        if (backend.terrain_height_amsl(base, terr) && isfinite(terr)) {
            if (!center_terrain_start_valid) {
                center_terrain_start_m = terr;
                center_terrain_start_valid = true;
            }
            center_terrain_max_m = MAX(center_terrain_max_m, terr);
            const float center_agl = projected_alt_m - terr;
            if (center_agl < buffer && scan.center_first_threat_time_s < 0.0f) {
                scan.center_first_threat_distance_m = forward_m;
                scan.center_first_threat_time_s = t;
            }
            if (center_agl < scan.center_min_agl_m) {
                scan.center_min_agl_m = center_agl;
                scan.center_worst_frac = forward_fracs[i];
            }
            scan.center_valid = true;
        }

        for (uint8_t j = 0; j < ARRAY_SIZE(lateral_fracs); j++) {
            const float lateral_m = lateral_base_m * lateral_fracs[j];
            Location rp = base;
            rp.offset_bearing(right_brg, lateral_m);
            if (_point_in_area(rp)) {
                float rt = 0.0f;
                if (backend.terrain_height_amsl(rp, rt) && isfinite(rt)) {
                    scan.right_min_agl_m = MIN(scan.right_min_agl_m, projected_alt_m - rt);
                    scan.right_valid = true;
                }
            }

            Location lp = base;
            lp.offset_bearing(left_brg, lateral_m);
            if (_point_in_area(lp)) {
                float lt = 0.0f;
                if (backend.terrain_height_amsl(lp, lt) && isfinite(lt)) {
                    scan.left_min_agl_m = MIN(scan.left_min_agl_m, projected_alt_m - lt);
                    scan.left_valid = true;
                }
            }
        }
    }

    scan.valid = scan.center_valid || scan.left_valid || scan.right_valid;
    if (!scan.valid) {
        return false;
    }

    if (!scan.center_valid) {
        scan.center_min_agl_m = hagl;
    }
    if (!scan.left_valid) {
        scan.left_min_agl_m = scan.center_min_agl_m;
    }
    if (!scan.right_valid) {
        scan.right_min_agl_m = scan.center_min_agl_m;
    }
    scan.corridor_min_agl_m = MIN(scan.center_min_agl_m, MIN(scan.left_min_agl_m, scan.right_min_agl_m));
    if (center_terrain_start_valid && center_terrain_max_m > -1.0e8f) {
        scan.center_terrain_delta_m = center_terrain_max_m - center_terrain_start_m;
    }

    const float trigger = buffer + _te_side_wall_trigger_margin_m(buffer);
    const float severe = buffer - (_te_entry_hysteresis_m(buffer) * 2.0f);
    const float side_bias_m = MAX(_te_side_wall_clear_margin_m(buffer) * 0.5f,
                                  MAX(_te_flat_delta_m(buffer) * 2.0f, _te_entry_hysteresis_m(buffer) * 3.0f));
    const float balanced_bias_m = MAX(_te_flat_delta_m(buffer), _te_entry_hysteresis_m(buffer) * 2.0f);

    const bool right_low = scan.right_valid && scan.right_min_agl_m < trigger;
    const bool left_low = scan.left_valid && scan.left_min_agl_m < trigger;
    const bool right_unsafe = scan.right_valid && scan.right_min_agl_m < severe;
    const bool left_unsafe = scan.left_valid && scan.left_min_agl_m < severe;
    const bool right_asymmetric = scan.right_valid &&
        ((scan.left_valid && scan.left_min_agl_m > scan.right_min_agl_m + side_bias_m) ||
         (scan.center_valid && scan.center_min_agl_m > scan.right_min_agl_m + side_bias_m));
    const bool left_asymmetric = scan.left_valid &&
        ((scan.right_valid && scan.right_min_agl_m > scan.left_min_agl_m + side_bias_m) ||
         (scan.center_valid && scan.center_min_agl_m > scan.left_min_agl_m + side_bias_m));
    scan.lateral_delta_m = fabsf(scan.left_min_agl_m - scan.right_min_agl_m);
    scan.center_low_delta_m = scan.center_min_agl_m - scan.corridor_min_agl_m;
    scan.weak_asymmetry = (right_low || left_low) && !right_asymmetric && !left_asymmetric;
    scan.uniform_low = (right_low || left_low || scan.center_min_agl_m < trigger) &&
                       scan.lateral_delta_m <= balanced_bias_m &&
                       scan.center_low_delta_m <= balanced_bias_m;
    const float center_deficit_m = buffer - scan.center_min_agl_m;
    const bool center_hard = scan.center_valid && scan.center_min_agl_m < TERRAIN_HARD_UNSAFE_AGL_M;
    const bool center_threat_late = scan.center_worst_frac >= TERRAIN_REENTRY_NEAR_FRAC;
    const bool center_margin_floor_ok = scan.center_min_agl_m >= buffer * TERRAIN_MARGIN_LOW_MIN_FRACTION;
    const bool center_margin_only = scan.center_valid &&
                                    scan.center_min_agl_m >= TERRAIN_HARD_UNSAFE_AGL_M &&
                                    center_deficit_m > 0.0f &&
                                    center_margin_floor_ok &&
                                    (center_deficit_m <= _te_margin_low_deficit_m(buffer) || center_threat_late) &&
                                    scan.center_low_delta_m <= balanced_bias_m &&
                                    scan.lateral_delta_m <= _te_side_wall_clear_margin_m(buffer);
    const bool center_requires_escape = scan.center_valid &&
                                        !center_margin_only &&
                                        (center_hard ||
                                         (scan.center_worst_frac < TERRAIN_REENTRY_NEAR_FRAC &&
                                          scan.center_min_agl_m < buffer - _te_margin_low_deficit_m(buffer)));
    const bool right_relevant = center_requires_escape && scan.right_min_agl_m < buffer - _te_entry_hysteresis_m(buffer);
    const bool left_relevant = center_requires_escape && scan.left_min_agl_m < buffer - _te_entry_hysteresis_m(buffer);
    const bool right_wall = right_low && right_asymmetric && right_relevant &&
                            (right_unsafe || scan.left_min_agl_m >= buffer);
    const bool left_wall = left_low && left_asymmetric && left_relevant &&
                           (left_unsafe || scan.right_min_agl_m >= buffer);

    const auto escape_turn_for = [&](float wall_agl, float escape_agl) {
        const bool escape_clear = escape_agl >= buffer + _te_exit_margin_m(buffer);
        const bool wall_hard = wall_agl < TERRAIN_HARD_UNSAFE_AGL_M || scan.center_min_agl_m < TERRAIN_HARD_UNSAFE_AGL_M;
        const bool wall_urgent = wall_agl < severe || scan.center_min_agl_m < buffer;
        if (!escape_clear && escape_agl < buffer) {
            return 45.0f;
        }
        if (wall_hard && escape_clear) {
            return 105.0f;
        }
        if (wall_urgent && escape_clear) {
            return 90.0f;
        }
        if (wall_urgent) {
            return 75.0f;
        }
        return 60.0f;
    };

    if (right_wall && (!left_wall || scan.left_min_agl_m > scan.right_min_agl_m + balanced_bias_m)) {
        scan.wall_side = 1;
        scan.escape_side = -1;
        const float turn = escape_turn_for(scan.right_min_agl_m, scan.left_min_agl_m);
        scan.escape_bearing_deg = _wrap_360(yaw_deg - turn);
        return true;
    }
    if (left_wall && (!right_wall || scan.right_min_agl_m > scan.left_min_agl_m + balanced_bias_m)) {
        scan.wall_side = -1;
        scan.escape_side = 1;
        const float turn = escape_turn_for(scan.left_min_agl_m, scan.right_min_agl_m);
        scan.escape_bearing_deg = _wrap_360(yaw_deg + turn);
        return true;
    }
    if (left_wall && right_wall) {
        if (scan.left_min_agl_m > scan.right_min_agl_m + balanced_bias_m) {
            scan.wall_side = 1;
            scan.escape_side = -1;
            const float turn = escape_turn_for(scan.right_min_agl_m, scan.left_min_agl_m);
            scan.escape_bearing_deg = _wrap_360(yaw_deg - turn);
            return true;
        }
        if (scan.right_min_agl_m > scan.left_min_agl_m + balanced_bias_m) {
            scan.wall_side = -1;
            scan.escape_side = 1;
            const float turn = escape_turn_for(scan.left_min_agl_m, scan.right_min_agl_m);
            scan.escape_bearing_deg = _wrap_360(yaw_deg + turn);
            return true;
        }
    }

    const bool low_corridor = center_requires_escape ||
                              (scan.corridor_min_agl_m < TERRAIN_HARD_UNSAFE_AGL_M &&
                               scan.center_min_agl_m < buffer - _te_entry_hysteresis_m(buffer));
    if (low_corridor && !center_margin_only && (scan.uniform_low || scan.weak_asymmetry || center_requires_escape)) {
        int8_t escape_side = 0;
        if (_te_wall_memory_active(AP_HAL::millis()) && _terrain.escape_side != 0 &&
            fabsf(scan.left_min_agl_m - scan.right_min_agl_m) <= balanced_bias_m) {
            escape_side = _terrain.escape_side;
        } else if (scan.right_min_agl_m >= scan.left_min_agl_m) {
            escape_side = 1;
        } else {
            escape_side = -1;
        }
        scan.escape_side = escape_side;
        scan.wall_side = int8_t(-escape_side);
        const float wall_agl = escape_side > 0 ? scan.left_min_agl_m : scan.right_min_agl_m;
        const float escape_agl = escape_side > 0 ? scan.right_min_agl_m : scan.left_min_agl_m;
        const float turn = escape_turn_for(wall_agl, escape_agl);
        scan.escape_bearing_deg = _wrap_360(yaw_deg + turn * float(escape_side));
        return true;
    }

    scan.wall_side = 0;
    scan.escape_side = 0;
    scan.escape_bearing_deg = yaw_deg;
    return true;
}

bool AP_SoarNav::_te_update_side_wall_memory(Backend &backend, uint32_t now_ms, const TerrainSideScan &scan)
{
    if (!scan.valid) {
        return _te_wall_memory_active(now_ms);
    }

    const float buffer = _terrain_buffer_m(backend);
    if (_terrain.side_wall_active && _terrain.escape_side != 0) {
        const float active_escape_agl = _terrain.escape_side > 0 ? scan.right_min_agl_m : scan.left_min_agl_m;
        const float opposite_agl = _terrain.escape_side > 0 ? scan.left_min_agl_m : scan.right_min_agl_m;
        const float best_other_agl = MAX(opposite_agl, scan.center_min_agl_m);
        const bool active_escape_worse = active_escape_agl + MAX(_te_entry_hysteresis_m(buffer) * 2.0f, _te_flat_delta_m(buffer)) < best_other_agl;
        const bool active_escape_hard_bad = active_escape_agl < TERRAIN_HARD_UNSAFE_AGL_M &&
                                            best_other_agl > active_escape_agl + _te_entry_headroom_m(buffer);
        if ((active_escape_worse || active_escape_hard_bad) && !_te_hard_unsafe_active(now_ms)) {
            _terrain.side_wall_active = false;
            _terrain.wall_side = 0;
            _terrain.escape_side = 0;
            _terrain.side_wall_hold_until_ms = 0;
            _terrain.escape_started_ms = 0;
            _terrain.pending_wall_side = 0;
            _terrain.pending_wall_count = 0;
        }
    }
    if (scan.center_min_agl_m < TERRAIN_HARD_UNSAFE_AGL_M) {
        (void)_te_note_hard_unsafe(backend, now_ms, scan.center_min_agl_m, buffer, "side_center");
    } else if (scan.center_min_agl_m >= buffer + _te_entry_hysteresis_m(buffer) &&
               scan.left_min_agl_m >= TERRAIN_HARD_UNSAFE_AGL_M &&
               scan.right_min_agl_m >= TERRAIN_HARD_UNSAFE_AGL_M) {
        _te_note_hard_safe(now_ms);
    }

    const bool forward_margin_only = scan.wall_side == 0 &&
                                     scan.center_min_agl_m >= TERRAIN_HARD_UNSAFE_AGL_M &&
                                     scan.corridor_min_agl_m >= TERRAIN_HARD_UNSAFE_AGL_M &&
                                     (scan.center_min_agl_m >= buffer - _te_margin_low_deficit_m(buffer) ||
                                      (scan.center_min_agl_m >= buffer * TERRAIN_MARGIN_LOW_MIN_FRACTION &&
                                       scan.center_worst_frac >= TERRAIN_REENTRY_NEAR_FRAC)) &&
                                     !_te_hard_unsafe_active(now_ms);
    if (scan.wall_side == 0 && (scan.weak_asymmetry || scan.uniform_low || forward_margin_only)) {
        const uint8_t reject_key = scan.uniform_low ? 1U : (forward_margin_only ? 2U : 3U);
        if (_log_level.get() > 1 &&
            (_terrain.last_side_reject_log_key != reject_key ||
             now_ms - _terrain.last_side_reject_log_ms >= TERRAIN_SIDE_REJECT_LOG_MIN_MS)) {
            _terrain.last_side_reject_log_key = reject_key;
            _terrain.last_side_reject_log_ms = now_ms;
            backend.send_text(MAV_SEVERITY_INFO, scan.uniform_low ?
                              "SoarNav: TEsrej uniform L%.0f R%.0f C%.0f" :
                              (forward_margin_only ?
                               "SoarNav: TEsrej fwdok L%.0f R%.0f C%.0f" :
                               "SoarNav: TEsrej weak L%.0f R%.0f C%.0f"),
                              (double)scan.left_min_agl_m,
                              (double)scan.right_min_agl_m,
                              (double)scan.center_min_agl_m);
        }
        if (_terrain.side_wall_active &&
            (scan.uniform_low || forward_margin_only) &&
            !_te_hard_unsafe_active(now_ms) &&
            scan.corridor_min_agl_m >= TERRAIN_HARD_UNSAFE_AGL_M &&
            (scan.center_min_agl_m >= buffer - _te_margin_low_deficit_m(buffer) ||
             (scan.center_min_agl_m >= buffer * TERRAIN_MARGIN_LOW_MIN_FRACTION &&
              scan.center_worst_frac >= TERRAIN_REENTRY_NEAR_FRAC))) {
            _terrain.side_wall_active = false;
            _terrain.wall_side = 0;
            _terrain.escape_side = 0;
            _terrain.side_wall_hold_until_ms = 0;
            _terrain.escape_started_ms = 0;
            _terrain.pending_wall_side = 0;
            _terrain.pending_wall_count = 0;
            _terrain.side_wall_clear_count = TERRAIN_SIDE_WALL_CLEAR_CHECKS;
        }
    }

    if (scan.wall_side != 0) {
        const bool active_memory = _te_wall_memory_active(now_ms) && _terrain.wall_side != 0;
        const bool opposite_wall = active_memory && scan.wall_side != _terrain.wall_side;
        if (opposite_wall) {
            if (_terrain.pending_wall_side == scan.wall_side) {
                if (_terrain.pending_wall_count < 255) {
                    _terrain.pending_wall_count++;
                }
            } else {
                _terrain.pending_wall_side = scan.wall_side;
                _terrain.pending_wall_count = 1;
            }
            const bool center_hard_flip = scan.center_min_agl_m < TERRAIN_HARD_UNSAFE_AGL_M;
            const bool both_sides_low = scan.left_min_agl_m < buffer && scan.right_min_agl_m < buffer;
            const bool center_not_open = scan.center_min_agl_m < buffer - _te_margin_low_deficit_m(buffer);
            const bool bad_to_bad_center_flip = center_hard_flip && both_sides_low && center_not_open;
            const uint8_t required = bad_to_bad_center_flip ? TERRAIN_SIDE_WALL_FLIP_HARD_CONFIRM_CHECKS :
                                     (center_hard_flip ? 1U : TERRAIN_SIDE_WALL_FLIP_CONFIRM_CHECKS);
            const bool switch_locked = (_terrain.escape_started_ms != 0 &&
                                        now_ms - _terrain.escape_started_ms < _te_side_wall_min_switch_ms()) ||
                                       (_terrain.last_wall_flip_ms != 0 &&
                                        now_ms - _terrain.last_wall_flip_ms < _te_side_wall_min_switch_ms());
            const float old_escape_agl = _terrain.escape_side > 0 ? scan.right_min_agl_m : scan.left_min_agl_m;
            const float new_escape_agl = scan.escape_side > 0 ? scan.right_min_agl_m : scan.left_min_agl_m;
            const float new_wall_agl = scan.wall_side > 0 ? scan.right_min_agl_m : scan.left_min_agl_m;
            const float flip_gain = new_escape_agl - old_escape_agl;
            const bool hard_flip = scan.center_min_agl_m < TERRAIN_HARD_UNSAFE_AGL_M ||
                                   new_wall_agl < TERRAIN_HARD_UNSAFE_AGL_M ||
                                   _te_hard_unsafe_active(now_ms);
            const bool both_escape_low = old_escape_agl < buffer && new_escape_agl < buffer;
            const float scheduler_s = float(TERRAIN_CHECK_INTERVAL_MS) * 0.001f;
            const float lock_turn_time_s = _te_turn_time_s(backend, TERRAIN_SIDE_WALL_HEADING_LOCK_DEG);
            const bool center_threat_is_late = scan.center_first_threat_time_s >= 0.0f &&
                                               scan.center_first_threat_time_s <= lock_turn_time_s + scheduler_s;
            const bool new_escape_clears_hard_floor = new_escape_agl >= TERRAIN_HARD_UNSAFE_AGL_M + _te_entry_headroom_m(buffer);
            const bool old_and_new_hard_bad = old_escape_agl < TERRAIN_HARD_UNSAFE_AGL_M &&
                                              new_escape_agl < TERRAIN_HARD_UNSAFE_AGL_M;
            const bool late_bad_to_bad_flip = both_escape_low && both_sides_low && center_not_open &&
                                              (center_threat_is_late || old_and_new_hard_bad);
            const float low_flip_scale = late_bad_to_bad_flip ? 2.50f : (both_escape_low ? 1.50f : 1.0f);
            const float required_flip_gain = _te_side_wall_flip_gain_m(buffer) * low_flip_scale;
            const bool weak_flip = flip_gain < required_flip_gain && !hard_flip;
            const bool unsafe_side_flip = both_escape_low &&
                                          (both_sides_low || center_not_open || hard_flip) &&
                                          (flip_gain < required_flip_gain ||
                                           (late_bad_to_bad_flip && !new_escape_clears_hard_floor));
            const bool active_escape_low = old_escape_agl < buffer - _te_margin_low_deficit_m(buffer);
            const bool forced_escape_flip = center_threat_is_late &&
                                             active_escape_low &&
                                             new_escape_clears_hard_floor &&
                                             flip_gain >= MAX(_te_entry_headroom_m(buffer),
                                                              _te_side_wall_flip_gain_m(buffer) * 0.30f);
            if ((_terrain.pending_wall_count < required && !forced_escape_flip) ||
                ((switch_locked && !hard_flip) && !forced_escape_flip) ||
                (weak_flip && !forced_escape_flip) ||
                (unsafe_side_flip && !forced_escape_flip)) {
                _terrain.side_wall_active = true;
                _terrain.side_wall_hold_until_ms = MAX(_terrain.side_wall_hold_until_ms, now_ms + _te_hold_min_ms());
                _terrain.hold_until_ms = MAX(_terrain.hold_until_ms, now_ms + _te_hold_min_ms());
                _te_block_reroute(now_ms, _te_post_reroute_block_ms());
                if (_log_level.get() > 1 && (_terrain.last_debug_key != 73 || now_ms - _terrain.last_debug_ms >= 3000U)) {
                    _terrain.last_debug_key = 73;
                    _terrain.last_debug_ms = now_ms;
                    backend.send_text(MAV_SEVERITY_INFO, "SoarNav: TEwf hold %c>%c n%u g%.0f%s%s",
                                      _te_side_char(_terrain.wall_side),
                                      _te_side_char(scan.wall_side),
                                      unsigned(_terrain.pending_wall_count),
                                      (double)flip_gain,
                                      switch_locked ? " lock" : "",
                                      weak_flip ? " weak" : (unsafe_side_flip ? " unsafe" : ""));
                }
                return true;
            }
        } else {
            _terrain.pending_wall_side = 0;
            _terrain.pending_wall_count = 0;
        }

        const bool same_wall = active_memory && scan.wall_side == _terrain.wall_side;
        const bool accepting_flip = active_memory && scan.wall_side != _terrain.wall_side;
        const float previous_escape_heading = _terrain.escape_heading_deg;
        const bool previous_escape_valid = _terrain.escape_started_ms != 0;

        if (accepting_flip) {
            _terrain.last_wall_flip_ms = now_ms;
            if (_log_level.get() > 1) {
                backend.send_text(MAV_SEVERITY_INFO, "SoarNav: TEwf accept %c>%c",
                                  _te_side_char(_terrain.wall_side),
                                  _te_side_char(scan.wall_side));
            }
        }
        _terrain.side_wall_active = true;
        _terrain.wall_side = scan.wall_side;
        _terrain.escape_side = scan.escape_side;
        if (!same_wall || !previous_escape_valid) {
            _terrain.escape_heading_deg = scan.escape_bearing_deg;
            _terrain.escape_started_ms = now_ms;
        } else {
            const float yaw_deg = _te_track_or_yaw_deg(backend);
            const float requested_delta = _wrap_180(scan.escape_bearing_deg - previous_escape_heading);
            const float update_limit = TERRAIN_SIDE_WALL_HEADING_LOCK_DEG * TERRAIN_SIDE_WALL_ESCAPE_UPDATE_FRACTION;
            const bool previous_turns_into_wall = _te_heading_turns_towards_wall(previous_escape_heading, yaw_deg, now_ms);
            if (previous_turns_into_wall || fabsf(requested_delta) <= update_limit) {
                _terrain.escape_heading_deg = scan.escape_bearing_deg;
            } else {
                _terrain.escape_heading_deg = previous_escape_heading;
                if (_log_level.get() > 1 && (_terrain.last_debug_key != 74 || now_ms - _terrain.last_debug_ms >= 3000U)) {
                    _terrain.last_debug_key = 74;
                    _terrain.last_debug_ms = now_ms;
                    backend.send_text(MAV_SEVERITY_INFO, "SoarNav: TEesc latch %c b%.0f r%.0f",
                                      _te_side_char(_terrain.wall_side),
                                      (double)previous_escape_heading,
                                      (double)scan.escape_bearing_deg);
                }
            }
        }
        _terrain.side_wall_hold_until_ms = MAX(_terrain.side_wall_hold_until_ms, now_ms + _te_side_wall_hold_ms());
        _terrain.side_wall_clear_count = 0;
        _terrain.pending_wall_side = 0;
        _terrain.pending_wall_count = 0;
        _terrain.hold_until_ms = MAX(_terrain.hold_until_ms, now_ms + _te_hold_min_ms());
        _te_block_reroute(now_ms, _te_post_reroute_block_ms());
        if (_log_level.get() > 1 && (_terrain.last_debug_key != 70 || now_ms - _terrain.last_debug_ms >= 3000U)) {
            _terrain.last_debug_key = 70;
            _terrain.last_debug_ms = now_ms;
            backend.send_text(MAV_SEVERITY_INFO, "SoarNav: TEside %c>%c a%.0f b%.0f",
                              _te_side_char(scan.wall_side),
                              _te_side_char(scan.escape_side),
                              (double)(scan.wall_side > 0 ? scan.right_min_agl_m : scan.left_min_agl_m),
                              (double)buffer);
            backend.send_text(MAV_SEVERITY_INFO, "SoarNav: TEs L%.0f R%.0f C%.0f M%.0f",
                              (double)scan.left_min_agl_m,
                              (double)scan.right_min_agl_m,
                              (double)scan.center_min_agl_m,
                              (double)scan.corridor_min_agl_m);
        }
        return true;
    }

    if (_terrain.side_wall_active) {
        const bool full_lateral_clear = scan.left_min_agl_m >= buffer + _te_side_wall_clear_margin_m(buffer) &&
                                        scan.right_min_agl_m >= buffer + _te_side_wall_clear_margin_m(buffer) &&
                                        scan.center_min_agl_m >= buffer + _te_exit_margin_m(buffer);
        const bool center_corridor_clear = scan.center_min_agl_m >= buffer + _te_exit_margin_m(buffer) &&
                                           scan.corridor_min_agl_m >= TERRAIN_HARD_UNSAFE_AGL_M &&
                                           !_te_hard_unsafe_active(now_ms);
        if (full_lateral_clear || center_corridor_clear) {
            if (_terrain.side_wall_clear_count < 255) {
                _terrain.side_wall_clear_count++;
            }
        } else {
            _terrain.side_wall_clear_count = 0;
        }
        if (_terrain.side_wall_clear_count >= TERRAIN_SIDE_WALL_CLEAR_CHECKS && now_ms >= _terrain.side_wall_hold_until_ms) {
            _terrain.side_wall_active = false;
            _terrain.wall_side = 0;
            _terrain.escape_side = 0;
            _terrain.pending_wall_side = 0;
            _terrain.pending_wall_count = 0;
        }
    }

    return _te_wall_memory_active(now_ms);
}

bool AP_SoarNav::_path_min_agl_probe(Backend &backend, const Location &start, const Location &target, float &min_agl, float &worst_frac, TerrainProbe *probe) const
{
    const float dist = start.get_distance(target);
    float hagl = 0.0f;
    float amsl_m = 0.0f;
    if (!_robust_hagl_and_amsl(backend, start, hagl, amsl_m)) {
        return false;
    }

    TerrainProbe local_probe{};
    local_probe.min_agl_m = hagl;
    local_probe.worst_frac = 0.0f;
    local_probe.target_distance_m = MAX(dist, 0.0f);
    local_probe.sample_distance_m = 0.0f;
    local_probe.bearing_deg = 0.0f;
    local_probe.center_min_agl_m = hagl;
    local_probe.left_min_agl_m = 1.0e9f;
    local_probe.right_min_agl_m = 1.0e9f;
    local_probe.corridor_min_agl_m = hagl;
    local_probe.corridor_score = 0.0f;
    local_probe.speed_mps = _nav_speed_mps(backend);
    local_probe.sink_mps = 0.0f;
    local_probe.vario_mps = 0.0f;
    local_probe.raw_climb_mps = 0.0f;
    local_probe.climb_credit_mps = 0.0f;
    local_probe.start_hagl_m = hagl;
    local_probe.terrain_start_m = -9999.0f;
    local_probe.terrain_worst_m = -9999.0f;
    local_probe.terrain_delta_m = 0.0f;
    local_probe.first_threat_distance_m = -1.0f;
    local_probe.first_threat_time_s = -1.0f;

    float terrain_start = 0.0f;
    if (backend.terrain_height_amsl(start, terrain_start) && isfinite(terrain_start)) {
        local_probe.terrain_start_m = terrain_start;
        local_probe.terrain_worst_m = terrain_start;
    }

    const float min_sample_distance_m = MAX(_te_candidate_min_m(backend) * 0.10f, _terrain_buffer_m(backend) * 0.10f);
    if (dist < min_sample_distance_m) {
        min_agl = hagl;
        worst_frac = 0.0f;
        local_probe.left_min_agl_m = hagl;
        local_probe.right_min_agl_m = hagl;
        local_probe.corridor_min_agl_m = hagl;
        if (probe != nullptr) {
            *probe = local_probe;
        }
        return true;
    }

    const float speed = MAX(local_probe.speed_mps, 1.0f);
    float sink = _sink_best_now(backend);
    if (!isfinite(sink) || sink < 0.2f || sink > 5.0f) {
        sink = 0.7f;
    }
    local_probe.sink_mps = sink;

    Vector3f vel;
    float raw_climb = 0.0f;
    float vario = 0.0f;
    if (backend.velocity_ned(vel)) {
        raw_climb = -vel.z;
    } else {
        raw_climb = backend.climb_rate_mps();
    }
    const float climb_credit_min = _te_climb_credit_min_mps(backend);
    const float climb_credit_max = _te_climb_credit_max_mps(backend);
    if (isfinite(raw_climb)) {
        if (raw_climb > climb_credit_min) {
            vario = MIN(raw_climb, climb_credit_max);
        } else if (raw_climb < 0.0f) {
            vario = raw_climb;
        }
    }
    local_probe.raw_climb_mps = raw_climb;
    local_probe.climb_credit_mps = vario > 0.0f ? vario : 0.0f;
    local_probe.vario_mps = vario;

    min_agl = 1.0e9f;
    worst_frac = 0.0f;
    const float brg = _bearing_deg(start, target);
    const float sample_dist = _te_sample_horizon_m(backend, dist);
    local_probe.bearing_deg = brg;
    local_probe.sample_distance_m = sample_dist;
    const float buffer = _terrain_buffer_m(backend);

    if (sample_dist < min_sample_distance_m) {
        min_agl = hagl;
        local_probe.left_min_agl_m = hagl;
        local_probe.right_min_agl_m = hagl;
        local_probe.corridor_min_agl_m = hagl;
        if (probe != nullptr) {
            local_probe.min_agl_m = min_agl;
            *probe = local_probe;
        }
        return true;
    }

    const float sample_spacing_m = constrain_float(_nav_speed_mps(backend) * 0.75f,
                                                   buffer * 0.20f,
                                                   buffer);
    const uint16_t sample_count = uint16_t(constrain_int32(int32_t(ceilf(sample_dist / sample_spacing_m)), 4, 32));
    for (uint16_t i = 1; i <= sample_count; i++) {
        const float f = float(i) / float(sample_count);
        const float along_m = sample_dist * f;
        Location p = start;
        p.offset_bearing(brg, along_m);
        if (!_point_in_area(p)) {
            min_agl = -1.0e9f;
            worst_frac = f;
            local_probe.min_agl_m = min_agl;
            local_probe.center_min_agl_m = min_agl;
            local_probe.corridor_min_agl_m = min_agl;
            local_probe.worst_frac = f;
            if (probe != nullptr) {
                *probe = local_probe;
            }
            return false;
        }
        float terr = 0.0f;
        if (!backend.terrain_height_amsl(p, terr) || !isfinite(terr)) {
            if (probe != nullptr) {
                local_probe.min_agl_m = min_agl;
                local_probe.worst_frac = f;
                *probe = local_probe;
            }
            return false;
        }
        const float t = along_m / speed;
        const float projected_alt_m = amsl_m + vario * t - sink * t;
        const float agl = projected_alt_m - terr;
        if (agl < buffer && local_probe.first_threat_distance_m < 0.0f) {
            local_probe.first_threat_distance_m = along_m;
            local_probe.first_threat_time_s = t;
        }
        local_probe.center_min_agl_m = MIN(local_probe.center_min_agl_m, agl);
        if (agl < min_agl) {
            min_agl = agl;
            worst_frac = f;
            local_probe.min_agl_m = agl;
            local_probe.worst_frac = f;
            local_probe.terrain_worst_m = terr;
        }
    }

    const float corridor_fracs[] = {0.25f, 0.50f, 0.75f, 1.0f};
    const float lateral_near_m = buffer * 0.40f;
    const float lateral_mid_m = buffer * 0.80f;
    const float lateral_far_m = buffer * 1.25f;
    const float lateral_offsets[] = {-lateral_near_m, lateral_near_m, -lateral_mid_m, lateral_mid_m, -lateral_far_m, lateral_far_m};
    bool left_valid = false;
    bool right_valid = false;
    for (uint8_t i = 0; i < ARRAY_SIZE(corridor_fracs); i++) {
        const float f = corridor_fracs[i];
        const float along_m = sample_dist * f;
        Location base = start;
        base.offset_bearing(brg, along_m);
        if (!_point_in_area(base)) {
            continue;
        }
        const float t = along_m / speed;
        const float projected_alt_m = amsl_m + vario * t - sink * t;
        for (uint8_t j = 0; j < ARRAY_SIZE(lateral_offsets); j++) {
            const float side_m = lateral_offsets[j];
            Location p = base;
            p.offset_bearing(_wrap_360(brg + (side_m > 0.0f ? 90.0f : -90.0f)), fabsf(side_m));
            if (!_point_in_area(p)) {
                continue;
            }
            float terr = 0.0f;
            if (!backend.terrain_height_amsl(p, terr) || !isfinite(terr)) {
                if (probe != nullptr) {
                    local_probe.min_agl_m = min_agl;
                    local_probe.worst_frac = f;
                    *probe = local_probe;
                }
                return false;
            }
            const float agl = projected_alt_m - terr;
            if (agl < buffer && local_probe.first_threat_distance_m < 0.0f) {
                local_probe.first_threat_distance_m = along_m;
                local_probe.first_threat_time_s = t;
            }
            if (agl < local_probe.min_agl_m) {
                local_probe.min_agl_m = agl;
                local_probe.worst_frac = f;
                local_probe.terrain_worst_m = terr;
            }
            if (side_m < 0.0f) {
                local_probe.left_min_agl_m = MIN(local_probe.left_min_agl_m, agl);
                left_valid = true;
            } else {
                local_probe.right_min_agl_m = MIN(local_probe.right_min_agl_m, agl);
                right_valid = true;
            }
        }
    }

    if (!left_valid) {
        local_probe.left_min_agl_m = local_probe.center_min_agl_m;
    }
    if (!right_valid) {
        local_probe.right_min_agl_m = local_probe.center_min_agl_m;
    }
    local_probe.corridor_min_agl_m = MIN(local_probe.center_min_agl_m, MIN(local_probe.left_min_agl_m, local_probe.right_min_agl_m));
    if (local_probe.terrain_start_m > -9000.0f && local_probe.terrain_worst_m > -9000.0f) {
        local_probe.terrain_delta_m = local_probe.terrain_worst_m - local_probe.terrain_start_m;
    }
    const float left_deficit_m = MAX(0.0f, buffer - local_probe.left_min_agl_m);
    const float right_deficit_m = MAX(0.0f, buffer - local_probe.right_min_agl_m);
    const float wall_deficit_m = left_deficit_m + right_deficit_m;
    local_probe.corridor_score = local_probe.center_min_agl_m -
                                  wall_deficit_m * 0.12f -
                                  fabsf(local_probe.left_min_agl_m - local_probe.right_min_agl_m) * 0.03f -
                                  _te_probe_risk_penalty_m(local_probe, buffer) * 0.25f;
    min_agl = local_probe.min_agl_m;

    if (probe != nullptr) {
        *probe = local_probe;
    }
    return true;
}

bool AP_SoarNav::_terrain_candidate(Backend &backend, const Location &loc, float bearing_deg, float dist_m, Location &candidate) const
{
    (void)backend;
    candidate = loc;
    candidate.offset_bearing(bearing_deg, dist_m);
    Location tmp = candidate;
    if (!_clamp_inside_area(loc, tmp)) {
        return false;
    }
    candidate = tmp;
    return true;
}

bool AP_SoarNav::_te_make_candidate(Backend &backend, const Location &loc, float bearing_deg, float dist_m, Location &candidate) const
{
    const float scales[] = {1.0f, 0.75f, 0.50f, 0.33f};
    for (uint8_t i = 0; i < ARRAY_SIZE(scales); i++) {
        Location p;
        if (!_terrain_candidate(backend, loc, bearing_deg, MAX(_te_candidate_min_m(backend), dist_m * scales[i]), p)) {
            continue;
        }
        if (!_point_in_area(p)) {
            continue;
        }
        if (_using_polygon && !_segment_stays_inside(loc, p)) {
            Location adjusted = _adjust_target_segment(loc, p);
            if (!_point_in_area(adjusted) || !_segment_stays_inside(loc, adjusted)) {
                continue;
            }
            p = adjusted;
        }
        candidate = p;
        return true;
    }
    return false;
}

float AP_SoarNav::_te_guided_target_distance(Backend &backend) const
{
    return _te_time_scaled_distance_m(backend, _te_guided_target_time_s(), _te_lever_min_time_s(), 1.0f);
}

float AP_SoarNav::_te_degraded_target_distance(Backend &backend) const
{
    return _te_time_scaled_distance_m(backend, _te_degraded_advance_time_s(), _te_lever_min_time_s(), 1.25f);
}

bool AP_SoarNav::_te_select_forward_corridor(Backend &backend, const Location &loc, float min_distance_m, Location &candidate, float &agl, float &dist, float &bearing_deg) const
{
    const uint32_t now = AP_HAL::millis();
    const float yaw_deg = _te_track_or_yaw_deg(backend);
    const float buffer = _terrain_buffer_m(backend);
    const float min_distance = MAX(min_distance_m, _te_candidate_min_m(backend));
    const float long_distance = MAX(min_distance, _te_target_sample_limit_m(backend));
    const float max_turn = MIN(TERRAIN_SIDE_WALL_HEADING_LOCK_DEG, TERRAIN_POST_EVASION_MAX_REVERSAL_DEG);
    const float step = MAX(TERRAIN_WALL_TURN_GUARD_DEG, max_turn * 0.25f);
    const float distance_scale[] = {1.0f, TERRAIN_SIDE_WALL_ESCAPE_UPDATE_FRACTION, TERRAIN_REENTRY_NEAR_FRAC, 1.25f};
    float base_bearings[3];
    uint8_t base_count = 0;
    const auto add_base = [&](float brg) {
        if (base_count >= ARRAY_SIZE(base_bearings)) {
            return;
        }
        const float wrapped = _wrap_360(brg);
        if (fabsf(_wrap_180(wrapped - yaw_deg)) > max_turn) {
            return;
        }
        for (uint8_t i = 0; i < base_count; i++) {
            if (fabsf(_wrap_180(base_bearings[i] - wrapped)) < TERRAIN_WALL_TURN_GUARD_DEG) {
                return;
            }
        }
        base_bearings[base_count++] = wrapped;
    };
    add_base(yaw_deg);
    if (_terrain.last_safe_heading_valid) {
        add_base(_terrain.last_safe_heading_deg);
    }
    if (_terrain.committed_turn_side != 0 && isfinite(_terrain.committed_bearing_deg)) {
        add_base(_terrain.committed_bearing_deg);
    }

    bool have_best = false;
    Location best;
    float best_agl = -1.0e9f;
    float best_dist = 0.0f;
    float best_bearing = yaw_deg;
    float best_score = -1.0e9f;

    for (uint8_t b = 0; b < base_count; b++) {
        const float base_bearing = base_bearings[b];
        for (int8_t k = 0; k <= 4; k++) {
            for (int8_t s = -1; s <= 1; s += 2) {
                if (k == 0 && s > -1) {
                    continue;
                }
                const float brg = _wrap_360(base_bearing + float(s * k) * step);
                const float turn = fabsf(_wrap_180(brg - yaw_deg));
                if (turn > max_turn) {
                    continue;
                }
                const bool turns_towards_wall = _te_heading_turns_towards_wall(brg, yaw_deg, now);
                for (uint8_t j = 0; j < ARRAY_SIZE(distance_scale); j++) {
                    Location test;
                    const float requested_dist = MAX(_te_candidate_min_m(backend), long_distance * distance_scale[j]);
                    if (!_te_make_candidate(backend, loc, brg, requested_dist, test)) {
                        continue;
                    }
                    float min_agl = 0.0f;
                    float worst = 0.0f;
                    TerrainProbe probe{};
                    if (!_path_min_agl_probe(backend, loc, test, min_agl, worst, &probe)) {
                        continue;
                    }
                    if (probe.target_distance_m > probe.sample_distance_m + _te_entry_hysteresis_m(buffer)) {
                        continue;
                    }
                    const float center_agl = probe.center_min_agl_m;
                    if (center_agl < TERRAIN_HARD_UNSAFE_AGL_M) {
                        continue;
                    }
                    const bool clear = center_agl >= buffer;
                    const bool margin = _te_probe_margin_low_ok(probe, buffer) && !_te_probe_immediate_threat(probe, buffer);
                    const bool buys_time = probe.first_threat_time_s < 0.0f ||
                                           probe.first_threat_time_s > _te_turn_time_s(backend, turn) + MAX(float(_terrain_lookahead_s.get()), 1.0f);
                    if (!clear && !margin && !(buys_time && center_agl >= buffer * TERRAIN_MARGIN_LOW_MIN_FRACTION)) {
                        continue;
                    }
                    const bool strong_wall_override = clear ||
                                                      (buys_time && center_agl >= buffer - _te_margin_low_deficit_m(buffer));
                    if (turns_towards_wall && !strong_wall_override) {
                        continue;
                    }
                    const float d = loc.get_distance(test);
                    const float base_dev = fabsf(_wrap_180(brg - base_bearing));
                    const float switch_dev = _terrain.last_safe_heading_valid ? fabsf(_wrap_180(brg - _terrain.last_safe_heading_deg)) : 0.0f;
                    const float continuity = _terrain.last_safe_heading_valid && switch_dev <= max_turn ? buffer * (1.0f - switch_dev / max_turn) : 0.0f;
                    const float threat_time_m = probe.first_threat_time_s < 0.0f ? _te_target_sample_limit_m(backend) : probe.first_threat_time_s * MAX(probe.speed_mps, 1.0f);
                    const float wall_memory_penalty = turns_towards_wall ? _te_entry_headroom_m(buffer) : 0.0f;
                    const float score = center_agl * 2.5f + probe.corridor_score * 0.70f + d * 0.10f + threat_time_m * 0.08f + continuity * 0.45f -
                                        turn * _te_candidate_min_m(backend) / MAX(max_turn, 1.0f) - base_dev * 0.45f - wall_memory_penalty -
                                        _te_probe_risk_penalty_m(probe, buffer) * 0.65f;
                    if (!have_best || score > best_score) {
                        best = test;
                        best_agl = center_agl;
                        best_dist = d;
                        best_bearing = brg;
                        best_score = score;
                        have_best = true;
                    }
                }
            }
        }
    }

    if (!have_best) {
        return false;
    }
    candidate = best;
    agl = best_agl;
    dist = best_dist;
    bearing_deg = best_bearing;
    return true;
}

bool AP_SoarNav::_te_select_corridor_best_effort(Backend &backend, const Location &loc, float reference_bearing_deg, float min_distance_m, bool allow_large_reversal, Location &candidate, float &agl, float &dist, float &bearing_deg) const
{
    const uint32_t now = AP_HAL::millis();
    const float yaw_deg = _te_track_or_yaw_deg(backend);
    const float reference = _wrap_360(reference_bearing_deg);
    const float buffer_m = _terrain_buffer_m(backend);
    const float min_distance = MAX(min_distance_m, _te_candidate_min_m(backend));
    const float distances[] = {min_distance, min_distance * 0.75f, min_distance * 0.50f};
    const float offsets[] = {0.0f, 15.0f, -15.0f, 30.0f, -30.0f, 45.0f, -45.0f, 60.0f, -60.0f, 75.0f, -75.0f, 90.0f, -90.0f, 105.0f, -105.0f, 120.0f, -120.0f};
    const bool hard_now = _te_hard_unsafe_active(now);
    const bool wall_active = _te_wall_memory_active(now) && _terrain.wall_side != 0;
    const bool committed_heading_valid = _terrain.last_safe_heading_valid ||
                                         _terrain.evasion_target_valid ||
                                         _terrain.escape_started_ms != 0;
    const float committed_heading = _terrain.last_safe_heading_valid ? _terrain.last_safe_heading_deg :
                                   (_terrain.escape_started_ms != 0 ? _terrain.escape_heading_deg : yaw_deg);
    bool have_best = false;
    Location best;
    float best_agl = -1.0e9f;
    float best_dist = 0.0f;
    float best_bearing = reference;
    float best_score = -1.0e9f;

    for (uint8_t i = 0; i < ARRAY_SIZE(offsets); i++) {
        const float brg = _wrap_360(reference + offsets[i]);
        if (_te_heading_turns_towards_wall(brg, yaw_deg, now)) {
            continue;
        }
        const float turn = fabsf(_wrap_180(brg - yaw_deg));
        if (turn > 120.0f && !allow_large_reversal && !hard_now) {
            continue;
        }
        if (turn > TERRAIN_EMERGENCY_REVERSAL_DEG && !hard_now) {
            continue;
        }
        if (wall_active && _terrain.escape_started_ms != 0 &&
            now - _terrain.escape_started_ms < _te_side_wall_min_switch_ms() &&
            fabsf(_wrap_180(brg - _terrain.escape_heading_deg)) > TERRAIN_SIDE_WALL_HEADING_LOCK_DEG) {
            continue;
        }
        for (uint8_t j = 0; j < ARRAY_SIZE(distances); j++) {
            Location test;
            if (!_te_make_candidate(backend, loc, brg, distances[j], test)) {
                continue;
            }
            float min_agl = 0.0f;
            float worst = 0.0f;
            TerrainProbe probe{};
            if (!_path_min_agl_probe(backend, loc, test, min_agl, worst, &probe)) {
                continue;
            }
            if (probe.target_distance_m > probe.sample_distance_m + _te_entry_hysteresis_m(buffer_m)) {
                continue;
            }
            const bool large_committed_reversal = committed_heading_valid &&
                                                 fabsf(_wrap_180(brg - committed_heading)) > TERRAIN_POST_EVASION_MAX_REVERSAL_DEG;
            if (large_committed_reversal &&
                (wall_active || _terrain.evasion_target_valid) &&
                min_agl < buffer_m + _te_dynamic_margin_m(buffer_m, 0.25f)) {
                continue;
            }

            const float d = loc.get_distance(test);
            const float reference_dev = fabsf(_wrap_180(brg - reference));
            const float switch_dev = _terrain.last_safe_heading_valid ? fabsf(_wrap_180(brg - _terrain.last_safe_heading_deg)) : 0.0f;
            const float turn_signed = _wrap_180(brg - yaw_deg);
            const bool escape_side = _terrain.escape_side == 0 || turn_signed * float(_terrain.escape_side) > 0.0f;
            const float side_bonus = wall_active && escape_side ? 80.0f : 0.0f;
            const float continuity_bonus = _terrain.last_safe_heading_valid && switch_dev <= 45.0f ? 30.0f - switch_dev * 0.4f : 0.0f;
            const float wide_side_bonus = wall_active && escape_side ? constrain_float(turn - 35.0f, 0.0f, 55.0f) * 1.2f : 0.0f;
            const float reversal_penalty = turn > 90.0f ? (turn - 90.0f) * (allow_large_reversal || hard_now ? 2.0f : 7.0f) : 0.0f;
            const float below_ground_penalty = min_agl < TERRAIN_HARD_UNSAFE_AGL_M ? fabsf(min_agl) * 1.5f : 0.0f;
            const float turn_clearance = _te_turn_clearance_m(buffer_m, turn);
            const float turn_margin_penalty = min_agl < buffer_m + turn_clearance ? (buffer_m + turn_clearance - min_agl) * 0.45f : 0.0f;
            const float turn_time_penalty = _te_turn_feasibility_penalty_m(backend, probe, turn) * 1.60f;
            const float score = min_agl * 2.0f + probe.corridor_score * 0.45f + d * 0.04f + side_bonus + wide_side_bonus + continuity_bonus - turn * (wall_active ? 1.1f : 1.8f) - reference_dev * 0.35f - switch_dev * 0.20f - reversal_penalty - below_ground_penalty - turn_margin_penalty - turn_time_penalty - _te_probe_risk_penalty_m(probe, buffer_m) * 0.80f;
            if (!have_best || score > best_score) {
                best = test;
                best_agl = min_agl;
                best_dist = d;
                best_bearing = brg;
                best_score = score;
                have_best = true;
            }
        }
    }

    if (!have_best) {
        return false;
    }
    candidate = best;
    agl = best_agl;
    dist = best_dist;
    bearing_deg = best_bearing;
    return true;
}

bool AP_SoarNav::_te_select_escape_fan_target(Backend &backend, const Location &loc, const TerrainPolicy &policy, int8_t required_side, Location &candidate, float &agl, float &dist, float &bearing_deg) const
{
    const uint32_t now = AP_HAL::millis();
    const float yaw_deg = policy.yaw_deg;
    const float buffer_m = policy.buffer_m;
    const float min_distance = MAX(policy.target_distance_m, _te_candidate_min_m(backend));
    int8_t escape_side = required_side;
    if (escape_side == 0 && policy.side_scan_valid && policy.side_scan.escape_side != 0) {
        escape_side = policy.side_scan.escape_side;
    }
    if (escape_side == 0 && _te_wall_memory_active(now) && _terrain.escape_side != 0) {
        escape_side = _terrain.escape_side;
    }

    const bool immediate_escape = policy.decision == TerrainDecision::CRITICAL_OVERRIDE ||
                                  policy.decision == TerrainDecision::EMERGENCY_BEST_OF_BAD ||
                                  policy.immediate_threat ||
                                  policy.turn_time_critical ||
                                  policy.hard_unsafe_active ||
                                  policy.current_hard_agl_m < TERRAIN_HARD_UNSAFE_AGL_M;
    const bool side_choice_ambiguous = policy.side_scan_valid &&
                                       (policy.side_scan.uniform_low ||
                                        (policy.side_scan.left_min_agl_m < buffer_m && policy.side_scan.right_min_agl_m < buffer_m) ||
                                        fabsf(policy.side_scan.left_min_agl_m - policy.side_scan.right_min_agl_m) <= _te_side_wall_flip_gain_m(buffer_m));
    const float side_offsets[] = {45.0f, 60.0f, 75.0f, 90.0f, 105.0f, 120.0f, 135.0f};
    const float distances[] = {min_distance, min_distance * 0.75f, min_distance * 1.25f, min_distance * 0.50f};
    bool have_correct = false;
    bool have_any = false;
    Location best_correct;
    Location best_any;
    float best_correct_score = -1.0e9f;
    float best_any_score = -1.0e9f;
    float best_correct_agl = -1.0e9f;
    float best_any_agl = -1.0e9f;
    float best_correct_dist = 0.0f;
    float best_any_dist = 0.0f;
    float best_correct_bearing = yaw_deg;
    float best_any_bearing = yaw_deg;

    const auto consider_side = [&](int8_t side) {
        if (side == 0) {
            return;
        }
        for (uint8_t i = 0; i < ARRAY_SIZE(side_offsets); i++) {
            const float brg = _wrap_360(yaw_deg + float(side) * side_offsets[i]);
            if (!immediate_escape && _te_heading_turns_towards_wall(brg, yaw_deg, now)) {
                continue;
            }
            const float signed_turn = _wrap_180(brg - yaw_deg);
            const float turn = fabsf(signed_turn);
            const bool correct_side = escape_side == 0 || signed_turn * float(escape_side) > 0.0f;
            for (uint8_t j = 0; j < ARRAY_SIZE(distances); j++) {
                Location test;
                if (!_te_make_candidate(backend, loc, brg, MAX(_te_candidate_min_m(backend), distances[j]), test)) {
                    continue;
                }
                float min_agl = 0.0f;
                float worst = 0.0f;
                TerrainProbe probe{};
                if (!_path_min_agl_probe(backend, loc, test, min_agl, worst, &probe)) {
                    continue;
                }
                if (probe.target_distance_m > probe.sample_distance_m + _te_entry_hysteresis_m(buffer_m)) {
                    continue;
                }

                const float center_agl = probe.center_min_agl_m;
                const float corridor_agl = probe.corridor_min_agl_m;
                const bool center_escape = center_agl >= TERRAIN_HARD_UNSAFE_AGL_M ||
                                           (isfinite(probe.start_hagl_m) &&
                                            center_agl > probe.start_hagl_m + _te_entry_headroom_m(buffer_m));
                const bool hard_now = _te_hard_unsafe_active(now) || policy.hard_unsafe_active ||
                                      policy.current_hard_agl_m < TERRAIN_HARD_UNSAFE_AGL_M;
                const bool hard_escape = isfinite(probe.start_hagl_m) &&
                                         probe.start_hagl_m < TERRAIN_HARD_UNSAFE_AGL_M &&
                                         center_agl > probe.start_hagl_m + _te_entry_headroom_m(buffer_m) &&
                                         center_agl >= MAX(TERRAIN_HARD_UNSAFE_AGL_M + _te_entry_headroom_m(buffer_m),
                                                           buffer_m * TERRAIN_CRITICAL_BUFFER_FRACTION);
                if (!center_escape && !hard_now) {
                    continue;
                }
                const bool severe_backtrack = turn >= TERRAIN_EMERGENCY_REVERSAL_DEG - TERRAIN_WALL_TURN_GUARD_DEG;
                const bool backtrack_has_clearance = center_agl >= buffer_m + _te_dynamic_margin_m(buffer_m, 0.25f);
                if (severe_backtrack && !backtrack_has_clearance && !hard_escape) {
                    continue;
                }

                const float d = loc.get_distance(test);
                const float first_threat_bonus = probe.first_threat_time_s > 0.0f ? MIN(probe.first_threat_time_s, 30.0f) * 5.0f : 0.0f;
                const float terrain_rise_penalty = MAX(0.0f, probe.terrain_delta_m) * 1.25f;
                const float center_deficit_penalty = center_agl < buffer_m ? (buffer_m - center_agl) * 0.55f : 0.0f;
                const float corridor_deficit_penalty = corridor_agl < buffer_m ? (buffer_m - corridor_agl) * 0.18f : 0.0f;
                const float wrong_side_penalty = (escape_side != 0 && !correct_side) ? MAX(buffer_m, 1.0f) * (side_choice_ambiguous ? 0.45f : 1.25f) : 0.0f;
                const float side_bonus = correct_side ? MAX(buffer_m, 1.0f) * (side_choice_ambiguous ? 0.15f : 0.35f) : 0.0f;
                const float turn_penalty = fabsf(turn - 90.0f) * (correct_side ? 0.8f : (side_choice_ambiguous ? 1.15f : 1.8f));
                const float score = center_agl * 3.4f + corridor_agl * 0.55f + first_threat_bonus +
                                    d * 0.015f + side_bonus - terrain_rise_penalty -
                                    center_deficit_penalty - corridor_deficit_penalty - wrong_side_penalty -
                                    turn_penalty - _te_turn_feasibility_penalty_m(backend, probe, turn) * 0.35f -
                                    _te_probe_risk_penalty_m(probe, buffer_m) * 0.25f;

                if (correct_side) {
                    if (!have_correct || score > best_correct_score) {
                        best_correct = test;
                        best_correct_score = score;
                        best_correct_agl = center_agl;
                        best_correct_dist = d;
                        best_correct_bearing = brg;
                        have_correct = true;
                    }
                } else if (!have_any || score > best_any_score) {
                    best_any = test;
                    best_any_score = score;
                    best_any_agl = center_agl;
                    best_any_dist = d;
                    best_any_bearing = brg;
                    have_any = true;
                }
            }
        }
    };

    if (escape_side != 0) {
        consider_side(escape_side);
        if (immediate_escape || side_choice_ambiguous || !have_correct) {
            consider_side(int8_t(-escape_side));
        }
    } else {
        consider_side(1);
        consider_side(-1);
    }

    if (have_correct && have_any && (immediate_escape || side_choice_ambiguous)) {
        const float switch_gain = MAX(_te_entry_headroom_m(buffer_m), _te_dynamic_margin_m(buffer_m, 0.25f)) *
                                  (side_choice_ambiguous ? 0.50f : 1.0f);
        const bool correct_weak = side_choice_ambiguous || best_correct_agl < buffer_m * TERRAIN_CRITICAL_BUFFER_FRACTION;
        const bool opposite_much_better = best_any_score > best_correct_score + switch_gain ||
                                          best_any_agl > best_correct_agl + switch_gain;
        if (correct_weak && opposite_much_better) {
            candidate = best_any;
            agl = best_any_agl;
            dist = best_any_dist;
            bearing_deg = best_any_bearing;
            return true;
        }
    }

    if (have_correct) {
        candidate = best_correct;
        agl = best_correct_agl;
        dist = best_correct_dist;
        bearing_deg = best_correct_bearing;
        return true;
    }
    if (have_any && (escape_side == 0 || immediate_escape || _te_hard_unsafe_active(now))) {
        candidate = best_any;
        agl = best_any_agl;
        dist = best_any_dist;
        bearing_deg = best_any_bearing;
        return true;
    }
    return false;
}

bool AP_SoarNav::_te_select_critical_best_effort(Backend &backend, const Location &loc, const TerrainPolicy &policy, Location &candidate, float &agl, float &dist, float &bearing_deg) const
{
    const uint32_t now = AP_HAL::millis();
    const float yaw_deg = _te_track_or_yaw_deg(backend);
    const float buffer_m = _terrain_buffer_m(backend);
    const float min_distance = MAX(policy.target_distance_m, _te_candidate_min_m(backend));
    const float distances[] = {min_distance, min_distance * 0.75f, min_distance * 0.50f, min_distance * 1.25f};
    const bool memory_active = _te_wall_memory_active(now) && _terrain.wall_side != 0;
    const bool side_escape_known = (policy.side_scan_valid && policy.side_scan.escape_side != 0) ||
                                   (memory_active && _terrain.escape_side != 0);
    const int8_t escape_side = (memory_active && _terrain.escape_side != 0) ? _terrain.escape_side :
                               ((policy.side_scan_valid && policy.side_scan.escape_side != 0) ? policy.side_scan.escape_side : 0);
    const bool immediate_escape = policy.reason == TerrainReplanReason::IMMEDIATE_COLLISION ||
                                  policy.critical_override ||
                                  policy.immediate_threat ||
                                  policy.turn_time_critical ||
                                  policy.current_hard_agl_m < TERRAIN_HARD_UNSAFE_AGL_M;
    const bool side_choice_ambiguous = policy.side_scan_valid &&
                                       (policy.side_scan.uniform_low ||
                                        (policy.side_scan.left_min_agl_m < buffer_m && policy.side_scan.right_min_agl_m < buffer_m) ||
                                        fabsf(policy.side_scan.left_min_agl_m - policy.side_scan.right_min_agl_m) <= _te_side_wall_flip_gain_m(buffer_m));
    const bool front_not_usable = !policy.front_clear && !policy.front_margin;
    const bool require_lateral_escape = immediate_escape && front_not_usable;
    const float min_lateral_turn = side_escape_known ? 55.0f : 45.0f;
    const float preferred_reference = side_escape_known ? policy.side_escape_bearing_deg : yaw_deg;
    const bool committed_heading_valid = _terrain.last_safe_heading_valid ||
                                         _terrain.evasion_target_valid ||
                                         _terrain.escape_started_ms != 0;
    const float committed_heading = _terrain.last_safe_heading_valid ? _terrain.last_safe_heading_deg :
                                   (_terrain.escape_started_ms != 0 ? _terrain.escape_heading_deg : yaw_deg);

    float bearings[28];
    uint8_t count = 0;
    const auto add_bearing = [&](float brg) {
        if (count >= ARRAY_SIZE(bearings)) {
            return;
        }
        const float wrapped = _wrap_360(brg);
        for (uint8_t i = 0; i < count; i++) {
            if (fabsf(_wrap_180(bearings[i] - wrapped)) < 1.0f) {
                return;
            }
        }
        bearings[count++] = wrapped;
    };

    if (side_escape_known) {
        add_bearing(preferred_reference);
        add_bearing(preferred_reference + float(escape_side) * 15.0f);
        add_bearing(preferred_reference - float(escape_side) * 15.0f);
        add_bearing(yaw_deg + float(escape_side) * 75.0f);
        add_bearing(yaw_deg + float(escape_side) * 90.0f);
        add_bearing(yaw_deg + float(escape_side) * 60.0f);
        add_bearing(yaw_deg + float(escape_side) * 105.0f);
        add_bearing(yaw_deg + float(escape_side) * 120.0f);
        add_bearing(yaw_deg + float(escape_side) * 45.0f);
        if (side_choice_ambiguous) {
            add_bearing(yaw_deg - float(escape_side) * 75.0f);
            add_bearing(yaw_deg - float(escape_side) * 90.0f);
            add_bearing(yaw_deg - float(escape_side) * 60.0f);
            add_bearing(yaw_deg - float(escape_side) * 105.0f);
            add_bearing(yaw_deg - float(escape_side) * 120.0f);
            add_bearing(yaw_deg - float(escape_side) * 45.0f);
        }
    } else {
        add_bearing(yaw_deg + 60.0f);
        add_bearing(yaw_deg - 60.0f);
        add_bearing(yaw_deg + 75.0f);
        add_bearing(yaw_deg - 75.0f);
        add_bearing(yaw_deg + 90.0f);
        add_bearing(yaw_deg - 90.0f);
        add_bearing(yaw_deg + 45.0f);
        add_bearing(yaw_deg - 45.0f);
        add_bearing(yaw_deg + 105.0f);
        add_bearing(yaw_deg - 105.0f);
        add_bearing(yaw_deg + 120.0f);
        add_bearing(yaw_deg - 120.0f);
    }

    if (!require_lateral_escape) {
        const float base_bearing = (_terrain.evasion_target_valid && _terrain.last_safe_heading_valid) ? _terrain.last_safe_heading_deg : yaw_deg;
        const float soft_offsets[] = {0.0f, 15.0f, -15.0f, 30.0f, -30.0f};
        for (uint8_t i = 0; i < ARRAY_SIZE(soft_offsets); i++) {
            add_bearing(base_bearing + soft_offsets[i]);
        }
    } else {
        add_bearing(yaw_deg + float(escape_side == 0 ? 1 : escape_side) * 135.0f);
        add_bearing(yaw_deg - float(escape_side == 0 ? 1 : escape_side) * 135.0f);
        add_bearing(yaw_deg + 180.0f);
    }

    add_bearing(policy.target_bearing_deg + 90.0f);
    add_bearing(policy.target_bearing_deg - 90.0f);
    add_bearing(policy.target_bearing_deg + 180.0f);

    bool have_preferred = false;
    bool have_lateral = false;
    bool have_direct = false;
    bool have_reversal = false;
    Location best_preferred;
    Location best_lateral;
    Location best_direct;
    Location best_reversal;
    float preferred_agl = -1.0e9f;
    float lateral_agl = -1.0e9f;
    float direct_agl = -1.0e9f;
    float reversal_agl = -1.0e9f;
    float preferred_dist = 0.0f;
    float lateral_dist = 0.0f;
    float direct_dist = 0.0f;
    float reversal_dist = 0.0f;
    float preferred_bearing = yaw_deg;
    float lateral_bearing = yaw_deg;
    float direct_bearing = yaw_deg;
    float reversal_bearing = yaw_deg;
    float preferred_score = -1.0e9f;
    float lateral_score = -1.0e9f;
    float direct_score = -1.0e9f;
    float reversal_score = -1.0e9f;

    for (uint8_t i = 0; i < count; i++) {
        const float brg = bearings[i];
        if (!immediate_escape && _te_heading_turns_towards_wall(brg, yaw_deg, now)) {
            continue;
        }
        const float turn_signed = _wrap_180(brg - yaw_deg);
        const float turn = fabsf(turn_signed);
        const bool correct_escape_side = escape_side == 0 || turn_signed * float(escape_side) > 0.0f;
        const bool lateral = turn >= min_lateral_turn && turn <= TERRAIN_EMERGENCY_REVERSAL_DEG;
        const bool reversal = turn > TERRAIN_EMERGENCY_REVERSAL_DEG;
        if (require_lateral_escape && !lateral && !reversal) {
            continue;
        }
        for (uint8_t j = 0; j < ARRAY_SIZE(distances); j++) {
            Location test;
            if (!_te_make_candidate(backend, loc, brg, MAX(_te_candidate_min_m(backend), distances[j]), test)) {
                continue;
            }
            float min_agl = 0.0f;
            float worst = 0.0f;
            TerrainProbe probe{};
            if (!_path_min_agl_probe(backend, loc, test, min_agl, worst, &probe)) {
                continue;
            }
            if (probe.target_distance_m > probe.sample_distance_m + _te_entry_hysteresis_m(buffer_m)) {
                continue;
            }

            const float center_agl = probe.center_min_agl_m;
            const float decision_agl = (require_lateral_escape && correct_escape_side && center_agl > min_agl) ? center_agl : min_agl;
            const bool critical_scan = policy.hard_unsafe_active ||
                                       (isfinite(probe.start_hagl_m) && probe.start_hagl_m < buffer_m * TERRAIN_CRITICAL_BUFFER_FRACTION) ||
                                       _te_probe_immediate_threat(probe, buffer_m);
            const bool soft_ok = decision_agl >= buffer_m || _te_probe_margin_low_ok(probe, buffer_m);
            const bool hard_escape = isfinite(probe.start_hagl_m) &&
                                     probe.start_hagl_m < TERRAIN_HARD_UNSAFE_AGL_M &&
                                     decision_agl > probe.start_hagl_m + _te_entry_headroom_m(buffer_m) &&
                                     decision_agl >= MAX(TERRAIN_HARD_UNSAFE_AGL_M + _te_entry_headroom_m(buffer_m),
                                                         buffer_m * TERRAIN_CRITICAL_BUFFER_FRACTION);
            if (!soft_ok && !hard_escape && !critical_scan && decision_agl < TERRAIN_HARD_UNSAFE_AGL_M) {
                continue;
            }
            const bool large_committed_reversal = committed_heading_valid &&
                                                 fabsf(_wrap_180(brg - committed_heading)) > TERRAIN_POST_EVASION_MAX_REVERSAL_DEG;
            if (large_committed_reversal &&
                (side_escape_known || memory_active || _terrain.evasion_target_valid) &&
                decision_agl < buffer_m + _te_dynamic_margin_m(buffer_m, 0.25f)) {
                continue;
            }

            const float d = loc.get_distance(test);
            const float first_threat_bonus = probe.first_threat_time_s > 0.0f ? MIN(probe.first_threat_time_s, 30.0f) * 4.0f : 0.0f;
            const float trend_penalty = MAX(0.0f, probe.terrain_delta_m) * 0.55f;
            const float below_ground_penalty = decision_agl < TERRAIN_HARD_UNSAFE_AGL_M ? fabsf(decision_agl) * 0.75f : 0.0f;
            const float direct_penalty = require_lateral_escape && turn < min_lateral_turn ? (min_lateral_turn - turn) * 12.0f + buffer_m : 0.0f;
            const float lateral_bonus = lateral ? constrain_float(turn - min_lateral_turn, 0.0f, 45.0f) * 2.2f : 0.0f;
            const float side_bonus = correct_escape_side && side_escape_known ? MAX(buffer_m, 1.0f) * (side_choice_ambiguous ? 0.25f : 0.55f) : 0.0f;
            const float wrong_side_penalty = side_escape_known && !correct_escape_side ? MAX(buffer_m, 1.0f) * (side_choice_ambiguous ? 0.55f : 1.40f) : 0.0f;
            const float reversal_penalty = reversal ? 420.0f + (turn - TERRAIN_EMERGENCY_REVERSAL_DEG) * 10.0f : 0.0f;
            const float turn_time_penalty = _te_turn_feasibility_penalty_m(backend, probe, turn) * (require_lateral_escape ? 0.80f : 2.20f);
            const float risk_penalty = _te_probe_risk_penalty_m(probe, buffer_m) * 0.35f;
            const float score = decision_agl * 3.0f + probe.corridor_score * 0.45f + first_threat_bonus + d * 0.015f +
                                lateral_bonus + side_bonus - turn * (correct_escape_side ? 0.85f : 1.65f) -
                                direct_penalty - wrong_side_penalty - reversal_penalty - trend_penalty -
                                below_ground_penalty - turn_time_penalty - risk_penalty;

            if (side_escape_known && correct_escape_side && lateral) {
                if (!have_preferred || score > preferred_score) {
                    best_preferred = test;
                    preferred_agl = decision_agl;
                    preferred_dist = d;
                    preferred_bearing = brg;
                    preferred_score = score;
                    have_preferred = true;
                }
            } else if (lateral) {
                if (!have_lateral || score > lateral_score) {
                    best_lateral = test;
                    lateral_agl = decision_agl;
                    lateral_dist = d;
                    lateral_bearing = brg;
                    lateral_score = score;
                    have_lateral = true;
                }
            } else if (reversal) {
                if (!have_reversal || score > reversal_score) {
                    best_reversal = test;
                    reversal_agl = decision_agl;
                    reversal_dist = d;
                    reversal_bearing = brg;
                    reversal_score = score;
                    have_reversal = true;
                }
            } else if (!have_direct || score > direct_score) {
                best_direct = test;
                direct_agl = decision_agl;
                direct_dist = d;
                direct_bearing = brg;
                direct_score = score;
                have_direct = true;
            }
        }
    }

    if (require_lateral_escape) {
        if (have_preferred && have_lateral && (immediate_escape || side_choice_ambiguous)) {
            const float switch_gain = MAX(_te_entry_headroom_m(buffer_m), _te_dynamic_margin_m(buffer_m, 0.25f)) *
                                      (side_choice_ambiguous ? 0.50f : 1.0f);
            const bool preferred_weak = side_choice_ambiguous || preferred_agl < buffer_m * TERRAIN_CRITICAL_BUFFER_FRACTION;
            const bool lateral_much_better = lateral_score > preferred_score + switch_gain ||
                                             lateral_agl > preferred_agl + switch_gain;
            if (preferred_weak && lateral_much_better) {
                candidate = best_lateral;
                agl = lateral_agl;
                dist = lateral_dist;
                bearing_deg = lateral_bearing;
                return true;
            }
        }
        if (have_preferred) {
            candidate = best_preferred;
            agl = preferred_agl;
            dist = preferred_dist;
            bearing_deg = preferred_bearing;
            return true;
        }
        if (have_lateral) {
            candidate = best_lateral;
            agl = lateral_agl;
            dist = lateral_dist;
            bearing_deg = lateral_bearing;
            return true;
        }
        if (have_reversal) {
            candidate = best_reversal;
            agl = reversal_agl;
            dist = reversal_dist;
            bearing_deg = reversal_bearing;
            return true;
        }
    } else if (have_preferred && (!have_direct || preferred_score > direct_score - MAX(buffer_m, 1.0f) * 0.30f)) {
        candidate = best_preferred;
        agl = preferred_agl;
        dist = preferred_dist;
        bearing_deg = preferred_bearing;
        return true;
    } else if (have_lateral && (!have_direct || lateral_score > direct_score + MAX(buffer_m, 1.0f) * 0.15f)) {
        candidate = best_lateral;
        agl = lateral_agl;
        dist = lateral_dist;
        bearing_deg = lateral_bearing;
        return true;
    }

    if (have_direct) {
        candidate = best_direct;
        agl = direct_agl;
        dist = direct_dist;
        bearing_deg = direct_bearing;
        return true;
    }
    if (have_preferred) {
        candidate = best_preferred;
        agl = preferred_agl;
        dist = preferred_dist;
        bearing_deg = preferred_bearing;
        return true;
    }
    if (have_lateral) {
        candidate = best_lateral;
        agl = lateral_agl;
        dist = lateral_dist;
        bearing_deg = lateral_bearing;
        return true;
    }
    if (have_reversal) {
        candidate = best_reversal;
        agl = reversal_agl;
        dist = reversal_dist;
        bearing_deg = reversal_bearing;
        return true;
    }
    return false;
}

bool AP_SoarNav::_te_emergency_area_target(Backend &backend, const Location &loc, float target_bearing_deg, float lever_m, Location &candidate)
{
    const float yaw_deg = _wrap_360(backend.yaw_rad() * 57.2957795f);
    const uint32_t now = AP_HAL::millis();
    const bool wall_active = _te_wall_memory_active(now) && _terrain.wall_side != 0;
    const int8_t escape_side = wall_active && _terrain.escape_side != 0 ? _terrain.escape_side : 0;

    float bearings[24];
    uint8_t count = 0;
    if (wall_active && _terrain.escape_started_ms != 0) {
        bearings[count++] = _wrap_360(yaw_deg + float(escape_side) * 90.0f);
        bearings[count++] = _wrap_360(yaw_deg + float(escape_side) * 75.0f);
        bearings[count++] = _wrap_360(yaw_deg + float(escape_side) * 105.0f);
        bearings[count++] = _wrap_360(_terrain.escape_heading_deg);
        bearings[count++] = _wrap_360(_terrain.escape_heading_deg + float(escape_side) * 15.0f);
        bearings[count++] = _wrap_360(_terrain.escape_heading_deg - float(escape_side) * 15.0f);
    }

    const float right_offsets[] = {60.0f, 75.0f, 90.0f, 105.0f, 120.0f, 45.0f, -45.0f, -60.0f, -75.0f, -90.0f, -105.0f, -120.0f, 135.0f, -135.0f, 180.0f};
    const float left_offsets[] = {-60.0f, -75.0f, -90.0f, -105.0f, -120.0f, -45.0f, 45.0f, 60.0f, 75.0f, 90.0f, 105.0f, 120.0f, -135.0f, 135.0f, 180.0f};
    const float neutral_offsets[] = {60.0f, -60.0f, 75.0f, -75.0f, 90.0f, -90.0f, 105.0f, -105.0f, 120.0f, -120.0f, 45.0f, -45.0f, 135.0f, -135.0f, 180.0f};
    const float *offsets = neutral_offsets;
    uint8_t offset_count = ARRAY_SIZE(neutral_offsets);
    if (escape_side > 0) {
        offsets = right_offsets;
        offset_count = ARRAY_SIZE(right_offsets);
    } else if (escape_side < 0) {
        offsets = left_offsets;
        offset_count = ARRAY_SIZE(left_offsets);
    }
    for (uint8_t i = 0; i < offset_count && count < ARRAY_SIZE(bearings); i++) {
        bearings[count++] = _wrap_360(yaw_deg + offsets[i]);
    }

    Location center;
    if (_get_active_center_location(backend, center) && count < ARRAY_SIZE(bearings)) {
        const float center_bearing = _bearing_deg(loc, center);
        const float center_turn = fabsf(_wrap_180(center_bearing - yaw_deg));
        if (center_turn <= 120.0f) {
            bearings[count++] = center_bearing;
        }
    }
    if (count < ARRAY_SIZE(bearings)) {
        const float target_left = _wrap_360(target_bearing_deg - 90.0f);
        const float target_right = _wrap_360(target_bearing_deg + 90.0f);
        bearings[count++] = escape_side < 0 ? target_left : target_right;
        if (count < ARRAY_SIZE(bearings)) {
            bearings[count++] = escape_side < 0 ? target_right : target_left;
        }
    }
    if (count < ARRAY_SIZE(bearings)) {
        bearings[count++] = _wrap_360(target_bearing_deg + 180.0f);
    }

    bool best_non_reversal_valid = false;
    bool best_reversal_valid = false;
    Location best_non_reversal;
    Location best_reversal;
    float best_non_reversal_score = -1.0e9f;
    float best_reversal_score = -1.0e9f;
    const float buffer = _terrain_buffer_m(backend);
    const bool hard_now = _te_hard_unsafe_active(now);
    const float distances[] = {lever_m, lever_m * 0.75f, lever_m * 0.50f};

    for (uint8_t i = 0; i < count; i++) {
        if (_te_heading_turns_towards_wall(bearings[i], yaw_deg, now)) {
            continue;
        }
        const float turn = fabsf(_wrap_180(bearings[i] - yaw_deg));
        const bool large_reversal = turn > 120.0f;
        for (uint8_t j = 0; j < ARRAY_SIZE(distances); j++) {
            Location test;
            if (!_te_make_candidate(backend, loc, bearings[i], MAX(_te_candidate_min_m(backend), distances[j]), test)) {
                continue;
            }
            float min_agl = 0.0f;
            float worst = 0.0f;
            TerrainProbe probe{};
            if (!_path_min_agl_probe(backend, loc, test, min_agl, worst, &probe)) {
                continue;
            }
            if (probe.target_distance_m > probe.sample_distance_m + _te_entry_hysteresis_m(buffer)) {
                continue;
            }
            const bool critical_scan = hard_now ||
                                       (isfinite(probe.start_hagl_m) && probe.start_hagl_m < buffer * TERRAIN_CRITICAL_BUFFER_FRACTION) ||
                                       _te_probe_immediate_threat(probe, buffer);
            if (min_agl < TERRAIN_HARD_UNSAFE_AGL_M && !critical_scan) {
                continue;
            }
            const bool soft_ok = min_agl >= buffer || _te_probe_margin_low_ok(probe, buffer);
            const bool current_hard_escape = isfinite(probe.start_hagl_m) &&
                                             probe.start_hagl_m < TERRAIN_HARD_UNSAFE_AGL_M &&
                                             min_agl > probe.start_hagl_m + _te_entry_headroom_m(buffer) &&
                                             min_agl >= MAX(TERRAIN_HARD_UNSAFE_AGL_M + _te_entry_headroom_m(buffer),
                                                            buffer * 0.50f);
            if (!soft_ok && !current_hard_escape && !critical_scan) {
                continue;
            }
            const float dist = loc.get_distance(test);
            const float target_dev = fabsf(_wrap_180(bearings[i] - target_bearing_deg));
            const bool emergency_escape_side = escape_side != 0 && _wrap_180(bearings[i] - yaw_deg) * float(escape_side) > 0.0f;
            const float side_bonus = emergency_escape_side ? 70.0f : 0.0f;
            const float wide_bonus = emergency_escape_side ? constrain_float(turn - 45.0f, 0.0f, 45.0f) * 1.4f - MAX(0.0f, turn - 110.0f) * 5.0f : 0.0f;
            const float timid_penalty = emergency_escape_side && turn < 60.0f ? (60.0f - turn) * 2.0f : 0.0f;
            const float reversal_penalty = large_reversal ? 260.0f + (turn - 120.0f) * 8.0f : (turn > 90.0f ? (turn - 90.0f) * 4.0f : 0.0f);
            const float score = min_agl * 1.6f + probe.corridor_score * 0.35f + dist * 0.08f + side_bonus + wide_bonus - turn * (emergency_escape_side ? 1.0f : 2.2f) - target_dev * 0.08f - timid_penalty - reversal_penalty;
            if (large_reversal) {
                if (!best_reversal_valid || score > best_reversal_score) {
                    best_reversal = test;
                    best_reversal_score = score;
                    best_reversal_valid = true;
                }
            } else if (!best_non_reversal_valid || score > best_non_reversal_score) {
                best_non_reversal = test;
                best_non_reversal_score = score;
                best_non_reversal_valid = true;
            }
        }
    }

    if (best_non_reversal_valid) {
        candidate = best_non_reversal;
        return true;
    }
    if (best_reversal_valid) {
        candidate = best_reversal;
        return true;
    }
    return false;
}

bool AP_SoarNav::_te_target_reached(Backend &backend, const Location &loc, const Location &target, float &distance_m) const
{
    distance_m = loc.get_distance(target);
    const float buffer = _terrain_buffer_m(backend);
    float radius = MAX(float(_wp_radius_m.get()) * 1.5f, buffer * 0.50f);
    if (_terrain.evasion_initial_distance_m > 0.0f) {
        radius = MAX(float(_wp_radius_m.get()) * 0.70f, MIN(radius, _terrain.evasion_initial_distance_m * 0.20f));
    }
    return distance_m <= radius;
}

bool AP_SoarNav::_te_target_usable(Backend &backend, const Location &loc, const Location &target, bool allow_degraded_hold) const
{
    if (!_point_in_area(target) || (_using_polygon && !_segment_stays_inside(loc, target))) {
        return false;
    }
    float d = 0.0f;
    if (_te_target_reached(backend, loc, target, d)) {
        return false;
    }
    const uint32_t now = AP_HAL::millis();
    const float bearing = _bearing_deg(loc, target);
    const float turn = fabsf(_wrap_180(bearing - _te_track_or_yaw_deg(backend)));
    if (turn > 100.0f && !_te_hard_unsafe_active(now)) {
        const bool active_hold = allow_degraded_hold && _terrain.hold_until_ms != 0 && now < _terrain.hold_until_ms;
        if (!active_hold || turn > 120.0f || _te_heading_turns_towards_wall(bearing, _te_track_or_yaw_deg(backend), now)) {
            return false;
        }
    }
    if (allow_degraded_hold) {
        const float buffer = _terrain_buffer_m(backend);
        float min_agl = 0.0f;
        float worst = 0.0f;
        TerrainProbe probe{};
        if (!_path_min_agl_probe(backend, loc, target, min_agl, worst, &probe)) {
            return false;
        }
        if ((_terrain.committed_turn_side != 0 || _terrain.last_turn_side != 0) &&
            probe.center_min_agl_m > min_agl &&
            probe.center_min_agl_m >= TERRAIN_HARD_UNSAFE_AGL_M) {
            min_agl = probe.center_min_agl_m;
        }
        const bool hard_hold = min_agl < TERRAIN_HARD_UNSAFE_AGL_M &&
                               _te_hard_unsafe_active(now) &&
                               min_agl >= _terrain.evasion_min_agl_m - _terrain_buffer_m(backend) * 0.50f;
        if (min_agl < TERRAIN_HARD_UNSAFE_AGL_M && !hard_hold) {
            return false;
        }
        const bool soft_ok = min_agl >= buffer || _te_probe_margin_low_ok(probe, buffer);
        const bool current_hard_escape = isfinite(probe.start_hagl_m) &&
                                         probe.start_hagl_m < TERRAIN_HARD_UNSAFE_AGL_M &&
                                         min_agl > probe.start_hagl_m + _te_entry_headroom_m(buffer) &&
                                         min_agl >= MAX(TERRAIN_HARD_UNSAFE_AGL_M + _te_entry_headroom_m(buffer),
                                                        buffer * 0.50f);
        const float allowed_commit_drop = _te_hold_allowed_drop_m(backend, buffer);
        const bool active_hold = allow_degraded_hold && _terrain.hold_until_ms != 0 && now < _terrain.hold_until_ms;
        const bool commit_drop = min_agl < _terrain.evasion_min_agl_m - allowed_commit_drop;
        if (active_hold) {
            if (commit_drop && _te_probe_immediate_threat(probe, buffer) && !current_hard_escape && !hard_hold) {
                return false;
            }
            return true;
        }
        if (_te_low_altitude_pressure(probe, buffer) &&
            commit_drop &&
            !current_hard_escape && !hard_hold) {
            return false;
        }
        return (soft_ok || current_hard_escape || hard_hold) &&
               min_agl >= _terrain.evasion_min_agl_m - _terrain_buffer_m(backend) * 0.50f;
    }

    return _is_path_terrain_safe(backend, target);
}

void AP_SoarNav::_te_clear_state(bool clear_resume)
{
    _terrain.state = TerrainState::IDLE;
    _terrain.hold_until_ms = 0;
    _terrain.next_check_ms = 0;
    _terrain.evasion_target_valid = false;
    _terrain.evasion_degraded = false;
    _terrain.evasion_target_updated_ms = 0;
    _terrain.evasion_initial_distance_m = 0.0f;
    _terrain.evasion_min_agl_m = 0.0f;
    _terrain.replan_not_before_ms = 0;
    _terrain.last_decision = TerrainDecision::CLEAR;
    _terrain.last_replan_reason = TerrainReplanReason::NONE;
    _terrain.last_commit_hold_log_ms = 0;
    if (clear_resume) {
        _terrain.last_safe_heading_valid = false;
        _terrain.last_safe_heading_deg = 0.0f;
        _terrain.committed_turn_side = 0;
        _terrain.commitment_started_ms = 0;
        _terrain.last_commit_log_ms = 0;
        _terrain.committed_bearing_deg = 0.0f;
        _terrain.committed_min_agl_m = 0.0f;
        _terrain.committed_first_threat_time_s = -1.0f;
        _terrain.committed_first_threat_distance_m = -1.0f;
        _terrain.resume_valid = false;
        _terrain.resume_safe_count = 0;
        _terrain.side_wall_active = false;
        _terrain.wall_side = 0;
        _terrain.escape_side = 0;
        _terrain.side_wall_hold_until_ms = 0;
        _terrain.escape_started_ms = 0;
        _terrain.last_wall_flip_ms = 0;
        _terrain.escape_heading_deg = 0.0f;
        _terrain.side_wall_clear_count = 0;
        _terrain.pending_wall_side = 0;
        _terrain.pending_wall_count = 0;
    }
}

float AP_SoarNav::_terrain_buffer_m(Backend &backend) const
{
    const float speed = _nav_speed_mps(backend);
    const float base = MAX(float(_terrain_buffer_min_m.get()), 0.0f);
    const float look = MAX(float(_terrain_lookahead_s.get()), 1.0f);
    return MAX(base, speed * look * 0.45f);
}

uint32_t AP_SoarNav::_te_policy_base_ms() const
{
    const float look_s = MAX(float(_terrain_lookahead_s.get()), 1.0f);
    return uint32_t(look_s * 1000.0f);
}

uint32_t AP_SoarNav::_te_replan_backoff_ms() const
{
    return MAX(TERRAIN_CHECK_INTERVAL_MS, _te_hold_min_ms() / 2U);
}

uint32_t AP_SoarNav::_te_hold_min_ms() const
{
    return MAX(TERRAIN_CHECK_INTERVAL_MS * 2U, _te_policy_base_ms());
}

uint32_t AP_SoarNav::_te_degraded_hold_min_ms() const
{
    return MAX(_te_hold_min_ms() * 2U, _te_policy_base_ms() * 3U);
}

uint32_t AP_SoarNav::_te_hold_max_ms() const
{
    return MAX(_te_degraded_hold_min_ms(), _te_policy_base_ms() * 6U);
}

uint32_t AP_SoarNav::_te_side_wall_hold_ms() const
{
    return MAX(_te_hold_min_ms(), (_te_policy_base_ms() * 5U) / 2U);
}

uint32_t AP_SoarNav::_te_side_wall_min_switch_ms() const
{
    return _te_side_wall_hold_ms();
}

uint32_t AP_SoarNav::_te_reentry_cooldown_ms() const
{
    return _te_hold_min_ms();
}

uint32_t AP_SoarNav::_te_post_reroute_block_ms() const
{
    return _te_hold_max_ms();
}

uint32_t AP_SoarNav::_te_hard_unsafe_hold_ms() const
{
    return MAX(_te_degraded_hold_min_ms(), _te_side_wall_hold_ms() + (_te_hold_min_ms() * 2U));
}

void AP_SoarNav::_update_polar_learning(Backend &backend, const Location &loc)
{
    (void)loc;
    if (_polar.learned || _state != State::NAVIGATING || _manual_override_active || backend.mode_number() == Backend::ModeNumber::THERMAL) {
        return;
    }
    if (backend.throttle_percent() > 10.0f) {
        return;
    }
    if (fabsf(degrees(backend.roll_rad())) > 12.0f) {
        return;
    }

    Vector3f vned;
    if (!backend.velocity_ned(vned)) {
        return;
    }
    Vector3f wind;
    if (!backend.wind_vector(wind)) {
        wind.zero();
    }
    const float va_n = vned.x - wind.x;
    const float va_e = vned.y - wind.y;
    const float v = sqrtf(va_n * va_n + va_e * va_e);
    if (!isfinite(v) || v <= 8.0f) {
        return;
    }

    const float sink_raw = vned.z;
    if (!isfinite(sink_raw) || sink_raw < 0.10f) {
        return;
    }

    const uint32_t now = AP_HAL::millis();
    if (_polar.first_sample_ms == 0) {
        _polar.first_sample_ms = now;
    }
    const uint32_t dt_ms = _polar.last_sample_ms == 0 ? 0 : now - _polar.last_sample_ms;
    _polar.last_sample_ms = now;

    if (dt_ms > 0) {
        const float k = 1.0f - expf(-float(dt_ms) / 60000.0f);
        _polar.sink_bias += k * (sink_raw - _polar.sink_bias);
        if (_polar.v_ema <= 0.0f) {
            _polar.v_ema = v;
        }
        if (_polar.v2_ema <= 0.0f) {
            _polar.v2_ema = v * v;
        }
        _polar.v_ema += k * (v - _polar.v_ema);
        _polar.v2_ema += k * (v * v - _polar.v2_ema);
    }
    const float sink = sink_raw - _polar.sink_bias;

    if (dt_ms > 0) {
        const float decay = expf(-float(dt_ms) / 120000.0f);
        _polar.s11 *= decay;
        _polar.s22 *= decay;
        _polar.s12 *= decay;
        _polar.y1 *= decay;
        _polar.y2 *= decay;
        _polar.n *= decay;
    }

    const float f1 = v * v * v;
    const float f2 = 1.0f / v;
    _polar.s11 += f1 * f1;
    _polar.s22 += f2 * f2;
    _polar.s12 += f1 * f2;
    _polar.y1 += f1 * sink;
    _polar.y2 += f2 * sink;
    _polar.n += 1.0f;
    if (_polar.good_samples < 65535) {
        _polar.good_samples++;
    }

    if (_log_level.get() > 1 && (_polar.last_debug_ms == 0 || now - _polar.last_debug_ms >= 30000U)) {
        _polar.last_debug_ms = now;
        backend.send_text(MAV_SEVERITY_INFO,
                          "SoarNav: PLrn S%u Ne%.0f V%.1f",
                          unsigned(_polar.good_samples),
                          double(_polar.n),
                          double(v));
    }

    if (now - _polar.first_sample_ms < 60000U || _polar.good_samples < 150U) {
        return;
    }
    if (_polar.last_fit_ms != 0 && now - _polar.last_fit_ms < 8000U) {
        return;
    }
    _polar.last_fit_ms = now;

    float cd0 = 0.0f;
    float bb = 0.0f;
    float vc = 15.0f;
    if (!backend.param_get_float("SOAR_POLAR_CD0", cd0) || !backend.param_get_float("SOAR_POLAR_B", bb)) {
        return;
    }
    backend.param_get_float("AIRSPEED_CRUISE", vc);
    if (vc <= 0.0f || !isfinite(vc)) {
        vc = 15.0f;
    }
    if (_polar.last_saved_cd0 <= 0.0f) {
        _polar.last_saved_cd0 = cd0;
    }
    if (_polar.last_saved_b <= 0.0f) {
        _polar.last_saved_b = bb;
    }

    const float v_var = MAX(0.0f, _polar.v2_ema - _polar.v_ema * _polar.v_ema);
    const float v_std = sqrtf(v_var);
    float new_cd0 = cd0;
    float new_b = bb;
    bool solution_valid = false;

    if (v_std < 0.8f) {
        const float sink_model = (cd0 * v * v + bb) / v;
        if (!isfinite(sink_model) || sink_model <= 0.0f) {
            return;
        }
        float s_mag_inst = sink / sink_model;
        s_mag_inst = constrain_float(s_mag_inst, 0.90f, 1.10f);
        _polar.s_mag_log_ema += 0.2f * (logf(s_mag_inst) - _polar.s_mag_log_ema);
        const float s_mag = expf(_polar.s_mag_log_ema);
        new_cd0 = cd0 * s_mag;
        new_b = bb * s_mag;
        new_cd0 = _round_to(constrain_float(new_cd0, 0.005f, 0.500f), 0.0001f);
        new_b = _round_to(constrain_float(new_b, 0.005f, 0.060f), 0.0001f);
        solution_valid = true;
    } else {
        const float det = _polar.s11 * _polar.s22 - _polar.s12 * _polar.s12;
        if (!isfinite(det) || det <= 1.0e-9f) {
            return;
        }
        const float a_est = (_polar.y1 * _polar.s22 - _polar.y2 * _polar.s12) / det;
        const float b_est = (_polar.y2 * _polar.s11 - _polar.y1 * _polar.s12) / det;
        if (!isfinite(a_est) || !isfinite(b_est) || a_est <= 0.0f || b_est <= 0.0f) {
            return;
        }
        const float pred = a_est * v * v * v + b_est / v;
        const float e = fabsf(sink - pred);
        if (!_polar.err_ema_valid) {
            _polar.err_ema = e;
            _polar.err_ema_valid = true;
        } else {
            _polar.err_ema = 0.9f * _polar.err_ema + 0.1f * e;
        }
        if (_polar.err_ema > 0.6f) {
            return;
        }
        const float r_est = b_est / a_est;
        const float r_cur = bb / cd0;
        if (!isfinite(r_est) || !isfinite(r_cur) || r_est <= 0.0f || r_cur <= 0.0f) {
            return;
        }
        float h = sqrtf(r_est / r_cur);
        h = constrain_float(h, 0.90f, 1.10f);
        const float cd0_shape = cd0 / h;
        const float b_shape = bb * h;
        const float denom = a_est * vc * vc + b_est / (vc * vc);
        if (!isfinite(denom) || denom <= 0.0f) {
            return;
        }
        const float eff = 1.0f / denom;
        if (!isfinite(eff) || eff <= 0.0f) {
            return;
        }
        const float target_prod = 1.0f / (4.0f * eff * eff);
        const float prod_shape = cd0_shape * b_shape;
        if (!isfinite(target_prod) || !isfinite(prod_shape) || target_prod <= 0.0f || prod_shape <= 0.0f) {
            return;
        }
        float s_prod = sqrtf(target_prod / prod_shape);
        s_prod = constrain_float(s_prod, 0.90f, 1.15f);
        new_cd0 = _round_to(constrain_float(cd0_shape * s_prod, 0.005f, 0.500f), 0.001f);
        new_b = _round_to(constrain_float(b_shape * s_prod, 0.005f, 0.060f), 0.001f);
        solution_valid = true;
    }

    if (!solution_valid) {
        return;
    }
    const bool first_commit = (_polar.last_commit_ms == 0);
    const bool rate_ok = first_commit || (now - _polar.last_commit_ms >= 90000U);
    const float rel_cd0 = _polar.last_saved_cd0 > 0.0f ? fabsf(new_cd0 - _polar.last_saved_cd0) / _polar.last_saved_cd0 : 1.0f;
    const float rel_b = _polar.last_saved_b > 0.0f ? fabsf(new_b - _polar.last_saved_b) / _polar.last_saved_b : 1.0f;
    const bool significant = first_commit || rel_cd0 >= 0.03f || rel_b >= 0.06f;

    if (significant && rate_ok) {
        _polar.stable_count++;
    } else {
        _polar.stable_count = 0;
    }
    if (_polar.stable_count >= 3) {
        backend.param_set_float("SOAR_POLAR_CD0", new_cd0);
        backend.param_set_float("SOAR_POLAR_B", new_b);
        _polar.last_commit_ms = now;
        _polar.last_saved_cd0 = new_cd0;
        _polar.last_saved_b = new_b;
        _polar.stable_count = 0;
        _polar.learned = true;
        if (_log_level.get() > 0) {
            backend.send_text(MAV_SEVERITY_INFO, "SoarNav: Polar Learn: CD0=%.4f B=%.4f", (double)new_cd0, (double)new_b);
        }
    }
}

void AP_SoarNav::_update_motor_failure(Backend &backend, const Location &loc)
{
    const int8_t dyn_mode = _dynamic_soar_alt.get();
    const Backend::ModeNumber mode = backend.mode_number();
    const float dist_home = _home_distance_m(backend, loc);
    const float wp_radius = MAX(float(_wp_radius_m.get()), 10.0f);
    const float suppress_radius = MAX(100.0f, wp_radius * 2.0f);

    if (!backend.armed() || _state != State::NAVIGATING || _manual_override_active ||
        dyn_mode == 0 || dyn_mode == 3 || mode == Backend::ModeNumber::THERMAL || dist_home <= suppress_radius) {
        _motor_failure_check_active = false;
        _motor_on_start_ms = 0;
        _rpm_failure_start_ms = 0;
        return;
    }

    float soar_alt_min = 0.0f;
    if (!backend.param_get_float("SOAR_ALT_MIN", soar_alt_min) || soar_alt_min <= 0.0f) {
        return;
    }

    const float current_alt = _alt_above_home_m(loc);
    const float throttle_output = backend.throttle_percent();
    const bool motor_should_be_on = current_alt < soar_alt_min && throttle_output > 10.0f;
    if (!motor_should_be_on) {
        _motor_failure_check_active = false;
        _motor_on_start_ms = 0;
        _rpm_failure_start_ms = 0;
        return;
    }

    const uint32_t now = AP_HAL::millis();
    auto trigger_rtl = [&]() {
        backend.send_text(MAV_SEVERITY_CRITICAL, "SoarNav: MOTOR FAILURE DETECTED! RTL ACTIVATED.");
        if (backend.set_rtl_mode()) {
            _restore_initial_soar_alts(backend);
            _set_state(State::ERROR, backend, "motor failure");
        }
    };

    float rpm_value = -1.0f;
    if (backend.rpm_reading(rpm_value)) {
        _motor_failure_check_active = false;
        _motor_on_start_ms = 0;
        const bool rpm_issue = throttle_output > 15.0f && rpm_value < 100.0f;
        if (rpm_issue) {
            if (_rpm_failure_start_ms == 0) {
                _rpm_failure_start_ms = now;
            } else if (now - _rpm_failure_start_ms > MOTOR_FAILURE_DEBOUNCE_MS) {
                trigger_rtl();
                _rpm_failure_start_ms = now + 60000U;
            }
        } else {
            _rpm_failure_start_ms = 0;
        }
        return;
    }

    _rpm_failure_start_ms = 0;
    if (!_motor_failure_check_active) {
        _motor_failure_check_active = true;
        _motor_on_start_ms = now;
        _altitude_at_motor_check_start_m = current_alt;
        _soar_alt_min_at_motor_check_m = soar_alt_min;
        return;
    }

    if (now - _motor_on_start_ms <= MOTOR_FAILURE_DELAY_MS) {
        return;
    }

    Vector3f vned;
    if (!backend.velocity_ned(vned)) {
        return;
    }
    const float climb_rate = -vned.z;
    const bool climbed_enough = current_alt > _altitude_at_motor_check_start_m + 1.0f;
    if (climb_rate < 0.20f && !climbed_enough) {
        trigger_rtl();
        _motor_failure_check_active = false;
        _motor_on_start_ms = 0;
    } else {
        _motor_failure_check_active = false;
        _motor_on_start_ms = 0;
    }
}

void AP_SoarNav::_update_rtlh(Backend &backend, const Location &loc)
{
    const Backend::ModeNumber mode = backend.mode_number();
    const uint32_t now = AP_HAL::millis();

    if (_rtlh.last_mode == Backend::ModeNumber::RTL && mode != Backend::ModeNumber::RTL && mode != Backend::ModeNumber::GUIDED) {
        _rtlh.active = false;
        _rtlh.engaged_guided = false;
        _rtlh.abort_until_next_rtl = true;
    }

    if (_rtlh.engaged_guided && mode != Backend::ModeNumber::GUIDED) {
        _rtlh.abort_until_next_rtl = true;
        _rtlh.engaged_guided = false;
    }

    if (_rtlh.last_mode != Backend::ModeNumber::RTL && mode == Backend::ModeNumber::RTL) {
        _rtlh.active = false;
        _rtlh.abort_until_next_rtl = false;
    }
    _rtlh.last_mode = mode;

    if (_rtlh.abort_until_next_rtl) {
        return;
    }

    if (!_using_rally_points) {
        if (_rtlh.engaged_guided && mode == Backend::ModeNumber::GUIDED) {
            backend.set_rtl_mode();
        }
        _rtlh.active = false;
        _rtlh.engaged_guided = false;
        return;
    }

    Location home;
    if (!backend.home_location(home)) {
        return;
    }

    float rtl_radius = 0.0f;
    if (!backend.param_get_float("RTL_RADIUS", rtl_radius) || rtl_radius <= 0.0f) {
        rtl_radius = 50.0f;
    }

    const float d = loc.get_distance(home);
    if (_rtlh.engaged_guided && mode == Backend::ModeNumber::GUIDED) {
        if (d <= rtl_radius) {
            backend.send_text(MAV_SEVERITY_INFO, "SoarNav: RTL Override: In home area. Resuming RTL.");
            if (backend.set_rtl_mode()) {
                _rtlh.abort_until_next_rtl = true;
                _rtlh.engaged_guided = false;
                _rtlh.active = false;
                _last_sent_valid = false;
            }
        }
        return;
    }

    if (mode != Backend::ModeNumber::RTL) {
        _rtlh.active = false;
        return;
    }

    if (!_rtlh.active) {
        _rtlh.active = true;
        _rtlh.t0_ms = now;
        _rtlh.d0_m = d;
        return;
    }

    if (now - _rtlh.t0_ms < 3000U) {
        return;
    }
    if (d <= rtl_radius * 1.5f) {
        return;
    }

    Vector3f vned;
    float gs = backend.ground_speed_mps();
    if (backend.velocity_ned(vned)) {
        gs = Vector2f(vned.x, vned.y).length();
    }
    const float needed_gain = MAX(15.0f, 0.3f * gs * 3.0f);
    if (_rtlh.d0_m - d >= needed_gain) {
        return;
    }

    float rtl_alt_m = 0.0f;
    backend.param_get_float("RTL_ALTITUDE", rtl_alt_m);
    home.set_alt_cm(int32_t(rtl_alt_m * 100.0f), Location::AltFrame::ABOVE_HOME);
    backend.send_text(MAV_SEVERITY_INFO, "SoarNav: RTL Stall: Force direct Home route @%.0fm", (double)d);
    if (backend.set_guided_mode() && backend.set_guided_target(home)) {
        _rtlh.engaged_guided = true;
        _rtlh.t0_ms = now;
    }
}

void AP_SoarNav::_recenter_area(Backend &backend, const Location &loc)
{
    if (_using_polygon) {
        if (_polygon_count < 3) {
            backend.send_text(MAV_SEVERITY_ERROR, "SoarNav: Polygon recenter failed: invalid polygon.");
            return;
        }
        for (uint8_t i = 0; i < _polygon_count; i++) {
            _polygon_points[i] = loc;
            _polygon_points[i].lat = loc.lat + _polygon_lat_offsets[i];
            _polygon_points[i].lng = loc.lng + _polygon_lng_offsets[i];
        }
        _dynamic_center_valid = false;
        if (!_finalize_polygon_points()) {
            backend.send_text(MAV_SEVERITY_ERROR, "SoarNav: Polygon recenter failed.");
            return;
        }
        backend.send_text(MAV_SEVERITY_INFO, "SoarNav: Stick CMD: Polygon re-centered.");
    } else {
        if (_radius_m.get() <= 0.0f) {
            return;
        }
        _dynamic_center = loc;
        _dynamic_center_valid = true;
        _center = loc;
        _center_valid = true;
        backend.send_text(MAV_SEVERITY_INFO, "SoarNav: Stick CMD: Radius Area re-centered.");
    }
    _target_valid = false;
    _last_sent_valid = false;
    _grid_initialized = false;
    _grid_force_reinit = true;
    _force_grid_after_reset = false;
    if (_state == State::PILOT_OVERRIDE || _manual_override_active) {
        _last_pilot_input_ms = AP_HAL::millis();
        return;
    }
    _set_state(State::NAVIGATING, backend, "recenter");
}

float AP_SoarNav::_home_distance_m(Backend &backend, const Location &loc) const
{
    Location home;
    if (!backend.home_location(home)) {
        return 0;
    }
    return loc.get_distance(home);
}

float AP_SoarNav::_alt_amsl_m(const Location &loc) const
{
    int32_t alt_cm = 0;
    if (loc.get_alt_cm(Location::AltFrame::ABSOLUTE, alt_cm)) {
        return alt_cm * 0.01f;
    }
    return loc.alt * 0.01f;
}

float AP_SoarNav::_alt_above_home_m(const Location &loc) const
{
    int32_t alt_cm = 0;
    if (loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, alt_cm)) {
        return alt_cm * 0.01f;
    }
    return loc.alt * 0.01f;
}

uint32_t AP_SoarNav::_rand()
{
    _rng_state ^= _rng_state << 13;
    _rng_state ^= _rng_state >> 17;
    _rng_state ^= _rng_state << 5;
    return _rng_state;
}

float AP_SoarNav::_randf(float min_v, float max_v)
{
    const float u = (_rand() & 0xFFFFFFU) / float(0xFFFFFFU);
    return min_v + (max_v - min_v) * u;
}

float AP_SoarNav::_wrap_360(float deg)
{
    while (deg < 0) { deg += 360.0f; }
    while (deg >= 360.0f) { deg -= 360.0f; }
    return deg;
}

float AP_SoarNav::_wrap_180(float deg)
{
    deg = _wrap_360(deg);
    if (deg > 180.0f) {
        deg -= 360.0f;
    }
    return deg;
}

float AP_SoarNav::_bearing_deg(const Location &from, const Location &to)
{
    return _wrap_360(from.get_bearing_to(to) * 0.01f);
}

float AP_SoarNav::_round_to(float v, float quantum)
{
    if (quantum <= 0.0f) {
        return v;
    }
    return floorf(v / quantum + 0.5f) * quantum;
}

const char *AP_SoarNav::_compass(float bearing_deg)
{
    static const char *dirs[] = {"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};
    const uint8_t idx = uint8_t((_wrap_360(bearing_deg) + 11.25f) / 22.5f) & 0x0F;
    return dirs[idx];
}

#endif
