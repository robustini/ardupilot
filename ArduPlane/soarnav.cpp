#include "Plane.h"

#if HAL_SOARNAV_ENABLED

#include <AP_Baro/AP_Baro.h>
#include <AP_RPM/AP_RPM.h>
#include <RC_Channel/RC_Channel.h>
#include <stdarg.h>

class PlaneSoarNavBackend : public AP_SoarNav::Backend {
public:
    explicit PlaneSoarNavBackend(Plane &p) : plane(p) {}

    bool armed() const override { return plane.arming.is_armed(); }
    bool is_flying() const override { return plane.is_flying(); }

    ModeNumber mode_number() const override { return plane.control_mode != nullptr ? convert_mode(plane.control_mode->mode_number()) : ModeNumber::UNKNOWN; }
    ModeNumber previous_mode_number() const override { return plane.previous_mode != nullptr ? convert_mode(plane.previous_mode->mode_number()) : ModeNumber::UNKNOWN; }

    bool set_guided_mode() override
    {
#if HAL_QUADPLANE_ENABLED
        if (plane.quadplane.in_vtol_mode()) {
            return false;
        }
#endif
        return plane.set_mode(plane.mode_guided, ModeReason::GCS_COMMAND);
    }

    bool set_rtl_mode() override
    {
        return plane.set_mode(plane.mode_rtl, ModeReason::GCS_COMMAND);
    }


    bool set_mode(ModeNumber mode) override
    {
        switch (mode) {
        case ModeNumber::FBWB:
            return plane.set_mode(plane.mode_fbwb, ModeReason::GCS_COMMAND);
        case ModeNumber::CRUISE:
            return plane.set_mode(plane.mode_cruise, ModeReason::GCS_COMMAND);
        case ModeNumber::RTL:
            return plane.set_mode(plane.mode_rtl, ModeReason::GCS_COMMAND);
        case ModeNumber::LOITER:
            return plane.set_mode(plane.mode_loiter, ModeReason::GCS_COMMAND);
        case ModeNumber::GUIDED:
            return set_guided_mode();
#if HAL_SOARING_ENABLED
        case ModeNumber::THERMAL:
            return plane.set_mode(plane.mode_thermal, ModeReason::GCS_COMMAND);
#endif
        default:
            return false;
        }
    }

    bool set_guided_target(const Location &loc, bool terrain_evasion_target) override
    {
        Location target = loc;
        if (plane.control_mode == nullptr) {
            return false;
        }
#if HAL_SOARING_ENABLED
        if (plane.control_mode == &plane.mode_thermal) {
            return plane.mode_guided.set_soaring_target(target, terrain_evasion_target);
        }
#endif
        if (plane.control_mode != &plane.mode_guided) {
            if (!set_guided_mode()) {
                return false;
            }
        }
#if HAL_SOARING_ENABLED
        return plane.mode_guided.handle_soarnav_guided_request(target, terrain_evasion_target);
#else
        return plane.mode_guided.handle_guided_request(target);
#endif
    }

    bool navigation_target(Location &loc) const override
    {
        if (plane.control_mode == nullptr) {
            return false;
        }
#if HAL_SOARING_ENABLED
        if (plane.control_mode == &plane.mode_thermal && plane.previous_mode == &plane.mode_guided) {
            Location prev;
            return plane.mode_guided.get_soaring_target(prev, loc);
        }
#endif
        if (plane.control_mode == &plane.mode_guided) {
            loc = plane.next_WP_loc;
            return loc.initialised();
        }
        return false;
    }

    bool current_location(Location &loc) const override
    {
        if (!plane.current_loc.initialised()) {
            return false;
        }
        loc = plane.current_loc;
        return true;
    }

    bool home_location(Location &loc) const override
    {
        if (!plane.ahrs.home_is_set()) {
            return false;
        }
        loc = plane.ahrs.get_home();
        return loc.initialised();
    }

    bool relative_position_ned_home(Vector3f &ned_m) const override
    {
        return plane.ahrs.get_relative_position_NED_home(ned_m);
    }

    bool terrain_height_amsl(const Location &loc, float &height_m) const override
    {
#if AP_TERRAIN_AVAILABLE
        return plane.terrain.height_amsl(loc, height_m, true);
#else
        return false;
#endif
    }

    bool height_above_ground_m(float &height_m) const override
    {
        return plane.ahrs.get_hagl(height_m);
    }

    bool wind_vector(Vector3f &wind_ned) const override
    {
        return plane.ahrs.get_wind(wind_ned);
    }

    bool velocity_ned(Vector3f &vel_ned) const override
    {
        return plane.ahrs.get_velocity_NED(vel_ned);
    }

    float ground_speed_mps() const override
    {
        return plane.ahrs.groundspeed();
    }

    float climb_rate_mps() const override
    {
        return AP::baro().get_climb_rate();
    }

    bool airspeed_estimate_mps(float &airspeed) const override
    {
        AP_AHRS::AirspeedEstimateType type = AP_AHRS::AirspeedEstimateType::NO_NEW_ESTIMATE;
        return plane.ahrs.airspeed_EAS(airspeed, type);
    }

    float throttle_percent() const override
    {
        return SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
    }

    bool motor_running() const override
    {
        return throttle_percent() > 5.0f;
    }

    bool rpm_ok(float min_rpm) const override
    {
        float value = 0.0f;
        if (!rpm_reading(value)) {
            return true;
        }
        return value >= min_rpm;
    }

    bool rpm_reading(float &rpm_value) const override
    {
#if AP_RPM_ENABLED
        AP_RPM *rpm = AP::rpm();
        if (rpm == nullptr) {
            return false;
        }
        bool have = false;
        float best = -1.0f;
        for (uint8_t i = 0; i < RPM_MAX_INSTANCES; i++) {
            if (!rpm->enabled(i)) {
                continue;
            }
            float value = 0.0f;
            if (rpm->get_rpm(i, value) && value >= 0.0f) {
                best = MAX(best, value);
                have = true;
            }
        }
        if (!have) {
            return false;
        }
        rpm_value = best;
        return true;
#else
        return false;
#endif
    }

    float roll_input_norm() const override
    {
        return plane.channel_roll != nullptr ? plane.channel_roll->norm_input_dz() : 0;
    }

    float roll_rad() const override
    {
        return plane.ahrs.get_roll_rad();
    }

    float yaw_rad() const override
    {
        return plane.ahrs.get_yaw_rad();
    }

    float pitch_input_norm() const override
    {
        return plane.channel_pitch != nullptr ? plane.channel_pitch->norm_input_dz() : 0;
    }

    float yaw_input_norm() const override
    {
        return plane.channel_rudder != nullptr ? plane.channel_rudder->norm_input_dz() : 0;
    }

    bool soar_switch_active() const override
    {
        const RC_Channel *chan = rc().find_channel_for_option(RC_Channel::AUX_FUNC::SOARING);
        return chan != nullptr && chan->get_aux_switch_pos() != RC_Channel::AuxSwitchPos::LOW;
    }

    bool autotune_active() const override
    {
        const RC_Channel *chan = rc().find_channel_for_option(RC_Channel::AUX_FUNC::FW_AUTOTUNE);
        if (chan != nullptr && chan->get_aux_switch_pos() != RC_Channel::AuxSwitchPos::LOW) {
            return true;
        }
        return plane.autotuning || plane.control_mode == &plane.mode_autotune;
    }

    bool soaring_active() const override
    {
        return plane.g2.soaring_controller.is_active();
    }

    bool soaring_throttle_suppressed() const override
    {
        return plane.g2.soaring_controller.get_throttle_suppressed();
    }

    bool param_get_float(const char *name, float &value) const override
    {
        enum ap_var_type ptype = AP_PARAM_NONE;
        AP_Param *vp = AP_Param::find(name, &ptype);
        if (vp == nullptr) {
            return false;
        }
        switch (ptype) {
        case AP_PARAM_FLOAT:
            value = ((AP_Float *)vp)->get();
            return true;
        case AP_PARAM_INT8:
            value = ((AP_Int8 *)vp)->get();
            return true;
        case AP_PARAM_INT16:
            value = ((AP_Int16 *)vp)->get();
            return true;
        case AP_PARAM_INT32:
            value = ((AP_Int32 *)vp)->get();
            return true;
        default:
            return false;
        }
    }

    bool param_set_float(const char *name, float value) override
    {
        return AP_Param::set_by_name(name, value);
    }

    uint8_t rally_count() const override
    {
#if HAL_RALLY_ENABLED
        return plane.rally.get_rally_total();
#else
        return 0;
#endif
    }

    bool rally_location(uint8_t index, Location &loc) const override
    {
#if HAL_RALLY_ENABLED
        return plane.rally.get_rally_location_with_index(index, loc);
#else
        return false;
#endif
    }

    uint32_t rally_last_change_ms() const override
    {
#if HAL_RALLY_ENABLED
        return plane.rally.last_change_time_ms();
#else
        return 0;
#endif
    }

    void send_text(MAV_SEVERITY severity, const char *fmt, ...) const override
    {
        char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN + 1];
        va_list ap;
        va_start(ap, fmt);
        hal.util->vsnprintf(text, sizeof(text), fmt, ap);
        va_end(ap);
        plane.gcs().send_text(severity, "%s", text);
    }

private:
    Plane &plane;

    static ModeNumber convert_mode(Mode::Number mode)
    {
        switch (mode) {
        case Mode::Number::FLY_BY_WIRE_B: return ModeNumber::FBWB;
        case Mode::Number::CRUISE: return ModeNumber::CRUISE;
        case Mode::Number::RTL: return ModeNumber::RTL;
        case Mode::Number::LOITER: return ModeNumber::LOITER;
        case Mode::Number::GUIDED: return ModeNumber::GUIDED;
#if HAL_SOARING_ENABLED
        case Mode::Number::THERMAL: return ModeNumber::THERMAL;
#endif
        default: return ModeNumber::UNKNOWN;
        }
    }

};

void Plane::refresh_soarnav_param_visibility()
{
    float soar_enable = 0.0f;
    const bool visible = AP_Param::get("SOAR_ENABLE", soar_enable) && soar_enable > 0.0f;
    AP_SoarNav::set_parameter_visibility(visible);
}

void Plane::update_soarnav()
{
    refresh_soarnav_param_visibility();
    PlaneSoarNavBackend backend(*this);
    if (AP_SoarNav::var_info_visible == nullptr) {
        if (g2.soarnav.running()) {
            g2.soarnav.stop(backend);
        }
        return;
    }
    g2.soarnav.update(backend);
}

void Plane::release_soarnav_for_guided_request()
{
    PlaneSoarNavBackend backend(*this);
    g2.soarnav.guided_override(backend);
}

#endif
