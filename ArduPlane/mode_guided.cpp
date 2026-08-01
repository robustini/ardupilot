#include "mode.h"
#include "Plane.h"

bool ModeGuided::_enter()
{
    plane.guided_throttle_passthru = false;

#if HAL_SOARING_ENABLED
    const bool restore_soaring_target = soaring_restore.valid && plane.previous_mode == &plane.mode_thermal;
#endif

    Location loc{plane.current_loc};

#if HAL_SOARING_ENABLED
    if (restore_soaring_target) {
        loc = soaring_restore.target_loc;
    } else
#endif
    {
#if HAL_QUADPLANE_ENABLED
        if (plane.quadplane.guided_mode_enabled()) {
            /*
              if using Q_GUIDED_MODE then project forward by the stopping distance
            */
            loc.offset_bearing(degrees(ahrs.groundspeed_vector().angle()),
                               plane.quadplane.stopping_distance_m());
        }
#endif
    }

#if HAL_SOARING_ENABLED
    active_radius_m = restore_soaring_target ? soaring_restore.radius_m : 0;
#else
    active_radius_m = 0;
#endif

#if HAL_SOARNAV_ENABLED
    soarnav_horizontal_target = restore_soaring_target && soaring_restore.horizontal_target;
    soarnav_terrain_corridor_target = soarnav_horizontal_target && soaring_restore.terrain_corridor_target;
    soarnav_energy_ref_valid = soarnav_horizontal_target && soaring_restore.energy_ref_valid;
    soarnav_energy_ref_amsl_cm = soarnav_energy_ref_valid ? soaring_restore.energy_ref_amsl_cm : 0;
    if (soarnav_horizontal_target) {
        prepare_soarnav_target_altitude(loc);
    }
#endif

    plane.set_guided_WP(loc);

#if HAL_SOARNAV_ENABLED
    if (soarnav_horizontal_target) {
        plane.g2.soaring_controller.init_cruising();
    }
#endif

#if HAL_SOARING_ENABLED
    if (restore_soaring_target) {
        plane.loiter.direction = soaring_restore.direction;
    }
    soaring_restore.valid = false;
#endif

    return true;
}

bool ModeGuided::does_automatic_thermal_switch() const
{
#if HAL_SOARNAV_ENABLED
    if (!soarnav_horizontal_target || !soarnav_xy_only_allowed() || plane.guided_throttle_passthru) {
        return false;
    }

    const uint32_t now = millis();
    const float timeout_ms = plane.g2.guided_timeout * 1000.0f;

    if ((plane.guided_state.last_forced_rpy_ms.x > 0 && now - plane.guided_state.last_forced_rpy_ms.x < timeout_ms) ||
        (plane.guided_state.last_forced_rpy_ms.y > 0 && now - plane.guided_state.last_forced_rpy_ms.y < timeout_ms) ||
        (plane.guided_state.last_forced_rpy_ms.z > 0 && now - plane.guided_state.last_forced_rpy_ms.z < timeout_ms) ||
        (plane.guided_state.last_forced_throttle_ms > 0 && now - plane.guided_state.last_forced_throttle_ms < timeout_ms)) {
        return false;
    }

#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
    if (plane.guided_state.target_heading_type != GUIDED_HEADING_NONE) {
        return false;
    }
#endif

    return true;
#else
    return false;
#endif
}

bool ModeGuided::inhibits_automatic_thermal_entry() const
{
#if HAL_SOARNAV_ENABLED
    if (!soarnav_horizontal_target) {
        return false;
    }

    if (soarnav_terrain_corridor_target) {
        return true;
    }

    return abs(plane.nav_roll_cd) > 700 ||
           abs(ahrs.roll_sensor) > 700 ||
           abs(plane.nav_pitch_cd) > 700;
#else
    return false;
#endif
}

void ModeGuided::update()
{
#if HAL_QUADPLANE_ENABLED
    if (plane.auto_state.vtol_loiter && plane.quadplane.available()) {
        plane.quadplane.guided_update();
        return;
    }
#endif

    // Received an external msg that guides roll within g2.guided_timeout?
    if (plane.guided_state.last_forced_rpy_ms.x > 0 &&
            millis() - plane.guided_state.last_forced_rpy_ms.x < plane.g2.guided_timeout*1000.0f) {
        plane.nav_roll_cd = constrain_int32(plane.guided_state.forced_rpy_cd.x, -plane.roll_limit_cd, plane.roll_limit_cd);
        plane.update_load_factor();

#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
    // guided_state.target_heading is radians at this point between -pi and pi ( defaults to -4 )
    // This function is used in Guided and AvoidADSB, check for guided
    } else if ((plane.control_mode == &plane.mode_guided) && (plane.guided_state.target_heading_type != GUIDED_HEADING_NONE) ) {
        uint32_t tnow = AP_HAL::millis();
        float delta = (tnow - plane.guided_state.target_heading_time_ms) * 1e-3f;
        plane.guided_state.target_heading_time_ms = tnow;

        float error = 0.0f;
        if (plane.guided_state.target_heading_type == GUIDED_HEADING_HEADING) {
            error = wrap_PI(plane.guided_state.target_heading - AP::ahrs().get_yaw_rad());
        } else {
            Vector2f groundspeed = AP::ahrs().groundspeed_vector();
            error = wrap_PI(plane.guided_state.target_heading - atan2f(-groundspeed.y, -groundspeed.x) + M_PI);
        }

        float bank_limit = degrees(atanf(plane.guided_state.target_heading_accel_limit/GRAVITY_MSS)) * 1e2f;
        bank_limit = MIN(bank_limit, plane.roll_limit_cd);

        // push error into AC_PID
        const float desired = plane.g2.guidedHeading.update_error(error, delta, plane.guided_state.target_heading_limit);

        // Check for output saturation
        plane.guided_state.target_heading_limit = fabsf(desired) >= bank_limit;

        plane.nav_roll_cd = constrain_int32(desired, -bank_limit, bank_limit);
        plane.update_load_factor();

#endif // AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
    } else {
        plane.calc_nav_roll();
    }

    if (plane.guided_state.last_forced_rpy_ms.y > 0 &&
            millis() - plane.guided_state.last_forced_rpy_ms.y < plane.g2.guided_timeout*1000.0f) {
        plane.nav_pitch_cd = constrain_int32(plane.guided_state.forced_rpy_cd.y, plane.pitch_limit_min*100, plane.aparm.pitch_limit_max.get()*100);
    } else {
        plane.calc_nav_pitch();
    }

    // Throttle output
    if (plane.guided_throttle_passthru) {
        // manual passthrough of throttle in fence breach
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.get_throttle_input(true));

    }  else if (plane.aparm.throttle_cruise > 1 &&
            plane.guided_state.last_forced_throttle_ms > 0 &&
            millis() - plane.guided_state.last_forced_throttle_ms < plane.g2.guided_timeout*1000.0f) {
        // Received an external msg that guides throttle within g2.guided_timeout?
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.guided_state.forced_throttle);

    } else {
        // TECS control
        plane.calc_throttle();

    }

}

void ModeGuided::navigate()
{
#if HAL_SOARNAV_ENABLED
    if (plane.g2.soaring_controller.is_active() && does_automatic_thermal_switch()) {
        plane.nav_controller->update_waypoint(plane.current_loc, plane.next_WP_loc);
        return;
    }
#endif

    plane.update_loiter(active_radius_m);
}

bool ModeGuided::handle_guided_request(Location target_loc)
{
#if HAL_SOARNAV_ENABLED
    clear_soarnav_guided_target_context();
#endif

    plane.fix_terrain_WP(target_loc, __AP_LINE__);
    // add home alt if needed
    if (!target_loc.terrain_alt) {
        target_loc.change_alt_frame(Location::AltFrame::ABSOLUTE);
    }

    plane.set_guided_WP(target_loc);

    return true;
}

#if HAL_SOARNAV_ENABLED
bool ModeGuided::handle_soarnav_guided_request(Location target_loc, bool terrain_evasion_target)
{
    if (!soarnav_xy_only_allowed()) {
        clear_soarnav_guided_target_context();
        return false;
    }

    plane.fix_terrain_WP(target_loc, __AP_LINE__);
    if (!target_loc.terrain_alt) {
        target_loc.change_alt_frame(Location::AltFrame::ABSOLUTE);
    }

    set_soarnav_guided_target_context(target_loc, terrain_evasion_target);
    prepare_soarnav_target_altitude(target_loc);

    plane.set_guided_WP(target_loc);
    update_soarnav_energy_target_altitude();

    return true;
}


void ModeGuided::set_soarnav_guided_target_context(const Location &loc, bool terrain_evasion_target)
{
    if (!soarnav_xy_only_allowed()) {
        clear_soarnav_guided_target_context();
        return;
    }
    soarnav_horizontal_target = true;
    soarnav_terrain_corridor_target = terrain_evasion_target && plane.current_loc.initialised() && loc.initialised();
}

void ModeGuided::clear_soarnav_guided_target_context()
{
    soarnav_horizontal_target = false;
    soarnav_terrain_corridor_target = false;
    soarnav_energy_ref_valid = false;
    soarnav_energy_ref_amsl_cm = 0;
}

bool ModeGuided::soarnav_param_enabled(const char *name) const
{
    enum ap_var_type ptype = AP_PARAM_NONE;
    AP_Param *vp = AP_Param::find(name, &ptype);
    if (vp == nullptr) {
        return false;
    }

    switch (ptype) {
    case AP_PARAM_INT8:
        return ((AP_Int8 *)vp)->get() > 0;
    case AP_PARAM_INT16:
        return ((AP_Int16 *)vp)->get() > 0;
    case AP_PARAM_INT32:
        return ((AP_Int32 *)vp)->get() > 0;
    case AP_PARAM_FLOAT:
        return ((AP_Float *)vp)->get() > 0.0f;
    default:
        return false;
    }
}

bool ModeGuided::soarnav_xy_only_allowed() const
{
    if (!soarnav_param_enabled("SNAV_ENABLE") || !soarnav_param_enabled("SOAR_ENABLE")) {
        return false;
    }

    const RC_Channel *chan = rc().find_channel_for_option(RC_Channel::AUX_FUNC::SOARING);
    if (chan == nullptr) {
        return false;
    }

    return chan->get_aux_switch_pos() != RC_Channel::AuxSwitchPos::LOW;
}

void ModeGuided::update_soarnav_energy_reference()
{
    if (!plane.current_loc.initialised()) {
        return;
    }

    int32_t current_alt_cm = 0;
    if (!plane.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, current_alt_cm)) {
        return;
    }

    if (!soarnav_energy_ref_valid || current_alt_cm > soarnav_energy_ref_amsl_cm) {
        soarnav_energy_ref_amsl_cm = current_alt_cm;
        soarnav_energy_ref_valid = true;
    }
}

void ModeGuided::apply_soarnav_energy_reference(Location &loc) const
{
    if (soarnav_energy_ref_valid) {
        loc.set_alt_cm(soarnav_energy_ref_amsl_cm, Location::AltFrame::ABSOLUTE);
        return;
    }

    if (plane.current_loc.initialised()) {
        loc.copy_alt_from(plane.current_loc);
    }
}

void ModeGuided::prepare_soarnav_target_altitude(Location &loc)
{
    update_soarnav_energy_reference();
    apply_soarnav_energy_reference(loc);
}

void ModeGuided::update_soarnav_energy_target_altitude()
{
    if (!soarnav_xy_only_allowed()) {
        clear_soarnav_guided_target_context();
        return;
    }

    if (!plane.current_loc.initialised()) {
        return;
    }

    Location alt_loc = plane.current_loc;
    update_soarnav_energy_reference();
    apply_soarnav_energy_reference(alt_loc);

    plane.set_target_altitude_location(alt_loc);
    plane.reset_offset_altitude();
}

bool ModeGuided::soarnav_horizontal_target_active() const
{
    return soarnav_horizontal_target;
}

float ModeGuided::soarnav_guided_cruise_airspeed() const
{
    return plane.g2.soaring_controller.get_cruising_target_airspeed();
}
#endif

#if HAL_SOARING_ENABLED
void ModeGuided::save_soaring_target()
{
    soaring_restore.prev_loc = plane.current_loc;
    soaring_restore.target_loc = plane.next_WP_loc;
#if HAL_SOARNAV_ENABLED
    soaring_restore.horizontal_target = soarnav_horizontal_target;
    soaring_restore.terrain_corridor_target = soarnav_terrain_corridor_target;
    soaring_restore.energy_ref_valid = soarnav_energy_ref_valid;
    soaring_restore.energy_ref_amsl_cm = soarnav_energy_ref_amsl_cm;
    if (soaring_restore.horizontal_target) {
        apply_soarnav_energy_reference(soaring_restore.target_loc);
    }
#endif
    soaring_restore.radius_m = active_radius_m;
    soaring_restore.direction = plane.loiter.direction;
    soaring_restore.valid = plane.current_loc.initialised() && plane.next_WP_loc.initialised();
}

bool ModeGuided::set_soaring_target(Location target_loc, bool terrain_evasion_target)
{
    if (!soaring_restore.valid) {
        return false;
    }

    plane.fix_terrain_WP(target_loc, __AP_LINE__);
    if (!target_loc.terrain_alt) {
        target_loc.change_alt_frame(Location::AltFrame::ABSOLUTE);
    }

#if HAL_SOARNAV_ENABLED
    if (soaring_restore.horizontal_target) {
        int32_t current_alt_cm = 0;
        if (plane.current_loc.initialised() &&
            plane.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, current_alt_cm) &&
            (!soaring_restore.energy_ref_valid || current_alt_cm > soaring_restore.energy_ref_amsl_cm)) {
            soaring_restore.energy_ref_amsl_cm = current_alt_cm;
            soaring_restore.energy_ref_valid = true;
        }
        if (soaring_restore.energy_ref_valid) {
            target_loc.set_alt_cm(soaring_restore.energy_ref_amsl_cm, Location::AltFrame::ABSOLUTE);
        } else if (plane.current_loc.initialised()) {
            target_loc.copy_alt_from(plane.current_loc);
        }
        soaring_restore.terrain_corridor_target = terrain_evasion_target && plane.current_loc.initialised() && target_loc.initialised();
    } else {
        soaring_restore.terrain_corridor_target = false;
        soaring_restore.energy_ref_valid = false;
        soaring_restore.energy_ref_amsl_cm = 0;
    }
#else
    (void)terrain_evasion_target;
#endif

    soaring_restore.target_loc = target_loc;
    return true;
}

bool ModeGuided::set_external_soaring_target(Location target_loc)
{
    if (!soaring_restore.valid) {
        return false;
    }

    plane.fix_terrain_WP(target_loc, __AP_LINE__);
    if (!target_loc.terrain_alt) {
        target_loc.change_alt_frame(Location::AltFrame::ABSOLUTE);
    }

#if HAL_SOARNAV_ENABLED
    clear_soarnav_guided_target_context();
    soaring_restore.horizontal_target = false;
    soaring_restore.terrain_corridor_target = false;
    soaring_restore.energy_ref_valid = false;
    soaring_restore.energy_ref_amsl_cm = 0;
#endif
    soaring_restore.target_loc = target_loc;
    return true;
}

void ModeGuided::set_soaring_radius_and_direction(const float radius, const bool direction_is_ccw)
{
    if (!soaring_restore.valid) {
        return;
    }

    soaring_restore.radius_m = constrain_int32(fabsf(radius), 0, UINT16_MAX);
    soaring_restore.direction = direction_is_ccw ? -1 : 1;
    soaring_restore.target_loc.loiter_ccw = direction_is_ccw;
}

bool ModeGuided::get_soaring_target(Location &prev_loc, Location &target_loc) const
{
    if (!soaring_restore.valid) {
        return false;
    }

    prev_loc = soaring_restore.prev_loc;
    target_loc = soaring_restore.target_loc;
    return true;
}
#endif

#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
bool ModeGuided::handle_change_airspeed(const float airspeed, const float acceleration)
{
    // reject airspeeds that are outside of the tuning envelope
    if (airspeed > plane.aparm.airspeed_max || airspeed < plane.aparm.airspeed_min) {
        return false;
    }

    // no need to process any new packet/s with the
    // same airspeed any further, if we are already doing it.
    float new_target_airspeed_cm = airspeed * 100;
    if (is_equal(new_target_airspeed_cm,plane.guided_state.target_airspeed_cm)) { 
        return true;
    }
    plane.guided_state.target_airspeed_cm = new_target_airspeed_cm;
    plane.guided_state.target_airspeed_time_ms = AP_HAL::millis();

    if (is_zero(acceleration)) {
        // the user wanted /maximum acceleration, pick a large value as close enough
        plane.guided_state.target_airspeed_accel = 1000.0f;
    } else {
        plane.guided_state.target_airspeed_accel = fabsf(acceleration);
    }

    // assign an acceleration direction
    if (plane.guided_state.target_airspeed_cm < plane.target_airspeed_cm) {
        plane.guided_state.target_airspeed_accel *= -1.0f;
    }
    return true; 
}
#endif // AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED

void ModeGuided::set_radius_and_direction(const float radius, const bool direction_is_ccw)
{
    // constrain to (uint16_t) range for update_loiter()
    active_radius_m = constrain_int32(fabsf(radius), 0, UINT16_MAX);
    plane.loiter.direction = direction_is_ccw ? -1 : 1;
}

#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
bool Plane::GuidedState::target_location_alt_is_minus_one() const
{
    int32_t target_location_alt = 0;
    UNUSED_RESULT(target_location.get_alt_cm(target_location.get_alt_frame(), target_location_alt));
    return target_location_alt == -1;
}
#endif // AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED

void ModeGuided::update_target_altitude()
{
#if HAL_SOARNAV_ENABLED
    if (soarnav_horizontal_target) {
        update_soarnav_energy_target_altitude();
        if (soarnav_horizontal_target) {
            return;
        }
    }

#endif

#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
    // target altitude can be negative (e.g. flying below home altitude from the top of a mountain)
    if ((plane.guided_state.target_alt_time_ms != 0) ||
        !plane.guided_state.target_location_alt_is_minus_one()) { // target_alt now defaults to -1, and _time_ms defaults to zero.
        // offboard altitude demanded
        uint32_t now = AP_HAL::millis();
        float delta = 1e-3f * (now - plane.guided_state.target_alt_time_ms);
        plane.guided_state.target_alt_time_ms = now;
        // determine delta accurately as a float
        float delta_amt_f = delta * plane.guided_state.target_alt_rate;
        // then scale x100 to match last_target_alt and convert to a signed int32_t as it may be negative
        int32_t delta_amt_i = (int32_t)(100.0 * delta_amt_f); 
        // To calculate the required velocity (up or down), we need to target and current altitudes in the target frame
        const Location::AltFrame target_frame = plane.guided_state.target_location.get_alt_frame();
        int32_t target_alt_previous_cm;
        if (plane.current_loc.initialised() && plane.guided_state.target_location.initialised() && 
            plane.current_loc.get_alt_cm(target_frame, target_alt_previous_cm)) {
            // create a new interim target location that that takes current_location and moves delta_amt_i in the right direction
            int32_t temp_alt_cm = constrain_int32(plane.guided_state.target_location.alt, target_alt_previous_cm - delta_amt_i,  target_alt_previous_cm + delta_amt_i);
            Location temp_location = plane.guided_state.target_location;
            temp_location.set_alt_cm(temp_alt_cm, target_frame);

            // incrementally step the altitude towards the target            
            plane.set_target_altitude_location(temp_location);
        }
    } else 
#endif // AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
        {
        Mode::update_target_altitude();
    }
}
