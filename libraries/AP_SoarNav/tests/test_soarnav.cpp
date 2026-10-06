#include <AP_gtest.h>
#include <AP_SoarNav/AP_SoarNav.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Mission/AP_Mission.h>
#include <GCS_MAVLink/GCS_Dummy.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#if HAL_SOARNAV_ENABLED

static AP_AHRS ahrs{AP_AHRS::FLAG_ALWAYS_USE_EKF};
static GCS_Dummy gcs_dummy;

#if AP_MISSION_ENABLED
class SoarNavMissionHolder {
public:
    bool start_command(const AP_Mission::Mission_Command &) { return false; }
    bool verify_command(const AP_Mission::Mission_Command &) { return false; }
    void exit_mission() {}
    AP_Mission mission{
        FUNCTOR_BIND_MEMBER(&SoarNavMissionHolder::start_command, bool, const AP_Mission::Mission_Command &),
        FUNCTOR_BIND_MEMBER(&SoarNavMissionHolder::verify_command, bool, const AP_Mission::Mission_Command &),
        FUNCTOR_BIND_MEMBER(&SoarNavMissionHolder::exit_mission, void)};
};
static SoarNavMissionHolder mission_holder;
#endif

class AP_SoarNav_Test_Backend final : public AP_SoarNav::Backend {
public:
    Vector3f wind{};
    Vector3f velocity{};
    Vector2f terrain_slope_ne{0.0f, 0.2f};
    bool terrain_model = false;
    mutable uint32_t terrain_calls = 0;
    bool hagl_valid = false;
    bool soaring_mode_change = false;
    bool runtime = false;
    bool flying = true;
    bool switch_high = true;
    bool velocity_valid = false;
    bool airspeed_valid = true;
    bool target_accepted = true;
    float airspeed_eas = 12.0f;
    float eas2tas_ratio = 1.0f;
    Location terrain_origin{};
    Location current{-353629380, 1491650850, 12000, Location::AltFrame::ABOVE_HOME};
    Location home = current;
    Location guided_target{};
    ModeNumber mode = ModeNumber::GUIDED;
    ModeNumber previous_mode = ModeNumber::CRUISE;
    uint16_t target_count = 0;
    uint16_t param_write_count = 0;

    struct Parameter {
        const char *name;
        float value;
    };
    Parameter params[12] = {
        {"SOAR_ALT_MIN", 20.0f},
        {"SOAR_ALT_CUTOFF", 120.0f},
        {"SOAR_ALT_MAX", 500.0f},
        {"SOAR_POLAR_CD0", 0.028f},
        {"SOAR_POLAR_B", 0.031f},
        {"SOAR_POLAR_K", 200.0f},
        {"AIRSPEED_CRUISE", 12.0f},
        {"TECS_SINK_MIN", 0.3f},
        {"TECS_SINK_MAX", 3.0f},
        {"RTL_RADIUS", 50.0f},
        {"RTL_ALTITUDE", 100.0f},
        {"ROLL_LIMIT_DEG", 15.0f},
    };

    bool armed() const override { return runtime; }
    bool is_flying() const override { return runtime && flying; }
    ModeNumber mode_number() const override { return mode; }
    ModeNumber previous_mode_number() const override { return previous_mode; }
    bool mode_change_is_soaring() const override { return soaring_mode_change; }
    bool set_guided_mode() override { return set_mode(ModeNumber::GUIDED); }
    bool set_rtl_mode() override { return set_mode(ModeNumber::RTL); }
    bool set_mode(ModeNumber requested) override
    {
        if (!runtime) {
            return false;
        }
        previous_mode = mode;
        mode = requested;
        soaring_mode_change = false;
        return true;
    }
    bool set_guided_target(const Location &loc, bool) override
    {
        if (!runtime || !target_accepted) {
            return false;
        }
        guided_target = loc;
        target_count++;
        return true;
    }
    bool navigation_target(Location &loc) const override { loc = guided_target; return loc.initialised(); }
    bool current_location(Location &loc) const override { loc = current; return runtime; }
    bool home_location(Location &loc) const override { loc = home; return runtime; }
    bool relative_position_ned_home(Vector3f &ned) const override
    {
        if (!hagl_valid) {
            return false;
        }
        const Vector2f ne = home.get_distance_NE(current);
        ned = {ne.x, ne.y, (home.alt - current.alt) * 0.01f};
        return true;
    }
    bool terrain_height_amsl(const Location &loc, float &height_m) const override
    {
        terrain_calls++;
        if (!terrain_model) {
            return false;
        }
        const Vector2f ne = terrain_origin.get_distance_NE(loc);
        height_m = 100.0f + terrain_slope_ne * ne;
        return true;
    }
    bool height_above_ground_m(float &) const override { return false; }
    bool wind_vector(Vector3f &wind_ned) const override { wind_ned = wind; return wind.length() >= 0.1f; }
    bool velocity_ned(Vector3f &vel) const override { vel = velocity; return velocity_valid; }
    float ground_speed_mps() const override { return runtime ? 12.0f : 0.0f; }
    float climb_rate_mps() const override { return -velocity.z; }
    bool airspeed_estimate_mps(float &airspeed) const override { airspeed = airspeed_eas; return runtime && airspeed_valid; }
    float eas2tas() const override { return eas2tas_ratio; }
    float throttle_percent() const override { return 0.0f; }
    bool motor_running() const override { return false; }
    bool rpm_ok(float) const override { return false; }
    bool rpm_reading(float &) const override { return false; }
    float roll_input_norm() const override { return 0.0f; }
    float roll_rad() const override { return 0.0f; }
    float yaw_rad() const override { return 0.0f; }
    float pitch_input_norm() const override { return 0.0f; }
    float yaw_input_norm() const override { return 0.0f; }
    bool soar_switch_active() const override { return runtime && switch_high; }
    bool autotune_active() const override { return false; }
    bool soaring_active() const override { return runtime; }
    bool soaring_throttle_suppressed() const override { return runtime; }
    bool param_get_float(const char *name, float &value) const override
    {
        if (runtime) {
            for (const auto &param : params) {
                if (strcmp(name, param.name) == 0) {
                    value = param.value;
                    return true;
                }
            }
        }
        return false;
    }
    bool param_set_float(const char *name, float value) override
    {
        if (runtime) {
            for (auto &param : params) {
                if (strcmp(name, param.name) == 0) {
                    param.value = value;
                    param_write_count++;
                    return true;
                }
            }
        }
        return false;
    }
    uint8_t rally_count() const override { return runtime ? 4 : 0; }
    bool rally_location(uint8_t index, Location &loc) const override
    {
        if (!runtime || index >= 4) {
            return false;
        }
        loc = home;
        const float north[] = {-2000.0f, -2000.0f, 2000.0f, 2000.0f};
        const float east[] = {-2000.0f, 2000.0f, 2000.0f, -2000.0f};
        loc.offset(north[index], east[index]);
        return true;
    }
    uint32_t rally_last_change_ms() const override { return runtime ? 1 : 0; }
    void send_text(MAV_SEVERITY, const char *, ...) const override {}
};

class AP_SoarNav_Test {
public:
    static float wind_to(AP_SoarNav &nav, const Vector3f &wind)
    {
        return nav._wind_to_bearing_deg(wind);
    }

    static float wind_from(AP_SoarNav &nav, const Vector3f &wind)
    {
        return nav._wind_from_bearing_deg(wind);
    }

    static Location drift(AP_SoarNav &nav, AP_SoarNav::Backend &backend, const Location &origin, const Vector3f &wind, uint32_t age_ms)
    {
        AP_SoarNav::Hotspot hotspot{};
        hotspot.loc = origin;
        hotspot.timestamp_ms = AP_HAL::millis() - age_ms;
        hotspot.wind_ned = wind;
        hotspot.valid = true;
        Location predicted;
        nav._thermal_memory_life_s.set(1200);
        EXPECT_TRUE(nav._predict_hotspot_drift(backend, hotspot, predicted));
        return predicted;
    }

    static void set_density_case(AP_SoarNav &nav, uint32_t neighbour_age_ms)
    {
        nav.reset();
        nav._thermal_memory_life_s.set(100);
        nav._effective_cluster_radius_m = 400.0f;
        Location a{-353629380, 1491650850, 0, Location::AltFrame::ABOVE_HOME};
        Location b = a;
        b.offset_bearing(90.0f, 100.0f);
        nav._hotspots[0] = {};
        nav._hotspots[0].loc = a;
        nav._hotspots[0].timestamp_ms = AP_HAL::millis();
        nav._hotspots[0].avg_strength_mps = 2.0f;
        nav._hotspots[0].valid = true;
        nav._hotspots[1] = {};
        nav._hotspots[1].loc = b;
        nav._hotspots[1].timestamp_ms = AP_HAL::millis() - neighbour_age_ms;
        nav._hotspots[1].avg_strength_mps = 2.0f;
        nav._hotspots[1].valid = true;
        nav._update_hotspot_density();
    }

    static float density(const AP_SoarNav &nav, uint8_t index)
    {
        return nav._hotspots[index].density;
    }

    static float ridge_score(AP_SoarNav &nav, AP_SoarNav::Backend &backend, const Location &loc)
    {
        nav._grid_cell_size_m = 50.0f;
        return nav._ridge_score_at_loc(backend, loc);
    }
    static void configure(AP_SoarNav &nav, AP_SoarNav_Test_Backend &backend)
    {
        nav.reset();
        nav._enable.set(1);
        nav._auto_start.set(1);
        nav._log_level.set(0);
        nav._radius_m.set(5000);
        nav._reroute_probability_pct.set(0);
        backend.runtime = true;
        backend.home.set_alt_cm(0, Location::AltFrame::ABOVE_HOME);
        nav._center = backend.home;
        nav._center_valid = true;
        nav._grid_force_reinit = false;
        nav._init_grid(backend, backend.current, true);
        nav._state = AP_SoarNav::State::NAVIGATING;
    }

    static bool send_target(AP_SoarNav &nav, AP_SoarNav::Backend &backend, const Location &target, bool force)
    {
        return nav._send_target(backend, target, "Pure", force);
    }

    static uint32_t waypoint_start(const AP_SoarNav &nav) { return nav._waypoint_start_ms; }
    static bool waypoint_expired(AP_SoarNav &nav, AP_SoarNav_Test_Backend &backend)
    {
        nav._wp_timeout_s.set(30);
        return nav._manage_waypoint_status(backend, backend.current);
    }

    static void update_energy(AP_SoarNav &nav, AP_SoarNav_Test_Backend &backend)
    {
        nav._update_energy(backend, backend.current);
    }
    static bool energy_critical(const AP_SoarNav &nav) { return nav._energy_state == AP_SoarNav::EnergyState::CRITICAL; }

    static void thermal_exit(AP_SoarNav &nav, AP_SoarNav_Test_Backend &backend, float strength)
    {
        nav._state = AP_SoarNav::State::THERMAL_PAUSE;
        nav._thermal.active = true;
        nav._thermal.start_loc = backend.current;
        nav._thermal.best_loc = backend.current;
        nav._thermal.best_loc_valid = true;
        nav._thermal.start_ms = AP_HAL::millis() - 20000U;
        nav._thermal.accum_strength = strength;
        nav._thermal.accum_weight = 1.0f;
        nav._thermal.max_strength = strength;
        nav._filtered_alt_factor = 0.0f;
        nav.on_exit_thermal(backend);
    }

    static void polar_sample(AP_SoarNav &nav, AP_SoarNav_Test_Backend &backend)
    {
        nav._update_polar_learning(backend, backend.current);
    }
    static bool polar_learned(const AP_SoarNav &nav) { return nav._polar.learned; }

    static void glide_cone(AP_SoarNav &nav, AP_SoarNav_Test_Backend &backend, int8_t mode = 1)
    {
        nav._dynamic_soar_alt.set(mode);
        nav._store_initial_soar_alts(backend);
        nav._update_dynamic_soar_alt(backend, backend.current);
    }

    static void store_soar_alts(AP_SoarNav &nav, AP_SoarNav_Test_Backend &backend) { nav._store_initial_soar_alts(backend); }
    static void restore_soar_alts(AP_SoarNav &nav, AP_SoarNav_Test_Backend &backend) { nav._restore_initial_soar_alts(backend); }
    static float initial_soar_min(const AP_SoarNav &nav) { return nav._initial_soar_alt_min_m; }
    static float turn_time(AP_SoarNav &nav, AP_SoarNav_Test_Backend &backend, float turn_deg) { return nav._te_turn_time_s(backend, turn_deg); }

    static void prepare_rtl(AP_SoarNav &nav, AP_SoarNav_Test_Backend &backend)
    {
        configure(nav, backend);
        nav._radius_m.set(0);
        backend.current.offset(1000.0f, 0.0f);
        backend.mode = AP_SoarNav::Backend::ModeNumber::RTL;
        nav._update_area(backend, backend.current, true);
        nav._init_grid(backend, backend.current, true);
        nav._grid_force_reinit = false;
        nav._original_entry_mode = AP_SoarNav::Backend::ModeNumber::CRUISE;
        nav._guided_entry_mode = AP_SoarNav::Backend::ModeNumber::CRUISE;
        nav._target = backend.current;
        nav._target_valid = true;
        nav._last_sent_target = nav._target;
        nav._last_sent_valid = true;
        strncpy(nav._target_source, "Pure", sizeof(nav._target_source));
        nav._has_been_activated = true;
        nav._rtlh.last_mode = AP_SoarNav::Backend::ModeNumber::RTL;
        nav._rtlh.active = true;
        nav._rtlh.t0_ms = AP_HAL::millis() - 4000U;
        nav._rtlh.d0_m = backend.current.get_distance(backend.home);
    }
    static void disable(AP_SoarNav &nav) { nav._enable.set(0); }
    static void dynamic_mode(AP_SoarNav &nav, int8_t mode) { nav._dynamic_soar_alt.set(mode); }
    static bool rtl_engaged(const AP_SoarNav &nav) { return nav._rtlh.engaged_guided; }
    static bool terrain_active(const AP_SoarNav &nav) { return nav._terrain.state != AP_SoarNav::TerrainState::IDLE; }
    static float terrain_resume_distance(const AP_SoarNav &nav, const Location &home)
    {
        return nav._terrain.resume_valid ? nav._terrain.resume_target.get_distance(home) : -1.0f;
    }

    static void soaring_mode(AP_SoarNav_Test_Backend &backend, AP_SoarNav::Backend::ModeNumber mode)
    {
        backend.previous_mode = backend.mode;
        backend.mode = mode;
        backend.soaring_mode_change = true;
    }

    static void anti_stuck_resume(AP_SoarNav &nav, AP_SoarNav_Test_Backend &backend)
    {
        nav._target = backend.current;
        nav._target_valid = true;
        strncpy(nav._target_source, "Anti-Stuck", sizeof(nav._target_source));
        nav._is_repositioning = true;
        nav._reposition_resume_target = backend.current;
        nav._reposition_resume_target.offset(400.0f, 0.0f);
        nav._reposition_resume_valid = true;
        strncpy(nav._reposition_resume_source, "Pure", sizeof(nav._reposition_resume_source));
        nav._handle_navigating(backend, backend.current, true);
    }
    static const char *source(const AP_SoarNav &nav) { return nav._target_source; }

};

static float bearing_error_deg(float a, float b)
{
    float d = fmodf(a - b + 540.0f, 360.0f) - 180.0f;
    return fabsf(d);
}

TEST(AP_SoarNav, WindCardinalBearings)
{
    AP_SoarNav nav;
    struct WindCase {
        Vector3f wind;
        float to_deg;
        float from_deg;
    };
    const WindCase cases[] = {
        {Vector3f{1.0f, 0.0f, 0.0f}, 0.0f, 180.0f},
        {Vector3f{0.0f, 1.0f, 0.0f}, 90.0f, 270.0f},
        {Vector3f{-1.0f, 0.0f, 0.0f}, 180.0f, 0.0f},
        {Vector3f{0.0f, -1.0f, 0.0f}, 270.0f, 90.0f},
        {Vector3f{1.0f, 1.0f, 0.0f}, 45.0f, 225.0f},
    };
    for (const auto &test : cases) {
        EXPECT_NEAR(AP_SoarNav_Test::wind_to(nav, test.wind), test.to_deg, 1.0e-4f);
        EXPECT_NEAR(AP_SoarNav_Test::wind_from(nav, test.wind), test.from_deg, 1.0e-4f);
    }
}

TEST(AP_SoarNav, ThermalDriftFollowsWindTo)
{
    AP_SoarNav nav;
    AP_SoarNav_Test_Backend backend;
    const Location origin{-353629380, 1491650850, 0, Location::AltFrame::ABOVE_HOME};
    struct DriftCase {
        Vector3f wind;
        float bearing_deg;
    };
    const DriftCase cases[] = {
        {Vector3f{8.0f, 0.0f, 0.0f}, 0.0f},
        {Vector3f{0.0f, 8.0f, 0.0f}, 90.0f},
        {Vector3f{-8.0f, 0.0f, 0.0f}, 180.0f},
        {Vector3f{0.0f, -8.0f, 0.0f}, 270.0f},
    };
    for (const auto &test : cases) {
        const Location predicted = AP_SoarNav_Test::drift(nav, backend, origin, test.wind, 10000U);
        const float bearing = degrees(origin.get_bearing(predicted));
        EXPECT_LT(bearing_error_deg(bearing, test.bearing_deg), 1.0f);
        EXPECT_GT(origin.get_distance(predicted), 20.0f);
    }
}

TEST(AP_SoarNav, RidgeScoreUsesEastNorthWindFrame)
{
    AP_SoarNav nav;
    AP_SoarNav_Test_Backend backend;
    const Location origin{-353629380, 1491650850, 0, Location::AltFrame::ABOVE_HOME};
    backend.wind = Vector3f{0.0f, 8.0f, 0.0f};
    backend.terrain_model = true;
    backend.terrain_origin = origin;
    EXPECT_GT(AP_SoarNav_Test::ridge_score(nav, backend, origin), 0.8f);
}

TEST(AP_SoarNav, ThermalDensityDecaysWithAge)
{
    AP_SoarNav nav;
    AP_SoarNav_Test::set_density_case(nav, 0U);
    const float fresh = AP_SoarNav_Test::density(nav, 0);
    AP_SoarNav_Test::set_density_case(nav, 50000U);
    const float half_life = AP_SoarNav_Test::density(nav, 0);
    AP_SoarNav_Test::set_density_case(nav, 90000U);
    const float old = AP_SoarNav_Test::density(nav, 0);
    EXPECT_GT(fresh, half_life);
    EXPECT_GT(half_life, old);
    EXPECT_NEAR(half_life, fresh * 0.5f, 0.02f);
    EXPECT_NEAR(old, fresh * 0.1f, 0.02f);
}

static void set_time_ms(uint32_t now_ms)
{
    static uint64_t offset_us = 0;
    const uint64_t requested_us = uint64_t(now_ms) * 1000U;
    const uint64_t current_us = AP_HAL::micros64();
    if (offset_us + requested_us < current_us) {
        offset_us = current_us;
    }
    hal.scheduler->stop_clock(offset_us + requested_us);
}

TEST(AP_SoarNav, TestClockPreservesElapsedTimeAcrossRestarts)
{
    set_time_ms(10000U);
    const uint32_t first_ms = AP_HAL::millis();
    set_time_ms(11000U);
    const uint32_t advanced_ms = AP_HAL::millis();
    EXPECT_EQ(advanced_ms - first_ms, 1000U);
    set_time_ms(10000U);
    const uint32_t restarted_ms = AP_HAL::millis();
    EXPECT_GE(restarted_ms, advanced_ms);
    set_time_ms(10100U);
    EXPECT_EQ(AP_HAL::millis() - restarted_ms, 100U);
}

TEST(AP_SoarNav, HorizontalTargetRefreshPreservesWaypointClock)
{
    set_time_ms(10000U);
    const uint32_t start_ms = AP_HAL::millis();
    AP_SoarNav nav;
    AP_SoarNav_Test_Backend backend;
    AP_SoarNav_Test::configure(nav, backend);
    Location target = backend.current;
    target.offset(1000.0f, 0.0f);
    ASSERT_TRUE(AP_SoarNav_Test::send_target(nav, backend, target, true));
    set_time_ms(21000U);
    backend.current.alt -= 1000;
    ASSERT_TRUE(AP_SoarNav_Test::send_target(nav, backend, target, false));
    EXPECT_EQ(AP_SoarNav_Test::waypoint_start(nav), start_ms);
    EXPECT_EQ(backend.target_count, 1U);
    set_time_ms(42000U);
    backend.current.alt -= 1000;
    ASSERT_TRUE(AP_SoarNav_Test::send_target(nav, backend, target, false));
    EXPECT_TRUE(AP_SoarNav_Test::waypoint_expired(nav, backend));
}

TEST(AP_SoarNav, SustainedDescentDetectedAtTenHz)
{
    AP_SoarNav nav;
    AP_SoarNav_Test_Backend backend;
    AP_SoarNav_Test::configure(nav, backend);
    for (uint32_t i = 0; i <= 45; i++) {
        set_time_ms(10000U + i * 100U);
        backend.current.alt = 30000 - i * 10;
        AP_SoarNav_Test::update_energy(nav, backend);
    }
    EXPECT_TRUE(AP_SoarNav_Test::energy_critical(nav));
    for (uint32_t i = 46; i <= 55; i++) {
        set_time_ms(10000U + i * 100U);
        AP_SoarNav_Test::update_energy(nav, backend);
    }
    EXPECT_FALSE(AP_SoarNav_Test::energy_critical(nav));
}

TEST(AP_SoarNav, ThermalExitDoesNotUndoPilotModeChange)
{
    for (const float strength : {0.1f, 1.0f}) {
        for (const auto mode : {AP_SoarNav::Backend::ModeNumber::CRUISE, AP_SoarNav::Backend::ModeNumber::FBWB}) {
            set_time_ms(30000U);
            AP_SoarNav nav;
            AP_SoarNav_Test_Backend backend;
            AP_SoarNav_Test::configure(nav, backend);
            backend.mode = mode;
            AP_SoarNav_Test::thermal_exit(nav, backend, strength);
            EXPECT_EQ(backend.mode, mode);
            EXPECT_EQ(backend.target_count, 0U);
            EXPECT_FALSE(nav.has_target());
        }
    }
}

TEST(AP_SoarNav, RidgeLiftRequiresUphillAirflow)
{
    AP_SoarNav nav;
    AP_SoarNav_Test_Backend backend;
    backend.terrain_model = true;
    backend.terrain_origin = backend.current;
    const Vector3f winds[] = {{8, 0, 0}, {0, 8, 0}, {-8, 0, 0}, {0, -8, 0}};
    for (const auto &wind : winds) {
        backend.terrain_slope_ne = wind.xy() * 0.025f;
        backend.wind = wind;
        EXPECT_GT(AP_SoarNav_Test::ridge_score(nav, backend, backend.current), 0.8f);
        backend.wind = -wind;
        EXPECT_NEAR(AP_SoarNav_Test::ridge_score(nav, backend, backend.current), 0.0f, 0.001f);
        backend.wind = Vector3f{-wind.y, wind.x, 0};
        EXPECT_NEAR(AP_SoarNav_Test::ridge_score(nav, backend, backend.current), 0.0f, 0.001f);
    }
}

TEST(AP_SoarNav, PolarLearningRequiresAirspeedVariation)
{
    AP_SoarNav nav;
    AP_SoarNav_Test_Backend backend;
    AP_SoarNav_Test::configure(nav, backend);
    backend.velocity_valid = true;
    const float v = 12.0f;
    backend.velocity = Vector3f{v, 0, 0.028f / 200.0f * v * v * v + 0.031f * 200.0f / v};
    for (uint32_t i = 0; i < 400; i++) {
        set_time_ms(10000U + i * 2000U);
        AP_SoarNav_Test::polar_sample(nav, backend);
    }
    EXPECT_FALSE(AP_SoarNav_Test::polar_learned(nav));
    EXPECT_EQ(backend.param_write_count, 0U);
}

TEST(AP_SoarNav, PolarLearningUsesNativePolarK)
{
    const float speeds[] = {10, 12, 14, 16};
    const float cd0 = 0.028f * 1.05f;
    const float b = 0.031f * 1.07f;
    for (const float polar_k : {25.6f, 200.0f, 320.0f}) {
        for (const bool airspeed_valid : {false, true}) {
            AP_SoarNav nav;
            AP_SoarNav_Test_Backend backend;
            AP_SoarNav_Test::configure(nav, backend);
            backend.velocity_valid = true;
            backend.airspeed_valid = airspeed_valid;
            backend.eas2tas_ratio = 1.2f;
            backend.wind = Vector3f{2.0f, 1.0f, 0.0f};
            ASSERT_TRUE(backend.param_set_float("SOAR_POLAR_K", polar_k));
            for (uint32_t i = 0; i < 600 && !AP_SoarNav_Test::polar_learned(nav); i++) {
                set_time_ms(10000U + i * 2000U);
                const float v = speeds[i % ARRAY_SIZE(speeds)];
                backend.airspeed_eas = v;
                backend.velocity = Vector3f{v * backend.eas2tas_ratio + backend.wind.x, backend.wind.y,
                                           (cd0 / polar_k * v * v * v + b * polar_k / v) * backend.eas2tas_ratio};
                AP_SoarNav_Test::polar_sample(nav, backend);
            }
            EXPECT_TRUE(AP_SoarNav_Test::polar_learned(nav));
            float actual_cd0 = 0;
            float actual_b = 0;
            ASSERT_TRUE(backend.param_get_float("SOAR_POLAR_CD0", actual_cd0));
            ASSERT_TRUE(backend.param_get_float("SOAR_POLAR_B", actual_b));
            EXPECT_NEAR(actual_cd0, cd0, 0.0002f);
            EXPECT_NEAR(actual_b, b, 0.0002f);
        }
    }
}

TEST(AP_SoarNav, GlideConeIncludesCrosswindAndNativeSink)
{
    const Vector3f winds[] = {{0, 0, 0}, {0, 10, 0}, {5, 0, 0}, {-5, 0, 0}, {0, 15, 0}};
    for (const float ratio : {1.0f, 1.2f}) {
        for (const auto &wind : winds) {
            set_time_ms(10000U);
            AP_SoarNav nav;
            AP_SoarNav_Test_Backend backend;
            AP_SoarNav_Test::configure(nav, backend);
            backend.current.set_alt_cm(5000, Location::AltFrame::ABOVE_HOME);
            backend.current.offset(1000.0f, 0.0f);
            backend.wind = wind;
            backend.eas2tas_ratio = ratio;
            AP_SoarNav_Test::glide_cone(nav, backend);
            const float sink = (0.028f / 200.0f * 12.0f * 12.0f * 12.0f + 0.031f * 200.0f / 12.0f) * ratio;
            const float tas = 12.0f * ratio;
            const float speed_home = fabsf(wind.y) < tas ? MAX(0.1f, sqrtf(tas * tas - wind.y * wind.y) - wind.x) : 0.1f;
            const float required = backend.current.get_distance(backend.home) * sink / speed_home + 45.0f;
            const float expected = floorf(floorf(required) / 10.0f + 0.5f) * 10.0f;
            float actual = 0;
            ASSERT_TRUE(backend.param_get_float("SOAR_ALT_MIN", actual));
            EXPECT_NEAR(actual, expected, 1.0f);
        }
    }
}

TEST(AP_SoarNav, RTLHelperOwnsItsGuidedRoute)
{
    set_time_ms(30000U);
    AP_SoarNav nav;
    AP_SoarNav_Test_Backend backend;
    AP_SoarNav_Test::prepare_rtl(nav, backend);
    nav.update(backend);
    EXPECT_EQ(backend.mode, AP_SoarNav::Backend::ModeNumber::GUIDED);
    EXPECT_EQ(backend.target_count, 1U);
    EXPECT_LT(backend.guided_target.get_distance(backend.home), 1.0f);
}

TEST(AP_SoarNav, DisablingRTLHelperRestoresRTL)
{
    set_time_ms(30000U);
    AP_SoarNav nav;
    AP_SoarNav_Test_Backend backend;
    AP_SoarNav_Test::prepare_rtl(nav, backend);
    nav.update(backend);
    AP_SoarNav_Test::disable(nav);
    set_time_ms(31000U);
    nav.update(backend);
    EXPECT_EQ(backend.mode, AP_SoarNav::Backend::ModeNumber::RTL);
}

TEST(AP_SoarNav, AntiStuckResumeKeepsTargetSource)
{
    set_time_ms(30000U);
    AP_SoarNav nav;
    AP_SoarNav_Test_Backend backend;
    AP_SoarNav_Test::configure(nav, backend);
    AP_SoarNav_Test::anti_stuck_resume(nav, backend);
    EXPECT_EQ(backend.target_count, 1U);
    EXPECT_STREQ(AP_SoarNav_Test::source(nav), "Pure");
}

TEST(AP_SoarNav, TerrainTurnUsesCurrentRollLimitParameter)
{
    AP_SoarNav nav;
    AP_SoarNav_Test_Backend backend;
    AP_SoarNav_Test::configure(nav, backend);
    for (const float bank_deg : {5.0f, 15.0f, 45.0f}) {
        ASSERT_TRUE(backend.param_set_float("ROLL_LIMIT_DEG", bank_deg));
        const float expected = radians(90.0f) * 12.0f / (GRAVITY_MSS * tanf(radians(bank_deg)));
        EXPECT_NEAR(AP_SoarNav_Test::turn_time(nav, backend, 90.0f), expected, 0.01f);
    }
}

TEST(AP_SoarNav, UntouchedSoaringLimitsAreNotRestored)
{
    AP_SoarNav nav;
    AP_SoarNav_Test_Backend backend;
    AP_SoarNav_Test::configure(nav, backend);
    AP_SoarNav_Test::store_soar_alts(nav, backend);
    ASSERT_TRUE(backend.param_set_float("SOAR_ALT_MIN", 65.0f));
    backend.param_write_count = 0;
    AP_SoarNav_Test::restore_soar_alts(nav, backend);
    float alt_min = 0.0f;
    ASSERT_TRUE(backend.param_get_float("SOAR_ALT_MIN", alt_min));
    EXPECT_FLOAT_EQ(alt_min, 65.0f);
    EXPECT_EQ(backend.param_write_count, 0U);
}

TEST(AP_SoarNav, GlideConeRestoresOnlyChangedLimits)
{
    set_time_ms(30000U);
    AP_SoarNav nav;
    AP_SoarNav_Test_Backend backend;
    AP_SoarNav_Test::configure(nav, backend);
    backend.current.offset(1000.0f, 0.0f);
    backend.current.set_alt_cm(5000, Location::AltFrame::ABOVE_HOME);
    AP_SoarNav_Test::glide_cone(nav, backend, 2);
    ASSERT_EQ(backend.param_write_count, 1U);
    ASSERT_TRUE(backend.param_set_float("SOAR_ALT_CUTOFF", 140.0f));
    ASSERT_TRUE(backend.param_set_float("SOAR_ALT_MAX", 650.0f));
    AP_SoarNav_Test::restore_soar_alts(nav, backend);
    float value = 0.0f;
    ASSERT_TRUE(backend.param_get_float("SOAR_ALT_MIN", value));
    EXPECT_FLOAT_EQ(value, 20.0f);
    ASSERT_TRUE(backend.param_get_float("SOAR_ALT_CUTOFF", value));
    EXPECT_FLOAT_EQ(value, 140.0f);
    ASSERT_TRUE(backend.param_get_float("SOAR_ALT_MAX", value));
    EXPECT_FLOAT_EQ(value, 650.0f);
    ASSERT_TRUE(backend.param_set_float("SOAR_ALT_MIN", 30.0f));
    AP_SoarNav_Test::store_soar_alts(nav, backend);
    EXPECT_FLOAT_EQ(AP_SoarNav_Test::initial_soar_min(nav), 30.0f);
}

TEST(AP_SoarNav, ThermalRefreshPreservesWaypointClock)
{
    set_time_ms(10000U);
    const uint32_t start_ms = AP_HAL::millis();
    AP_SoarNav nav;
    AP_SoarNav_Test_Backend backend;
    AP_SoarNav_Test::configure(nav, backend);
    Location target = backend.current;
    target.offset(500.0f, 0.0f);
    ASSERT_TRUE(AP_SoarNav_Test::send_target(nav, backend, target, true));
    backend.mode = AP_SoarNav::Backend::ModeNumber::THERMAL;
    set_time_ms(15000U);
    ASSERT_TRUE(AP_SoarNav_Test::send_target(nav, backend, target, true));
    EXPECT_EQ(backend.target_count, 2U);
    EXPECT_EQ(AP_SoarNav_Test::waypoint_start(nav), start_ms);
}

TEST(AP_SoarNav, ThermalExitInGuidedCanReengage)
{
    set_time_ms(30000U);
    AP_SoarNav nav;
    AP_SoarNav_Test_Backend backend;
    AP_SoarNav_Test::configure(nav, backend);
    AP_SoarNav_Test::thermal_exit(nav, backend, 0.1f);
    EXPECT_EQ(backend.mode, AP_SoarNav::Backend::ModeNumber::GUIDED);
    EXPECT_EQ(backend.target_count, 1U);
    EXPECT_TRUE(nav.has_target());
}

TEST(AP_SoarNav, RTLTargetFailureRestoresRTL)
{
    set_time_ms(30000U);
    AP_SoarNav nav;
    AP_SoarNav_Test_Backend backend;
    AP_SoarNav_Test::prepare_rtl(nav, backend);
    backend.target_accepted = false;
    nav.update(backend);
    EXPECT_EQ(backend.mode, AP_SoarNav::Backend::ModeNumber::RTL);
    EXPECT_EQ(backend.target_count, 0U);
}

TEST(AP_SoarNav, StoppingRTLHelperPreservesPilotMode)
{
    set_time_ms(30000U);
    AP_SoarNav nav;
    AP_SoarNav_Test_Backend backend;
    AP_SoarNav_Test::prepare_rtl(nav, backend);
    nav.update(backend);
    ASSERT_EQ(backend.mode, AP_SoarNav::Backend::ModeNumber::GUIDED);
    backend.mode = AP_SoarNav::Backend::ModeNumber::CRUISE;
    nav.stop(backend);
    EXPECT_EQ(backend.mode, AP_SoarNav::Backend::ModeNumber::CRUISE);
}

TEST(AP_SoarNav, SwitchingOffRTLHelperRestoresRTL)
{
    set_time_ms(30000U);
    AP_SoarNav nav;
    AP_SoarNav_Test_Backend backend;
    AP_SoarNav_Test::prepare_rtl(nav, backend);
    nav.update(backend);
    ASSERT_EQ(backend.mode, AP_SoarNav::Backend::ModeNumber::GUIDED);
    backend.switch_high = false;
    set_time_ms(31000U);
    nav.update(backend);
    EXPECT_EQ(backend.mode, AP_SoarNav::Backend::ModeNumber::RTL);
}

TEST(AP_SoarNav, RTLHelperWaitsForFlight)
{
    set_time_ms(30000U);
    AP_SoarNav nav;
    AP_SoarNav_Test_Backend backend;
    AP_SoarNav_Test::prepare_rtl(nav, backend);
    backend.flying = false;
    nav.update(backend);
    EXPECT_EQ(backend.mode, AP_SoarNav::Backend::ModeNumber::RTL);
    EXPECT_EQ(backend.target_count, 0U);
    backend.flying = true;
    set_time_ms(31000U);
    nav.update(backend);
    EXPECT_EQ(backend.mode, AP_SoarNav::Backend::ModeNumber::RTL);
    set_time_ms(35000U);
    nav.update(backend);
    EXPECT_EQ(backend.mode, AP_SoarNav::Backend::ModeNumber::GUIDED);
    EXPECT_EQ(backend.target_count, 1U);
}

TEST(AP_SoarNav, EnteringRTLRestoresGlideConeLimits)
{
    set_time_ms(30000U);
    AP_SoarNav nav;
    AP_SoarNav_Test_Backend backend;
    AP_SoarNav_Test::prepare_rtl(nav, backend);
    backend.current.set_alt_cm(5000, Location::AltFrame::ABOVE_HOME);
    AP_SoarNav_Test::glide_cone(nav, backend, 2);
    nav.update(backend);
    float alt_min = 0.0f;
    ASSERT_TRUE(backend.param_get_float("SOAR_ALT_MIN", alt_min));
    EXPECT_FLOAT_EQ(alt_min, 20.0f);
    EXPECT_EQ(backend.target_count, 1U);
    EXPECT_TRUE(nav.has_target());
    EXPECT_STREQ(AP_SoarNav_Test::source(nav), "RTL Home");
    EXPECT_LT(backend.guided_target.get_distance(backend.home), 1.0f);
}

TEST(AP_SoarNav, RTLHelperSurvivesAutomaticThermalDetour)
{
    set_time_ms(30000U);
    AP_SoarNav nav;
    AP_SoarNav_Test_Backend backend;
    AP_SoarNav_Test::prepare_rtl(nav, backend);
    nav.update(backend);
    ASSERT_TRUE(AP_SoarNav_Test::rtl_engaged(nav));
    AP_SoarNav_Test::soaring_mode(backend, AP_SoarNav::Backend::ModeNumber::THERMAL);
    set_time_ms(31000U);
    nav.update(backend);
    EXPECT_TRUE(AP_SoarNav_Test::rtl_engaged(nav));
    EXPECT_EQ(backend.mode, AP_SoarNav::Backend::ModeNumber::THERMAL);
    AP_SoarNav_Test::soaring_mode(backend, AP_SoarNav::Backend::ModeNumber::GUIDED);
    set_time_ms(32000U);
    nav.update(backend);
    EXPECT_TRUE(AP_SoarNav_Test::rtl_engaged(nav));
    EXPECT_LT(backend.guided_target.get_distance(backend.home), 1.0f);
    backend.current = backend.home;
    backend.current.set_alt_cm(12000, Location::AltFrame::ABOVE_HOME);
    set_time_ms(33000U);
    nav.update(backend);
    EXPECT_EQ(backend.mode, AP_SoarNav::Backend::ModeNumber::RTL);
    EXPECT_FALSE(AP_SoarNav_Test::rtl_engaged(nav));
}

TEST(AP_SoarNav, RTLHelperRespectsManualThermalChanges)
{
    for (const bool manual_exit : {false, true}) {
        set_time_ms(30000U);
        AP_SoarNav nav;
        AP_SoarNav_Test_Backend backend;
        AP_SoarNav_Test::prepare_rtl(nav, backend);
        nav.update(backend);
        ASSERT_TRUE(AP_SoarNav_Test::rtl_engaged(nav));
        AP_SoarNav_Test::soaring_mode(backend, AP_SoarNav::Backend::ModeNumber::THERMAL);
        if (manual_exit) {
            set_time_ms(31000U);
            nav.update(backend);
            ASSERT_TRUE(AP_SoarNav_Test::rtl_engaged(nav));
            backend.previous_mode = backend.mode;
            backend.mode = AP_SoarNav::Backend::ModeNumber::GUIDED;
        }
        backend.soaring_mode_change = false;
        const auto pilot_mode = backend.mode;
        const uint16_t targets = backend.target_count;
        set_time_ms(32000U);
        nav.update(backend);
        EXPECT_EQ(backend.mode, pilot_mode);
        EXPECT_EQ(backend.target_count, targets);
        EXPECT_FALSE(AP_SoarNav_Test::rtl_engaged(nav));
        EXPECT_FALSE(nav.has_target());
    }
}

TEST(AP_SoarNav, RTLHelperDisablingDuringAutomaticThermalRestoresRTL)
{
    for (uint8_t action = 0; action < 3; action++) {
        set_time_ms(30000U);
        AP_SoarNav nav;
        AP_SoarNav_Test_Backend backend;
        AP_SoarNav_Test::prepare_rtl(nav, backend);
        nav.update(backend);
        AP_SoarNav_Test::soaring_mode(backend, AP_SoarNav::Backend::ModeNumber::THERMAL);
        set_time_ms(31000U);
        nav.update(backend);
        ASSERT_TRUE(AP_SoarNav_Test::rtl_engaged(nav));
        if (action == 0) {
            nav.stop(backend);
        } else {
            if (action == 1) {
                AP_SoarNav_Test::disable(nav);
            } else {
                backend.switch_high = false;
            }
            set_time_ms(32000U);
            nav.update(backend);
        }
        EXPECT_EQ(backend.mode, AP_SoarNav::Backend::ModeNumber::RTL);
        EXPECT_FALSE(AP_SoarNav_Test::rtl_engaged(nav));
    }
}

TEST(AP_SoarNav, NoWindGlideConeIsIndependentOfDensityAltitude)
{
    float reference = 0.0f;
    bool have_reference = false;
    for (const float ratio : {1.0f, 1.2f, 1.5f}) {
        set_time_ms(10000U);
        AP_SoarNav nav;
        AP_SoarNav_Test_Backend backend;
        AP_SoarNav_Test::configure(nav, backend);
        backend.current.offset(2000.0f, 0.0f);
        backend.current.set_alt_cm(5000, Location::AltFrame::ABOVE_HOME);
        backend.eas2tas_ratio = ratio;
        AP_SoarNav_Test::glide_cone(nav, backend);
        float limit = 0.0f;
        ASSERT_TRUE(backend.param_get_float("SOAR_ALT_MIN", limit));
        if (!have_reference) {
            reference = limit;
            have_reference = true;
        }
        EXPECT_FLOAT_EQ(limit, reference);
    }
}

TEST(AP_SoarNav, RTLHelperTerrainEvasionResumesHome)
{
    for (uint8_t scenario = 0; scenario < 4; scenario++) {
        const bool thermal = (scenario & 1U) != 0;
        const bool disable_evasion = (scenario & 2U) != 0;
        set_time_ms(30000U);
        AP_SoarNav nav;
        AP_SoarNav_Test_Backend backend;
        AP_SoarNav_Test::prepare_rtl(nav, backend);
        backend.current.set_alt_cm(12000, Location::AltFrame::ABSOLUTE);
        backend.home.set_alt_cm(10000, Location::AltFrame::ABSOLUTE);
        ASSERT_TRUE(ahrs.set_home(backend.home));
        backend.terrain_origin = backend.current;
        backend.terrain_slope_ne = {0.0f, 0.0f};
        backend.terrain_model = true;
        backend.hagl_valid = true;
        AP_SoarNav_Test::dynamic_mode(nav, 3);
        nav.update(backend);
        ASSERT_TRUE(AP_SoarNav_Test::rtl_engaged(nav));
        if (thermal) {
            AP_SoarNav_Test::soaring_mode(backend, AP_SoarNav::Backend::ModeNumber::THERMAL);
        }
        backend.terrain_calls = 0;
        set_time_ms(31000U);
        nav.update(backend);
        EXPECT_GT(backend.terrain_calls, 0U);
        ASSERT_TRUE(AP_SoarNav_Test::terrain_active(nav));
        ASSERT_STREQ(AP_SoarNav_Test::source(nav), "Terrain Evasion");
        EXPECT_EQ(backend.mode, AP_SoarNav::Backend::ModeNumber::GUIDED);
        EXPECT_TRUE(AP_SoarNav_Test::rtl_engaged(nav));
        EXPECT_NEAR(AP_SoarNav_Test::terrain_resume_distance(nav, backend.home), 0.0f, 1.0f);
        if (disable_evasion) {
            AP_SoarNav_Test::dynamic_mode(nav, 0);
        } else {
            backend.current.set_alt_cm(50000, Location::AltFrame::ABSOLUTE);
        }
        for (uint32_t i = 0; i < 60 && AP_SoarNav_Test::terrain_active(nav); i++) {
            set_time_ms(32000U + i * 1000U);
            nav.update(backend);
        }
        ASSERT_FALSE(AP_SoarNav_Test::terrain_active(nav));
        set_time_ms(93000U);
        nav.update(backend);
        EXPECT_STREQ(AP_SoarNav_Test::source(nav), "RTL Home");
        EXPECT_LT(backend.guided_target.get_distance(backend.home), 1.0f);
        EXPECT_TRUE(AP_SoarNav_Test::rtl_engaged(nav));
    }
}

#endif

AP_GTEST_MAIN()
