#include <AP_gtest.h>
#include <AP_SoarNav/AP_SoarNav.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#if HAL_SOARNAV_ENABLED

class AP_SoarNav_Test_Backend final : public AP_SoarNav::Backend {
public:
    Vector3f wind{};
    bool terrain_model = false;
    Location terrain_origin{};

    bool armed() const override { return false; }
    bool is_flying() const override { return false; }
    ModeNumber mode_number() const override { return ModeNumber::UNKNOWN; }
    ModeNumber previous_mode_number() const override { return ModeNumber::UNKNOWN; }
    bool set_guided_mode() override { return false; }
    bool set_rtl_mode() override { return false; }
    bool set_mode(ModeNumber) override { return false; }
    bool set_guided_target(const Location &, bool) override { return false; }
    bool navigation_target(Location &) const override { return false; }
    bool current_location(Location &) const override { return false; }
    bool home_location(Location &) const override { return false; }
    bool relative_position_ned_home(Vector3f &) const override { return false; }
    bool terrain_height_amsl(const Location &loc, float &height_m) const override
    {
        if (!terrain_model) {
            return false;
        }
        const Vector2f ne = terrain_origin.get_distance_NE(loc);
        height_m = 100.0f - 0.2f * ne.y;
        return true;
    }
    bool height_above_ground_m(float &) const override { return false; }
    bool wind_vector(Vector3f &wind_ned) const override { wind_ned = wind; return wind.length() >= 0.1f; }
    bool velocity_ned(Vector3f &) const override { return false; }
    float ground_speed_mps() const override { return 0.0f; }
    float climb_rate_mps() const override { return 0.0f; }
    bool airspeed_estimate_mps(float &) const override { return false; }
    float throttle_percent() const override { return 0.0f; }
    bool motor_running() const override { return false; }
    bool rpm_ok(float) const override { return false; }
    bool rpm_reading(float &) const override { return false; }
    float roll_input_norm() const override { return 0.0f; }
    float roll_rad() const override { return 0.0f; }
    float yaw_rad() const override { return 0.0f; }
    float pitch_input_norm() const override { return 0.0f; }
    float yaw_input_norm() const override { return 0.0f; }
    bool soar_switch_active() const override { return false; }
    bool autotune_active() const override { return false; }
    bool soaring_active() const override { return false; }
    bool soaring_throttle_suppressed() const override { return false; }
    bool param_get_float(const char *, float &) const override { return false; }
    bool param_set_float(const char *, float) override { return false; }
    uint8_t rally_count() const override { return 0; }
    bool rally_location(uint8_t, Location &) const override { return false; }
    uint32_t rally_last_change_ms() const override { return 0; }
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

#endif

AP_GTEST_MAIN()
