#include <catch2/catch_test_macros.hpp>

#include "VehicleLights.h"

#include <set>
#include <string>

// ---------------------------------------------------------------------------
// Static metadata
// ---------------------------------------------------------------------------

TEST_CASE("LightID enum has 19 bulbs", "[VehicleLights]") {
    CHECK(NUM_LIGHTS == 19);
}

TEST_CASE("LightIDName returns a unique non-empty string for every LightID",
          "[VehicleLights]") {
    std::set<std::string> names;
    for (int i = 0; i < NUM_LIGHTS; ++i) {
        const char* name = LightIDName(static_cast<LightID>(i));
        REQUIRE(name != nullptr);
        CHECK(std::string(name).size() > 0);
        CHECK(std::string(name) != "???");
        CHECK(names.insert(name).second);
    }
}

// ---------------------------------------------------------------------------
// State round-trip
// ---------------------------------------------------------------------------

TEST_CASE("New VehicleLights starts with all bulbs off and uninitialised",
          "[VehicleLights]") {
    VehicleLights v;
    CHECK_FALSE(v.IsInitialized());
    CHECK(v.GetChassisNode() == nullptr);
    for (int i = 0; i < NUM_LIGHTS; ++i)
        CHECK_FALSE(v.GetState(static_cast<LightID>(i)));
}

TEST_CASE("SetState/GetState round-trip independently for every LightID",
          "[VehicleLights]") {
    VehicleLights v;

    // Set one, confirm only that one is on.
    v.SetState(LightID::CHMSL, true);
    for (int i = 0; i < NUM_LIGHTS; ++i) {
        bool expected = (static_cast<LightID>(i) == LightID::CHMSL);
        CHECK(v.GetState(static_cast<LightID>(i)) == expected);
    }

    // Set all, confirm all on.
    for (int i = 0; i < NUM_LIGHTS; ++i)
        v.SetState(static_cast<LightID>(i), true);
    for (int i = 0; i < NUM_LIGHTS; ++i)
        CHECK(v.GetState(static_cast<LightID>(i)));

    // Clear one, rest stay on.
    v.SetState(LightID::LHBH, false);
    CHECK_FALSE(v.GetState(LightID::LHBH));
    CHECK(v.GetState(LightID::RHBH));
}

// ---------------------------------------------------------------------------
// Demo mode — every bulb must blink at its own frequency.
// ---------------------------------------------------------------------------

TEST_CASE("UpdateDemoMode toggles every bulb at least once in 10 s",
          "[VehicleLights]") {
    VehicleLights v;
    bool saw_on[NUM_LIGHTS]  = {};
    bool saw_off[NUM_LIGHTS] = {};

    // Sample at 100 Hz for 10 s — well above the slowest bulb (~0.5 Hz).
    for (int step = 0; step < 1000; ++step) {
        double t = step * 0.01;
        v.UpdateDemoMode(t);
        for (int i = 0; i < NUM_LIGHTS; ++i) {
            if (v.GetState(static_cast<LightID>(i))) saw_on[i]  = true;
            else                                      saw_off[i] = true;
        }
    }

    for (int i = 0; i < NUM_LIGHTS; ++i) {
        INFO("LightID " << LightIDName(static_cast<LightID>(i)));
        CHECK(saw_on[i]);
        CHECK(saw_off[i]);
    }
}

TEST_CASE("UpdateDemoMode at t=0 lights every bulb (start of period)",
          "[VehicleLights]") {
    VehicleLights v;
    v.UpdateDemoMode(0.0);
    // phase=0, period*0.5 > 0, so all are on.
    for (int i = 0; i < NUM_LIGHTS; ++i)
        CHECK(v.GetState(static_cast<LightID>(i)));
}

// ---------------------------------------------------------------------------
// Chase demo — one bulb at a time, deterministic order starting at CHMSL.
// ---------------------------------------------------------------------------

static int CountBulbsOn(const VehicleLights& v) {
    int n = 0;
    for (int i = 0; i < NUM_LIGHTS; ++i)
        if (v.GetState(static_cast<LightID>(i))) ++n;
    return n;
}

TEST_CASE("UpdateChaseDemo keeps exactly one bulb on at every sample",
          "[VehicleLights]") {
    VehicleLights v;
    // Cover two full cycles (19 bulbs * 0.2 s = 3.8 s per cycle).
    for (int step = 0; step < 800; ++step) {
        double t = step * 0.01;
        v.UpdateChaseDemo(t);
        INFO("t = " << t);
        CHECK(CountBulbsOn(v) == 1);
    }
}

TEST_CASE("UpdateChaseDemo starts at CHMSL", "[VehicleLights]") {
    VehicleLights v;
    v.UpdateChaseDemo(0.0);
    CHECK(v.GetState(LightID::CHMSL));

    // A tick before the first step boundary (0.2 s) should still be CHMSL.
    v.UpdateChaseDemo(0.19);
    CHECK(v.GetState(LightID::CHMSL));
}

TEST_CASE("UpdateChaseDemo advances to the next bulb after 0.2 s",
          "[VehicleLights]") {
    VehicleLights v;
    v.UpdateChaseDemo(0.0);
    CHECK(v.GetState(LightID::CHMSL));

    v.UpdateChaseDemo(0.2);
    CHECK_FALSE(v.GetState(LightID::CHMSL));
    CHECK(v.GetState(LightID::RBL));   // Chase order starts rear-right.

    v.UpdateChaseDemo(0.4);
    CHECK_FALSE(v.GetState(LightID::RBL));
    CHECK(v.GetState(LightID::RRTL));
}

TEST_CASE("UpdateChaseDemo wraps after NUM_LIGHTS steps", "[VehicleLights]") {
    VehicleLights v;
    const double cycle = NUM_LIGHTS * 0.2;

    v.UpdateChaseDemo(0.05);
    bool chmsl_first = v.GetState(LightID::CHMSL);
    v.UpdateChaseDemo(cycle + 0.05);
    bool chmsl_wrap = v.GetState(LightID::CHMSL);

    CHECK(chmsl_first);
    CHECK(chmsl_wrap);
}

TEST_CASE("UpdateChaseDemo tolerates negative sim_time", "[VehicleLights]") {
    VehicleLights v;
    v.UpdateChaseDemo(-5.0);
    CHECK(CountBulbsOn(v) == 1);
    CHECK(v.GetState(LightID::CHMSL));
}

// ---------------------------------------------------------------------------
// Out-of-range LightID access is silently ignored (defence in depth for
// any caller that casts from an int signal_id).
// ---------------------------------------------------------------------------

TEST_CASE("Out-of-range LightID access does not crash or leak", "[VehicleLights]") {
    VehicleLights v;
    LightID bogus = static_cast<LightID>(999);
    v.SetState(bogus, true);
    CHECK_FALSE(v.GetState(bogus));
    // And didn't touch any real bulb.
    for (int i = 0; i < NUM_LIGHTS; ++i)
        CHECK_FALSE(v.GetState(static_cast<LightID>(i)));
}
