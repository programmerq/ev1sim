#include <catch2/catch_test_macros.hpp>

// Two-stage hood latch model + its VehiclePanels facade delegation.
#include "Hood.h"
#include "VehiclePanels.h"   // the HOOD panel delegates to Hood

using S = Hood::State;

TEST_CASE("Hood: defaults to CLOSED with the ajar sensor off", "[Hood]") {
    Hood h;
    CHECK(h.state() == S::CLOSED);
    CHECK_FALSE(h.ajar_sensor());
}

TEST_CASE("Hood: ajar sensor trips as soon as popped (POPPED == OPEN on the bus)", "[Hood]") {
    Hood h;
    h.interior_release();
    CHECK(h.state() == S::POPPED);
    CHECK(h.ajar_sensor());          // primary latch open -> sensor asserted
    h.raise();
    CHECK(h.state() == S::OPEN);
    CHECK(h.ajar_sensor());          // still asserted, indistinguishable from POPPED
}

TEST_CASE("Hood: raise is gated behind a pop (no CLOSED -> OPEN jump)", "[Hood]") {
    Hood h;
    h.raise();                       // can't raise while fully closed
    CHECK(h.state() == S::CLOSED);

    h.interior_release();            // CLOSED -> POPPED
    h.raise();                       // POPPED -> OPEN
    CHECK(h.state() == S::OPEN);

    h.raise();                       // already open — no-op
    CHECK(h.state() == S::OPEN);
}

TEST_CASE("Hood: interior_release only acts from CLOSED", "[Hood]") {
    Hood h;
    h.interior_release();            // CLOSED -> POPPED
    h.interior_release();            // no-op from POPPED
    CHECK(h.state() == S::POPPED);
}

TEST_CASE("Hood: lower_latch steps OPEN -> POPPED -> CLOSED one stage at a time", "[Hood]") {
    Hood h;
    h.interior_release();
    h.raise();
    REQUIRE(h.state() == S::OPEN);

    h.lower_latch();
    CHECK(h.state() == S::POPPED);   // lowered onto the safety catch
    CHECK(h.ajar_sensor());
    h.lower_latch();
    CHECK(h.state() == S::CLOSED);   // pressed past the catch -> primary latched
    CHECK_FALSE(h.ajar_sensor());
    h.lower_latch();                 // already closed — no-op
    CHECK(h.state() == S::CLOSED);
}

TEST_CASE("Hood: quick_toggle (F) pops then closes, but never raises", "[Hood]") {
    Hood h;
    h.quick_toggle();                // CLOSED -> POPPED
    CHECK(h.state() == S::POPPED);
    h.quick_toggle();                // POPPED -> CLOSED
    CHECK(h.state() == S::CLOSED);

    // From OPEN, the F shortcut closes one stage but won't lift further.
    h.interior_release();
    h.raise();
    REQUIRE(h.state() == S::OPEN);
    h.quick_toggle();                // OPEN -> POPPED (no raise)
    CHECK(h.state() == S::POPPED);
}

// --- VehiclePanels facade: HOOD delegates to the latch, others stay boolean ---

TEST_CASE("VehiclePanels: HOOD delegates to the two-stage latch", "[Hood][VehiclePanels]") {
    VehiclePanels panels;
    CHECK_FALSE(panels.IsOpen(PanelID::HOOD));
    CHECK(panels.hood().state() == S::CLOSED);

    panels.Toggle(PanelID::HOOD);            // F shortcut: CLOSED -> POPPED
    CHECK(panels.hood().state() == S::POPPED);
    CHECK(panels.IsOpen(PanelID::HOOD));     // derived ajar sensor trips on pop

    panels.hood().raise();                   // staged raise via the accessor
    CHECK(panels.hood().state() == S::OPEN);
    CHECK(panels.IsOpen(PanelID::HOOD));     // POPPED and OPEN both read ajar

    panels.Toggle(PanelID::HOOD);            // F from OPEN: lower one stage
    CHECK(panels.hood().state() == S::POPPED);
}

TEST_CASE("VehiclePanels: non-hood panels still toggle as simple booleans", "[Hood][VehiclePanels]") {
    VehiclePanels panels;
    CHECK_FALSE(panels.IsOpen(PanelID::TRUNK));
    panels.Toggle(PanelID::TRUNK);
    CHECK(panels.IsOpen(PanelID::TRUNK));
    panels.Toggle(PanelID::TRUNK);
    CHECK_FALSE(panels.IsOpen(PanelID::TRUNK));
}
