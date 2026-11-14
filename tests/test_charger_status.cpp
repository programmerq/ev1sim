// ChargerStatus — decoding the off-board charger's report and the status
// rows the floating UI shows for it.

#include <catch2/catch_test_macros.hpp>

#include "ChargerStatus.h"

using namespace ev1sim;

TEST_CASE("Charger cells: signed values ride in uint32 bit patterns", "[charger]") {
    CHECK(SignedFromCell(0u) == 0);
    CHECK(SignedFromCell(125u) == 125);
    CHECK(SignedFromCell(0xFFFFFF83u) == -125);   // -12.5 A in deciamps
    CHECK(SignedFromCell(0xFFFFFFF6u) == -10);    // -10 C
}

TEST_CASE("Charger state names cover the charger's state machine", "[charger]") {
    CHECK(ChargerStateName(0) == "WAIT FOR PADDLE");
    CHECK(ChargerStateName(1) == "IDLE");
    CHECK(ChargerStateName(2) == "CHARGING");
    CHECK(ChargerStateName(3) == "TAPER");
    CHECK(ChargerStateName(4) == "COMPLETE");
    CHECK(ChargerStateName(5) == "FAULT");
    CHECK(ChargerStateName(9) == "STATE 9");
}

TEST_CASE("Charger fault names list each set bit", "[charger]") {
    CHECK(ChargerFaultNames(0u) == "none");
    CHECK(ChargerFaultNames(ChargerFault::kOvertemp) == "OVERTEMP");
    CHECK(ChargerFaultNames(ChargerFault::kCommLoss | ChargerFault::kCouplerOvertemp) ==
          "COMM LOSS, PADDLE OVERTEMP");
    CHECK(ChargerFaultNames(ChargerFault::kAcInputBrownout | (1u << 2)) == "MAINS LOW, BIT 2");
}

TEST_CASE("Charger status rows show values, or --- before the charger writes",
          "[charger]") {
    ChargerStatus none;
    CHECK(FormatChargerStateLabel(none) == L"Charger: ---");
    CHECK(FormatChargerOutputLabel(none) == L"Charge: --- A at --- V");
    CHECK(FormatChargerMainsLabel(none) == L"Mains: --- V");
    CHECK(FormatChargerTempsLabel(none) == L"Charger temp --- C, paddle --- C");
    CHECK(FormatChargerFaultsLabel(none) == L"Charger faults: ---");

    ChargerStatus s;
    s.state = 2u;
    s.current_a = -12.5;   // charging is negative on the cell
    s.voltage_v = 352.0;
    s.ac_input_v = 239.6;
    s.charger_temp_c = 41;
    s.coupler_temp_c = -5;
    s.fault_bits = 0u;
    CHECK(FormatChargerStateLabel(s) == L"Charger: CHARGING");
    CHECK(FormatChargerOutputLabel(s) == L"Charge: 12.5 A at 352.0 V");
    CHECK(FormatChargerMainsLabel(s) == L"Mains: 240 V");
    CHECK(FormatChargerTempsLabel(s) == L"Charger temp 41 C, paddle -5 C");
    CHECK(FormatChargerFaultsLabel(s) == L"Charger faults: none");
}
