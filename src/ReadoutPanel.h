#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// ReadoutPanel — the shared pieces of the on-screen readouts (ClimateReadout.h,
// DriveReadout.h): a table of rows, each one decoded cell with its scenario-CSV
// column, label, value formatter and section; the readout's text lines; the
// column lookup; and the panel drawn on the right of the screen.
//
// A value is a double decoded from its cell, nullopt until electricsim has
// written the cell, which the readout shows as "---" and the CSV leaves blank.
// ---------------------------------------------------------------------------

namespace irr { class IrrlichtDevice; }

namespace ev1sim {

/// Text for one written value, without the label ("1520 kPa", "ON"). Pure.
using ReadoutFormat = std::string (*)(double);

struct ReadoutRow {
    const char* column;    // scenario-CSV column, after the readout's prefix
    const char* label;     // readout label
    ReadoutFormat format;
    const char* group;     // section the row is listed under
};

// Formatters shared by the readouts. Pure.
namespace readout_format {
std::string DegC(double v);        // "-10.0 C"
std::string Kpa(double v);         // "1520 kPa"
std::string Volts(double v);       // "47.1 V"
std::string Percent(double v);     // "45 %"
std::string Hz(double v);          // "70 Hz"
std::string OnOff(double v);       // nonzero "ON", zero "OFF"
std::string OfFull255(double v);   // "128 of 255"
std::string Count(double v);       // "3"
std::string Kohm(double v);        // "606 kohm"
std::string RadPerSec(double v);   // "42.0 rad/s"
std::string KphFromMps(double v);  // metres per second shown as "36.0 km/h"
}  // namespace readout_format

/// "<label>: <value>", or "<label>: ---" before the cell is written. Pure.
std::string FormatReadoutRow(const ReadoutRow& row, std::optional<double> value);

/// The whole readout: a header line per section (unindented), then one
/// indented line per row. Pure.
std::vector<std::string> ReadoutLines(const ReadoutRow* rows, const std::optional<double>* values,
                                      std::size_t count);

/// The index of the row whose CSV column is `prefix` + row.column, or nullopt.
/// Pure.
std::optional<std::size_t> ReadoutRowForColumn(const ReadoutRow* rows, std::size_t count,
                                               const std::string& prefix,
                                               const std::string& column);

/// The readout pages J steps through: none, climate, then drive and power.
enum class ReadoutPage { kNone, kClimate, kDrive };

/// The page after `page`, wrapping back to none. Pure.
ReadoutPage NextReadoutPage(ReadoutPage page);

/// Draw `lines` (from ReadoutLines) as a panel on the right of the screen,
/// below the snapshot overlay, under `title`. Call between BeginScene/
/// EndScene. Null-safe.
void DrawReadoutPanel(irr::IrrlichtDevice* device, const std::string& title,
                      const std::vector<std::string>& lines);

}  // namespace ev1sim
