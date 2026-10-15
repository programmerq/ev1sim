#!/usr/bin/env python3
"""Write a weather file for ev1sim's ambient almanac from one real day.

Fetches a day's hourly air temperature and relative humidity at a place from
Open-Meteo (https://open-meteo.com, free, no key), fits them to the diurnal
model AmbientTempSensor runs on, and writes the five numbers as JSON:

    temp_c       = mean_temp_c       + diurnal_amp_c        * cos(a)
    humidity_pct = mean_humidity_pct - diurnal_humidity_amp * cos(a)
    a            = 2 pi (hour - phase_offset_hours) / 24

A config names the file under environment.weather (path relative to the
config).  The sim reads it once at startup and never touches the network; if
the file is missing or unreadable the almanac keeps its defaults, which is
also the fallback when this script cannot reach the service.

    scripts/fetch_weather.py --lat 34.05 --lon -118.24 --date 2026-06-01 -o la_june.json
    scripts/fetch_weather.py --from-json saved_response.json -o out.json   # offline
    scripts/fetch_weather.py --selftest

The output is regenerable from its "//source" line, so keep it out of the
repository unless it is a test fixture.
"""

import argparse
import datetime as dt
import json
import math
import sys
import urllib.error
import urllib.parse
import urllib.request

ARCHIVE = "https://archive-api.open-meteo.com/v1/archive"
FORECAST = "https://api.open-meteo.com/v1/forecast"
# The archive lags real time by a few days; newer dates come from the
# forecast endpoint, which also serves the recent past.
ARCHIVE_LAG_DAYS = 5
KEYS = ("mean_temp_c", "diurnal_amp_c", "mean_humidity_pct",
        "diurnal_humidity_amp", "phase_offset_hours")


def fetch(lat, lon, date):
    """Return Open-Meteo's JSON response for one local day."""
    recent = (dt.date.today() - date).days < ARCHIVE_LAG_DAYS
    query = urllib.parse.urlencode({
        "latitude": lat, "longitude": lon,
        "start_date": date.isoformat(), "end_date": date.isoformat(),
        "hourly": "temperature_2m,relative_humidity_2m",
        "timezone": "auto",
    })
    url = (FORECAST if recent else ARCHIVE) + "?" + query
    with urllib.request.urlopen(url, timeout=30) as r:
        return json.load(r)


def hourly_series(response):
    """(hours, temps, humidities) for the 24 local hours in a response."""
    h = response["hourly"]
    hours = [int(t[11:13]) for t in h["time"]]
    temps, hums = h["temperature_2m"], h["relative_humidity_2m"]
    if len(hours) != 24 or None in temps or None in hums:
        raise ValueError("expected 24 complete hourly values, got %d" % len(hours))
    return hours, temps, hums


def fit(hours, temps, hums):
    """First-harmonic fit of the almanac's model to 24 hourly values."""
    n = len(hours)
    w = [2 * math.pi * h / 24 for h in hours]
    mean_t = sum(temps) / n
    a = 2 / n * sum(t * math.cos(x) for t, x in zip(temps, w))
    b = 2 / n * sum(t * math.sin(x) for t, x in zip(temps, w))
    phase = (math.atan2(b, a) * 24 / (2 * math.pi)) % 24
    mean_h = sum(hums) / n
    # Humidity shares the temperature's phase in the model; its amplitude is
    # its projection onto -cos at that phase.
    amp_h = -2 / n * sum(rh * math.cos(2 * math.pi * (hr - phase) / 24)
                         for rh, hr in zip(hums, hours))
    return {
        "mean_temp_c": round(mean_t, 2),
        "diurnal_amp_c": round(math.hypot(a, b), 2),
        "mean_humidity_pct": round(mean_h, 1),
        "diurnal_humidity_amp": round(amp_h, 1),
        "phase_offset_hours": round(phase, 2),
    }


def selftest():
    """The fit recovers a model day exactly, and its keys are Config's."""
    truth = {"mean_temp_c": 21.5, "diurnal_amp_c": 6.25, "mean_humidity_pct": 48.0,
             "diurnal_humidity_amp": 12.0, "phase_offset_hours": 15.5}
    hours = list(range(24))
    ang = [2 * math.pi * (h - truth["phase_offset_hours"]) / 24 for h in hours]
    temps = [truth["mean_temp_c"] + truth["diurnal_amp_c"] * math.cos(x) for x in ang]
    hums = [truth["mean_humidity_pct"] - truth["diurnal_humidity_amp"] * math.cos(x) for x in ang]
    got = fit(hours, temps, hums)
    bad = [k for k in KEYS if abs(got[k] - truth[k]) > 0.01]
    response = {"hourly": {"time": ["2026-06-01T%02d:00" % h for h in hours],
                           "temperature_2m": temps, "relative_humidity_2m": hums}}
    if hourly_series(response)[0] != hours:
        bad.append("hourly_series")
    try:
        hourly_series({"hourly": {"time": response["hourly"]["time"][:23],
                                  "temperature_2m": temps[:23],
                                  "relative_humidity_2m": hums[:23]}})
        bad.append("short day accepted")
    except ValueError:
        pass
    print("selftest:", "FAIL " + ", ".join(bad) if bad else "ok")
    return 1 if bad else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--lat", type=float)
    ap.add_argument("--lon", type=float)
    ap.add_argument("--date", type=dt.date.fromisoformat, help="local day, YYYY-MM-DD")
    ap.add_argument("--from-json", help="fit a saved Open-Meteo response instead of fetching")
    ap.add_argument("-o", "--output", help="weather file to write (default: stdout)")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    if args.selftest:
        return selftest()

    if args.from_json:
        with open(args.from_json) as f:
            response = json.load(f)
        source = "Open-Meteo response %s" % args.from_json
    else:
        if args.lat is None or args.lon is None or args.date is None:
            ap.error("--lat, --lon and --date are required unless --from-json is given")
        try:
            response = fetch(args.lat, args.lon, args.date)
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError) as e:
            print("fetch_weather: cannot reach Open-Meteo (%s); no file written, "
                  "so the almanac keeps its defaults" % e, file=sys.stderr)
            return 1
        source = "Open-Meteo hourly, lat %s lon %s, %s (%s)" % (
            args.lat, args.lon, args.date.isoformat(), response.get("timezone", "?"))

    out = {"//source": source}
    out.update(fit(*hourly_series(response)))
    text = json.dumps(out, indent=2) + "\n"
    if args.output:
        with open(args.output, "w") as f:
            f.write(text)
    else:
        sys.stdout.write(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
