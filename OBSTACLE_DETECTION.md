# Continuous compact obstacle bypass

The active controlled-test route is hardcoded and configurable in `src/config.h`:

| Parameter | Implemented value |
|---|---:|
| LEFT departure target | −43° |
| Estimated axle offset | −219 mm |
| Second RIGHT target | Absolute relative 0° |
| Parallel heading target | 0° |
| RIGHT return target | +65° |
| Final LEFT target | 0°, acceptance ±5° with both sensors black |
| Chassis | 170 × 140 mm |
| Axle to front / rear | 60 / 110 mm |
| Line sensors ahead of axle | 45 mm |
| Retained wheel track | 125 mm |

`BYPASS_COMPACT_OFFSET_MM`, `BYPASS_COMPACT_DEPARTURE_DEG` and
`BYPASS_COMPACT_RETURN_DEG` select the fixed route. There is no runtime angle
or offset search, and the unapproved 270/280 mm route is not selected. The
checkpoint diagnostic and its continuation handler have been removed.

## Why the requested 190 mm / 40° route was adjusted

For the confirmed centred 200 mm obstacle, at heading zero the nominal side
clearance is `offset − 100 − 70`. At 190 mm this is just **20 mm**, leaving no
allowance beyond the existing collision floor. The retained footprint model
reserves 20 mm minimum clearance, 10 mm position uncertainty and ±5° heading
uncertainty; it also checks ±2° measured tracking error and motion between
samples. It uses the actual asymmetric axle position, not a centred axle.

Searching offsets in 1 mm steps and departure angles in 1° steps with a fixed
65° return yields **219 mm** as the smallest feasible offset, with a **43°**
departure, at the nominal starting distance. No offset in the tested 190…500 mm
range passes the complete check with a 40° departure. A larger offset cannot
simply cure that departure: it also extends its forward travel toward the
obstacle. The implemented fixed route is checked again for the actual start
range before movement; if unsafe, it fails braked rather than changing targets.

These numerical resolutions are planning resolutions, not claims of physical
millimetre/degree accuracy. Nominal parallel clearance is **49 mm**, not a
verified physical 100 mm. The earlier observed 150 mm gap with a configured
220 mm offset remains inconsistent with the uncalibrated model. This change
shortens the route without spending the retained collision allowances; it
cannot substantiate a 100 mm actual gap without physical testing.

## Distances and coordinates

x is right-positive; forward displacement is y (called `g_z` in the controller).
Heading zero is the original relative forward direction, not compass north.
Position and heading are reset at the stopped start. A chassis point (s,f),
with s∈[−70,70], f∈[−110,60], has world coordinates relative to the initial
front bumper:

```
x_point = axle_x + s*cos(h) + f*sin(h)
y_point = axle_displacement_y - 60 - s*sin(h) + f*cos(h)
```

At nominal ultrasonic range 300 mm and retained 60 mm sensor setback, initial
bumper gap is 240 mm. The assumed obstacle depth is 100 mm. Whole-car clearance
station is `240 + 100 + 170 + 20 = 530 mm` axle forward displacement.

```
departure = 219/sin(43°) = 321.1 mm
forward contribution = 219/tan(43°) = 234.8 mm
parallel = 530 - 234.8 = 295.2 mm
return to axle x=0 = 219/sin(65°) = 241.6 mm
total ideal axle path = 857.9 mm
```

Return distance includes the 45 mm line-sensor-to-axle advance once: ideal
contact occurs after approximately 196.6 mm of return, then advance 45 mm.
Tape width, sensor lateral separation, braking and encoder errors alter that
endpoint; successful alignment still requires actual line readings.
The final axle station is approximately 632.1 mm forward. Maximum nominal
left chassis extent, including pivots, is about 349.4 mm from the line.
Estimated mean wheel travel including pivots is about 1051.1 mm with the
retained 82/82 direction coefficients; this is not measured chassis motion.

The ideal axle path is about 6.9% shorter than the original 921.1 mm geometric
reference, and 11.8% shorter than the previous 973.1 mm revision. These are
model comparisons, not measured physical reductions or elapsed-time claims.

## Safety and recovery

The existing scan must complete with a stable centre return, no close echoes,
and no timeout/cancellation. Movement assumes this confirmed controlled scenario:
a centred 200 ×100 mm rectangle and an operator-verified empty LEFT corridor.
The ±30° scan cannot verify the entire corridor; no echo is not clearance proof.
Scanner interpretation, raw width telemetry and MQTT payloads are unchanged.

The full rectangle and uncertainty corridors are checked through the initial
pivot, departure, straightening pivot, parallel leg, return pivot/leg and final
alignment. Sampling is at most 5 mm /2°, with an additional bound on corner
motion between samples. Current footprint clearance is checked during motion.
Unsafe starting ranges fail instead of forcing the hardcoded geometry.

Heading is encoder-based. Separate left/right pivot coefficients, 120 ms
encoder-quiet settling before/after pivots, and existing near-target pacing
remain. BRAKE integrates residual forward encoder counts as translation.
Pivot position itself still assumes stationary rotation; actual unequal-wheel
pivot translation/slip remains an acknowledged model limitation.

PASS steers directly to 0° and fails on estimated heading outside ±2° or lateral
drift beyond 10 mm. It does not deliberately steer outward to chase a side
position. RETURN uses +65°, brakes on first line contact and confirms three
readings. It advances at the contact heading by 45 mm, bounded by a 50 mm
lateral cap, then brakes, pivots LEFT to zero and settles. Only both sensors
black and estimated heading within ±5° for three readings permit following.
Missing line, excessive overshoot or unsafe clearance fails braked.

Existing STOP paths, 15 s per-phase deadline, 10 s return/alignment deadlines,
700 mm PASS cap, heading-runaway stop and close/invalid-range checks remain.
Return distance is bounded by `219/sin(65°−5°)+60`, approximately 313 mm.
Normal line following, barcode/hump/IMU functions, networking, diagnostics,
scanner and motor interfaces are retained. Legacy curve/stack mode remains
available behind its existing configuration flag.

## Rehearsal and verification

USB/MQTT `v` is continuous again: start only while parked with no scan running,
rehearse the full fixed route on empty floor without line search, then end
STOPPED. Repeated v during motion cannot restart or continue anything. STOP
(USB x, existing MQTT/dashboard stop) cancels any phase. No checkpoint pauses
or two-minute measurement timers remain. Normal bypass uses line recovery.

Firmware is built but not automatically flashed. Existing roll, track and pivot
calibrations remain unverified on the current surface. No new measurements are
required to run the implemented route; physical clearance/success is not claimed.

```
cmake --build build --target supercar -j4
python3 tests/test_barcode_host.py
python3 tests/test_obstacle_host.py
python3 tests/test_bypass_host.py
python3 tests/test_bypass_geometry_host.py
python3 tests/test_ultrasonic_host.py
python3 tests/test_stationary_scan_host.py
python3 tests/test_web_host.py
```

The real-controller host tests exercise continuous dry and line-recovery routes,
heading restoration, encoder direction coefficients, settling/coast, STOP,
close echoes, sensor waits and bounded failure. Independent dense polygon tests
check the full footprint with tracking extremes, ±5° yaw uncertainty and 10 mm
position shifts: minimum sampled clearance is about **29.7 mm**, above the
retained 20 mm floor. They also reject offsets 190 and 218 mm. Those synthetic
checks are not physical vehicle validation.
