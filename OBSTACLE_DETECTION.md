# Measurement reporting update

This section supersedes earlier width-classification and reporting notes.
The scanner has no nominal obstacle dimensions, width acceptance window or
150/250 mm classification. Continuous width is derived from measured edge
geometry. Reliable confirmed geometry is published; insufficient edge evidence
produces UNKNOWN/null. A missing centre echo remains unavailable and does not
borrow the trigger range or a neighbouring ray. Raw samples retain their actual
range and quality, including weak returns and no-echo observations.

The full 31-angle, three-attempt sweep and 200 mm threshold remain. Timeout or
STOP can interrupt a sweep. MQTT and both dashboard APIs publish null for
unavailable numeric measurements. The retired calibration/corrected-width JSON
keys are removed; consumers must use width and edge status. Low-level profile
compatibility slots remain zero and are never published as measurements.

Bypass planning exclusively supplies BYPASS_FIXED_WIDTH_MM (150 mm), the
unchanged effective planning width with the previously unpopulated calibration
table. Nominal depth and movement constants remain planning-only. No scan width
is constrained to this nominal size. The return, post-contact advance, lateral
cap, final correction, state handoff and bounded IR recovery are unchanged.

Limitations: geometric width is an estimate from angular sampling and the
ultrasonic beam, not a calibrated precision measurement. No continuous physical
calibration data exists, so none is fabricated. Unreliable data may now yield
UNKNOWN more often. Strict BARR-C compliance is still not claimed; existing
C11 SDK, naming/type and long-function deviations remain in BARR_C_REVIEW.md.
Numeric Teleplot channels retain explicit invalid sentinels; JSON uses null.

Current verification: review/measurement-tests.txt, measurement-build.txt and
measurement-barr.json. Previous review patches, reports and binary comparisons
are snapshots of the earlier submission refactor, not this updated firmware.
No firmware has been flashed or pushed.

---

# Submission implementation — 24 November 2026

This section supersedes earlier dated implementation notes below.

- The trigger is **200 mm measured from the ultrasonic sensor**. After stopping,
  the existing planner consumes the actual stationary scan front range; it does
  not substitute the threshold for a measured distance.
- The original complete centre-outward sweep remains: every configured angle
  from -30 through +30 degrees, 2-degree spacing (31 angles), three genuine
  echoes attempted per angle. The scan waits for servo settling and fresh echo
  sequences. Missing echoes stay missing; no random or nominal range is injected.
- Original left/right edge brackets, apparent width, sampling interval and
  physical-envelope calibration calculations are retained. Detailed terminal
  output is restored directly from bbedf5d before the standards-only refactor.
  Unknown/unreliable edges and calibration are explicitly labelled.
- Calibration rows are currently zero/unpopulated. There is no physical table
  to apply yet; calibrated width is unknown until actual calibration is supplied.
  A reliable calibrated 140–160 mm width is retained when available; otherwise
  the separate nominal 150 mm estimate remains. Planning still uses at least
  150 mm width and 150 mm depth. Raw points and measured distance are preserved.
- Completion and cancellation recenter the servo. Motors remain stopped during
  autonomous scanning; STOP, MQTT, USB, barcode and other controls are retained.
- Bypass algorithm and limits remain unchanged: 210 mm nominal axle offset,
  existing 40 mm pass trim, +65 degree return, **140 mm** post-IR advance,
  **160 mm** lateral cap, final heading target zero, and bounded 240 mm / 8 s
  recovery. Stable detection from either IR sensor with heading confirmation
  permits autonomous FOLLOW_LINE. Manual `v` ends STOPPED.

For an exact stationary range of 200 mm and the existing -10 degree initial
heading model, the unchanged planner selects a 59 degree departure target
(49 degree left rotation from -10), no reverse, approximately 245.0 mm departure,
320.8 mm parallel travel and 231.7 mm ideal return. Ideal translation is 797.5 mm;
the empirical recovery extension above the 45 mm sensor model adds 95 mm,
giving about **892.5 mm before any additional bounded search**. These are
planner outputs, not replacement hardcoded movement values. At 150 mm the same
planner still produces the previous 77 degree departure. Measured range,
encoder calibration and line contact determine the actual executed movement.

Independent dense footprint checks at 200 mm, including the extension and final
rotation, give **38.04 mm minimum overall clearance** and **40.90 mm parallel
side clearance** with the existing uncertainty model. This does not claim a
40 mm margin at every transient orientation or guarantee real floor clearance.
No physical test of the 200 mm trigger has been performed in this work.

See [BARR_C_REVIEW.md](BARR_C_REVIEW.md), the separate review patches and test
reports for the source-wide refactoring and its explicit compliance limits.

---

# Current physical-test adjustment: 140 mm advance and bounded IR search

Departure, parallel travel, +65° return and the original final +65°→0°
correction are unchanged. The post-IR-contact advance is now **140 mm**, up
from 65 mm. That adds `75*sin(65°)=68.0 mm` sideways, close to the reported
70 mm displacement (`70/sin(65°)=77.2 mm` extra forward distance). The physical
sensor offset remains 45 mm; 140 mm is the explicitly requested empirical
compensation, not a fabricated mounting measurement.

The lateral cap is **160 mm**. At the planned heading, 140 mm projects to
126.9 mm laterally (128.9 mm at 67°), leaving room for contact coast. The cap
remains a safety bound. The existing footprint validator derives its extended
return endpoint/final pivot from the advance constant: it now checks the full
**95 mm extension beyond the ideal axle-centred return** (140−45), not the
previous 20 mm extension. Independent dense footprint cases include this
extension for both 150 and 160 mm planning widths. No turn angle was retuned.
The dry route is unchanged; the ideal normal estimate increases by 75 mm,
from 816.5 to approximately 891.5 mm before any additional search. Actual
contact geometry remains a physical tuning uncertainty.

## What the old failure and line-following code show

The logged 4.3° heading was inside the old nudge heading allowance, with both
IR readings white. The old 60 mm/3 s correction and ±5° target provided only
about 5 mm lateral travel even in an ideal straight angled probe, insufficient
for a large remaining offset. The old log does not identify uniquely whether
distance, elapsed time or a range guard ended that run. Failure logs now also
include travel, elapsed milliseconds and actual ultrasonic range.

The actual normal follower treats BOTH black as centred, LEFT-only black as
an instruction to arc left, and RIGHT-only black as an instruction to arc right.
It can continue across both-white using previous tracking history; both-white
alone is not fresh evidence of reacquisition. LINE_CAL stops/returns READY if
it starts on both white and does not preserve a track-heading target, so moving
the bypass into that mode would not solve this missing-line case.

The bypass now uses a **stable left OR right black reading with heading within
±5°** for a valid handoff. BOTH black is accepted but is no longer mandatory.
The existing `line_filter_reset` seeds fresh sensor/filter/side history on
handoff, and the follower's 300 ms straight launch is skipped specifically for
BYPASS→FOLLOW_LINE, allowing the normal arc correction immediately. Other
START, barcode and line-calibration behaviour is unchanged. A fresh both-white
handoff is still rejected; the car is not released on an invented centre state.

## Bounded recovery

After the unchanged final turn and settling, missing line starts one low-speed
forward search, bounded by **240 mm / 8 seconds**. Search heading targets are
**±25°**, with a 2° tracking guard. A single visible sensor sets a gentler
**±8°** remembered correction side. On detecting either sensor, steering targets
zero; completion requires stable line readings and heading within ±1° in the
search phase (which satisfies the overall ±5° requirement). If contact is lost,
it follows the remembered side within the same time/distance budget.

If neither sensor has ever found black, it may change search side once after
75% of its distance budget. There are no repeated sweeps, resets of the
recovery deadline, checkpoints or blind transitions to FOLLOW. An incorrect
pose/side estimate can still produce a bounded failure; search is not a
physical guarantee for every off-track location.

A full axle turning-circle envelope must already be behind the obstacle
before search is permitted. Heading remains forward-going and live footprint
checks remain active. Search has a conservative 150 mm forward close-echo
stop (the existing 120 mm bypass guard remains for other phases). Invalid
range, heading violation, STOP, timeout or distance exhaustion fails stopped.
After valid recovery, normal operation transitions automatically to FOLLOW_LINE
and immediately uses its established IR steering. Continuous manual v remains
STOPPED at completion and never enters this line-search phase.

The 150 mm detection trigger, all servo scan angles, raw distance, raw width,
calibrated/corrected-width fallback and planning-depth rules are unchanged.
Regression tests cover extended advance/cap, footprint sweep, one-sensor
success, immediate follower steering, both-white rejection, search-side change,
heading/range/time/distance failures, STOP and continuous v. Firmware is rebuilt,
not automatically flashed or pushed; actual centring remains unverified.
All older 65 mm/±5°/both-sensors-required notes below are superseded here.

---

# Pre-flash verification: bounded final line correction

The 40 mm parallel trim is applied to `clear_y`, which also positions the
return pivot, return corridor and final alignment sweep. It is not just a
shortened distance counter. Independent footprint tests cover both 150 and
160 mm planning widths and the empirical recovery extension.

Sequence remains: first IR contact → brake/confirm → **65 mm advance** →
encoder-quiet brake → existing final LEFT correction to **0°** → confirmation.
The main return heading remains **+65°**; its final target/turn calibration have
not been retuned. The advance finishes before final rotation starts.
At +65°, the nominal lateral movement is `65*sin(65°)=58.9 mm`, or about
59.8 mm at +67° measured heading. The **70 mm lateral cap therefore does not
truncate the planned 65 mm advance**, while retaining a guard against excess
movement since first contact/coast. This does not claim unmeasured braking
coast can never consume the remaining margin.

If the original final turn is aligned/settled but BOTH IR readings are missing,
normal bypass now gets **one bounded forward correction** before failure:

- At most **60 mm encoder travel or 3000 ms**, including sensor waits.
- Lower drive command 9, both wheels forward; target steering within **±5°**.
- A single black sensor selects correction direction. With both white, the
  estimate/known return side selects direction; it does not execute an open-ended
  left/right sweep. An incorrect side estimate may still cause a safe failure.
- Both black steers back toward heading zero. Success requires consecutive
  BOTH-IR readings and heading within **±1°** in this correction phase, which
  satisfies the existing overall ±5° requirement.
- Measured heading beyond the ±5° target plus 2° tracking allowance, close echo,
  invalid range, distance/time limit or any existing safety guard fails stopped.

The correction is allowed only if a full axle-centred turning circle plus
position/collision allowances is already behind the obstacle rear plane.
Since both wheels drive forward with small heading error, forward movement
increases this rear clearance. Live full-footprint checks remain active too.
This is a local recovery adjustment; departure, overtaking offset, main return
angle and original final pivot are unchanged. A large line offset is not
silently treated as successful recovery. There is no repeat/restart loop.

Normal success still undergoes a live BOTH-IR recheck at vehicle handoff and
transitions automatically to **FOLLOW_LINE**, with forward motor output on the
next control tick. Failure remains **STOPPED**. Manual continuous **v** remains
a dry rehearsal and ends STOPPED; it does not use this line-recovery phase.

Focused tests verify the first-contact/advance/pivot order, continued motion at
64.9 mm without triggering the lateral cap, the unchanged zero target, bounded
correction success, one-sensor rejection, timeout during a sensor wait, distance/
heading/clearance failure, STOP cancellation and normal FOLLOW/forward handoff.
Firmware is rebuilt but has not been flashed. Physical performance remains
unverified; the 65 mm empirical tuning may still require adjustment on the car.

---

# Current update: measured-width fallback and recovery tuning

The 150 mm ultrasonic trigger, all existing servo angles, genuine median range
samples, edge search and calibration lookup are retained. No range or raw width
is overwritten. A separate corrected-width product is computed:

- Reliable calibrated width in **140–160 mm inclusive**, with both edges and
  a usable centre reading: retain that calibrated value, fallback=false.
- Invalid calibration, out-of-band calibrated width, unusable/missing edges,
  cancelled/timed-out data or invalid width: **150 mm**, fallback=true.

The existing calibration is a measured class lookup (currently capable of 150
or 250 mm), not a newly invented continuous width calibration. Unknown data
remain unknown in the raw/calibrated outputs. A fallback estimate is not proof
of verified edges or corridor clearance. Normal bypass still requires a
completed nonfailed sweep and genuine stable forward obstacle reading.
The vehicle controller automatically assesses/starts bypass after the sweep;
manual stationary scans remain stationary. All existing scan angles remain.

Dashboard scan JSON, graph text, MQTT obstacle event and serial log expose
`corrected_width` / `width_fallback` separately from raw `width` and
`calibrated_width`. Genuine measured `front` range is retained. Planning uses
**max(150, corrected_width)** and fixed depth **150 mm**. Widths up to 160 mm
increase offset only as required by the existing side-clearance check (210 mm
at planning width 150; 215 mm at 160). The measured value can be 140 mm without
allowing a narrower-than-150 mm planning envelope.

The requested 50 mm shorter parallel leg was checked against the complete
footprint. **40 mm shorter passes; 45–50 mm fails** at the existing 65° return.
The implemented trim is therefore **40 mm**, rather than weakening the safety
check or changing the return angle. For nominal 150 mm width/range, the straight
leg becomes about **349.2 mm**, down from 389.2 mm. This earlier return is
validated while the rear may still be alongside the obstacle; the full swept
footprint replaces the old conservative whole-car-past station as the deciding
constraint. All live collision/heading/STOP/timeout protections remain.

`BYPASS_REACQUIRE_ADVANCE_MM=65` is a separate empirical post-contact advance;
`BYPASS_LINE_SENSOR_FORWARD_MM=45` remains the physical mounting dimension.
The lateral post-contact cap is increased from 50 to **70 mm**, so it does not
truncate the requested 65 mm movement. The extra diagonal endpoint and final
pivot are included in footprint validation. Both dry and extended endpoints
are tested. The early return changes the forward station; lateral distance is
unchanged, so the pre-contact return is still controlled by real IR detection.

This 65 mm setting is a requested physical tuning change, not a claim that a
45 mm ideal sensor model mathematically needs 65 mm. Tests include a synthetic
reported-effective-offset recovery case and an uncompensated ideal-mount case
that safely refuses success when the extra advance leaves the sensors off-line.
Physical line centring remains unverified. No test invents a successful reading
on the car. BOTH IR sensors and heading within ±5° are still mandatory before
normal FOLLOW; v remains a continuous dry rehearsal ending STOPPED.

`BYPASS recovery failed` is not bypassed. Failures now additionally log the
actual phase, estimated x/y, heading in tenths of a degree, both IR readings and
confirmation count. A failed sensor/heading confirmation still stops rather
than driving off-line. These logs distinguish alignment failure from collision,
heading or distance guards. No arbitrary post-failure restart has been added.

For comparison, the dry geometric axle path is now about **796.5 mm**. With an
ideal 45 mm contact projection and the 65 mm empirical advance, the normal
estimate is about **816.5 mm**; actual tape contact/pivot translation determines
the real length. The reference calculations below precede this trim/tuning and
are superseded by this section. Firmware is built, not flashed or pushed.

---

# Forward-first bypass from a 150 mm ultrasonic reading

The active scenario is a **180 ×140 mm car**, a **150 ×150 mm obstacle centred
on the original track**, and initial heading approximately **−10° relative to
that track**. The starting measurement is **150 mm from the ultrasonic sensor**;
this route does not reuse the previous 80 mm starting-bumper-gap calculation.

Retained mounting/calibration: axle-to-front 60 mm, axle-to-rear 120 mm,
line sensors 45 mm forward, ultrasonic sensor 60 mm behind the front bumper,
wheel track 125 mm, LEFT/RIGHT encoders 13307/13710 edges/metre, and separate
pivot direction coefficients (currently 82/82). These remain configured
calibrations, not newly measured accuracy or forward-arc turning radii.

## Sensor frame, track frame and initial orientation

150 mm minus the 60 mm mounting setback gives **90 mm bumper gap along the
car's forward axis**, not 150 mm bumper clearance. The rotated corner is closer.
With the initial axle as (0,0), x right-positive, y forward-positive and heading
right-positive, a chassis point (s,f) has:

```
x_point = axle_x + s*cos(h) + f*sin(h)
y_point = axle_y - s*sin(h) + f*cos(h)
s ∈ [-70,70], f ∈ [-120,60]
obstacle_front_y = (sensor_range + axle_to_front - sensor_setback)*cos(initial_h)
                 = 150*cos(-10°) = 147.721 mm
```

Initial front-bumper-centre track-y is 60*cos(10°)=59.089 mm; the nearest
forward corner has y=71.244 mm. Thus the nominal closest front-plane gap is
about **76.5 mm**, despite the 90 mm axial gap. The footprint calculation
includes those corners explicitly. Internally `front=87.721` is obstacle y
minus a common 60 mm coordinate translation; it is not the sensor distance.

**Yaw limitation:** the current LSM303DLHC module exposes raw accelerometer/
magnetometer axes and pitch/roll, but no calibrated yaw or gyroscope. This
implementation does not substitute an uncalibrated compass bearing for yaw.
Heading feedback remains encoder-based, with track zero established using the
supplied `BYPASS_INITIAL_HEADING_DEG=-10` assumption. IMU-based yaw steering is
not implemented with this driver. Actual starting-yaw error and wheel slip can
invalidate the estimates; physical ±5° accuracy is not independently verified.
All targets below are absolute within this track-relative frame, not compass
bearings or additional angles relative to the tilted car.

## Route and shortest practical forward candidate

PLAN runs while braked, testing forward-only candidates at 1° increments from
20° to 85°. Each candidate checks the complete footprint, uncertainty corridors
and all rotations. At a 150 mm sensor reading, 76° fails and **77° is the first
accepted departure**. With fixed offset and whole-car clearance station,
translation and estimated wheel travel increase with departure angle, so the
first safe angle is the shortest in this validated forward pivot/straight
family. This is not a global optimum among arbitrary uncalibrated curves.

| Movement | Nominal value |
|---|---:|
| Reverse travel | **0 mm** |
| First LEFT rotation | 67°, from −10° to **−77°** |
| Departure diagonal | **215.5 mm** |
| Estimated axle lateral displacement | **210 mm LEFT** |
| Second RIGHT rotation | 77°, restoring **0°** |
| Parallel travel at heading 0° | **389.2 mm** |
| Return RIGHT rotation | 65°, targeting **+65°** |
| Return travel including sensor advance | **231.7 mm** |
| Final LEFT correction | 65°, restoring **0°** |
| Total ideal axle travel | **836.5 mm** |
| Estimated mean wheel travel including pivots | **1081.6 mm** |

The return remains 25° away from perpendicular. The 65° practical cap balances
travel, pivot rotation and line approach; it avoids assuming a new calibrated
arc radius or adding a perpendicular crossing. Existing paced pivots and quiet
settling remain. Fractional wheel-steering corrections accumulate instead of
being rounded away, improving small heading corrections during continuous
forward travel. This is forward-only **translation**: in-place pivots still
counter-rotate the drive wheels.

The old extra 30 mm parallel allowance has been removed after complete return-
rotation checks passed without it. The entire car still clears before return;
`BYPASS_PASS_EXTRA_MM=0` retains the independent 20 mm whole-car margin:

```
return_start_y = 147.721 obstacle front + 150 depth + 120 rear overhang + 20
               = 437.721 mm
departure = 210/sin(77°) = 215.524 mm
departure_forward = 210/tan(77°) = 48.482 mm
parallel = 437.721 - 48.482 = 389.239 mm
return = 210/sin(65°) = 231.709 mm
total = 215.524 + 389.239 + 231.709 = 836.472 mm
```

Ideal line contact occurs about 186.7 mm into return, followed by the measured
45 mm sensor-to-axle advance (included once above). The final axle station is
about 535.6 mm forward. Actual tape width, sensor spacing, braking and slip
change the contact point, so geometry alone cannot authorize success.

If the fresh scan reports a closer stopped position, the planner recomputes
all distances and again tries every supported forward-only angle first.
Only after all fail does it try a bounded 77° departure with reverse in 2 mm
increments, up to 220 mm. For example, at an actual 144 mm reading a forward
81° candidate passes; at 140 mm no supported forward candidate passes and a
12 mm reverse at −10° passes. An unsafe initial footprint or exhausted bound
fails stopped. Those are model results, not a proof that every conceivable
forward curve is impossible. Rear space must be clear if fallback reversing
is required: there is no rear sensor. No reverse is selected at nominal 150 mm.
The existing per-phase deadline bounds the stopped planning process too.

## Clearance and entire-footprint verification

The desired 200 mm displacement gives 55 mm nominal side clearance, but less
than 40 mm under the retained error budgets. With 2° measured tracking error,
5° yaw uncertainty, 10 mm position allowance and the 120 mm rear overhang:

```
worst_side_projection = 70*cos(7°) + 120*sin(7°) = 84.103 mm
minimum_offset = ceil(75 + 84.103 + 10 + 40) = 210 mm
```

The chosen offset therefore gives **65 mm nominal side clearance**, and
approximately **40.9 mm minimum during parallel overtaking** within these
assumptions. PASS monitors that side-clearance bound in addition to estimated
heading and lateral drift. No observed 150 mm physical gap is used as a new
encoder calibration or blindly subtracted from a target.

`bypass_geometry.h` checks the full rectangle through pivots and straight legs
using separating projections. It includes the asymmetric axle position,
possible arrival-y corridors for ±2° tracking, 10 mm position allowance and
±5° yaw uncertainty. Samples are no farther than 5 mm/2°, with an additional
bound on corner motion between them. The complete-route collision floor is
20 mm; the parallel section additionally enforces the 40 mm side requirement.
These are configured uncertainty assumptions, not measured sensor accuracy.

Independent dense polygon-distance checks with tracking extremes, yaw
perturbations and position shifts found **33.7 mm minimum across the complete
route**, and **40.9 mm during parallel overtaking**. Thus the 40 mm target is
a side-clearance requirement, not a claim that all front/rear corners stay
40 mm from the obstacle at every instant. Off-centre obstacles, different
mounting or unbounded slip are outside this controlled scenario.

## Heading recovery, state transitions and safety

Normal bypass always requests line recovery. It brakes on first IR contact,
confirms the reading, advances at the contact heading by 45 mm (bounded by a
50 mm lateral cap), settles, then performs the paced LEFT correction to zero.
It does not resume merely because one sensor touched black or distance elapsed.
Both sensors must confirm black for consecutive readings AND estimated heading
must be within ±5°. The live IR pair is checked again at vehicle handoff.
Only successful normal bypass enters **FOLLOW_LINE**, enabling forward motion
on the next control tick. Missing line, poor heading or any failed safety
condition ends STOPPED. The user-requested continuous `v` command remains a
dry rehearsal with no checkpoints, no line search, and an unconditional
STOPPED finish even if started over black.

Signed unequal-wheel pivot translation and reverse coast are integrated;
position is no longer silently frozen during every pivot. Separate LEFT/RIGHT
pivot scales, 120 ms encoder-quiet settling, existing pulse pacing, motor
interfaces and STOP paths remain. The 150 mm obstacle trigger does not disable
detection on curves. Slowdown at 350 mm, hard stop 80 mm, close echo stop
120 mm, invalid-range rejection, fresh scanning, 15 s phase deadlines, 10 s
return/alignment deadlines and distance/heading limits remain. The legacy
curve/stack mode retains its own previous 75° limit. MQTT, USB, barcode,
IMU/hump telemetry and unrelated functions are preserved.

## Build and tests

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

Tests cover the nominal forward-only route, shorter-range reverse fallback,
continuous STOPPED rehearsal, both-IR/heading recovery, normal FOLLOW and
forward motor output, handoff rejection with one missing sensor, settling,
signed coast/pivot motion, STOP, timeouts, close echoes and independent dense
footprint checks. The firmware is built, not automatically flashed or pushed.
Physical clearance, physical heading accuracy and IMU yaw control have not
been demonstrated.
