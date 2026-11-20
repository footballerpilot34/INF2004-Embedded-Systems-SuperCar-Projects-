# Fixed 20 cm obstacle bypass test

Target: `supercar`; output `build/supercar.uf2`. The normal scanner, MQTT
publisher, dashboard and line follower remain in use. This is a controlled
floor test. The user physically tested the previous 90° route and confirmed
obstacle clearance, but reported an approximately 110° first left turn and
poor final alignment. The revised turns/alignment below need floor validation.

## Test assumptions and behaviour

- Car 180 × 140 mm; sensor 60 mm behind its front bumper.
- Centred obstacle: **assumed 200 mm wide and 100 mm deep for movement only**.
- Stop threshold is now 300 mm from the sensor, giving a nominal 240 mm
  bumper gap. It was 250 mm. The slow threshold stays 350 mm; the release
  threshold is 350 mm to retain 50 mm hysteresis above the new stop threshold.
- The existing 60–120° scan is unchanged: centre first, then left/right,
  2° steps, three pings per point. No width algorithm changes in this stage.
- **The operator must establish an empty LEFT bypass corridor**, including
  turning room. The ±30° scan cannot verify the whole corridor. This is
  logged as a test assumption, never claimed as sensor-verified clearance.

After a completed scan with a stable, plausible centre return:

1. Pivot **60° LEFT**, using existing wheel commands and braking pulses
   during the final 15° of the turn.
2. Drive diagonally along that heading until encoder-estimated lateral
   displacement reaches **220 mm** (`100 + 70 + 50` mm).
3. Turn **60° RIGHT back to heading zero**, then travel parallel to the line.
4. Pass the far face and whole vehicle before turning **60° RIGHT** toward
   the line. All turn targets are relative to the original forward heading.
5. Drive toward the original line, checking the existing line sensors each
   control tick. Brake on the first line reading and confirm three readings.
6. Apply the existing 40 mm sensor-to-axle advance, then use ALIGN to turn
   LEFT toward heading zero. Only **both sensors black AND estimated heading
   within ±5° of forward for three consecutive readings** permit following.
   Brake on the first qualifying pair, so confirmation does not rotate away
   from the line. Reaching zero without both sensors means holding braked
   until the existing alignment timeout; there is no blind extended spin.

The previous sharp turn commanded 90° with opposite wheel directions at
17% power; the unchanged driver applies its existing 25%/60 ms startup boost.
It was encoder-controlled, not a timed turn. The user's approximately 110°
observation for that command gives a provisional pivot correction of
`100 * 90/110 ≈ 82`. The correction is not a precision measurement, and both
turn directions need validation. Near-target pacing reuses the existing
60 ms ON /180 ms braked pulse durations only for the final 15°. Target crossing
brakes rather than reversing and retriggering the boost; excessive estimated
overshoot fails. Motor driver, steady speeds and the line follower are unchanged.

A smooth forward arc at the existing speed ratio needs a nominal ~150 mm
radius. Combining that arc with the remaining diagonal departure would consume
the conservative front/corner clearance budget at this start distance. The
smaller paced pivot is therefore retained instead of adding a new trajectory.
The 220 mm lateral target remains unchanged. Following successful floor tests
of obstacle clearance and line recovery, the forward return point is now
trimmed by 100 mm as requested; see the clearance calculation below.

Unknown, clipped or unreliable width does not veto this route. Raw width,
calibrated width, edge positions and edge warnings do **not** select the side,
lateral distance or forward clearance. The previous curve/stack controller
is retained behind `BYPASS_FIXED_WIDTH_TEST=false`; it is not the default
route for this experiment.

## Footprint and forward clearance

The motor driver supports in-place pivots; wheel encoders work and have
recorded rolling-distance calibration. The initial turn check conservatively
reserves `ceil(hypot(180,70)) + 20 = 214 mm` of bumper gap. This covers any
longitudinal pivot-axis position inside the chassis, on its centreline,
without inventing an axle-to-bumper measurement. The nominal 240 mm gap
passes that check. The new diagonal also reserves its forward travel plus
half-width and the existing 20 mm margin before the front face: approximately
225 mm bumper gap is required, allowing the existing 1.5° turn-completion
tolerance. The 240 mm nominal gap passes; too-close starts stay blocked.
A live departure bound also stops if the front-plane budget is exhausted
before lateral clearance. Verify the drive pivot axis lies inside the footprint.

During the parallel leg, existing heading steering holds the departure
lateral position. A corner-projection check includes the whole car length
and half-width, and stops if the remaining lateral allowance is too small.
These checks assume the configured geometry and calibrated motion actually
match the floor behaviour; they cannot compensate for wheel slip.

Required forward progress of the original front bumper is:

```
(sensor distance - 60) + 100 obstacle depth + 180 car length
+ 70 corner allowance + 50 margin - 100 forward trim
```

At a 300 mm sensor start this is **540 mm from the stopped position**, 100 mm
less than the previous 640 mm goal. `BYPASS_FORWARD_TRIM_MM=100` records the
user's floor-test adjustment after successful clearance and line recovery.
The goal is bounded below by the whole-car pass plus the existing 20 mm
minimum clearance margin; a larger future trim cannot remove that minimum.
This removes part of the earlier conservative turning reserve; validate the
earlier return on the actual obstacle before relying on it.

PASS compares accumulated forward progress with that goal, so the diagonal
departure's approximately 127 mm forward contribution is counted. The nominal
parallel leg is now about **413 mm**, previously 513 mm. The 60° turn targets,
220 mm lateral offset, approximately 254 mm diagonal legs, 100 mm obstacle
depth and 40 mm line-sensor advance are unchanged. All distances remain
encoder-based geometric references, not guaranteed physical travel.

## Protections retained

- STOP/button/web/MQTT stop cancels through the existing vehicle state.
- A failed, cancelled, missing-centre or timed-out scan cannot start a normal
  bypass. An actual close scan echo also prevents starting.
- Servo settling and a fresh measurement are required after each change of
  body heading. A returned forward echo <=120 mm, or invalid zero range,
  fails the manoeuvre. At the centred servo this corresponds to a nominal
  60 mm bumper gap. Physical braking distance still requires testing.
- Missing echoes during movement are permitted **only under the explicit
  operator-verified empty-corridor test assumption**. They do not verify
  clearance, and cannot distinguish open space from sensor disconnection.
- Existing 15 s per-phase deadline and heading-runaway stop remain.
  Departure has a travel/progress bound; PASS retains the 700 mm travel cap.
- Return has a 10 s timeout and 400 mm maximum travel (`220 + 180`). Final
  alignment also has a 10 s timeout. Missing line, only one aligned sensor,
  or a sideways heading cannot declare successful reacquisition.
- Normal obstacle-triggering is already excluded in `VEHICLE_BYPASS`.
  No new detection cycle starts while returning. Success resets following
  history through the existing state transition; failure ends STOPPED.
- The old blind reverse is not enabled.

## Configuration and calibration

All movement values are in `src/config.h`:

| Constant | Initial value / purpose |
|---|---|
| `BYPASS_FIXED_WIDTH_TEST` | `true`: controlled fixed-width route |
| `BYPASS_FIXED_WIDTH_MM` | 200, movement only |
| `BYPASS_FIXED_MARGIN_MM` | 50, gives derived 220 mm lateral target |
| `BYPASS_FORWARD_TRIM_MM` | 100, user-requested earlier return after floor tests |
| `BYPASS_ASSUMED_DEPTH_MM` | 100 |
| `SENSOR_TO_BUMPER_MM` | 60, user measured |
| `ENC_LEFT_EDGES_PER_M` / `ENC_RIGHT_EDGES_PER_M` | 13307 / 13710, recorded 300 mm roll calibration |
| `WHEEL_TRACK_MM` | 125; verify wheel contact-centre spacing |
| `BYPASS_PIVOT_SLIP_X100` | 82, provisional `100 * 90/110` from user floor feedback |
| `BYPASS_DIAGONAL_DEG` | 60, departure and return; intermediate/final heading zero |
| `BYPASS_TURN_SLOW_DEG` | 15, final portion paced using existing pulse durations |
| `BYPASS_ALIGN_TOL_DEG` | 5, requires both sensors as well as forward heading |
| `BYPASS_PIVOT_SPEED` / `BYPASS_DRIVE_SPEED` | existing 17 / 12 percent |
| `BYPASS_LINE_ADVANCE_MM` | existing 40; validate actual sensor-to-axle advance |
| `BYPASS_LINE_TIMEOUT_MSEC` | 10000 |
| `BYPASS_STEP_MAX_MSEC` | existing 15000 |

No new timed movement calibration is introduced: the route uses existing
encoders. Recheck the recorded 300 mm roll and physically measure the turns.
Measure the new 60° LEFT and RIGHT turns and the restored forward headings.
If a turn stops short, raise `BYPASS_PIVOT_SLIP_X100`; if it overshoots, lower
it. Keep a record of commanded versus actual angles; the initial 82 factor
comes from an approximate observation, not validated turning accuracy.
The 40 mm line advance is an inherited starting value, not a newly measured
sensor offset; adjust it so the final pivot leaves the line under the sensors.
Do not treat any of these initial settings as demonstrated physical accuracy.

Width calibration is not required to try this fixed route. The previous
raw estimator and optional 15/25 cm calibration table are left intact for
telemetry/future work; the table is not used by the fixed route.

## Physical testing in stages

Use a clear floor, low existing speeds and an immediately available STOP.
Leave generous room to the left (about 0.5 m) and ahead (about 1 m).
Flash manually only when ready; nothing is flashed automatically.

1. **Motor movement, no obstacle:** while READY/STOPPED, send **`v`** by USB serial or to the existing MQTT `…/cmd` topic.
   This explicitly rehearses the fixed route on an empty floor using the
   nominal 300 mm start geometry. It skips the scan, publishes no invented
   scan, returns by encoder position without requiring a line, then ends
   **STOPPED**. Mark the initial centreline and measure lateral displacement
   at `pass`: at least 220 mm. Check the first 60° LEFT turn is moderate, the
   next RIGHT turn restores forward orientation, and the return restores
   forward orientation again before the rehearsal ends. Check pacing and STOP.
   Repeat until encoder travel and actual movement agree adequately.
2. **Obstacle clearance:** provide a short starting line for normal START,
   put the 200 ×100 mm obstacle 300 mm from the sensor (240 mm from bumper),
   and verify the left corridor is empty. START normally. Observe stop,
   sweep, left departure and parallel pass. Check the rear corner clears
   before return; use STOP to end this stage. Without a continuation line,
   the bounded return must eventually stop rather than resume following.
3. **Line reacquisition:** extend the original straight line beyond the
   obstacle. Repeat. Verify braking at the detected line, the short advance,
   final LEFT alignment, both-sensor/±5° confirmation and normal following.
   Detecting the line while oblique must not start following. If alignment
   fails, record actual heading and both sensor states, then tune the
   inherited 40 mm advance or pivot correction instead of weakening the gate.
   Remove the return line once to verify the failure stop.
4. **Full integration:** approach from beyond 300 mm and repeat
   Detect → Stop → Sweep → MQTT → Bypass → Clear → Return → Follow.
   Check raw MQTT widths vary with actual scans and are not forced to 200.
   Try STOP in each phase and an unexpected close object in the travel
   direction. Record actual side/rear clearance and repeatability.

The dry run uses the same handler from USB serial or MQTT and is rejected
while scanning, driving or in IR DIAG. Repeated `v` commands do not restart
an active run. Send MQTT `stop` to interrupt any phase; existing topics and
acknowledgements are unchanged. No dashboard controls were added.

## Software verification and files

Baseline verification: the firmware build and five host checks passed. The
barcode host check initially failed because its Pico time stubs were missing;
those stubs are now supplied without changing the barcode driver. Final
verification passed all six host checks and the `supercar` firmware build. Re-run:

```
python3 tests/test_barcode_host.py
python3 tests/test_obstacle_host.py
python3 tests/test_bypass_host.py
python3 tests/test_ultrasonic_host.py
python3 tests/test_stationary_scan_host.py
python3 tests/test_web_host.py
cmake --build build --target supercar -j 4
```

`bypass.c/.h` adds the fixed route and dry-run entry using existing phases,
pivot/steering helpers and odometry. `config.h` contains dimensions, movement
constants and the 300 mm threshold. `vehicle.c/.h` integrates the dry run and
ensures it ends STOPPED. `supercar_main.c` and `web.h` dispatch `v` through
the existing command mechanism; `web.c` recognises the same word from MQTT. Scanner, MQTT payload generation,
web dashboard and the following algorithm are unchanged in this stage.
Tests cover ideal encoder-driven full routes, moderate departure with full
lateral/rear clearance, heading restoration, both-sensor alignment, rejection
of sideways/one-sensor success, transient detections, turn crossing/coast,
pacing, missed line, close echoes, unknown width,
insufficient starting space, timeouts, STOP and unchanged raw MQTT output.
Host tests cannot prove physical clearance, pivot calibration or line recovery.
