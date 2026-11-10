# Obstacle detection and avoidance phases

Target firmware: `supercar` (`src/`), producing `build/supercar.uf2`.
The older `robocar` manual-driving target is separate.

## Confirmed test setup

- Safe servo commands: 60° left, 90° forward, 120° right.
- Approximate car footprint: 160 mm long and 130 mm wide. Path planning
  must include the full footprint when turning, not just the sensor point.
- Expected obstacle width: at most 200–300 mm. Maximum front-to-back depth
  remains unconfirmed.
- The line behind the obstacle is straight, with space on both sides.
- Both wheel encoders work (user confirmed). Distance and turning calibration
  are not available yet. Historical wiring notes in `config.h` are not a
  statement that the current encoders are broken.

## Phase 1: stop and forward sensor ownership — implemented

- Brake when a measured forward distance is **400 mm or less**, including
  during initial line calibration, barcode turns and hump handling.
- Slow to creep speed within 500 mm while following the line.
- Centre the pan servo while driving and blocked. The automatic stopped
  scan owns the servo during its sweep. Ignore manual pan requests during
  a run, including while blocked and scanning.
- After changing the servo angle, keep the motors braked for a configurable
  300 ms settling interval and until a new ultrasonic measurement finishes.
  Discard old and in-flight measurements from the previous angle. Commands,
  including STOP, continue to be processed during settling.
- Stop, scan, then remain BLOCKED until avoidance is implemented. Resume
  line following after valid distances exceed 450 mm continuously for
  more than 200 ms. Missing or stale echoes cannot release BLOCKED.

The threshold is sensor-to-obstacle distance. Physical stopping distance
depends on control/sensor latency, braking, speed and sensor mounting.
The existing driver's no-echo policy still permits starting/driving when
there is no returned echo; a timeout alone cannot distinguish open space
from a disconnected sensor.

## Phase 2: scanning and profile — implemented

The vehicle enters OBSTACLE_SCAN with both wheels braked at the 400 mm
threshold. `servo.c` owns PWM and settling; `obstacle.c` owns scan scheduling
and geometry. The scanner measures the centre first, then coarse angles
every 10° across 60–120°, with three independent pings at each angle. It
refines foreground transitions every 2°. Repeated reads of the same ping
cannot count as extra samples. Inconsistent echoes produce unknown rays;
three timeouts produce a no-echo ray. A vanished/unstable centre terminates
the scan. A 20-second deadline terminates a stuck scan. STOP cancels it and
recentres the servo; it never waits for a scan to finish.

The foreground is the contiguous group around the centre within a configurable
80 mm forward-depth tolerance. Edge positions use the measured centre distance
and the midpoint of each angular edge bracket (`x = front * tan(angle)`).
This is a front-surface model, not a complete 3D reconstruction: slanted,
irregular, overlapping or strongly reflective targets can distort it.

The profile contains signed left/right positions (negative left, positive
right), angles, width estimate, angular width interval, raw angle/range points,
and valid endpoint ray distances. Both edge flags are required before using
width. A target extending beyond a sweep end has an unknown edge. No-echo
outside a foreground edge gives only a **tentative** edge estimate; both
brackets need stable background echoes for `edges_confirmed`. Neither status
proves that a full car-width corridor is clear. The raw points are available
through `obstacle_scan_points()` and logged on the web Obstacle / Hump tab.
The web card and Teleplot expose width and edge status.

The width interval expresses angular sampling resolution, not a guarantee
of physical accuracy. Ultrasonic beam width and surface reflection affect
edge accuracy; the picture's width-error target requires physical testing.

A stationary frontal scan cannot reliably measure the hidden rear face of
an obstacle. Its full front-to-back depth needs either a confirmed bound or
additional observations while passing it.

## Phase 3: bypass and recovery — pending movement calibration

Choose a side using measured clearance and the car's full footprint plus a
confirmed margin. Plan departure, lateral clearance, travel past the rear
of the obstacle, and approach back toward the original line. Reject plans
when either edge or required clearance is unknown. Establish calibrated
distance and turning control before relying on centimetre-based waypoints.
Because depth is unknown, this phase needs additional observations while
passing to establish the rear edge; it cannot simply travel 300 mm and
assume the obstacle has ended. This observation strategy is not implemented.

Continue checking the travel corridor during the bypass. Use bounded line
search and alignment after passing the obstacle, confirm the black line
over multiple readings, then reset line-following history and resume the
existing follower. STOP, blocked routes, missing progress and search timeout
must brake the car. Reverse actions require a confirmed clear rear path.

## Encoder calibration preparation — implemented

Use a USB serial monitor. With the car STOPPED, press **e**, roll the car
straight along the floor for exactly **300 mm**, then press **e** again.
The log reports raw LEFT/RIGHT edge deltas. Keep it stopped throughout;
lifting or spinning the wheels invalidates a ground-distance measurement.
Any vehicle command cancels an unfinished mark. Repeat three times and
send the reported counts. Nothing applies these values automatically.

This directly measures counts per distance; wheel diameter and counts per
revolution are unnecessary if the roll is measured accurately. Turning
also needs calibration: measure the lateral spacing between the two drive
wheel contact centres, then validate actual 90° turns on the floor. The
existing barcode turn timing alone is not a calibrated avoidance controller.

## Review and validation

Pass 1 found the original 250 mm stop threshold, servo centring only at boot,
and obstacle checks that calibration/turn states could bypass.

Pass 2 added the 400 mm priority stop and nonblocking servo settling with
fresh-measurement gating; review also prevented invalid echoes from releasing
the blocked state.

Phase 2 review pass 1 checked scan geometry using modelled 200/250/300 mm
targets at offsets -40/0/+40 mm, missing background echoes, an obstacle wider
than the sweep and inconsistent measurements. Modelled width errors stayed
within 10%; this does not establish real sensor accuracy.

Phase 2 review pass 2 exercised the real vehicle and scanner together:
401 mm permits following, 400 mm stops and scans, obstacle detection outranks
tilt, calibration and turns are protected, scanning never drives the wheels,
completion returns to BLOCKED, STOP cancels, invalid echoes cannot release
BLOCKED, valid sustained clearance resumes, and barcode test retains its
stop-only behaviour. Fresh-ping accounting, cancellation, timeout and timer
wrap also pass host checks.

Phase 2 review pass 3 checked the Pico W build and embedded web JavaScript
syntax. Ultrasonic driver checks also pass (400 mm conversion, invalidation,
late-edge rejection, minimum ping interval, stale readings and timeouts).
No physical car tests have been performed. The conservative scan takes
several seconds; it does not meet the picture's under-3-second full bypass
target. Settling and scan speed must be tuned from hardware measurements.

Run checks with `python3 tests/test_ultrasonic_host.py` and
`python3 tests/test_obstacle_host.py`; build with
`cmake --build build --target supercar -j 4`.

Hardware acceptance sequence:

1. Pan manually while stopped, then START: verify centring before movement;
   press STOP during settling and verify the car stays stopped.
2. Approach a target through 50/45/40/35 cm. Verify the brake command at or
   below 40 cm in normal following, calibration and barcode turns.
3. With BLOCKED active, remove echoes or unplug the sensor: verify no resume.
   Supply valid readings beyond 45 cm for over 200 ms: verify resume.
4. After Phase 2, measure known 20/25/30 cm targets at different lateral
   offsets and surfaces. Compare both estimated edges and width to ground
   truth; test obstacles clipped by the scan limits and noisy/missing echoes.
5. After Phase 3, run repeated left-clear/right-clear/both-blocked trials,
   including line-search failure and STOP during every phase. Log detection
   rate, width error, direction success, line reacquisition rate and bypass
   time, matching the picture's metrics. The top band requires 100% rates,
   width error at most 10%, and bypass under 3 seconds; these are unverified.

Self-assessment against the full prompt/picture: **6/10**. Stop, servo
ownership, coarse/fine scan, edge/width profile, logging, web display and
calibration preparation are implemented and checked. Full corridor validation,
path selection, bypass and line reacquisition remain unimplemented. Real
detection rate, width accuracy and timing remain unverified. The full solution
cannot honestly receive 9/10 until the remaining behaviour is implemented
and validated on the car.
