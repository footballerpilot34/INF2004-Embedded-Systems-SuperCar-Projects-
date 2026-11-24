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
