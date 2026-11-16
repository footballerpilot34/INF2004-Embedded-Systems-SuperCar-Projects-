# Ultrasonic Distance Scan

Open the existing SuperCar dashboard and press **SCAN NOW** under
**Ultrasonic Distance Scan**, below **Obstacle scan**. The same command is
available under **More controls → STATIONARY SCAN TEST** and serial `g`.
The car must be READY or STOPPED, with a current obstacle reading greater
than zero and no farther than `OBSTACLE_STOP_MM`. Rejections appear above
the graph. STOP/RESET cancels the scan and keeps the wheels braked.

Each scan measures the centre first, then goes outward left and right in
`OBSTACLE_SCAN_FINE_DEG` steps (currently 2°), covering the configured
`SERVO_CENTER_DEG ± SERVO_RANGE_DEG` (currently 60°–120°). Three ping
attempts are made per angle. One echo is retained as-is; multiple echoes
use their median (the mean of the two middle values for an even count).
Missing echoes are `null` in `/scan` and gaps in the Canvas graph.
Yellow dots denote sparse or inconsistent echoes; they remain actual
measurements, not interpolated values.

The first local forward-depth increase of `OBSTACLE_SCAN_EDGE_JUMP_MM`,
or the first missing echo, brackets an edge immediately. The displayed
angle is the midpoint between the last foreground ray and the first
background/missing ray. Width is estimated from the two edge angles and
their foreground distances. An obstacle that fills the scan range gets
explicit scan-limit estimates; its real edges may lie outside the range.
If no usable distance is obtained, width and edge markers are unavailable.
A missing centre uses the nearest available foreground ray for geometry,
or the pre-scan forward distance when none is available. That fallback is
never plotted as a successful echo and never verifies clearance.

The graph polls `/scan`, updates during scanning, and retains the previous
points after completion or cancellation until a fresh scan starts.
Status is Idle, Scanning, Complete, or Failed (including timeout/cancellation).
The original obstacle results and serial log remain available.

Edge estimation does not authorize bypass. The existing three-ray clearance
gate (`OBSTACLE_SCAN_EDGE_CONFIRM_RAYS`), stable-echo test
(`OBSTACLE_SCAN_STABLE_SPREAD_MM`), turning footprint and safety margin
remain in force. A geometric candidate still does not verify a driving
corridor or rear clearance; the vehicle remains braked. Manual scans reject
movement and other motor tests while active, and reserve the scan servo.

## Raw and calibrated width

The raw estimate remains unchanged:

```
theta = servo angle - SERVO_CENTER_DEG
D_L = radial echo at last left foreground ray * cos(that ray's theta)
D_R = radial echo at last right foreground ray * cos(that ray's theta)
W_raw = D_R * tan(right edge theta) - D_L * tan(left edge theta)
```

Angles are converted to radians before trigonometric calls. The midpoint of
each observed edge bracket supplies its edge theta. Each raw depth comes
from its own last foreground ray, so the two depths may differ. This is a
pencil-ray, planar-front approximation, not a direct measurement of bottle
diameter. At greater distance, the same angular span produces greater width.

The dashboard now also shows a **provisional calibrated width**. An effective
beam half-angle beta moves the apparent boundaries inward. It uses one
measured central range as the common forward-depth reference:

```
W_cal = centre_range * [tan(theta_right - beta) - tan(theta_left + beta)]
```

The first supplied summary (67/105 degrees, 193 mm raw, approximate 300 mm
centre range) and second (71/105 degrees, 125 mm raw, approximate 220 mm
centre range) give a least-squares effective beta of 9.8 degrees. This predicts
97.7 and 55.7 mm, not two forced 80 mm answers. Those residuals (+17.7/-24.3 mm)
are calibration fit errors on approximate inputs, NOT independent evidence
that hardware accuracy is within +/-50 mm. The missing per-ray echoes mean
the exact reported raw widths cannot be reconstructed uniquely. Regression
fixtures explicitly reconstruct compatible edge echoes rather than claiming
to replay complete hardware logs.

Configuration in `src/config.h`:

- `OBSTACLE_WIDTH_CALIBRATION_ENABLED`: disable to show raw width only.
- `OBSTACLE_WIDTH_BEAM_HALF_DEG_X10`: effective half-angle, currently 98.
- `OBSTACLE_WIDTH_CAL_MIN_MM` / `MAX_MM`: 200-320 mm calibration interval.

Calibration is unavailable outside that interval, for an unmeasured centre,
for scan-limit/uncertain edges, or when corrected edges overlap. Missing-echo
edges remain estimates. The graph's readings, markers and all safety geometry
use the original observations. Calibrated width is NEVER used for bypass,
turning clearance, expected-width checks or motor decisions.

To replace the provisional fit, measure the sensor's physical servo angles
and centre first. Collect at least five scans of measured-width reference
targets at allowed stationary start distances, such as 200, 240, 280 and
300 mm, including modest left/right offsets. The existing 300 mm scan-start
safety gate is unchanged; do not relax it to collect calibration data.
Use a clear background and the same mounting height and target material.
Record the actual centre echo and the two detected edge angles. Fit beta
with `python3 tests/calibrate_obstacle_width.py measurements.csv`, using CSV
columns `distance_mm,left_deg,right_deg,width_mm`. The example fit is
reproducible with `python3 tests/calibrate_obstacle_width.py --examples`.
Set the resulting constant and validate on separate scans/distances that
were NOT used in fitting. Check absolute errors against the +/-50 mm goal.
If residuals depend on range/offset/target shape, a single beam-angle model
does not suffice; retain raw and provisional labels until independent tests
justify a distance-dependent model or a different sensor.

HC-SR04 specifications list a nominal detecting angle, but it is not an
exact angular correction for every target. Beam patterns depend on target
and range: [HC-SR04 overview](https://learn.sparkfun.com/tutorials/qwiic-ultrasonic-distance-sensor-hc-sr04-hookup-guide/hardware-overview),
[manufacturer beam-pattern explanation](https://maxbotix.com/pages/reading-maxsonar-beam-patterns).
