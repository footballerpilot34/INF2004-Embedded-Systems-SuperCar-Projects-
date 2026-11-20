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
pencil-ray, planar-front approximation, not a direct measurement of physical
obstacle width. At greater distance, the same angular span produces greater width.

The dashboard also shows **calibrated obstacle width** as 150 mm, 250 mm,
or unknown. The former bottle-based effective beam-angle fit has been removed
for these rectangular tests. The new lookup compares observed angular span
with physically measured span envelopes at the measured centre distance.
Uncalibrated, clipped, weak, ambiguous or incomplete scans remain unknown.
Raw range points and raw width are unchanged.

Configuration is in `src/config.h`: `OBSTACLE_WIDTH_CALIBRATION_ENABLED` and
`OBSTACLE_WIDTH_CALIBRATION_ROWS`. All table rows initially contain zero;
no rectangular calibration data are available yet. In the current fixed-width
experiment these results are telemetry only: the avoidance route assumes
200 mm width regardless of raw width, class or edge status. It still requires
a plausible forward obstacle; corridor clearance is an explicit operator
assumption for this controlled test.

See [OBSTACLE_DETECTION.md](OBSTACLE_DETECTION.md) for the fixed-width
route, staged physical testing, the confirmed 60 mm sensor setback and full
180×140 mm footprint. The stationary scan limit is now the 300 mm stop threshold. Do not
change safe mechanical servo limits or treat missing echoes as clearance.
