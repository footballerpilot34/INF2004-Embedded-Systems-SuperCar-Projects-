# Main integration verification

Integrated origin/main 6954eea into the audited firmware. Preserved the pickup
STOPPED handoff, IMU lift telemetry and kick logs, alongside all bypass and
sensor-reporting changes. HUMP_ENTER_HEIGHT_X10 remains 10 (1.0 mm).
All seven host suites, independent footprints and the UF2 build pass.
Existing vendor lwIP compiler warnings remain. The two reporting issues from
the read-only review were not changed by this merge.
Final verification: main-merge-tests.txt and main-merge-build.txt.
