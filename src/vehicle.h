/** @file vehicle.h
 *
 * @brief Mission state machine: line following, obstacle bypass, humps.
 */

#ifndef VEHICLE_H
#define VEHICLE_H

#include <stdbool.h>
#include <stdint.h>

#include "barcode.h"

typedef enum
{
    VEHICLE_READY = 0,          // Waiting for START. Braked.
    VEHICLE_FOLLOW_LINE,        // Following the line.
    VEHICLE_PROBE,              // Something ahead: creep to tell hump/wall.
    VEHICLE_HUMP,               // Climbing over a hump.
    VEHICLE_BLOCKED,            // Stopped at a wall; resumes when clear.
    VEHICLE_STOPPED,            // Stopped by STOP. Braked.
    VEHICLE_LINE_CAL,           // START sweep: learn and check the line.
    VEHICLE_TURN,               // Barcode command: turn, then follow.
    VEHICLE_BARCODE_TEST,       // Drive straight, ignoring the line, and read.
    VEHICLE_OBSTACLE_SCAN,      // Stop and profile the obstacle with sonar.
    VEHICLE_OBSTACLE_BYPASS,    // Steer around it, then find the line again.
    VEHICLE_OBSTACLE_REACQUIRE  // Follow the line sensors back onto the track.
} vehicle_state_t;

void            vehicle_init(void);
void            vehicle_start(void);
void            vehicle_stop(void);
void            vehicle_barcode_command(barcode_command_t command);
void            vehicle_barcode_test(void);
void            vehicle_task(void);
vehicle_state_t vehicle_state(void);
char const *    vehicle_state_name(vehicle_state_t state);
int32_t         vehicle_hump_peak_tilt_x10(void);
bool            vehicle_servo_take(int32_t * p_degrees);
int32_t         vehicle_obstacle_width_mm(void);
int32_t         vehicle_obstacle_left_clearance_mm(void);
int32_t         vehicle_obstacle_right_clearance_mm(void);
int32_t         vehicle_obstacle_side(void);
int32_t         vehicle_obstacle_scan_angle_deg(void);
int32_t         vehicle_obstacle_scan_range_mm(void);

#endif /* VEHICLE_H */

/*** end of file ***/
