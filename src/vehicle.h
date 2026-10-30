/** @file vehicle.h
 *
 * @brief Mission state machine: line following, obstacle stop, humps.
 */

#ifndef VEHICLE_H
#define VEHICLE_H

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
    VEHICLE_BARCODE_TEST        // Drive straight, ignoring the line, and
                                // read barcodes.
} vehicle_state_t;

void            vehicle_init(void);
void            vehicle_start(void);
void            vehicle_stop(void);
void            vehicle_barcode_command(barcode_command_t command);
void            vehicle_barcode_test(void);
void            vehicle_task(void);
vehicle_state_t vehicle_state(void);
char const *    vehicle_state_name(vehicle_state_t state);

#endif /* VEHICLE_H */

/*** end of file ***/
