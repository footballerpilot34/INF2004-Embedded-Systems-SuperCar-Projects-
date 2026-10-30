/** @file barcode.h
 *
 * @brief Code 39 barcode reader using the side IR sensor.
 */

#ifndef BARCODE_H
#define BARCODE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    BARCODE_NONE,       // Nothing finished this tick.
    BARCODE_DECODED,    // A valid message is ready: see barcode_message().
    BARCODE_INVALID     // A scan finished but could not be decoded.
} barcode_result_t;

// What a decoded barcode asks the car to do. Commands trump the line
// sensors; anything that is not a command is a plain message.
typedef enum
{
    BARCODE_CMD_NONE,   // A plain message: show it, keep following.
    BARCODE_CMD_LEFT,   // Turn left about 90 degrees.
    BARCODE_CMD_RIGHT,  // Turn right about 90 degrees.
    BARCODE_CMD_UTURN   // Turn round and follow the line back.
} barcode_command_t;

void              barcode_init(void);
void              barcode_edge_isr(uint32_t events);
barcode_result_t  barcode_task(void);
void              barcode_set_enabled(bool b_enabled);
char const *      barcode_message(void);
barcode_command_t barcode_command(void);
char const *      barcode_command_name(barcode_command_t command);
bool              barcode_is_black(void);
bool              barcode_is_capturing(void);

#endif /* BARCODE_H */

/*** end of file ***/
