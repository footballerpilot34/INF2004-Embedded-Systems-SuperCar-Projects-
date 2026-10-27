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

void             barcode_init(void);
void             barcode_edge_isr(uint32_t events);
barcode_result_t barcode_task(void);
void             barcode_set_enabled(bool b_enabled);
char const *     barcode_message(void);
bool             barcode_is_black(void);

#endif /* BARCODE_H */

/*** end of file ***/
