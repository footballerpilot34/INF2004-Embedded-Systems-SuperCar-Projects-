/** @file bypass.h
 *
 * @brief Drive round a scanned obstacle and back onto the same line.
 */

#ifndef BYPASS_H
#define BYPASS_H

#include <stdbool.h>
#include <stdint.h>

#include "obstacle.h"

typedef enum
{
    BYPASS_RUNNING, // Still going round.
    BYPASS_DONE,    // Back on the line: follow it.
    BYPASS_FAILED   // Could not finish: braked, stop the run.
} bypass_result_t;

bool bypass_begin(obstacle_profile_t const * p_profile, uint32_t now_msec,
                  bool b_use_line);
bypass_result_t bypass_step(uint32_t now_msec, uint32_t range_mm);
bool            bypass_test_begin(
    uint32_t now_msec); // Continuous dry rehearsal; always ends stopped.
bool         bypass_test_uses_line(void);
void         bypass_cancel(void);
char const * bypass_phase_name(void);
uint32_t     bypass_phase_id(void);

#endif /* BYPASS_H */

/*** end of file ***/
