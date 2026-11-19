/** @file hump.h
 *
 * @brief Hump height: how high the car climbs, from pitch and distance.
 *        Heights are in tenths of a mm.
 */

#ifndef HUMP_H
#define HUMP_H

#include <stdbool.h>
#include <stdint.h>

void     hump_task(bool b_forward, bool b_on_hump);
void     hump_finish(int32_t peak_tilt_x10);
void     hump_new_run(void);
int32_t  hump_height_x10(void);
int32_t  hump_on_mm(void);
int32_t  hump_last_height_x10(void);
int32_t  hump_last_tilt_x10(void);
int32_t  hump_run_height_x10(void);
int32_t  hump_run_tilt_x10(void);
uint32_t hump_count(void);

#endif /* HUMP_H */

/*** end of file ***/
