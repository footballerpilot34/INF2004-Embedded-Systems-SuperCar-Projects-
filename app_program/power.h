/** @file power.h
 *
 * @brief Supply voltage (VSYS) watch, to spot a sagging battery.
 */

#ifndef POWER_H
#define POWER_H

#include <stdbool.h>
#include <stdint.h>

void     power_init(void);
void     power_task(void);
bool     power_is_measured(void);
uint32_t power_vsys_mv(void);
uint32_t power_vsys_min_mv(void);

#endif /* POWER_H */

/*** end of file ***/
