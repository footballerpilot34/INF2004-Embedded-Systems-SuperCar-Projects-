/** @file ultrasonic.h
 *
 * @brief HC-SR04 (3.3 V) forward range sensor, timed by edge interrupt.
 */

#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdbool.h>
#include <stdint.h>

// Returned when there is no echo, the target is out of range, or the last
// reading is stale.
#define ULTRASONIC_NO_RANGE     UINT32_MAX

void     ultrasonic_init(void);
void     ultrasonic_echo_isr(uint32_t events);
void     ultrasonic_task(void);
uint32_t ultrasonic_range_mm(void);
// Drop measurements taken at an old servo angle; wait for a new ping.
void     ultrasonic_invalidate(void);
bool     ultrasonic_measurement_ready(void);
// Increments once per completed ping (including a no-echo timeout).
uint32_t ultrasonic_sample_sequence(void);

#endif /* ULTRASONIC_H */

/*** end of file ***/
