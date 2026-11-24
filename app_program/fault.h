/** @file fault.h
 *
 * @brief One shared fault bitmask (change requirement 1): each fault
 *        condition is one bit of a single byte, not a separate variable.
 *        Also measures how long each control loop pass takes.
 */

#ifndef FAULT_H
#define FAULT_H

#include <stdbool.h>
#include <stdint.h>

// Bit numbers in the fault byte. Also used by the binary MQTT frames
// (mqtt_frame.h), so keep them in step with tools/mqtt_frame.py.
#define FAULT_MOTOR_STALL (1u << 0)  // Driven, but neither wheel turns.
#define FAULT_ENCODER (1u << 1)      // One wheel turns, its encoder is silent.
#define FAULT_IMU (1u << 2)          // IMU missing or its reading invalid.
#define FAULT_NO_ECHO (1u << 3)      // Last ultrasonic ping had no echo.
#define FAULT_WIFI_LOSS (1u << 4)    // Not on a hotspot.
#define FAULT_MQTT_DOWN (1u << 5)    // Not connected to the broker.
#define FAULT_BATTERY_LOW (1u << 6)  // VSYS below POWER_LOW_MV.
#define FAULT_LOOP_OVERRUN (1u << 7) // A loop pass took over TICK_USEC.
#define FAULT_BITS 8u                // Bits in the fault byte.

void     fault_init(void);
void     fault_set(uint32_t mask);
void     fault_clear(uint32_t mask);
void     fault_update(uint32_t mask, bool b_active);
uint32_t fault_get(void);
void     fault_task(uint32_t now_msec);
void     fault_loop_record(uint32_t work_usec);
uint32_t fault_loop_max_usec_take(void);
uint32_t fault_loop_overruns(void);

#endif /* FAULT_H */

/*** end of file ***/
