/** @file stage.h
 *
 * @brief Freeze finder: records which part of the main loop is running in
 *        a watchdog scratch register, which survives a watchdog restart.
 *
 * @par
 * If the loop stops, the watchdog restarts the Pico, and the next boot
 * reads the last stage recorded (see restart_reason() in supercar_main.c)
 * to report where it froze. Scratch registers 0-3 are free for programs;
 * the SDK uses 4-7. A power cut clears them.
 */

#ifndef STAGE_H
#define STAGE_H

#include <stdint.h>

#include "hardware/structs/watchdog.h"

#define STAGE_MAGIC         0x57A6E000u     // Marks a stage we wrote.
#define STAGE_MAGIC_MASK    0xFFFFF000u
#define STAGE_SCRATCH       0u              // Which part of the loop.
#define STAGE_TIME_SCRATCH  1u              // ms since boot, last tick.
#define STAGE_FAULT_SCRATCH 2u              // lwIP message (pointer).
#define STAGE_KIND_SCRATCH  3u              // Which kind of crash.
#define STAGE_KIND_ASSERT   0xA55E7000u     // lwIP check failed.
#define STAGE_KIND_HARDFAULT 0x4A4DF000u    // CPU hard fault.

typedef enum
{
    STAGE_NONE = 0,
    STAGE_WEB_POLL,         // cyw43_arch_poll(): Wi-Fi chip and lwIP.
    STAGE_WEB_SERVER,       // Web server listener check.
    STAGE_WEB_HOTSPOT,      // Hotspot join / recovery state machine.
    STAGE_POWER,
    STAGE_BUTTONS,
    STAGE_SERIAL,
    STAGE_COMMANDS,         // Running a button/serial/web command.
    STAGE_SERVO,
    STAGE_ULTRASONIC,
    STAGE_IMU,
    STAGE_BARCODE,
    STAGE_VEHICLE,
    STAGE_SCAN_TEST,
    STAGE_LINE_DIAG,
    STAGE_BUZZER,
    STAGE_TELEMETRY,
    STAGE_MQTT,             // MQTT connection and publishing.
    STAGE_TICK_WAIT,        // Idle, waiting for the next 10 ms tick.
    STAGE_COUNT
} stage_t;

/*!
 * @brief Record the part of the loop about to run.
 *
 * @param[in] stage The stage.
 */
static inline void
stage_set (stage_t stage)
{
    watchdog_hw->scratch[STAGE_SCRATCH] = STAGE_MAGIC | (uint32_t) stage;
}   /* stage_set() */

#endif /* STAGE_H */

/*** end of file ***/
