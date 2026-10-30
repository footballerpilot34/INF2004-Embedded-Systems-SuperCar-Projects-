/** @file telemetry.c
 *
 * @brief Teleplot telemetry over USB serial.
 *
 * @par Format
 * Teleplot (VS Code extension) graphs every line of the form
 * ">name:value" and shows ">name:text|t" as a text field. Any other line
 * appears in its console, so status messages can be mixed in.
 *
 * @par Graphs
 * line_L, line_R, barcode_IR (1 = black); line_L_raw, line_R_raw (raw
 * pin level, 1 = HIGH); range_cm (-1 = nothing ahead);
 * motor_L, motor_R (percent); state
 * (number). Text fields: state_name and barcode (last message).
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "barcode.h"
#include "line_sensor.h"
#include "motor.h"
#include "telemetry.h"
#include "ultrasonic.h"
#include "vehicle.h"


#define TENTHS              10
#define NO_RANGE_PLOT       (-1)
#define STATE_TEXT_EVERY    20u     // Resend the state name every 1 s.


static void value_print(char const * p_name, int32_t value);
static void tenths_print(char const * p_name, int32_t value_x10);


/*!
 * @brief Send one set of Teleplot points. Call every TELEMETRY_EVERY_TICKS.
 */
void
telemetry_task (void)
{
    static uint32_t        report_count = 0u;
    static vehicle_state_t last_state   = VEHICLE_READY;
    line_state_t           line         = line_sensor_read();
    line_raw_t             raw          = line_sensor_raw();
    uint32_t               range_mm     = ultrasonic_range_mm();
    vehicle_state_t        state        = vehicle_state();

    value_print("line_L", (line.b_left_black ? 1 : 0));
    value_print("line_R", (line.b_right_black ? 1 : 0));
    value_print("line_L_raw", (raw.b_left_high ? 1 : 0));
    value_print("line_R_raw", (raw.b_right_high ? 1 : 0));
    value_print("barcode_IR", (barcode_is_black() ? 1 : 0));
    value_print("barcode_AO", barcode_level());

    if (ULTRASONIC_NO_RANGE == range_mm)
    {
        value_print("range_cm", NO_RANGE_PLOT);
    }
    else
    {
        // NOTE: The range is at most about 5000 mm, so it fits int32.
        tenths_print("range_cm", (int32_t) range_mm);
    }

    value_print("motor_L", motor_left_speed());
    value_print("motor_R", motor_right_speed());
    value_print("state", (int32_t) state);

    // Text only needs sending when it changes, plus a refresh each second
    // in case Teleplot was opened late.
    if ((state != last_state) || (0u == (report_count % STATE_TEXT_EVERY)))
    {
        telemetry_text("state_name", vehicle_state_name(state));
        last_state = state;
    }

    report_count++;
}   /* telemetry_task() */


/*!
 * @brief Send a Teleplot text field, such as ">barcode:HELLO|t".
 *
 * @param[in] p_name The field name.
 * @param[in] p_text The text to show.
 */
void
telemetry_text (char const * p_name, char const * p_text)
{
    printf(">%s:%s|t\n", p_name, p_text);
}   /* telemetry_text() */


/*!
 * @brief Send one whole-number Teleplot point.
 *
 * @param[in] p_name The graph name.
 * @param[in] value  The value.
 */
static void
value_print (char const * p_name, int32_t value)
{
    printf(">%s:%" PRId32 "\n", p_name, value);
}   /* value_print() */


/*!
 * @brief Send one Teleplot point with one decimal place.
 *
 * @param[in] p_name    The graph name.
 * @param[in] value_x10 The value in tenths (mm for cm, tenths for deg).
 */
static void
tenths_print (char const * p_name, int32_t value_x10)
{
    int32_t magnitude = ((value_x10 < 0) ? -value_x10 : value_x10);

    printf(">%s:%s%" PRId32 ".%" PRId32 "\n", p_name,
           ((value_x10 < 0) ? "-" : ""), magnitude / TENTHS,
           magnitude % TENTHS);
}   /* tenths_print() */

/*** end of file ***/
