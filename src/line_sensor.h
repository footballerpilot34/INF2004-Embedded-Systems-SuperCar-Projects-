/** @file line_sensor.h
 *
 * @brief Left and right line IR sensors (digital DOUT).
 */

#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <stdbool.h>

typedef struct
{
    bool b_left_black;      // Left sensor is over the line.
    bool b_right_black;     // Right sensor is over the line.
} line_state_t;

typedef struct
{
    bool b_left_high;       // Raw left DOUT level.
    bool b_right_high;      // Raw right DOUT level.
} line_raw_t;

void         line_sensor_init(void);
line_state_t line_sensor_read(void);
line_raw_t   line_sensor_raw(void);
bool         line_sensor_is_driven(bool b_left);
void         line_sensor_set_black_level(bool b_left, bool b_black_is_high);
bool         line_sensor_black_level(bool b_left);

#endif /* LINE_SENSOR_H */

/*** end of file ***/
