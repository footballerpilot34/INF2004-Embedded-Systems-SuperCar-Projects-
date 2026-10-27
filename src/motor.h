/** @file motor.h
 *
 * @brief Left and right wheel motor driver (Robo Pico, 20 kHz PWM).
 */

#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

void    motor_init(void);
void    motor_set(int32_t left, int32_t right);
int32_t motor_left_speed(void);
int32_t motor_right_speed(void);

#endif /* MOTOR_H */

/*** end of file ***/
