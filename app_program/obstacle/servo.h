/** @file servo.h
 * @brief Shared declarations and configuration for the SuperCar project.
 *
 * NOTE: Remaining standards deviations are tracked in BARR_C_REVIEW.md.
 */
#ifndef SERVO_H
#define SERVO_H

#include <stdbool.h>
#include <stdint.h>

void    servo_init(void);
void    servo_set_angle(int32_t degrees);
int32_t servo_angle(void);
bool    servo_is_settled(void);
void    servo_idle_release(bool b_allow);

#endif

/*** end of file ***/
