#ifndef SERVO_H
#define SERVO_H

#include <stdbool.h>
#include <stdint.h>

void servo_init(void);
void servo_set_angle(int32_t degrees);
int32_t servo_angle(void);
bool servo_is_settled(void);

#endif
