/** @file buzzer.h
 *
 * @brief Robo Pico piezo buzzer (GP22): short non-blocking beeps.
 */

#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>

void buzzer_init(void);
void buzzer_beep(uint32_t duration_msec);
void buzzer_task(void);

#endif /* BUZZER_H */

/*** end of file ***/
