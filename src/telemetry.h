/** @file telemetry.h
 *
 * @brief Teleplot telemetry over USB serial.
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

void telemetry_task(void);
void telemetry_text(char const * p_name, char const * p_text);

#endif /* TELEMETRY_H */

/*** end of file ***/
