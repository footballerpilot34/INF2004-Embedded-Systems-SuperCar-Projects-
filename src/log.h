/** @file log.h
 *
 * @brief Status messages: printed to USB serial and kept for the web UI.
 */

#ifndef LOG_H
#define LOG_H

#include <stdbool.h>
#include <stdint.h>

void     log_printf(char const * p_format, ...);
uint32_t log_next_seq(void);
bool     log_get(uint32_t seq, char const ** pp_line);

#endif /* LOG_H */

/*** end of file ***/
