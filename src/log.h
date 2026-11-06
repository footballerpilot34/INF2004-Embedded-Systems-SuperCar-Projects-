/** @file log.h
 *
 * @brief Status messages: printed to USB serial and kept for the web UI,
 *        each with a category the web page's serial monitor filters on.
 */

#ifndef LOG_H
#define LOG_H

#include <stdbool.h>
#include <stdint.h>

// Serial monitor tabs. Every message shows on "All"; these pick the others.
// NOTE: The page uses the numbers, so keep them in step with web.c.
typedef enum
{
    LOG_CAT_SYSTEM   = 0,   // Start-up, Wi-Fi, buttons, tests: "All" only.
    LOG_CAT_OBSTACLE = 1,   // Obstacle and hump detection.
    LOG_CAT_LINE     = 2,   // Line following, barcodes and crossroads.
    LOG_CAT_STATE    = 3    // Vehicle state changes.
} log_cat_t;

void     log_printf(char const * p_format, ...);
void     log_obstacle(char const * p_format, ...);
void     log_line(char const * p_format, ...);
void     log_state(char const * p_format, ...);
uint32_t log_next_seq(void);
bool     log_get(uint32_t seq, char const ** pp_line, log_cat_t * p_cat);

#endif /* LOG_H */

/*** end of file ***/
