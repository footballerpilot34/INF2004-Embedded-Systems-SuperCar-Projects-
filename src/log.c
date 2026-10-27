/** @file log.c
 *
 * @brief Status messages: printed to USB serial and kept for the web UI.
 *
 * @par
 * Each message gets a sequence number and is kept in a ring of the last
 * LOG_LINES messages. The web page asks for "everything after sequence N",
 * so it shows every message without needing the USB cable.
 *
 * @par
 * Only the main loop calls these functions (never an ISR), so no locking
 * is needed.
 */

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "log.h"


#define LOG_LINES       32u     // Messages kept for the web UI.
#define LOG_LINE_MAX    96u     // Longest message kept (longer is cut).


static char     g_lines[LOG_LINES][LOG_LINE_MAX];
static uint32_t g_next_seq = 0u;


/*!
 * @brief Print a status message and keep it for the web UI.
 *
 * @param[in] p_format A printf format string, then its arguments.
 */
void
log_printf (char const * p_format, ...)
{
    char *  p_line = g_lines[g_next_seq % LOG_LINES];
    size_t  length = 0u;
    va_list args;

    va_start(args, p_format);
    (void) vsnprintf(p_line, LOG_LINE_MAX, p_format, args);  // Cut if long.
    va_end(args);

    // Keep the stored copy on one line; serial gets exactly one newline.
    length = strlen(p_line);
    while ((length > 0u) && ('\n' == p_line[length - 1u]))
    {
        length--;
        p_line[length] = '\0';
    }

    printf("%s\n", p_line);
    g_next_seq++;
}   /* log_printf() */


/*!
 * @brief Report the sequence number the next message will get.
 *
 * @return The next sequence number.
 */
uint32_t
log_next_seq (void)
{
    return (g_next_seq);
}   /* log_next_seq() */


/*!
 * @brief Fetch a message by sequence number, if it is still kept.
 *
 * @param[in]  seq     The sequence number.
 * @param[out] pp_line Set to the message text if found.
 *
 * @return True if the message is still in the ring.
 */
bool
log_get (uint32_t seq, char const ** pp_line)
{
    bool b_found = ((seq < g_next_seq) && ((g_next_seq - seq) <= LOG_LINES));

    if (b_found)
    {
        *pp_line = g_lines[seq % LOG_LINES];
    }

    return (b_found);
}   /* log_get() */

/*** end of file ***/
