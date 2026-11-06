/** @file log.c
 *
 * @brief Status messages: printed to USB serial and kept for the web UI.
 *
 * @par
 * Each message gets a sequence number and a category (log_cat_t) and is
 * kept in a ring of the last LOG_LINES messages. The web page asks for
 * "everything after sequence N", so it shows every message without
 * needing the USB cable, and sorts them into its serial monitor tabs.
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


static char      g_lines[LOG_LINES][LOG_LINE_MAX];
static log_cat_t g_cats[LOG_LINES];
static uint32_t  g_next_seq = 0u;


static void log_vprintf(log_cat_t cat, char const * p_format, va_list args);


/*!
 * @brief Print a general (start-up, Wi-Fi, button, test) message.
 *
 * @param[in] p_format A printf format string, then its arguments.
 */
void
log_printf (char const * p_format, ...)
{
    va_list args;

    va_start(args, p_format);
    log_vprintf(LOG_CAT_SYSTEM, p_format, args);
    va_end(args);
}   /* log_printf() */


/*!
 * @brief Print an obstacle or hump detection message.
 *
 * @param[in] p_format A printf format string, then its arguments.
 */
void
log_obstacle (char const * p_format, ...)
{
    va_list args;

    va_start(args, p_format);
    log_vprintf(LOG_CAT_OBSTACLE, p_format, args);
    va_end(args);
}   /* log_obstacle() */


/*!
 * @brief Print a line following, barcode or crossroad message.
 *
 * @param[in] p_format A printf format string, then its arguments.
 */
void
log_line (char const * p_format, ...)
{
    va_list args;

    va_start(args, p_format);
    log_vprintf(LOG_CAT_LINE, p_format, args);
    va_end(args);
}   /* log_line() */


/*!
 * @brief Print a vehicle state message.
 *
 * @param[in] p_format A printf format string, then its arguments.
 */
void
log_state (char const * p_format, ...)
{
    va_list args;

    va_start(args, p_format);
    log_vprintf(LOG_CAT_STATE, p_format, args);
    va_end(args);
}   /* log_state() */


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
 * @param[out] p_cat   Set to the message's category if found.
 *
 * @return True if the message is still in the ring.
 */
bool
log_get (uint32_t seq, char const ** pp_line, log_cat_t * p_cat)
{
    bool b_found = ((seq < g_next_seq) && ((g_next_seq - seq) <= LOG_LINES));

    if (b_found)
    {
        *pp_line = g_lines[seq % LOG_LINES];
        *p_cat   = g_cats[seq % LOG_LINES];
    }

    return (b_found);
}   /* log_get() */


/*!
 * @brief Keep a message with its category and print it to USB serial.
 *
 * @param[in] cat      The message's category.
 * @param[in] p_format A printf format string.
 * @param[in] args     Its arguments.
 */
static void
log_vprintf (log_cat_t cat, char const * p_format, va_list args)
{
    char * p_line = g_lines[g_next_seq % LOG_LINES];
    size_t length = 0u;

    (void) vsnprintf(p_line, LOG_LINE_MAX, p_format, args);    // Cut if long.
    g_cats[g_next_seq % LOG_LINES] = cat;

    // Keep the stored copy on one line; serial gets exactly one newline.
    length = strlen(p_line);
    while ((length > 0u) && ('\n' == p_line[length - 1u]))
    {
        length--;
        p_line[length] = '\0';
    }

    printf("%s\n", p_line);
    g_next_seq++;
}   /* log_vprintf() */

/*** end of file ***/
