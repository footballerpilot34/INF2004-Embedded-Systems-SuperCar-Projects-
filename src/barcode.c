/** @file barcode.c
 *
 * @brief Code 39 barcode reader using the side IR sensor.
 *
 * @par How it works
 * 1. barcode_edge_isr() timestamps every black/white edge into a ring
 *    buffer; it does no decoding.
 * 2. barcode_task() turns the edges into element widths (bar, space, bar,
 *    ...). A scan starts on the first white-to-black edge and ends after
 *    a quiet gap (no edges) much longer than any element.
 * 3. The scan is decoded as Code 39: each character is 9 elements (5 bars,
 *    4 spaces), exactly 3 of them wide, followed by a narrow gap. The
 *    message must start and end with '*'. Widths are compared within each
 *    character only, so a steady speed is needed but its value is not.
 * 4. If a forward decode fails, the scan is decoded in reverse, so the
 *    car can cross the barcode in either direction.
 * 5. The message is checked against the command texts in config.h
 *    (BARCODE_TEXT_...): a match is a command (turn left, turn right,
 *    U-turn) that trumps the line sensors; anything else is a message.
 *
 * @par Flicker
 * The IR module can flicker as a bar edge passes under it. A colour that
 * lasts less than BARCODE_GLITCH_USEC is not a bar or space: it is merged
 * back into the element it interrupted.
 *
 * @par Reference
 * Code 39 symbology: ISO/IEC 16388.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "barcode.h"
#include "config.h"


#define EDGE_RING_SIZE          256u    // Power of two.
#define EDGE_RING_MASK          (EDGE_RING_SIZE - 1u)
#define MAX_ELEMENTS            200u    // 20 characters.
#define ELEMENTS_PER_CHAR       9u
#define ELEMENTS_WITH_GAP       10u     // 9 elements + inter-character gap.
#define WIDE_PER_CHAR           3u
#define MIN_CHARS               3u      // '*', one character, '*'.
#define MIN_ELEMENTS            ((MIN_CHARS * ELEMENTS_WITH_GAP) - 1u)
#define WIDE_RATIO_NUM          5u      // Narrowest wide must be at least
#define WIDE_RATIO_DEN          4u      // 1.25x the widest narrow.
#define START_STOP_CHAR         '*'
#define NUM_CODE39_SYMBOLS      44u
#define NUM_COMMAND_TEXTS       6u

typedef struct
{
    uint16_t pattern;   // 9 bits, first element in bit 8; 1 = wide.
    char     symbol;
} code39_t;

// Code 39 table: element order is bar, space, bar, space ... bar.
static code39_t const g_code39[NUM_CODE39_SYMBOLS] =
{
    { 0x034u, '0' }, { 0x121u, '1' }, { 0x061u, '2' }, { 0x160u, '3' },
    { 0x031u, '4' }, { 0x130u, '5' }, { 0x070u, '6' }, { 0x025u, '7' },
    { 0x124u, '8' }, { 0x064u, '9' }, { 0x109u, 'A' }, { 0x049u, 'B' },
    { 0x148u, 'C' }, { 0x019u, 'D' }, { 0x118u, 'E' }, { 0x058u, 'F' },
    { 0x00Du, 'G' }, { 0x10Cu, 'H' }, { 0x04Cu, 'I' }, { 0x01Cu, 'J' },
    { 0x103u, 'K' }, { 0x043u, 'L' }, { 0x142u, 'M' }, { 0x013u, 'N' },
    { 0x112u, 'O' }, { 0x052u, 'P' }, { 0x007u, 'Q' }, { 0x106u, 'R' },
    { 0x046u, 'S' }, { 0x016u, 'T' }, { 0x181u, 'U' }, { 0x0C1u, 'V' },
    { 0x1C0u, 'W' }, { 0x091u, 'X' }, { 0x190u, 'Y' }, { 0x0D0u, 'Z' },
    { 0x085u, '-' }, { 0x184u, '.' }, { 0x0C4u, ' ' }, { 0x094u, '*' },
    { 0x0A8u, '$' }, { 0x0A2u, '/' }, { 0x08Au, '+' }, { 0x02Au, '%' }
};

typedef struct
{
    char const *      p_text;
    barcode_command_t command;
} command_text_t;

// Messages that are commands (exact match); see config.h.
static command_text_t const g_command_texts[NUM_COMMAND_TEXTS] =
{
    { BARCODE_TEXT_LEFT,        BARCODE_CMD_LEFT  },
    { BARCODE_TEXT_LEFT_SHORT,  BARCODE_CMD_LEFT  },
    { BARCODE_TEXT_RIGHT,       BARCODE_CMD_RIGHT },
    { BARCODE_TEXT_RIGHT_SHORT, BARCODE_CMD_RIGHT },
    { BARCODE_TEXT_UTURN,       BARCODE_CMD_UTURN },
    { BARCODE_TEXT_UTURN_SHORT, BARCODE_CMD_UTURN }
};

// Edge ring, filled by the ISR. The head only ever increases (it wraps
// naturally at 2^32), so head - tail is the number of unread edges.
static volatile uint32_t g_edge_usec[EDGE_RING_SIZE];
static volatile bool     g_edge_black[EDGE_RING_SIZE];
static volatile uint32_t g_edge_head = 0u;
static uint32_t          g_edge_tail = 0u;

// Scan in progress.
static uint32_t g_elements[MAX_ELEMENTS];
static uint32_t g_num_elements   = 0u;
static uint32_t g_widest_usec    = 0u;
static uint32_t g_last_edge_usec = 0u;
static bool     gb_last_black    = false;
static bool     gb_capturing     = false;
static bool     gb_overflow      = false;
static bool     gb_enabled       = true;

static char              g_message[BARCODE_MESSAGE_MAX + 1u];
static barcode_command_t g_command = BARCODE_CMD_NONE;


static void             edge_add(uint32_t edge_usec, bool b_black);
static barcode_result_t scan_finish(void);
static bool             scan_decode(bool b_reverse);
static char             char_decode(uint32_t first, bool b_reverse);
static uint32_t         element_get(uint32_t index, bool b_reverse);
static barcode_command_t command_match(char const * p_message);


/*!
 * @brief Set up the barcode sensor pin.
 *
 * @par
 * The edge interrupt itself is enabled by gpio_isr_init().
 */
void
barcode_init (void)
{
    gpio_init(PIN_BARCODE);
    gpio_set_dir(PIN_BARCODE, GPIO_IN);
    gpio_disable_pulls(PIN_BARCODE);
    g_message[0] = '\0';
}   /* barcode_init() */


/*!
 * @brief Record one barcode edge. Called from the GPIO interrupt.
 *
 * @param[in] events The edge events (unused; the pin is read directly so a
 *                   glitch that rises and falls together is handled).
 */
void
barcode_edge_isr (uint32_t events)
{
    uint32_t head = g_edge_head;

    (void) events;  // The level is read from the pin instead.

    g_edge_usec[head & EDGE_RING_MASK]  = time_us_32();
    g_edge_black[head & EDGE_RING_MASK] = (BARCODE_BLACK_IS_HIGH ==
                                           gpio_get(PIN_BARCODE));
    g_edge_head = head + 1u;
}   /* barcode_edge_isr() */


/*!
 * @brief Process new edges and decode a scan once it has ended.
 *
 * @par
 * Call every control tick.
 *
 * @return BARCODE_DECODED when a new message is ready, BARCODE_INVALID
 *         when a scan could not be decoded, otherwise BARCODE_NONE.
 */
barcode_result_t
barcode_task (void)
{
    barcode_result_t result   = BARCODE_NONE;
    uint32_t         head     = g_edge_head;
    uint32_t         end_gap  = 0u;

    if ((head - g_edge_tail) > EDGE_RING_SIZE)
    {
        // The task fell too far behind; the scan has lost edges.
        g_edge_tail = head;
        gb_overflow = true;
    }

    while (g_edge_tail != head)
    {
        uint32_t idx = g_edge_tail & EDGE_RING_MASK;

        if (gb_enabled)
        {
            edge_add(g_edge_usec[idx], g_edge_black[idx]);
        }
        g_edge_tail++;
    }

    end_gap = g_widest_usec * BARCODE_END_GAP_FACTOR;
    end_gap = ((end_gap > BARCODE_END_GAP_MIN_USEC) ?
               end_gap : BARCODE_END_GAP_MIN_USEC);

    if (gb_capturing && ((time_us_32() - g_last_edge_usec) > end_gap))
    {
        result = scan_finish();
    }

    return (result);
}   /* barcode_task() */


/*!
 * @brief Turn barcode reading on or off.
 *
 * @par
 * Reading is switched off while climbing a hump or handling an obstacle,
 * where unrelated edges could be mistaken for bars.
 *
 * @param[in] b_enabled True to read barcodes.
 */
void
barcode_set_enabled (bool b_enabled)
{
    if (!b_enabled)
    {
        gb_capturing = false;
    }

    gb_enabled = b_enabled;
}   /* barcode_set_enabled() */


/*!
 * @brief Return the last decoded message (without the '*' characters).
 *
 * @return A null-terminated string, empty until the first decode.
 */
char const *
barcode_message (void)
{
    return (g_message);
}   /* barcode_message() */


/*!
 * @brief Return what the last decoded message asks the car to do.
 *
 * @return The command, or BARCODE_CMD_NONE for a plain message.
 */
barcode_command_t
barcode_command (void)
{
    return (g_command);
}   /* barcode_command() */


/*!
 * @brief Name a barcode command for the log and the web page.
 *
 * @param[in] command The command.
 *
 * @return A constant string.
 */
char const *
barcode_command_name (barcode_command_t command)
{
    char const * p_name = "MESSAGE";

    switch (command)
    {
    case BARCODE_CMD_LEFT:
        p_name = "TURN LEFT";
    break;

    case BARCODE_CMD_RIGHT:
        p_name = "TURN RIGHT";
    break;

    case BARCODE_CMD_UTURN:
        p_name = "U-TURN";
    break;

    case BARCODE_CMD_NONE:
    default:
        p_name = "MESSAGE";
    break;
    }

    return (p_name);
}   /* barcode_command_name() */


/*!
 * @brief Report whether a barcode is being read right now.
 *
 * @par
 * True from the first bar until the quiet gap after the last one. The car
 * drives straight and steady meanwhile, since widths are measured in time.
 *
 * @return True while a scan is in progress.
 */
bool
barcode_is_capturing (void)
{
    return (gb_capturing);
}   /* barcode_is_capturing() */


/*!
 * @brief Report whether the barcode sensor sees black right now.
 *
 * @return True over a bar.
 */
bool
barcode_is_black (void)
{
    return (BARCODE_BLACK_IS_HIGH == gpio_get(PIN_BARCODE));
}   /* barcode_is_black() */


/*!
 * @brief Add one edge to the current scan, or start a scan on a bar.
 *
 * @param[in] edge_usec When the edge happened.
 * @param[in] b_black   True if the sensor sees black after the edge.
 */
static void
edge_add (uint32_t edge_usec, bool b_black)
{
    if (!gb_capturing)
    {
        // A scan starts at the leading edge of the first bar.
        if (b_black)
        {
            gb_capturing     = true;
            gb_overflow      = false;
            gb_last_black    = true;
            g_num_elements   = 0u;
            g_widest_usec    = 0u;
            g_last_edge_usec = edge_usec;
        }
    }
    else if (b_black == gb_last_black)
    {
        // Same colour twice: a glitch too short to measure. Ignore it.
    }
    else if ((edge_usec - g_last_edge_usec) < BARCODE_GLITCH_USEC)
    {
        // The last colour flickered for too short a time to be a bar or a
        // space: undo it, so the element before it simply continues.
        if (0u == g_num_elements)
        {
            gb_capturing = false;   // The "first bar" was only a flicker.
        }
        else
        {
            g_num_elements--;
            g_last_edge_usec -= g_elements[g_num_elements];
            gb_last_black     = b_black;
        }
    }
    else
    {
        uint32_t width = edge_usec - g_last_edge_usec;

        if (g_num_elements < MAX_ELEMENTS)
        {
            g_elements[g_num_elements] = width;
            g_num_elements++;
        }
        else
        {
            gb_overflow = true;
        }

        g_widest_usec    = ((width > g_widest_usec) ? width
                                                    : g_widest_usec);
        g_last_edge_usec = edge_usec;
        gb_last_black    = b_black;
    }
}   /* edge_add() */


/*!
 * @brief Decode the finished scan, forwards and then backwards.
 *
 * @return BARCODE_DECODED, BARCODE_INVALID, or BARCODE_NONE if the scan
 *         was too short to be a barcode (for example, a line crossing).
 */
static barcode_result_t
scan_finish (void)
{
    barcode_result_t result = BARCODE_NONE;

    gb_capturing = false;

    if (gb_overflow)
    {
        result = BARCODE_INVALID;
    }
    else if (g_num_elements < MIN_ELEMENTS)
    {
        // Too few edges for even "*X*": not a barcode.
    }
    else if (scan_decode(false) || scan_decode(true))
    {
        result = BARCODE_DECODED;
    }
    else
    {
        result = BARCODE_INVALID;
    }

    g_widest_usec = 0u;

    return (result);
}   /* scan_finish() */


/*!
 * @brief Decode the scan as Code 39 and store the message if valid.
 *
 * @param[in] b_reverse True to read the elements last-to-first.
 *
 * @return True if the scan was a valid "*...*" Code 39 message.
 */
static bool
scan_decode (bool b_reverse)
{
    char     decoded[BARCODE_MESSAGE_MAX + 1u] = { 0 };
    uint32_t num_chars = (g_num_elements + 1u) / ELEMENTS_WITH_GAP;
    uint32_t out_len   = 0u;
    bool     b_ok      = (0u == ((g_num_elements + 1u) % ELEMENTS_WITH_GAP));

    for (uint32_t ch = 0u; (ch < num_chars) && b_ok; ch++)
    {
        char symbol   = char_decode(ch * ELEMENTS_WITH_GAP, b_reverse);
        bool b_is_end = ((0u == ch) || ((num_chars - 1u) == ch));

        if ('\0' == symbol)
        {
            b_ok = false;
        }
        else if (b_is_end)
        {
            b_ok = (START_STOP_CHAR == symbol);
        }
        else if ((START_STOP_CHAR == symbol) ||
                 (out_len >= BARCODE_MESSAGE_MAX))
        {
            b_ok = false;
        }
        else
        {
            decoded[out_len] = symbol;
            out_len++;
        }
    }

    if (b_ok)
    {
        decoded[out_len] = '\0';
        (void) strcpy(g_message, decoded);  // Lengths match; result unused.
        g_command = command_match(g_message);
    }

    return (b_ok);
}   /* scan_decode() */


/*!
 * @brief Decode one 9-element character by finding its 3 widest elements.
 *
 * @param[in] first     Index of the character's first element.
 * @param[in] b_reverse True to read the elements last-to-first.
 *
 * @return The character, or '\0' if the pattern is invalid or ambiguous.
 */
static char
char_decode (uint32_t first, bool b_reverse)
{
    uint16_t pattern     = 0u;
    uint32_t widest_narrow = 0u;
    uint32_t narrowest_wide = UINT32_MAX;
    char     symbol      = '\0';

    // Mark the 3 widest elements as wide.
    for (uint32_t pick = 0u; pick < WIDE_PER_CHAR; pick++)
    {
        uint32_t best_idx   = 0u;
        uint32_t best_width = 0u;

        for (uint32_t el = 0u; el < ELEMENTS_PER_CHAR; el++)
        {
            uint16_t bit   = (uint16_t) (1u << (ELEMENTS_PER_CHAR - 1u - el));
            uint32_t width = element_get(first + el, b_reverse);

            if ((0u == (pattern & bit)) && (width > best_width))
            {
                best_width = width;
                best_idx   = el;
            }
        }

        pattern |= (uint16_t) (1u << (ELEMENTS_PER_CHAR - 1u - best_idx));
    }

    // Check the wide and narrow groups are clearly different.
    for (uint32_t el = 0u; el < ELEMENTS_PER_CHAR; el++)
    {
        uint16_t bit   = (uint16_t) (1u << (ELEMENTS_PER_CHAR - 1u - el));
        uint32_t width = element_get(first + el, b_reverse);

        if (0u != (pattern & bit))
        {
            narrowest_wide = ((width < narrowest_wide) ? width
                                                       : narrowest_wide);
        }
        else
        {
            widest_narrow = ((width > widest_narrow) ? width
                                                     : widest_narrow);
        }
    }

    if ((narrowest_wide * WIDE_RATIO_DEN) >= (widest_narrow * WIDE_RATIO_NUM))
    {
        for (uint32_t idx = 0u; idx < NUM_CODE39_SYMBOLS; idx++)
        {
            if (g_code39[idx].pattern == pattern)
            {
                symbol = g_code39[idx].symbol;
            }
        }
    }

    return (symbol);
}   /* char_decode() */


/*!
 * @brief Read one element width, from either end of the scan.
 *
 * @param[in] index     Position counted from the chosen end.
 * @param[in] b_reverse True to count from the last element.
 *
 * @return The element width in microseconds.
 */
static uint32_t
element_get (uint32_t index, bool b_reverse)
{
    uint32_t actual = (b_reverse ? ((g_num_elements - 1u) - index) : index);

    return (g_elements[actual]);
}   /* element_get() */


/*!
 * @brief Check whether a message is one of the command texts.
 *
 * @param[in] p_message The decoded message.
 *
 * @return The command, or BARCODE_CMD_NONE if it is a plain message.
 */
static barcode_command_t
command_match (char const * p_message)
{
    barcode_command_t command = BARCODE_CMD_NONE;

    for (uint32_t idx = 0u; idx < NUM_COMMAND_TEXTS; idx++)
    {
        if (0 == strcmp(p_message, g_command_texts[idx].p_text))
        {
            command = g_command_texts[idx].command;
        }
    }

    return (command);
}   /* command_match() */

/*** end of file ***/
