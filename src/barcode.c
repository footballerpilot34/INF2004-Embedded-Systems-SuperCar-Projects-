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
 *    (BARCODE_TEXT_...): a match is a command (turn left or right at the
 *    next crossroad, U-turn); anything else is a message.
 *
 * @par Long black marks
 * A scan is never dropped while still on a long first bar: when the thin
 * bars at a barcode's start are missed, the first bar seen can be a wide
 * one. A scan only ends after a quiet gap on white (or
 * BARCODE_BLACK_MAX_USEC on black). Only once a first mark has ended, and
 * lasted as long as a crossroad branch (LINE_JUNCTION_MIN_MSEC, which
 * line_sensor.c reports), is the scan dropped, so a barcode soon after a
 * crossroad is still read on its own.
 *
 * @par Analog reading (BARCODE_USE_ANALOG)
 * The module's comparator output misses thin bars, so a timer samples the
 * analog output every BARCODE_AO_SAMPLE_USEC and finds edges by peak and
 * dip tracking: a rise of BARCODE_AO_DELTA above the last dip is a bar, a
 * fall of BARCODE_AO_DELTA below the last peak is a space. This needs no
 * fixed black/white level, so faint thin bars, faded print and room light
 * changes are all handled. The edges go into the same ring as before.
 *
 * @par Flicker
 * The IR module can flicker as a bar edge passes under it. A colour that
 * lasts less than BARCODE_GLITCH_USEC is not a bar or space: it is merged
 * back into the element it interrupted.
 *
 * @par Reference
 * Code 39 symbology: ISO/IEC 16388.
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"

#include "barcode.h"
#include "config.h"
#include "encoder.h"
#include "log.h"


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
#define REPORT_MIN_ELEMENTS     4u      // Shorter scans are not reported.
#define DUMP_PER_LINE           10u     // Widths per log line (fits 96).
#define DUMP_LINE_BYTES         96u     // Must match LOG_LINE_MAX in log.c.
#define USEC_PER_MSEC           1000u
#define DUMP_TENTHS             10u     // Distance widths in 0.1 ticks.
#define BRANCH_MIN_USEC        (LINE_JUNCTION_MIN_MSEC * USEC_PER_MSEC)

// Lenient decode of one letter, "*X*" (see scan_decode_lenient()).
#define LENIENT_ELEMENTS        29u     // '*' gap letter gap '*'.
#define LENIENT_LETTER_FIRST    10u     // The letter is elements 10..18.
#define LENIENT_GAP_A           9u      // Gaps between the characters.
#define LENIENT_GAP_B           19u
#define LENIENT_STOP_FIRST      20u     // The second '*' is 20..28.
#define LENIENT_MAX_MISSING     4u      // Elements one end may lose...
#define LENIENT_MAX_MISSING_ALL 4u      // ...and both ends together, or
#define LENIENT_MAX_ONE_SIDED   8u      // one end alone while the other
                                        // '*' is complete (gap kept).
#define LENIENT_RATIO_X10       20u     // Wide >= 2.0x narrow (same kind).
#define LENIENT_BAND_PERCENT    20u     // Within 20% of the narrow/wide
                                        // split: too unclear to trust.
#define LENIENT_LONG_X          2u      // A cut-off end element may not be
                                        // longer than 2x the split.
#define LENIENT_SUM_MIN_PERCENT 70u     // An end's elements must add up to
#define LENIENT_SUM_MAX_PERCENT 140u    // 70-140% of their expected width.
#define BARS_PER_CHAR           5u
#define WIDE_BARS_PER_CHAR      2u      // A-Z and '*': 2 wide bars and
#define UNITS_PER_CHAR          15u     // 1 wide space = 6x1 + 3x3 units.
#define WIDE_UNITS              3u
#define PERCENT_100             100u
#define STAR_PATTERN            0x094u  // '*' in g_code39.
#define ADC_FIRST_PIN           26u     // ADC0 is GP26, ADC1 GP27, ...
#define ADC_FULL_SCALE          4095    // 12-bit ADC.
#define AO_FILTER_DIV           8       // Smoothing: 1/8 of each change
                                        // (1.6 ms; a bar lasts 10+ ms).
#define AO_WHITE_DIV            64      // Paper-white tracking (13 ms).
#define AO_GUARD_DIV            2       // Guard band: DELTA / 2 above white.

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

// Narrow/wide splits learned from a cleanly decoded letter.
typedef struct
{
    uint16_t pattern;       // The letter's pattern.
    uint32_t bar_split;     // Bars wider than this are wide.
    uint32_t space_split;   // Spaces wider than this are wide.
    uint32_t unit;          // One narrow unit (1/15 of the letter).
} split_t;

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
static volatile uint32_t g_edge_pos[EDGE_RING_SIZE];   // encoder_position().
static volatile bool     g_edge_black[EDGE_RING_SIZE];
static volatile uint32_t g_edge_head = 0u;
static uint32_t          g_edge_tail = 0u;

// Scan in progress. Widths are kept both in time and in distance; the
// decoder uses g_elements, which scan_finish() fills from the better one.
static uint32_t g_elements[MAX_ELEMENTS];
static uint32_t g_elem_usec[MAX_ELEMENTS];
static uint32_t g_elem_dist[MAX_ELEMENTS];
static uint32_t g_num_elements   = 0u;
static uint32_t g_widest_usec    = 0u;
static uint32_t g_widest_dist    = 0u;
static uint32_t g_last_edge_usec = 0u;
static uint32_t g_last_edge_pos  = 0u;
static uint32_t g_scan_start_pos = 0u;
static bool     gb_widths_dist   = false;   // g_elements is in distance.
static bool     gb_last_black    = false;
static bool     gb_capturing     = false;
static bool     gb_overflow      = false;
static bool     gb_enabled       = true;

// Analog reader state. Written by the sampling timer only.
static repeating_timer_t g_ao_timer;
static volatile int32_t  g_ao_level   = 0;      // Smoothed darkness.
static volatile bool     gb_ao_black  = false;  // Over a bar now.
static int32_t           g_ao_white   = 0;      // Paper white, tracked.
static int32_t           g_ao_extreme = 0;      // Last dip (on white) or
                                                // peak (on black).

static char              g_message[BARCODE_MESSAGE_MAX + 1u];
static barcode_command_t g_command = BARCODE_CMD_NONE;

// Why the last lenient line-up failed, for the log.
static char const *      gp_lenient_why = "";
static uint32_t          g_lenient_pos  = 0u;   // Scan position (1-based).


static void             edge_push(uint32_t edge_usec, bool b_black);
static bool             ao_sample_isr(repeating_timer_t * p_timer);
static int32_t          ao_darkness(void);
static void             edge_add(uint32_t edge_usec, uint32_t edge_pos,
                                 bool b_black);
static bool             scan_has_ended(uint32_t end_gap_usec);
static void             widths_select(void);
static barcode_result_t scan_finish(void);
static void             scan_dump(void);
static bool             scan_decode(bool b_reverse);
static bool             scan_decode_lenient(void);
static char             lenient_try(bool b_reverse, uint32_t lead);
static char             letter_decode(uint32_t first, bool b_reverse,
                                      split_t * p_split);
static bool             lenient_check(uint32_t j, uint32_t lead,
                                      bool b_reverse,
                                      split_t const * p_split);
static bool             lenient_sum_ok(uint32_t first_j, uint32_t last_j,
                                       uint32_t lead, bool b_reverse,
                                       split_t const * p_split);
static bool             expected_wide(uint32_t idx, uint16_t letter);
static char             char_decode(uint32_t first, bool b_reverse);
static uint32_t         element_get(uint32_t index, bool b_reverse);
static barcode_command_t command_match(char const * p_message);


/*!
 * @brief Set up the barcode sensor: the analog sampler, or the DOUT pin.
 *
 * @par
 * For DOUT, the edge interrupt itself is enabled by gpio_isr_init().
 */
void
barcode_init (void)
{
    gpio_init(PIN_BARCODE);
    gpio_set_dir(PIN_BARCODE, GPIO_IN);
    gpio_disable_pulls(PIN_BARCODE);
    g_message[0] = '\0';

    if (BARCODE_USE_ANALOG)
    {
        adc_init();
        adc_gpio_init(PIN_BARCODE_AO);
        adc_select_input(PIN_BARCODE_AO - ADC_FIRST_PIN);

        // Start from the current reading, so power-up is not an edge.
        g_ao_level   = ao_darkness();
        g_ao_extreme = g_ao_level;
        g_ao_white   = g_ao_level;

        // NOTE: A negative period keeps samples evenly spaced.
        if (add_repeating_timer_us(-((int64_t) BARCODE_AO_SAMPLE_USEC),
                                   ao_sample_isr, NULL, &g_ao_timer))
        {
            log_printf("Barcode: analog reader on GP%u, level %" PRId32
                       " now.", PIN_BARCODE_AO, g_ao_level);
        }
        else
        {
            log_printf("Barcode: FAILED to start the analog sampler.");
        }
    }
}   /* barcode_init() */


/*!
 * @brief Record one barcode edge. Called from the GPIO interrupt (DOUT
 *        reading only).
 *
 * @param[in] events The edge events (unused; the pin is read directly so a
 *                   glitch that rises and falls together is handled).
 */
void
barcode_edge_isr (uint32_t events)
{
    (void) events;  // The level is read from the pin instead.

    edge_push(time_us_32(), (BARCODE_BLACK_IS_HIGH == gpio_get(PIN_BARCODE)));
}   /* barcode_edge_isr() */


/*!
 * @brief Report the barcode sensor's smoothed darkness, for tuning.
 *
 * @return 0 (white) to 4095 (black) with the analog reader; with DOUT,
 *         0 or 4095.
 */
int32_t
barcode_level (void)
{
    int32_t level = g_ao_level;

    if (!BARCODE_USE_ANALOG)
    {
        level = (barcode_is_black() ? ADC_FULL_SCALE : 0);
    }

    return (level);
}   /* barcode_level() */


/*!
 * @brief Report how many bars and spaces the last scan had.
 *
 * @return The count; a valid Code 39 scan has 10 per character, less 1.
 */
uint32_t
barcode_scan_size (void)
{
    return (g_num_elements);
}   /* barcode_scan_size() */


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
            edge_add(g_edge_usec[idx], g_edge_pos[idx], g_edge_black[idx]);
        }
        g_edge_tail++;
    }

    end_gap = g_widest_usec * BARCODE_END_GAP_FACTOR;
    end_gap = ((end_gap > BARCODE_END_GAP_MIN_USEC) ?
               end_gap : BARCODE_END_GAP_MIN_USEC);

    if (gb_capturing && scan_has_ended(end_gap))
    {
        result = scan_finish();
    }
    else
    {
        // Nothing finished this tick.
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
    if (!b_enabled && gb_capturing)
    {
        // Say so: a scan cut off by STOP or a state change would otherwise
        // vanish without a trace.
        log_printf("Barcode: scan discarded (reading switched off after %"
                   PRIu32 " bars+spaces).", g_num_elements);
        widths_select();
        scan_dump();
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
 * @brief Forget the last decoded message (web CLEAR OUTPUT).
 *
 * @par
 * Only the shown message is cleared; a pending turn is kept.
 */
void
barcode_clear_message (void)
{
    g_message[0] = '\0';
    g_command    = BARCODE_CMD_NONE;
}   /* barcode_clear_message() */


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
    bool b_black = gb_ao_black;

    if (!BARCODE_USE_ANALOG)
    {
        b_black = (BARCODE_BLACK_IS_HIGH == gpio_get(PIN_BARCODE));
    }

    return (b_black);
}   /* barcode_is_black() */


/*!
 * @brief Put one edge into the ring for barcode_task(). Interrupt context.
 *
 * @par
 * The car's position (encoder_position()) is recorded with it, so bar
 * widths can be measured in distance as well as time.
 *
 * @param[in] edge_usec When the edge happened.
 * @param[in] b_black   True if the sensor sees black after the edge.
 */
static void
edge_push (uint32_t edge_usec, bool b_black)
{
    uint32_t head = g_edge_head;

    g_edge_usec[head & EDGE_RING_MASK]  = edge_usec;
    g_edge_pos[head & EDGE_RING_MASK]   = encoder_position();
    g_edge_black[head & EDGE_RING_MASK] = b_black;
    g_edge_head = head + 1u;
}   /* edge_push() */


/*!
 * @brief Sample the analog output and turn peaks and dips into edges.
 *        Timer interrupt, every BARCODE_AO_SAMPLE_USEC.
 *
 * @param[in] p_timer The timer (unused).
 *
 * @return True, to keep sampling.
 */
static bool
ao_sample_isr (repeating_timer_t * p_timer)
{
    int32_t level = g_ao_level;
    int32_t guard = 0;

    (void) p_timer;

    // Smoothing removes ADC noise without blurring a bar.
    level += (ao_darkness() - level) / AO_FILTER_DIV;

    // Nothing whiter than paper white plus the guard band is ever a bar.
    guard = g_ao_white + (BARCODE_AO_DELTA / AO_GUARD_DIV);

    if (!gb_ao_black)
    {
        // Follow the paper slowly, so one noisy sample cannot move it.
        g_ao_white += (level - g_ao_white) / AO_WHITE_DIV;

        if (level < g_ao_extreme)
        {
            g_ao_extreme = level;           // Whiter still: new dip.
        }
        else if ((level >= (g_ao_extreme + BARCODE_AO_DELTA)) &&
                 (level >= guard))
        {
            gb_ao_black  = true;            // Darkened enough: a bar.
            g_ao_extreme = level;
            edge_push(time_us_32(), true);
        }
        else
        {
            // Within DELTA of the dip: no change.
        }
    }
    else
    {
        if (level > g_ao_extreme)
        {
            g_ao_extreme = level;           // Darker still: new peak.
        }
        else if ((level <= (g_ao_extreme - BARCODE_AO_DELTA)) ||
                 (level < guard))
        {
            // Lightened enough, or back at paper white (so noise can never
            // leave the reader stuck on "black"): a space.
            gb_ao_black  = false;
            g_ao_extreme = level;
            edge_push(time_us_32(), false);
        }
        else
        {
            // Within DELTA of the peak: no change.
        }
    }

    g_ao_level = level;

    return (true);
}   /* ao_sample_isr() */


/*!
 * @brief Read the analog output as darkness: higher means blacker.
 *
 * @return 0 to 4095.
 */
static int32_t
ao_darkness (void)
{
    int32_t sum = 0;
    int32_t raw = 0;

    // Average several conversions (about 2 us each): the ADC noise is
    // about the size of a thin bar's dip, and averaging N reads cuts it
    // by roughly the square root of N.
    for (uint32_t read = 0u; read < BARCODE_AO_OVERSAMPLE; read++)
    {
        sum += (int32_t) adc_read();
    }
    raw = sum / (int32_t) BARCODE_AO_OVERSAMPLE;

    return (BARCODE_AO_BLACK_IS_HIGH ? raw : (ADC_FULL_SCALE - raw));
}   /* ao_darkness() */


/*!
 * @brief Add one edge to the current scan, or start a scan on a bar.
 *
 * @param[in] edge_usec When the edge happened.
 * @param[in] edge_pos  Where the car was then (encoder_position()).
 * @param[in] b_black   True if the sensor sees black after the edge.
 */
static void
edge_add (uint32_t edge_usec, uint32_t edge_pos, bool b_black)
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
            g_widest_dist    = 0u;
            g_last_edge_usec = edge_usec;
            g_last_edge_pos  = edge_pos;
            g_scan_start_pos = edge_pos;
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
            g_last_edge_usec -= g_elem_usec[g_num_elements];
            g_last_edge_pos  -= g_elem_dist[g_num_elements];
            gb_last_black     = b_black;
        }
    }
    else if ((0u == g_num_elements) &&
             ((edge_usec - g_last_edge_usec) >= BRANCH_MIN_USEC))
    {
        // The first mark just ended and was as long as a crossroad branch
        // (line_sensor.c has reported it): not a barcode. Drop it, so a
        // barcode soon after it is read on its own.
        gb_capturing = false;
    }
    else
    {
        uint32_t width = edge_usec - g_last_edge_usec;
        uint32_t dist  = edge_pos - g_last_edge_pos;

        if (g_num_elements < MAX_ELEMENTS)
        {
            g_elem_usec[g_num_elements] = width;
            g_elem_dist[g_num_elements] = dist;
            g_num_elements++;
        }
        else
        {
            gb_overflow = true;
        }

        g_widest_usec    = ((width > g_widest_usec) ? width
                                                    : g_widest_usec);
        g_widest_dist    = ((dist > g_widest_dist) ? dist : g_widest_dist);
        g_last_edge_usec = edge_usec;
        g_last_edge_pos  = edge_pos;
        gb_last_black    = b_black;
    }
}   /* edge_add() */


/*!
 * @brief Decide whether the scan in progress has ended.
 *
 * @par
 * With the encoders turning (BARCODE_USE_ENCODERS), the quiet gap after
 * the last bar is measured in distance, so a pause over white does not
 * end a scan; otherwise in time. On black it never ends sooner than
 * BARCODE_BLACK_MAX_USEC (a wide or first bar can be long), and then only
 * once the car has moved, so a pause on a bar does not end it either.
 * Nothing at all for BARCODE_STALL_USEC ends it regardless.
 *
 * @param[in] end_gap_usec The quiet time that ends a scan by time.
 *
 * @return True if the scan has ended.
 */
static bool
scan_has_ended (uint32_t end_gap_usec)
{
    uint32_t now_pos    = encoder_position();
    uint32_t quiet_usec = time_us_32() - g_last_edge_usec;
    uint32_t moved      = now_pos - g_last_edge_pos;
    bool     b_enc      = (BARCODE_USE_ENCODERS &&
                           ((now_pos - g_scan_start_pos) >=
                            ENCODER_POS_SCALE));
    bool     b_end      = false;

    if (quiet_usec > BARCODE_STALL_USEC)
    {
        b_end = true;           // Nothing for a long time: stop waiting.
    }
    else if (gb_last_black)
    {
        b_end = ((quiet_usec > BARCODE_BLACK_MAX_USEC) &&
                 ((!b_enc) || (moved >= ENCODER_POS_SCALE)));
    }
    else if (b_enc && (g_widest_dist > 0u))
    {
        b_end = ((quiet_usec > BARCODE_END_GAP_MIN_USEC) &&
                 (moved > (g_widest_dist * BARCODE_END_GAP_FACTOR)));
    }
    else
    {
        b_end = (quiet_usec > end_gap_usec);
    }

    return (b_end);
}   /* scan_has_ended() */


/*!
 * @brief Choose the widths to decode: distance if the encoders turned
 *        enough during the scan, otherwise time.
 */
static void
widths_select (void)
{
    uint32_t ticks = (g_last_edge_pos - g_scan_start_pos) /
                     ENCODER_POS_SCALE;

    gb_widths_dist = (BARCODE_USE_ENCODERS &&
                      (ticks >= BARCODE_MIN_SCAN_TICKS));

    for (uint32_t el = 0u; el < g_num_elements; el++)
    {
        g_elements[el] = (gb_widths_dist ? g_elem_dist[el] : g_elem_usec[el]);
    }

    if (BARCODE_USE_ENCODERS && !gb_widths_dist)
    {
        log_printf("Barcode: encoders moved only %" PRIu32 " ticks over the "
                   "scan -> widths measured in time.", ticks);
    }
}   /* widths_select() */


/*!
 * @brief Decode the finished scan, forwards and then backwards; if that
 *        fails, try the lenient one-letter decode.
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
    else if (g_num_elements >= REPORT_MIN_ELEMENTS)
    {
        widths_select();    // Distance or time, before decoding.
    }
    else
    {
        // Too short to decode; reported below.
    }

    if (gb_overflow)
    {
        // Already invalid.
    }
    else if (g_num_elements < REPORT_MIN_ELEMENTS)
    {
        // One or two marks (a stray line or smudge): not a barcode. Not a
        // result either, but logged so a missed barcode can be diagnosed.
        log_printf("Barcode: only %" PRIu32 " bars+spaces seen, ignored.",
                   g_num_elements);
    }
    else if ((g_num_elements >= MIN_ELEMENTS) &&
             (scan_decode(false) || scan_decode(true)))
    {
        result = BARCODE_DECODED;   // Every element read cleanly.
    }
    else if (scan_decode_lenient())
    {
        result = BARCODE_DECODED;   // One letter, ends partly missed.
    }
    else
    {
        result = BARCODE_INVALID;
    }

    if (BARCODE_INVALID == result)
    {
        scan_dump();
    }

    g_widest_usec = 0u;

    return (result);
}   /* scan_finish() */


/*!
 * @brief Log every element width of the current scan, to diagnose it.
 *
 * @par
 * Widths are in the units decoded: tenths of an encoder tick (both wheels
 * added) or ms. The first element is always a bar, so odd numbers (#1,
 * #3, ...) are bars and even numbers are spaces.
 */
static void
scan_dump (void)
{
    char   text[DUMP_LINE_BYTES] = { 0 };
    size_t length = 0u;

    log_printf("Barcode widths in %s (#1 is a bar, then space, bar, ...):",
               (gb_widths_dist ? "0.1 encoder ticks" : "ms"));

    // NOTE: A line holds a 6-character prefix plus DUMP_PER_LINE widths of
    // at most 8 characters each (86 bytes), so snprintf never truncates.
    for (uint32_t el = 0u; el < g_num_elements; el++)
    {
        if (0u == (el % DUMP_PER_LINE))
        {
            length = (size_t) snprintf(text, sizeof(text), " #%" PRIu32 ":",
                                       el + 1u);
        }

        length += (size_t) snprintf(text + length, sizeof(text) - length,
                                    " %" PRIu32,
                                    (gb_widths_dist ?
                                     ((g_elements[el] * DUMP_TENTHS) /
                                      ENCODER_POS_SCALE) :
                                     (g_elements[el] / USEC_PER_MSEC)));

        if (((DUMP_PER_LINE - 1u) == (el % DUMP_PER_LINE)) ||
            ((el + 1u) == g_num_elements))
        {
            log_printf("%s", text);
        }
    }
}   /* scan_dump() */


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
 * @brief Lenient decode of a one-letter barcode "*X*" (X = A to Z) whose
 *        outer ends were partly missed.
 *
 * @par
 * The side sensor can slip past the tips of the outermost bars, so a scan
 * may lose elements at its ends (and only half-see the next one): up to
 * LENIENT_MAX_MISSING_ALL in all, or up to LENIENT_MAX_ONE_SIDED at one
 * end when the '*' at the other end is complete. Every way the scan could
 * line up with "*X*" is tried, in both directions; a line-up is accepted
 * only if:
 * - the letter is complete and decodes cleanly to A-Z, with bars and
 *   spaces judged separately (2 wide bars, 1 wide space) and every wide
 *   at least LENIENT_RATIO_X10 / 10 times every narrow of its kind;
 * - every other visible element is clearly narrow or wide (not within
 *   LENIENT_BAND_PERCENT of the split) and matches '*' or the narrow gap,
 *   except that a cut-off outermost element may read shorter;
 * - each end's elements add up to 70-140% of their expected width, so no
 *   element is missing or extra inside an end.
 * If two line-ups give different letters, nothing is accepted. If none
 * fits, the reason each line-up failed is logged.
 *
 * @return True if a letter was decoded and stored as the message.
 */
static bool
scan_decode_lenient (void)
{
    char     found     = '\0';
    bool     b_clash   = false;
    bool     b_tried   = false;
    uint32_t found_lead = 0u;
    uint32_t n         = g_num_elements;

    // A scan starts and ends on a bar, so it loses elements in pairs.
    for (uint32_t dir = 0u; dir < 2u; dir++)
    {
        for (uint32_t lead = 0u; lead <= LENIENT_MAX_ONE_SIDED; lead += 2u)
        {
            char     symbol = '\0';
            uint32_t trail  = LENIENT_ELEMENTS - lead - n;
            bool     b_both = ((lead <= LENIENT_MAX_MISSING) &&
                               (trail <= LENIENT_MAX_MISSING) &&
                               ((lead + trail) <= LENIENT_MAX_MISSING_ALL));
            bool     b_one  = (((0u == lead) || (0u == trail)) &&
                               ((lead + trail) <= LENIENT_MAX_ONE_SIDED));

            // Only whole bar+space pairs can be missing at the other end
            // too, within the limits above.
            if (((lead + n) <= LENIENT_ELEMENTS) && (0u == (trail % 2u)) &&
                (b_both || b_one))
            {
                b_tried = true;
                symbol  = lenient_try((1u == dir), lead);

                if ('\0' == symbol)
                {
                    log_printf("Barcode lenient %s, %" PRIu32 "+%" PRIu32
                               " missing: %s (#%" PRIu32 ").",
                               ((1u == dir) ? "reversed" : "forward"), lead,
                               trail, gp_lenient_why, g_lenient_pos);
                }
            }

            if ('\0' == symbol)
            {
                // This line-up does not fit.
            }
            else if (('\0' != found) && (symbol != found))
            {
                b_clash = true;
            }
            else
            {
                found      = symbol;
                found_lead = lead;
            }
        }
    }

    if (('\0' != found) && !b_clash)
    {
        g_message[0] = found;
        g_message[1] = '\0';
        g_command    = command_match(g_message);
        log_printf("Barcode: read leniently (%" PRIu32 " of %u bars+spaces, "
                   "%" PRIu32 " missing at one end, %" PRIu32 " at the "
                   "other).", n, LENIENT_ELEMENTS, found_lead,
                   (LENIENT_ELEMENTS - found_lead - n));
    }
    else if (b_clash)
    {
        log_printf("Barcode: lenient read ambiguous (two letters fit), "
                   "rejected.");
    }
    else if (!b_tried)
    {
        log_printf("Barcode lenient: %" PRIu32 " bars+spaces cannot be "
                   "\"*X*\" with at most %u missing.", n,
                   LENIENT_MAX_ONE_SIDED);
    }
    else
    {
        // Nothing fits; each line-up's reason is logged above.
    }

    return (('\0' != found) && !b_clash);
}   /* scan_decode_lenient() */


/*!
 * @brief Try one line-up for the lenient decode.
 *
 * @param[in] b_reverse True to read the elements last-to-first.
 * @param[in] lead      Elements of "*X*" missing before the scan's first.
 *
 * @return The letter (A to Z), or '\0' if this line-up does not fit.
 */
static char
lenient_try (bool b_reverse, uint32_t lead)
{
    split_t  split;
    uint32_t n      = g_num_elements;
    uint32_t last_j = n - 1u;
    uint32_t first  = LENIENT_LETTER_FIRST - lead;
    char     symbol = letter_decode(first, b_reverse, &split);
    bool     b_ok   = (('A' <= symbol) && (symbol <= 'Z'));

    // Scan positions in the log count from 1, in scan order (as the
    // width dump does).
    gp_lenient_why = "letter not a clean A-Z";
    g_lenient_pos  = (b_reverse ? (n - first) : (first + 1u));

    // Every element outside the letter must match '*' or a narrow gap.
    for (uint32_t j = 0u; (j < n) && b_ok; j++)
    {
        uint32_t idx = j + lead;

        if ((idx < LENIENT_LETTER_FIRST) || (idx > LENIENT_GAP_B - 1u))
        {
            b_ok = lenient_check(j, lead, b_reverse, &split);

            if (!b_ok)
            {
                gp_lenient_why = (expected_wide(idx, split.pattern) ?
                                  "should be wide" : "should be narrow");
                g_lenient_pos  = (b_reverse ? (n - j) : (j + 1u));
            }
        }
    }

    // Each end's complete elements (not the cut-off outermost one) must
    // add up to about the right width: nothing missing or extra inside.
    if (b_ok)
    {
        b_ok = lenient_sum_ok(1u, (LENIENT_GAP_A - lead), lead, b_reverse,
                              &split);
        gp_lenient_why = "first end's width off";
        g_lenient_pos  = (b_reverse ? n : 1u);
    }
    if (b_ok)
    {
        b_ok = lenient_sum_ok((LENIENT_GAP_B - lead), (last_j - 1u), lead,
                              b_reverse, &split);
        gp_lenient_why = "last end's width off";
        g_lenient_pos  = (b_reverse ? 1u : n);
    }

    return (b_ok ? symbol : '\0');
}   /* lenient_try() */


/*!
 * @brief Decode one complete character, judging bars and spaces apart:
 *        its 2 widest bars and its widest space are wide.
 *
 * @param[in]  first     Index of the character's first element.
 * @param[in]  b_reverse True to read the elements last-to-first.
 * @param[out] p_split   The pattern and narrow/wide splits it shows.
 *
 * @return The character, or '\0' if the wide and narrow elements are not
 *         clearly apart (LENIENT_RATIO_X10) or the pattern is unknown.
 */
static char
letter_decode (uint32_t first, bool b_reverse, split_t * p_split)
{
    uint32_t bar_wide_min    = UINT32_MAX;
    uint32_t bar_narrow_max  = 0u;
    uint32_t bar_narrow_sum  = 0u;
    uint32_t bar_wide_sum    = 0u;
    uint32_t space_wide      = 0u;
    uint32_t space_narrow_max = 0u;
    uint32_t space_narrow_sum = 0u;
    uint32_t total           = 0u;
    uint16_t pattern         = 0u;
    char     symbol          = '\0';

    // Mark the 2 widest bars (even elements) wide.
    for (uint32_t pick = 0u; pick < WIDE_BARS_PER_CHAR; pick++)
    {
        uint32_t best_el    = 0u;
        uint32_t best_width = 0u;

        for (uint32_t el = 0u; el < ELEMENTS_PER_CHAR; el += 2u)
        {
            uint16_t bit   = (uint16_t) (1u << (ELEMENTS_PER_CHAR - 1u - el));
            uint32_t width = element_get(first + el, b_reverse);

            if ((0u == (pattern & bit)) && (width > best_width))
            {
                best_width = width;
                best_el    = el;
            }
        }
        pattern |= (uint16_t) (1u << (ELEMENTS_PER_CHAR - 1u - best_el));
    }

    // Mark the widest space (odd elements) wide.
    {
        uint32_t best_el    = 1u;
        uint32_t best_width = 0u;

        for (uint32_t el = 1u; el < ELEMENTS_PER_CHAR; el += 2u)
        {
            uint32_t width = element_get(first + el, b_reverse);

            if (width > best_width)
            {
                best_width = width;
                best_el    = el;
            }
        }
        pattern |= (uint16_t) (1u << (ELEMENTS_PER_CHAR - 1u - best_el));
    }

    // Measure each group, to check they are clearly apart.
    for (uint32_t el = 0u; el < ELEMENTS_PER_CHAR; el++)
    {
        uint16_t bit    = (uint16_t) (1u << (ELEMENTS_PER_CHAR - 1u - el));
        uint32_t width  = element_get(first + el, b_reverse);
        bool     b_wide = (0u != (pattern & bit));

        total += width;

        if (0u == (el % 2u))
        {
            if (b_wide)
            {
                bar_wide_sum += width;
                bar_wide_min  = ((width < bar_wide_min) ? width
                                                        : bar_wide_min);
            }
            else
            {
                bar_narrow_sum += width;
                bar_narrow_max  = ((width > bar_narrow_max) ? width
                                                            : bar_narrow_max);
            }
        }
        else if (b_wide)
        {
            space_wide = width;
        }
        else
        {
            space_narrow_sum += width;
            space_narrow_max  = ((width > space_narrow_max) ? width
                                                            : space_narrow_max);
        }
    }

    if (((bar_wide_min * 10u) >= (bar_narrow_max * LENIENT_RATIO_X10)) &&
        ((space_wide * 10u) >= (space_narrow_max * LENIENT_RATIO_X10)))
    {
        for (uint32_t idx = 0u; idx < NUM_CODE39_SYMBOLS; idx++)
        {
            if (g_code39[idx].pattern == pattern)
            {
                symbol = g_code39[idx].symbol;
            }
        }
    }

    // Splits halfway between the average narrow and wide of each kind.
    p_split->pattern     = pattern;
    p_split->bar_split   = ((bar_narrow_sum / (BARS_PER_CHAR -
                                               WIDE_BARS_PER_CHAR)) +
                            (bar_wide_sum / WIDE_BARS_PER_CHAR)) / 2u;
    p_split->space_split = ((space_narrow_sum /
                             (ELEMENTS_PER_CHAR - BARS_PER_CHAR - 1u)) +
                            space_wide) / 2u;
    p_split->unit        = total / UNITS_PER_CHAR;

    return (symbol);
}   /* letter_decode() */


/*!
 * @brief Check one element outside the letter against "*X*".
 *
 * @param[in] j         The element's position in the scan.
 * @param[in] lead      Elements missing before the scan's first.
 * @param[in] b_reverse True to read the elements last-to-first.
 * @param[in] p_split   The splits learned from the letter.
 *
 * @return True if it is clearly the width "*X*" needs there. The scan's
 *         first and last elements may have been cut off by the sensor, so
 *         there a wide element may also read shorter (but not absurdly
 *         long).
 */
static bool
lenient_check (uint32_t j, uint32_t lead, bool b_reverse,
               split_t const * p_split)
{
    uint32_t idx      = j + lead;
    bool     b_bar    = (0u == (idx % 2u));
    bool     b_wide   = expected_wide(idx, p_split->pattern);
    uint32_t split    = (b_bar ? p_split->bar_split : p_split->space_split);
    uint32_t width    = element_get(j, b_reverse);
    bool     b_outer  = ((0u == j) || ((g_num_elements - 1u) == j));
    bool     b_narrow_clear = ((width * PERCENT_100) <=
                               (split * (PERCENT_100 - LENIENT_BAND_PERCENT)));
    bool     b_wide_clear   = ((width * PERCENT_100) >=
                               (split * (PERCENT_100 + LENIENT_BAND_PERCENT)));
    bool     b_ok     = false;

    if (b_outer && b_wide)
    {
        b_ok = (width <= (split * LENIENT_LONG_X));  // May be cut short.
    }
    else if (b_wide)
    {
        b_ok = b_wide_clear;
    }
    else
    {
        b_ok = b_narrow_clear;
    }

    return (b_ok);
}   /* lenient_check() */


/*!
 * @brief Check that a run of elements adds up to about its expected
 *        width, so none is missing or extra in it.
 *
 * @param[in] first_j   First scan position of the run.
 * @param[in] last_j    Last scan position of the run (inclusive).
 * @param[in] lead      Elements missing before the scan's first.
 * @param[in] b_reverse True to read the elements last-to-first.
 * @param[in] p_split   The splits learned from the letter.
 *
 * @return True if the run is within 70-140% of its expected width (or is
 *         empty).
 */
static bool
lenient_sum_ok (uint32_t first_j, uint32_t last_j, uint32_t lead,
                bool b_reverse, split_t const * p_split)
{
    uint32_t width = 0u;
    uint32_t units = 0u;
    bool     b_ok  = true;

    for (uint32_t j = first_j; (j <= last_j) && (j < g_num_elements); j++)
    {
        width += element_get(j, b_reverse);
        units += (expected_wide(j + lead, p_split->pattern) ? WIDE_UNITS
                                                            : 1u);
    }

    if (units > 0u)
    {
        uint32_t expected = units * p_split->unit;

        b_ok = (((width * PERCENT_100) >=
                 (expected * LENIENT_SUM_MIN_PERCENT)) &&
                ((width * PERCENT_100) <=
                 (expected * LENIENT_SUM_MAX_PERCENT)));
    }

    return (b_ok);
}   /* lenient_sum_ok() */


/*!
 * @brief Report whether element idx of "*X*" is wide.
 *
 * @param[in] idx    Element 0 to 28 of "*X*".
 * @param[in] letter The letter's pattern.
 *
 * @return True if that element is wide.
 */
static bool
expected_wide (uint32_t idx, uint16_t letter)
{
    uint16_t pattern = STAR_PATTERN;
    uint32_t el      = idx;
    bool     b_wide  = false;

    if ((LENIENT_GAP_A == idx) || (LENIENT_GAP_B == idx))
    {
        b_wide = false;                     // Gaps are narrow.
    }
    else
    {
        if (idx >= LENIENT_STOP_FIRST)
        {
            el = idx - LENIENT_STOP_FIRST;
        }
        else if (idx >= LENIENT_LETTER_FIRST)
        {
            el      = idx - LENIENT_LETTER_FIRST;
            pattern = letter;
        }
        else
        {
            // The first '*'.
        }

        b_wide = (0u != (pattern & (1u << (ELEMENTS_PER_CHAR - 1u - el))));
    }

    return (b_wide);
}   /* expected_wide() */


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
