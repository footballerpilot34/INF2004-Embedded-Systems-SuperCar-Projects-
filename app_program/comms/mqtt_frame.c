/** @file mqtt_frame.c
 *
 * @brief Compact binary MQTT payloads: pack the car's readings and events
 *        into frames, and unpack command frames (see mqtt_frame.h).
 *
 * @par
 * Fields are packed with explicit masks and shifts (bitpack.c), never by
 * sending a C struct, so struct padding, bit-field order and byte order
 * cannot change what goes on the network (BARR-C 5.3.a, 5.3.b).
 *
 * @par
 * Called from the main loop only, on buffers the caller owns.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "bitpack.h"
#include "mqtt_frame.h"

#define HEADER_TYPE_BITS 4u
#define HEADER_VERSION_SHIFT 4u
#define TEXT_LEN_BITS 5u // Barcode message: up to 31 characters.
#define TEXT_LEN_MAX 31u
#define CODE39_BITS 6u // 44 Code 39 symbols (and "unknown").
#define CODE39_UNKNOWN 63u
#define LOG_CAT_BITS 3u
#define LOG_PAD_BITS 5u // The category byte's spare bits.
#define OPCODE_BITS 5u
#define LETTER_BITS 5u
#define ACTION_BITS 3u
#define LETTERS 26u

// Code 39 symbols, in the order their 6-bit codes are sent.
static char const g_code39[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%*";

// SET_LETTER actions, by their 3-bit code: none, left, right, straight,
// U-turn (the letters web.c's "bc" command takes).
static char const g_actions[] = "nlrsu";

// Command words, by opcode (the same words the web page sends), each in
// MQTT_FRAME_WORD_BYTES with its terminator. NOTE: Keep that size at the
// longest word ("calibrate") + 1: C accepts a word that fills the row
// exactly and drops its terminator. test_mqtt_frame_host.py decodes every
// word, the longest included.
static char const g_words[MQTT_CMD_COUNT][MQTT_FRAME_WORD_BYTES] = {
    "stop",  "start",  "calibrate", "wheel", "diag", "black",
    "white", "bt",     "bs",        "scan",  "v",    "mark",
    "clear", "bclist", "bcreset",   "bc",    "ft"};

static void header_put(bitpack_t * p_pack, uint32_t type);

/*!
 * @brief Pack the live readings.
 *
 * @param[in]  p_tele   The readings.
 * @param[out] p_out    The frame.
 * @param[in]  capacity Its room (MQTT_FRAME_MAX_BYTES is enough).
 *
 * @return The frame's length in bytes (24), or 0 if it did not fit.
 */
uint32_t
mqtt_frame_telemetry(mqtt_tele_t const * p_tele, uint8_t * p_out,
                     uint32_t capacity)
{
    bitpack_t pack;

    bitpack_init(&pack, p_out, capacity);
    header_put(&pack, MQTT_FRAME_TELEMETRY);
    bitpack_put(&pack, p_tele->t_msec, 32u);
    bitpack_put_sat(&pack, p_tele->state, 4u);
    bitpack_put_sat(&pack, p_tele->bypass_phase, 4u);
    bitpack_put_signed(&pack, p_tele->motor_left, 8u);
    bitpack_put_signed(&pack, p_tele->motor_right, 8u);
    bitpack_put(&pack, p_tele->enc_left, 16u);
    bitpack_put(&pack, p_tele->enc_right, 16u);
    bitpack_put(&pack, (p_tele->b_line_left ? 1u : 0u), 1u);
    bitpack_put(&pack, (p_tele->b_line_right ? 1u : 0u), 1u);
    bitpack_put(&pack, (p_tele->b_barcode ? 1u : 0u), 1u);
    bitpack_put_sat(&pack, p_tele->range_mm, 12u);
    bitpack_put_signed(&pack, p_tele->pitch_x10, 12u);
    bitpack_put_signed(&pack, p_tele->roll_x10, 12u);
    bitpack_put_sat(&pack, p_tele->servo_deg, 8u);
    bitpack_put_signed(&pack, p_tele->hump_x10, 11u);
    bitpack_put_sat(&pack, p_tele->accel_mg, 13u);
    bitpack_put_signed(&pack, p_tele->lift_mm_s, 12u);
    bitpack_put_sat(&pack, p_tele->pending_turn, 3u);
    bitpack_put(&pack, p_tele->faults, 8u);

    return (pack.b_overrun ? 0u : bitpack_bytes(&pack));
} /* mqtt_frame_telemetry() */

/*!
 * @brief Pack the heartbeat.
 *
 * @param[in]  p_beat   The heartbeat.
 * @param[out] p_out    The frame.
 * @param[in]  capacity Its room.
 *
 * @return The frame's length in bytes (28), or 0 if it did not fit.
 */
uint32_t
mqtt_frame_heartbeat(mqtt_heartbeat_t const * p_beat, uint8_t * p_out,
                     uint32_t capacity)
{
    bitpack_t pack;

    bitpack_init(&pack, p_out, capacity);
    header_put(&pack, MQTT_FRAME_HEARTBEAT);
    bitpack_put(&pack, p_beat->seq, 16u);
    bitpack_put_sat(&pack, p_beat->up_s, 24u);
    bitpack_put_sat(&pack, p_beat->vsys_mv, 13u);
    bitpack_put(&pack, p_beat->sent, 16u);
    bitpack_put(&pack, p_beat->dropped, 16u);
    bitpack_put(&pack, p_beat->connects, 8u);
    bitpack_put_sat(&pack, p_beat->wifi_mode, 3u);
    bitpack_put_sat(&pack, p_beat->hotspot, 3u);
    bitpack_put(&pack, p_beat->ip, 32u);
    bitpack_put_signed(&pack, p_beat->rssi_dbm, 8u);
    bitpack_put(&pack, (p_beat->b_ping_ok ? 1u : 0u), 1u);
    bitpack_put_sat(&pack, p_beat->ping_msec, 10u);
    bitpack_put(&pack, p_beat->ping_age_s, 8u);
    bitpack_put(&pack, p_beat->link_drops, 8u);
    bitpack_put(&pack, p_beat->restart, 8u);
    bitpack_put(&pack, p_beat->faults, 8u);
    bitpack_put(&pack, p_beat->loop_max_us, 16u);
    bitpack_put(&pack, p_beat->overruns, 16u);

    return (pack.b_overrun ? 0u : bitpack_bytes(&pack));
} /* mqtt_frame_heartbeat() */

/*!
 * @brief Pack a decoded barcode: its command and its text, 6 bits per
 *        Code 39 character instead of 8.
 *
 * @param[in]  t_msec    When it was read.
 * @param[in]  command   barcode_command_t (3 bits).
 * @param[in]  p_message The text (up to 31 characters are sent).
 * @param[out] p_out     The frame.
 * @param[in]  capacity  Its room.
 *
 * @return The frame's length in bytes, or 0 if it did not fit.
 */
uint32_t
mqtt_frame_barcode(uint32_t t_msec, uint32_t command, char const * p_message,
                   uint8_t * p_out, uint32_t capacity)
{
    bitpack_t pack;
    uint32_t  length = (uint32_t)strlen(p_message);

    if (length > TEXT_LEN_MAX)
    {
        length = TEXT_LEN_MAX;
    }

    bitpack_init(&pack, p_out, capacity);
    header_put(&pack, MQTT_FRAME_BARCODE);
    bitpack_put(&pack, t_msec, 32u);
    bitpack_put_sat(&pack, command, 3u);
    bitpack_put(&pack, length, TEXT_LEN_BITS);
    for (uint32_t idx = 0u; idx < length; idx++)
    {
        bitpack_put(&pack, mqtt_frame_code39_index(p_message[idx]),
                    CODE39_BITS);
    }

    return (pack.b_overrun ? 0u : bitpack_bytes(&pack));
} /* mqtt_frame_barcode() */

/*!
 * @brief Pack a finished obstacle scan.
 *
 * @param[in]  p_scan   The scan's result.
 * @param[out] p_out    The frame.
 * @param[in]  capacity Its room.
 *
 * @return The frame's length in bytes (14), or 0 if it did not fit.
 */
uint32_t
mqtt_frame_obstacle(mqtt_obstacle_t const * p_scan, uint8_t * p_out,
                    uint32_t capacity)
{
    bitpack_t pack;

    bitpack_init(&pack, p_out, capacity);
    header_put(&pack, MQTT_FRAME_OBSTACLE);
    bitpack_put(&pack, p_scan->t_msec, 32u);
    bitpack_put_sat(&pack, p_scan->front_mm, 12u);
    bitpack_put_signed(&pack, p_scan->left_mm, 13u);
    bitpack_put_signed(&pack, p_scan->right_mm, 13u);
    bitpack_put_signed(&pack, p_scan->mid_mm, 13u);
    bitpack_put_sat(&pack, p_scan->width_mm, 12u);
    bitpack_put(&pack, (p_scan->b_geometric ? 1u : 0u), 1u);
    bitpack_put_sat(&pack, p_scan->left_status, 3u);
    bitpack_put_sat(&pack, p_scan->right_status, 3u);

    return (pack.b_overrun ? 0u : bitpack_bytes(&pack));
} /* mqtt_frame_obstacle() */

/*!
 * @brief Pack a cleared hump.
 *
 * @param[in]  p_hump   The hump.
 * @param[out] p_out    The frame.
 * @param[in]  capacity Its room.
 *
 * @return The frame's length in bytes (12), or 0 if it did not fit.
 */
uint32_t
mqtt_frame_hump(mqtt_hump_t const * p_hump, uint8_t * p_out, uint32_t capacity)
{
    bitpack_t pack;

    bitpack_init(&pack, p_out, capacity);
    header_put(&pack, MQTT_FRAME_HUMP);
    bitpack_put(&pack, p_hump->t_msec, 32u);
    bitpack_put_signed(&pack, p_hump->height_x10, 11u);
    bitpack_put_signed(&pack, p_hump->tilt_x10, 12u);
    bitpack_put_signed(&pack, p_hump->run_height_x10, 11u);
    bitpack_put_signed(&pack, p_hump->run_tilt_x10, 12u);
    bitpack_put_sat(&pack, p_hump->count, 8u);

    return (pack.b_overrun ? 0u : bitpack_bytes(&pack));
} /* mqtt_frame_hump() */

/*!
 * @brief Pack a log line: the header, one byte holding the category
 *        (3 bits), then the text itself (UTF-8, no terminator). A log
 *        line is text for people to read, so the text is kept as it is.
 *
 * @param[in]  category log_cat_t (0..4).
 * @param[in]  p_text   The line.
 * @param[out] p_out    The frame.
 * @param[in]  capacity Its room; a longer line is cut short.
 *
 * @return The frame's length in bytes, or 0 if not even the header fit.
 */
uint32_t
mqtt_frame_log(uint32_t category, char const * p_text, uint8_t * p_out,
               uint32_t capacity)
{
    bitpack_t pack;
    uint32_t  length = (uint32_t)strlen(p_text);
    uint32_t  used   = 0u;

    bitpack_init(&pack, p_out, capacity);
    header_put(&pack, MQTT_FRAME_LOG);
    bitpack_put_sat(&pack, category, LOG_CAT_BITS);
    bitpack_put(&pack, 0u, LOG_PAD_BITS);
    if (pack.b_overrun)
    {
        return (0u);
    }

    used = bitpack_bytes(&pack);
    if (length > (capacity - used))
    {
        length = capacity - used;
    }
    (void)memcpy(p_out + used, p_text, length);

    return (used + length);
} /* mqtt_frame_log() */

/*!
 * @brief Unpack a command frame into the word the web page would send
 *        (for web_command_submit()).
 *
 * @param[in]  p_in          The payload.
 * @param[in]  length        Its length in bytes.
 * @param[out] p_word        The word, e.g. "start", "bcal", "ft1101".
 * @param[in]  word_capacity Its room (MQTT_FRAME_WORD_BYTES is enough).
 *
 * @return True if it was a valid command frame.
 */
bool
mqtt_frame_command(uint8_t const * p_in, uint32_t length, char * p_word,
                   uint32_t word_capacity)
{
    bitpack_t pack;
    uint32_t  version = 0u;
    uint32_t  type    = 0u;
    uint32_t  opcode  = 0u;
    bool      b_ok    = false;

    if ((length < 2u) || (word_capacity < MQTT_FRAME_WORD_BYTES))
    {
        return (false);
    }

    // The unpacker only reads, so casting away const is safe.
    bitpack_init(&pack, (uint8_t *)p_in, length);
    type    = bitpack_get(&pack, HEADER_TYPE_BITS);
    version = bitpack_get(&pack, HEADER_TYPE_BITS);
    opcode  = bitpack_get(&pack, OPCODE_BITS);

    if (((MQTT_FRAME_VERSION != version) || (MQTT_FRAME_COMMAND != type)) ||
        (opcode >= MQTT_CMD_COUNT))
    {
        return (false);
    }

    (void)memcpy(p_word, g_words[opcode], MQTT_FRAME_WORD_BYTES);
    if (MQTT_CMD_SET_LETTER == opcode)
    {
        uint32_t letter = bitpack_get(&pack, LETTER_BITS);
        uint32_t action = bitpack_get(&pack, ACTION_BITS);

        if ((letter < LETTERS) && (action < ((sizeof(g_actions)) - 1u)))
        {
            p_word[2] = (char)('a' + (char)letter);
            p_word[3] = g_actions[action];
            p_word[4] = '\0';
            b_ok      = true;
        }
    }
    else if (MQTT_CMD_FEATURES == opcode)
    {
        for (uint32_t idx = 0u; idx < 4u; idx++)
        {
            p_word[2u + idx] = ((0u != bitpack_get(&pack, 1u)) ? '1' : '0');
        }
        p_word[6] = '\0';
        b_ok      = true;
    }
    else
    {
        b_ok = true;
    }

    return (b_ok && (!pack.b_overrun));
} /* mqtt_frame_command() */

/*!
 * @brief Make the acknowledgement of an accepted command: the same frame
 *        with the ACK type in its header.
 *
 * @param[in]  p_command The command frame.
 * @param[in]  length    Its length.
 * @param[out] p_out     The acknowledgement.
 * @param[in]  capacity  Its room.
 *
 * @return The acknowledgement's length, or 0 if it did not fit.
 */
uint32_t
mqtt_frame_ack(uint8_t const * p_command, uint32_t length, uint8_t * p_out,
               uint32_t capacity)
{
    if ((0u == length) || (length > capacity))
    {
        return (0u);
    }

    (void)memcpy(p_out, p_command, length);
    p_out[0] = (uint8_t)((MQTT_FRAME_VERSION << HEADER_VERSION_SHIFT) |
                         MQTT_FRAME_ACK);

    return (length);
} /* mqtt_frame_ack() */

/*!
 * @brief Give a Code 39 character's 6-bit code.
 *
 * @param[in] symbol The character (lower case is sent as upper case).
 *
 * @return 0..43, or 63 for a character Code 39 does not have.
 */
uint32_t
mqtt_frame_code39_index(char symbol)
{
    char upper = symbol;

    if ((upper >= 'a') && (upper <= 'z'))
    {
        upper = (char)(upper - ('a' - 'A'));
    }
    for (uint32_t idx = 0u; '\0' != g_code39[idx]; idx++)
    {
        if (g_code39[idx] == upper)
        {
            return (idx);
        }
    }

    return (CODE39_UNKNOWN);
} /* mqtt_frame_code39_index() */

/*!
 * @brief Write the header byte: type in bits 0-3, version in bits 4-7.
 *
 * @param[in,out] p_pack The packer.
 * @param[in]     type   MQTT_FRAME_...
 */
static void
header_put(bitpack_t * p_pack, uint32_t type)
{
    bitpack_put(p_pack, type, HEADER_TYPE_BITS);
    bitpack_put(p_pack, MQTT_FRAME_VERSION, HEADER_TYPE_BITS);
} /* header_put() */

/*** end of file ***/
