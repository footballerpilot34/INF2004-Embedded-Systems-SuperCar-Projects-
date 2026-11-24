/** @file mqtt_frame.h
 *
 * @brief Compact binary MQTT payloads (change requirement 1): every value
 *        is packed into only the bits its range needs, instead of JSON
 *        text.
 *
 * @par Frame
 * Byte 0 is the header: the frame type (MQTT_FRAME_...) in bits 0-3 and
 * MQTT_FRAME_VERSION in bits 4-7 (0xA1 to 0xA8, never a printable
 * character, so an old JSON or text payload is never mistaken for a
 * frame). The fields follow, packed LSB first with no padding (bitpack.h),
 * in the order and widths listed in testing/min_bits/DATA_TABLE.md. The
 * last byte is padded with zero bits.
 *
 * @par Decoders
 * tools/mqtt_frame.py (Python) and tools/mqtt_car.ps1 (the laptop's MQTT
 * windows) decode the same layouts; change all three together.
 *
 * @par
 * Plain C with no Pico SDK calls, so the host tests can compile it.
 */

#ifndef MQTT_FRAME_H
#define MQTT_FRAME_H

#include <stdbool.h>
#include <stdint.h>

#define MQTT_FRAME_VERSION 0xAu
#define MQTT_FRAME_TELEMETRY 1u
#define MQTT_FRAME_HEARTBEAT 2u
#define MQTT_FRAME_BARCODE 3u
#define MQTT_FRAME_OBSTACLE 4u
#define MQTT_FRAME_HUMP 5u
#define MQTT_FRAME_LOG 6u
#define MQTT_FRAME_COMMAND 7u
#define MQTT_FRAME_ACK 8u

#define MQTT_FRAME_MAX_BYTES 32u  // Largest fixed frame (heartbeat: 28).
#define MQTT_FRAME_WORD_BYTES 10u // "calibrate" + terminator.

// Distances that may be missing.
#define MQTT_FRAME_NO_RANGE 0u   // Unsigned mm fields: no reading.
#define MQTT_FRAME_NO_MM (-4096) // Signed 13-bit mm fields: no reading.

// Command opcodes (5 bits). The car turns each back into the word the
// web page sends, so MQTT and the page still share one command path.
#define MQTT_CMD_STOP 0u
#define MQTT_CMD_START 1u
#define MQTT_CMD_CALIBRATE 2u
#define MQTT_CMD_WHEEL 3u
#define MQTT_CMD_DIAG 4u
#define MQTT_CMD_BLACK 5u
#define MQTT_CMD_WHITE 6u
#define MQTT_CMD_BARCODE_TEST 7u
#define MQTT_CMD_BARCODE_STATIC 8u
#define MQTT_CMD_SCAN 9u
#define MQTT_CMD_BYPASS_TEST 10u
#define MQTT_CMD_MARK 11u
#define MQTT_CMD_CLEAR 12u
#define MQTT_CMD_LETTERS 13u
#define MQTT_CMD_LETTERS_RESET 14u
#define MQTT_CMD_SET_LETTER 15u // + letter (5 bits, A = 0) + action (3).
#define MQTT_CMD_FEATURES 16u   // + line, avoid, barcode, hump (1 bit each).
#define MQTT_CMD_COUNT 17u

// Live readings (every MQTT_TELEMETRY_MSEC). Field widths in comments.
typedef struct
{
    uint32_t t_msec;       // 32: ms since boot.
    uint8_t  state;        // 4: vehicle_state_t (12 states).
    uint8_t  bypass_phase; // 4: bypass_phase_id() in BYPASS, else 0.
    int8_t   motor_left;   // 8 signed: -100..100 %.
    int8_t   motor_right;  // 8 signed.
    uint16_t enc_left;     // 16: edge count mod 65536 (receiver unwraps).
    uint16_t enc_right;    // 16.
    bool     b_line_left;  // 1.
    bool     b_line_right; // 1.
    bool     b_barcode;    // 1: barcode sensor on black.
    uint16_t range_mm;     // 12: 1..4095 mm, 0 = no echo.
    int16_t  pitch_x10;    // 12 signed: -180.0..180.0 deg.
    int16_t  roll_x10;     // 12 signed.
    uint8_t  servo_deg;    // 8: 0..180 deg.
    int16_t  hump_x10;     // 11 signed: -102.4..102.3 mm.
    uint16_t accel_mg;     // 13: 0..8191 mg.
    int16_t  lift_mm_s;    // 12 signed: -2048..2047 mm/s.
    uint8_t  pending_turn; // 3: barcode_command_t.
    uint8_t  faults;       // 8: FAULT_... bits (fault.h).
} mqtt_tele_t;

// Heartbeat (every MQTT_HEARTBEAT_MSEC).
typedef struct
{
    uint16_t seq;         // 16: wraps.
    uint32_t up_s;        // 24: seconds since boot (194 days).
    uint16_t vsys_mv;     // 13: 0..8191 mV.
    uint16_t sent;        // 16: messages queued, mod 65536.
    uint16_t dropped;     // 16: messages dropped, mod 65536.
    uint8_t  connects;    // 8: broker connections (held at 255).
    uint8_t  wifi_mode;   // 3: web_link_t mode.
    uint8_t  hotspot;     // 3: HOTSPOT_LIST index.
    uint32_t ip;          // 32: IPv4, first octet in the low byte.
    int8_t   rssi_dbm;    // 8 signed: dBm.
    bool     b_ping_ok;   // 1.
    uint16_t ping_msec;   // 10: 0..1023 ms.
    uint8_t  ping_age_s;  // 8: 0..255 s.
    uint8_t  link_drops;  // 8: hotspot links lost (held at 255).
    uint8_t  restart;     // 8: web_restart_code().
    uint8_t  faults;      // 8: FAULT_... bits.
    uint16_t loop_max_us; // 16: longest loop pass since the last beat.
    uint16_t overruns;    // 16: passes over TICK_USEC since boot.
} mqtt_heartbeat_t;

// A finished obstacle scan.
typedef struct
{
    uint32_t t_msec;       // 32.
    uint16_t front_mm;     // 12: 0 = none.
    int16_t  left_mm;      // 13 signed: MQTT_FRAME_NO_MM = none.
    int16_t  right_mm;     // 13 signed.
    int16_t  mid_mm;       // 13 signed.
    uint16_t width_mm;     // 12: 0 = none.
    bool     b_geometric;  // 1: width from confirmed edges.
    uint8_t  left_status;  // 3: obstacle_edge_status_t.
    uint8_t  right_status; // 3.
} mqtt_obstacle_t;

// A cleared hump.
typedef struct
{
    uint32_t t_msec;         // 32.
    int16_t  height_x10;     // 11 signed: mm x 10.
    int16_t  tilt_x10;       // 12 signed: deg x 10.
    int16_t  run_height_x10; // 11 signed.
    int16_t  run_tilt_x10;   // 12 signed.
    uint8_t  count;          // 8: humps this run (held at 255).
} mqtt_hump_t;

uint32_t mqtt_frame_telemetry(mqtt_tele_t const * p_tele, uint8_t * p_out,
                              uint32_t capacity);
uint32_t mqtt_frame_heartbeat(mqtt_heartbeat_t const * p_beat, uint8_t * p_out,
                              uint32_t capacity);
uint32_t mqtt_frame_barcode(uint32_t t_msec, uint32_t command,
                            char const * p_message, uint8_t * p_out,
                            uint32_t capacity);
uint32_t mqtt_frame_obstacle(mqtt_obstacle_t const * p_scan, uint8_t * p_out,
                             uint32_t capacity);
uint32_t mqtt_frame_hump(mqtt_hump_t const * p_hump, uint8_t * p_out,
                         uint32_t capacity);
uint32_t mqtt_frame_log(uint32_t category, char const * p_text, uint8_t * p_out,
                        uint32_t capacity);
bool mqtt_frame_command(uint8_t const * p_in, uint32_t length, char * p_word,
                        uint32_t word_capacity);
uint32_t mqtt_frame_ack(uint8_t const * p_command, uint32_t length,
                        uint8_t * p_out, uint32_t capacity);
uint32_t mqtt_frame_code39_index(char symbol);

#endif /* MQTT_FRAME_H */

/*** end of file ***/
