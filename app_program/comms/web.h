/** @file web.h
 *
 * @brief Wireless UI with vehicle controls, live readings and IMU graphs.
 */

#ifndef WEB_H
#define WEB_H

#include <stdbool.h>
#include <stdint.h>

#include "barcode.h"

typedef enum
{
    WEB_CMD_NONE,
    WEB_CMD_START,
    WEB_CMD_STOP,
    WEB_CMD_IMU_CALIBRATE,
    WEB_CMD_WHEEL_TEST,
    WEB_CMD_DIAG,            // Toggle IR diagnostic mode (motors off).
    WEB_CMD_LEARN_BLACK,     // Both line sensors are over black now.
    WEB_CMD_LEARN_WHITE,     // Both line sensors are over white now.
    WEB_CMD_TURN_LEFT,       // Test: act as if a barcode said LEFT.
    WEB_CMD_TURN_RIGHT,      // Test: act as if a barcode said RIGHT.
    WEB_CMD_UTURN,           // Test: act as if a barcode said UTURN.
    WEB_CMD_BARCODE_TEST,    // Drive straight, ignoring the line sensors.
    WEB_CMD_BARCODE_STATIC,  // Stand still; barcodes are slid past by hand.
    WEB_CMD_STATIONARY_SCAN, // Motors off; test the existing obstacle scan.
    WEB_CMD_ENCODER_MARK,    // Encoder 300 mm roll: start, then finish.
    WEB_CMD_BYPASS_TEST, // Serial/MQTT v: empty-floor rehearsal; ends stopped.
    WEB_CMD_FEATURE_TEST // Run with chosen features (web_features()).
} web_command_t;

// The Wi-Fi link as numbers (web_link_get()), for the binary heartbeat.
typedef struct
{
    uint8_t  mode;       // 0 off, 1 joining, 2 joined, 3 rejoining, 4 own
                         // network: 3 bits.
    uint8_t  hotspot;    // Index in HOTSPOT_LIST (while joined).
    uint32_t ip;         // IPv4 as lwIP holds it (first octet in the low
                         // byte); 0 when not joined.
    int16_t  rssi_dbm;   // Signal, dBm (0 = unknown).
    bool     b_ping_ok;  // The hotspot answers pings.
    uint32_t ping_msec;  // Last round trip.
    uint32_t ping_age_s; // Since the last reply.
    uint32_t drops;      // Hotspot links lost.
} web_link_t;

bool          web_init(void);
void          web_task(void);
web_command_t web_command_take(void);
void          web_features(bool * p_line, bool * p_avoid, bool * p_barcode,
                           bool * p_hump);
char const *  web_wifi_text(void);
// Why the Pico last started, as one byte: bits 0-2 the kind, bits 3-7
// the loop stage (stage_t) of a watchdog or hard-fault restart.
#define WEB_RESTART_UNKNOWN 0u
#define WEB_RESTART_POWER_ON 1u  // Power-on or brown-out.
#define WEB_RESTART_RUN_PIN 2u   // Reset button.
#define WEB_RESTART_DEBUGGER 3u  // Debugger restart.
#define WEB_RESTART_SOFTWARE 4u  // Software reset (after flashing).
#define WEB_RESTART_WATCHDOG 5u  // The loop froze.
#define WEB_RESTART_HARDFAULT 6u // CPU hard fault.
#define WEB_RESTART_NET_CHECK 7u // lwIP failed check.
#define WEB_RESTART_STAGE_SHIFT 3u

void web_set_restart_reason(char const * p_reason, uint32_t code);
// Pass a string literal; feedback remains visible until the next test event.
void web_set_scan_test_status(char const * p_status);
void web_set_scan_test_busy(bool busy);
void web_barcode_record(char const * p_message, barcode_command_t command);
bool web_servo_take(int32_t * p_degrees);
bool web_is_online(void);
char const * web_restart_reason(void);
uint32_t     web_restart_code(void);
void         web_link_get(web_link_t * p_link);
void         web_command_submit(char const * p_name);

#endif /* WEB_H */

/*** end of file ***/
