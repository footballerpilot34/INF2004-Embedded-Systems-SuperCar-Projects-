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
    WEB_CMD_DIAG,           // Toggle IR diagnostic mode (motors off).
    WEB_CMD_LEARN_BLACK,    // Both line sensors are over black now.
    WEB_CMD_LEARN_WHITE,    // Both line sensors are over white now.
    WEB_CMD_TURN_LEFT,      // Test: act as if a barcode said LEFT.
    WEB_CMD_TURN_RIGHT,     // Test: act as if a barcode said RIGHT.
    WEB_CMD_UTURN,          // Test: act as if a barcode said UTURN.
    WEB_CMD_BARCODE_TEST,   // Drive straight, ignoring the line sensors.
    WEB_CMD_BARCODE_STATIC, // Stand still; barcodes are slid past by hand.
    WEB_CMD_STATIONARY_SCAN, // Motors off; test the existing obstacle scan.
    WEB_CMD_ENCODER_MARK,   // Encoder 300 mm roll: start, then finish.
    WEB_CMD_FEATURE_TEST    // Run with chosen features (web_features()).
} web_command_t;

bool          web_init(void);
void          web_task(void);
web_command_t web_command_take(void);
void          web_features(bool * p_line, bool * p_avoid,
                           bool * p_barcode);
char const *  web_wifi_text(void);
void          web_set_restart_reason(char const * p_reason);
// Pass a string literal; feedback remains visible until the next test event.
void          web_set_scan_test_status(char const * p_status);
void          web_set_scan_test_busy(bool busy);
void          web_barcode_record(char const * p_message,
                                 barcode_command_t command);
bool          web_servo_take(int32_t * p_degrees);
bool          web_is_online(void);
char const *  web_restart_reason(void);
void          web_command_submit(char const * p_name);

#endif /* WEB_H */

/*** end of file ***/
