/** @file web.h
 *
 * @brief Wireless UI: Wi-Fi hotspot plus a small web page with controls,
 *        live readings, the barcode message and the message log.
 */

#ifndef WEB_H
#define WEB_H

#include <stdbool.h>

typedef enum
{
    WEB_CMD_NONE,
    WEB_CMD_START,
    WEB_CMD_STOP,
    WEB_CMD_CALIBRATE,
    WEB_CMD_WHEEL_TEST,
    WEB_CMD_DIAG,           // Toggle IR diagnostic mode (motors off).
    WEB_CMD_LEARN_BLACK,    // Both line sensors are over black now.
    WEB_CMD_LEARN_WHITE,    // Both line sensors are over white now.
    WEB_CMD_TURN_LEFT,      // Test: act as if a barcode said LEFT.
    WEB_CMD_TURN_RIGHT,     // Test: act as if a barcode said RIGHT.
    WEB_CMD_UTURN           // Test: act as if a barcode said UTURN.
} web_command_t;

bool          web_init(void);
void          web_task(void);
web_command_t web_command_take(void);

#endif /* WEB_H */

/*** end of file ***/
