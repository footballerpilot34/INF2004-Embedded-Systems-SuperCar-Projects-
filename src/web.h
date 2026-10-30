/** @file web.h
 *
 * @brief Wireless UI with vehicle controls, live readings and IMU graphs.
 */

#ifndef WEB_H
#define WEB_H

#include <stdbool.h>

typedef enum
{
    WEB_CMD_NONE,
    WEB_CMD_START,
    WEB_CMD_STOP,
    WEB_CMD_IMU_CHECK,
    WEB_CMD_WHEEL_TEST,
    WEB_CMD_DIAG,           // Toggle IR diagnostic mode (motors off).
    WEB_CMD_LEARN_BLACK,    // Both line sensors are over black now.
    WEB_CMD_LEARN_WHITE     // Both line sensors are over white now.
} web_command_t;

bool          web_init(void);
void          web_task(void);
web_command_t web_command_take(void);

#endif /* WEB_H */

/*** end of file ***/
