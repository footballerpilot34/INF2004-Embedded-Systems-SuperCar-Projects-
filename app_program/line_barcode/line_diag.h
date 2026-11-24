/** @file line_diag.h
 *
 * @brief Line IR sensor diagnostic mode (motors off).
 */

#ifndef LINE_DIAG_H
#define LINE_DIAG_H

#include <stdbool.h>

void line_diag_enter(void);
void line_diag_exit(void);
bool line_diag_is_active(void);
void line_diag_task(void);
void line_diag_learn(bool b_black);

#endif /* LINE_DIAG_H */

/*** end of file ***/
