/** @file imu.h
 *
 * @brief MPU-6050 pitch estimate for hump detection.
 */

#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>

bool    imu_init(void);
bool    imu_calibrate(void);
void    imu_task(void);
bool    imu_is_ok(void);
int32_t imu_pitch_x10(void);
int32_t imu_pitch_rate_x10(void);

#endif /* IMU_H */

/*** end of file ***/
