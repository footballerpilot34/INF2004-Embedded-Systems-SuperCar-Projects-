/** @file imu.h
 *
 * @brief LSM303DLHC raw accelerometer and magnetometer axis readings.
 */

#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    int32_t ax_mg;
    int32_t ay_mg;
    int32_t az_mg;
    int32_t mx_mgauss;
    int32_t my_mgauss;
    int32_t mz_mgauss;
} imu_axes_t;

bool    imu_init(void);
void    imu_task(void);
bool    imu_is_ok(void);
bool    imu_reading_is_valid(void);
void    imu_get_axes(imu_axes_t * p_axes);
bool    imu_has_gyroscope(void);
int32_t imu_pitch_x10(void);
int32_t imu_pitch_rate_x10(void);

#endif /* IMU_H */

/*** end of file ***/
