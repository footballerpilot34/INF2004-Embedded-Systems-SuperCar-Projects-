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
    int16_t ax_mg;
    int16_t ay_mg;
    int16_t az_mg;
    int16_t mx_mgauss;
    int16_t my_mgauss;
    int16_t mz_mgauss;
} imu_axes_t;

bool    imu_init(void);
void    imu_task(void);
bool    imu_calibrate(void);
bool    imu_is_ok(void);
bool    imu_reading_is_valid(void);
bool    imu_is_calibrated(void);
void    imu_get_axes(imu_axes_t * p_axes);
int32_t imu_pitch_x10(void);
int32_t imu_roll_x10(void);
int32_t imu_max_tilt_x10(void);
int32_t imu_total_tilt_x10(void);
int32_t imu_accel_mg(void);
int32_t imu_lift_speed_mm_s(void);

#endif /* IMU_H */

/*** end of file ***/
