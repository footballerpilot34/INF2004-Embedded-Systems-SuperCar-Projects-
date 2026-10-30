/** @file imu.c
 *
 * @brief Raw LSM303DLHC accelerometer and magnetometer readings.
 *
 * The LSM303DLHC contains an accelerometer and a magnetometer, but no
 * gyroscope. This module reports only the six physical axis values; it
 * does not estimate orientation or motion events.
 */

#include <stdbool.h>
#include <limits.h>
#include <math.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

#include "config.h"
#include "imu.h"
#include "log.h"


#define IMU_I2C                         i2c1
#define IMU_I2C_HZ                      100000u
#define I2C_TIMEOUT_USEC                5000u
#define SAMPLE_PERIOD_USEC              200000u

#define ACCEL_ADDRESS                   0x19u
#define MAG_ADDRESS                     0x1Eu

#define ACCEL_WHO_AM_I                  0x0Fu
#define ACCEL_WHO_AM_I_VALUE            0x33u
#define ACCEL_CTRL_REG1                 0x20u
#define ACCEL_CTRL_REG4                 0x23u
#define ACCEL_OUT_X_L                   0x28u
#define ACCEL_AUTO_INCREMENT            0x80u

#define MAG_CRA_REG                     0x00u
#define MAG_CRB_REG                     0x01u
#define MAG_MODE_REG                    0x02u
#define MAG_OUT_X_H                     0x03u
#define MAG_ID_REG                      0x0Au
#define MAG_ID_H                        0x48u
#define MAG_ID_4                        0x34u
#define MAG_ID_3                        0x33u

#define MAG_XY_SENSITIVITY_LSB_G        1100
#define MAG_Z_SENSITIVITY_LSB_G         980
#define MILLI_PER_UNIT                  1000
#define RAW_AXIS_BYTES                  6u
#define CALIBRATION_SAMPLES              25u
#define ACCEL_MIN_MG                     800
#define ACCEL_MAX_MG                     1200
#define PITCH_FILTER_DIVISOR             3
#define RAD_TO_TENTH_DEG                 572.958f


static bool         gb_sensor_ok       = false;
static bool         gb_reading_valid   = false;
static uint64_t     g_last_sample_usec = 0u;
static imu_axes_t   g_axes             = { 0, 0, 0, 0, 0, 0 };
static bool         gb_calibrated       = false;
static int32_t      g_pitch_zero_x10    = 0;
static int32_t      g_roll_zero_x10     = 0;
static int32_t      g_pitch_x10         = 0;
static int32_t      g_roll_x10          = 0;


static bool    reg_write(uint8_t address, uint8_t reg, uint8_t value);
static bool    reg_read(uint8_t address, uint8_t reg, uint8_t * p_data,
                        size_t length);
static bool    sample_read(imu_axes_t * p_axes);
static int32_t signed_be(uint8_t msb, uint8_t lsb);
static int32_t accel_mg(uint8_t msb, uint8_t lsb);
static int32_t raw_pitch_x10(imu_axes_t const * p_axes);
static int32_t raw_roll_x10(imu_axes_t const * p_axes);


/*!
 * @brief Initialize I2C and both LSM303DLHC sensor cores.
 *
 * The bus is I2C1 on the project's existing GP2 SDA / GP3 SCL pins.
 *
 * @return True when both core identities and configuration writes succeed.
 */
bool
imu_init (void)
{
    uint8_t accel_id = 0u;
    uint8_t mag_id[3] = { 0u, 0u, 0u };
    bool    accel_ok = false;
    bool    mag_ok   = false;

    i2c_init(IMU_I2C, IMU_I2C_HZ);
    gpio_set_function(PIN_IMU_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_IMU_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_IMU_SDA);
    gpio_pull_up(PIN_IMU_SCL);

    accel_ok = reg_read(ACCEL_ADDRESS, ACCEL_WHO_AM_I, &accel_id, 1u) &&
                (ACCEL_WHO_AM_I_VALUE == accel_id);
    mag_ok   = reg_read(MAG_ADDRESS, MAG_ID_REG, mag_id, sizeof(mag_id)) &&
                (MAG_ID_H == mag_id[0]) && (MAG_ID_4 == mag_id[1]) &&
                (MAG_ID_3 == mag_id[2]);

    if (!accel_ok)
    {
        log_printf("LSM303: accelerometer not detected at 0x%02X "
                   "(WHO_AM_I 0x%02X, expected 0x%02X).",
                   ACCEL_ADDRESS, accel_id, ACCEL_WHO_AM_I_VALUE);
    }
    if (!mag_ok)
    {
        log_printf("LSM303: magnetometer not detected at 0x%02X "
                   "(ID %02X %02X %02X, expected 48 34 33).",
                   MAG_ADDRESS, mag_id[0], mag_id[1], mag_id[2]);
    }

    gb_sensor_ok = false;
    gb_reading_valid = false;

    if (accel_ok && mag_ok)
    {
        accel_ok = reg_write(ACCEL_ADDRESS, ACCEL_CTRL_REG1, 0x27u) &&
                   reg_write(ACCEL_ADDRESS, ACCEL_CTRL_REG4, 0x88u);
        mag_ok   = reg_write(MAG_ADDRESS, MAG_CRA_REG, 0x10u) &&
                   reg_write(MAG_ADDRESS, MAG_CRB_REG, 0x20u) &&
                   reg_write(MAG_ADDRESS, MAG_MODE_REG, 0x00u);
        gb_sensor_ok = (accel_ok && mag_ok);

        if (gb_sensor_ok)
        {
            log_printf("LSM303DLHC ready on I2C1 GP2/GP3: accel 0x19, "
                       "mag 0x1E. Live axes are shown in the UI.");
            sleep_ms(120u);
            gb_reading_valid = sample_read(&g_axes);
            g_last_sample_usec = time_us_64();
        }
        else
        {
            log_printf("LSM303: sensor configuration failed.");
        }
    }

    return (gb_sensor_ok);
}   /* imu_init() */


/*!
 * @brief Refresh all six physical axis values every 200 ms.
 *
 * Call from the main control loop. The filtered pitch is the only derived
 * value; the other five axes remain direct physical-unit readings.
 */
void
imu_task (void)
{
    if (gb_sensor_ok)
    {
        uint64_t now_usec = time_us_64();

        if ((now_usec - g_last_sample_usec) >= SAMPLE_PERIOD_USEC)
        {
            g_last_sample_usec = now_usec;
            gb_reading_valid = sample_read(&g_axes);
            if (gb_reading_valid && gb_calibrated)
            {
                int32_t pitch = raw_pitch_x10(&g_axes) - g_pitch_zero_x10;
                int32_t roll = raw_roll_x10(&g_axes) - g_roll_zero_x10;
                g_pitch_x10 += (pitch - g_pitch_x10) /
                               PITCH_FILTER_DIVISOR;
                g_roll_x10 += (roll - g_roll_x10) / PITCH_FILTER_DIVISOR;
            }
        }
    }
}   /* imu_task() */


/*!
 * @brief Calibrate the level pitch reference while the stopped car is still.
 *
 * Takes a five-second sample window and rejects implausible gravity magnitude
 * or excessive tilt variation. The vehicle must be stationary during this.
 */
bool
imu_calibrate (void)
{
    int64_t pitch_sum = 0;
    int64_t roll_sum = 0;
    int32_t pitch_min = INT32_MAX;
    int32_t pitch_max = INT32_MIN;
    int32_t roll_min = INT32_MAX;
    int32_t roll_max = INT32_MIN;
    bool    b_ok      = gb_sensor_ok;

    gb_calibrated = false;
    for (uint32_t idx = 0u; (idx < CALIBRATION_SAMPLES) && b_ok; idx++)
    {
        int32_t pitch = 0;
        int32_t roll = 0;
        float   magnitude = 0.0f;

        sleep_ms(200u);
        b_ok = sample_read(&g_axes);
        if (b_ok)
        {
            magnitude = sqrtf((float) (g_axes.ax_mg * g_axes.ax_mg) +
                              (float) (g_axes.ay_mg * g_axes.ay_mg) +
                              (float) (g_axes.az_mg * g_axes.az_mg));
            pitch = raw_pitch_x10(&g_axes);
            roll = raw_roll_x10(&g_axes);
            b_ok = (magnitude >= ACCEL_MIN_MG) &&
                   (magnitude <= ACCEL_MAX_MG);
            if (pitch < pitch_min) pitch_min = pitch;
            if (pitch > pitch_max) pitch_max = pitch;
            if (roll < roll_min) roll_min = roll;
            if (roll > roll_max) roll_max = roll;
            pitch_sum += pitch;
            roll_sum += roll;
        }
    }

    if (b_ok)
    {
        b_ok = ((pitch_max - pitch_min) <= 30) &&
               ((roll_max - roll_min) <= 30);
    }
    if (b_ok)
    {
        g_pitch_zero_x10 = (int32_t) (pitch_sum / CALIBRATION_SAMPLES);
        g_roll_zero_x10 = (int32_t) (roll_sum / CALIBRATION_SAMPLES);
        g_pitch_x10 = 0;
        g_roll_x10 = 0;
        gb_calibrated = true;
    }
    gb_reading_valid = b_ok;
    g_last_sample_usec = time_us_64();
    return (b_ok);
}   /* imu_calibrate() */


/*!
 * @brief Report whether both LSM303DLHC sensor cores initialized.
 */
bool
imu_is_ok (void)
{
    return (gb_sensor_ok);
}   /* imu_is_ok() */


/*!
 * @brief Report whether the latest six-axis read succeeded.
 */
bool
imu_reading_is_valid (void)
{
    return (gb_reading_valid);
}   /* imu_reading_is_valid() */


/*!
 * @brief Report whether a still, level pitch reference has been calibrated.
 */
bool
imu_is_calibrated (void)
{
    return (gb_calibrated);
}   /* imu_is_calibrated() */


/*!
 * @brief Copy the most recent axis values in thousandths of their units.
 *
 * Acceleration values are mg; magnetic values are mGauss.
 */
void
imu_get_axes (imu_axes_t * p_axes)
{
    *p_axes = g_axes;
}   /* imu_get_axes() */


/*!
 * @brief Return filtered pitch relative to the calibrated level reference.
 */
int32_t
imu_pitch_x10 (void)
{
    return (gb_calibrated && gb_reading_valid ? g_pitch_x10 : 0);
}   /* imu_pitch_x10() */


/*!
 * @brief Return filtered roll relative to the calibrated level reference.
 */
int32_t
imu_roll_x10 (void)
{
    return (gb_calibrated && gb_reading_valid ? g_roll_x10 : 0);
}   /* imu_roll_x10() */


/*!
 * @brief Return the signed pitch or roll with the greater tilt magnitude.
 */
int32_t
imu_max_tilt_x10 (void)
{
    int32_t pitch = imu_pitch_x10();
    int32_t roll = imu_roll_x10();

    return (((pitch < 0 ? -pitch : pitch) >=
             (roll < 0 ? -roll : roll)) ? pitch : roll);
}   /* imu_max_tilt_x10() */


/*!
 * @brief Read acceleration and magnetic output registers and convert units.
 *
 * @return True if both I2C transfers completed.
 */
static bool
sample_read (imu_axes_t * p_axes)
{
    uint8_t accel_data[RAW_AXIS_BYTES] = { 0u };
    uint8_t mag_data[RAW_AXIS_BYTES]   = { 0u };
    bool    accel_ok = reg_read(ACCEL_ADDRESS,
                                (uint8_t) (ACCEL_OUT_X_L |
                                           ACCEL_AUTO_INCREMENT),
                                accel_data, sizeof(accel_data));
    bool    mag_ok   = reg_read(MAG_ADDRESS, MAG_OUT_X_H,
                                mag_data, sizeof(mag_data));

    if (accel_ok && mag_ok)
    {
        p_axes->ax_mg = accel_mg(accel_data[1], accel_data[0]);
        p_axes->ay_mg = accel_mg(accel_data[3], accel_data[2]);
        p_axes->az_mg = accel_mg(accel_data[5], accel_data[4]);

        p_axes->mx_mgauss = (signed_be(mag_data[0], mag_data[1]) *
                             MILLI_PER_UNIT) / MAG_XY_SENSITIVITY_LSB_G;
        p_axes->mz_mgauss = (signed_be(mag_data[2], mag_data[3]) *
                             MILLI_PER_UNIT) / MAG_Z_SENSITIVITY_LSB_G;
        p_axes->my_mgauss = (signed_be(mag_data[4], mag_data[5]) *
                             MILLI_PER_UNIT) / MAG_XY_SENSITIVITY_LSB_G;
    }

    return (accel_ok && mag_ok);
}   /* sample_read() */


/*!
 * @brief Convert little-endian, left-aligned high-resolution acceleration.
 */
static int32_t
accel_mg (uint8_t msb, uint8_t lsb)
{
    int32_t raw = (((int32_t) msb << 8) | lsb);

    if (raw >= 32768)
    {
        raw -= 65536;
    }
    if (raw < 0)
    {
        raw = -(((-raw) + 15) / 16);
    }
    else
    {
        raw /= 16;
    }

    return (raw);       // +/-2 g, 1 mg per count.
}   /* accel_mg() */


/*!
 * @brief Calculate raw pitch for Y-forward / Z-up sensor mounting.
 */
static int32_t
raw_pitch_x10 (imu_axes_t const * p_axes)
{
    float angle = atan2f((float) p_axes->ay_mg, (float) p_axes->az_mg);

    return ((int32_t) (angle * RAD_TO_TENTH_DEG));
}   /* raw_pitch_x10() */


/*!
 * @brief Calculate raw roll for X-right / Z-up sensor mounting.
 */
static int32_t
raw_roll_x10 (imu_axes_t const * p_axes)
{
    float angle = atan2f((float) -p_axes->ax_mg, (float) p_axes->az_mg);

    return ((int32_t) (angle * RAD_TO_TENTH_DEG));
}   /* raw_roll_x10() */


/*!
 * @brief Convert one big-endian two's-complement word to signed counts.
 */
static int32_t
signed_be (uint8_t msb, uint8_t lsb)
{
    int32_t value = (((int32_t) msb << 8) | lsb);

    if (value >= 32768)
    {
        value -= 65536;
    }

    return (value);
}   /* signed_be() */


/*!
 * @brief Write a single register on one sensor core.
 */
static bool
reg_write (uint8_t address, uint8_t reg, uint8_t value)
{
    uint8_t frame[2] = { reg, value };
    int     written  = i2c_write_timeout_us(IMU_I2C, address, frame,
                                            sizeof(frame), false,
                                            I2C_TIMEOUT_USEC);

    return ((int) sizeof(frame) == written);
}   /* reg_write() */


/*!
 * @brief Read consecutive registers using a repeated-start I2C transfer.
 */
static bool
reg_read (uint8_t address, uint8_t reg, uint8_t * p_data, size_t length)
{
    int result = i2c_write_timeout_us(IMU_I2C, address, &reg, 1u, true,
                                      I2C_TIMEOUT_USEC);

    if (1 == result)
    {
        result = i2c_read_timeout_us(IMU_I2C, address, p_data, length, false,
                                     I2C_TIMEOUT_USEC);
    }

    return ((int) length == result);
}   /* reg_read() */

/*** end of file ***/
