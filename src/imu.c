/** @file imu.c
 *
 * @brief MPU-6050 pitch estimate for hump detection.
 *
 * @par How it works
 * Pitch (nose up = positive) is a complementary filter:
 * - the gyro rate is integrated every tick, which reacts fast but drifts;
 * - each tick the estimate is pulled 1/50 of the way towards the pitch
 *   from the accelerometer, which is noisy but does not drift.
 * The accelerometer pitch uses the small-angle approximation
 * (angle = forward g, in radians), which is within 2% up to 20 degrees,
 * so no floating point or trigonometry is needed.
 *
 * @par
 * All angles are kept in millidegrees; the public functions report tenths
 * of a degree. Calibration (car still) zeroes the gyro bias and the
 * mounting tilt, so pitch reads 0 on flat ground.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

#include "config.h"
#include "imu.h"
#include "log.h"


#define IMU_I2C                 i2c1
#define IMU_I2C_HZ              100000u // Safe for jumper wires.
#define I2C_TIMEOUT_USEC        2000u

#define MPU_ADDRESS_LOW         0x68u
#define MPU_ADDRESS_HIGH        0x69u
#define REG_SMPLRT_DIV          0x19u
#define REG_CONFIG              0x1Au
#define REG_GYRO_CONFIG         0x1Bu
#define REG_ACCEL_CONFIG        0x1Cu
#define REG_ACCEL_XOUT_H        0x3Bu
#define REG_PWR_MGMT_1          0x6Bu
#define REG_WHO_AM_I            0x75u

#define PWR_WAKE_PLL_X          0x01u   // Leave sleep; clock from gyro X.
#define DLPF_44HZ               0x03u
#define SAMPLE_DIV_100HZ        9u      // 1 kHz / (1 + 9) = 100 Hz.
#define GYRO_RANGE_250DPS       0x00u
#define ACCEL_RANGE_2G          0x00u
#define POWER_UP_MSEC           100u

#define NUM_AXES                3u
#define RAW_FRAME_BYTES         14u
#define GYRO_FRAME_OFFSET       8u
#define BYTES_PER_VALUE         2u

#define GYRO_LSB_PER_DPS        131
#define ACCEL_LSB_PER_G         16384
#define MDEG_PER_RAD            57296   // 180000 / pi.
#define MDEG_PER_DEG            1000
#define MDEG_PER_TENTH          100
#define USEC_PER_SECOND         1000000
#define ACCEL_WEIGHT_DIV        50      // Pull 2% towards accel per tick.

#define CAL_SAMPLES             100     // 1 s at 10 ms.
#define CAL_SAMPLE_USEC         10000u


static uint8_t  g_address        = 0u;
static bool     gb_ok            = false;
static int32_t  g_gyro_bias      = 0;   // Pitch-axis gyro bias, counts.
static int32_t  g_accel_ref_mdeg = 0;   // Mounting tilt when level.
static int32_t  g_pitch_mdeg     = 0;
static int32_t  g_rate_mdps      = 0;
static uint64_t g_last_usec      = 0u;


static bool    reg_write(uint8_t reg, uint8_t value);
static bool    reg_read(uint8_t reg, uint8_t * p_data, size_t length);
static bool    sample_read(int32_t * p_accel_mdeg, int32_t * p_gyro_counts);


/*!
 * @brief Find, configure and calibrate the MPU-6050.
 *
 * @par
 * The car must be still and level while this runs (about 1 s).
 *
 * @return True if the IMU is ready.
 */
bool
imu_init (void)
{
    static uint8_t const addresses[] = { MPU_ADDRESS_LOW, MPU_ADDRESS_HIGH };
    uint8_t              identity    = 0u;

    i2c_init(IMU_I2C, IMU_I2C_HZ);
    gpio_set_function(PIN_IMU_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_IMU_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_IMU_SDA);
    gpio_pull_up(PIN_IMU_SCL);

    gb_ok = false;

    for (uint32_t idx = 0u; (idx < sizeof(addresses)) && (!gb_ok); idx++)
    {
        g_address = addresses[idx];
        gb_ok     = reg_read(REG_WHO_AM_I, &identity, 1u);
    }

    if (gb_ok)
    {
        gb_ok = reg_write(REG_PWR_MGMT_1, PWR_WAKE_PLL_X);
        sleep_ms(POWER_UP_MSEC);
        gb_ok = (gb_ok && reg_write(REG_CONFIG, DLPF_44HZ));
        gb_ok = (gb_ok && reg_write(REG_SMPLRT_DIV, SAMPLE_DIV_100HZ));
        gb_ok = (gb_ok && reg_write(REG_GYRO_CONFIG, GYRO_RANGE_250DPS));
        gb_ok = (gb_ok && reg_write(REG_ACCEL_CONFIG, ACCEL_RANGE_2G));
        gb_ok = (gb_ok && imu_calibrate());
    }

    if (gb_ok)
    {
        log_printf("IMU: MPU-6050 at 0x%02X (WHO_AM_I 0x%02X), calibrated.\n",
               g_address, identity);
    }
    else
    {
        log_printf("IMU: MPU-6050 NOT found on SDA GP2 / SCL GP3. Hump "
               "detection is OFF; obstacles will always stop the car.\n");
    }

    return (gb_ok);
}   /* imu_init() */


/*!
 * @brief Measure the gyro bias and mounting tilt. Keep the car still.
 *
 * @return True if every calibration read succeeded.
 */
bool
imu_calibrate (void)
{
    int32_t accel_mdeg = 0;
    int32_t gyro       = 0;
    int32_t gyro_sum   = 0;
    int32_t accel_sum  = 0;
    bool    b_ok       = true;

    for (int32_t sample = 0; (sample < CAL_SAMPLES) && b_ok; sample++)
    {
        b_ok       = sample_read(&accel_mdeg, &gyro);
        gyro_sum  += gyro;
        accel_sum += accel_mdeg;
        sleep_us(CAL_SAMPLE_USEC);
    }

    if (b_ok)
    {
        g_gyro_bias      = gyro_sum / CAL_SAMPLES;
        g_accel_ref_mdeg = accel_sum / CAL_SAMPLES;
        g_pitch_mdeg     = 0;
        g_rate_mdps      = 0;
        g_last_usec      = time_us_64();
    }

    return (b_ok);
}   /* imu_calibrate() */


/*!
 * @brief Update the pitch estimate. Call every control tick.
 */
void
imu_task (void)
{
    int32_t accel_mdeg = 0;
    int32_t gyro       = 0;

    if (gb_ok && sample_read(&accel_mdeg, &gyro))
    {
        uint64_t now_usec = time_us_64();
        // NOTE: Ticks are 10 ms apart, so the gap always fits int64.
        int64_t  dt_usec  = (int64_t) (now_usec - g_last_usec);
        int32_t  pitch_accel = accel_mdeg - g_accel_ref_mdeg;

        g_last_usec = now_usec;

        // Gyro: rate in millidegrees per second, integrated over dt.
        // NOTE: The rate is at most 250000 mdps and dt about 10000 us, so
        // the 64-bit product cannot overflow and the result fits int32.
        g_rate_mdps   = ((gyro - g_gyro_bias) * MDEG_PER_DEG) /
                        GYRO_LSB_PER_DPS;
        g_pitch_mdeg += (int32_t) (((int64_t) g_rate_mdps * dt_usec) /
                                   USEC_PER_SECOND);

        // Accelerometer: remove drift slowly.
        g_pitch_mdeg += (pitch_accel - g_pitch_mdeg) / ACCEL_WEIGHT_DIV;
    }
}   /* imu_task() */


/*!
 * @brief Report whether the IMU is working.
 *
 * @return True if it was found and calibrated.
 */
bool
imu_is_ok (void)
{
    return (gb_ok);
}   /* imu_is_ok() */


/*!
 * @brief Report the pitch angle.
 *
 * @return Pitch in tenths of a degree; positive is nose up. 0 if the IMU
 *         is not working.
 */
int32_t
imu_pitch_x10 (void)
{
    return (g_pitch_mdeg / MDEG_PER_TENTH);
}   /* imu_pitch_x10() */


/*!
 * @brief Report the pitch rate.
 *
 * @return Pitch rate in tenths of a degree per second.
 */
int32_t
imu_pitch_rate_x10 (void)
{
    return (g_rate_mdps / MDEG_PER_TENTH);
}   /* imu_pitch_rate_x10() */


/*!
 * @brief Read one sample: accelerometer pitch and pitch-axis gyro counts.
 *
 * @param[out] p_accel_mdeg  Accelerometer pitch, in millidegrees, signed
 *                           so that nose up is positive.
 * @param[out] p_gyro_counts Raw pitch-axis gyro, signed so that nose-up
 *                           rotation is positive.
 *
 * @return True if the read succeeded.
 */
static bool
sample_read (int32_t * p_accel_mdeg, int32_t * p_gyro_counts)
{
    uint8_t  raw[RAW_FRAME_BYTES] = { 0u };
    bool     b_ok        = reg_read(REG_ACCEL_XOUT_H, raw, sizeof(raw));
    uint32_t accel_idx   = IMU_FORWARD_ACCEL_AXIS * BYTES_PER_VALUE;
    uint32_t gyro_idx    = GYRO_FRAME_OFFSET +
                           (IMU_PITCH_GYRO_AXIS * BYTES_PER_VALUE);

    if (b_ok)
    {
        // NOTE: Each value is a big-endian two's-complement 16-bit number;
        // the casts rebuild that bit pattern before sign-extending it.
        int32_t forward = (int16_t) (((uint16_t) raw[accel_idx] << 8) |
                                     raw[accel_idx + 1u]);
        int32_t rate    = (int16_t) (((uint16_t) raw[gyro_idx] << 8) |
                                     raw[gyro_idx + 1u]);

        // Small angle: pitch (rad) = forward g, so mdeg = g x 57296.
        *p_accel_mdeg  = IMU_ACCEL_PITCH_SIGN *
                         ((forward * MDEG_PER_RAD) / ACCEL_LSB_PER_G);
        *p_gyro_counts = IMU_GYRO_PITCH_SIGN * rate;
    }

    return (b_ok);
}   /* sample_read() */


/*!
 * @brief Write one MPU-6050 register.
 *
 * @param[in] reg   The register.
 * @param[in] value The value.
 *
 * @return True if the device acknowledged.
 */
static bool
reg_write (uint8_t reg, uint8_t value)
{
    uint8_t frame[2] = { reg, value };
    int     written  = i2c_write_timeout_us(IMU_I2C, g_address, frame,
                                            sizeof(frame), false,
                                            I2C_TIMEOUT_USEC);

    return ((int) sizeof(frame) == written);
}   /* reg_write() */


/*!
 * @brief Read consecutive MPU-6050 registers.
 *
 * @param[in]  reg    The first register.
 * @param[out] p_data Where to store the bytes.
 * @param[in]  length The number of bytes (at most 14).
 *
 * @return True if every byte was read.
 */
static bool
reg_read (uint8_t reg, uint8_t * p_data, size_t length)
{
    bool b_ok   = false;
    int  result = i2c_write_timeout_us(IMU_I2C, g_address, &reg, 1u, true,
                                       I2C_TIMEOUT_USEC);

    if (1 == result)
    {
        result = i2c_read_timeout_us(IMU_I2C, g_address, p_data, length,
                                     false, I2C_TIMEOUT_USEC);

        // NOTE: length is at most 14, so it fits an int.
        b_ok = ((int) length == result);
    }

    return (b_ok);
}   /* reg_read() */

/*** end of file ***/
