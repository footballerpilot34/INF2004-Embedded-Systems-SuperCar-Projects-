/** @file gyro_test.c
 *
 * @brief MPU-6050 gyroscope test: prints X/Y/Z rotation rates, the angle
 *        turned about each axis, and X/Y/Z acceleration.
 *
 * @par How it works
 * 1. Finds the MPU-6050 on I2C1 (SDA GP2, SCL GP3) at 0x68 or 0x69.
 * 2. Sets +/-250 dps gyro, +/-2 g accelerometer, 44 Hz filter, 100 Hz rate.
 * 3. Averages 1 s of gyro readings while the car is still, to remove bias.
 * 4. Samples every 10 ms (the IMU Task period).
 *
 * @par Output modes
 * - Teleplot (default): sends ">name:value" lines every 20 ms, which the
 *   Teleplot VS Code extension draws as live graphs.
 * - Text: prints one readable line every 100 ms.
 *
 * @par Serial commands
 * Type 'z' to zero the angles, 'c' to recalibrate (keep the car still), or
 * 't' to switch between Teleplot and text output.
 *
 * @par Coding standard
 * BARR-C:2018. Deviations: Rule 4.1.d (main() lives in a file without
 * "main" in its name) and Rule 5.2.a (Pico SDK I2C calls return int).
 *
 * TODO: Record the name of the person who approved these deviations.
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"


// IMU bus wiring and speed.
//
#define I2C_PORT                i2c1
#define SDA_PIN                 2u
#define SCL_PIN                 3u
#define I2C_HZ                  100000u
#define I2C_TIMEOUT_USEC        5000u
#define I2C_FIRST_ADDRESS       0x08u   // 0x00-0x07 are reserved.
#define I2C_LAST_ADDRESS        0x77u   // 0x78-0x7F are reserved.

// MPU-6050 addresses and registers.
//
#define MPU_ADDRESS_LOW         0x68u
#define MPU_ADDRESS_HIGH        0x69u
#define REG_SMPLRT_DIV          0x19u
#define REG_CONFIG              0x1Au
#define REG_GYRO_CONFIG         0x1Bu
#define REG_ACCEL_CONFIG        0x1Cu
#define REG_ACCEL_XOUT_H        0x3Bu
#define REG_PWR_MGMT_1          0x6Bu
#define REG_WHO_AM_I            0x75u

// MPU-6050 settings.
//
#define PWR_WAKE_PLL_X          0x01u   // Leave sleep; clock from gyro X.
#define DLPF_44HZ               0x03u   // 44 Hz filter, 1 kHz internal rate.
#define SAMPLE_DIV_100HZ        9u      // 1 kHz / (1 + 9) = 100 Hz.
#define GYRO_RANGE_250DPS       0x00u
#define ACCEL_RANGE_2G          0x00u
#define POWER_UP_MSEC           100u

// Raw data layout: accel X/Y/Z, temperature, gyro X/Y/Z, 2 bytes each.
//
#define NUM_AXES                3u
#define RAW_FRAME_BYTES         14u
#define GYRO_FRAME_OFFSET       8u
#define BYTES_PER_VALUE         2u

// Scaling. Bias is kept in 1/16 counts so averaging keeps sub-count detail.
//
#define GYRO_LSB_PER_DPS        131
#define ACCEL_LSB_PER_G         16384
#define BIAS_FRACTION           16
#define USEC_PER_SECOND         1000000
#define TENTHS                  10
#define HUNDREDTHS              100

// Calibration.
//
#define CAL_SAMPLES             100     // 1 s at 10 ms per sample.
#define CAL_MAX_SPREAD          262     // 2 dps; more means the car moved.

// Timing.
//
#define SAMPLE_PERIOD_USEC      10000u
#define TEXT_EVERY_SAMPLES      10u     // 100 ms between text lines.
#define PLOT_EVERY_SAMPLES      2u      // 20 ms (50 Hz) between plot points.
#define USB_WAIT_MSEC           10000u
#define USB_POLL_MSEC           100u
#define RETRY_MSEC              2000u
#define LINE_SETTLE_USEC        50u

// Serial commands.
//
#define CMD_ZERO_ANGLES         'z'
#define CMD_RECALIBRATE         'c'
#define CMD_TOGGLE_OUTPUT       't'


static void usb_wait(void);
static void bus_init(void);
static bool reg_write(uint8_t address, uint8_t reg, uint8_t value);
static bool reg_read(uint8_t address, uint8_t reg, uint8_t * p_data,
                     size_t length);
static bool mpu_find(uint8_t * p_address);
static void bus_diagnose(void);
static bool mpu_configure(uint8_t address);
static bool mpu_read(uint8_t address, int32_t * p_accel, int32_t * p_gyro);
static bool gyro_calibrate(uint8_t address, int32_t * p_bias_q4);
static void fixed_print(char const * p_label, int32_t value, int32_t scale);
static void plot_print(char const * p_name, int32_t value, int32_t scale);
static void values_scale(int32_t const * p_rate_q4,
                         int64_t const * p_angle_sum,
                         int32_t const * p_accel, int32_t * p_rate_x10,
                         int32_t * p_angle_x10, int32_t * p_accel_x100);
static void report_text(int32_t const * p_rate_x10,
                        int32_t const * p_angle_x10,
                        int32_t const * p_accel_x100);
static void report_plot(int32_t const * p_rate_x10,
                        int32_t const * p_angle_x10,
                        int32_t const * p_accel_x100);
static void test_task(uint8_t address);


/*!
 * @brief Find the MPU-6050, then run the gyroscope test forever.
 *
 * @return Never returns.
 */
int
main (void)
{
    uint8_t address = 0u;

    stdio_init_all();
    usb_wait();
    bus_init();

    printf("\n=== MPU-6050 gyroscope test (SDA GP2, SCL GP3) ===\n");

    for (;;)
    {
        if (!mpu_find(&address))
        {
            printf("MPU-6050 not found at 0x68 or 0x69.\n");
            bus_diagnose();
            printf("Retrying in 2 s.\n\n");
            sleep_ms(RETRY_MSEC);
        }
        else if (!mpu_configure(address))
        {
            printf("MPU-6050 found but setup failed; retrying in 2 s.\n");
            sleep_ms(RETRY_MSEC);
        }
        else
        {
            test_task(address);
        }
    }

    return (0);
}   /* main() */


/*!
 * @brief Calibrate, then sample every 10 ms and print every 100 ms.
 *
 * @par
 * Returns only if a read fails, so main() can find the sensor again.
 *
 * @param[in] address The MPU-6050's I2C address.
 */
static void
test_task (uint8_t address)
{
    int32_t  bias_q4[NUM_AXES]    = { 0 };
    int64_t  angle_sum[NUM_AXES]  = { 0 };
    int32_t  accel[NUM_AXES]      = { 0 };
    int32_t  gyro[NUM_AXES]       = { 0 };
    int32_t  rate_q4[NUM_AXES]    = { 0 };
    int32_t  rate_x10[NUM_AXES]   = { 0 };
    int32_t  angle_x10[NUM_AXES]  = { 0 };
    int32_t  accel_x100[NUM_AXES] = { 0 };
    uint32_t sample_count         = 0u;
    bool     b_plot_mode          = true;
    bool     b_ok                 = gyro_calibrate(address, bias_q4);
    uint64_t last_usec            = time_us_64();
    uint64_t next_usec            = last_usec + SAMPLE_PERIOD_USEC;

    while (b_ok)
    {
        int command = getchar_timeout_us(0u);

        if (CMD_ZERO_ANGLES == command)
        {
            for (uint32_t axis = 0u; axis < NUM_AXES; axis++)
            {
                angle_sum[axis] = 0;
            }
            printf("Angles zeroed.\n");
        }
        else if (CMD_RECALIBRATE == command)
        {
            b_ok = gyro_calibrate(address, bias_q4);
        }
        else if (CMD_TOGGLE_OUTPUT == command)
        {
            b_plot_mode = !b_plot_mode;
            printf("Output: %s\n", (b_plot_mode ? "Teleplot graphs"
                                                 : "text"));
        }
        else
        {
            // No command waiting.
        }

        // Wait for the next 10 ms sample slot.
        while (time_us_64() < next_usec)
        {
            tight_loop_contents();
        }
        next_usec += SAMPLE_PERIOD_USEC;

        if (b_ok && mpu_read(address, accel, gyro))
        {
            uint64_t now_usec = time_us_64();
            // NOTE: Samples are 10 ms apart, so the gap always fits int64.
            int64_t  dt_usec  = (int64_t) (now_usec - last_usec);

            last_usec = now_usec;

            // Remove bias, then add rate x time to each axis's angle.
            for (uint32_t axis = 0u; axis < NUM_AXES; axis++)
            {
                rate_q4[axis]    = (gyro[axis] * BIAS_FRACTION) -
                                   bias_q4[axis];
                angle_sum[axis] += (int64_t) rate_q4[axis] * dt_usec;
            }

            sample_count++;
        }
        else
        {
            printf("Read FAILED: connection to the MPU-6050 lost.\n");
            b_ok = false;
        }

        if (b_ok)
        {
            values_scale(rate_q4, angle_sum, accel, rate_x10, angle_x10,
                         accel_x100);

            if (b_plot_mode &&
                (0u == (sample_count % PLOT_EVERY_SAMPLES)))
            {
                report_plot(rate_x10, angle_x10, accel_x100);
            }
            else if ((!b_plot_mode) &&
                     (0u == (sample_count % TEXT_EVERY_SAMPLES)))
            {
                report_text(rate_x10, angle_x10, accel_x100);
            }
            else
            {
                // Not time to report yet.
            }
        }
    }
}   /* test_task() */


/*!
 * @brief Convert raw sums into printable fixed-point values.
 *
 * @param[in]  p_rate_q4    Bias-corrected gyro rates, in 1/16 counts.
 * @param[in]  p_angle_sum  Angle sums, in 1/16 counts x microseconds.
 * @param[in]  p_accel      Raw accelerometer counts.
 * @param[out] p_rate_x10   Rates in tenths of a degree per second.
 * @param[out] p_angle_x10  Angles in tenths of a degree.
 * @param[out] p_accel_x100 Acceleration in hundredths of a g.
 */
static void
values_scale (int32_t const * p_rate_q4, int64_t const * p_angle_sum,
              int32_t const * p_accel, int32_t * p_rate_x10,
              int32_t * p_angle_x10, int32_t * p_accel_x100)
{
    // NOTE: The angle divisor is counts x microseconds per tenth of a
    // degree, so the quotient always fits int32 for any angle a test run
    // can reach.
    int64_t angle_divisor = ((int64_t) GYRO_LSB_PER_DPS * BIAS_FRACTION *
                             USEC_PER_SECOND) / TENTHS;

    for (uint32_t axis = 0u; axis < NUM_AXES; axis++)
    {
        p_rate_x10[axis]   = (p_rate_q4[axis] * TENTHS) /
                             (GYRO_LSB_PER_DPS * BIAS_FRACTION);
        p_angle_x10[axis]  = (int32_t) (p_angle_sum[axis] / angle_divisor);
        p_accel_x100[axis] = (p_accel[axis] * HUNDREDTHS) / ACCEL_LSB_PER_G;
    }
}   /* values_scale() */


/*!
 * @brief Print one readable line of rates, angles and acceleration.
 *
 * @param[in] p_rate_x10   Rates in tenths of a degree per second.
 * @param[in] p_angle_x10  Angles in tenths of a degree.
 * @param[in] p_accel_x100 Acceleration in hundredths of a g.
 */
static void
report_text (int32_t const * p_rate_x10, int32_t const * p_angle_x10,
             int32_t const * p_accel_x100)
{
    printf("Gyro dps");
    fixed_print(" X:", p_rate_x10[0], TENTHS);
    fixed_print(" Y:", p_rate_x10[1], TENTHS);
    fixed_print(" Z:", p_rate_x10[2], TENTHS);

    printf(" | Angle deg");
    fixed_print(" X:", p_angle_x10[0], TENTHS);
    fixed_print(" Y:", p_angle_x10[1], TENTHS);
    fixed_print(" Z:", p_angle_x10[2], TENTHS);

    printf(" | Accel g");
    fixed_print(" X:", p_accel_x100[0], HUNDREDTHS);
    fixed_print(" Y:", p_accel_x100[1], HUNDREDTHS);
    fixed_print(" Z:", p_accel_x100[2], HUNDREDTHS);

    printf("\n");
}   /* report_text() */


/*!
 * @brief Send one point to each Teleplot graph.
 *
 * @par
 * Teleplot draws one graph per name; X/Y/Z traces of the same quantity can
 * be dragged onto one chart in the Teleplot window.
 *
 * @param[in] p_rate_x10   Rates in tenths of a degree per second.
 * @param[in] p_angle_x10  Angles in tenths of a degree.
 * @param[in] p_accel_x100 Acceleration in hundredths of a g.
 */
static void
report_plot (int32_t const * p_rate_x10, int32_t const * p_angle_x10,
             int32_t const * p_accel_x100)
{
    plot_print("gyroX_dps", p_rate_x10[0], TENTHS);
    plot_print("gyroY_dps", p_rate_x10[1], TENTHS);
    plot_print("gyroZ_dps", p_rate_x10[2], TENTHS);
    plot_print("angleX_deg", p_angle_x10[0], TENTHS);
    plot_print("angleY_deg", p_angle_x10[1], TENTHS);
    plot_print("angleZ_deg", p_angle_x10[2], TENTHS);
    plot_print("accelX_g", p_accel_x100[0], HUNDREDTHS);
    plot_print("accelY_g", p_accel_x100[1], HUNDREDTHS);
    plot_print("accelZ_g", p_accel_x100[2], HUNDREDTHS);
}   /* report_plot() */


/*!
 * @brief Print one Teleplot point, such as ">gyroZ_dps:-12.3".
 *
 * @param[in] p_name The trace name.
 * @param[in] value  The value, scaled by scale.
 * @param[in] scale  10 for one decimal place, 100 for two.
 */
static void
plot_print (char const * p_name, int32_t value, int32_t scale)
{
    int32_t magnitude = ((value < 0) ? -value : value);
    int     digits    = ((HUNDREDTHS == scale) ? 2 : 1);

    printf(">%s:%s%" PRId32 ".%0*" PRId32 "\n", p_name,
           ((value < 0) ? "-" : ""), magnitude / scale, digits,
           magnitude % scale);
}   /* plot_print() */


/*!
 * @brief Average 1 s of still readings to find each gyro axis's bias.
 *
 * @param[in]  address   The MPU-6050's I2C address.
 * @param[out] p_bias_q4 Three biases, in 1/16 counts.
 *
 * @return True if every calibration read succeeded.
 */
static bool
gyro_calibrate (uint8_t address, int32_t * p_bias_q4)
{
    int32_t sum[NUM_AXES]     = { 0 };
    int32_t minimum[NUM_AXES] = { INT32_MAX, INT32_MAX, INT32_MAX };
    int32_t maximum[NUM_AXES] = { INT32_MIN, INT32_MIN, INT32_MIN };
    int32_t accel[NUM_AXES]   = { 0 };
    int32_t gyro[NUM_AXES]    = { 0 };
    bool    b_ok              = true;
    bool    b_moved           = false;

    printf("Calibrating: keep the car still for 1 s...\n");

    for (uint32_t sample = 0u; (sample < CAL_SAMPLES) && b_ok; sample++)
    {
        b_ok = mpu_read(address, accel, gyro);

        for (uint32_t axis = 0u; (axis < NUM_AXES) && b_ok; axis++)
        {
            sum[axis]    += gyro[axis];
            minimum[axis] = ((gyro[axis] < minimum[axis]) ? gyro[axis]
                                                          : minimum[axis]);
            maximum[axis] = ((gyro[axis] > maximum[axis]) ? gyro[axis]
                                                          : maximum[axis]);
        }

        sleep_us(SAMPLE_PERIOD_USEC);
    }

    for (uint32_t axis = 0u; (axis < NUM_AXES) && b_ok; axis++)
    {
        p_bias_q4[axis] = (sum[axis] * BIAS_FRACTION) / CAL_SAMPLES;
        b_moved = (b_moved || ((maximum[axis] - minimum[axis]) >
                               CAL_MAX_SPREAD));
    }

    if (!b_ok)
    {
        printf("Calibration FAILED: could not read the MPU-6050.\n");
    }
    else
    {
        printf("Calibrated. Bias in dps:");
        fixed_print(" X:", (p_bias_q4[0] * TENTHS) /
                    (GYRO_LSB_PER_DPS * BIAS_FRACTION), TENTHS);
        fixed_print(" Y:", (p_bias_q4[1] * TENTHS) /
                    (GYRO_LSB_PER_DPS * BIAS_FRACTION), TENTHS);
        fixed_print(" Z:", (p_bias_q4[2] * TENTHS) /
                    (GYRO_LSB_PER_DPS * BIAS_FRACTION), TENTHS);
        printf("\n%s", (b_moved ? "WARNING: the car moved during "
                                  "calibration; type 'c' to redo it.\n"
                                : ""));
        printf("Type 'z' to zero the angles, 'c' to recalibrate, 't' for "
               "text/graph output.\n");
    }

    return (b_ok);
}   /* gyro_calibrate() */


/*!
 * @brief Read acceleration and rotation rate on all three axes.
 *
 * @param[in]  address The MPU-6050's I2C address.
 * @param[out] p_accel Raw accelerometer counts, X/Y/Z.
 * @param[out] p_gyro  Raw gyroscope counts, X/Y/Z.
 *
 * @return True if the read succeeded.
 */
static bool
mpu_read (uint8_t address, int32_t * p_accel, int32_t * p_gyro)
{
    uint8_t raw[RAW_FRAME_BYTES] = { 0u };
    bool    b_ok = reg_read(address, REG_ACCEL_XOUT_H, raw, sizeof(raw));

    for (uint32_t axis = 0u; (axis < NUM_AXES) && b_ok; axis++)
    {
        uint32_t accel_idx = axis * BYTES_PER_VALUE;
        uint32_t gyro_idx  = GYRO_FRAME_OFFSET + accel_idx;

        // NOTE: Each value is a big-endian two's-complement 16-bit number;
        // the casts rebuild that bit pattern before sign-extending it.
        p_accel[axis] = (int16_t) (((uint16_t) raw[accel_idx] << 8) |
                                   raw[accel_idx + 1u]);
        p_gyro[axis]  = (int16_t) (((uint16_t) raw[gyro_idx] << 8) |
                                   raw[gyro_idx + 1u]);
    }

    return (b_ok);
}   /* mpu_read() */


/*!
 * @brief Find the MPU-6050 at 0x68 or 0x69.
 *
 * @param[out] p_address The address it answered at.
 *
 * @return True if it was found.
 */
static bool
mpu_find (uint8_t * p_address)
{
    static uint8_t const addresses[] = { MPU_ADDRESS_LOW, MPU_ADDRESS_HIGH };
    uint8_t              identity    = 0u;
    bool                 b_found     = false;

    for (uint32_t idx = 0u; (idx < sizeof(addresses)) && (!b_found); idx++)
    {
        if (reg_read(addresses[idx], REG_WHO_AM_I, &identity, 1u))
        {
            *p_address = addresses[idx];
            b_found    = true;
            printf("Found MPU-6050 at 0x%02X (WHO_AM_I 0x%02X).\n",
                   addresses[idx], identity);
        }
    }

    return (b_found);
}   /* mpu_find() */


/*!
 * @brief Explain why the MPU-6050 was not found: check the idle bus lines,
 *        then list every address that answers.
 */
static void
bus_diagnose (void)
{
    bool     b_sda_high = false;
    bool     b_scl_high = false;
    uint32_t num_found  = 0u;

    // Read the idle lines as plain GPIO inputs (the pull-ups stay on).
    gpio_set_function(SDA_PIN, GPIO_FUNC_SIO);
    gpio_set_function(SCL_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(SDA_PIN, GPIO_IN);
    gpio_set_dir(SCL_PIN, GPIO_IN);
    sleep_us(LINE_SETTLE_USEC);
    b_sda_high = gpio_get(SDA_PIN);
    b_scl_high = gpio_get(SCL_PIN);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

    printf("  Idle lines: SDA(GP2)=%s SCL(GP3)=%s\n",
           (b_sda_high ? "HIGH" : "LOW"), (b_scl_high ? "HIGH" : "LOW"));

    printf("  Devices found:");
    for (uint8_t address = I2C_FIRST_ADDRESS; address <= I2C_LAST_ADDRESS;
         address++)
    {
        uint8_t dummy  = 0u;
        int     result = i2c_read_timeout_us(I2C_PORT, address, &dummy, 1u,
                                             false, I2C_TIMEOUT_USEC);

        if (1 == result)
        {
            printf(" 0x%02X", address);
            num_found++;
        }
    }
    printf((0u == num_found) ? " none\n" : "\n");

    if ((!b_sda_high) || (!b_scl_high))
    {
        printf("  -> A LOW line: the IMU has no power (check VCC/GND), or "
               "that wire touches GND.\n");
    }
    else if (0u == num_found)
    {
        printf("  -> Lines OK but nothing answers: SDA and SCL are probably "
               "swapped, or the wires are on the wrong pins.\n");
    }
    else
    {
        printf("  -> Something answers, but not at 0x68/0x69: this is not an "
               "MPU-6050. Tell me the address(es) above.\n");
    }
}   /* bus_diagnose() */


/*!
 * @brief Wake the MPU-6050 and set its ranges, filter and sample rate.
 *
 * @param[in] address The MPU-6050's I2C address.
 *
 * @return True if every register was written.
 */
static bool
mpu_configure (uint8_t address)
{
    bool b_ok = reg_write(address, REG_PWR_MGMT_1, PWR_WAKE_PLL_X);

    sleep_ms(POWER_UP_MSEC);
    b_ok = (b_ok && reg_write(address, REG_CONFIG, DLPF_44HZ));
    b_ok = (b_ok && reg_write(address, REG_SMPLRT_DIV, SAMPLE_DIV_100HZ));
    b_ok = (b_ok && reg_write(address, REG_GYRO_CONFIG, GYRO_RANGE_250DPS));
    b_ok = (b_ok && reg_write(address, REG_ACCEL_CONFIG, ACCEL_RANGE_2G));

    return (b_ok);
}   /* mpu_configure() */


/*!
 * @brief Write one register.
 *
 * @param[in] address The device's I2C address.
 * @param[in] reg     The register to write.
 * @param[in] value   The value to write.
 *
 * @return True if the device acknowledged both bytes.
 */
static bool
reg_write (uint8_t address, uint8_t reg, uint8_t value)
{
    uint8_t frame[2] = { reg, value };
    int     written  = i2c_write_timeout_us(I2C_PORT, address, frame,
                                            sizeof(frame), false,
                                            I2C_TIMEOUT_USEC);

    return ((int) sizeof(frame) == written);
}   /* reg_write() */


/*!
 * @brief Read one or more consecutive registers.
 *
 * @param[in]  address The device's I2C address.
 * @param[in]  reg     The first register to read.
 * @param[out] p_data  Where to store the bytes.
 * @param[in]  length  The number of bytes to read.
 *
 * @return True if every byte was read.
 */
static bool
reg_read (uint8_t address, uint8_t reg, uint8_t * p_data, size_t length)
{
    bool b_ok   = false;
    int  result = i2c_write_timeout_us(I2C_PORT, address, &reg, 1u, true,
                                       I2C_TIMEOUT_USEC);

    if (1 == result)
    {
        result = i2c_read_timeout_us(I2C_PORT, address, p_data, length,
                                     false, I2C_TIMEOUT_USEC);

        // NOTE: length is at most 14 here, so it fits an int.
        b_ok = ((int) length == result);
    }

    return (b_ok);
}   /* reg_read() */


/*!
 * @brief Print a fixed-point value with a sign, such as " X: -12.3".
 *
 * @param[in] p_label The label to print first.
 * @param[in] value   The value, scaled by scale.
 * @param[in] scale   10 for one decimal place, 100 for two.
 */
static void
fixed_print (char const * p_label, int32_t value, int32_t scale)
{
    char    sign      = ((value < 0) ? '-' : '+');
    int32_t magnitude = ((value < 0) ? -value : value);
    int     digits    = ((HUNDREDTHS == scale) ? 2 : 1);

    printf("%s%c%3" PRId32 ".%0*" PRId32, p_label, sign, magnitude / scale,
           digits, magnitude % scale);
}   /* fixed_print() */


/*!
 * @brief Set up I2C1 on GP2 (SDA) and GP3 (SCL) at 100 kHz.
 */
static void
bus_init (void)
{
    i2c_init(I2C_PORT, I2C_HZ);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
}   /* bus_init() */


/*!
 * @brief Wait up to 10 s for a serial monitor to connect.
 */
static void
usb_wait (void)
{
    uint32_t waited_ms = 0u;

    while ((!stdio_usb_connected()) && (waited_ms < USB_WAIT_MSEC))
    {
        sleep_ms(USB_POLL_MSEC);
        waited_ms += USB_POLL_MSEC;
    }
}   /* usb_wait() */

/*** end of file ***/
