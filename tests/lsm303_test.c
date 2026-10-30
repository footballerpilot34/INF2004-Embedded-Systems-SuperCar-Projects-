/** @file lsm303_test.c
 *
 * @brief Standalone USB serial check for all six LSM303DLHC axes.
 *
 * Uses the Robo Pico's Grove 2 I2C bus (I2C1, GP2 SDA / GP3 SCL), checks
 * both LSM303DLHC core identities, configures the accelerometer for +/-2 g
 * high-resolution output and the magnetometer for +/-1.3 gauss, and prints
 * readings in physical units five times per second.
 *
 * Each reading is also emitted in Teleplot format (">name:value"), so the
 * VS Code Teleplot extension can graph all six axes live from the serial
 * port; the human-readable lines still appear in its console.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

#include "config.h"


#define TEST_I2C                    i2c1
#define TEST_I2C_HZ                 100000u
#define I2C_TIMEOUT_USEC            5000u
#define SAMPLE_PERIOD_MSEC          200u
#define USB_CONNECT_WAIT_MSEC       10000u
#define USB_CONNECT_POLL_MSEC       100u

#define ACCEL_ADDRESS               0x19u
#define MAG_ADDRESS                 0x1Eu

#define ACCEL_WHO_AM_I              0x0Fu
#define ACCEL_WHO_AM_I_VALUE        0x33u
#define ACCEL_CTRL_REG1             0x20u
#define ACCEL_CTRL_REG4             0x23u
#define ACCEL_OUT_X_L               0x28u
#define ACCEL_AUTO_INCREMENT        0x80u

#define MAG_CRA_REG                 0x00u
#define MAG_CRB_REG                 0x01u
#define MAG_MODE_REG                0x02u
#define MAG_OUT_X_H                 0x03u
#define MAG_ID_REG                  0x0Au
#define MAG_ID_H                    0x48u
#define MAG_ID_4                    0x34u
#define MAG_ID_3                    0x33u

#define ACCEL_SENSITIVITY_MG        1
#define MAG_XY_SENSITIVITY_LSB_G    1100
#define MAG_Z_SENSITIVITY_LSB_G     980
#define MILLI_PER_UNIT              1000
#define MAG_RAW_MIN                 (-2048)
#define MAG_RAW_MAX                 2047
#define MAG_INVALID_COUNT           10u
#define RAW_AXIS_BYTES              6u


typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} axis_data_t;


static bool    reg_write(uint8_t address, uint8_t reg, uint8_t value);
static bool    reg_read(uint8_t address, uint8_t reg, uint8_t * p_data,
                        size_t length);
static bool    lsm303_init(void);
static bool    lsm303_read(axis_data_t * p_accel, axis_data_t * p_mag);
static int16_t signed_be(uint8_t msb, uint8_t lsb);
static int32_t accel_counts(int16_t raw);
static void    print_value(char const * p_axis, int32_t milli,
                           char const * p_units);
static void    print_mag_value(char const * p_axis, int16_t raw,
                               int32_t sensitivity);
static void    teleplot_value(char const * p_name, int32_t milli);
static void    teleplot_mag(char const * p_name, int16_t raw,
                            int32_t sensitivity);


/*!
 * @brief Initialize the USB console and print all six sensor axes forever.
 *
 * @return One if either sensor is missing or fails configuration; otherwise
 *         this test keeps running and does not return.
 */
int
main (void)
{
    axis_data_t accel     = { 0, 0, 0 };
    axis_data_t mag       = { 0, 0, 0 };
    uint32_t    wait_msec = 0u;
    uint32_t    zero_count = 0u;

    stdio_init_all();

    // Give the USB CDC serial monitor time to enumerate and open the port.
    while ((!stdio_usb_connected()) &&
           (wait_msec < USB_CONNECT_WAIT_MSEC))
    {
        sleep_ms(USB_CONNECT_POLL_MSEC);
        wait_msec += USB_CONNECT_POLL_MSEC;
    }
    sleep_ms(500u);

    i2c_init(TEST_I2C, TEST_I2C_HZ);
    gpio_set_function(PIN_IMU_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_IMU_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_IMU_SDA);
    gpio_pull_up(PIN_IMU_SCL);

    printf("LSM303DLHC sensor test\r\n");
    printf("I2C1: SDA GP%u, SCL GP%u; accel 0x%02X, mag 0x%02X\r\n",
           (unsigned int) PIN_IMU_SDA, (unsigned int) PIN_IMU_SCL,
           ACCEL_ADDRESS, MAG_ADDRESS);
    stdio_flush();

    if (!lsm303_init())
    {
        return (1);
    }

    printf("Reading all six axes at 5 Hz. Hold still, then tilt and rotate.\r\n");
    stdio_flush();

    for (;;)
    {
        if (lsm303_read(&accel, &mag))
        {
            printf("========== LSM303DLHC ==========\r\n");
            printf("Accelerometer:\r\n");
            print_value("AX", accel_counts(accel.x) * ACCEL_SENSITIVITY_MG,
                        "g");
            print_value("AY", accel_counts(accel.y) * ACCEL_SENSITIVITY_MG,
                        "g");
            print_value("AZ", accel_counts(accel.z) * ACCEL_SENSITIVITY_MG,
                        "g");
            printf("\r\nMagnetometer:\r\n");
            print_mag_value("MX", mag.x, MAG_XY_SENSITIVITY_LSB_G);
            print_mag_value("MY", mag.y, MAG_XY_SENSITIVITY_LSB_G);
            print_mag_value("MZ", mag.z, MAG_Z_SENSITIVITY_LSB_G);
            printf("================================\r\n\r\n");

            // Teleplot graph series: accelerometer in g, magnetometer in gauss.
            teleplot_value("accel_x", accel_counts(accel.x) *
                                      ACCEL_SENSITIVITY_MG);
            teleplot_value("accel_y", accel_counts(accel.y) *
                                      ACCEL_SENSITIVITY_MG);
            teleplot_value("accel_z", accel_counts(accel.z) *
                                      ACCEL_SENSITIVITY_MG);
            teleplot_mag("mag_x", mag.x, MAG_XY_SENSITIVITY_LSB_G);
            teleplot_mag("mag_y", mag.y, MAG_XY_SENSITIVITY_LSB_G);
            teleplot_mag("mag_z", mag.z, MAG_Z_SENSITIVITY_LSB_G);

            if ((0 == accel.x) && (0 == accel.y) && (0 == accel.z) &&
                (0 == mag.x) && (0 == mag.y) && (0 == mag.z))
            {
                zero_count++;
                if (MAG_INVALID_COUNT == zero_count)
                {
                    printf("WARNING: all six raw readings stayed zero for "
                           "2 seconds; check sensor power and wiring.\r\n");
                }
            }
            else
            {
                zero_count = 0u;
            }
        }
        else
        {
            printf("I2C read failed at address 0x%02X or 0x%02X; "
                   "check wiring; retrying.\r\n",
                   ACCEL_ADDRESS, MAG_ADDRESS);
        }

        stdio_flush();
        sleep_ms(SAMPLE_PERIOD_MSEC);
    }

    return (0);
}   /* main() */


/*!
 * @brief Verify both sensor identities, then enable continuous output.
 *
 * @return True when both sensor cores are present and configured.
 */
static bool
lsm303_init (void)
{
    uint8_t accel_id = 0u;
    uint8_t mag_id[3] = { 0u, 0u, 0u };
    bool    accel_ok = false;
    bool    mag_ok   = false;

    accel_ok = reg_read(ACCEL_ADDRESS, ACCEL_WHO_AM_I, &accel_id, 1u) &&
                (ACCEL_WHO_AM_I_VALUE == accel_id);
    mag_ok   = reg_read(MAG_ADDRESS, MAG_ID_REG, mag_id, sizeof(mag_id)) &&
                (MAG_ID_H == mag_id[0]) && (MAG_ID_4 == mag_id[1]) &&
                (MAG_ID_3 == mag_id[2]);

    if (!accel_ok)
    {
        printf("ERROR: accelerometer not detected at 7-bit address 0x%02X "
               "(WHO_AM_I 0x%02X, expected 0x%02X).\r\n",
               ACCEL_ADDRESS, accel_id, ACCEL_WHO_AM_I_VALUE);
    }
    if (!mag_ok)
    {
        printf("ERROR: magnetometer not detected at 7-bit address 0x%02X "
               "(ID %02X %02X %02X, expected 48 34 33).\r\n",
               MAG_ADDRESS, mag_id[0], mag_id[1], mag_id[2]);
    }

    if (accel_ok && mag_ok)
    {
        accel_ok = reg_write(ACCEL_ADDRESS, ACCEL_CTRL_REG1, 0x27u) &&
                   reg_write(ACCEL_ADDRESS, ACCEL_CTRL_REG4, 0x88u);
        mag_ok   = reg_write(MAG_ADDRESS, MAG_CRA_REG, 0x10u) &&
                   reg_write(MAG_ADDRESS, MAG_CRB_REG, 0x20u) &&
                   reg_write(MAG_ADDRESS, MAG_MODE_REG, 0x00u);

        if (!accel_ok)
        {
            printf("ERROR: accelerometer configuration failed at 0x%02X.\r\n",
                   ACCEL_ADDRESS);
        }
        if (!mag_ok)
        {
            printf("ERROR: magnetometer configuration failed at 0x%02X.\r\n",
                   MAG_ADDRESS);
        }
    }

    stdio_flush();
    return (accel_ok && mag_ok);
}   /* lsm303_init() */


/*!
 * @brief Read six output registers from each sensor core.
 *
 * The accelerometer outputs little-endian, 12-bit high-resolution values
 * left-aligned in a 16-bit word. The magnetometer outputs big-endian X, Z,
 * Y words in that register order.
 *
 * @return True if both I2C reads completed.
 */
static bool
lsm303_read (axis_data_t * p_accel, axis_data_t * p_mag)
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
        p_accel->x = signed_be(accel_data[1], accel_data[0]);
        p_accel->y = signed_be(accel_data[3], accel_data[2]);
        p_accel->z = signed_be(accel_data[5], accel_data[4]);

        p_mag->x = signed_be(mag_data[0], mag_data[1]);
        p_mag->z = signed_be(mag_data[2], mag_data[3]);
        p_mag->y = signed_be(mag_data[4], mag_data[5]);
    }

    return (accel_ok && mag_ok);
}   /* lsm303_read() */


/*!
 * @brief Join two big-endian bytes as a portable signed 16-bit value.
 */
static int16_t
signed_be (uint8_t msb, uint8_t lsb)
{
    uint16_t bits  = (uint16_t) (((uint16_t) msb << 8) | lsb);
    int32_t  value = ((bits <= INT16_MAX) ? (int32_t) bits :
                      ((int32_t) bits - 65536));

    return ((int16_t) value);
}   /* signed_be() */


/*!
 * @brief Convert left-aligned, signed 12-bit accelerometer data to counts.
 */
static int32_t
accel_counts (int16_t raw)
{
    int32_t counts = (int32_t) raw;

    if (counts < 0)
    {
        counts = -(((-counts) + 15) / 16);
    }
    else
    {
        counts /= 16;
    }

    return (counts);
}   /* accel_counts() */


/*!
 * @brief Print a signed value represented in thousandths of its unit.
 */
static void
print_value (char const * p_axis, int32_t milli, char const * p_units)
{
    int32_t magnitude = ((milli < 0) ? -milli : milli);

    printf("%s: %s%ld.%03ld %s\r\n", p_axis,
           ((milli < 0) ? "-" : "+"), (long) (magnitude / MILLI_PER_UNIT),
           (long) (magnitude % MILLI_PER_UNIT), p_units);
}   /* print_value() */


/*!
 * @brief Convert magnetometer counts to milligauss and print safely.
 */
static void
print_mag_value (char const * p_axis, int16_t raw, int32_t sensitivity)
{
    if (((int32_t) raw < MAG_RAW_MIN) || ((int32_t) raw > MAG_RAW_MAX))
    {
        printf("%s: OUT OF RANGE (raw %ld)\r\n", p_axis, (long) raw);
    }
    else
    {
        int32_t milligauss = ((int32_t) raw * MILLI_PER_UNIT) / sensitivity;
        print_value(p_axis, milligauss, "gauss");
    }
}   /* print_mag_value() */


/*!
 * @brief Emit one Teleplot sample (">name:value") from thousandths of a unit.
 */
static void
teleplot_value (char const * p_name, int32_t milli)
{
    int32_t magnitude = ((milli < 0) ? -milli : milli);

    printf(">%s:%s%ld.%03ld\r\n", p_name, ((milli < 0) ? "-" : ""),
           (long) (magnitude / MILLI_PER_UNIT),
           (long) (magnitude % MILLI_PER_UNIT));
}   /* teleplot_value() */


/*!
 * @brief Emit a magnetometer Teleplot sample in gauss, skipping overflow.
 */
static void
teleplot_mag (char const * p_name, int16_t raw, int32_t sensitivity)
{
    if (((int32_t) raw >= MAG_RAW_MIN) && ((int32_t) raw <= MAG_RAW_MAX))
    {
        teleplot_value(p_name, ((int32_t) raw * MILLI_PER_UNIT) /
                               sensitivity);
    }
}   /* teleplot_mag() */


/*!
 * @brief Write a byte to one sensor core.
 */
static bool
reg_write (uint8_t address, uint8_t reg, uint8_t value)
{
    uint8_t frame[2] = { reg, value };
    int     written  = i2c_write_timeout_us(TEST_I2C, address, frame,
                                            sizeof(frame), false,
                                            I2C_TIMEOUT_USEC);

    return ((int) sizeof(frame) == written);
}   /* reg_write() */


/*!
 * @brief Read consecutive registers from one sensor core.
 */
static bool
reg_read (uint8_t address, uint8_t reg, uint8_t * p_data, size_t length)
{
    int result = i2c_write_timeout_us(TEST_I2C, address, &reg, 1u, true,
                                      I2C_TIMEOUT_USEC);

    if (1 == result)
    {
        result = i2c_read_timeout_us(TEST_I2C, address, p_data, length, false,
                                     I2C_TIMEOUT_USEC);
    }

    return ((int) length == result);
}   /* reg_read() */

/*** end of file ***/
