/** @file i2cscanner.c
 *
 * @brief Gyroscope I2C connection test: checks the bus lines, scans every
 *        address, and reads the MPU-6050 WHO_AM_I register.
 *
 * @par
 * The IMU is on I2C1 with SDA on GP2 and SCL on GP3. The test repeats every
 * 2 s, so the result is shown whenever the serial monitor is opened and
 * wiring can be fixed while it runs.
 *
 * @par Coding standard
 * BARR-C:2018. Deviations: Rule 4.1.d (main() lives in a file without
 * "main" in its name) and Rule 5.2.a (Pico SDK I2C calls return int).
 *
 * TODO: Record the name of the person who approved these deviations.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"


// IMU bus wiring and speed.
//
#define I2C_PORT            i2c1
#define SDA_PIN             2u
#define SCL_PIN             3u
#define I2C_HZ              100000u
#define I2C_TIMEOUT_USEC    5000u

// Valid 7-bit addresses (0x00-0x07 and 0x78-0x7F are reserved).
//
#define I2C_FIRST_ADDRESS   0x08u
#define I2C_LAST_ADDRESS    0x77u

// MPU-6050 addresses and identity register.
//
#define MPU_ADDRESS_LOW     0x68u   // AD0 pin low or unconnected.
#define MPU_ADDRESS_HIGH    0x69u   // AD0 pin high.
#define WHO_AM_I_REG        0x75u
#define MPU6050_IDENTITY    0x68u

// Timing.
//
#define LINE_SETTLE_USEC    50u
#define USB_WAIT_MSEC       10000u
#define USB_POLL_MSEC       100u
#define REPEAT_PERIOD_MSEC  2000u


static void bus_init(void);
static bool bus_lines_check(void);
static int  bus_scan(void);
static bool mpu_identify(uint8_t address);
static void usb_wait(void);


/*!
 * @brief Run the gyroscope connection test every 2 s, forever.
 *
 * @return Never returns.
 */
int
main (void)
{
    stdio_init_all();
    usb_wait();
    bus_init();

    for (;;)
    {
        bool b_found = false;

        printf("\n=== Gyroscope I2C connection test (SDA GP2, SCL GP3) ===\n");

        if (!bus_lines_check())
        {
            printf("A LOW line means the IMU has no power, or the wire is "
                   "shorted to GND. Skipping the scan.\n");
        }
        else if (0 == bus_scan())
        {
            printf("Lines are HIGH but nothing answered: SDA and SCL are "
                   "probably swapped.\n");
        }
        else
        {
            b_found = mpu_identify(MPU_ADDRESS_LOW);
            b_found = (mpu_identify(MPU_ADDRESS_HIGH) || b_found);

            if (!b_found)
            {
                printf("Devices answered, but not at 0x68 or 0x69, so this "
                       "is not an MPU-6050.\n");
            }
        }

        sleep_ms(REPEAT_PERIOD_MSEC);
    }

    return (0);
}   /* main() */


/*!
 * @brief Set up I2C1 on GP2 (SDA) and GP3 (SCL) at 100 kHz.
 */
static void
bus_init (void)
{
    i2c_init(I2C_PORT, I2C_HZ);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

    // NOTE: The internal pull-ups are weak; most IMU breakout boards add
    // their own.
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
}   /* bus_init() */


/*!
 * @brief Read the idle level of SDA and SCL. An idle I2C bus sits HIGH.
 *
 * @return True if both lines are HIGH.
 */
static bool
bus_lines_check (void)
{
    bool b_sda_high = false;
    bool b_scl_high = false;

    // Briefly hand both pins back to plain GPIO inputs (pull-ups stay on).
    gpio_set_function(SDA_PIN, GPIO_FUNC_SIO);
    gpio_set_function(SCL_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(SDA_PIN, GPIO_IN);
    gpio_set_dir(SCL_PIN, GPIO_IN);
    sleep_us(LINE_SETTLE_USEC);

    b_sda_high = gpio_get(SDA_PIN);
    b_scl_high = gpio_get(SCL_PIN);

    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

    printf("Idle lines: SDA=%s SCL=%s\n", (b_sda_high ? "HIGH" : "LOW"),
           (b_scl_high ? "HIGH" : "LOW"));

    return (b_sda_high && b_scl_high);
}   /* bus_lines_check() */


/*!
 * @brief Print every address on the bus that answers.
 *
 * @return The number of devices found.
 */
static int
bus_scan (void)
{
    int num_found = 0;

    printf("Devices found:");

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

    printf((0 == num_found) ? " none\n" : "\n");

    return (num_found);
}   /* bus_scan() */


/*!
 * @brief Read and print the WHO_AM_I register at one MPU-6050 address.
 *
 * @param[in] address The address to try (0x68 or 0x69).
 *
 * @return True if a device answered with its identity.
 */
static bool
mpu_identify (uint8_t address)
{
    uint8_t reg      = WHO_AM_I_REG;
    uint8_t identity = 0u;
    bool    b_found  = false;
    int     result   = i2c_write_timeout_us(I2C_PORT, address, &reg, 1u,
                                            true, I2C_TIMEOUT_USEC);

    if (1 == result)
    {
        result = i2c_read_timeout_us(I2C_PORT, address, &identity, 1u,
                                     false, I2C_TIMEOUT_USEC);
    }

    if (1 != result)
    {
        printf("No response at 0x%02X\n", address);
    }
    else if (MPU6050_IDENTITY == identity)
    {
        printf("0x%02X: WHO_AM_I = 0x%02X -> MPU-6050 detected!\n", address,
               identity);
        b_found = true;
    }
    else
    {
        // NOTE: Clone boards and related parts (0x70 MPU-6500, 0x71
        // MPU-9250, 0x98 ICM-20689) report other values.
        printf("0x%02X: WHO_AM_I = 0x%02X -> responding, but not the usual "
               "MPU-6050 value.\n", address, identity);
        b_found = true;
    }

    return (b_found);
}   /* mpu_identify() */


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
