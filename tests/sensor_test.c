/** @file sensor_test.c
 *
 * @brief Bench test that checks the left and right line IR sensors and the
 *        IMU gyroscope are connected and responding.
 *
 * @par What it does
 * - Every 200 ms it prints each line IR sensor's level, or "NOT CONNECTED"
 *   if the pin is floating (nothing is driving it).
 * - At start-up it scans the I2C1 bus (SDA GP2, SCL GP3), prints every
 *   address that answers, and identifies the IMU from its ID register.
 * - If the IMU is a supported gyroscope, it prints live rotation rates in
 *   degrees per second. If no gyroscope is found, it rescans every 3 s, so
 *   wiring can be fixed while the test runs.
 *
 * @par How the IR connection check works
 * Each IR module drives its DOUT pin. The test reads the pin once with the
 * internal pull-up on and once with the pull-down on. A driven pin reads the
 * same both times; a floating pin follows the pull, so it reads differently.
 *
 * @par Pins (updated pin assignment)
 * LINE_L on GP28 (Grove 7), LINE_R on GP0 (Grove 1), and the IMU on I2C1
 * with SDA GP2 and SCL GP3.
 *
 * @par Coding standard
 * BARR-C:2018. Deviations: Rule 4.1.d (this test's main() lives in a file
 * without "main" in its name) and Rule 5.2.a (Pico SDK functions return
 * int status codes).
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


// Line IR sensor pins.
//
#define LINE_L_PIN              28u
#define LINE_R_PIN              0u
#define PULL_SETTLE_USEC        50u

// IMU bus settings. 100 kHz is the safest speed for long jumper wires.
//
#define IMU_I2C                 i2c1
#define IMU_SDA_PIN             2u
#define IMU_SCL_PIN             3u
#define IMU_I2C_HZ              100000u
#define I2C_TIMEOUT_USEC        5000u
#define I2C_FIRST_ADDRESS       0x08u
#define I2C_LAST_ADDRESS        0x77u

// MPU-6050 family (InvenSense) registers.
//
#define MPU_ADDRESS_LOW         0x68u
#define MPU_ADDRESS_HIGH        0x69u
#define MPU_REG_WHO_AM_I        0x75u
#define MPU_REG_PWR_MGMT_1      0x6Bu
#define MPU_REG_GYRO_XOUT_H     0x43u
#define MPU_WAKE_PLL_X          0x01u   // Leave sleep; clock from gyro X.

// Bosch chips at the same addresses, identified but not streamed.
//
#define BOSCH_REG_CHIP_ID       0x00u
#define BMI160_CHIP_ID          0xD1u
#define ICM20948_CHIP_ID        0xEAu

// ST LSM6DS and L3GD20 family registers.
//
#define ST_ADDRESS_LOW          0x6Au
#define ST_ADDRESS_HIGH         0x6Bu
#define ST_REG_WHO_AM_I         0x0Fu
#define LSM6DS3_ID              0x69u
#define LSM6DSL_ID              0x6Au
#define LSM6DSO_ID              0x6Cu
#define L3GD20_ID               0xD4u
#define L3GD20H_ID              0xD7u
#define LSM6DS_REG_CTRL2_G      0x11u
#define LSM6DS_REG_OUTX_L_G     0x22u
#define LSM6DS_GYRO_104HZ_250   0x40u
#define L3GD20_REG_CTRL1        0x20u
#define L3GD20_REG_OUT_X_L      0x28u
#define L3GD20_POWER_ON_XYZ     0x0Fu
#define L3GD20_AUTO_INCREMENT   0x80u

// Accelerometer and magnetometer only (no gyroscope) addresses.
//
#define LSM303_ACCEL_ADDRESS    0x19u
#define MAGNETOMETER_ADDRESS    0x1Eu

// Gyroscope scaling, in tenths of a degree per second.
// MPU: 131 LSB per dps at +/-250 dps. ST: 8.75 mdps per LSB at 245/250 dps.
//
#define NUM_AXES                3u
#define AXIS_BYTES              2u
#define MPU_LSB_PER_DPS         131
#define ST_MDPS_X100_PER_LSB    875
#define ST_SCALE_DIVISOR        10000
#define TENTHS_PER_UNIT         10
#define TENTHS_SCALE            10

// Timing.
//
#define USB_WAIT_MSEC           10000u
#define USB_POLL_MSEC           100u
#define REPORT_PERIOD_MSEC      200u
#define RESCAN_PERIOD_MSEC      3000u

typedef enum
{
    GYRO_NONE,
    GYRO_MPU,
    GYRO_LSM6DS,
    GYRO_L3GD20
} gyro_family_t;

typedef struct
{
    gyro_family_t family;
    uint8_t       address;
} gyro_t;


static void   usb_wait(void);
static void   ir_init(uint32_t pin);
static bool   ir_probe(uint32_t pin, bool * p_b_level);
static void   ir_report(char const * p_name, uint32_t pin);
static void   imu_bus_init(void);
static bool   i2c_reg_read(uint8_t address, uint8_t reg, uint8_t * p_data,
                           size_t length);
static bool   i2c_reg_write(uint8_t address, uint8_t reg, uint8_t value);
static bool   i2c_device_present(uint8_t address);
static void   i2c_lines_check(void);
static void   i2c_bus_scan(void);
static bool   gyro_identify_mpu(uint8_t address, gyro_t * p_gyro);
static bool   gyro_identify_st(uint8_t address, gyro_t * p_gyro);
static bool   gyro_connect(gyro_t * p_gyro);
static bool   gyro_start(gyro_t const * p_gyro);
static bool   gyro_read_dps_x10(gyro_t const * p_gyro, int32_t * p_rates);
static void   dps_print(char axis, int32_t tenths);


/*!
 * @brief Run the IR and gyroscope connection test forever.
 *
 * @return Never returns.
 */
int
main (void)
{
    gyro_t   gyro        = { GYRO_NONE, 0u };
    bool     b_gyro_ok   = false;
    uint32_t elapsed_ms  = RESCAN_PERIOD_MSEC;
    int32_t  rates[NUM_AXES] = { 0 };

    stdio_init_all();
    usb_wait();

    printf("\n=== IR + gyroscope connection test ===\n");
    ir_init(LINE_L_PIN);
    ir_init(LINE_R_PIN);
    imu_bus_init();

    for (;;)
    {
        // Retry the gyroscope until it is found and running.
        if ((!b_gyro_ok) && (elapsed_ms >= RESCAN_PERIOD_MSEC))
        {
            b_gyro_ok  = gyro_connect(&gyro);
            elapsed_ms = 0u;
        }

        ir_report("IR L (GP28)", LINE_L_PIN);
        ir_report(" | IR R (GP0)", LINE_R_PIN);

        if (!b_gyro_ok)
        {
            printf(" | Gyro: not found\n");
        }
        else if (!gyro_read_dps_x10(&gyro, rates))
        {
            printf(" | Gyro: read FAILED (connection lost)\n");
            b_gyro_ok = false;
        }
        else
        {
            printf(" | Gyro dps");
            dps_print('x', rates[0]);
            dps_print('y', rates[1]);
            dps_print('z', rates[2]);
            printf("\n");
        }

        sleep_ms(REPORT_PERIOD_MSEC);
        elapsed_ms += REPORT_PERIOD_MSEC;
    }

    return (0);
}   /* main() */


/*!
 * @brief Wait up to 10 s for a serial monitor, so the start-up scan is seen.
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


/*!
 * @brief Set up one IR sensor pin as a plain input.
 *
 * @param[in] pin The GPIO the sensor's DOUT is wired to.
 */
static void
ir_init (uint32_t pin)
{
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_disable_pulls(pin);
}   /* ir_init() */


/*!
 * @brief Check whether an IR sensor is driving its pin, and read its level.
 *
 * @param[in]  pin       The GPIO to check.
 * @param[out] p_b_level The pin's level, valid only if the sensor is
 *                       connected.
 *
 * @return True if something is driving the pin; false if it is floating.
 */
static bool
ir_probe (uint32_t pin, bool * p_b_level)
{
    bool b_with_pull_up   = false;
    bool b_with_pull_down = false;

    gpio_pull_up(pin);
    sleep_us(PULL_SETTLE_USEC);
    b_with_pull_up = gpio_get(pin);

    gpio_pull_down(pin);
    sleep_us(PULL_SETTLE_USEC);
    b_with_pull_down = gpio_get(pin);

    gpio_disable_pulls(pin);
    *p_b_level = b_with_pull_up;

    return (b_with_pull_up == b_with_pull_down);
}   /* ir_probe() */


/*!
 * @brief Print one IR sensor's state, without a newline.
 *
 * @param[in] p_name The label to print.
 * @param[in] pin    The GPIO the sensor is wired to.
 */
static void
ir_report (char const * p_name, uint32_t pin)
{
    bool b_level = false;

    if (!ir_probe(pin, &b_level))
    {
        printf("%s: NOT CONNECTED", p_name);
    }
    else
    {
        printf("%s: %s", p_name, (b_level ? "HIGH" : "LOW "));
    }
}   /* ir_report() */


/*!
 * @brief Set up I2C1 on GP2 (SDA) and GP3 (SCL) for the IMU.
 */
static void
imu_bus_init (void)
{
    i2c_init(IMU_I2C, IMU_I2C_HZ);
    gpio_set_function(IMU_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(IMU_SCL_PIN, GPIO_FUNC_I2C);

    // NOTE: The internal pull-ups are weak; most IMU breakout boards add
    // their own, which is what makes the bus reliable.
    gpio_pull_up(IMU_SDA_PIN);
    gpio_pull_up(IMU_SCL_PIN);
}   /* imu_bus_init() */


/*!
 * @brief Read one or more consecutive registers from an I2C device.
 *
 * @param[in]  address The 7-bit device address.
 * @param[in]  reg     The first register to read.
 * @param[out] p_data  Where to store the bytes read.
 * @param[in]  length  The number of bytes to read.
 *
 * @return True if the device acknowledged and every byte was read.
 */
static bool
i2c_reg_read (uint8_t address, uint8_t reg, uint8_t * p_data, size_t length)
{
    bool b_ok    = false;
    int  written = i2c_write_timeout_us(IMU_I2C, address, &reg, 1u, true,
                                        I2C_TIMEOUT_USEC);

    if (1 == written)
    {
        int read = i2c_read_timeout_us(IMU_I2C, address, p_data, length,
                                       false, I2C_TIMEOUT_USEC);

        // NOTE: length is at most 6 here, so it fits an int.
        b_ok = ((int) length == read);
    }

    return (b_ok);
}   /* i2c_reg_read() */


/*!
 * @brief Write one register on an I2C device.
 *
 * @param[in] address The 7-bit device address.
 * @param[in] reg     The register to write.
 * @param[in] value   The value to write.
 *
 * @return True if the device acknowledged both bytes.
 */
static bool
i2c_reg_write (uint8_t address, uint8_t reg, uint8_t value)
{
    uint8_t frame[2] = { reg, value };
    int     written  = i2c_write_timeout_us(IMU_I2C, address, frame,
                                            sizeof(frame), false,
                                            I2C_TIMEOUT_USEC);

    return ((int) sizeof(frame) == written);
}   /* i2c_reg_write() */


/*!
 * @brief Check whether a device acknowledges an address.
 *
 * @param[in] address The 7-bit address to try.
 *
 * @return True if a device answered.
 */
static bool
i2c_device_present (uint8_t address)
{
    uint8_t dummy = 0u;
    int     read  = i2c_read_timeout_us(IMU_I2C, address, &dummy, 1u, false,
                                        I2C_TIMEOUT_USEC);

    return (1 == read);
}   /* i2c_device_present() */


/*!
 * @brief Read the idle level of SDA and SCL and explain what it means.
 *
 * @par
 * An idle I2C bus sits HIGH. A line held LOW usually means the IMU has no
 * power (its own pull-ups then drag the line to 0 V) or the wire is
 * shorted to GND.
 */
static void
i2c_lines_check (void)
{
    bool b_sda_high = false;
    bool b_scl_high = false;

    // Briefly hand both pins back to plain GPIO inputs with pull-ups.
    gpio_set_function(IMU_SDA_PIN, GPIO_FUNC_SIO);
    gpio_set_function(IMU_SCL_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(IMU_SDA_PIN, GPIO_IN);
    gpio_set_dir(IMU_SCL_PIN, GPIO_IN);
    sleep_us(PULL_SETTLE_USEC);

    b_sda_high = gpio_get(IMU_SDA_PIN);
    b_scl_high = gpio_get(IMU_SCL_PIN);

    gpio_set_function(IMU_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(IMU_SCL_PIN, GPIO_FUNC_I2C);

    printf("I2C idle lines: SDA(GP2)=%s SCL(GP3)=%s",
           (b_sda_high ? "HIGH" : "LOW"), (b_scl_high ? "HIGH" : "LOW"));

    if (b_sda_high && b_scl_high)
    {
        printf(" (OK)\n");
    }
    else
    {
        printf(" <- a LOW line means the IMU has no power or the wire is "
               "shorted to GND.\n");
    }
}   /* i2c_lines_check() */


/*!
 * @brief Print every address on the IMU bus that answers.
 */
static void
i2c_bus_scan (void)
{
    uint32_t num_found = 0u;

    i2c_lines_check();
    printf("I2C1 scan (SDA GP2, SCL GP3):");

    for (uint8_t address = I2C_FIRST_ADDRESS; address <= I2C_LAST_ADDRESS;
         address++)
    {
        if (i2c_device_present(address))
        {
            printf(" 0x%02X", address);
            num_found++;
        }
    }

    if (0u == num_found)
    {
        printf(" nothing found. Check SDA/SCL wiring, 3V3 and GND.\n");
    }
    else
    {
        printf("\n");
    }

    if (i2c_device_present(LSM303_ACCEL_ADDRESS) ||
        i2c_device_present(MAGNETOMETER_ADDRESS))
    {
        printf("Note: 0x%02X/0x%02X is an accelerometer or magnetometer "
               "(e.g. LSM303); it has no gyroscope.\n",
               LSM303_ACCEL_ADDRESS, MAGNETOMETER_ADDRESS);
    }
}   /* i2c_bus_scan() */


/*!
 * @brief Identify an InvenSense or Bosch IMU at 0x68 or 0x69.
 *
 * @param[in]  address The address to check.
 * @param[out] p_gyro  Filled in if a streamable gyroscope is found.
 *
 * @return True if the device can be streamed as an MPU-family gyroscope.
 */
static bool
gyro_identify_mpu (uint8_t address, gyro_t * p_gyro)
{
    uint8_t who_am_i = 0u;
    uint8_t chip_id  = 0u;
    bool    b_found  = false;

    if (i2c_reg_read(address, BOSCH_REG_CHIP_ID, &chip_id, 1u) &&
        ((BMI160_CHIP_ID == chip_id) || (ICM20948_CHIP_ID == chip_id)))
    {
        printf("Found %s at 0x%02X (ID 0x%02X). Identified only; this "
               "test does not stream it.\n",
               ((BMI160_CHIP_ID == chip_id) ? "BMI160" : "ICM-20948"),
               address, chip_id);
    }
    else if (i2c_reg_read(address, MPU_REG_WHO_AM_I, &who_am_i, 1u))
    {
        // NOTE: Clone MPU-6050 boards often report other WHO_AM_I values
        // but use the same registers, so any answer is tried.
        printf("Found MPU-family IMU at 0x%02X (WHO_AM_I 0x%02X; 0x68 is "
               "a genuine MPU-6050, 0x70 MPU-6500, 0x71 MPU-9250).\n",
               address, who_am_i);
        p_gyro->family  = GYRO_MPU;
        p_gyro->address = address;
        b_found         = true;
    }
    else
    {
        printf("Device at 0x%02X did not answer its ID register.\n",
               address);
    }

    return (b_found);
}   /* gyro_identify_mpu() */


/*!
 * @brief Identify an ST LSM6DS or L3GD20 gyroscope at 0x6A or 0x6B.
 *
 * @param[in]  address The address to check.
 * @param[out] p_gyro  Filled in if a supported gyroscope is found.
 *
 * @return True if the device is a supported ST gyroscope.
 */
static bool
gyro_identify_st (uint8_t address, gyro_t * p_gyro)
{
    uint8_t who_am_i = 0u;
    bool    b_found  = false;

    if (!i2c_reg_read(address, ST_REG_WHO_AM_I, &who_am_i, 1u))
    {
        printf("Device at 0x%02X did not answer its ID register.\n",
               address);
    }
    else if ((LSM6DS3_ID == who_am_i) || (LSM6DSL_ID == who_am_i) ||
             (LSM6DSO_ID == who_am_i))
    {
        printf("Found ST LSM6DS-family IMU at 0x%02X (WHO_AM_I 0x%02X).\n",
               address, who_am_i);
        p_gyro->family  = GYRO_LSM6DS;
        p_gyro->address = address;
        b_found         = true;
    }
    else if ((L3GD20_ID == who_am_i) || (L3GD20H_ID == who_am_i))
    {
        printf("Found ST L3GD20 gyroscope at 0x%02X (WHO_AM_I 0x%02X).\n",
               address, who_am_i);
        p_gyro->family  = GYRO_L3GD20;
        p_gyro->address = address;
        b_found         = true;
    }
    else
    {
        printf("Unknown ST device at 0x%02X (WHO_AM_I 0x%02X).\n",
               address, who_am_i);
    }

    return (b_found);
}   /* gyro_identify_st() */


/*!
 * @brief Scan the bus, identify a gyroscope and start it.
 *
 * @param[out] p_gyro The gyroscope found, if any.
 *
 * @return True if a gyroscope was found and started.
 */
static bool
gyro_connect (gyro_t * p_gyro)
{
    static uint8_t const mpu_addresses[] =
    {
        MPU_ADDRESS_LOW, MPU_ADDRESS_HIGH
    };
    static uint8_t const st_addresses[] =
    {
        ST_ADDRESS_LOW, ST_ADDRESS_HIGH
    };
    bool b_found   = false;
    bool b_started = false;

    p_gyro->family = GYRO_NONE;
    i2c_bus_scan();

    for (uint32_t idx = 0u; (idx < sizeof(mpu_addresses)) && (!b_found);
         idx++)
    {
        if (i2c_device_present(mpu_addresses[idx]))
        {
            b_found = gyro_identify_mpu(mpu_addresses[idx], p_gyro);
        }
    }

    for (uint32_t idx = 0u; (idx < sizeof(st_addresses)) && (!b_found);
         idx++)
    {
        if (i2c_device_present(st_addresses[idx]))
        {
            b_found = gyro_identify_st(st_addresses[idx], p_gyro);
        }
    }

    if (!b_found)
    {
        printf("No supported gyroscope found; retrying in 3 s.\n");
    }
    else if (!gyro_start(p_gyro))
    {
        printf("Gyroscope found but would not start; retrying in 3 s.\n");
    }
    else
    {
        printf("Gyroscope running. Rotate the car to see the rates move.\n");
        b_started = true;
    }

    return (b_started);
}   /* gyro_connect() */


/*!
 * @brief Wake the gyroscope and set it to +/-250 dps.
 *
 * @param[in] p_gyro The gyroscope to start.
 *
 * @return True if the configuration was written.
 */
static bool
gyro_start (gyro_t const * p_gyro)
{
    bool b_ok = false;

    switch (p_gyro->family)
    {
    case GYRO_MPU:
        b_ok = i2c_reg_write(p_gyro->address, MPU_REG_PWR_MGMT_1,
                             MPU_WAKE_PLL_X);
    break;

    case GYRO_LSM6DS:
        b_ok = i2c_reg_write(p_gyro->address, LSM6DS_REG_CTRL2_G,
                             LSM6DS_GYRO_104HZ_250);
    break;

    case GYRO_L3GD20:
        b_ok = i2c_reg_write(p_gyro->address, L3GD20_REG_CTRL1,
                             L3GD20_POWER_ON_XYZ);
    break;

    default:
        b_ok = false;
    break;
    }

    return (b_ok);
}   /* gyro_start() */


/*!
 * @brief Read all three rotation rates in tenths of a degree per second.
 *
 * @param[in]  p_gyro  The running gyroscope.
 * @param[out] p_rates Three rates: x, y and z.
 *
 * @return True if the read succeeded.
 */
static bool
gyro_read_dps_x10 (gyro_t const * p_gyro, int32_t * p_rates)
{
    uint8_t raw[NUM_AXES * AXIS_BYTES] = { 0u };
    bool    b_ok = false;

    if (GYRO_MPU == p_gyro->family)
    {
        b_ok = i2c_reg_read(p_gyro->address, MPU_REG_GYRO_XOUT_H, raw,
                            sizeof(raw));
    }
    else if (GYRO_LSM6DS == p_gyro->family)
    {
        b_ok = i2c_reg_read(p_gyro->address, LSM6DS_REG_OUTX_L_G, raw,
                            sizeof(raw));
    }
    else
    {
        b_ok = i2c_reg_read(p_gyro->address,
                            (L3GD20_REG_OUT_X_L | L3GD20_AUTO_INCREMENT),
                            raw, sizeof(raw));
    }

    for (uint32_t axis = 0u; (axis < NUM_AXES) && b_ok; axis++)
    {
        uint32_t lo_idx = axis * AXIS_BYTES;
        int16_t  counts = 0;

        // NOTE: Each axis is a two's-complement 16-bit value; MPU parts send
        // the high byte first and ST parts send the low byte first. The
        // casts reassemble the 16-bit pattern without sign errors.
        if (GYRO_MPU == p_gyro->family)
        {
            counts = (int16_t) (((uint16_t) raw[lo_idx] << 8) |
                                raw[lo_idx + 1u]);
            p_rates[axis] = ((int32_t) counts * TENTHS_SCALE) /
                            MPU_LSB_PER_DPS;
        }
        else
        {
            counts = (int16_t) (((uint16_t) raw[lo_idx + 1u] << 8) |
                                raw[lo_idx]);
            p_rates[axis] = ((int32_t) counts * ST_MDPS_X100_PER_LSB) /
                            ST_SCALE_DIVISOR;
        }
    }

    return (b_ok);
}   /* gyro_read_dps_x10() */


/*!
 * @brief Print one axis rate, such as " x=-12.3".
 *
 * @param[in] axis   The axis letter.
 * @param[in] tenths The rate in tenths of a degree per second.
 */
static void
dps_print (char axis, int32_t tenths)
{
    char    sign      = ((tenths < 0) ? '-' : '+');
    int32_t magnitude = ((tenths < 0) ? -tenths : tenths);

    printf(" %c=%c%" PRId32 ".%" PRId32, axis, sign,
           magnitude / TENTHS_PER_UNIT, magnitude % TENTHS_PER_UNIT);
}   /* dps_print() */

/*** end of file ***/
