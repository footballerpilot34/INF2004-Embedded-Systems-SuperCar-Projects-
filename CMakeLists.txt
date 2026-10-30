# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.13)

# Which Pico is on your Robo Pico?  Pico W -> pico_w   |   Pico 2 W -> pico2_w
set(PICO_BOARD pico_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(robocar C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

add_executable(robocar
    robocar.c
    dhcpserver/dhcpserver.c
)

target_include_directories(robocar PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}             # lwipopts.h
    ${CMAKE_CURRENT_LIST_DIR}/dhcpserver
)

target_link_libraries(robocar
    pico_stdlib
    hardware_pwm
    pico_cyw43_arch_lwip_poll             # Wi-Fi chip + network stack
)

pico_enable_stdio_usb(robocar 1)          # printf() over USB cable (handy for debugging)
pico_enable_stdio_uart(robocar 0)

pico_add_extra_outputs(robocar)           # makes robocar.uf2

# Bench test: line IR sensors + IMU gyroscope connection check.
add_executable(sensor_test
    tests/sensor_test.c
)

target_link_libraries(sensor_test
    pico_stdlib
    hardware_i2c
)

pico_enable_stdio_usb(sensor_test 1)
pico_enable_stdio_uart(sensor_test 0)

pico_add_extra_outputs(sensor_test)       # makes sensor_test.uf2

# Bench test: gyroscope I2C bus check, scan and WHO_AM_I read.
add_executable(i2cscanner
    tests/i2cscanner.c
)

target_link_libraries(i2cscanner
    pico_stdlib
    hardware_i2c
)

pico_enable_stdio_usb(i2cscanner 1)
pico_enable_stdio_uart(i2cscanner 0)

pico_add_extra_outputs(i2cscanner)        # makes i2cscanner.uf2

# Bench test: MPU-6050 gyroscope X/Y/Z rates, angles and acceleration.
add_executable(gyro_test
    tests/gyro_test.c
)

target_link_libraries(gyro_test
    pico_stdlib
    hardware_i2c
)

pico_enable_stdio_usb(gyro_test 1)
pico_enable_stdio_uart(gyro_test 0)

pico_add_extra_outputs(gyro_test)         # makes gyro_test.uf2

# Standalone LSM303DLHC accelerometer and magnetometer serial test.
add_executable(lsm303_test
    tests/lsm303_test.c
)

target_include_directories(lsm303_test PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
)

target_link_libraries(lsm303_test
    pico_stdlib
    hardware_i2c
)

pico_enable_stdio_usb(lsm303_test 1)
pico_enable_stdio_uart(lsm303_test 0)

pico_add_extra_outputs(lsm303_test)

# Final firmware: line following, Code 39 barcode, hump vs wall,
# Teleplot telemetry and a wireless web UI.
add_executable(supercar
    src/supercar_main.c
    src/barcode.c
    src/buzzer.c
    src/gpio_isr.c
    src/imu.c
    src/line_diag.c
    src/line_sensor.c
    src/log.c
    src/motor.c
    src/telemetry.c
    src/ultrasonic.c
    src/vehicle.c
    src/web.c
    dhcpserver/dhcpserver.c
)

target_include_directories(supercar PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
    ${CMAKE_CURRENT_LIST_DIR}             # lwipopts.h
    ${CMAKE_CURRENT_LIST_DIR}/dhcpserver
)

target_link_libraries(supercar
    pico_stdlib
    hardware_i2c
    hardware_pwm
    pico_cyw43_arch_lwip_poll             # Wi-Fi hotspot + web UI
)

pico_enable_stdio_usb(supercar 1)
pico_enable_stdio_uart(supercar 0)

pico_add_extra_outputs(supercar)          # makes supercar.uf2
