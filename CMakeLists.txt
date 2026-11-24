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

# The board: Pico W (pico_w). Pico 2 W (pico2_w) is NOT supported: the
# firmware reads RP2040-only reset registers (vreg_and_chip_reset in
# app_main.c), and it has never been built or run on a Pico 2 W.
set(PICO_BOARD pico_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(supercar C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

# Final firmware: line following, Code 39 barcode, hump vs wall,
# Teleplot telemetry and a wireless web UI.
add_executable(supercar
    app_program/app_main.c
    app_program/line_barcode/barcode.c
    app_program/obstacle/bypass.c
    app_program/motion/encoder.c
    app_program/buzzer.c
    app_program/gpio_isr.c
    app_program/imu_hump/hump.c
    app_program/imu_hump/imu.c
    app_program/line_barcode/line_diag.c
    app_program/line_barcode/line_sensor.c
    app_program/comms/log.c
    app_program/motion/motor.c
    app_program/comms/mqtt_comm.c
    app_program/power.c
    app_program/obstacle/servo.c
    app_program/obstacle/obstacle.c
    app_program/comms/telemetry.c
    app_program/obstacle/ultrasonic.c
    app_program/vehicle.c
    app_program/comms/web.c
    app_program/bitpack.c
    app_program/fault.c
    app_program/comms/mqtt_frame.c
    lib/libwifi/dhcpserver/dhcpserver.c
)

target_include_directories(supercar PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/config      # config.h, lwipopts.h, wifi_secrets.h
    ${CMAKE_CURRENT_LIST_DIR}/app_program
    ${CMAKE_CURRENT_LIST_DIR}/app_program/line_barcode
    ${CMAKE_CURRENT_LIST_DIR}/app_program/motion
    ${CMAKE_CURRENT_LIST_DIR}/app_program/obstacle
    ${CMAKE_CURRENT_LIST_DIR}/app_program/comms
    ${CMAKE_CURRENT_LIST_DIR}/app_program/imu_hump
    ${CMAKE_CURRENT_LIST_DIR}/lib/libwifi/dhcpserver
)

target_link_libraries(supercar
    pico_stdlib
    hardware_adc                          # Barcode IR analog output
    hardware_i2c
    hardware_pwm
    hardware_watchdog                     # Restart reason
    m
    pico_cyw43_arch_lwip_poll             # Wi-Fi hotspot + web UI
    pico_lwip_mdns                        # http://supercar-aai14.local
    pico_lwip_mqtt                        # MQTT telemetry and commands
)

# Enforce the portable layout checks without touching SDK or vendor sources.
find_package(Python3 COMPONENTS Interpreter REQUIRED)
add_custom_target(supercar_style_check
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_LIST_DIR}/tools/check_barr.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
    COMMENT "Checking project-owned C layout"
)
add_dependencies(supercar supercar_style_check)

# Name shown in a phone hotspot's device list (= WIFI_HOST_NAME).
target_compile_definitions(supercar PRIVATE
    CYW43_HOST_NAME="supercar-aai14"
    SUPERCAR_RECORD_ASSERTS=1             # lwIP failures are reported
    # A connected serial monitor that stops reading must not starve Wi-Fi polling.
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000
)

pico_enable_stdio_usb(supercar 1)
pico_enable_stdio_uart(supercar 0)

pico_add_extra_outputs(supercar)          # makes supercar.uf2
