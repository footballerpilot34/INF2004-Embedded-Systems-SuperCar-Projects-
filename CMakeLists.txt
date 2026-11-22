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

project(supercar C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

# The car's firmware: line following, Code 39 barcodes, humps, obstacle
# bypass, MQTT telemetry and commands, and a wireless web UI.
add_executable(supercar
    src/supercar_main.c
    src/barcode.c
    src/bypass.c
    src/encoder.c
    src/buzzer.c
    src/gpio_isr.c
    src/hump.c
    src/imu.c
    src/line_diag.c
    src/line_sensor.c
    src/log.c
    src/motor.c
    src/mqtt_comm.c
    src/power.c
    src/servo.c
    src/obstacle.c
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
    hardware_adc                          # Barcode IR analog output
    hardware_i2c
    hardware_pwm
    hardware_watchdog                     # Restart reason
    m
    pico_cyw43_arch_lwip_poll             # Wi-Fi hotspot + web UI
    pico_lwip_mdns                        # http://supercar-aai14.local
    pico_lwip_mqtt                        # MQTT telemetry and commands
)

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
