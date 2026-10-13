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
