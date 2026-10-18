# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

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
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(PicoPilot C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

option(PICOPILOT_ENABLE_DIAGNOSTICS "Enable periodic USB sensor diagnostics" ON)

# Add executable. Default name is the project name, version 0.1

add_executable(PicoPilot PicoPilot.cpp )
if(PICOPILOT_ENABLE_DIAGNOSTICS)
    target_compile_definitions(PicoPilot PRIVATE PICOPILOT_ENABLE_DIAGNOSTICS=1)
endif()

pico_set_program_name(PicoPilot "PicoPilot")
pico_set_program_version(PicoPilot "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(PicoPilot 0)
pico_enable_stdio_usb(PicoPilot ${PICOPILOT_ENABLE_DIAGNOSTICS})

# Add the standard library to the build
target_link_libraries(PicoPilot
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(PicoPilot PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

add_subdirectory(include)

target_link_libraries(PicoPilot
        hardware_sync
        hardware_timer
        hardware_gpio
        hardware_uart
        hardware_i2c
        hardware_pwm
        hardware_watchdog
        hardware_clocks
        pico_multicore
        hardware_dma

        picopilot_acs712
        picopilot_bme280
        picopilot_mpu6500
        picopilot_neo6m
        picopilot_sensor_fusion
        picopilot_control
        picopilot_routing
    )

pico_add_extra_outputs(PicoPilot)
