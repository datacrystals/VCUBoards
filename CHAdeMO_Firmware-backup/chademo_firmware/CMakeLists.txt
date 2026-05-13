# CHAdeMO Controller — CMake Build Configuration
# Target: Raspberry Pi Pico (RP2040)
#
# Build instructions:
#   1. Set PICO_SDK_PATH environment variable to your Pico SDK
#   2. mkdir build && cd build
#   3. cmake .. -DCHADEMO_ROLE=VEHICLE   (or STATION)
#   4. make -j4
#   5. cp chademo_controller.uf2 /media/RPI-RP2/  (or use picotool)
#
# Role selection:
#   -DCHADEMO_ROLE=VEHICLE   → Build for EV (vehicle side)
#   -DCHADEMO_ROLE=STATION   → Build for EVSE (charging station side)

cmake_minimum_required(VERSION 3.13)

# Detect CHAdeMO role from CMake option
if(NOT DEFINED CHADEMO_ROLE)
    set(CHADEMO_ROLE "VEHICLE" CACHE STRING "CHAdeMO role: VEHICLE or STATION")
endif()

if(CHADEMO_ROLE STREQUAL "VEHICLE")
    add_compile_definitions(CHADEMO_ROLE_VEHICLE)
    set(PROJECT_SUFFIX "vehicle")
elseif(CHADEMO_ROLE STREQUAL "STATION")
    add_compile_definitions(CHADEMO_ROLE_STATION)
    set(PROJECT_SUFFIX "station")
else()
    message(FATAL_ERROR "CHADEMO_ROLE must be VEHICLE or STATION")
endif()

# Initialize Pico SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(chademo_controller_${PROJECT_SUFFIX} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# ---- Source files ----
set(SOURCES
    main.c
    chademo_hal.c
    chademo_can.c
    chademo_fsm.c
)

# ---- Executable ----
add_executable(${PROJECT_NAME}
    ${SOURCES}
)

# ---- Include directories ----
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# ---- Link libraries ----
target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    hardware_spi
    hardware_gpio
    hardware_timer
    hardware_watchdog
    hardware_uart
)

# ---- Compiler warnings (treat warnings as errors for safety) ----
target_compile_options(${PROJECT_NAME} PRIVATE
    -Wall
    -Wextra
    -Wshadow
    -Wcast-align
    -Wunused
    -Wnull-dereference
    -Wdouble-promotion
    -Wformat=2
    -Wstrict-overflow=3
    -fno-strict-aliasing
    -O2
)

# ---- Enable USB output for printf, disable UART stdio ----
# Comment out the next line and uncomment the following to use UART for printf
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)
# pico_enable_stdio_usb(${PROJECT_NAME} 0)
# pico_enable_stdio_uart(${PROJECT_NAME} 1)

# ---- Create UF2 and ELF files ----
pico_add_extra_outputs(${PROJECT_NAME})

# ============================================================================
# CAN Loopback Test Target (standalone — does not need CHAdeMO role logic)
# ============================================================================
add_executable(can_loopback_test
    can_test_main.c
    chademo_hal.c
    chademo_can.c
)

target_include_directories(can_loopback_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(can_loopback_test
    pico_stdlib
    hardware_spi
    hardware_gpio
    hardware_timer
    hardware_watchdog
    hardware_uart
)

target_compile_options(can_loopback_test PRIVATE
    -Wall
    -Wextra
    -Wshadow
    -Wcast-align
    -Wunused
    -Wnull-dereference
    -Wdouble-promotion
    -Wformat=2
    -Wstrict-overflow=3
    -fno-strict-aliasing
    -O2
)

# Force VEHICLE role so chademo_config.h is happy; test doesn't use role GPIOs
target_compile_definitions(can_loopback_test PRIVATE CHADEMO_ROLE_VEHICLE)

pico_enable_stdio_usb(can_loopback_test 1)
pico_enable_stdio_uart(can_loopback_test 0)
pico_add_extra_outputs(can_loopback_test)

# ---- Flash target ----
add_custom_target(flash
    COMMAND ${CMAKE_SOURCE_DIR}/flash.sh ${CMAKE_BINARY_DIR} ${CHADEMO_ROLE}
    COMMENT "Flashing ${PROJECT_NAME} to connected RP2040/RP2350..."
    DEPENDS ${PROJECT_NAME}
    VERBATIM
)
# ---- Print build info ----
message(STATUS "")
message(STATUS "========================================")
message(STATUS "  CHAdeMO Controller Build")
message(STATUS "  Role: ${CHADEMO_ROLE}")
message(STATUS "  Target: Raspberry Pi Pico (RP2040)")
message(STATUS "========================================")
message(STATUS "")
