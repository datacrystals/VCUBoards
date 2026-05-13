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
set(PICO_PLATFORM rp2350 CACHE STRING "Pico target platform")

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
# Energica Display CAN Probe Target
# ============================================================================
add_executable(energica_display_probe
    energica_display_probe.c
    chademo_hal.c
    chademo_can.c
)

target_include_directories(energica_display_probe PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(energica_display_probe
    pico_stdlib
    hardware_spi
    hardware_gpio
    hardware_timer
    hardware_watchdog
    hardware_uart
)

target_compile_options(energica_display_probe PRIVATE
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

# Force VEHICLE role so chademo_config.h is happy; probe doesn't use role GPIOs
target_compile_definitions(energica_display_probe PRIVATE CHADEMO_ROLE_VEHICLE)

pico_enable_stdio_usb(energica_display_probe 1)
pico_enable_stdio_uart(energica_display_probe 0)
pico_add_extra_outputs(energica_display_probe)

# ============================================================================
# Energica Display Focused Probe Target
# ============================================================================
add_executable(energica_display_focused
    energica_display_focused.c
    chademo_hal.c
    chademo_can.c
)

target_include_directories(energica_display_focused PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(energica_display_focused
    pico_stdlib
    hardware_spi
    hardware_gpio
    hardware_timer
    hardware_watchdog
    hardware_uart
)

target_compile_options(energica_display_focused PRIVATE
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

target_compile_definitions(energica_display_focused PRIVATE CHADEMO_ROLE_VEHICLE)

pico_enable_stdio_usb(energica_display_focused 1)
pico_enable_stdio_uart(energica_display_focused 0)
pico_add_extra_outputs(energica_display_focused)

# ============================================================================
# Energica Display Alive / Keep-Awake Target
# ============================================================================
add_executable(energica_display_alive
    energica_display_alive.c
    chademo_hal.c
    chademo_can.c
)

target_include_directories(energica_display_alive PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(energica_display_alive
    pico_stdlib
    hardware_spi
    hardware_gpio
    hardware_timer
    hardware_watchdog
    hardware_uart
)

target_compile_options(energica_display_alive PRIVATE
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

target_compile_definitions(energica_display_alive PRIVATE CHADEMO_ROLE_VEHICLE)

pico_enable_stdio_usb(energica_display_alive 1)
pico_enable_stdio_uart(energica_display_alive 0)
pico_add_extra_outputs(energica_display_alive)

# ============================================================================
# Energica Display Manual Playground Target
# ============================================================================
add_executable(energica_display_manual
    energica_display_manual.c
    chademo_hal.c
    chademo_can.c
)

target_include_directories(energica_display_manual PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(energica_display_manual
    pico_stdlib
    hardware_spi
    hardware_gpio
    hardware_timer
    hardware_watchdog
    hardware_uart
)

target_compile_options(energica_display_manual PRIVATE
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

target_compile_definitions(energica_display_manual PRIVATE CHADEMO_ROLE_VEHICLE)

pico_enable_stdio_usb(energica_display_manual 1)
pico_enable_stdio_uart(energica_display_manual 0)
pico_add_extra_outputs(energica_display_manual)

# ============================================================================
# Energica CAN Bridge Target (raw USB-serial ↔ CAN bridge)
# ============================================================================
add_executable(energica_can_bridge
    energica_can_bridge.c
    chademo_hal.c
    chademo_can.c
)

target_include_directories(energica_can_bridge PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(energica_can_bridge
    pico_stdlib
    hardware_spi
    hardware_gpio
    hardware_timer
    hardware_watchdog
    hardware_uart
)

target_compile_options(energica_can_bridge PRIVATE
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

target_compile_definitions(energica_can_bridge PRIVATE CHADEMO_ROLE_VEHICLE)

pico_enable_stdio_usb(energica_can_bridge 1)
pico_enable_stdio_uart(energica_can_bridge 0)
pico_add_extra_outputs(energica_can_bridge)

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
message(STATUS "  Target: Raspberry Pi Pico 2 (RP2350)")
message(STATUS "========================================")
message(STATUS "")
