cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once (CubeMX does not overwrite it).
# cmake/stm32cubemx/CMakeLists.txt IS overwritten on Generate Code.
# Application .c files: list them below AND in .extSettings.
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Lab_PD_PSU)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# CubeMX overwrites cmake/stm32cubemx/CMakeLists.txt on Generate Code.
# Keep application sources here (this file is generated only once).
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Explicit app list (no GLOB — avoids picking up junk / gui.c).
# NEW Core/Src/*.c for the firmware: add it HERE and in .extSettings.
# Do not list CubeMX-owned files (main.c, stm32g4xx_it.c, stm32g4xx_hal_msp.c,
# system_stm32g4xx.c, sysmem.c, syscalls.c) — they come from cmake/stm32cubemx.
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Src/app.c
    Core/Src/board_mx.c
    Core/Src/bq25731.c
    Core/Src/bq76922.c
    Core/Src/control_cv.c
    Core/Src/debug_uart.c
    Core/Src/fan_pwm.c
    Core/Src/host_link.c
    Core/Src/ldo_link.c
    Core/Src/ldo_prereg.c
    Core/Src/measurements.c
    Core/Src/power_manager.c
    Core/Src/power_stage.c
    Core/Src/psu_gui_api.c
    Core/Src/status_led.c
    Core/Src/tps25751.c
    Core/Src/tps_int_event.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Drivers/ssd1306
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Build-and-flash target for VS Code / STM32Cube extension without starting a debug session.
set(STM32_PROGRAMMER_CLI_DEFAULT
    "/Users/kacpergawel/Library/Application Support/stm32cube/bundles/programmer/2.21.0/bin/STM32_Programmer_CLI")
set(STM32_PROGRAMMER_CLI "${STM32_PROGRAMMER_CLI_DEFAULT}"
    CACHE FILEPATH "Path to STM32_Programmer_CLI")
if(NOT STM32_PROGRAMMER_CLI)
    set(STM32_PROGRAMMER_CLI "${STM32_PROGRAMMER_CLI_DEFAULT}"
        CACHE FILEPATH "Path to STM32_Programmer_CLI" FORCE)
endif()

add_custom_target(flash
    COMMAND "${STM32_PROGRAMMER_CLI}"
            -c port=SWD mode=UR reset=HWrst
            -w "$<TARGET_FILE:${CMAKE_PROJECT_NAME}>"
            -v
            -rst
    DEPENDS ${CMAKE_PROJECT_NAME}
    USES_TERMINAL
    COMMENT "Build and flash ${CMAKE_PROJECT_NAME}"
)

# VS Code can keep an old selected target in its internal state.
# Keep this alias so a stale "erase" button still performs the intended flash.
add_custom_target(erase
    DEPENDS flash
    COMMENT "Compatibility alias: erase -> flash"
)
