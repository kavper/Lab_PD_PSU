cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
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

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Src/app.c
    Core/Src/bq25731.c
    Core/Src/control_cv.c
    Core/Src/debug_uart.c
    Core/Src/gui.c
    Core/Src/measurements.c
    Core/Src/power_manager.c
    Core/Src/power_stage.c
    Core/Src/psu_gui_api.c
    Core/Src/tps25751.c
    Core/Src/tps_int_event.c
    Drivers/ssd1306/ssd1306.c
    Drivers/ssd1306/ssd1306_fonts.c
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
set(STM32_PROGRAMMER_CLI
    "/Users/kacpergawel/Library/Application Support/stm32cube/bundles/programmer/2.21.0/bin/STM32_Programmer_CLI"
    CACHE FILEPATH "Path to STM32_Programmer_CLI")

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
