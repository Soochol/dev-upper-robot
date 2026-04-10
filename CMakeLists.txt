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
set(CMAKE_PROJECT_NAME PSP)

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
    # SEGGER RTT — high-speed logging over SWD, used by __io_putchar in main.c
    Core/Third_Party/SEGGER_RTT/SEGGER_RTT.c

    # Application tasks (Phase 1: stub bodies). Listed explicitly so a new
    # file added here also triggers a CMake reconfigure on next build.
    Core/Src/app/t_state.c
    Core/Src/app/t_pid.c
    Core/Src/app/t_ml.c
    Core/Src/app/t_logger.c
    Core/Src/app/rtt_log.c

    # FSM (Phase 2): pure transition logic + state lookup table.
    Core/Src/app/fsm.c
    Core/Src/app/state_table.c

    # I2C sensor drivers (Phase 3a): TBP-H70 IR temperature + bus scanner.
    Core/Src/app/sensors_i2c.c

    # Control loop (Phase 3b): PID controller and TIM1/GPIO actuators.
    Core/Src/app/pid.c
    Core/Src/app/actuators.c

    # Trigger providers (Phase 4): strategy pattern + rule/ML providers.
    Core/Src/app/trigger.c
    Core/Src/app/trigger_rule.c
    Core/Src/app/trigger_ml.c
    Core/Src/app/model_rf.c
    Core/Src/app/sd_logger.c

    # Feature extraction: IMU tilt with adaptive LPF.
    Core/Src/app/features.c

    # SK6812MINI RGB LED driver (TIM2_CH2 + DMA).
    Core/Src/app/sk6812.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Third_Party/SEGGER_RTT
    # Application headers under Core/Inc/app are referenced as "app/foo.h"
    # so we add Core/Inc as the search root.
    Core/Inc
)

# ML workflow build-time configuration.
# Usage: cmake --preset Debug -DDATA_COLLECT=ON   (SD logging + button control)
#        cmake --preset Debug -DML_TRIGGER=ON      (ML inference instead of rule)
option(DATA_COLLECT "Enable SD card CSV logging with button control" OFF)
option(ML_TRIGGER   "Use ML trigger provider instead of rule-based"  OFF)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    $<$<BOOL:${DATA_COLLECT}>:DATA_COLLECT_MODE=1>
    $<$<NOT:$<BOOL:${DATA_COLLECT}>>:DATA_COLLECT_MODE=0>
    $<$<BOOL:${ML_TRIGGER}>:TRIGGER_SOURCE=1>
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
