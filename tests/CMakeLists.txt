cmake_minimum_required(VERSION 3.22)
project(PSP_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Unity test framework
add_library(unity STATIC unity/unity.c)
target_include_directories(unity PUBLIC unity/)

# Application source under test (HAL/FreeRTOS-free only)
set(APP_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../Core/Src/app)
set(APP_INC ${CMAKE_CURRENT_SOURCE_DIR}/../Core/Inc)

add_executable(run_tests
    run_all.c
    test_fsm.c
    test_pid.c
    test_features.c
    test_state_table.c
    ${APP_SRC}/fsm.c
    ${APP_SRC}/state_table.c
    ${APP_SRC}/pid.c
    ${APP_SRC}/features.c
)

target_include_directories(run_tests PRIVATE
    ${APP_INC}          # Core/Inc/ (for app/ headers)
    stubs/              # FreeRTOS/HAL stubs
)

target_link_libraries(run_tests unity m)
