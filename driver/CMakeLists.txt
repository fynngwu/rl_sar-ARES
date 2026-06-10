cmake_minimum_required(VERSION 3.16)
project(dog_driver CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)

add_library(dog_driver SHARED
    src/dog_driver.cpp
    src/robstride.cpp
    src/can_interface.cpp
    src/observations.cpp
    src/serial.c
    src/wit_c_sdk.c
)

target_include_directories(dog_driver PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_definitions(dog_driver PRIVATE DOG_DRIVER_EXPORTS)

target_link_libraries(dog_driver
    Threads::Threads
)

target_compile_options(dog_driver PRIVATE -Wall -Wextra -fPIC)

add_executable(collect_pace_data
    src/collect_pace_data.cpp
)

target_include_directories(collect_pace_data PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(collect_pace_data
    dog_driver
    Threads::Threads
)

target_compile_options(collect_pace_data PRIVATE -Wall -Wextra)

add_executable(motor_diag
    ../tools/motor_diagnostics/motor_diag.cpp
)

target_include_directories(motor_diag PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(motor_diag
    dog_driver
    Threads::Threads
)

target_compile_options(motor_diag PRIVATE -Wall -Wextra)

add_executable(motor_squat_diag
    ../tools/motor_diagnostics/motor_squat_diag.cpp
)

target_include_directories(motor_squat_diag PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(motor_squat_diag
    dog_driver
    Threads::Threads
)

target_compile_options(motor_squat_diag PRIVATE -Wall -Wextra)
