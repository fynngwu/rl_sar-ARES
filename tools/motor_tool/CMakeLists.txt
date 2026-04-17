cmake_minimum_required(VERSION 3.16)
project(motor_tool CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)

set(DRIVER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../driver)

add_executable(motor_tool
    main.cpp
    ${DRIVER_DIR}/src/robstride.cpp
    ${DRIVER_DIR}/src/can_interface.cpp
    ${DRIVER_DIR}/src/observations.cpp
    ${DRIVER_DIR}/src/wit_c_sdk.c
    ${DRIVER_DIR}/src/serial.c
)

target_include_directories(motor_tool PRIVATE ${DRIVER_DIR}/include)
target_link_libraries(motor_tool Threads::Threads)
