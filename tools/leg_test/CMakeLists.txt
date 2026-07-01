cmake_minimum_required(VERSION 3.16)
project(leg_test CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)

set(DRIVER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../driver)
set(RL_SAR_INC ${CMAKE_CURRENT_SOURCE_DIR}/../../src/rl_sar/include)

add_executable(leg_test
    main.cpp
    ${DRIVER_DIR}/src/dog_driver.cpp
    ${DRIVER_DIR}/src/robstride.cpp
    ${DRIVER_DIR}/src/can_interface.cpp
    ${DRIVER_DIR}/src/observations.cpp
    ${DRIVER_DIR}/src/wit_c_sdk.c
    ${DRIVER_DIR}/src/serial.c
)

target_include_directories(leg_test PRIVATE
    ${DRIVER_DIR}/include
    ${RL_SAR_INC}
    ${EIGEN3_INCLUDE_DIR}
)
target_compile_definitions(leg_test PRIVATE DOG_DRIVER_EXPORTS)
target_link_libraries(leg_test Threads::Threads)
