cmake_minimum_required(VERSION 3.16)
project(kpkd_response_test CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

set(DRIVER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../driver)

add_executable(kpkd_response_test
    main.cpp
    ${DRIVER_DIR}/src/dog_driver.cpp
    ${DRIVER_DIR}/src/robstride.cpp
    ${DRIVER_DIR}/src/can_interface.cpp
    ${DRIVER_DIR}/src/observations.cpp
    ${DRIVER_DIR}/src/wit_c_sdk.c
    ${DRIVER_DIR}/src/serial.c
)

target_include_directories(kpkd_response_test PRIVATE
    ${DRIVER_DIR}/include
)

target_compile_definitions(kpkd_response_test PRIVATE DOG_DRIVER_EXPORTS)
target_link_libraries(kpkd_response_test Threads::Threads yaml-cpp)
target_compile_options(kpkd_response_test PRIVATE -Wall -Wextra)
