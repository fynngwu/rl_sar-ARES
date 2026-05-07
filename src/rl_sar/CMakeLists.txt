cmake_minimum_required(VERSION 3.10)
project(rl_sar VERSION 4.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(SYSTEM_TYPE ${CMAKE_SYSTEM_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

add_compile_definitions(USE_ROS2)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
string(TOUPPER "${CMAKE_BUILD_TYPE}" UPPER_BUILD_TYPE)
if(UPPER_BUILD_TYPE STREQUAL "DEBUG")
    add_compile_options(-g)
elseif(UPPER_BUILD_TYPE STREQUAL "RELEASE")
    add_compile_options(-O3)
endif()

get_filename_component(PROJECT_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

set(DRIVER_DIR "${PROJECT_ROOT_DIR}/driver")

add_definitions(-DCMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
add_definitions(-DPOLICY_DIR="${PROJECT_ROOT_DIR}/policy")
add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# NOTE: No "interfaces" package — ARES uses standard ROS2 messages only

# Inference Runtime
set(INFERENCE_RUNTIME_DIR "${PROJECT_ROOT_DIR}/library/inference_runtime")

# ONNX Runtime Setup
set(USE_ONNX "OFF")
set(ONNX_RUNTIME_LIB "")
set(ONNX_RUNTIME_DIR "${INFERENCE_RUNTIME_DIR}/onnxruntime")

# Detect Jetson platform
set(IS_JETSON FALSE)
if(EXISTS "/etc/nv_tegra_release")
    set(IS_JETSON TRUE)
    message(STATUS "Jetson platform detected - ONNX Runtime enabled")
endif()

if(EXISTS "${ONNX_RUNTIME_DIR}/include" AND EXISTS "${ONNX_RUNTIME_DIR}/lib")
    set(USE_ONNX "ON")
    add_compile_definitions(USE_ONNX)

    message(STATUS "Found ONNX Runtime at: ${ONNX_RUNTIME_DIR}")

    # Read ONNX Runtime version
    if(EXISTS "${ONNX_RUNTIME_DIR}/VERSION_NUMBER")
        file(READ "${ONNX_RUNTIME_DIR}/VERSION_NUMBER" ONNX_RUNTIME_VERSION)
        string(STRIP "${ONNX_RUNTIME_VERSION}" ONNX_RUNTIME_VERSION)
    endif()

    include_directories("${ONNX_RUNTIME_DIR}/include")

    # Detect library extension
    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        set(LIB_EXT "so")
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
        set(LIB_EXT "dylib")
    endif()

    # Find library
    if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
        set(LIB_NAMES onnxruntime.lib)
    else()
        set(LIB_NAMES libonnxruntime.${LIB_EXT} onnxruntime.${LIB_EXT})
    endif()

    find_library(ONNX_RUNTIME_LIB_PATH
        NAMES ${LIB_NAMES}
        PATHS "${ONNX_RUNTIME_DIR}/lib"
        NO_DEFAULT_PATH
    )

    if(ONNX_RUNTIME_LIB_PATH)
        if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
            add_library(onnxruntime_prebuilt STATIC IMPORTED GLOBAL)
        else()
            add_library(onnxruntime_prebuilt SHARED IMPORTED GLOBAL)
        endif()

        set_target_properties(onnxruntime_prebuilt PROPERTIES
            IMPORTED_LOCATION "${ONNX_RUNTIME_LIB_PATH}"
            INTERFACE_INCLUDE_DIRECTORIES "${ONNX_RUNTIME_DIR}/include"
        )

        set(ONNX_RUNTIME_LIB onnxruntime_prebuilt)
        message(STATUS "ONNX Runtime library: ${ONNX_RUNTIME_LIB_PATH}")
    else()
        message(WARNING "ONNX Runtime library not found")
        set(USE_ONNX "OFF")
    endif()
else()
    message(WARNING "ONNX Runtime not found. Please run: bash download_inference_runtime.sh onnx")
endif()

find_package(Threads REQUIRED)

if(SYSTEM_TYPE STREQUAL "Linux")
    set(YAML_CPP_LIBRARIES yaml-cpp)
    link_directories(/usr/local/lib)
    include_directories(${YAML_CPP_INCLUDE_DIR})
elseif(SYSTEM_TYPE STREQUAL "Darwin")
    find_library(YAML_CPP_LIB yaml-cpp REQUIRED)
    set(YAML_CPP_LIBRARIES ${YAML_CPP_LIB})
    link_directories(/opt/homebrew/lib)
    if(YAML_CPP_INCLUDE_DIR)
        include_directories(${YAML_CPP_INCLUDE_DIR})
    endif()
endif()

# Dog driver library (pre-built)
# Looks for libdog_driver.so in the ARES driver directory
find_library(DOG_DRIVER_LIB
    NAMES dog_driver
    PATHS "${DRIVER_DIR}"
    NO_DEFAULT_PATH
)
if(DOG_DRIVER_LIB)
    message(STATUS "Found dog_driver library: ${DOG_DRIVER_LIB}")
    add_library(dog_driver SHARED IMPORTED GLOBAL)
    set_target_properties(dog_driver PROPERTIES
        IMPORTED_LOCATION "${DOG_DRIVER_LIB}"
        INTERFACE_INCLUDE_DIRECTORIES "${DRIVER_DIR}/include"
    )
else()
    message(WARNING "dog_driver library not found in ${DRIVER_DIR}")
endif()

# RPATH Configuration for external libraries
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)

set(PREBUILT_LIB_PATHS "")
if(DOG_DRIVER_LIB)
    list(APPEND PREBUILT_LIB_PATHS "${DRIVER_DIR}")
endif()
if(USE_ONNX)
    list(APPEND PREBUILT_LIB_PATHS "${ONNX_RUNTIME_DIR}/lib")
endif()
set(CMAKE_INSTALL_RPATH "${PREBUILT_LIB_PATHS}")

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--disable-new-dtags")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--disable-new-dtags")
endif()

message(STATUS "Configured RPATH:")
foreach(path ${PREBUILT_LIB_PATHS})
    message(STATUS "  - ${path}")
endforeach()

include_directories(
    include
    library/core/observation_buffer
    library/core/loop
    library/core/inference_runtime
    library/core/logger
)

# library for inference runtime
add_library(inference_runtime library/core/inference_runtime/inference_runtime.cpp)
if(USE_ONNX)
    target_link_libraries(inference_runtime PUBLIC ${ONNX_RUNTIME_LIB})
    target_compile_definitions(inference_runtime PRIVATE USE_ONNX)
endif()

# library for observation buffer
add_library(observation_buffer library/core/observation_buffer/observation_buffer.cpp)

# executable for ARES RL inference node (ROS2 topic interface)
if(SYSTEM_TYPE STREQUAL "Linux")
    add_executable(ares src/rl_real_ares.cpp)
    target_link_libraries(ares
        inference_runtime
        observation_buffer
        ${YAML_CPP_LIBRARIES}
        Threads::Threads
        rclcpp::rclcpp
        sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
        geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
    )

if(DOG_DRIVER_LIB)
        add_library(ares_driver_core STATIC src/ares_driver_core.cpp)
        target_link_libraries(ares_driver_core
            dog_driver
            ${YAML_CPP_LIBRARIES}
            Threads::Threads
        )

        add_executable(ares_driver_node
            src/ares_driver_node.cpp
        )
        target_link_libraries(ares_driver_node
            ares_driver_core
            rclcpp::rclcpp
            sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
            geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
        )
        target_precompile_headers(ares_driver_node PRIVATE
            <rclcpp/rclcpp.hpp>
            <sensor_msgs/msg/joint_state.hpp>
            <sensor_msgs/msg/imu.hpp>
            <geometry_msgs/msg/twist.hpp>
            <yaml-cpp/yaml.h>
        )
    endif()
endif()

message(STATUS "========================================")
message(STATUS "       RL_SAR Build Configuration       ")
message(STATUS "========================================")
message(STATUS "Project: ${PROJECT_NAME} v${PROJECT_VERSION}")
message(STATUS "System: ${SYSTEM_TYPE}")
message(STATUS "Architecture: ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Compiler: ${CMAKE_COMPILER_ID} ${CMAKE_COMPILER_VERSION}")
message(STATUS "----------------------------------------")
message(STATUS "Backend Configuration:")
message(STATUS "USE_ONNX: ${USE_ONNX}")
if(USE_ONNX)
    if(ONNX_RUNTIME_VERSION)
        message(STATUS "ONNX Runtime version: ${ONNX_RUNTIME_VERSION}")
    elseif(onnxruntime_VERSION)
        message(STATUS "ONNX Runtime version: ${onnxruntime_VERSION}")
    else()
        message(STATUS "ONNX Runtime: Available")
    endif()
endif()
message(STATUS "dog_driver: ${DOG_DRIVER_LIB}")
message(STATUS "========================================")
