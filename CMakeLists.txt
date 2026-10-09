cmake_minimum_required(VERSION 3.22)

# ------------------------------------------------------------
# Toolchain
# A preset / command-line setting takes precedence.
# ------------------------------------------------------------
if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/gcc-arm-none-eabi.cmake"
        CACHE FILEPATH "ARM toolchain file"
    )
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(ROBOT_FIRST LANGUAGES C CXX ASM)

add_executable(${PROJECT_NAME})

# CubeMX-generated sources, includes and linker configuration.
add_subdirectory(cmake/stm32cubemx)

set(USER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/User")

set(USER_SOURCES
    # BSP
    "${USER_DIR}/Bsp/bsp_delay.cpp"
    "${USER_DIR}/Bsp/bsp_bmi088.cpp"
    "${USER_DIR}/Bsp/bsp_can.cpp"

    # Drivers
    "${USER_DIR}/Driver/Sensor/bmi088.cpp"
    "${USER_DIR}/Driver/Motor/motor_dji.cpp"
    "${USER_DIR}/Driver/Motor/motor_dm.cpp"
    "${USER_DIR}/Driver/Remote/remote.cpp"

    # Services
    "${USER_DIR}/Service/imu_service.cpp"

    # Algorithms
    "${USER_DIR}/Algorithm/pid.cpp"

    # Application
    "${USER_DIR}/Application/robot.cpp"
    "${USER_DIR}/Application/robot_api.cpp"
    "${USER_DIR}/Application/chassis.cpp"
    "${USER_DIR}/Application/manipulator.cpp"
    "${USER_DIR}/Application/communication.cpp"

    # Debug
    "${USER_DIR}/Debug/debug.cpp"
)

set(USER_INCLUDE_DIRS
    "${USER_DIR}/Bsp"
    "${USER_DIR}/Driver/Sensor"
    "${USER_DIR}/Driver/Motor"
    "${USER_DIR}/Driver/Remote"
    "${USER_DIR}/Service"
    "${USER_DIR}/Algorithm"
    "${USER_DIR}/Application"
    "${USER_DIR}/Debug"
)

foreach(USER_FILE IN LISTS USER_SOURCES)
    if(NOT EXISTS "${USER_FILE}")
        message(FATAL_ERROR "Missing User source: ${USER_FILE}")
    endif()
endforeach()

foreach(USER_INCLUDE_DIR IN LISTS USER_INCLUDE_DIRS)
    if(NOT IS_DIRECTORY "${USER_INCLUDE_DIR}")
        message(FATAL_ERROR
            "Missing User include directory: ${USER_INCLUDE_DIR}"
        )
    endif()
endforeach()

target_sources(${PROJECT_NAME} PRIVATE ${USER_SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${USER_INCLUDE_DIRS}
)

message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Toolchain: ${CMAKE_TOOLCHAIN_FILE}")