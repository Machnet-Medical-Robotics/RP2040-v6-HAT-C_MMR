# CMake minimum required version
cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
if(NOT DEFINED PICO_SDK_PATH)
    set(PICO_SDK_PATH ${CMAKE_SOURCE_DIR}/libraries/pico-sdk)
    message(STATUS "PICO_SDK_PATH = ${PICO_SDK_PATH}")
endif()

include(rp2040_v6_hat_c-patch.cmake)
include(pico_sdk_import.cmake)
include(rp2040_v6_hat_c_sdk_version.cmake)

# Set project name
set(PROJECT_NAME RP2040-v6-HAT-C)

# Set project informations
project(${PROJECT_NAME} C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# MMR: IMPORTANT: In order to solve a build issue with these two executables, we avoid building them here
# See: https://forum.wiznet.io/t/topic/11673

# MMR: Avoid building 'pioasm'
add_executable(Pioasm IMPORTED)
set_property(TARGET Pioasm PROPERTY IMPORTED_LOCATION
"C:\\Program Files\\Raspberry Pi\\Pico SDK v1.5.1\\picotool\\pioasm.exe")
set(Pioasm_FOUND 1)

# MMR: Avoid building 'elf2uf2'
add_executable(ELF2UF2 IMPORTED)
set_property(TARGET ELF2UF2 PROPERTY IMPORTED_LOCATION
"C:\\Program Files\\Raspberry Pi\\Pico SDK v1.5.1\\picotool\\elf2uf2.exe")
set(ELF2UF2_FOUND 1)

# Initialize the SDK
pico_sdk_init()

# Set ethernet chip
set(WIZNET_CHIP W6100)

if(${WIZNET_CHIP} STREQUAL W6100)
    add_definitions(-D_WIZCHIP_=W6100)
else()
    message(FATAL_ERROR "WIZNET_CHIP is wrong = ${WIZNET_CHIP}")
endif()

message(STATUS "WIZNET_CHIP = ${WIZNET_CHIP}")

# Set the project root directory if it's not already defined, as may happen if
# the tests folder is included directly by a parent project, without including
# the top level CMakeLists.txt.
if(NOT DEFINED WIZNET_DIR)
    set(WIZNET_DIR ${CMAKE_SOURCE_DIR}/libraries/io6Library)
    message(STATUS "WIZNET_DIR = ${WIZNET_DIR}")
endif()

if(NOT DEFINED MBEDTLS_DIR)
    set(MBEDTLS_DIR ${CMAKE_SOURCE_DIR}/libraries/mbedtls)
    message(STATUS "MBEDTLS_DIR = ${MBEDTLS_DIR}")
endif()

if(NOT DEFINED PORT_DIR)
    set(PORT_DIR ${CMAKE_SOURCE_DIR}/port)
    message(STATUS "PORT_DIR = ${PORT_DIR}")
endif()

# Turn off mbedtls test mode
set(ENABLE_PROGRAMS OFF CACHE BOOL "Build mbedtls programs")
set(ENABLE_TESTING OFF CACHE BOOL "Build mbedtls testing")
add_definitions(-DMBEDTLS_CONFIG_FILE="${PORT_DIR}/mbedtls/inc/ssl_config.h")
add_definitions(-DSET_TRUSTED_CERT_IN_SAMPLES)
# MMR: remove these warnings
add_definitions(-D_CRT_SECURE_NO_WARNINGS)

# Hardware-specific examples in subdirectories:
add_subdirectory(examples)

# MMR: cardiovasc apps subdirectory:
add_subdirectory(cardiovasc_apps)

# Add libraries in subdirectories
add_subdirectory(${CMAKE_SOURCE_DIR}/libraries)
add_subdirectory(${MBEDTLS_DIR})
add_subdirectory(${PORT_DIR})

# Set compile options
add_compile_options(
        -Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -Wno-maybe-uninitialized
        )
