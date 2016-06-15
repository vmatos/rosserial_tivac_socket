# 
# Copyright (c) 2016
# Author: Vitor Matos
#
# Quick and dirty cmake script which prepares cross-compilation for catkinized, rosserial_client enabled projects.
# 
# Requires two enviroment variables:
#   TIVA_WARE_PATH        - TI TivaWare SDK path
#   TIVA_FLASH_EXECUTABLE - lm4flash tool executable
#
# Defines function to be included in cross-compiled project's CMakeLists.txt:
#   generate_tivac_firmware
# 
# With Arguments:
#     STARTUP - Optional argument. Takes the name of custom startup file.
#     SRCS    - Required. List of source files to compile.
#     INCS    - Optional. List of include directories.
#     LIBS    - Optional. List of libraries to be linked.
#
# Creates custom targets for project:
#   <parent_catkin_project>_${CMAKE_PROJECT_NAME}.axf   - Build binary file
#   <parent_catkin_project>_${CMAKE_PROJECT_NAME}_flash - Flashes board with binary file
#   <parent_catkin_project>_${CMAKE_PROJECT_NAME}_size  - Prints out the size of sections and totals of binary file
#   <parent_catkin_project>_${CMAKE_PROJECT_NAME}_dump  - Prints table of symbols and such
#
# Example usage:
#
#   generate_tivac_firmware(
#     STARTUP custom_startup.c
#     SRCS buttons.cpp buttons.c
#     INCS .
#   )
# 
# This example prepares a cmake project for Tiva C Connected Launchpad.
# Adds two source files to be compiled on the project.
# Includes the project directory.
# Includes a custom startup file to specify on the interrupt vector some interrupt handlers.
# 

cmake_minimum_required(VERSION 2.8.3)
include(CMakeParseArguments)

if(NOT WIN32)
  string(ASCII 27 Esc)
  set(Red         "${Esc}[31m")
  set(ColourReset "${Esc}[m")
endif()

# Set cross compilation information
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# GCC toolchain prefix
set(TOOLCHAIN_PREFIX "arm-none-eabi")
set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}-gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}-g++)
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}-as)
set(CMAKE_AR ${TOOLCHAIN_PREFIX}-ar)
set(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}-objcopy)
set(CMAKE_OBJDUMP ${TOOLCHAIN_PREFIX}-objdump)
set(CMAKE_SIZE ${TOOLCHAIN_PREFIX}-size)

# This hangs CMake when called by catkin
# enable_language(ASM) 

set(CPU "-mcpu=cortex-m4")
set(FPU "-mfpu=fpv4-sp-d16 -mfloat-abi=softfp")
# Cache if it set before project command (e.g. in toolchain):
set(CMAKE_ASM_FLAGS "-mthumb ${CPU} ${FPU} -MD" CACHE STRING "" FORCE)
set(CMAKE_C_FLAGS "-mthumb ${CPU} ${FPU} -std=gnu99 -Os -ffunction-sections -fdata-sections -MD -Wall -pedantic -fsingle-precision-constant" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS "-mthumb ${CPU} ${FPU} -std=c++11 -Os -ffunction-sections -fdata-sections -MD -Wall -pedantic -fno-exceptions -fno-rtti -fsingle-precision-constant -nostartfiles" CACHE STRING "" FORCE)

set(LINKER_SCRIPT_TM4C1294XL ${CMAKE_CURRENT_LIST_DIR}/../tm4c1294.ld)
set(LINKER_SPECS ${CMAKE_CURRENT_LIST_DIR}/../tiva.specs)

set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")
set(CMAKE_EXE_LINKER_FLAGS "-T${LINKER_SCRIPT_TM4C1294XL} -specs=${LINKER_SPECS} -Wl,-Map=memmap.map" CACHE STRING "" FORCE)

# Processor specific definitions
add_definitions(-Dgcc)

# How could we find Tivaware SDK paths? similarly to Arduino toolchain?
# Just like ArduinoToolchain.cmake from Tomasz Bogdal (QueezyTheGreat) - https://github.com/queezythegreat/arduino-cmake
set(TIVA_WARE_PATH $ENV{TIVA_WARE_PATH})
include_directories($ENV{TIVA_WARE_PATH})
set(FLASH_EXECUTABLE $ENV{TIVA_FLASH_EXECUTABLE})

# Generates targets based on options and definitions
function(GENERATE_TIVAC_FIRMWARE)
  message(STATUS "[TIVAC] Generating firmware ${CMAKE_PROJECT_NAME}")
  set(options )
  set(oneValueArgs STARTUP)
  set(multiValueArgs SRCS INCS LIBS)
  cmake_parse_arguments(INPUT "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
  
  message(STATUS "[TIVAC] Configuring board for ${CMAKE_PROJECT_NAME}")
  add_definitions(-DPART_TM4C1294NCPDT)
  add_definitions(-DTARGET_IS_TM4C129_RA0)
  set(CMAKE_EXE_LINKER_FLAGS "-T${LINKER_SCRIPT_TM4C1294XL} -specs=${LINKER_SPECS} -Wl,-Map=memmap.map" CACHE STRING "" FORCE)
  
  if(INPUT_STARTUP)
    message(STATUS "[TIVAC] Using custom startup file ${INPUT_STARTUP} for ${CMAKE_PROJECT_NAME}")
    list(APPEND INPUT_SRCS ${INPUT_STARTUP})
  else()
    list(APPEND INPUT_SRCS ${ROS_LIB_DIR}/startup_gcc.c)
  endif()

  # driverlib dependency
  add_library(driverlib STATIC IMPORTED)
  set_target_properties(driverlib PROPERTIES
    IMPORTED_LOCATION ${TIVA_WARE_PATH}/driverlib/gcc/libdriver.a
  )
  add_custom_target(build_driverlib 
    COMMAND ${CMAKE_MAKE_PROGRAM}
    WORKING_DIRECTORY ${TIVA_WARE_PATH}/driverlib
    COMMENT "[TIVAC] Building driverlib using original makefile"
  )
  add_dependencies(driverlib build_driverlib)

  include_directories(
    ${INPUT_INCS}
    ${ROS_LIB_DIR}
    ${TIVA_WARE_PATH}/third_party/lwip-1.4.1/src/include
    ${TIVA_WARE_PATH}/third_party/lwip-1.4.1/src/include/ipv4
    ${TIVA_WARE_PATH}/third_party/lwip-1.4.1/ports/tiva-tm4c129/include
  )
  add_executable(${CMAKE_PROJECT_NAME}.axf
    ${INPUT_SRCS}
    ${ROS_LIB_DIR}/duration.cpp 
    ${ROS_LIB_DIR}/time.cpp
    ${ROS_LIB_DIR}/tivac_hardware.cpp
    ${ROS_LIB_DIR}/ethClient.c
    ${TIVA_WARE_PATH}/utils/lwiplib.c
    ${TIVA_WARE_PATH}/utils/ringbuf.c
    # For debugging
    ${TIVA_WARE_PATH}/utils/uartstdio.c
    ${OTHER_SRCS}
  )
  target_link_libraries(${CMAKE_PROJECT_NAME}.axf 
    ${INPUT_LIBS}
    driverlib
  )

  add_custom_target("dump" DEPENDS ${CMAKE_PROJECT_NAME}.axf 
    COMMAND ${CMAKE_OBJDUMP} -x ${EXECUTABLE_OUTPUT_PATH}/${CMAKE_PROJECT_NAME}.axf
  )
  
  add_custom_target("size" DEPENDS ${CMAKE_PROJECT_NAME}.axf 
    COMMAND ${CMAKE_SIZE} -A -d ${EXECUTABLE_OUTPUT_PATH}/${CMAKE_PROJECT_NAME}.axf
    COMMAND ${CMAKE_SIZE} -A -x ${EXECUTABLE_OUTPUT_PATH}/${CMAKE_PROJECT_NAME}.axf
  )

  add_custom_target("flash" DEPENDS ${CMAKE_PROJECT_NAME}.axf 
    COMMAND ${CMAKE_OBJCOPY} -O binary ${EXECUTABLE_OUTPUT_PATH}/${CMAKE_PROJECT_NAME}.axf ${EXECUTABLE_OUTPUT_PATH}/${CMAKE_PROJECT_NAME}.bin 
    COMMAND ${FLASH_EXECUTABLE} ${EXECUTABLE_OUTPUT_PATH}/${CMAKE_PROJECT_NAME}.bin
  )
endfunction()
