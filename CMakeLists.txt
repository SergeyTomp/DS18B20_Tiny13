cmake_minimum_required(VERSION 2.8)

### TOOLCHAIN SETUP AREA #################################################
# Set any variables used in the toolchain prior project() call. In that
# case they are already set and used.
##########################################################################

##################################################################################
# tools to be used for programming the AVR
##################################################################################
set(AVR_UPLOADTOOL avrdude)
set(AVR_PROGRAMMER USBasp)
set(AVR_UPLOADTOOL_PORT usb)
set(AVR_UPLOADTOOL_BITRATE 9600)

#### For Arduino bootloader:
#set(AVR_PROGRAMMER arduino)
#set(AVR_UPLOADTOOL_PORT /dev/tty.wchusbserial1420)

# AVR and fuses needs to be set
set(AVR_MCU attiny13)
set(AVR_H_FUSE 0xFF)
set(AVR_L_FUSE 0x6A)
set(AVR_EX_FUSE 0×00)
set(MCU_SPEED "1200000UL")

#set(SOURCE_FILES main.c)

### printf support
#set(EXTRA_FLAGS "-Wl,-u,vfprintf -lprintf_min")

### END TOOLCHAIN SETUP AREA #############################################
INCLUDE(generic-gcc-avr.cmake)

##########################################################################
# name your project
##########################################################################
project(DS18B20_Tiny13 C)


##################################################################################
# add all sources and headers from ./src
##################################################################################
file(GLOB_RECURSE SRC_C_FILES ${PROJECT_SOURCE_DIR}/src/*.c)
file(GLOB_RECURSE SRC_CXX_FILES ${PROJECT_SOURCE_DIR}/src/*.cpp)
file(GLOB_RECURSE SRC_HEADERS ${PROJECT_SOURCE_DIR}/src/*.h)
##################################################################################
# add AVR executable
##################################################################################
add_avr_executable(
        ${PROJECT_NAME}
        ${SRC_C_FILES} ${SRC_CXX_FILES} ${SRC_HEADERS}
)

find_library(C_LIB c)

message(STATUS "avr-libc: ${C_LIB}")

avr_target_link_libraries(${PROJECT_NAME} ${C_LIB})


##################################################################################
# add AVR library
##################################################################################
#add_avr_library(
#   <library-name>
#   <sources> <headers>
#)

##################################################################################
# link library to executable
# NOTE: It needs to be the elf target.
##################################################################################
# target_link_libraries(<executable-name>-${AVR_MCU}.elf <library-name>-${AVR_MCU})
# OR easier