# Setup toolchain
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_SYSTEM_PROCESSOR stm32-f1xx)

find_program(CMAKE_MAKE_PROGRAM make)
find_program(CMAKE_C_COMPILER arm-none-eabi-gcc)
find_program(CMAKE_CXX_COMPILER arm-none-eabi-g++)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

find_program(ARM_SIZE_EXECUTABLE arm-none-eabi-size)
find_program(ARM_GDB_EXECUTABLE arm-none-eabi-gdb)

set(shared_options "-mcpu=cortex-m3 -mthumb -Og -Wall -fno-math-errno -ffunction-sections -fdata-sections -g -gdwarf-2")

set(CMAKE_C_FLAGS_INIT "-Os -std=gnu99 ${shared_options}" CACHE INTERNAL "Initial options for C compiler.")
set(CMAKE_CXX_FLAGS_INIT "-Os -std=gnu++11 -fstrict-enums -fsized-deallocation -fno-rtti ${shared_options}" CACHE INTERNAL "Initial options for CXX compiler.")
set(CMAKE_EXE_LINKER_FLAGS_INIT "${linker_flags} ${shared_options}" CACHE INTERNAL "Initial options for executable linker.")

