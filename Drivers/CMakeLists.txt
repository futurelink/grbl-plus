set(CMAKE_TOOLCHAIN_FILE stm32-f1xx.cmake)

project(Drivers C ASM)

cmake_minimum_required(VERSION 3.5.0)

include_directories(include)
include_directories(STM32F1xx_HAL_Driver/Inc)
include_directories(CMSIS/Include)
include_directories(CMSIS/Device/ST/STM32F1xx/Include)

file(GLOB_RECURSE DRIVERS_SOURCES "STM32F1xx_HAL_Driver/Src/*.c" "CMSIS/Device/ST/STM32F1xx/Source/*.c")
add_library(${PROJECT_NAME} ${DRIVERS_SOURCES})
