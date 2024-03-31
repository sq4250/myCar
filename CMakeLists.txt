cmake_minimum_required(VERSION 3.15)
project(test C CXX ASM)

add_definitions(-DUSE_HAL_DRIVER)
add_definitions(-DSTM32F103xB)

file(GLOB_RECURSE SRC "Core/Src/*.c" "Drivers/*.c")
file(GLOB_RECURSE CXXSRC "Core/Src/*.cpp")

include_directories(Core/Inc)
include_directories(Drivers/STM32F1xx_HAL_Driver/Inc)
include_directories(Drivers/STM32F1xx_HAL_Driver/Inc/Legacy)
include_directories(Drivers/CMSIS/Device/ST/STM32F1xx/Include)
include_directories(Drivers/CMSIS/Include)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_SYSTEM_NAME Generic)

set(COMMON_FLAGS "-mcpu=cortex-m3 -mthumb -mthumb-interwork -Os -Wall -ffunction-sections -fdata-sections -fno-common")
set(CMAKE_CXX_FLAGS "${COMMON_FLAGS} -std=c++17")
set(CMAKE_C_FLAGS "${COMMON_FLAGS} -std=gnu11")

set(CMAKE_EXE_LINKER_FLAGS "-Wl,--gc-sections --specs=nosys.specs -T${CMAKE_SOURCE_DIR}/STM32F103C8Tx_FLASH.ld")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${CMAKE_SOURCE_DIR}/build/${PROJECT_NAME}.map,--cref")

set_property(SOURCE startup_stm32f103xb.s PROPERTY LANGUAGE C)
set_source_files_properties(startup_stm32f103xb.s PROPERTIES COMPILE_FLAGS "-x assembler-with-cpp")

add_library(STARTUP startup_stm32f103xb.s)

add_executable(${PROJECT_NAME}.elf ${SRC} ${CXXSRC})

target_link_libraries(${PROJECT_NAME}.elf STARTUP)

set(HEX_FILE ${CMAKE_SOURCE_DIR}/build/${PROJECT_NAME}.hex)
set(BIN_FILE ${CMAKE_SOURCE_DIR}/build/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE} \nBuilding ${BIN_FILE}")
