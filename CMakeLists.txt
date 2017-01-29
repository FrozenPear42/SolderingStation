cmake_minimum_required(VERSION 3.6)
project(OLED C ASM)

add_definitions(-DSTM32F107xC)

file(GLOB_RECURSE USER_SOURCES "Src/*.c")

file(GLOB_RECURSE HAL_SOURCES "Drivers/STM32F1xx_HAL_Driver/Src/*.c")

add_library(CMSIS Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/gcc/startup_stm32f107xc.s)

include_directories(Inc)
include_directories(Drivers/STM32F1xx_HAL_Driver/Inc)
include_directories(Drivers/CMSIS/Include)
include_directories(Drivers/CMSIS/Device/ST/STM32F1xx/Include)
include_directories(usr/lib/arm-none-eabi/include)

add_executable(${PROJECT_NAME}.elf ${USER_SOURCES} ${HAL_SOURCES} ${LINKER_SCRIPT} Inc/SSD1306.h)

target_link_libraries(${PROJECT_NAME}.elf CMSIS)

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${PROJECT_SOURCE_DIR}/build/${PROJECT_NAME}.map")
set(HEX_FILE ${PROJECT_SOURCE_DIR}/build/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_SOURCE_DIR}/build/${PROJECT_NAME}.bin)
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMAND echo Sections:
        COMMAND readelf -S $<TARGET_FILE:${PROJECT_NAME}.elf> | grep 'Name\\|isr\\|text\\|bss\\|data\\|array'
        COMMENT "Building ${HEX_FILE} \nBuilding ${BIN_FILE}")
