INCLUDE(CMakeForceCompiler)

SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_VERSION 1)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)

SET(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F107VCTx_FLASH.ld)
SET(COMMON_FLAGS "-mcpu=cortex-m3 -mthumb -mthumb-interwork -mfix-cortex-m3-ldrd -mlittle-endian -mfpu=vfp -mfloat-abi=soft -fno-builtin -ffunction-sections -fdata-sections -gdwarf-2 -g -fno-common -fmessage-length=0")
SET(CMAKE_CXX_FLAGS "${COMMON_FLAGS} -std=c++11")
SET(CMAKE_C_FLAGS "${COMMON_FLAGS} -std=gnu99")
SET(CMAKE_EXE_LINKER_FLAGS "-Wl,-gc-sections -T ${LINKER_SCRIPT}")