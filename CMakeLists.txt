cmake_minimum_required(VERSION 3.12)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/toolchain-ch32v00x.cmake)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(exit-button)

link_directories(${CMAKE_SOURCE_DIR}/libs)

set(TARGET_ELF ${PROJECT_NAME}.elf)

file(GLOB SRC_FILES
        "src/main.c"
        "src/system_ch32v00x.c"
        "src/ch32v00x_it.c"
        "src/modbus.c")

add_executable(${TARGET_ELF} ${SRC_FILES})

target_include_directories(${TARGET_ELF} PUBLIC ${CMAKE_SOURCE_DIR}/global_include)

enable_language(C CXX ASM)
target_compile_features(${TARGET_ELF} PRIVATE cxx_std_17)

add_subdirectory(wch)

add_custom_command(TARGET ${TARGET_ELF} POST_BUILD
        COMMAND ${CROSS_OBJDUMP} "-CdSx" $<TARGET_FILE:${TARGET_ELF}> > $<TARGET_FILE:${TARGET_ELF}>.lst
        COMMENT "Generating disassembler file")
add_custom_command(TARGET ${TARGET_ELF} POST_BUILD
        COMMAND ${CROSS_OBJCOPY} "-Obinary" $<TARGET_FILE:${TARGET_ELF}> $<TARGET_FILE:${TARGET_ELF}>.bin
        COMMENT "Generating bin file")
add_custom_command(TARGET ${TARGET_ELF} POST_BUILD
        COMMAND ${CROSS_OBJCOPY} "-Oihex" $<TARGET_FILE:${TARGET_ELF}> $<TARGET_FILE:${TARGET_ELF}>.hex
        COMMENT "Generating hex file")