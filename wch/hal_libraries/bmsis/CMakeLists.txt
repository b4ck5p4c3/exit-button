target_sources(${TARGET_ELF} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/source/core_riscv.c
    ${CMAKE_CURRENT_SOURCE_DIR}/source/startup_ch32v00x.S
    ${CMAKE_CURRENT_SOURCE_DIR}/source/debug.c
    ${CMAKE_CURRENT_SOURCE_DIR}/include/core_riscv.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/debug.h
)

target_include_directories(${TARGET_ELF} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
