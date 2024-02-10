file(GLOB HAL_SOURCE source/*.c)
file(GLOB HAL_INCLUDE include/*.h)

target_sources(${TARGET_ELF} PUBLIC
    ${HAL_SOURCE}
    ${HAL_INCLUDE}
)

target_include_directories(${TARGET_ELF} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
