add_executable(pico_emb
    main.c
    tft_lcd_ili9341/ili9341/ili9341.c
    tft_lcd_ili9341/gfx/gfx_ili9341.c
)

target_include_directories(pico_emb PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/tft_lcd_ili9341/gfx
    ${CMAKE_CURRENT_LIST_DIR}/tft_lcd_ili9341/ili9341
)

target_link_libraries(pico_emb
    pico_stdlib
    pico_multicore
    hardware_gpio
    hardware_timer
    hardware_pwm
    hardware_spi
    hardware_dma
)

set_target_properties(pico_emb PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
pico_add_extra_outputs(pico_emb)