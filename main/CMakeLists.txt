# Get all UI source files from the ui directory
file(GLOB_RECURSE SRC_UI "${CMAKE_CURRENT_SOURCE_DIR}/ui/*.c")

set(srcs
    "spi_lcd_touch_example_main.c"
    "ads1115.c"
    ${SRC_UI}
)

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "." "ui"
    REQUIRES esp_lcd lvgl fatfs sdmmc driver 
)