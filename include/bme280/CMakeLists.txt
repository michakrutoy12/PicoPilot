add_library(picopilot_bme280 bme280.hpp bme280.cpp)

target_link_libraries(picopilot_bme280 pico_stdlib hardware_i2c)

target_include_directories(picopilot_bme280 PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")