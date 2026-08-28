add_library(pico_bme280 bme280.hpp bme280.cpp)

target_link_libraries(pico_bme280 pico_stdlib hardware_i2c)

target_include_directories(pico_bme280 PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")