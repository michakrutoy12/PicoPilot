add_library(pico_mpu6500 mpu6500.hpp mpu6500.cpp)

target_link_libraries(pico_mpu6500 pico_stdlib hardware_i2c)

target_include_directories(pico_mpu6500 PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")