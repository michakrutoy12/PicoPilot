add_library(pico_sensor_fusion imu.hpp imu.cpp correction.hpp altitude.hpp)

target_link_libraries(pico_sensor_fusion pico_stdlib)

target_include_directories(pico_sensor_fusion PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")