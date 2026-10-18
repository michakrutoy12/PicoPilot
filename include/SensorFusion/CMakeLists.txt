add_library(picopilot_sensor_fusion imu.hpp imu.cpp correction.hpp altitude.hpp)

target_link_libraries(picopilot_sensor_fusion pico_stdlib)

target_include_directories(picopilot_sensor_fusion PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")