add_library(picopilot_control control.hpp control.cpp)

target_link_libraries(picopilot_control pico_stdlib)

target_include_directories(picopilot_control PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")