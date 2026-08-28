add_library(pico_control control.hpp control.cpp)

target_link_libraries(pico_control pico_stdlib)

target_include_directories(pico_control PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")