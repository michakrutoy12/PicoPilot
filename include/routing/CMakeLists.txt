add_library(picopilot_routing routing.cpp routing.hpp)

target_link_libraries(picopilot_routing pico_stdlib)

target_include_directories(picopilot_routing PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")