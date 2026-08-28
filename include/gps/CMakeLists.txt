add_library(pico_neo6m neo6m.hpp neo6m.cpp ring_buffer.hpp)

target_link_libraries(pico_neo6m pico_stdlib hardware_uart)

target_include_directories(pico_neo6m PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")