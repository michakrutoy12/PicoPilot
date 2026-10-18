add_library(picopilot_acs712 acs712.hpp acs712.cpp)

target_link_libraries(picopilot_acs712 pico_stdlib hardware_adc)

target_include_directories(picopilot_acs712 PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")