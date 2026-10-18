add_library(picopilot_routing routing.cpp routing.hpp storage.hpp storage.cpp)

target_link_libraries(picopilot_routing pico_stdlib hardware_flash pico_flash)

target_include_directories(picopilot_routing PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

target_compile_definitions(picopilot_routing PRIVATE ROUTE_STORAGE_FLASH_OFFSET=0x1FE000)