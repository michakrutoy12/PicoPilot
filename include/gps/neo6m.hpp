#pragma once

#include <pico/stdlib.h>
#include <string>
#include <charconv>

class Neo6M {
public:
    double latitude = 0.0;
    double longitude = 0.0;
    float altitude = 0.0f;
    float speed = 0.0f;
    int fix_state = 0;
    int satellites = 0;
    float hdop = 0.0f;
    float course = 0.0f;

    uint gps_time = 0;
    uint gps_date = 0;

    Neo6M();
    
    bool update(char c);

private:
    std::string buffer;
    bool data_updated = false;
    
    void parse_line(std::string_view line);
    void process_gga(std::string_view line);
    void process_rmc(std::string_view line);
    void process_vtg(std::string_view line);
    
    std::string_view get_field(std::string_view line, int field_index);
};