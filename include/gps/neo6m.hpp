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

    uint32_t time = 0;
    uint32_t date = 0;

    // Creates a parser with a reserved buffer for incoming NMEA sentences.
    Neo6M();
    
    // Adds one character and reports when a sentence updates navigation data.
    bool update(char c);

private:
    std::string buffer;
    bool data_updated = false;
    
    // Dispatches a complete NMEA sentence to its message-specific parser.
    void parse_line(std::string_view line);
    // Updates position, fix quality, and altitude from a GGA sentence.
    void process_gga(std::string_view line);
    // Updates speed, course, and time from a valid RMC sentence.
    void process_rmc(std::string_view line);
    // Updates speed from a VTG sentence.
    void process_vtg(std::string_view line);
    
    // Returns a comma-separated field from an NMEA sentence.
    std::string_view get_field(std::string_view line, int field_index);
};