#include "neo6m.hpp"
#include <pico/stdlib.h>
#include <cmath>
#include <limits>

namespace {
    float to_float(std::string_view sv, float default_val = 0.0f) {
        if (sv.empty()) return default_val;
        float val;
        auto [ptr, ec] = std::from_chars(sv.data(), sv.data() + sv.size(), val);
        return (ec == std::errc{}) ? val : default_val;
    }

    double to_double(std::string_view sv, double default_val = 0.0) {
        if (sv.empty()) return default_val;
        double val;
        auto [ptr, ec] = std::from_chars(sv.data(), sv.data() + sv.size(), val);
        return (ec == std::errc{}) ? val : default_val;
    }

    long to_long(std::string_view sv, int base = 10, long default_val = 0) {
        if (sv.empty()) return default_val;
        long val;
        auto [ptr, ec] = std::from_chars(sv.data(), sv.data() + sv.size(), val, base);
        return (ec == std::errc{}) ? val : default_val;
    }

    double parse_coordinate(std::string_view value_field, std::string_view hemisphere_field) {
        if (value_field.empty()) return 0.0;

        const double value = to_double(value_field);
        if (!std::isfinite(value)) return 0.0;

        const double degrees = std::floor(value / 100.0);
        const double minutes = value - degrees * 100.0;
        double coord = degrees + (minutes / 60.0);

        if (!hemisphere_field.empty()) {
            const char hemi = hemisphere_field[0];
            if (hemi == 'S' || hemi == 'W') {
                coord = -coord;
            }
        }

        return coord;
    }
}

Neo6M::Neo6M() {
    buffer.reserve(128);
}

bool Neo6M::update(char c) {
    if (c == '\n' || c == '\r') {
        if (!buffer.empty()) {
            parse_line(buffer);
            buffer.clear();
            if (data_updated) {
                data_updated = false;
                return true;
            }
        }
    } else {
        if (buffer.size() < 120) {
            buffer.push_back(c);
        } else {
            buffer.clear();
        }
    }
    return false;
}

void Neo6M::parse_line(std::string_view line) {
    if (line.empty() || line[0] != '$') return;

    const size_t checksum_separator = line.find('*');
    if (checksum_separator == std::string_view::npos ||
        checksum_separator + 3 != line.size()) return;

    auto hex_value = [](char c) -> int {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        return -1;
    };
    const int checksum_high = hex_value(line[checksum_separator + 1]);
    const int checksum_low = hex_value(line[checksum_separator + 2]);
    if (checksum_high < 0 || checksum_low < 0) return;

    uint8_t checksum = 0;
    for (size_t i = 1; i < checksum_separator; ++i) {
        checksum ^= static_cast<uint8_t>(line[i]);
    }
    if (checksum != static_cast<uint8_t>((checksum_high << 4) | checksum_low)) return;

    if (line.find("GGA") != std::string_view::npos) {
        process_gga(line);
    } else if (line.find("RMC") != std::string_view::npos) {
        process_rmc(line);
    } // else if (line.find("VTG") != std::string_view::npos) {
    //     process_vtg(line);
    // }
}

std::string_view Neo6M::get_field(std::string_view line, int field_index) {
    size_t start = 0;
    int current_field = 0;

    while (current_field < field_index) {
        start = line.find(',', start);
        if (start == std::string_view::npos) return "";
        start++;
        current_field++;
    }

    size_t end = line.find(',', start);
    if (end == std::string_view::npos) {
        end = line.find('*', start);
        if (end == std::string_view::npos) end = line.length();
    }

    return line.substr(start, end - start);
}

void Neo6M::process_gga(std::string_view line) {
    std::string_view s_lat = get_field(line, 2);
    std::string_view s_lat_hem = get_field(line, 3);
    std::string_view s_lon = get_field(line, 4);
    std::string_view s_lon_hem = get_field(line, 5);
    std::string_view s_fix = get_field(line, 6);
    std::string_view s_sat = get_field(line, 7);
    std::string_view s_hdop = get_field(line, 8);
    std::string_view s_alt = get_field(line, 9);

    if (!s_lat.empty()) {
        latitude = parse_coordinate(s_lat, s_lat_hem);
    }

    if (!s_lon.empty()) {
        longitude = parse_coordinate(s_lon, s_lon_hem);
    }

    if (!s_fix.empty())  fix_state = to_long(s_fix);
    if (!s_sat.empty())  satellites = to_long(s_sat);
    if (!s_hdop.empty()) hdop = to_float(s_hdop);
    if (!s_alt.empty())  altitude = to_float(s_alt);

    data_updated = true;
}

void Neo6M::process_rmc(std::string_view line) {
    std::string_view s_lat = get_field(line, 3);
    std::string_view s_lat_hem = get_field(line, 4);
    std::string_view s_lon = get_field(line, 5);
    std::string_view s_lon_hem = get_field(line, 6);
    std::string_view s_date = get_field(line, 9);
    std::string_view s_time = get_field(line, 1);
    std::string_view s_speed = get_field(line, 7);
    std::string_view s_course = get_field(line, 8);
    std::string_view s_status = get_field(line, 2);

    if (!s_lat.empty()) {
        latitude = parse_coordinate(s_lat, s_lat_hem);
    }

    if (!s_lon.empty()) {
        longitude = parse_coordinate(s_lon, s_lon_hem);
    }

    if (s_status != "A") {
        course_valid = false;
        return;
    }

    if (!s_speed.empty()) speed = to_float(s_speed) * 0.514f;
    if (s_time.length() >= 6) gps_time = to_long(s_time);
    if (s_date.length() == 6) gps_date = to_long(s_date);
    if (!s_course.empty()) {
        const float course_deg = to_float(s_course, std::numeric_limits<float>::quiet_NaN());
        course_valid = std::isfinite(course_deg) && course_deg >= 0.0f && course_deg <= 360.0f;
        if (course_valid) {
            constexpr float degrees_to_radians = 0.01745329251994329577f;
            course = course_deg * degrees_to_radians;
            course_timestamp_us = time_us_32();
        }
    } else {
        course_valid = false;
    }

    data_updated = true;
}

void Neo6M::process_vtg(std::string_view line) {
    std::string_view s_course = get_field(line, 1);
    std::string_view s_speed_kmh = get_field(line, 7);

    if (!s_course.empty()) {
        const float course_deg = to_float(s_course, std::numeric_limits<float>::quiet_NaN());
        course_valid = std::isfinite(course_deg) && course_deg >= 0.0f && course_deg <= 360.0f;
        if (course_valid) {
            constexpr float degrees_to_radians = 0.01745329251994329577f;
            course = course_deg * degrees_to_radians;
            course_timestamp_us = time_us_32();
        }
    }

    if (!s_speed_kmh.empty()) {
        speed = to_float(s_speed_kmh) / 3.6f;
    }

    data_updated = true;
}