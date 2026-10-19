
#pragma once

#include <cstddef>
#include <cstdint>

namespace RouteStorage {

constexpr uint32_t ROUTE_MAGIC = 0x524F5554;
constexpr uint16_t ROUTE_VERSION = 1;
constexpr std::size_t MAX_WAYPOINTS = 64;

struct Waypoint {
    double latitude;
    double longitude;
    float altitude;
    float speed;
};

struct Route {
    uint16_t current_waypoint;
    uint32_t magic = ROUTE_MAGIC;
    uint16_t version = ROUTE_VERSION;
    uint16_t count;
    Waypoint points[MAX_WAYPOINTS];
};

class RouteLoader {
public:
    RouteLoader() = default;

    bool load(Route& route);
    bool validate(const Route& route) const;
    bool save(const Route& route);
};

} // namespace RouteStorage