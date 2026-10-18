#pragma once

#include "storage.hpp"

float calculate_distance(double lat1, double lon1, double lat2, double lon2);
float calculate_bearing(double lat1, double lon1, double lat2, double lon2);
float calculate_orbit_bearing(double lat, double lon, double center_lat, double center_lon, double radius, bool clockwise);

namespace Avionics {

struct RouterInputs {
    double lat;
    double lon;
};

struct RouterOutputs {
    float altitude;
    float speed;
    float bearing;
    bool route_active;
};

class Router {
public:
    Router();

    bool set_route(const RouteStorage::Route& route);
    void update(const RouterInputs& inputs, RouterOutputs& outputs);

private:
    RouteStorage::Route route_{};
    bool route_active = false;
    bool waypoint_departure_required_ = false;
    double last_reached_latitude_ = 0.0;
    double last_reached_longitude_ = 0.0;
    float waypoint_radius = 15.0f;
};

} // namespace Avionics
