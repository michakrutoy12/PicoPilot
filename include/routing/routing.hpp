#pragma once

#include "storage.hpp"

// Estimates the surface distance between two latitude/longitude points in metres.
float calculate_distance(double lat1, double lon1, double lat2, double lon2);
// Returns the initial bearing between two points in radians.
float calculate_bearing(double lat1, double lon1, double lat2, double lon2);
// Computes a bearing that guides a vehicle around a center at the requested radius.
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
    bool route_complete;
};

class Router {
public:
    // Creates a router with no active route.
    Router();

    // Validates and activates a route, starting at its first waypoint.
    bool set_route(const RouteStorage::Route& route);
    // Updates the next waypoint and navigation targets for the current position.
    void update(const RouterInputs& inputs, RouterOutputs& outputs);

private:
    RouteStorage::Route route_{};
    bool route_active = false;
    bool route_complete_ = false;
    bool waypoint_departure_required_ = false;
    double last_reached_latitude_ = 0.0;
    double last_reached_longitude_ = 0.0;
    float waypoint_radius = 15.0f;
};

} // namespace Avionics
