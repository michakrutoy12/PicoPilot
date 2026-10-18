#include "routing.hpp"

#include <cmath>

namespace {

constexpr double PI = 3.14159265358979323846;
constexpr double DEG_TO_RAD = PI / 180.0;

} // namespace

float calculate_distance(double lat1, double lon1, double lat2, double lon2) {
    constexpr double EARTH_RADIUS_M = 6371000.88;

    double dlat = (lat2 - lat1) * DEG_TO_RAD;
    double dlon = (lon2 - lon1) * DEG_TO_RAD;

    double mean_lat = ((lat1 + lat2) / 2.0) * DEG_TO_RAD;

    double x = dlon * std::cos(mean_lat);
    double y = dlat;

    return static_cast<float>(EARTH_RADIUS_M * std::sqrt(x * x + y * y));
}

float calculate_bearing(double lat1, double lon1, double lat2, double lon2) {
    double phi1 = lat1 * DEG_TO_RAD;
    double phi2 = lat2 * DEG_TO_RAD;
    double dlambda = (lon2 - lon1) * DEG_TO_RAD;

    double y = std::sin(dlambda) * std::cos(phi2);
    double x = std::cos(phi1) * std::sin(phi2) - std::sin(phi1) * std::cos(phi2) * std::cos(dlambda);

    return static_cast<float>(std::atan2(y, x));
}

float calculate_orbit_bearing(double lat, double lon, double center_lat, double center_lon, double radius, bool clockwise) {
    double bearing_to_center = calculate_bearing(lat, lon, center_lat, center_lon);

    double distance = calculate_distance(lat, lon, center_lat, center_lon);
    double tangent = bearing_to_center + (clockwise ? 90.0 : -90.0) * DEG_TO_RAD;
    double error = distance - radius;
    double correction = std::atan2(error, radius);
    double bearing = tangent - correction;
    bearing = std::fmod(bearing + 2.0 * PI, 2.0 * PI);

    return static_cast<float>(bearing);
}

namespace Avionics {

Router::Router() = default;

bool Router::set_route(const RouteStorage::Route& route) {
    const RouteStorage::RouteLoader route_loader;
    if (!route_loader.validate(route)) {
        route_active = false;
        route_ = RouteStorage::Route {};
        return false;
    }

    route_ = route;
    route_active = true;
    waypoint_departure_required_ = false;

    return true;
}

void Router::update(const RouterInputs& inputs, RouterOutputs& outputs) {
    if (!route_active ||
        route_.current_waypoint >= route_.count) {
        outputs.route_active = false;
        return;
    }

    const auto& target = route_.points[route_.current_waypoint];

    const float distance = calculate_distance(
        inputs.lat,
        inputs.lon,
        target.latitude,
        target.longitude
    );

    bool can_advance = true;
    if (waypoint_departure_required_) {
        const float distance_from_last_reached = calculate_distance(
            inputs.lat,
            inputs.lon,
            last_reached_latitude_,
            last_reached_longitude_
        );
        can_advance = distance_from_last_reached > waypoint_radius;
        if (can_advance) {
            waypoint_departure_required_ = false;
        }
    }

    if (can_advance && distance <= waypoint_radius) {
        if (route_.current_waypoint + 1 < route_.count) {
            last_reached_latitude_ = target.latitude;
            last_reached_longitude_ = target.longitude;
            waypoint_departure_required_ = true;
            ++route_.current_waypoint;
        } else {
            route_active = false;
            outputs.route_active = false;

            outputs.altitude = target.altitude;
            outputs.speed = target.speed;
            outputs.bearing = calculate_bearing(
                inputs.lat,
                inputs.lon,
                target.latitude,
                target.longitude
            );

            return;
        }
    }

    const auto& next = route_.points[route_.current_waypoint];

    outputs.altitude = next.altitude;
    outputs.speed = next.speed;
    outputs.bearing = calculate_bearing(
        inputs.lat,
        inputs.lon,
        next.latitude,
        next.longitude
    );

    outputs.route_active = true;
}

} // namespace Avionics
