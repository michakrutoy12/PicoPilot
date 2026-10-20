#include "storage.hpp"

#include <array>
#include <cmath>
#include <cstring>

#include "hardware/flash.h"
#include "pico/flash.h"

namespace {

static_assert(PICO_FLASH_SIZE_BYTES >= FLASH_SECTOR_SIZE);
static_assert(ROUTE_STORAGE_FLASH_OFFSET % FLASH_SECTOR_SIZE == 0);
static_assert(
    ROUTE_STORAGE_FLASH_OFFSET <= PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE,
    "Route storage offset must leave a complete flash sector"
);

constexpr std::size_t PROGRAM_SIZE =
    ((sizeof(RouteStorage::Route) + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE) * FLASH_PAGE_SIZE;
static_assert(PROGRAM_SIZE <= FLASH_SECTOR_SIZE, "Route does not fit in its flash sector");

alignas(FLASH_PAGE_SIZE) std::array<uint8_t, PROGRAM_SIZE> route_flash_buffer;

struct FlashWriteParameters {
    uint32_t offset;
    const uint8_t* data;
    std::size_t size;
};

// Erases the route sector and programs its prepared page-aligned contents.
void write_route_to_flash(void* context) {
    const auto& parameters = *static_cast<const FlashWriteParameters*>(context);
    flash_range_erase(parameters.offset, FLASH_SECTOR_SIZE);
    flash_range_program(parameters.offset, parameters.data, parameters.size);
}

} // namespace

namespace RouteStorage {

bool RouteLoader::load(Route& route) {
    const auto* flash_data = reinterpret_cast<const uint8_t*>(XIP_BASE + ROUTE_STORAGE_FLASH_OFFSET);
    std::memcpy(&route, flash_data, sizeof(route));
    if (validate(route)) {
        return true;
    }

    route = {};
    return false;
}

bool RouteLoader::validate(const Route& route) const {
    if (route.magic != ROUTE_MAGIC ||
        route.version != ROUTE_VERSION ||
        route.count == 0 ||
        route.count > MAX_WAYPOINTS) {
        return false;
    }

    for (std::size_t i = 0; i < route.count; ++i) {
        const auto& point = route.points[i];
        if (!std::isfinite(point.latitude) ||
            point.latitude < -90.0 || point.latitude > 90.0 ||
            !std::isfinite(point.longitude) ||
            point.longitude < -180.0 || point.longitude > 180.0 ||
            !std::isfinite(point.altitude) ||
            !std::isfinite(point.speed) ||
            point.speed < 0.0f) {
            return false;
        }
    }

    return true;
}

bool RouteLoader::save(const Route& route) {
    if (!validate(route)) {
        return false;
    }

    route_flash_buffer.fill(0xFF);
    std::memcpy(route_flash_buffer.data(), &route, sizeof(route));

    FlashWriteParameters parameters {
        ROUTE_STORAGE_FLASH_OFFSET,
        route_flash_buffer.data(),
        route_flash_buffer.size()
    };
    return flash_safe_execute(write_route_to_flash, &parameters, UINT32_MAX) == 0;
}

} // namespace RouteStorage
