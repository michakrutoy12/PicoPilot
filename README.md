# PicoPilot

PicoPilot is experimental flight-control firmware for the Raspberry Pi Pico. It reads an MPU6500 IMU, a BME280 barometer and a NEO-6M GPS receiver, then uses those measurements for attitude and altitude estimates, basic fixed-wing control, and GPS navigation.

The firmware supports direct RC control, altitude and course hold, optional waypoint following, and return-to-home after RC signal loss. Routes are stored in flash. Sensor and navigation data is also sent over UART as binary telemetry.

## Project status

This is a prototype, not a tested or safety-certified flight controller. Its control behavior, sensor calibration, coordinate references, wiring and failsafes have not been validated for flight. In particular, the speed estimate is GPS ground speed rather than airspeed, and the altitude reference used for Home has not been checked against the fused altitude estimate.

Do not connect this firmware to an aircraft or other equipment where a failure could cause injury or damage. Test it with actuators disconnected, and use appropriate simulation and independent safety systems before any hardware trials.

## Hardware

The current pin assignments are defined in [`PicoPilot.cpp`](./PicoPilot.cpp).

| Function | Pico pin |
| --- | --- |
| Throttle PWM | GPIO 14 |
| Roll PWM | GPIO 13 |
| Pitch PWM | GPIO 12 |
| Yaw PWM | GPIO 11 |
| PPM receiver input | GPIO 9 |
| IMU data-ready interrupt | GPIO 2 |
| I2C0 SDA / SCL | GPIO 20 / GPIO 21 |
| UART0 GPS RX / telemetry TX | GPIO 17 / GPIO 16 |

The MPU6500 and BME280 share I2C0; the BME280 is configured for address `0x76`. UART0 runs at 9600 baud. The ADC inputs are initialized, but the ACS712 driver is not used by the main firmware.

Receiver channels are read in this order: roll, pitch, throttle, yaw, mode, and speed. GPIO assignments and control directions should be checked against the actual airframe before connecting any actuators.

## Control modes

The route-mode setting, `AUTOPILOT_MODE_ROUTE`, is `false` by default in the source.

With route mode disabled, channel 5 selects direct control below 1500 µs and autopilot above that threshold. The autopilot holds the altitude and GPS course captured when it engages. Channel 6 selects an automatic speed target from 1200 µs; below that, throttle remains under pilot control.

When route mode is enabled and a valid route and navigation data are available, the firmware follows waypoint altitude, speed and bearing targets. While the RC signal is present, it uses direct pilot control; if the signal is lost, it follows the route. Route mode does not use the separate return-to-home behavior.

With route mode disabled, loss of the receiver signal for two seconds requests return-to-home. The firmware needs a saved Home position and fresh GPS and sensor data to navigate. It targets at least 30 m above Home, steers toward Home, then requests a 60 m orbit nearby. This is not an automatic landing procedure. If required navigation data is unavailable, it commands minimum throttle and neutral control surfaces.

## Routes

A route can contain up to 64 waypoints. Each waypoint has latitude and longitude in decimal degrees, altitude in metres, and speed in metres per second. The example [`route.csv`](./route.csv) shows the comma-separated format:

```csv
latitude,longitude,altitude,speed
56.02409,92.93459,500,20
56.10046,92.45026,1000,20
55.95662,92.37586,500,20
```

To upload a route, connect to the Pico's USB CDC serial port and send `START`, the waypoint lines, and `END`, each on its own line. Do not send the CSV header. The firmware validates the route and saves it to flash. Input lines must fit in the 64-byte receive buffer.

To use waypoint navigation, set `AUTOPILOT_MODE_ROUTE` to `true` in [`PicoPilot.cpp`](./PicoPilot.cpp) and rebuild. A valid route must already be stored on the Pico; route mode is disabled at startup if none is found.

## Build

The project uses CMake, Ninja and the Raspberry Pi Pico SDK. CMake is configured for the `pico` board and Pico SDK 2.3.1.

```sh
cmake -S . -B build -G Ninja
cmake --build build --parallel
```

The build produces `PicoPilot.elf` and the Pico SDK output files, including UF2, in `build/`.

USB diagnostics are enabled by default. In that configuration, startup waits for a USB CDC connection and prints sensor, GPS, estimator and receiver status periodically. To build without diagnostics:

```sh
cmake -S . -B build -G Ninja -DPICOPILOT_ENABLE_DIAGNOSTICS=OFF
cmake --build build --parallel
```

## Telemetry

The firmware sends a packed binary telemetry packet over UART0 TX at about 5 Hz. Packets start with `0xAA 0x55` and contain a timestamp, attitude quaternion, altitude and vertical speed, GPS position and motion data, Home distance and status, and an XOR checksum. The packet uses the firmware's native numeric representations; it is not a text or versioned protocol.

## Source layout

- [`PicoPilot.cpp`](./PicoPilot.cpp) — board setup, sensor and receiver handling, control loop, route upload and telemetry.
- [`include/SensorFusion/`](./include/SensorFusion) — attitude and altitude estimation.
- [`include/control/`](./include/control) — attitude and energy-based control.
- [`include/routing/`](./include/routing) — waypoint navigation and flash storage.
- [`include/gps/`](./include/gps) — NEO-6M parsing and receive buffer.
- [`include/mpu6500/`](./include/mpu6500), [`include/bme280/`](./include/bme280), [`include/acs712/`](./include/acs712) — sensor drivers.
