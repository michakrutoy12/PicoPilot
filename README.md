# PicoPilot

PicoPilot is experimental flight-control firmware for the Raspberry Pi Pico. It combines data from an MPU6500 IMU, a BME280 barometer and a NEO-6M GPS receiver to estimate attitude and altitude, control a basic fixed-wing aircraft, and support GPS navigation.

The firmware includes direct RC control, altitude and course hold, optional waypoint navigation, and return-to-home after loss of the RC signal. Routes are stored in flash, and sensor and navigation data is sent over UART as binary telemetry.

## Project status

PicoPilot is a prototype, not a tested or safety-certified flight controller. Its control behavior, sensor calibration, coordinate references, wiring and failsafes have not been validated for flight. The speed estimate is GPS ground speed, not airspeed, and the Home altitude reference has not been verified against the fused altitude estimate.

Do not connect this firmware to an aircraft or other equipment where a failure could cause injury or damage. Test with actuators disconnected, and use appropriate simulation and independent safety systems before any hardware trials.

## Hardware

Current pin assignments are defined in [`PicoPilot.cpp`](./PicoPilot.cpp).

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

The MPU6500 and BME280 share I2C0; the BME280 uses address `0x76`. UART0 runs at 9600 baud. ADC inputs are initialized, but the main firmware does not use the ACS712 driver.

Receiver channels are read in this order: roll, pitch, throttle, yaw, mode and speed. Check pin assignments and control directions against the airframe before connecting actuators.

The MPU6500 samples at 100 Hz, while PWM outputs run at 50 Hz. The attitude controller also runs at 100 Hz so it does not repeatedly integrate or differentiate the same IMU sample. Roll and pitch rate PID values are conservative starting points, not flight-tested gains. Verify control directions with the propeller removed, and tune only after safe ground checks.

## Control modes

`AUTOPILOT_MODE_ROUTE` is `false` by default in the source.

With route mode disabled, channel 5 selects direct control below 1500 µs and autopilot at or above that threshold. When autopilot engages, it holds the captured altitude and GPS course. Channel 6 selects an automatic speed target from 1200 µs; below that point, throttle remains under pilot control.

With route mode enabled and a valid route and navigation data available, the firmware follows waypoint altitude, speed and bearing targets. While the RC signal is present, it uses direct pilot control; after signal loss, it follows the route. Route mode does not use the separate return-to-home behavior.

With route mode disabled, losing the receiver signal for two seconds requests return-to-home. The firmware needs a saved Home position and fresh GPS and sensor data to navigate. It targets at least 30 m above Home, steers toward Home, then requests a 60 m orbit nearby. This is not an automatic landing procedure. If required navigation data is unavailable, the firmware commands minimum throttle and neutral control surfaces.

## Routes

A route can contain up to 64 waypoints. Each waypoint specifies latitude and longitude in decimal degrees, altitude in metres and speed in metres per second. [`route.csv`](./route.csv) shows the comma-separated format:

```csv
latitude,longitude,altitude,speed
56.02409,92.93459,500,20
56.10046,92.45026,1000,20
55.95662,92.37586,500,20
```

To upload a route, connect to the Pico's USB CDC serial port and send `START`, the waypoint lines, then `END`, with each on its own line. Do not include the CSV header. The firmware validates the route and saves it to flash. Each input line must fit in the 64-byte receive buffer.

Waypoint navigation requires `AUTOPILOT_MODE_ROUTE` to be set to `true` in [`PicoPilot.cpp`](./PicoPilot.cpp), followed by a rebuild. A valid route must already be stored on the Pico; route mode is disabled at startup if no route is found.

## Build

The project uses CMake, Ninja and the Raspberry Pi Pico SDK. CMake is configured for the `pico` board and Pico SDK 2.3.1.

```sh
cmake -S . -B build -G Ninja
cmake --build build --parallel
```

The build produces `PicoPilot.elf` and the Pico SDK output files, including UF2, in `build/`.

USB diagnostics are enabled by default. With diagnostics enabled, startup waits for a USB CDC connection and periodically prints sensor, GPS, estimator and receiver status. To disable diagnostics:

```sh
cmake -S . -B build -G Ninja -DPICOPILOT_ENABLE_DIAGNOSTICS=OFF
cmake --build build --parallel
```

## Telemetry

The firmware sends a packed binary telemetry packet over UART0 TX at about 5 Hz. Each packet starts with `0xAA 0x55` and contains a timestamp, attitude quaternion, altitude and vertical speed, GPS position and motion data, Home distance and status, and an XOR checksum. Numeric fields use the firmware's native representations; the packet is not a text or versioned protocol.

## Source layout

- [`PicoPilot.cpp`](./PicoPilot.cpp) — board setup, sensor and receiver handling, control loop, route upload and telemetry.
- [`include/SensorFusion/`](./include/SensorFusion) — attitude and altitude estimation.
- [`include/control/`](./include/control) — attitude and energy-based control.
- [`include/routing/`](./include/routing) — waypoint navigation and flash storage.
- [`include/gps/`](./include/gps) — NEO-6M parsing and receive buffer.
- [`include/mpu6500/`](./include/mpu6500), [`include/bme280/`](./include/bme280), [`include/acs712/`](./include/acs712) — sensor drivers.
