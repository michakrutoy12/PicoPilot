# PicoPilot

PicoPilot is an experimental C++17 firmware project for the Raspberry Pi Pico. It combines sensor acquisition, attitude and altitude estimation, basic flight control, GPS waypoint navigation, and return-to-home (RTH) behavior.

> **Experimental and not flight-safe.** This software has not been validated as a reliable flight controller. Control laws, sensor data, coordinate conventions, actuator directions, failure handling, and hardware wiring may be incorrect or incomplete. Do not use it on an aircraft or other system where a failure could cause injury or damage. Build and test with actuators disconnected, then use an appropriate simulator and independent safety systems.

## Implemented features

- MPU6500 IMU acquisition and quaternion attitude estimation with an extended Kalman filter.
- GPS course-assisted yaw-drift correction.
- BME280 barometer and GPS altitude fusion for altitude and vertical-speed estimates.
- TECS-style altitude and speed control, with attitude and rate control for the servos.
- Six-channel PPM receiver input and direct-control/autopilot selection.
- GPS waypoint route loading and navigation.
- Loss-of-receiver-signal RTH with a target altitude and an orbit around Home.
- UART GPS input and binary telemetry output.
- USB serial diagnostics, optionally enabled at build time.
- RP2040 dual-core processing, interrupts, PWM, DMA, I2C, UART, and watchdog support.

## Autopilot behavior

`AUTOPILOT_MODE_ROUTE` in [`PicoPilot.cpp`](./PicoPilot.cpp) selects the normal autopilot navigation mode. It is currently initialized to `false`; it is a firmware setting, not a live receiver mode switch.

| Condition | Behavior |
| --- | --- |
| Receiver signal is valid and channel 5 is below 1500 µs | Pass the receiver's throttle, roll, pitch, and yaw values directly to the outputs. |
| Receiver signal is valid, channel 5 is at least 1500 µs, and route mode is off | Hold the altitude and course captured on entering autopilot. The speed target is set from channel 6. |
| Receiver signal is valid, channel 5 is at least 1500 µs, and route mode is on | Follow the route's bearing, altitude, and speed targets. Automatic throttle control is used. A valid route and fresh GPS are required. |
| Receiver signal has been absent for 2 seconds | Request RTH, overriding direct control and route-mode selection. RTH requires a saved Home position and valid, fresh GPS. |

In non-route autopilot mode, channel 6 below 1200 µs leaves throttle under receiver control. At or above 1200 µs, channel 6 is mapped using a 900–2200 µs range to a speed target; because of the throttle-control threshold, the effective commanded range is approximately 6.2–27 m/s. Route mode and RTH use TECS throttle control regardless of this channel setting. The speed target used for RTH is the last accepted channel-6 speed target, or 15 m/s if none has been set.

RTH targets the greater of the EKF's altitude when RTH starts and `Home altitude + 30 m`. It steers directly toward Home while farther than 120 m away; at or inside 120 m it requests a 60 m-radius orbit around Home. This is orbit guidance only, not an automatic landing procedure. If Home or fresh GPS is unavailable, the firmware sets minimum throttle and neutral control outputs instead of attempting RTH. A stale IMU or failed EKF heartbeat also causes minimum-throttle/neutral outputs.

The current speed estimate comes from GPS ground speed, not airspeed. The TECS input floors measured speed at 15 m/s, which can hide a real speed drop and does not account for wind. The Home altitude is GPS altitude while the current altitude estimate is fused from barometer and GPS; their datum and offset have not been validated. These behaviors require careful review and testing before any hardware experiment.

## Sensors, controls, and pins

The current pin assignments are defined near the top of [`PicoPilot.cpp`](./PicoPilot.cpp):

| Function | GPIO / setting |
| --- | --- |
| Throttle PWM | 14 |
| Roll PWM | 13 |
| Pitch PWM | 12 |
| Yaw PWM | 11 |
| PPM input | 9 |
| IMU data-ready interrupt | 2 |
| I2C0 SDA / SCL | 20 / 21 |
| UART0 GPS RX / TX | 17 / 16 |
| Route-load button | 14 |
| ADC inputs initialized | 26, 27, 28 |

> **GPIO14 conflict:** the route-load button and throttle PWM are both assigned to GPIO14. The button is sampled at startup before PWM initialization, but this shared pin is a hardware/configuration conflict that should be resolved before connecting a receiver, ESC, or servo.

The BME280 is initialized at I2C address `0x76`. UART0 is configured for 9600 baud; GPS data is received on GPIO17 and binary telemetry is transmitted on GPIO16. PWM outputs are generated for throttle, roll, pitch, and yaw. The ADC pins are initialized, but the ACS712 driver is not currently used by the main control loop.

Receiver channels are interpreted in this order: roll, pitch, throttle, yaw, mode (channel 5), and speed (channel 6). Direct control is selected when channel 5 is below 1500 µs; autopilot is selected at or above 1500 µs. PWM output conversion uses 1500 µs as servo center and maps normalized servo commands to approximately 1000–2000 µs; throttle maps 0–1 to 1040–2000 µs.

## Routes

Routes are stored in flash and contain up to 64 waypoints. Each waypoint has latitude and longitude in decimal degrees, altitude in metres, and speed in metres per second.

The input format is one CSV waypoint per line:

```csv
latitude,longitude,altitude,speed
56.02409,92.93459,500,20
56.10046,92.45026,1000,20
55.95662,92.37586,500,20
```

[`route.csv`](./route.csv) is an example. To upload a route:

1. Start the Pico with the route-load button held. The firmware checks the button during the first second after startup.
2. Connect to the USB CDC serial port. Route upload mode initializes USB stdio and waits for a host connection.
3. Send `START`, then the waypoint lines, then `END`, each terminated with a newline. Do not send the CSV header row (`latitude,longitude,altitude,speed`); the firmware treats every line between `START` and `END` as a waypoint.
4. The firmware validates and writes the route to flash.

The route loader accepts up to 64 waypoints and expects short lines (the input buffer is 64 bytes). The first waypoint is selected when the route is activated. Waypoint advancement uses a 15 m acceptance radius. At the final waypoint, the controller retains that waypoint's altitude and speed and holds the current GPS course.

The configured route is loaded and validated at boot. To enable route navigation, set `AUTOPILOT_MODE_ROUTE` to `true` in the firmware and rebuild. If diagnostics are enabled but no valid route is available, the firmware reports this over USB and disables route mode. Without diagnostics, route mode with no valid route produces minimum-throttle/neutral outputs while the receiver signal is present.

## Build

The project uses CMake, Ninja (in the provided build configuration), and the Raspberry Pi Pico SDK for the RP2040.

Configure with a Pico SDK available through the environment or the SDK import configuration:

```sh
cmake -S . -B build -G Ninja
cmake --build build --parallel
```

The default board is `pico`. The build produces `PicoPilot.elf` and Pico SDK output formats such as UF2 in the build directory.

Periodic USB diagnostics are enabled by default:

```sh
cmake -S . -B build -G Ninja -DPICOPILOT_ENABLE_DIAGNOSTICS=ON
cmake --build build --parallel
```

With diagnostics enabled, startup waits for a USB CDC connection unless the device is in route-upload mode (which also waits for USB). Diagnostics print sensor, EKF, GPS, and PPM state approximately once per second. Disable diagnostics with `-DPICOPILOT_ENABLE_DIAGNOSTICS=OFF` to avoid the diagnostic USB initialization and wait in normal startup.

## Telemetry

Telemetry is sent over UART0 TX using DMA at approximately 5 Hz. The packed binary packet starts with bytes `0xAA 0x55` and contains a millisecond timestamp, attitude quaternion, altitude, vertical speed, latitude, longitude, distance from Home, Home-valid flag, GPS speed, course, HDOP, satellite count, and an XOR checksum.

The packet uses the firmware's native floating-point and integer representations; it is not a text stream. A receiver should parse the packed structure and verify the checksum. GPS course is obtained from the NMEA receiver, whose course field is in degrees.

## Project layout

- [`PicoPilot.cpp`](./PicoPilot.cpp) — board setup, sensor/receiver acquisition, main control loop, route upload, and telemetry.
- [`include/SensorFusion/`](./include/SensorFusion) — attitude filter, altitude estimator, and acceleration compensation.
- [`include/control/`](./include/control) — TECS-style energy controller, attitude controller, and PID implementation.
- [`include/routing/`](./include/routing) — geographic calculations, waypoint navigation, and flash route storage.
- [`include/gps/`](./include/gps) — NEO-6M parsing and GPS ring buffer.
- [`include/mpu6500/`](./include/mpu6500), [`include/bme280/`](./include/bme280), [`include/acs712/`](./include/acs712) — sensor drivers.

## Known limitations

- This is a prototype, not a safety-certified autopilot; no flight or hardware-in-the-loop validation is provided.
- No airspeed sensor, wind estimation, landing, geofencing, or obstacle/terrain avoidance is implemented.
- GPS course is track over ground and can differ from the aircraft's heading.
- Home altitude and EKF altitude reference compatibility has not been established.
- RTH fails safe to minimum throttle and neutral surfaces if GPS/Home is unavailable; this is not necessarily a safe recovery for every aircraft.
- The route-load button and throttle output share GPIO14.
- UART telemetry packet layout is an implementation detail and has no separate version field.
- Automatic tests and hardware validation are limited; configuration and control behavior may change.

Review the source and validate all pin assignments, sensor orientation, control directions, receiver calibration, GPS quality, and failure behavior in a safe test setup before connecting actuators.
