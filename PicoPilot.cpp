#include "pico/stdlib.h"
#include "pico/flash.h"
#include <cmath>
#include <cstdlib>
#include <cstring>
#include "pico/stdio_usb.h"
#include "stdio.h"
#include <cstdio>

#include "pico/multicore.h"
#include "hardware/sync.h"
#include "hardware/dma.h"

#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/watchdog.h"

#include "mpu6500.hpp"
#include "bme280.hpp"
#include "acs712.hpp"
#include "neo6m.hpp"
#include "ring_buffer.hpp"

#include "imu.hpp"
#include "altitude.hpp"
#include "correction.hpp"
#include "control.hpp"
#include "routing.hpp"
#include "storage.hpp"

/* CONTROL MOTORS */
#define THROTTLE_PIN 14
#define ROLL_PIN 13
#define PITCH_PIN 12
#define YAW_PIN 11
/* ============== */

/* PPM RECIVER SETTINGS */
#define PPM_PIN 9
#define N_CHANNELS 6
#define PAUSE_PULSE_MIN_LENGTH 5000
#define PAUSE_PULSE_MAX_LENGTH 20000

#define SIGNAL_POWER_THRESHOLD 1400
#define MAX_NO_SIGNAL_TIME 2000000
/* ==================== */

/* GPS RECIVER/FEEDBACK TRANSIVER SETTINGS */
#define UART uart0
#define UART_BAUDRATE 9600
#define UART_RX 17
#define UART_TX 16
/* ======================================= */

/* BME280/MPU6500 SETTINGS */
#define I2C_PORT i2c0
#define SDA_PIN 20
#define SCL_PIN 21
#define IMU_INT_PIN 2
/* ======================= */

bool AUTOPILOT_MODE_ROUTE = false;

const uint8_t BUTTON_PIN = 14;
bool route_load_mode = false;
const uint8_t MAX_LINE_LENGTH = 64;

RouteStorage::RouteLoader route_loader;

int telemetry_dma_chan = -1;

MPU6500 imu(I2C_PORT);
BME280 bme(I2C_PORT, 0x76);
Neo6M gps;
RingBuffer<384> gps_ring_buffer;

CentripetalConfig compensator_config;
const Vector3f lever_arm_m = {0.17f, 0.03f, 0.0f};

#pragma pack(push, 1)
struct TelemetryPacket {
    uint8_t header1 = 0xAA;
    uint8_t header2 = 0x55;

    uint32_t timestamp_ms;

    float q1, q2, q3, q4;
    float altitude;
    float vertical_speed;

    double lat, lon;
    float home_distance;
    bool home_set;

    float speed;
    float course;
    float hdop;
    uint8_t satellites;

    uint8_t checksum;
};
#pragma pack(pop)

struct RawSensorData {
    uint32_t imu_timestamp_us;
    uint32_t baro_timestamp_us;
    uint32_t gps_timestamp_us;

    Vector3f accel;
    Vector3f gyro;
    float pressure;

    double lat, lon;
    float home_distance;
    float gps_altitude;
    float speed;
    float course;
    float hdop;
    uint8_t satellites;
};

struct EKFOutput {
    float q1 = 1.0f, q2 = 0.0f, q3 = 0.0f, q4 = 0.0f;
    float vertical_speed = 0.0f;
    float altitude = 0.0f;
};

double home_lat = 0.0, home_lon = 0.0;
float home_altitude = 0.0f;
bool home_set = false;

RawSensorData global_sensor_data;
spin_lock_t *sensor_lock;

EKFOutput global_ekf_output;
spin_lock_t *ekf_lock;

volatile bool imu_data_ready = false;
uint32_t imu_irq_timestamp_us = 0;
volatile uint32_t core1_heartbeat_us = 0;

/* PPM */
volatile bool sync_stop = true;
volatile uint8_t pulses_nr = 0;
volatile uint16_t pulses[N_CHANNELS] = {1500, 1500, 1040, 1500, 1500, 1500}; // roll, pitch, throttle, yaw, CH5, CH6
volatile uint16_t ppm_capture[N_CHANNELS] = {};
volatile bool ppm_frame_valid = false;
volatile uint32_t last_valid_ppm_us = 0;
volatile uint32_t interruptTime, previousTime = 0;
/* === */

static TelemetryPacket tx_packet_buffer;

constexpr double DEG_TO_RAD = M_PI / 180.0;

float map(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint8_t calculate_checksum(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; ++i)
    {
        crc ^= data[i];
    }
    return crc;
}

void send_telemetry_dma(const RawSensorData &sensors, const EKFOutput &ekf) {
    // Reuse the packet buffer only when the previous UART DMA transfer is done.
    if (dma_channel_is_busy(telemetry_dma_chan)) {
        return;
    }

    uint32_t ts = time_us_32() / 1000;
    memcpy(&tx_packet_buffer.timestamp_ms, &ts, sizeof(ts));
    
    memcpy(&tx_packet_buffer.q1, &ekf.q1, sizeof(float));
    memcpy(&tx_packet_buffer.q2, &ekf.q2, sizeof(float));
    memcpy(&tx_packet_buffer.q3, &ekf.q3, sizeof(float));
    memcpy(&tx_packet_buffer.q4, &ekf.q4, sizeof(float));
    
    memcpy(&tx_packet_buffer.altitude, &ekf.altitude, sizeof(float));
    memcpy(&tx_packet_buffer.vertical_speed, &ekf.vertical_speed, sizeof(float));
    
    memcpy(&tx_packet_buffer.speed, &sensors.speed, sizeof(float));
    memcpy(&tx_packet_buffer.course, &sensors.course, sizeof(float));
    memcpy(&tx_packet_buffer.hdop, &sensors.hdop, sizeof(float));
    
    tx_packet_buffer.satellites = sensors.satellites;
    tx_packet_buffer.home_set = home_set;

    memcpy(&tx_packet_buffer.lat, &sensors.lat, sizeof(double));
    memcpy(&tx_packet_buffer.lon, &sensors.lon, sizeof(double));
    memcpy(&tx_packet_buffer.home_distance, &sensors.home_distance, sizeof(float));

    uint8_t *ptr = reinterpret_cast<uint8_t *>(&tx_packet_buffer);
    size_t payload_len = sizeof(TelemetryPacket) - 3;
    tx_packet_buffer.checksum = calculate_checksum(ptr + 2, payload_len);

    asm volatile("" ::: "memory");

    dma_channel_set_read_addr(telemetry_dma_chan, &tx_packet_buffer, false);
    dma_channel_set_trans_count(telemetry_dma_chan, sizeof(TelemetryPacket), true);
}

void init_telemetry_dma() {
    telemetry_dma_chan = dma_claim_unused_channel(true);

    dma_channel_config c = dma_channel_get_default_config(telemetry_dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, uart_get_dreq(UART, true));

    dma_channel_configure(
        telemetry_dma_chan,
        &c,
        &uart_get_hw(UART)->dr,
        NULL,
        0,
        false
    );
}

void send_sensors_to_core1(const RawSensorData &data) {
    // Protect the shared sensor snapshot while handing it to the estimator core.
    uint32_t save = spin_lock_blocking(sensor_lock);
    global_sensor_data = data;
    spin_unlock(sensor_lock, save);
}

RawSensorData get_sensors_on_core1() {
    // Copy the latest sensor snapshot without exposing shared state to the caller.
    RawSensorData local_copy;
    uint32_t save = spin_lock_blocking(sensor_lock);
    local_copy = global_sensor_data;
    spin_unlock(sensor_lock, save);
    return local_copy;
}

void send_ekf_to_core0(const EKFOutput &ekf) {
    // Publish a complete estimator result to the control core under the lock.
    uint32_t save = spin_lock_blocking(ekf_lock);
    global_ekf_output = ekf;
    spin_unlock(ekf_lock, save);
}

EKFOutput get_ekf_on_core0() {
    // Return a consistent copy of the estimator result shared by the other core.
    EKFOutput local_copy;
    uint32_t save = spin_lock_blocking(ekf_lock);
    local_copy = global_ekf_output;
    spin_unlock(ekf_lock, save);
    return local_copy;
}

static inline uint16_t servo_to_pwm(float val) {
    if (!std::isfinite(val)) return 1500;
    val = std::clamp(val, -1.0f, 1.0f);
    return static_cast<uint16_t>(1500.0f + val * 500.0f);
}

static inline uint16_t throttle_to_pwm(float val) {
    if (!std::isfinite(val)) return 1040;
    val = std::clamp(val, 0.0f, 1.0f);
    return static_cast<uint16_t>(1040.0f + val * 960.0f);
}

float get_yaw_from_quaternion(float q1, float q2, float q3, float q4) {
    float siny_cosp = 2.0f * (q1 * q4 + q2 * q3);
    float cosy_cosp = 1.0f - 2.0f * (q3 * q3 + q4 * q4);
    return std::atan2(siny_cosp, cosy_cosp);
}

#ifdef PICOPILOT_ENABLE_DIAGNOSTICS
void print_sensor_diagnostics(const RawSensorData& sensors, const EKFOutput& ekf,
                              uint32_t now_us, bool signal_ok) {
    if (!stdio_usb_connected()) return;

    uint32_t imu_age_ms = (now_us - sensors.imu_timestamp_us) / 1000;
    uint32_t baro_age_ms = (now_us - sensors.baro_timestamp_us) / 1000;
    uint32_t gps_age_ms = (now_us - sensors.gps_timestamp_us) / 1000;

    printf("\n--- Sensor status ---\n");
    printf("IMU age=%lu ms accel[m/s^2]=%.2f %.2f %.2f gyro[rad/s]=%.3f %.3f %.3f temp=%.1f C\n",
           static_cast<unsigned long>(imu_age_ms),
           imu.ax, imu.ay, imu.az, imu.gx, imu.gy, imu.gz, imu.temp);
    printf("BME280 age=%lu ms pressure=%.1f Pa temp=%.2f C baro_alt=%.1f m\n",
           static_cast<unsigned long>(baro_age_ms),
           sensors.pressure, bme.get_temperature(),
           Variometer::pressure_to_altitude(sensors.pressure));
    printf("GPS age=%lu ms fix=%d sats=%u hdop=%.2f lat=%.6f lon=%.6f alt=%.1f m speed=%.2f m/s course=%.1f deg\n",
           static_cast<unsigned long>(gps_age_ms), gps.fix_state,
           static_cast<unsigned>(sensors.satellites), sensors.hdop,
           sensors.lat, sensors.lon, sensors.gps_altitude, sensors.speed,
           sensors.course * (180.0f / 3.14159265358979323846f));
    printf("EKF q=%.4f %.4f %.4f %.4f altitude=%.1f m vertical_speed=%.2f m/s\n",
           ekf.q1, ekf.q2, ekf.q3, ekf.q4, ekf.altitude, ekf.vertical_speed);
    printf("PPM valid=%s channels[roll,pitch,throttle,yaw,mode,speed]=%u %u %u %u %u %u us\n",
           signal_ok ? "yes" : "no",
           static_cast<unsigned>(pulses[0]), static_cast<unsigned>(pulses[1]),
           static_cast<unsigned>(pulses[2]), static_cast<unsigned>(pulses[3]),
           static_cast<unsigned>(pulses[4]), static_cast<unsigned>(pulses[5]));
}

void print_route(const RouteStorage::Route* route) {
    printf("\n--- Route: %u waypoints ---\n", route->count);

    for (uint16_t i = 0; i < route->count; ++i) {
        const RouteStorage::Waypoint* p = &route->points[i];

        printf(
            "\nWaypoint %u:\n"
            "  Latitude:  %.7f\n"
            "  Longitude: %.7f\n"
            "  Altitude:  %.2f m\n"
            "  Speed:     %.2f\n",
            (unsigned)(i + 1),
            p->latitude,
            p->longitude,
            (double)p->altitude,
            (double)p->speed
        );

        if (i + 1 < route->count) {
            const RouteStorage::Waypoint* next = &route->points[i + 1];

            float distance = calculate_distance(
                p->latitude, p->longitude,
                next->latitude, next->longitude
            );

            float bearing = calculate_bearing(
                p->latitude, p->longitude,
                next->latitude, next->longitude
            );

            printf(
                "  Distance to next: %.2f m\n"
                "  Required course:   %.2f degrees\n",
                (double)distance,
                (double)bearing
            );
        } else {
            printf("  End of route.\n");
        }
    }

    printf("\n--- End of route ---\n");
}
#endif

void ppm_handler() {
    previousTime = interruptTime;
    interruptTime = time_us_32();
    uint32_t pulseLength = interruptTime - previousTime;

    // A long pause marks a frame boundary; publish only complete, in-range frames.
    if (pulseLength > PAUSE_PULSE_MIN_LENGTH && pulseLength < PAUSE_PULSE_MAX_LENGTH) {
        if (pulses_nr == N_CHANNELS) {
            bool valid_frame = true;
            for (uint8_t i = 0; i < N_CHANNELS; ++i) {
                if (ppm_capture[i] < 900 || ppm_capture[i] > 2200) {
                    valid_frame = false;
                    break;
                }
            }
            if (valid_frame) {
                for (uint8_t i = 0; i < N_CHANNELS; ++i) {
                    pulses[i] = ppm_capture[i];
                }
                ppm_frame_valid = true;
                last_valid_ppm_us = interruptTime;
            }
        }
        pulses_nr = 0;
    }
    else {
        if (pulses_nr < N_CHANNELS) {
            if (pulseLength >= 900 && pulseLength <= 2200) {
                ppm_capture[pulses_nr] = static_cast<uint16_t>(pulseLength);
            } else {
                ppm_capture[pulses_nr] = 0;
            }
            pulses_nr++;
        }
    }
}

void gps_handler() {
    uart_get_hw(UART)->icr = 0x7FF;

    // Keep the interrupt short by queuing bytes for parsing in the main loop.
    while (uart_is_readable(UART)) {
        char c = uart_getc(UART);
        gps_ring_buffer.push(c);
    }
}

void gpio_callback(uint gpio, uint32_t events) {
    if (gpio == PPM_PIN) {
        ppm_handler();
    }
    else if (gpio == IMU_INT_PIN) {
        imu_data_ready = true;
        imu_irq_timestamp_us = time_us_32();
    }
}

void set_pwm(uint8_t gp, uint16_t duty_us) {
    pwm_set_gpio_level(gp, duty_us);
}

void init_pwm(uint8_t gp, uint16_t start_duty_ns) {
    gpio_init(gp);
    gpio_set_function(gp, GPIO_FUNC_PWM);

    uint slice_num = pwm_gpio_to_slice_num(gp);

    uint32_t clk = clock_get_hz(clk_sys);
    float div = clk / 20000.0f / 50.0f;

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, div);
    pwm_config_set_wrap(&config, 20000);

    pwm_init(slice_num, &config, false);
    pwm_set_enabled(slice_num, true);
    pwm_set_gpio_level(gp, start_duty_ns);
}

void hardware_init() {
    // Set up communication buses, PWM outputs, sensors, interrupts, and shared locks.
    /* INIT UART */
    gpio_set_function(UART_RX, GPIO_FUNC_UART);
    gpio_set_function(UART_TX, GPIO_FUNC_UART);
    uart_init(UART, UART_BAUDRATE);

    irq_set_exclusive_handler(UART0_IRQ, gps_handler);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irqs_enabled(UART, true, false);

    /* INIT I2C */
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    /* INITIALIZING PWM */
    init_pwm(THROTTLE_PIN, 1040);
    init_pwm(ROLL_PIN, 1500);
    init_pwm(PITCH_PIN, 1500);
    init_pwm(YAW_PIN, 1500);

    /* INITIALIZING ADC */
    adc_init();
    adc_gpio_init(26);
    adc_gpio_init(27);
    adc_gpio_init(28);

    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);

    if (!imu.begin() || !bme.init())
    {
        gpio_put(25, 1);
        while (1)
            tight_loop_contents();
    }

    gpio_init(PPM_PIN);
    gpio_set_dir(PPM_PIN, GPIO_IN);
    gpio_pull_down(PPM_PIN);
    gpio_set_irq_enabled(PPM_PIN, GPIO_IRQ_EDGE_RISE, true);

    gpio_init(IMU_INT_PIN);
    gpio_set_dir(IMU_INT_PIN, GPIO_IN);
    gpio_pull_down(IMU_INT_PIN);
    gpio_set_irq_enabled(IMU_INT_PIN, GPIO_IRQ_EDGE_RISE, true);

    gpio_set_irq_callback(&gpio_callback);
    irq_set_enabled(IO_IRQ_BANK0, true);

    init_telemetry_dma();

    sensor_lock = spin_lock_init(spin_lock_claim_unused(true));
    ekf_lock = spin_lock_init(spin_lock_claim_unused(true));

}

void core1_entry() {
    if (!flash_safe_execute_core_init()) {
        panic("Failed to initialize flash-safe execution on core 1");
    }

    static ExtendedKalmanFilter ekf;
    static Variometer variometer;
    static CentripetalCompensator compensator(compensator_config);

    RawSensorData sensors;
    EKFOutput ekf_result;
    Vector3f gyro_clean, accel_corr;
    float adapted_r;
    core1_heartbeat_us = time_us_32();

    uint32_t last_imu_timestamp_us = 0;
    uint32_t last_baro_timestamp_us = 0;
    uint32_t last_gps_timestamp_us = 0;
    uint32_t loop_counter = 0;

    while (true) {
        multicore_fifo_pop_blocking();
        multicore_fifo_drain();
        sensors = get_sensors_on_core1();

        if (last_imu_timestamp_us == 0) {
            last_imu_timestamp_us = sensors.imu_timestamp_us;
            continue;
        }

        float dt = (float)(sensors.imu_timestamp_us - last_imu_timestamp_us) / 1000000.0f;
        last_imu_timestamp_us = sensors.imu_timestamp_us;

        if (dt <= 0.0f || dt > 0.02f) {
            dt = 0.0025f;
        }

        float bx, by, bz;
        ekf.getGyroBias(bx, by, bz);
        // Remove estimated gyro bias before correcting rotational acceleration.
        gyro_clean = {
            sensors.gyro.x - bx,
            sensors.gyro.y - by,
            sensors.gyro.z - bz
        };

        compensator.process(
            sensors.accel,
            gyro_clean,
            dt,
            sensors.speed,
            accel_corr,
            adapted_r
        );

        // Adapt accelerometer trust, correct yaw drift, then predict attitude from gyro.
        ekf.set_r_accel(adapted_r);
        ekf.correctYawDrift(sensors.course * DEG_TO_RAD, sensors.speed, dt);

        ekf.predict(gyro_clean.x, gyro_clean.y, gyro_clean.z, dt);

        if (loop_counter % 2 == 0) {
            ekf.updateAccel(accel_corr.x, accel_corr.y, accel_corr.z);
        }

        // Fuse each new barometer and GPS altitude sample into the variometer.
        if (sensors.baro_timestamp_us != last_baro_timestamp_us) {
            float dt_baro = (float)(sensors.baro_timestamp_us - last_baro_timestamp_us) / 1000000.0f;
            last_baro_timestamp_us = sensors.baro_timestamp_us;

            if (dt_baro > 0.0f && dt_baro < 0.5f) {
                float raw_altitude = Variometer::pressure_to_altitude(sensors.pressure);
                variometer.update_baro(raw_altitude, dt_baro);
            }
        }

        if (sensors.gps_timestamp_us != last_gps_timestamp_us) {
            float dt_gps = (float)(sensors.gps_timestamp_us - last_gps_timestamp_us) / 1000000.0f;
            last_gps_timestamp_us = sensors.gps_timestamp_us;

            if (sensors.hdop < 2.0f && dt_gps > 0.0f && dt_gps < 2.0f) {
                variometer.update_gps(sensors.gps_altitude, dt_gps, sensors.hdop * 2.0f);
            }
        }

        ekf.getQuaternion(ekf_result.q1, ekf_result.q2, ekf_result.q3, ekf_result.q4);
        ekf_result.altitude = variometer.get_altitude();
        ekf_result.vertical_speed = variometer.get_vertical_speed();

        loop_counter++;
        send_ekf_to_core0(ekf_result);
        core1_heartbeat_us = time_us_32();
    }
}

void core0_entry() {
    RawSensorData current_sensors = {};
    EKFOutput ekf = {};

    Avionics::FlightController flight_controller;
    Avionics::TECSController tecs_controller;

    RouteStorage::Route route {};
    Avionics::Router router;

    const bool route_loaded = route_loader.load(route);
    const bool route_ready = route_loaded && router.set_route(route);

    // Disable route mode at startup if flash contains no valid route.
    if (AUTOPILOT_MODE_ROUTE && !route_ready) {
        AUTOPILOT_MODE_ROUTE = false;
    }
#ifdef PICOPILOT_ENABLE_DIAGNOSTICS
    if (AUTOPILOT_MODE_ROUTE && !route_ready) {
        printf("Route autopilot requested, but no valid route is loaded.\n");
    }
    if (route_loaded) print_route(&route);
#endif

    uint32_t last_bme_read_us = 0;
    const uint32_t BME_READ_INTERVAL_US = 20000;

    uint32_t last_telemetry_tx_us = 0;
    const uint32_t TELEMETRY_INTERVAL_US = 200000;

    uint32_t last_control_time_us = time_us_32();
    constexpr uint32_t CONTROL_INTERVAL_US = 2500;
    constexpr uint32_t MAX_IMU_AGE_US = 50000;
    constexpr uint32_t CORE1_STALE_TIMEOUT_US = 100000;
    constexpr uint32_t MAX_GPS_AGE_US = 1500000;
    constexpr float RTH_ALTITUDE_CLEARANCE_M = 30.0f;
    constexpr float RTH_ORBIT_RADIUS_M = 60.0f;
    constexpr float RTH_DEFAULT_SPEED_M_S = 15.0f;

#ifdef PICOPILOT_ENABLE_DIAGNOSTICS
    constexpr uint32_t DIAGNOSTICS_INTERVAL_US = 1000000;
    uint32_t last_diagnostics_us = last_control_time_us;
#endif

    float target_altitude_m = 0.0f;
    bool altitude_hold_captured = false;
    float target_course_rad = 0.0f;
    bool course_hold_captured = false;
    float last_speed_target_m_s = RTH_DEFAULT_SPEED_M_S;
    float rth_altitude_m = 0.0f;
    bool rth_altitude_captured = false;
    bool rth_active = false;
    float v_dot_filtered = 0.0f;

    while (true) {
        uint32_t now_us = time_us_32();
        // Treat receiver and estimator data as usable only while their heartbeats are fresh.
        bool signal_ok = ppm_frame_valid &&
                         (now_us - last_valid_ppm_us) < MAX_NO_SIGNAL_TIME;
        bool core1_healthy = (now_us - core1_heartbeat_us) < CORE1_STALE_TIMEOUT_US;
        if (core1_healthy) {
            watchdog_update();
        }

        char c;
        // Parse queued GPS bytes and save the first suitably accurate fix as Home.
        while (gps_ring_buffer.pop(c)) {
            if (gps.update(c)) {
                if (!home_set && gps.fix_state > 0 && gps.satellites >= 4 &&
                    gps.hdop > 0.0f && gps.hdop < 3.5f &&
                    std::isfinite(gps.latitude) && gps.latitude >= -90.0 && gps.latitude <= 90.0 &&
                    std::isfinite(gps.longitude) && gps.longitude >= -180.0 && gps.longitude <= 180.0 &&
                    std::isfinite(gps.altitude)) {
                    home_lat = gps.latitude;
                    home_lon = gps.longitude;
                    home_altitude = gps.altitude;
                    home_set = true;
                }

                current_sensors.gps_timestamp_us = time_us_32();
                current_sensors.gps_altitude = gps.altitude;
                current_sensors.speed = gps.speed;
                current_sensors.course = gps.course;
                current_sensors.hdop = gps.hdop;
                current_sensors.satellites = gps.satellites;
                current_sensors.lat = gps.latitude;
                current_sensors.lon = gps.longitude;

                float home_distance = 0.0f;
                if (home_set) home_distance = calculate_distance(home_lat, home_lon, gps.latitude, gps.longitude);
                current_sensors.home_distance = home_distance;
            }
        }

        if (now_us - last_bme_read_us >= BME_READ_INTERVAL_US) {
            last_bme_read_us = now_us;
            if (bme.update_pressure()) {
                current_sensors.baro_timestamp_us = now_us;
                current_sensors.pressure = bme.get_pressure();
            }
        }

        if (imu_data_ready) {
            imu_data_ready = false;
            if (imu.read_data()) {
                current_sensors.accel = {imu.ax, imu.ay, imu.az};
                current_sensors.gyro = {imu.gx, imu.gy, imu.gz};
                current_sensors.imu_timestamp_us = imu_irq_timestamp_us;

                send_sensors_to_core1(current_sensors);
                if (multicore_fifo_wready())
                    multicore_fifo_push_blocking(1);
            }
        }

        if (now_us - last_control_time_us >= CONTROL_INTERVAL_US) {
            float dt_ctrl = static_cast<float>(now_us - last_control_time_us) / 1000000.0f;
            last_control_time_us = now_us;
            if (dt_ctrl <= 0.0f || dt_ctrl > 0.05f) dt_ctrl = 0.0025f;

            ekf = get_ekf_on_core0();
            bool imu_ok = current_sensors.imu_timestamp_us != 0 &&
                          (now_us - current_sensors.imu_timestamp_us) < MAX_IMU_AGE_US;
            bool gps_ok = current_sensors.gps_timestamp_us != 0 &&
                          (now_us - current_sensors.gps_timestamp_us) < MAX_GPS_AGE_US &&
                          current_sensors.satellites >= 4 &&
                          current_sensors.hdop > 0.0f &&
                          current_sensors.hdop < 7.0f &&
                          std::isfinite(current_sensors.lat) &&
                          std::isfinite(current_sensors.lon) &&
                          std::isfinite(current_sensors.speed) &&
                          std::isfinite(current_sensors.course);
            const bool rth_requested = !signal_ok;

            // Capture one RTH altitude target on entry and clear it after signal recovery.
            if (rth_requested && home_set && !rth_altitude_captured) {
                rth_altitude_m = std::max(
                    ekf.altitude,
                    home_altitude + RTH_ALTITUDE_CLEARANCE_M
                );
                rth_altitude_captured = true;
            } else if (!rth_requested) {
                rth_altitude_captured = false;
            } // Return to home altitude capture

            if (!rth_requested && rth_active) {
                altitude_hold_captured = false;
                course_hold_captured = false;
            } // Capture canceling when rth is not longer needed.
            rth_active = rth_requested;

            if (signal_ok && pulses[5] >= 1200) {
                last_speed_target_m_s = std::clamp(
                    map(pulses[5], 900.0f, 2200.0f, 0.0f, 27.0f),
                    0.0f,
                    27.0f
                );
            } // Speed target if manual mode

            uint16_t outputs[4];

            // Apply safety fallbacks before selecting direct control or an autopilot mode.
            if (rth_requested && (!home_set || !gps_ok)) {
                // RTH is not possible without a saved home position and fresh GPS.
                outputs[0] = 1040;
                outputs[1] = 1500;
                outputs[2] = 1500;
                outputs[3] = 1500;
            } else if (signal_ok && pulses[4] < 1500) {
                // Direct control
                outputs[0] = pulses[2];
                outputs[1] = pulses[0];
                outputs[2] = pulses[1];
                outputs[3] = pulses[3];
                altitude_hold_captured = false;
                course_hold_captured = false;
            } else if (!core1_healthy || !imu_ok) {
                // Critical Failure
                outputs[0] = 1040;
                outputs[1] = 1500;
                outputs[2] = 1500;
                outputs[3] = 1500;
                altitude_hold_captured = false;
                course_hold_captured = false;
            } else if (AUTOPILOT_MODE_ROUTE && !rth_requested && (!route_ready || !gps_ok)) {
                // Route mode requires an active route and fresh GPS.
                outputs[0] = 1040;
                outputs[1] = 1500;
                outputs[2] = 1500;
                outputs[3] = 1500;
                altitude_hold_captured = false;
                course_hold_captured = false;
            } else {
                Avionics::Quaternion q_curr = {ekf.q1, ekf.q2, ekf.q3, ekf.q4};
                Avionics::Vector3 gyro_rad_s = {-imu.gx, -imu.gy, imu.gz};
                float yaw_curr_rad = get_yaw_from_quaternion(ekf.q1, ekf.q2, ekf.q3, ekf.q4);

                if (!rth_requested && !AUTOPILOT_MODE_ROUTE && !course_hold_captured) {
                    target_course_rad = gps_ok && current_sensors.speed >= 1.0f
                        ? current_sensors.course * static_cast<float>(DEG_TO_RAD)
                        : yaw_curr_rad;
                    course_hold_captured = true;
                } // Following the current heading in manual mode

                if (!rth_requested && !AUTOPILOT_MODE_ROUTE && !altitude_hold_captured) {
                    target_altitude_m = ekf.altitude;
                    altitude_hold_captured = true;
                } // Holding the current altitude in manual mode

                // Estimate forward acceleration in the earth frame and smooth it for TECS.
                float sin_pitch = 2.0f * (ekf.q1 * ekf.q3 - ekf.q2 * ekf.q4);
                float v_dot_imu = imu.ax - 9.80665f * sin_pitch;
                v_dot_filtered += 0.1f * (v_dot_imu - v_dot_filtered);

                Avionics::TECSInputs tecs_in;
                tecs_in.h_set = target_altitude_m;
                tecs_in.h_curr = ekf.altitude;
                tecs_in.h_dot_curr = ekf.vertical_speed;
                tecs_in.v_set = last_speed_target_m_s;
                tecs_in.v_curr = gps_ok
                    ? std::max(current_sensors.speed, 7.0f)
                    : 15.0f;
                tecs_in.v_dot_curr = v_dot_filtered;

                float desired_course_rad = target_course_rad;
                bool use_tecs_throttle = rth_requested;
                if (rth_requested) {
                    // Navigate home outside the orbit radius, then follow a circular hold.
                    tecs_in.h_set = rth_altitude_m;
                    if (current_sensors.home_distance > 2.0f * RTH_ORBIT_RADIUS_M) {
                        desired_course_rad = calculate_bearing(
                            current_sensors.lat,
                            current_sensors.lon,
                            home_lat,
                            home_lon
                        );
                    } else {
                        desired_course_rad = calculate_orbit_bearing(
                            current_sensors.lat,
                            current_sensors.lon,
                            home_lat,
                            home_lon,
                            RTH_ORBIT_RADIUS_M,
                            true
                        );
                    }
                } else if (AUTOPILOT_MODE_ROUTE) {
                    // Use route waypoint targets when available, otherwise hold GPS course.
                    Avionics::RouterInputs nav_in {
                        current_sensors.lat,
                        current_sensors.lon
                    };
                    Avionics::RouterOutputs nav_out {};
                    router.update(nav_in, nav_out);

                    tecs_in.h_set = nav_out.altitude;
                    tecs_in.v_set = nav_out.speed;
                    if (nav_out.route_active) {
                        desired_course_rad = nav_out.bearing;
                    } else {
                        desired_course_rad = current_sensors.course *
                                             static_cast<float>(DEG_TO_RAD);
                    }
                    use_tecs_throttle = true;
                }

                const float measured_course_rad =
                    gps_ok && current_sensors.speed >= 1.0f
                        ? current_sensors.course * static_cast<float>(DEG_TO_RAD)
                        : yaw_curr_rad;
                flight_controller.updateGPSCourse(
                    desired_course_rad,
                    measured_course_rad,
                    gps_ok ? current_sensors.speed : 0.0f
                );

                // Convert navigation targets and current estimates into actuator commands.
                Avionics::TECSOutputs tecs_out = tecs_controller.update(tecs_in, dt_ctrl);
                Avionics::ServoOutputs servos = flight_controller.updateAttitude(
                    dt_ctrl,
                    q_curr,
                    gyro_rad_s,
                    imu.ay,
                    yaw_curr_rad,
                    tecs_out
                );

                if (!use_tecs_throttle && pulses[5] < 1200) outputs[0] = pulses[2];
                else outputs[0] = throttle_to_pwm(servos.throttle);

                outputs[1] = servo_to_pwm(-servos.aileron);
                outputs[2] = servo_to_pwm(servos.elevator);
                outputs[3] = servo_to_pwm(-servos.rudder);
            }
            set_pwm(THROTTLE_PIN, outputs[0]);
            set_pwm(ROLL_PIN, outputs[1]);
            set_pwm(PITCH_PIN, outputs[2]);
            set_pwm(YAW_PIN, outputs[3]);
        }

        if (now_us - last_telemetry_tx_us >= TELEMETRY_INTERVAL_US) {
            last_telemetry_tx_us = now_us;
            send_telemetry_dma(current_sensors, ekf);
        }

#ifdef PICOPILOT_ENABLE_DIAGNOSTICS
        if (now_us - last_diagnostics_us >= DIAGNOSTICS_INTERVAL_US) {
            last_diagnostics_us = now_us;
            print_sensor_diagnostics(current_sensors, ekf, now_us, signal_ok);
        }
#endif
    }
}

void parse_csv_line(char* line, RouteStorage::Waypoint& waypoint) {
    char* token = strtok(line, ",");
    if (token != NULL) waypoint.latitude = strtod(token, NULL);

    token = strtok(NULL, ",");
    if (token != NULL) waypoint.longitude = strtod(token, NULL);

    token = strtok(NULL, ",");
    if (token != NULL) waypoint.altitude = strtof(token, NULL);

    token = strtok(NULL, ",");
    if (token != NULL) waypoint.speed = strtof(token, NULL);
}


void load_route(){
    char buffer[MAX_LINE_LENGTH];
    uint8_t index = 0;
    bool receiving = false;

    RouteStorage::Route route {};

    // Receive START, waypoint CSV lines, and END over USB before saving the route.
    while(true){
        int c = getchar_timeout_us(0);
        if(c == PICO_ERROR_TIMEOUT) continue;

        if(c == '\n' || index >= MAX_LINE_LENGTH - 1){
            buffer[index] = '\0';
            
            if(index > 0){
                if(strcmp(buffer, "START") == 0){
                    route.count = 0;
                    receiving = true;
                    printf("Ready to receive a route.\n");
                } 

                else if(strcmp(buffer, "END") == 0){
                    if(receiving){
                        if(!route_loader.save(route)){
                            printf("Failed to save the route!\n");
                            continue;
                        }
                        printf("Successfully saved the route!\n");
                        break;
                    }
                } 
                else if(receiving && route.count < RouteStorage::MAX_WAYPOINTS){
                    parse_csv_line(buffer, route.points[route.count++]);
                }
            }
            index = 0;
        } else if(c != '\r'){
            buffer[index++] = (char)c;
        }
    }
}


int main() {
    compensator_config.lever_arm_m = lever_arm_m;

    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_PIN);
    
    uint32_t now_us = time_us_32();
    // Sample the route-load button briefly during startup before initializing PWM.
    while(time_us_32() - now_us <= 1000000){
        if(gpio_get(BUTTON_PIN) == 0){
            route_load_mode = true;
            break;
        }
    }

    if(route_load_mode){
        stdio_init_all();
        while (!stdio_usb_connected()) {
            sleep_ms(100);
        }
        printf("USB connected.\n");

        load_route();
    }

#ifdef PICOPILOT_ENABLE_DIAGNOSTICS
    if(!route_load_mode) stdio_init_all();
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
#endif
    hardware_init();

    watchdog_enable(1000, true);

    core1_heartbeat_us = time_us_32();
    multicore_launch_core1(&core1_entry);
    core0_entry();

    return 0;
}

extern "C" void isr_hardfault() {
    sio_hw->gpio_oe_set = (1u << 25);
    while (true) {
        sio_hw->gpio_togl = (1u << 25);
        for (volatile int i = 0; i < 500000; i++)
            asm volatile("nop");
    }
}