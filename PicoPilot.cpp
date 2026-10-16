#include "pico/stdlib.h"
#include <cmath>
#ifdef PICOPILOT_ENABLE_DIAGNOSTICS
#include <cstdio>
#endif
#include <cstring>

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

int telemetry_dma_chan = -1;

MPU6500 imu(I2C_PORT);
BME280 bme(I2C_PORT, 0x76);
Neo6M gps;
RingBuffer<384> gps_ring_buffer;

#ifdef PICOPILOT_ENABLE_DIAGNOSTICS
bool imu_initialized = false;
bool bme_initialized = false;
bool adc_initialized = false;
bool last_imu_read_ok = false;
bool last_bme_read_ok = false;
uint32_t last_bme_success_us = 0;
uint32_t imu_read_success_count = 0;
uint32_t imu_read_failure_count = 0;
uint32_t bme_read_success_count = 0;
uint32_t bme_read_failure_count = 0;
uint32_t gps_parsed_update_count = 0;
uint32_t telemetry_dma_started_count = 0;
uint32_t telemetry_dma_busy_count = 0;
volatile uint32_t imu_irq_count = 0;

volatile uint32_t ppm_edge_count = 0;
volatile uint32_t ppm_valid_frame_count = 0;
volatile uint32_t gps_irq_count = 0;
volatile uint32_t gps_bytes_received = 0;
volatile uint32_t gps_ring_overflow_count = 0;
volatile uint32_t gps_irq_timestamp_us = 0;
volatile uint32_t core1_processed_imu_count = 0;
volatile uint32_t ekf_predict_success_count = 0;
volatile uint32_t ekf_predict_failure_count = 0;
volatile uint32_t ekf_input_reject_count = 0;
volatile uint32_t ekf_accel_update_success_count = 0;
volatile uint32_t ekf_accel_update_failure_count = 0;
volatile uint32_t ekf_baro_update_success_count = 0;
volatile uint32_t ekf_baro_update_failure_count = 0;
volatile uint32_t ekf_gps_altitude_update_success_count = 0;
volatile uint32_t ekf_gps_altitude_update_failure_count = 0;
volatile uint32_t ekf_yaw_correction_count = 0;
uint16_t last_pwm_output[4] = {1040, 1500, 1500, 1500};
const char* control_mode = "STARTING";
bool gps_navigation_valid = false;
float diagnostic_target_course_rad = 0.0f;
Avionics::TECSInputs diagnostic_tecs_inputs = {};
Avionics::TECSOutputs diagnostic_tecs_outputs = {};
Avionics::ServoOutputs diagnostic_servo_outputs = {};
float diagnostic_dt_ctrl = 0.0f;
#define DIAGNOSTIC_COUNT(counter) (++(counter))
#else
#define DIAGNOSTIC_COUNT(counter) ((void)0)
#endif

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
    uint32_t course_timestamp_us;

    Vector3f accel;
    Vector3f gyro;
    float pressure;

    double lat, lon;
    float home_distance;
    float gps_altitude;
    float speed;
    float course;
    bool course_valid;
    bool gps_fix_valid;
    float hdop;
    uint8_t satellites;
};

struct EKFOutput {
    float q1 = 1.0f, q2 = 0.0f, q3 = 0.0f, q4 = 0.0f;
    float gyro_bias_x = 0.0f, gyro_bias_y = 0.0f, gyro_bias_z = 0.0f;
    float vertical_speed = 0.0f;
    float altitude = 0.0f;
    bool attitude_valid = false;
    bool altitude_valid = false;
};

extern double home_lat, home_lon;
extern float home_altitude;
extern bool home_set;
extern volatile uint16_t pulses[N_CHANNELS];
extern volatile uint32_t last_valid_ppm_us;
extern volatile uint32_t core1_heartbeat;
extern volatile uint32_t imu_irq_timestamp_us;
extern volatile bool imu_data_ready;
extern uint16_t signal_power;

#ifdef PICOPILOT_ENABLE_DIAGNOSTICS
void print_diagnostics(const RawSensorData& sensors, const EKFOutput& ekf,
                       bool signal_ok, uint32_t last_valid_imu_us,
                       float target_altitude_m, bool altitude_hold_captured,
                       float v_dot_filtered) {
    const uint32_t now_us = time_us_32();
    const uint32_t imu_age_us = last_valid_imu_us == 0 ? UINT32_MAX : now_us - last_valid_imu_us;
    const uint32_t imu_irq_age_us = imu_irq_timestamp_us == 0 ? UINT32_MAX : now_us - imu_irq_timestamp_us;
    const uint32_t bme_age_us = last_bme_success_us == 0 ? UINT32_MAX : now_us - last_bme_success_us;
    const uint32_t gps_age_us = sensors.gps_timestamp_us == 0 ? UINT32_MAX : now_us - sensors.gps_timestamp_us;
    const uint32_t course_age_us = sensors.course_timestamp_us == 0 ? UINT32_MAX : now_us - sensors.course_timestamp_us;
    const uint32_t ppm_age_us = last_valid_ppm_us == 0 ? UINT32_MAX : now_us - last_valid_ppm_us;
    const uint32_t gps_irq_age_us = gps_irq_timestamp_us == 0 ? UINT32_MAX : now_us - gps_irq_timestamp_us;

    printf("\n=== DIAGNOSTICS t=%lu ms ===\n", static_cast<unsigned long>(now_us / 1000u));
    printf("[IRQ] IMU=%lu age=%lu us pending=%s | GPS=%lu bytes=%lu age=%lu us overflow=%lu parsed=%lu ring_empty=%s"
           " | PPM edges=%lu frames=%lu age=%lu us\n",
           static_cast<unsigned long>(imu_irq_count), static_cast<unsigned long>(imu_irq_age_us),
           imu_data_ready ? "YES" : "NO",
           static_cast<unsigned long>(gps_irq_count), static_cast<unsigned long>(gps_bytes_received),
           static_cast<unsigned long>(gps_irq_age_us), static_cast<unsigned long>(gps_ring_overflow_count),
           static_cast<unsigned long>(gps_parsed_update_count), gps_ring_buffer.empty() ? "YES" : "NO",
           static_cast<unsigned long>(ppm_edge_count),
           static_cast<unsigned long>(ppm_valid_frame_count), static_cast<unsigned long>(ppm_age_us));

    printf("[MPU6500] init=%s read=%s ok=%lu fail=%lu age=%lu us"
           " accel=(%.3f, %.3f, %.3f) m/s^2 gyro=(%.5f, %.5f, %.5f) rad/s temp=%.2f C\n",
           imu_initialized ? "OK" : "FAIL", last_imu_read_ok ? "OK" : "FAIL",
           static_cast<unsigned long>(imu_read_success_count), static_cast<unsigned long>(imu_read_failure_count),
           static_cast<unsigned long>(imu_age_us), imu.ax, imu.ay, imu.az, imu.gx, imu.gy, imu.gz, imu.temp);

    printf("[BME280] init=%s read=%s ok=%lu fail=%lu age=%lu us"
           " temp=%.2f C pressure=%.2f Pa humidity=%.2f %%\n",
           bme_initialized ? "OK" : "FAIL", last_bme_read_ok ? "OK" : "FAIL",
           static_cast<unsigned long>(bme_read_success_count), static_cast<unsigned long>(bme_read_failure_count),
           static_cast<unsigned long>(bme_age_us), bme.get_temperature(), bme.get_pressure(), bme.get_humidity());

    printf("[GPS] fix_state=%d fix_valid=%s satellites=%u HDOP=%.2f"
           " lat=%.7f lon=%.7f altitude=%.2f m speed=%.2f m/s course=%.5f rad (%s, age=%lu us)"
           " fix_age=%lu us"
           " time=%u date=%u home=%s (%.7f, %.7f, %.2f m) distance=%.2f m\n",
           gps.fix_state, sensors.gps_fix_valid ? "YES" : "NO", static_cast<unsigned>(gps.satellites), gps.hdop,
           gps.latitude, gps.longitude, gps.altitude, gps.speed, gps.course,
           gps.course_valid ? "VALID" : "INVALID", static_cast<unsigned long>(course_age_us),
           static_cast<unsigned long>(gps_age_us),
           gps.gps_time, gps.gps_date, home_set ? "SET" : "UNSET",
           home_lat, home_lon, home_altitude, sensors.home_distance);

    printf("[FUSION] core1 heartbeat=%lu processed_imu=%lu input_reject=%lu predict(ok=%lu fail=%lu)"
           " accel_update(ok=%lu reject=%lu) baro_update(ok=%lu reject=%lu)"
           " gps_alt_update(ok=%lu reject=%lu) yaw_correction=%lu\n",
           static_cast<unsigned long>(core1_heartbeat), static_cast<unsigned long>(core1_processed_imu_count),
           static_cast<unsigned long>(ekf_input_reject_count),
           static_cast<unsigned long>(ekf_predict_success_count), static_cast<unsigned long>(ekf_predict_failure_count),
           static_cast<unsigned long>(ekf_accel_update_success_count), static_cast<unsigned long>(ekf_accel_update_failure_count),
           static_cast<unsigned long>(ekf_baro_update_success_count), static_cast<unsigned long>(ekf_baro_update_failure_count),
           static_cast<unsigned long>(ekf_gps_altitude_update_success_count),
           static_cast<unsigned long>(ekf_gps_altitude_update_failure_count),
           static_cast<unsigned long>(ekf_yaw_correction_count));
    printf("[EKF/ALT] attitude=%s altitude=%s q=(%.5f, %.5f, %.5f, %.5f)"
           " gyro_bias=(%.6f, %.6f, %.6f) rad/s"
           " altitude=%.2f m vertical_speed=%.2f m/s\n",
           ekf.attitude_valid ? "VALID" : "INVALID", ekf.altitude_valid ? "VALID" : "INVALID",
           ekf.q1, ekf.q2, ekf.q3, ekf.q4,
           ekf.gyro_bias_x, ekf.gyro_bias_y, ekf.gyro_bias_z, ekf.altitude, ekf.vertical_speed);

    printf("[CONTROL] mode=%s signal=%s signal_power=%u threshold=%u PPM_CH=(%u,%u,%u,%u,%u,%u)"
           " dt=%.5f s altitude_target=%.2f m altitude_hold=%s vertical_accel=%.3f m/s^2\n",
           control_mode, signal_ok ? "OK" : "LOST", static_cast<unsigned>(signal_power),
           static_cast<unsigned>(SIGNAL_POWER_THRESHOLD),
           static_cast<unsigned>(pulses[0]), static_cast<unsigned>(pulses[1]),
           static_cast<unsigned>(pulses[2]), static_cast<unsigned>(pulses[3]),
           static_cast<unsigned>(pulses[4]), static_cast<unsigned>(pulses[5]),
           diagnostic_dt_ctrl, target_altitude_m, altitude_hold_captured ? "YES" : "NO", v_dot_filtered);
    printf("[TECS/NAV] h_set=%.2f h=%.2f h_dot=%.2f v_set=%.2f v=%.2f v_dot=%.2f"
           " throttle=%.3f pitch=%.4f rad GPS_nav=%s target_course=%.5f rad"
           " servos=(aileron=%.3f elevator=%.3f rudder=%.3f throttle=%.3f)\n",
           diagnostic_tecs_inputs.h_set, diagnostic_tecs_inputs.h_curr, diagnostic_tecs_inputs.h_dot_curr,
           diagnostic_tecs_inputs.v_set, diagnostic_tecs_inputs.v_curr, diagnostic_tecs_inputs.v_dot_curr,
           diagnostic_tecs_outputs.throttle, diagnostic_tecs_outputs.pitch_des_rad,
           gps_navigation_valid ? "VALID" : "INVALID", diagnostic_target_course_rad,
           diagnostic_servo_outputs.aileron, diagnostic_servo_outputs.elevator,
           diagnostic_servo_outputs.rudder, diagnostic_servo_outputs.throttle);
    printf("[PWM] throttle=%u roll=%u pitch=%u yaw=%u | [DMA] channel=%d busy=%s started=%lu skipped_busy=%lu\n",
           static_cast<unsigned>(last_pwm_output[0]), static_cast<unsigned>(last_pwm_output[1]),
           static_cast<unsigned>(last_pwm_output[2]), static_cast<unsigned>(last_pwm_output[3]),
           telemetry_dma_chan, telemetry_dma_chan >= 0 && dma_channel_is_busy(telemetry_dma_chan) ? "YES" : "NO",
           static_cast<unsigned long>(telemetry_dma_started_count),
           static_cast<unsigned long>(telemetry_dma_busy_count));
    printf("[HW] UART baud=%u I2C=%u Hz PPM_pin=%u IMU_INT_pin=%u USB_stdio=enabled\n",
           static_cast<unsigned>(UART_BAUDRATE), 400000u,
           static_cast<unsigned>(PPM_PIN), static_cast<unsigned>(IMU_INT_PIN));

    for (uint8_t channel = 0; channel < 3; ++channel) {
        adc_select_input(channel);
        const uint16_t raw = adc_read();
        printf("[ADC] GPIO%u raw=%u voltage=%.3f V\n",
               static_cast<unsigned>(26 + channel), static_cast<unsigned>(raw), raw * (3.3f / 4095.0f));
    }
}
#endif

double home_lat = 0.0, home_lon = 0.0;
float home_altitude = 0.0f;
bool home_set = false;

RawSensorData global_sensor_data;
spin_lock_t *sensor_lock;

EKFOutput global_ekf_output;
spin_lock_t *ekf_lock;

volatile bool imu_data_ready = false;
volatile uint32_t imu_irq_timestamp_us = 0;
volatile uint32_t core1_heartbeat = 0;

/* PPM */
volatile bool sync_stop = true;
volatile uint8_t pulses_nr = 0;
volatile uint16_t pulses[N_CHANNELS] = {1500, 1500, 1040, 1500, 1500, 1500}; // roll, pitch, throttle, yaw, CH5, CH6
volatile uint32_t last_valid_ppm_us = 0;
volatile uint32_t interruptTime, previousTime = 0;

uint16_t signal_power = 1500;
uint32_t signal_timer = 0;
/* === */

static TelemetryPacket tx_packet_buffer;

float map(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float calculate_distance(double lat1, double lon1, double lat2, double lon2) {
    constexpr double EARTH_RADIUS_M = 6371000.88;
    constexpr double DEG_TO_RAD = M_PI / 180.0;

    double dlat = (lat2 - lat1) * DEG_TO_RAD;
    double dlon = (lon2 - lon1) * DEG_TO_RAD;

    double mean_lat = ((lat1 + lat2) / 2.0) * DEG_TO_RAD;

    double x = dlon * std::cos(mean_lat);
    double y = dlat;

    return static_cast<float>(EARTH_RADIUS_M * std::sqrt(x * x + y * y));
}

float calculate_bearing(double lat1, double lon1, double lat2, double lon2) {
    constexpr double DEG_TO_RAD = M_PI / 180.0;

    double phi1 = lat1 * DEG_TO_RAD;
    double phi2 = lat2 * DEG_TO_RAD;
    double dlambda = (lon2 - lon1) * DEG_TO_RAD;

    double y = std::sin(dlambda) * std::cos(phi2);
    double x = std::cos(phi1) * std::sin(phi2) - std::sin(phi1) * std::cos(phi2) * std::cos(dlambda);

    return static_cast<float>(std::atan2(y, x));
}

bool calculate_orbit_course(double current_lat, double current_lon,
                            double center_lat, double center_lon,
                            float radius_m, bool clockwise,
                            float& target_course_rad) {
    constexpr float PI_F = 3.14159265358979323846f;
    constexpr float TWO_PI_F = 2.0f * PI_F;

    if (!std::isfinite(current_lat) || !std::isfinite(current_lon) ||
        !std::isfinite(center_lat) || !std::isfinite(center_lon) ||
        current_lat < -90.0 || current_lat > 90.0 ||
        center_lat < -90.0 || center_lat > 90.0 ||
        current_lon < -180.0 || current_lon > 180.0 ||
        center_lon < -180.0 || center_lon > 180.0 ||
        !std::isfinite(radius_m) || radius_m <= 0.0f) {
        return false;
    }

    const float distance_m = calculate_distance(center_lat, center_lon, current_lat, current_lon);
    if (!std::isfinite(distance_m) || distance_m < 0.1f) {
        return false;
    }

    const float radial_bearing = calculate_bearing(center_lat, center_lon, current_lat, current_lon);
    const float direction = clockwise ? 1.0f : -1.0f;
    const float radial_correction = std::atan2(distance_m - radius_m, radius_m);
    float course = radial_bearing + direction * (PI_F / 2.0f + radial_correction);
    course = std::fmod(course, TWO_PI_F);
    if (course < 0.0f) course += TWO_PI_F;

    target_course_rad = course;
    return true;
}

constexpr float HOME_ORBIT_RADIUS_M = 50.0f;
constexpr bool HOME_ORBIT_CLOCKWISE = true;
constexpr uint32_t GPS_NAVIGATION_TIMEOUT_US = 1500000u;

uint8_t calculate_checksum(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; ++i)
    {
        crc ^= data[i];
    }
    return crc;
}

void send_telemetry_dma(const RawSensorData &sensors, const EKFOutput &ekf) {
    if (dma_channel_is_busy(telemetry_dma_chan)) {
        DIAGNOSTIC_COUNT(telemetry_dma_busy_count);
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
    DIAGNOSTIC_COUNT(telemetry_dma_started_count);
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
    uint32_t save = spin_lock_blocking(sensor_lock);
    global_sensor_data = data;
    spin_unlock(sensor_lock, save);
}

RawSensorData get_sensors_on_core1() {
    RawSensorData local_copy;
    uint32_t save = spin_lock_blocking(sensor_lock);
    local_copy = global_sensor_data;
    spin_unlock(sensor_lock, save);
    return local_copy;
}

void send_ekf_to_core0(const EKFOutput &ekf) {
    uint32_t save = spin_lock_blocking(ekf_lock);
    global_ekf_output = ekf;
    spin_unlock(ekf_lock, save);
}

EKFOutput get_ekf_on_core0() {
    EKFOutput local_copy;
    uint32_t save = spin_lock_blocking(ekf_lock);
    local_copy = global_ekf_output;
    spin_unlock(ekf_lock, save);
    return local_copy;
}

static inline uint16_t servo_to_pwm(float val) {
    val = std::clamp(val, -1.0f, 1.0f);
    return static_cast<uint16_t>(1500.0f + val * 500.0f);
}

static inline uint16_t throttle_to_pwm(float val) {
    val = std::clamp(val, 0.0f, 1.0f);
    return static_cast<uint16_t>(1040.0f + val * 960.0f);
}

float get_yaw_from_quaternion(float q1, float q2, float q3, float q4) {
    float siny_cosp = 2.0f * (q1 * q4 + q2 * q3);
    float cosy_cosp = 1.0f - 2.0f * (q3 * q3 + q4 * q4);
    return std::atan2(siny_cosp, cosy_cosp);
}

void ppm_handler() {
    DIAGNOSTIC_COUNT(ppm_edge_count);
    previousTime = interruptTime;
    interruptTime = time_us_64();
    uint16_t pulseLength = (uint16_t)(interruptTime - previousTime);

    if (pulseLength > PAUSE_PULSE_MIN_LENGTH && pulseLength < PAUSE_PULSE_MAX_LENGTH) {
        if (pulses_nr == N_CHANNELS) {
            last_valid_ppm_us = interruptTime;
            DIAGNOSTIC_COUNT(ppm_valid_frame_count);
        }
        pulses_nr = 0;
    }
    else {
        if (pulses_nr < N_CHANNELS) {
            pulses[pulses_nr] = pulseLength;
            pulses_nr++;
        }
    }
}

void gps_handler()
{
    uart_get_hw(UART)->icr = 0x7FF;
    DIAGNOSTIC_COUNT(gps_irq_count);
#ifdef PICOPILOT_ENABLE_DIAGNOSTICS
    gps_irq_timestamp_us = time_us_32();
#endif

    while (uart_is_readable(UART)) {
        char c = uart_getc(UART);
        DIAGNOSTIC_COUNT(gps_bytes_received);
        if (!gps_ring_buffer.push(c)) {
            DIAGNOSTIC_COUNT(gps_ring_overflow_count);
        }
    }
}

void gpio_callback(uint gpio, uint32_t events) {
    if (gpio == PPM_PIN) {
        ppm_handler();
    }
    else if (gpio == IMU_INT_PIN) {
        imu_data_ready = true;
        imu_irq_timestamp_us = time_us_32();
        DIAGNOSTIC_COUNT(imu_irq_count);
    }
}

void set_pwm(uint8_t gp, uint16_t duty_us) {
    pwm_set_gpio_level(gp, duty_us);
#ifdef PICOPILOT_ENABLE_DIAGNOSTICS
    if (gp == THROTTLE_PIN) last_pwm_output[0] = duty_us;
    else if (gp == ROLL_PIN) last_pwm_output[1] = duty_us;
    else if (gp == PITCH_PIN) last_pwm_output[2] = duty_us;
    else if (gp == YAW_PIN) last_pwm_output[3] = duty_us;
#endif
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
#ifdef PICOPILOT_ENABLE_DIAGNOSTICS
    adc_initialized = true;
#endif

    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);

    const bool imu_ok = imu.begin();
    const bool bme_ok = bme.init();
#ifdef PICOPILOT_ENABLE_DIAGNOSTICS
    imu_initialized = imu_ok;
    bme_initialized = bme_ok;
    printf("[INIT] MPU6500=%s BME280=%s\n",
           imu_initialized ? "OK" : "FAIL", bme_initialized ? "OK" : "FAIL");
#endif
    if (!imu_ok || !bme_ok)
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
    static ExtendedKalmanFilter ekf;
    static Variometer variometer;
    static CentripetalCompensator compensator(compensator_config);

    RawSensorData sensors;
    EKFOutput ekf_result;
    Vector3f gyro_clean, accel_corr;
    float adapted_r;

    uint32_t last_imu_timestamp_us = 0;
    uint32_t last_baro_timestamp_us = 0;
    uint32_t last_gps_timestamp_us = 0;
    uint32_t loop_counter = 0;
    uint32_t last_accel_update_us = 0;
    uint8_t consecutive_accel_updates = 0;
    uint32_t last_altitude_measurement_us = 0;

    while (true) {
        ++core1_heartbeat;
        if (!multicore_fifo_rvalid()) {
            sleep_ms(1);
            continue;
        }
        multicore_fifo_pop_blocking();
        multicore_fifo_drain();
        sensors = get_sensors_on_core1();

        if (last_imu_timestamp_us == 0) {
            last_imu_timestamp_us = sensors.imu_timestamp_us;
            continue;
        }

        float dt = (float)(sensors.imu_timestamp_us - last_imu_timestamp_us) / 1000000.0f;
        last_imu_timestamp_us = sensors.imu_timestamp_us;
        DIAGNOSTIC_COUNT(core1_processed_imu_count);

        const bool imu_dt_valid = std::isfinite(dt) && dt > 0.0f && dt <= 0.02f;
        const bool imu_data_valid =
            std::isfinite(sensors.gyro.x) && std::isfinite(sensors.gyro.y) &&
            std::isfinite(sensors.gyro.z) && std::isfinite(sensors.accel.x) &&
            std::isfinite(sensors.accel.y) && std::isfinite(sensors.accel.z);
        if (!imu_dt_valid || !imu_data_valid) {
            DIAGNOSTIC_COUNT(ekf_input_reject_count);
        }
        bool attitude_update_valid = imu_dt_valid && imu_data_valid;

        if (imu_dt_valid) variometer.predict(dt);

        if (attitude_update_valid) {
            float bx, by, bz;
            ekf.getGyroBias(bx, by, bz);
            gyro_clean = {
                sensors.gyro.x - bx,
                sensors.gyro.y - by,
                sensors.gyro.z - bz};

            compensator.process(
                sensors.accel,
                gyro_clean,
                dt,
                sensors.speed,
                accel_corr,
                adapted_r
            );

            if (sensors.course_valid && sensors.gps_fix_valid &&
                std::isfinite(sensors.speed) && sensors.speed >= 1.0f &&
                sensors.course_timestamp_us != 0 &&
                (uint32_t)(time_us_32() - sensors.course_timestamp_us) <= 1500000u) {
                ekf.correctYawDrift(sensors.course, sensors.speed, dt);
                DIAGNOSTIC_COUNT(ekf_yaw_correction_count);
            }

            attitude_update_valid = ekf.predict(gyro_clean.x, gyro_clean.y, gyro_clean.z, dt);
            if (attitude_update_valid) {
                DIAGNOSTIC_COUNT(ekf_predict_success_count);
            } else {
                DIAGNOSTIC_COUNT(ekf_predict_failure_count);
            }

            if (attitude_update_valid && loop_counter % 2 == 0) {
                if (!ekf.set_r_accel(adapted_r)) {
                    attitude_update_valid = false;
                    DIAGNOSTIC_COUNT(ekf_accel_update_failure_count);
                } else {
                    if (ekf.updateAccel(accel_corr.x, accel_corr.y, accel_corr.z)) {
                        last_accel_update_us = time_us_32();
                        DIAGNOSTIC_COUNT(ekf_accel_update_success_count);
                        if (consecutive_accel_updates < 10) ++consecutive_accel_updates;
                    } else {
                        DIAGNOSTIC_COUNT(ekf_accel_update_failure_count);
                        consecutive_accel_updates = 0;
                    }
                }
            }
        }

        if (sensors.baro_timestamp_us != last_baro_timestamp_us) {
            float dt_baro = (float)(sensors.baro_timestamp_us - last_baro_timestamp_us) / 1000000.0f;
            last_baro_timestamp_us = sensors.baro_timestamp_us;

            if (dt_baro > 0.0f && dt_baro < 0.5f) {
                float raw_altitude;
                if (Variometer::pressure_to_altitude(sensors.pressure, raw_altitude)) {
                    if (variometer.update_baro(raw_altitude)) {
                        last_altitude_measurement_us = time_us_32();
                        DIAGNOSTIC_COUNT(ekf_baro_update_success_count);
                    } else {
                        DIAGNOSTIC_COUNT(ekf_baro_update_failure_count);
                    }
                } else {
                    DIAGNOSTIC_COUNT(ekf_baro_update_failure_count);
                }
            }
        }

        if (sensors.gps_timestamp_us != last_gps_timestamp_us) {
            float dt_gps = (float)(sensors.gps_timestamp_us - last_gps_timestamp_us) / 1000000.0f;
            last_gps_timestamp_us = sensors.gps_timestamp_us;

            if (sensors.gps_fix_valid && std::isfinite(sensors.hdop) &&
                sensors.hdop > 0.0f && sensors.hdop < 2.0f &&
                dt_gps > 0.0f && dt_gps < 2.0f) {
                if (variometer.update_gps(sensors.gps_altitude)) {
                    last_altitude_measurement_us = time_us_32();
                    DIAGNOSTIC_COUNT(ekf_gps_altitude_update_success_count);
                } else {
                    DIAGNOSTIC_COUNT(ekf_gps_altitude_update_failure_count);
                }
            } else {
                DIAGNOSTIC_COUNT(ekf_gps_altitude_update_failure_count);
            }
        }

        ekf.getQuaternion(ekf_result.q1, ekf_result.q2, ekf_result.q3, ekf_result.q4);
        ekf.getGyroBias(ekf_result.gyro_bias_x, ekf_result.gyro_bias_y, ekf_result.gyro_bias_z);
        ekf_result.altitude = variometer.get_altitude();
        ekf_result.vertical_speed = variometer.get_vertical_speed();
        const uint32_t result_timestamp_us = time_us_32();
        ekf_result.attitude_valid =
            attitude_update_valid && consecutive_accel_updates >= 10 &&
            last_accel_update_us != 0 &&
            (uint32_t)(result_timestamp_us - last_accel_update_us) <= 100000u;
        ekf_result.altitude_valid =
            variometer.is_initialized() && last_altitude_measurement_us != 0 &&
            (uint32_t)(result_timestamp_us - last_altitude_measurement_us) <= 250000u;

        loop_counter++;
        send_ekf_to_core0(ekf_result);
    }
}

void core0_entry() {
    RawSensorData current_sensors = {};
    EKFOutput ekf = {};

    Avionics::FlightController flight_controller;
    Avionics::TECSController tecs_controller;

    uint32_t last_bme_read_us = 0;
    const uint32_t BME_READ_INTERVAL_US = 20000;

    uint32_t last_telemetry_tx_us = 0;
    const uint32_t TELEMETRY_INTERVAL_US = 200000;
#ifdef PICOPILOT_ENABLE_DIAGNOSTICS
    uint32_t last_diagnostics_us = 0;
    const uint32_t DIAGNOSTICS_INTERVAL_US = 1000000;
#endif

    uint32_t last_control_time_us = time_us_32();
    uint32_t last_valid_imu_us = last_control_time_us;
    uint32_t last_watchdog_check_us = last_control_time_us;
    uint32_t observed_core1_heartbeat = core1_heartbeat;

    float target_altitude_m = 0.0f;
    bool altitude_hold_captured = false;
    float v_dot_filtered = 0.0f;

    while (true) {
        uint32_t now_us = time_us_32();
        if (now_us - last_watchdog_check_us >= 100000u) {
            last_watchdog_check_us = now_us;
            const uint32_t current_core1_heartbeat = core1_heartbeat;
            if (current_core1_heartbeat != observed_core1_heartbeat) {
                watchdog_update();
            }
            observed_core1_heartbeat = current_core1_heartbeat;
        }

        bool signal_ok = (now_us - last_valid_ppm_us) < MAX_NO_SIGNAL_TIME && signal_power > SIGNAL_POWER_THRESHOLD;

        if (now_us - last_valid_imu_us > 50000u) {
#ifdef PICOPILOT_ENABLE_DIAGNOSTICS
            control_mode = "IMU_TIMEOUT";
#endif
            set_pwm(THROTTLE_PIN, 1040);
            set_pwm(ROLL_PIN, 1500);
            set_pwm(PITCH_PIN, 1500);
            set_pwm(YAW_PIN, 1500);
            altitude_hold_captured = false;
        }

        char c;
        while (gps_ring_buffer.pop(c)) {
            if (gps.update(c)) {
                DIAGNOSTIC_COUNT(gps_parsed_update_count);
                if (!home_set && gps.satellites >= 4 && gps.hdop > 0.0f && gps.hdop < 3.5f && 
                    std::abs(gps.latitude) > 0.001 && std::abs(gps.longitude) > 0.001) {
                    home_lat = gps.latitude;
                    home_lon = gps.longitude;
                    home_altitude = gps.altitude;
                    home_set = true;
                }

                current_sensors.gps_timestamp_us = time_us_32();
                current_sensors.course_timestamp_us = gps.course_timestamp_us;
                current_sensors.gps_altitude = gps.altitude;
                current_sensors.speed = gps.speed;
                current_sensors.course = gps.course;
                current_sensors.course_valid = gps.course_valid;
                current_sensors.gps_fix_valid = gps.fix_state > 0 && gps.satellites >= 4;
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
            const bool bme_read_ok =
#ifdef PICOPILOT_ENABLE_DIAGNOSTICS
                bme.update_all();
#else
                bme.update_pressure();
#endif
#ifdef PICOPILOT_ENABLE_DIAGNOSTICS
            last_bme_read_ok = bme_read_ok;
#endif
            if (bme_read_ok) {
                DIAGNOSTIC_COUNT(bme_read_success_count);
#ifdef PICOPILOT_ENABLE_DIAGNOSTICS
                last_bme_success_us = now_us;
#endif
                current_sensors.baro_timestamp_us = now_us;
                current_sensors.pressure = bme.get_pressure();
            } else {
                DIAGNOSTIC_COUNT(bme_read_failure_count);
            }
        }

#ifdef PICOPILOT_ENABLE_DIAGNOSTICS
        if (now_us - last_diagnostics_us >= DIAGNOSTICS_INTERVAL_US) {
            last_diagnostics_us = now_us;
            print_diagnostics(current_sensors, ekf, signal_ok, last_valid_imu_us,
                              target_altitude_m, altitude_hold_captured, v_dot_filtered);
        }
#endif

        if (imu_data_ready) {
            imu_data_ready = false;
            const bool imu_read_ok = imu.read_data();
#ifdef PICOPILOT_ENABLE_DIAGNOSTICS
            last_imu_read_ok = imu_read_ok;
#endif
            if (!imu_read_ok) {
                DIAGNOSTIC_COUNT(imu_read_failure_count);
                continue;
            }
            DIAGNOSTIC_COUNT(imu_read_success_count);
            last_valid_imu_us = time_us_32();

            current_sensors.accel = {imu.ax, imu.ay, imu.az};
            current_sensors.gyro = {imu.gx, imu.gy, imu.gz};
            current_sensors.imu_timestamp_us = imu_irq_timestamp_us;

            send_sensors_to_core1(current_sensors);
            if (multicore_fifo_wready())
                multicore_fifo_push_blocking(1);

            ekf = get_ekf_on_core0();

            float dt_ctrl = static_cast<float>(now_us - last_control_time_us) / 1000000.0f;
            last_control_time_us = now_us;
            if (dt_ctrl <= 0.0f || dt_ctrl > 0.05f)
                dt_ctrl = 0.0025f;
#ifdef PICOPILOT_ENABLE_DIAGNOSTICS
            diagnostic_dt_ctrl = dt_ctrl;
#endif

#ifdef PICOPILOT_ENABLE_DIAGNOSTICS
            ::gps_navigation_valid = false;
#endif

            if (!signal_ok ||
                (pulses[4] >= 1500 && (!ekf.attitude_valid || !ekf.altitude_valid))) {
#ifdef PICOPILOT_ENABLE_DIAGNOSTICS
                control_mode = signal_ok ? "FAILSAFE_ESTIMATOR" : "FAILSAFE_PPM";
#endif
                set_pwm(THROTTLE_PIN, 1040);
                set_pwm(ROLL_PIN, 1500);
                set_pwm(PITCH_PIN, 1500);
                set_pwm(YAW_PIN, 1500);
                altitude_hold_captured = false;
            } else if (pulses[4] < 1500) {
#ifdef PICOPILOT_ENABLE_DIAGNOSTICS
                control_mode = "MANUAL";
#endif
                set_pwm(THROTTLE_PIN, pulses[2]);
                set_pwm(ROLL_PIN, pulses[0]);
                set_pwm(PITCH_PIN, pulses[1]);
                set_pwm(YAW_PIN, pulses[3]);
                altitude_hold_captured = false;
            } else {
#ifdef PICOPILOT_ENABLE_DIAGNOSTICS
                control_mode = "AUTO";
#endif
                Avionics::Quaternion q_curr = {ekf.q1, ekf.q2, ekf.q3, ekf.q4};
                Avionics::Vector3 gyro_rad_s = {-imu.gx, -imu.gy, imu.gz};
                float yaw_curr_rad = get_yaw_from_quaternion(ekf.q1, ekf.q2, ekf.q3, ekf.q4);

                if (!altitude_hold_captured) {
                    target_altitude_m = ekf.altitude;
                    altitude_hold_captured = true;
                }

                float sin_pitch = 2.0f * (ekf.q1 * ekf.q3 - ekf.q2 * ekf.q4);
                float v_dot_imu = imu.ax - 9.80665f * sin_pitch;
                v_dot_filtered += 0.1f * (v_dot_imu - v_dot_filtered);

                Avionics::TECSInputs tecs_in;
                tecs_in.h_set = target_altitude_m;
                tecs_in.h_curr = ekf.altitude;
                tecs_in.h_dot_curr = ekf.vertical_speed;
                tecs_in.v_set = map(pulses[5], 900.0f, 2200.0f, 0.0f, 27.0f);
                tecs_in.v_curr = current_sensors.speed;
                tecs_in.v_dot_curr = v_dot_filtered;
#ifdef PICOPILOT_ENABLE_DIAGNOSTICS
                diagnostic_tecs_inputs = tecs_in;
#endif

                Avionics::TECSOutputs tecs_out = tecs_controller.update(tecs_in, dt_ctrl);
#ifdef PICOPILOT_ENABLE_DIAGNOSTICS
                diagnostic_tecs_outputs = tecs_out;
#endif

                const uint32_t gps_navigation_time_us = time_us_32();
                const bool gps_navigation_valid =
                    home_set &&
                    current_sensors.gps_fix_valid &&
                    current_sensors.course_valid &&
                    current_sensors.gps_timestamp_us != 0 &&
                    current_sensors.course_timestamp_us != 0 &&
                    (uint32_t)(gps_navigation_time_us - current_sensors.gps_timestamp_us) <= GPS_NAVIGATION_TIMEOUT_US &&
                    (uint32_t)(gps_navigation_time_us - current_sensors.course_timestamp_us) <= GPS_NAVIGATION_TIMEOUT_US &&
                    std::isfinite(current_sensors.speed) &&
                    std::isfinite(current_sensors.course) &&
                    current_sensors.speed >= 1.0f;
#ifdef PICOPILOT_ENABLE_DIAGNOSTICS
                ::gps_navigation_valid = gps_navigation_valid;
#endif
                float target_course_rad;
                if (gps_navigation_valid &&
                    calculate_orbit_course(
                        current_sensors.lat, current_sensors.lon,
                        home_lat, home_lon,
                        HOME_ORBIT_RADIUS_M, HOME_ORBIT_CLOCKWISE,
                        target_course_rad)) {
                    flight_controller.updateGPSCourse(
                        target_course_rad,
                        current_sensors.course * (M_PI / 180.0f),
                        current_sensors.speed);
#ifdef PICOPILOT_ENABLE_DIAGNOSTICS
                    diagnostic_target_course_rad = target_course_rad;
#endif
                } else {
                    flight_controller.clearGPSCourse();
#ifdef PICOPILOT_ENABLE_DIAGNOSTICS
                    diagnostic_target_course_rad = 0.0f;
#endif
                }

                Avionics::ServoOutputs servos = flight_controller.updateAttitude(
                    dt_ctrl,
                    q_curr,
                    gyro_rad_s,
                    imu.ay,
                    yaw_curr_rad,
                    tecs_out
                );
#ifdef PICOPILOT_ENABLE_DIAGNOSTICS
                diagnostic_servo_outputs = servos;
#endif

                if(pulses[5] < 1200){
                    set_pwm(THROTTLE_PIN, pulses[2]);
                } else {
                    set_pwm(THROTTLE_PIN, throttle_to_pwm(servos.throttle));
                }
                set_pwm(ROLL_PIN, servo_to_pwm(-servos.aileron));
                set_pwm(PITCH_PIN, servo_to_pwm(servos.elevator));
                set_pwm(YAW_PIN, servo_to_pwm(-servos.rudder));
            }
        }

        if (now_us - last_telemetry_tx_us >= TELEMETRY_INTERVAL_US) {
            last_telemetry_tx_us = now_us;
            send_telemetry_dma(current_sensors, ekf);
        }

    }
}

int main() {
#ifdef PICOPILOT_ENABLE_DIAGNOSTICS
    stdio_init_all();
#endif
    watchdog_enable(1000, true);
    compensator_config.lever_arm_m = lever_arm_m;

    hardware_init();

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