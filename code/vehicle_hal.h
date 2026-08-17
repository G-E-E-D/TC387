#ifndef VEHICLE_HAL_H
#define VEHICLE_HAL_H

#include "vehicle_calibration.h"
#include "vehicle_types.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum VehicleHalTestMotor
{
    VEHICLE_HAL_TEST_LEFT_MOTOR = 0,
    VEHICLE_HAL_TEST_RIGHT_MOTOR,
    VEHICLE_HAL_TEST_STEERING_MOTOR
};

struct VehicleHalStatus
{
    bool motor_outputs_ready;
    bool rear_encoders_ready;
    bool steering_sensor_ready;
    bool imu_ready;
    bool display_ready;
    bool timer_ready;
    bool critical_ready;
};

struct VehicleHalImuRaw
{
    int16_t acceleration[3];
    int16_t angular_rate[3];
    int16_t magnetic_field[3];
    int16_t temperature;
    bool accel_gyro_communication_ok;
    bool magnetometer_communication_ok;
};

void vehicle_hal_force_safe_outputs();
VehicleHalStatus vehicle_hal_init();
uint64_t vehicle_hal_now_us();
void vehicle_hal_timer_tick_isr();
bool vehicle_hal_take_fast_tick();
void vehicle_hal_service_control_watchdog();
bool vehicle_hal_control_watchdog_expired();
bool vehicle_hal_clear_control_watchdog_fault();
void vehicle_hal_capture_encoder_counts();
void vehicle_hal_get_rear_encoder_raw(uint16_t *left, uint16_t *right);
void vehicle_hal_zero_rear_encoder_raw();
/* Returns the 12-bit SPI encoder count after a bounded two-byte transaction. */
bool vehicle_hal_read_steering_raw(uint16_t *raw_count);
bool vehicle_hal_read_imu(VehicleHalImuRaw *raw);
void vehicle_hal_get_imu_scale(float *acceleration_mps2_per_lsb,
                               float *angular_rate_radps_per_lsb,
                               float *magnetic_gauss_per_lsb,
                               float *temperature_c_per_lsb,
                               float *temperature_offset_c);
void vehicle_hal_apply_actuators(const VehicleActuatorCommand *command,
                                 const VehicleCalibration *calibration,
                                 bool power_allowed);
void vehicle_hal_apply_calibration_test(VehicleHalTestMotor motor,
                                        float signed_raw_direction_duty);
bool vehicle_hal_uart_read_byte(uint8_t *byte);
bool vehicle_hal_uart_try_write_byte(uint8_t byte, void *context);
void vehicle_hal_display_line(uint8_t row, const char *text, void *context);
uint8_t vehicle_hal_read_key_events();

#endif
