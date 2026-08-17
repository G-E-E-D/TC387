#include "vehicle_hal.h"

#include "vehicle_config.h"
#include "vehicle_hardware_config.h"
#include "vehicle_math.h"

#include "zf_common_headfile.h"
#include "IfxQspi.h"
#include "IFXQSPI_REGDEF.h"

#include <stdio.h>
#include <string.h>

constexpr auto VEHICLE_PWM_DUTY_MAX_U32 = 10000U;
constexpr auto VEHICLE_UART_TX_FIFO_LIMIT = 15U;
constexpr auto VEHICLE_IMU_FROZEN_LIMIT =
    static_cast<uint32_t>(VEHICLE_SENSOR_TIMEOUT_US / VEHICLE_ESTIMATOR_PERIOD_US) +
    1U;
constexpr auto VEHICLE_ISR_WATCHDOG_TICKS = 30U;

static volatile uint32_t g_fast_tick_count;
static volatile uint32_t g_control_heartbeat_ticks;
static volatile bool g_control_watchdog_armed;
static volatile bool g_control_watchdog_fault;
static uint64_t g_time_start_ticks;
static uint32_t g_time_frequency_hz;
static uint16_t g_left_encoder_raw;
static uint16_t g_right_encoder_raw;
static int16_t g_last_imu_motion_raw[7];
static uint32_t g_imu_unchanged_count;
static uint8_t g_previous_keys;
static bool g_time_initialized;
static bool g_imu_sample_seen;

extern "C"
{
    /* Reuse the BACON word prepared by zf_driver_spi, but poll the FIFO with
       a deadline so a missing encoder cannot hang the estimator task. */
    extern Ifx_QSPI_BACON bacon[5];
}

static bool steering_spi_timed_out(uint32_t start_us)
{
    return (static_cast<uint32_t>(system_getval_us()) - start_us) >=
           STEERING_ENCODER_SPI_TIMEOUT_US;
}

static bool steering_spi_wait_expired(uint32_t start_us,
                                      uint32_t *poll_budget)
{
    if(steering_spi_timed_out(start_us) ||
       (poll_budget == nullptr) || (*poll_budget == 0U))
    {
        return true;
    }
    --(*poll_budget);
    return false;
}

static void steering_spi_clear(Ifx_QSPI *module)
{
    uint32_t fifo_level = module->STATUS.B.RXFIFOLEVEL;
    while(fifo_level > 0U)
    {
        static_cast<void>(IfxQspi_readReceiveFifo(module));
        --fifo_level;
    }
    IfxQspi_clearAllEventFlags(module);
}

static bool steering_spi_transfer_bounded(const uint8_t *write_buffer,
                                          uint8_t *read_buffer,
                                          uint32_t length)
{
    Ifx_QSPI *module = IfxQspi_getAddress(
        static_cast<IfxQspi_Index>(VEHICLE_STEERING_ENCODER_SPI_INDEX));
    const uint32_t start_us = system_getval_us();
    uint32_t poll_budget = 100000U;

    if((module == nullptr) || (write_buffer == nullptr) || (length == 0U))
    {
        return false;
    }

    IfxQspi_writeBasicConfigurationBeginStream(
        module, bacon[VEHICLE_STEERING_ENCODER_SPI_INDEX].U);
    steering_spi_clear(module);

    do
    {
        if(length == 1U)
        {
            IfxQspi_writeBasicConfigurationEndStream(
                module, bacon[VEHICLE_STEERING_ENCODER_SPI_INDEX].U);
        }
        if(steering_spi_wait_expired(start_us, &poll_budget))
        {
            goto timeout;
        }

        IfxQspi_writeTransmitFifo(module, *write_buffer++);
        if(read_buffer != nullptr)
        {
            while(module->STATUS.B.RXFIFOLEVEL == 0U)
            {
                if(steering_spi_wait_expired(start_us, &poll_budget))
                {
                    goto timeout;
                }
            }
            *read_buffer++ = static_cast<uint8_t>(IfxQspi_readReceiveFifo(module));
        }
        else
        {
            while(module->STATUS.B.TXFIFOLEVEL != 0U)
            {
                if(steering_spi_wait_expired(start_us, &poll_budget))
                {
                    goto timeout;
                }
            }
        }
    } while(--length);

    while(module->STATUS.B.PT1F == 0U)
    {
        if(steering_spi_wait_expired(start_us, &poll_budget))
        {
            goto timeout;
        }
    }

    IfxQspi_clearAllEventFlags(module);
    return true;

timeout:
    IfxQspi_writeBasicConfigurationEndStream(
        module, bacon[VEHICLE_STEERING_ENCODER_SPI_INDEX].U);
    steering_spi_clear(module);
    return false;
}

static bool steering_spi_read_register(uint8_t reg, uint8_t *value)
{
    uint8_t command[2] = { static_cast<uint8_t>(reg | 0x40U), 0U };
    uint8_t response[2] = { 0U, 0U };

    if(value == nullptr || !steering_spi_transfer_bounded(
           command, response, 2U))
    {
        return false;
    }
    system_delay_us(1U);
    command[0] = 0U;
    command[1] = 0U;
    response[0] = 0U;
    response[1] = 0U;
    if(!steering_spi_transfer_bounded(command, response, 2U))
    {
        return false;
    }
    *value = response[0];
    return true;
}

static bool steering_spi_write_register(uint8_t reg, uint8_t value)
{
    uint8_t command[2] = { static_cast<uint8_t>(reg | 0x80U), value };
    uint8_t response[2] = { 0U, 0U };

    if(!steering_spi_transfer_bounded(command, response, 2U))
    {
        return false;
    }
    system_delay_us(1U);
    command[0] = 0U;
    command[1] = 0U;
    response[0] = 0U;
    response[1] = 0U;
    return steering_spi_transfer_bounded(command, response, 2U);
}

static bool steering_spi_self_check()
{
    static const uint8_t defaults[6] = { 0U, 0U, 0U, 0xC0U, 0xFFU, 0x1CU };
    uint8_t status = 0U;
    uint32_t attempt;
    uint8_t index;

    for(attempt = 0U; attempt <= 100U; ++attempt)
    {
        if(!steering_spi_read_register(6U, &status))
        {
            return false;
        }
        if(status == 0x1CU)
        {
            /* Keep the sensor's direction/zero registers deterministic; the
               vehicle still applies its measured center in software. */
            return steering_spi_write_register(0x09U, 0x00U) &&
                   steering_spi_write_register(0x00U, 0x00U) &&
                   steering_spi_write_register(0x01U, 0x00U);
        }
        for(index = 0U; index < 6U; ++index)
        {
            if(!steering_spi_write_register(static_cast<uint8_t>(index + 1U),
                                            defaults[index]))
            {
                return false;
            }
            system_delay_ms(1U);
        }
    }
    return false;
}

static uint32_t duty_to_counts(float duty)
{
    float magnitude = (duty < 0.0f) ? -duty : duty;
    if(!vehicle_float_is_finite(magnitude))
    {
        return 0U;
    }
    magnitude = vehicle_clampf(magnitude, 0.0f, 1.0f);
    return static_cast<uint32_t>(magnitude * static_cast<float>(VEHICLE_PWM_DUTY_MAX_U32) + 0.5f);
}

static void set_motor(pwm_channel_enum pwm_pin, gpio_pin_enum direction_pin,
                      float signed_duty, int8_t positive_direction)
{
    uint32_t duty = duty_to_counts(signed_duty);
    bool direction_high;
    if((duty == 0U) || ((positive_direction != 1) && (positive_direction != -1)))
    {
        pwm_set_duty(pwm_pin, 0U);
        gpio_set_level(direction_pin, GPIO_LOW);
        return;
    }
    direction_high = ((signed_duty > 0.0f) && (positive_direction > 0)) ||
                     ((signed_duty < 0.0f) && (positive_direction < 0));
    pwm_set_duty(pwm_pin, 0U);
    gpio_set_level(direction_pin, direction_high ? GPIO_HIGH : GPIO_LOW);
    pwm_set_duty(pwm_pin, duty);
}

static void set_raw_test_motor(pwm_channel_enum pwm_pin,
                               gpio_pin_enum direction_pin, float signed_duty)
{
    uint32_t duty = duty_to_counts(vehicle_clampf(signed_duty,
                                                   -CALIBRATION_TEST_MAX_DUTY,
                                                   CALIBRATION_TEST_MAX_DUTY));
    pwm_set_duty(pwm_pin, 0U);
    gpio_set_level(direction_pin, (signed_duty > 0.0f) ? GPIO_HIGH : GPIO_LOW);
    pwm_set_duty(pwm_pin, duty);
}

void vehicle_hal_force_safe_outputs()
{
    pwm_set_duty(VEHICLE_LEFT_PWM_PIN, 0U);
    pwm_set_duty(VEHICLE_RIGHT_PWM_PIN, 0U);
    pwm_set_duty(VEHICLE_STEERING_PWM_PIN, 0U);
    gpio_set_level(VEHICLE_LEFT_DIRECTION_PIN, GPIO_LOW);
    gpio_set_level(VEHICLE_RIGHT_DIRECTION_PIN, GPIO_LOW);
    gpio_set_level(VEHICLE_STEERING_DIRECTION_PIN, GPIO_LOW);
}

VehicleHalStatus vehicle_hal_init()
{
    VehicleHalStatus status;
    status = {};

    gpio_init(VEHICLE_LEFT_DIRECTION_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(VEHICLE_RIGHT_DIRECTION_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(VEHICLE_STEERING_DIRECTION_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    pwm_init(VEHICLE_LEFT_PWM_PIN, VEHICLE_MOTOR_PWM_HZ, 0U);
    pwm_init(VEHICLE_RIGHT_PWM_PIN, VEHICLE_MOTOR_PWM_HZ, 0U);
    pwm_init(VEHICLE_STEERING_PWM_PIN, VEHICLE_MOTOR_PWM_HZ, 0U);
    vehicle_hal_force_safe_outputs();
    status.motor_outputs_ready = true;

    /* Rear encoders provide A phase plus DIR, not quadrature A/B. */
    encoder_dir_init(VEHICLE_LEFT_ENCODER_INDEX,
                     VEHICLE_LEFT_ENCODER_A_PIN,
                     VEHICLE_LEFT_ENCODER_DIR_PIN);
    encoder_dir_init(VEHICLE_RIGHT_ENCODER_INDEX,
                     VEHICLE_RIGHT_ENCODER_A_PIN,
                     VEHICLE_RIGHT_ENCODER_DIR_PIN);
    encoder_clear_count(VEHICLE_LEFT_ENCODER_INDEX);
    encoder_clear_count(VEHICLE_RIGHT_ENCODER_INDEX);
    g_left_encoder_raw = 0U;
    g_right_encoder_raw = 0U;
    status.rear_encoders_ready = true;

    spi_init(VEHICLE_STEERING_ENCODER_SPI_INDEX,
             VEHICLE_STEERING_ENCODER_SPI_MODE,
             VEHICLE_STEERING_ENCODER_SPI_HZ,
             VEHICLE_STEERING_ENCODER_SCLK,
             VEHICLE_STEERING_ENCODER_MOSI,
             VEHICLE_STEERING_ENCODER_MISO,
             VEHICLE_STEERING_ENCODER_CS);
    status.steering_sensor_ready = steering_spi_self_check();

    status.imu_ready = (imu963ra_init() == 0U);
    g_imu_unchanged_count = 0U;
    g_imu_sample_seen = false;

#if VEHICLE_IPS200_ENABLE
    ips200_init(IPS200_TYPE_SPI);
    ips200_set_dir(IPS200_PORTAIT);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_clear();
    status.display_ready = true;
#else
    status.display_ready = false;
#endif

    gpio_init(VEHICLE_KEY_1_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(VEHICLE_KEY_2_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    g_previous_keys = 0U;

    system_start();
    {
        Ifx_STM *stm = IfxStm_getAddress((IfxStm_Index)IfxCpu_getCoreId());
        g_time_start_ticks = static_cast<uint64_t>(IfxStm_get(stm));
        g_time_frequency_hz = static_cast<uint32_t>(IfxStm_getFrequency(stm));
    }
    g_time_initialized = true;
    g_fast_tick_count = 0U;
    g_control_heartbeat_ticks = 0U;
    g_control_watchdog_armed = false;
    g_control_watchdog_fault = false;
    pit_ms_init(CCU60_CH0, 1U);
    status.timer_ready = true;

    status.critical_ready = status.motor_outputs_ready &&
                            status.rear_encoders_ready &&
                            status.steering_sensor_ready &&
                            status.imu_ready && status.timer_ready;
    return status;
}

uint64_t vehicle_hal_now_us()
{
    Ifx_STM *stm;
    uint64_t elapsed_ticks;
    uint64_t seconds;
    uint64_t remainder;
    if(!g_time_initialized || (g_time_frequency_hz == 0U))
    {
        return 0U;
    }
    stm = IfxStm_getAddress((IfxStm_Index)IfxCpu_getCoreId());
    elapsed_ticks = static_cast<uint64_t>(IfxStm_get(stm)) - g_time_start_ticks;
    seconds = elapsed_ticks / static_cast<uint64_t>(g_time_frequency_hz);
    remainder = elapsed_ticks % static_cast<uint64_t>(g_time_frequency_hz);
    return seconds * UINT64_C(1000000) +
           (remainder * UINT64_C(1000000)) /
           static_cast<uint64_t>(g_time_frequency_hz);
}

void vehicle_hal_timer_tick_isr()
{
    if(g_fast_tick_count < UINT32_MAX)
    {
        g_fast_tick_count++;
    }
    if(g_control_watchdog_armed)
    {
        if(g_control_heartbeat_ticks < UINT32_MAX)
        {
            g_control_heartbeat_ticks++;
        }
        if(g_control_heartbeat_ticks > VEHICLE_ISR_WATCHDOG_TICKS)
        {
            pwm_set_duty(VEHICLE_LEFT_PWM_PIN, 0U);
            pwm_set_duty(VEHICLE_RIGHT_PWM_PIN, 0U);
            pwm_set_duty(VEHICLE_STEERING_PWM_PIN, 0U);
            gpio_set_level(VEHICLE_LEFT_DIRECTION_PIN, GPIO_LOW);
            gpio_set_level(VEHICLE_RIGHT_DIRECTION_PIN, GPIO_LOW);
            gpio_set_level(VEHICLE_STEERING_DIRECTION_PIN, GPIO_LOW);
            g_control_watchdog_fault = true;
        }
    }
}

bool vehicle_hal_take_fast_tick()
{
    bool available = g_fast_tick_count > 0U;
    if(available)
    {
        g_fast_tick_count--;
    }
    return available;
}

void vehicle_hal_service_control_watchdog()
{
    g_control_heartbeat_ticks = 0U;
    g_control_watchdog_armed = true;
}

bool vehicle_hal_control_watchdog_expired()
{
    return g_control_watchdog_fault;
}

bool vehicle_hal_clear_control_watchdog_fault()
{
    if(g_control_heartbeat_ticks > VEHICLE_ISR_WATCHDOG_TICKS)
    {
        return false;
    }
    g_control_watchdog_fault = false;
    return true;
}

void vehicle_hal_capture_encoder_counts()
{
    int16_t left_delta = encoder_get_count(VEHICLE_LEFT_ENCODER_INDEX);
    int16_t right_delta = encoder_get_count(VEHICLE_RIGHT_ENCODER_INDEX);
    encoder_clear_count(VEHICLE_LEFT_ENCODER_INDEX);
    encoder_clear_count(VEHICLE_RIGHT_ENCODER_INDEX);
    g_left_encoder_raw = static_cast<uint16_t>(g_left_encoder_raw + static_cast<uint16_t>(left_delta));
    g_right_encoder_raw = static_cast<uint16_t>(g_right_encoder_raw + static_cast<uint16_t>(right_delta));
}

void vehicle_hal_get_rear_encoder_raw(uint16_t *left, uint16_t *right)
{
    if(left != nullptr)
    {
        *left = g_left_encoder_raw;
    }
    if(right != nullptr)
    {
        *right = g_right_encoder_raw;
    }
}

void vehicle_hal_zero_rear_encoder_raw()
{
    encoder_clear_count(VEHICLE_LEFT_ENCODER_INDEX);
    encoder_clear_count(VEHICLE_RIGHT_ENCODER_INDEX);
    g_left_encoder_raw = 0U;
    g_right_encoder_raw = 0U;
}

bool vehicle_hal_read_steering_raw(uint16_t *raw_count)
{
    uint8_t transmit[2] = {0U, 0U};
    uint8_t receive[2] = {0U, 0U};
    uint16_t frame;

    if(raw_count == nullptr)
    {
        return false;
    }
    if(!steering_spi_transfer_bounded(transmit, receive, 2U))
    {
        return false;
    }
    frame = static_cast<uint16_t>((static_cast<uint16_t>(receive[0]) << 8U) |
                                  receive[1]);
    /* The encoder exposes a 12-bit value in bits 15..4; reserved low bits
       must stay zero so a stuck 0xFFFF/bus frame is not accepted as 4095. */
    if((frame & 0x000FU) != 0U)
    {
        return false;
    }
    *raw_count = static_cast<uint16_t>((frame >> 4U) & 0x0FFFU);
    return true;
}

bool vehicle_hal_read_imu(VehicleHalImuRaw *raw)
{
    int16_t current_motion[7];
    bool unchanged;
    if(raw == nullptr)
    {
        return false;
    }
    imu963ra_get_acc();
    imu963ra_get_gyro();
    imu963ra_get_temperature();
    if(imu963ra_mag_available != 0U)
    {
        imu963ra_get_mag();
    }
    raw->acceleration[0] = imu963ra_acc_x;
    raw->acceleration[1] = imu963ra_acc_y;
    raw->acceleration[2] = imu963ra_acc_z;
    raw->angular_rate[0] = imu963ra_gyro_x;
    raw->angular_rate[1] = imu963ra_gyro_y;
    raw->angular_rate[2] = imu963ra_gyro_z;
    raw->magnetic_field[0] = imu963ra_mag_x;
    raw->magnetic_field[1] = imu963ra_mag_y;
    raw->magnetic_field[2] = imu963ra_mag_z;
    raw->temperature = imu963ra_temperature;
    current_motion[0] = raw->acceleration[0];
    current_motion[1] = raw->acceleration[1];
    current_motion[2] = raw->acceleration[2];
    current_motion[3] = raw->angular_rate[0];
    current_motion[4] = raw->angular_rate[1];
    current_motion[5] = raw->angular_rate[2];
    current_motion[6] = raw->temperature;
    unchanged = g_imu_sample_seen &&
                (memcmp(current_motion, g_last_imu_motion_raw,
                        sizeof(current_motion)) == 0);
    if(unchanged)
    {
        if(g_imu_unchanged_count < UINT32_MAX)
        {
            g_imu_unchanged_count++;
        }
    }
    else
    {
        memcpy(g_last_imu_motion_raw, current_motion, sizeof(current_motion));
        g_imu_unchanged_count = 0U;
        g_imu_sample_seen = true;
    }
    raw->accel_gyro_communication_ok =
        g_imu_unchanged_count < VEHICLE_IMU_FROZEN_LIMIT;
    raw->magnetometer_communication_ok = imu963ra_mag_available != 0U;
    return raw->accel_gyro_communication_ok;
}

void vehicle_hal_get_imu_scale(float *acceleration_mps2_per_lsb,
                               float *angular_rate_radps_per_lsb,
                               float *magnetic_gauss_per_lsb,
                               float *temperature_c_per_lsb,
                               float *temperature_offset_c)
{
    if(acceleration_mps2_per_lsb != nullptr)
    {
        *acceleration_mps2_per_lsb = 9.80665f /
            imu963ra_transition_factor[0];
    }
    if(angular_rate_radps_per_lsb != nullptr)
    {
        *angular_rate_radps_per_lsb = (3.14159265358979323846f / 180.0f) /
            imu963ra_transition_factor[1];
    }
    if(magnetic_gauss_per_lsb != nullptr)
    {
        *magnetic_gauss_per_lsb = 1.0f / imu963ra_transition_factor[2];
    }
    if(temperature_c_per_lsb != nullptr)
    {
        *temperature_c_per_lsb = 1.0f / 256.0f;
    }
    if(temperature_offset_c != nullptr)
    {
        *temperature_offset_c = 25.0f;
    }
}

void vehicle_hal_apply_actuators(const VehicleActuatorCommand *command,
                                 const VehicleCalibration *calibration,
                                 bool power_allowed)
{
    if((command == nullptr) || (calibration == nullptr) || !power_allowed ||
       command->immediate_stop || g_control_watchdog_fault)
    {
        vehicle_hal_force_safe_outputs();
        return;
    }
    set_motor(VEHICLE_LEFT_PWM_PIN, VEHICLE_LEFT_DIRECTION_PIN,
              command->left_motor_duty,
              calibration->left_motor_forward_direction);
    set_motor(VEHICLE_RIGHT_PWM_PIN, VEHICLE_RIGHT_DIRECTION_PIN,
              command->right_motor_duty,
              calibration->right_motor_forward_direction);
    set_motor(VEHICLE_STEERING_PWM_PIN, VEHICLE_STEERING_DIRECTION_PIN,
              command->steering_motor_duty,
              calibration->steering_left_direction);
}

void vehicle_hal_apply_calibration_test(VehicleHalTestMotor motor,
                                        float signed_raw_direction_duty)
{
    vehicle_hal_force_safe_outputs();
    if(g_control_watchdog_fault)
    {
        return;
    }
    switch(motor)
    {
        case VEHICLE_HAL_TEST_LEFT_MOTOR:
            set_raw_test_motor(VEHICLE_LEFT_PWM_PIN,
                               VEHICLE_LEFT_DIRECTION_PIN,
                               signed_raw_direction_duty);
            break;
        case VEHICLE_HAL_TEST_RIGHT_MOTOR:
            set_raw_test_motor(VEHICLE_RIGHT_PWM_PIN,
                               VEHICLE_RIGHT_DIRECTION_PIN,
                               signed_raw_direction_duty);
            break;
        case VEHICLE_HAL_TEST_STEERING_MOTOR:
            set_raw_test_motor(VEHICLE_STEERING_PWM_PIN,
                               VEHICLE_STEERING_DIRECTION_PIN,
                               signed_raw_direction_duty);
            break;
        default:
            break;
    }
}

bool vehicle_hal_uart_read_byte(uint8_t *byte)
{
    if(byte == nullptr)
    {
        return false;
    }
#if DEBUG_UART_USE_INTERRUPT
    return debug_read_ring_buffer((uint8 *)byte, 1U) == 1U;
#else
    return uart_query_byte(DEBUG_UART_INDEX, (uint8 *)byte) != 0U;
#endif
}

bool vehicle_hal_uart_try_write_byte(uint8_t byte, void *context)
{
    static_cast<void>(context);
    if(IfxAsclin_getTxFifoFillLevel(uart0_handle.asclin) >=
       VEHICLE_UART_TX_FIFO_LIMIT)
    {
        return false;
    }
    uart0_handle.asclin->TXDATA.U = byte;
    return true;
}

void vehicle_hal_display_line(uint8_t row, const char *text, void *context)
{
    static_cast<void>(context);
#if VEHICLE_IPS200_ENABLE
    char padded[31];
    if(text != nullptr)
    {
        static_cast<void>(snprintf(padded, sizeof(padded), "%-29.29s", text));
        ips200_show_string(0U, (uint16)(row * 16U), padded);
    }
#else
    static_cast<void>(row);
    static_cast<void>(text);
#endif
}

uint8_t vehicle_hal_read_key_events()
{
    uint8_t current = 0U;
    uint8_t pressed;
    if(gpio_get_level(VEHICLE_KEY_1_PIN) == GPIO_LOW)
    {
        current |= 1U;
    }
    if(gpio_get_level(VEHICLE_KEY_2_PIN) == GPIO_LOW)
    {
        current |= 2U;
    }
    pressed = static_cast<uint8_t>(current &
                                   static_cast<uint8_t>(~g_previous_keys));
    g_previous_keys = current;
    return pressed;
}
