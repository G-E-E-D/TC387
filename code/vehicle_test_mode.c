#include "vehicle_test_mode.h"

#include "vehicle_config.h"
#include "vehicle_hal.h"
#include "vehicle_hardware_config.h"
#include "vehicle_vision_camera.h"
#include "vision_shared.h"

#include "zf_common_headfile.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define TEST_PAGE_COUNT                 (6U)
#define TEST_SAMPLE_PERIOD_US          (20000ULL)
#define TEST_DISPLAY_PERIOD_US         (100000ULL)
#define TEST_MOTOR_ARM_HOLD_US         (2000000ULL)
#define TEST_MOTOR_STEP_TIMEOUT_US     (2000000ULL)
#define TEST_MOTOR_DUTY                (0.08f)

typedef enum
{
    TEST_PAGE_OVERVIEW = 0U,
    TEST_PAGE_ENCODERS,
    TEST_PAGE_STEERING,
    TEST_PAGE_IMU,
    TEST_PAGE_VISION,
    TEST_PAGE_MOTORS
} VehicleTestPage;

typedef struct
{
    VehicleHalStatus hal;
    VehicleTestPage page;
    uint64_t start_us;
    uint64_t next_sample_us;
    uint64_t next_display_us;
    uint64_t last_key_us;
    uint16_t left_raw;
    uint16_t right_raw;
    uint16_t previous_left_raw;
    uint16_t previous_right_raw;
    int16_t left_delta;
    int16_t right_delta;
    bool rear_sample_seen;
    uint16_t steering_raw;
    VehicleMt6701AbStatus steering_status;
    VehicleHalImuRaw imu;
    bool imu_read_ok;
    vision_runtime_snapshot_t vision;
    bool vision_read_ok;
    bool vision_camera_ready;
    uint32_t previous_camera_sequence;
    uint64_t previous_camera_timestamp_us;
    uint32_t vision_fps_x10;
    uint64_t motor_chord_start_us;
    bool motor_armed;
    uint8_t motor_step;
    uint64_t motor_step_deadline_us;
} VehicleTestState;

static VehicleTestState g_test;

static void test_line(uint8_t row, const char *format, ...)
{
    char line[64];
    va_list arguments;
    va_start(arguments, format);
    (void)vsnprintf(line, sizeof(line), format, arguments);
    va_end(arguments);
    vehicle_hal_display_line(row, line, NULL);
}

static bool both_buttons_down(void)
{
    return (gpio_get_level(VEHICLE_KEY_1_PIN) == GPIO_LOW) &&
           (gpio_get_level(VEHICLE_KEY_2_PIN) == GPIO_LOW);
}

static void motor_output_for_step(uint8_t step)
{
    switch(step)
    {
        case 1U:
            vehicle_hal_apply_calibration_test(VEHICLE_HAL_TEST_LEFT_MOTOR,
                                                TEST_MOTOR_DUTY);
            break;
        case 2U:
            vehicle_hal_apply_calibration_test(VEHICLE_HAL_TEST_LEFT_MOTOR,
                                                -TEST_MOTOR_DUTY);
            break;
        case 3U:
            vehicle_hal_apply_calibration_test(VEHICLE_HAL_TEST_RIGHT_MOTOR,
                                                TEST_MOTOR_DUTY);
            break;
        case 4U:
            vehicle_hal_apply_calibration_test(VEHICLE_HAL_TEST_RIGHT_MOTOR,
                                                -TEST_MOTOR_DUTY);
            break;
        case 5U:
            vehicle_hal_apply_calibration_test(VEHICLE_HAL_TEST_STEERING_MOTOR,
                                                TEST_MOTOR_DUTY);
            break;
        case 6U:
            vehicle_hal_apply_calibration_test(VEHICLE_HAL_TEST_STEERING_MOTOR,
                                                -TEST_MOTOR_DUTY);
            break;
        default:
            vehicle_hal_force_safe_outputs();
            break;
    }
}

static void disarm_motor_test(void)
{
    g_test.motor_armed = false;
    g_test.motor_step = 0U;
    g_test.motor_step_deadline_us = 0U;
    vehicle_hal_force_safe_outputs();
}

static void sample_inputs(uint64_t now_us)
{
    uint16_t left_raw;
    uint16_t right_raw;
    VehicleMt6701AbStatus steering_status;
    vehicle_hal_get_rear_encoder_raw(&left_raw, &right_raw);
    if(g_test.rear_sample_seen)
    {
        g_test.left_delta = (int16_t)(uint16_t)(left_raw -
                                                g_test.previous_left_raw);
        g_test.right_delta = (int16_t)(uint16_t)(right_raw -
                                                 g_test.previous_right_raw);
    }
    else
    {
        g_test.left_delta = 0;
        g_test.right_delta = 0;
        g_test.rear_sample_seen = true;
    }
    g_test.left_raw = left_raw;
    g_test.right_raw = right_raw;
    g_test.previous_left_raw = left_raw;
    g_test.previous_right_raw = right_raw;

    if(vehicle_hal_get_mt6701_ab_status(&steering_status))
    {
        g_test.steering_status = steering_status;
        g_test.steering_raw = steering_status.raw;
    }

    memset(&g_test.imu, 0, sizeof(g_test.imu));
    g_test.imu_read_ok = vehicle_hal_read_imu(&g_test.imu);

    g_test.vision_read_ok = false;
#if VEHICLE_VISION_ENABLE
    g_test.vision_read_ok = vision_shared_read(&g_test.vision) != 0U;
    if(g_test.vision_read_ok &&
       (g_test.vision.camera_sequence != g_test.previous_camera_sequence))
    {
        if((g_test.previous_camera_timestamp_us != 0U) &&
           (now_us > g_test.previous_camera_timestamp_us))
        {
            uint32_t sequence_delta = g_test.vision.camera_sequence -
                                       g_test.previous_camera_sequence;
            uint64_t elapsed_us = now_us -
                                  g_test.previous_camera_timestamp_us;
            if(elapsed_us > 0U)
            {
                g_test.vision_fps_x10 = (uint32_t)
                    (((uint64_t)sequence_delta * 10000000ULL) / elapsed_us);
            }
        }
        g_test.previous_camera_sequence = g_test.vision.camera_sequence;
        g_test.previous_camera_timestamp_us = now_us;
    }
#else
    (void)now_us;
#endif
}

static void handle_keys(uint64_t now_us)
{
    uint8_t keys = vehicle_hal_read_key_events();
    bool both_down = both_buttons_down();
    if(g_test.page == TEST_PAGE_MOTORS)
    {
        if(both_down)
        {
            if(g_test.motor_chord_start_us == 0U)
            {
                g_test.motor_chord_start_us = now_us;
            }
            else if(!g_test.motor_armed &&
                    ((now_us - g_test.motor_chord_start_us) >=
                     TEST_MOTOR_ARM_HOLD_US))
            {
                g_test.motor_armed = true;
                g_test.motor_step = 0U;
                g_test.motor_step_deadline_us = 0U;
                vehicle_hal_force_safe_outputs();
            }
        }
        else
        {
            g_test.motor_chord_start_us = 0U;
        }
        if(g_test.motor_armed && ((keys & 1U) != 0U))
        {
            disarm_motor_test();
        }
        else if(g_test.motor_armed && ((keys & 2U) != 0U))
        {
            g_test.motor_step = (uint8_t)((g_test.motor_step + 1U) % 7U);
            motor_output_for_step(g_test.motor_step);
            g_test.motor_step_deadline_us = now_us +
                                            TEST_MOTOR_STEP_TIMEOUT_US;
        }
        if(g_test.motor_armed &&
           (g_test.motor_step_deadline_us != 0U) &&
           (now_us >= g_test.motor_step_deadline_us))
        {
            disarm_motor_test();
        }
        return;
    }
    disarm_motor_test();
    if((keys & 1U) != 0U)
    {
        g_test.page = (VehicleTestPage)((g_test.page + 1U) %
                                        TEST_PAGE_COUNT);
    }
    if((keys & 2U) != 0U)
    {
        vehicle_hal_force_safe_outputs();
    }
}

static void render_overview(void)
{
    test_line(0U, "TEST 1/6  K1:NEXT");
    test_line(1U, "K2:SAFE STOP");
    test_line(2U, "MT AB A:P10.3 B:P10.2");
    test_line(3U, "Z:P10.5 D:P10.1 V/G OK");
    test_line(4U, "CAM:%c 188x120 50FPS",
              g_test.vision_camera_ready ? 'Y' : 'N');
    test_line(5U, "HAL M:%c E:%c S:%c",
              g_test.hal.motor_outputs_ready ? 'Y' : 'N',
              g_test.hal.rear_encoders_ready ? 'Y' : 'N',
              g_test.hal.steering_sensor_ready ? 'Y' : 'N');
    test_line(6U, "IMU:%c IPS:%c",
              g_test.hal.imu_ready ? 'Y' : 'N',
              g_test.hal.display_ready ? 'Y' : 'N');
    test_line(7U, "PWM SAFE (bench mode)");
    test_line(8U, "UPTIME:%lus", (unsigned long)
              ((vehicle_hal_now_us() - g_test.start_us) / 1000000ULL));
    test_line(9U, "K1 cycle; K2 stop");
}

static void render_encoders(void)
{
    test_line(0U, "TEST 2/6 ENCODERS");
    test_line(1U, "L raw:%u d:%d", (unsigned int)g_test.left_raw,
              (int)g_test.left_delta);
    test_line(2U, "R raw:%u d:%d", (unsigned int)g_test.right_raw,
              (int)g_test.right_delta);
    test_line(3U, "TIM6 L / TIM3 R");
    test_line(4U, "Spin wheels by hand");
    test_line(5U, "Signs: forward > 0");
    test_line(6U, "delta every 20ms");
    test_line(7U, "PWM forced 0");
    test_line(8U, "L/R raw are uint16");
    test_line(9U, "K1 next  K2 stop");
}

static void render_steering(void)
{
    test_line(0U, "TEST 3/6 MT6701 AB");
    test_line(1U, "raw:%u cnt:%lld", (unsigned int)g_test.steering_raw,
              (long long)g_test.steering_status.continuous_count);
    test_line(2U, "A:P10.3 B:P10.2");
    test_line(3U, "A:%u B:%u Z:%u D:%u",
              (unsigned int)g_test.steering_status.a_level,
              (unsigned int)g_test.steering_status.b_level,
              (unsigned int)g_test.steering_status.z_level,
              (unsigned int)g_test.steering_status.dir_level);
    test_line(4U, "IDX:%lu SEEN:%c",
              (unsigned long)g_test.steering_status.index_pulse_count,
              g_test.steering_status.index_seen ? 'Y' : 'N');
    if(g_test.steering_status.last_index_interval_valid)
    {
        test_line(5U, "dZ:%lld", (long long)
                  g_test.steering_status.last_index_interval_count);
    }
    else
    {
        test_line(5U, "dZ:wait next Z");
    }
    test_line(6U, "BAD:%lu DM:%lu", (unsigned long)
              g_test.steering_status.invalid_transition_count,
              (unsigned long)g_test.steering_status.dir_mismatch_count);
    test_line(7U, "Turn left/right slowly");
    test_line(8U, "Z validates; no reset");
    test_line(9U, "K1 next  K2 stop");
}

static void render_imu(void)
{
    test_line(0U, "TEST 4/6 IMU");
    test_line(1U, "A %d %d %d", g_test.imu.acceleration[0],
              g_test.imu.acceleration[1], g_test.imu.acceleration[2]);
    test_line(2U, "G %d %d %d", g_test.imu.angular_rate[0],
              g_test.imu.angular_rate[1], g_test.imu.angular_rate[2]);
    test_line(3U, "M %d %d %d", g_test.imu.magnetic_field[0],
              g_test.imu.magnetic_field[1], g_test.imu.magnetic_field[2]);
    test_line(4U, "TEMP:%d", g_test.imu.temperature);
    test_line(5U, "AG COMM:%c MAG:%c",
              g_test.imu.accel_gyro_communication_ok ? 'Y' : 'N',
              g_test.imu.magnetometer_communication_ok ? 'Y' : 'N');
    test_line(6U, "Hold still; then rotate");
    test_line(7U, "raw LSB values");
    test_line(8U, "PWM forced 0");
    test_line(9U, "K1 next  K2 stop");
}

static void render_vision(void)
{
    const vision_tag_result_t *result = &g_test.vision.result;
    test_line(0U, "TEST 5/6 VISION");
    test_line(1U, "READ:%c SEQ:%lu FPSx10:%lu",
              g_test.vision_read_ok ? 'Y' : 'N',
              (unsigned long)g_test.vision.camera_sequence,
              (unsigned long)g_test.vision_fps_x10);
    test_line(2U, "D:%u V:%u P:%u L:%u", (unsigned int)result->detected,
              (unsigned int)result->valid, (unsigned int)result->predicted,
              (unsigned int)result->lost_frames);
    test_line(3U, "CX:%d EX:%d Q:%d", (int)result->center_x,
              (int)result->error_x_px, (int)result->error_x_q15);
    test_line(4U, "SIZE:%u CONF:%u", (unsigned int)result->size_px,
              (unsigned int)result->confidence);
    test_line(5U, "GRAY:%u/%u/%u", (unsigned int)result->gray_p10,
              (unsigned int)result->gray_p50, (unsigned int)result->gray_p90);
    test_line(6U, "SAT:%u C:%u", (unsigned int)result->saturated_permille,
              (unsigned int)result->tag_contrast);
    test_line(7U, "PROC:%lu us", (unsigned long)g_test.vision.process_us_last);
    test_line(8U, "EXPECT 188x120 50FPS");
    test_line(9U, "K1 next  K2 stop");
}

static void render_motors(void)
{
    const char *state = g_test.motor_armed ? "ARMED" : "SAFE";
    test_line(0U, "TEST 6/6 MOTORS %s", state);
    test_line(1U, "hold K1+K2 2s arm");
    test_line(2U, "K2 step; K1 disarm");
    test_line(3U, "STEP:%u DUTY:%.2f", (unsigned int)g_test.motor_step,
              (double)TEST_MOTOR_DUTY);
    test_line(4U, "1 L+ 2 L- 3 R+");
    test_line(5U, "4 R- 5 S+ 6 S-");
    test_line(6U, "each step timeout 2s");
    test_line(7U, "ENC d:%d/%d", (int)g_test.left_delta,
              (int)g_test.right_delta);
    test_line(8U, "K1 stop; auto 2s");
    test_line(9U, "off ground only");
}

static void render_page(void)
{
    switch(g_test.page)
    {
        case TEST_PAGE_OVERVIEW:
            render_overview();
            break;
        case TEST_PAGE_ENCODERS:
            render_encoders();
            break;
        case TEST_PAGE_STEERING:
            render_steering();
            break;
        case TEST_PAGE_IMU:
            render_imu();
            break;
        case TEST_PAGE_VISION:
            render_vision();
            break;
        case TEST_PAGE_MOTORS:
            render_motors();
            break;
        default:
            g_test.page = TEST_PAGE_OVERVIEW;
            render_overview();
            break;
    }
}

bool vehicle_test_mode_init(void)
{
    memset(&g_test, 0, sizeof(g_test));
    g_test.hal = vehicle_hal_init();
    vision_shared_init();
#if VEHICLE_VISION_ENABLE
    g_test.vision_camera_ready = vehicle_vision_camera_init();
#endif
    vehicle_hal_force_safe_outputs();
    g_test.start_us = vehicle_hal_now_us();
    g_test.next_sample_us = g_test.start_us;
    g_test.next_display_us = g_test.start_us;
    g_test.last_key_us = g_test.start_us;
    render_page();
    return g_test.hal.display_ready;
}

void vehicle_test_mode_process(void)
{
    uint64_t now_us = vehicle_hal_now_us();
    uint32_t catchup = 0U;
    while(vehicle_hal_take_fast_tick() && (catchup < 10U))
    {
        vehicle_hal_capture_encoder_counts();
        catchup++;
    }
    if(now_us >= g_test.next_sample_us)
    {
        sample_inputs(now_us);
        g_test.next_sample_us = now_us + TEST_SAMPLE_PERIOD_US;
    }
    if((now_us - g_test.last_key_us) >= 30000ULL)
    {
        handle_keys(now_us);
        g_test.last_key_us = now_us;
    }
    if(now_us >= g_test.next_display_us)
    {
        render_page();
        g_test.next_display_us = now_us + TEST_DISPLAY_PERIOD_US;
    }
}
