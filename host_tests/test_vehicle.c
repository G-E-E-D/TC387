#if defined(__TASKING__)

typedef int vehicle_host_tests_are_not_target_firmware;

#else

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "vehicle_calibration.h"
#include "vehicle_config.h"
#include "vehicle_control.h"
#include "vehicle_diagnostics.h"
#include "vehicle_encoder.h"
#include "vehicle_fault.h"
#include "vehicle_localization.h"
#include "vehicle_math.h"
#include "vehicle_mt6701.h"
#include "vehicle_path.h"
#include "vehicle_reverse_tracker.h"
#include "vehicle_safety.h"
#include "vehicle_state_machine.h"

#define ARRAY_COUNT(array) (sizeof(array) / sizeof((array)[0]))

#define TEST_REQUIRE(condition)                                                \
    do                                                                         \
    {                                                                          \
        if(!(condition))                                                       \
        {                                                                      \
            fprintf(stderr, "    check failed at %s:%d: %s\n",              \
                    __FILE__, __LINE__, #condition);                           \
            return false;                                                      \
        }                                                                      \
    } while(0)

typedef bool (*TestFunction)(void);

typedef struct
{
    const char *name;
    TestFunction function;
} TestCase;

static bool nearly_equal(float actual, float expected, float tolerance)
{
    return isfinite(actual) && isfinite(expected) &&
           (fabsf(actual - expected) <= tolerance);
}

static WheelEncoderSample make_wheel_sample(uint64_t timestamp_us,
                                             int32_t delta_count,
                                             float speed_mps)
{
    WheelEncoderSample sample = {0};

    sample.timestamp_us = timestamp_us;
    sample.delta_count = delta_count;
    sample.speed_mps = speed_mps;
    sample.valid = true;
    return sample;
}

static ImuSample make_imu_sample(uint64_t timestamp_us, float gyro_z_radps)
{
    ImuSample sample = {0};

    sample.timestamp_us = timestamp_us;
    sample.angular_rate_radps[2] = gyro_z_radps;
    sample.temperature_c = 25.0f;
    sample.accel_gyro_valid = true;
    return sample;
}

static void feed_diagnostic_text(VehicleDiagnostics *diagnostics,
                                 const char *text)
{
    if((diagnostics == NULL) || (text == NULL))
    {
        return;
    }
    while(*text != '\0')
    {
        vehicle_diagnostics_feed_byte(diagnostics, (uint8_t)*text);
        ++text;
    }
}

static bool run_localization_case(float speed_mps, float curvature_per_m,
                                  VehiclePose *result_pose)
{
    const float meter_per_count = 0.00001f;
    const float dt_s = 0.01f;
    const uint64_t first_timestamp_us = UINT64_C(1000000);
    const uint64_t second_timestamp_us = UINT64_C(1010000);
    VehicleLocalization localization;
    VehiclePose pose;
    WheelEncoderSample left;
    WheelEncoderSample right;
    ImuSample imu;
    float left_speed_mps;
    float right_speed_mps;
    float steering_angle_rad;

    if((result_pose == NULL) ||
       !vehicle_control_compute_wheel_targets_from_curvature(
           speed_mps, curvature_per_m, &left_speed_mps, &right_speed_mps) ||
       !vehicle_localization_init(&localization, meter_per_count,
                                  meter_per_count, 0.0f))
    {
        return false;
    }

    left = make_wheel_sample(first_timestamp_us, 0, 0.0f);
    right = make_wheel_sample(first_timestamp_us, 0, 0.0f);
    imu = make_imu_sample(first_timestamp_us, 0.0f);
    steering_angle_rad = atanf(curvature_per_m * VEHICLE_WHEELBASE_M);
    if(!vehicle_localization_update(&localization, first_timestamp_us,
                                    &left, &right, &imu,
                                    steering_angle_rad, true, &pose))
    {
        return false;
    }

    left = make_wheel_sample(
        second_timestamp_us,
        (int32_t)lroundf(left_speed_mps * dt_s / meter_per_count),
        left_speed_mps);
    right = make_wheel_sample(
        second_timestamp_us,
        (int32_t)lroundf(right_speed_mps * dt_s / meter_per_count),
        right_speed_mps);
    imu = make_imu_sample(second_timestamp_us,
                          speed_mps * curvature_per_m);
    if(!vehicle_localization_update(&localization, second_timestamp_us,
                                    &left, &right, &imu,
                                    steering_angle_rad, true, &pose))
    {
        return false;
    }

    *result_pose = pose;
    return true;
}

static void build_path(PathPoint *points, uint32_t point_count,
                       float curvature_per_m, float spacing_m)
{
    uint32_t index;

    for(index = 0U; index < point_count; ++index)
    {
        float arc_length_m = spacing_m * (float)index;
        float yaw_rad = curvature_per_m * arc_length_m;

        if(fabsf(curvature_per_m) < 1.0e-6f)
        {
            points[index].x = arc_length_m;
            points[index].y = 0.0f;
        }
        else
        {
            points[index].x = sinf(yaw_rad) / curvature_per_m;
            points[index].y = (1.0f - cosf(yaw_rad)) / curvature_per_m;
        }
        points[index].yaw_body = yaw_rad;
        points[index].s = arc_length_m;
        points[index].curvature_forward = curvature_per_m;
        points[index].recorded_speed = 0.8f;
    }
}

static uint32_t make_mt6701_ssi_frame(uint16_t raw_angle,
                                      uint8_t magnetic_status)
{
    uint32_t payload =
        ((uint32_t)(raw_angle & VEHICLE_MT6701_ANGLE_MASK) << 4U) |
        (uint32_t)(magnetic_status & UINT8_C(0x0F));

    return (payload << 6U) | (uint32_t)vehicle_mt6701_crc6(payload);
}

static void build_s_path(PathPoint *points, uint32_t point_count,
                         float spacing_m, float peak_curvature_per_m)
{
    float total_length_m;
    float x_m = 0.0f;
    float y_m = 0.0f;
    float yaw_rad = 0.0f;
    uint32_t index;

    if((points == NULL) || (point_count < 2U))
    {
        return;
    }
    total_length_m = spacing_m * (float)(point_count - 1U);
    for(index = 0U; index < point_count; ++index)
    {
        float arc_length_m = spacing_m * (float)index;

        if(index > 0U)
        {
            float midpoint_s_m = spacing_m * ((float)index - 0.5f);
            float midpoint_curvature_per_m = peak_curvature_per_m *
                sinf(VEHICLE_TWO_PI_F * midpoint_s_m / total_length_m);
            float midpoint_yaw_rad = yaw_rad +
                0.5f * midpoint_curvature_per_m * spacing_m;

            x_m += spacing_m * cosf(midpoint_yaw_rad);
            y_m += spacing_m * sinf(midpoint_yaw_rad);
            yaw_rad += midpoint_curvature_per_m * spacing_m;
        }
        points[index].x = x_m;
        points[index].y = y_m;
        points[index].yaw_body = yaw_rad;
        points[index].s = arc_length_m;
        points[index].curvature_forward = peak_curvature_per_m *
            sinf(VEHICLE_TWO_PI_F * arc_length_m / total_length_m);
        points[index].recorded_speed = 0.8f;
    }
}

static void build_near_closed_path(PathPoint *points, uint32_t point_count,
                                   float radius_m,
                                   float endpoint_gap_angle_rad)
{
    float total_angle_rad;
    uint32_t index;

    if((points == NULL) || (point_count < 2U) || !(radius_m > 0.0f))
    {
        return;
    }
    total_angle_rad = VEHICLE_TWO_PI_F - endpoint_gap_angle_rad;
    for(index = 0U; index < point_count; ++index)
    {
        float ratio = (float)index / (float)(point_count - 1U);
        float angle_rad = total_angle_rad * ratio;

        points[index].x = radius_m * sinf(angle_rad);
        points[index].y = radius_m * (1.0f - cosf(angle_rad));
        points[index].yaw_body = angle_rad;
        points[index].s = radius_m * angle_rad;
        points[index].curvature_forward = 1.0f / radius_m;
        points[index].recorded_speed = 0.8f;
    }
}

static bool tracker_at_path_end(float curvature_per_m,
                                ReverseTrackerOutput *output)
{
    enum { TRACKER_POINT_COUNT = 41 };
    static PathPoint path[TRACKER_POINT_COUNT];
    VehicleReverseTracker tracker;
    VehicleReverseTrackerInput input;
    VehicleReverseTrackerStatus status;

    if(output == NULL)
    {
        return false;
    }
    build_path(path, TRACKER_POINT_COUNT, curvature_per_m, 0.05f);
    status = vehicle_reverse_tracker_init(&tracker, NULL, path,
                                          TRACKER_POINT_COUNT);
    if(status != VEHICLE_REVERSE_TRACKER_ACTIVE)
    {
        return false;
    }

    input.x_m = path[TRACKER_POINT_COUNT - 1U].x;
    input.y_m = path[TRACKER_POINT_COUNT - 1U].y;
    input.body_yaw_rad = path[TRACKER_POINT_COUNT - 1U].yaw_body;
    input.speed_mps = 0.0f;
    input.dt_s = 0.01f;
    input.localization_valid = true;
    status = vehicle_reverse_tracker_update(&tracker, &input, output);
    return (status == VEHICLE_REVERSE_TRACKER_ACTIVE) && output->valid;
}

static bool test_01_encoder_forward_reverse(void)
{
    VehicleEncoder encoder;
    VehicleEncoder reverse_sign_encoder;
    WheelEncoderSample sample;

    TEST_REQUIRE(vehicle_encoder_init(&encoder, 1, 0.001f));
    TEST_REQUIRE(vehicle_encoder_update(&encoder, 100U, UINT64_C(1000),
                                        &sample));
    TEST_REQUIRE(vehicle_encoder_update(&encoder, 110U, UINT64_C(11000),
                                        &sample));
    TEST_REQUIRE(sample.delta_count == 10);
    TEST_REQUIRE(sample.count == 10);
    TEST_REQUIRE(nearly_equal(sample.speed_mps, 1.0f, 1.0e-5f));
    TEST_REQUIRE(vehicle_encoder_update(&encoder, 105U, UINT64_C(21000),
                                        &sample));
    TEST_REQUIRE(sample.delta_count == -5);
    TEST_REQUIRE(sample.count == 5);
    TEST_REQUIRE(nearly_equal(sample.speed_mps, -0.5f, 1.0e-5f));

    TEST_REQUIRE(vehicle_encoder_init(&reverse_sign_encoder, -1, 0.001f));
    TEST_REQUIRE(vehicle_encoder_update(&reverse_sign_encoder, 100U,
                                        UINT64_C(1000), &sample));
    TEST_REQUIRE(vehicle_encoder_update(&reverse_sign_encoder, 110U,
                                        UINT64_C(11000), &sample));
    TEST_REQUIRE(sample.delta_count == -10);
    TEST_REQUIRE(sample.speed_mps < 0.0f);
    return true;
}

static bool test_02_encoder_counter_overflow(void)
{
    VehicleEncoder encoder;
    WheelEncoderSample sample;

    TEST_REQUIRE(vehicle_encoder_delta16(3U, 65534U) == 5);
    TEST_REQUIRE(vehicle_encoder_delta16(65534U, 3U) == -5);
    TEST_REQUIRE(vehicle_encoder_init(&encoder, 1, 0.001f));
    TEST_REQUIRE(vehicle_encoder_update(&encoder, 65534U, UINT64_C(1000),
                                        &sample));
    TEST_REQUIRE(vehicle_encoder_update(&encoder, 3U, UINT64_C(11000),
                                        &sample));
    TEST_REQUIRE(sample.valid && sample.delta_count == 5 &&
                 sample.count == 5);
    TEST_REQUIRE(vehicle_encoder_update(&encoder, 65534U, UINT64_C(21000),
                                        &sample));
    TEST_REQUIRE(sample.valid && sample.delta_count == -5 &&
                 sample.count == 0);
    return true;
}

static bool test_03_mt6701_cross_zero(void)
{
    VehicleMt6701 sensor;
    SteeringSample sample;

    vehicle_mt6701_init(&sensor);
    TEST_REQUIRE(vehicle_mt6701_update_angle(&sensor, 16380U, 0U,
                                             UINT64_C(1000), true, &sample));
    TEST_REQUIRE(vehicle_mt6701_update_angle(&sensor, 3U, 0U,
                                             UINT64_C(2000), true, &sample));
    TEST_REQUIRE(sample.continuous_count == 16387);
    TEST_REQUIRE(sample.relative_count == 7);
    TEST_REQUIRE(vehicle_mt6701_update_angle(&sensor, 16380U, 0U,
                                             UINT64_C(3000), true, &sample));
    TEST_REQUIRE(sample.continuous_count == 16380);
    TEST_REQUIRE(sample.relative_count == 0);
    return true;
}

static bool test_04_mt6701_multiturn_return_zero(void)
{
    VehicleMt6701 sensor;
    SteeringSample sample;
    uint64_t timestamp_us = UINT64_C(1000);
    int32_t step;

    vehicle_mt6701_init(&sensor);
    TEST_REQUIRE(vehicle_mt6701_update_angle(&sensor, 100U, 0U,
                                             timestamp_us, true, &sample));
    TEST_REQUIRE(vehicle_mt6701_set_relative_zero(&sensor));
    for(step = 1; step <= 32; ++step)
    {
        uint16_t raw = (uint16_t)((100 + step * 1024) & 0x3FFF);
        timestamp_us += UINT64_C(1000);
        TEST_REQUIRE(vehicle_mt6701_update_angle(&sensor, raw, 0U,
                                                 timestamp_us, true,
                                                 &sample));
    }
    TEST_REQUIRE(sample.relative_count == 32768);
    for(step = 31; step >= 0; --step)
    {
        uint16_t raw = (uint16_t)((100 + step * 1024) & 0x3FFF);
        timestamp_us += UINT64_C(1000);
        TEST_REQUIRE(vehicle_mt6701_update_angle(&sensor, raw, 0U,
                                                 timestamp_us, true,
                                                 &sample));
    }
    TEST_REQUIRE(sample.continuous_count == 100);
    TEST_REQUIRE(sample.relative_count == 0);
    TEST_REQUIRE(nearly_equal(sample.angle_rad, 0.0f, 1.0e-6f));
    return true;
}

static bool test_05_steering_interpolation_and_limits(void)
{
    const SteeringCalibrationPoint points[] =
    {
        { -1000, -0.5f },
        { 0, 0.0f },
        { 1000, 0.5f }
    };
    float angle_rad;
    int64_t count;

    TEST_REQUIRE(vehicle_steering_table_count_to_angle(
        points, ARRAY_COUNT(points), 500, &angle_rad));
    TEST_REQUIRE(nearly_equal(angle_rad, 0.25f, 1.0e-6f));
    TEST_REQUIRE(vehicle_steering_table_count_to_angle(
        points, ARRAY_COUNT(points), -5000, &angle_rad));
    TEST_REQUIRE(nearly_equal(angle_rad, -0.5f, 1.0e-6f));
    TEST_REQUIRE(vehicle_steering_table_count_to_angle(
        points, ARRAY_COUNT(points), 5000, &angle_rad));
    TEST_REQUIRE(nearly_equal(angle_rad, 0.5f, 1.0e-6f));

    TEST_REQUIRE(vehicle_steering_table_angle_to_count(
        points, ARRAY_COUNT(points), 0.25f, &count));
    TEST_REQUIRE(count == 500);
    TEST_REQUIRE(vehicle_steering_table_angle_to_count(
        points, ARRAY_COUNT(points), -2.0f, &count));
    TEST_REQUIRE(count == -1000);
    TEST_REQUIRE(vehicle_steering_table_angle_to_count(
        points, ARRAY_COUNT(points), 2.0f, &count));
    TEST_REQUIRE(count == 1000);
    return true;
}

static bool test_06_localization_straight_forward_reverse(void)
{
    VehiclePose forward_pose;
    VehiclePose reverse_pose;

    TEST_REQUIRE(run_localization_case(1.0f, 0.0f, &forward_pose));
    TEST_REQUIRE(run_localization_case(-1.0f, 0.0f, &reverse_pose));
    TEST_REQUIRE(forward_pose.valid && reverse_pose.valid);
    TEST_REQUIRE(nearly_equal(forward_pose.x_m, 0.01f, 2.0e-5f));
    TEST_REQUIRE(nearly_equal(reverse_pose.x_m, -0.01f, 2.0e-5f));
    TEST_REQUIRE(nearly_equal(forward_pose.y_m, 0.0f, 1.0e-6f));
    TEST_REQUIRE(nearly_equal(reverse_pose.y_m, 0.0f, 1.0e-6f));
    TEST_REQUIRE(nearly_equal(forward_pose.yaw_rad, 0.0f, 1.0e-6f));
    TEST_REQUIRE(nearly_equal(reverse_pose.yaw_rad, 0.0f, 1.0e-6f));
    return true;
}

static bool test_07_localization_fixed_curvature_forward_reverse(void)
{
    VehiclePose forward_pose;
    VehiclePose reverse_pose;
    const float curvature_per_m = 0.5f;

    TEST_REQUIRE(run_localization_case(1.0f, curvature_per_m,
                                       &forward_pose));
    TEST_REQUIRE(run_localization_case(-1.0f, curvature_per_m,
                                       &reverse_pose));
    TEST_REQUIRE(forward_pose.yaw_rad > 0.0f);
    TEST_REQUIRE(reverse_pose.yaw_rad < 0.0f);
    TEST_REQUIRE(nearly_equal(forward_pose.yaw_rad, 0.005f, 2.0e-4f));
    TEST_REQUIRE(nearly_equal(reverse_pose.yaw_rad, -0.005f, 2.0e-4f));
    TEST_REQUIRE(forward_pose.x_m > 0.0f);
    TEST_REQUIRE(reverse_pose.x_m < 0.0f);
    return true;
}

static bool test_08_path_capacity(void)
{
    static PathPoint points[PATH_MAX_POINTS];
    VehiclePathInfo info;
    uint32_t index;

    for(index = 0U; index < PATH_MAX_POINTS; ++index)
    {
        points[index].x = 0.001f * (float)index;
        points[index].y = 0.0f;
        points[index].yaw_body = 0.0f;
        points[index].s = 0.001f * (float)index;
        points[index].curvature_forward = 0.0f;
        points[index].recorded_speed = 0.5f;
    }
    TEST_REQUIRE(vehicle_path_load_raw_points(points, PATH_MAX_POINTS) ==
                 VEHICLE_PATH_RESULT_OK);
    TEST_REQUIRE(vehicle_path_get_count() == PATH_MAX_POINTS);
    TEST_REQUIRE(vehicle_path_load_raw_points(points, PATH_MAX_POINTS + 1U) ==
                 VEHICLE_PATH_RESULT_OVERFLOW);
    info = vehicle_path_get_info();
    TEST_REQUIRE(info.overflowed);
    TEST_REQUIRE(info.point_count == 0U);
    return true;
}

static bool test_09_path_yaw_crosses_pi(void)
{
    enum { RAW_POINT_COUNT = 12 };
    PathPoint raw[RAW_POINT_COUNT];
    const PathPoint *processed;
    uint32_t count;
    uint32_t index;

    for(index = 0U; index < RAW_POINT_COUNT; ++index)
    {
        raw[index].x = 0.04f * (float)index;
        raw[index].y = 0.0f;
        raw[index].yaw_body = vehicle_wrap_pi(3.05f + 0.035f * (float)index);
        raw[index].s = 0.04f * (float)index;
        raw[index].curvature_forward = 0.0f;
        raw[index].recorded_speed = 0.5f;
    }
    TEST_REQUIRE(vehicle_path_load_raw_points(raw, RAW_POINT_COUNT) ==
                 VEHICLE_PATH_RESULT_OK);
    TEST_REQUIRE(vehicle_path_preprocess() == VEHICLE_PATH_RESULT_OK);
    processed = vehicle_path_get_points();
    count = vehicle_path_get_count();
    TEST_REQUIRE(vehicle_path_is_valid());
    TEST_REQUIRE(count >= PATH_MIN_VALID_POINTS);
    TEST_REQUIRE(processed[count - 1U].yaw_body > VEHICLE_PI_F);
    for(index = 1U; index < count; ++index)
    {
        TEST_REQUIRE(fabsf(processed[index].yaw_body -
                           processed[index - 1U].yaw_body) < 0.10f);
    }
    return true;
}

static bool test_10_path_resampling(void)
{
    static const float x_positions[] =
    {
        0.00f, 0.07f, 0.11f, 0.20f,
        0.29f, 0.37f, 0.50f, 0.65f
    };
    PathPoint raw[ARRAY_COUNT(x_positions)];
    const PathPoint *processed;
    uint32_t count;
    uint32_t index;

    for(index = 0U; index < ARRAY_COUNT(x_positions); ++index)
    {
        raw[index].x = x_positions[index];
        raw[index].y = 0.0f;
        raw[index].yaw_body = 0.0f;
        raw[index].s = x_positions[index];
        raw[index].curvature_forward = 0.0f;
        raw[index].recorded_speed = 0.6f;
    }
    TEST_REQUIRE(vehicle_path_load_raw_points(
                     raw, (uint32_t)ARRAY_COUNT(raw)) ==
                 VEHICLE_PATH_RESULT_OK);
    TEST_REQUIRE(vehicle_path_preprocess() == VEHICLE_PATH_RESULT_OK);
    processed = vehicle_path_get_points();
    count = vehicle_path_get_count();
    TEST_REQUIRE(count > ARRAY_COUNT(raw));
    TEST_REQUIRE(nearly_equal(processed[0].s, 0.0f, 1.0e-6f));
    TEST_REQUIRE(nearly_equal(processed[count - 1U].s, 0.65f, 1.0e-4f));
    for(index = 1U; index < count; ++index)
    {
        float spacing_m = processed[index].s - processed[index - 1U].s;
        TEST_REQUIRE(spacing_m >= 0.025f && spacing_m <= 0.035f);
        TEST_REQUIRE(processed[index].x > processed[index - 1U].x);
    }
    return true;
}

static bool test_11_reverse_path_index_traversal(void)
{
    enum { POINT_COUNT = 41 };
    PathPoint path[POINT_COUNT];
    const uint32_t visit_indices[] = { 40U, 35U, 30U, 25U, 20U, 15U, 10U, 5U };
    VehicleReverseTracker tracker;
    VehicleReverseTrackerInput input;
    ReverseTrackerOutput output;
    uint32_t previous_index = POINT_COUNT - 1U;
    uint32_t visit;

    build_path(path, POINT_COUNT, 0.0f, 0.05f);
    TEST_REQUIRE(vehicle_reverse_tracker_init(&tracker, NULL, path,
                                              POINT_COUNT) ==
                 VEHICLE_REVERSE_TRACKER_ACTIVE);
    for(visit = 0U; visit < ARRAY_COUNT(visit_indices); ++visit)
    {
        uint32_t index = visit_indices[visit];

        input.x_m = path[index].x;
        input.y_m = path[index].y;
        input.body_yaw_rad = path[index].yaw_body;
        input.speed_mps = (visit == 0U) ? 0.0f : -0.2f;
        input.dt_s = 0.01f;
        input.localization_valid = true;
        TEST_REQUIRE(vehicle_reverse_tracker_update(&tracker, &input,
                                                    &output) ==
                     VEHICLE_REVERSE_TRACKER_ACTIVE);
        TEST_REQUIRE(output.valid);
        TEST_REQUIRE(output.nearest_index <= previous_index);
        TEST_REQUIRE(output.target_index <= output.nearest_index);
        TEST_REQUIRE(output.target_speed_mps < 0.0f);
        previous_index = output.nearest_index;
    }
    TEST_REQUIRE(previous_index <= visit_indices[ARRAY_COUNT(visit_indices) - 1U]);
    return true;
}

static bool test_12_reverse_tracker_straight_sign(void)
{
    ReverseTrackerOutput output;

    TEST_REQUIRE(tracker_at_path_end(0.0f, &output));
    TEST_REQUIRE(output.target_speed_mps < 0.0f);
    TEST_REQUIRE(fabsf(output.target_steering_rad) < 1.0e-4f);
    return true;
}

static bool test_13_reverse_tracker_left_curve_sign(void)
{
    ReverseTrackerOutput output;

    TEST_REQUIRE(tracker_at_path_end(0.5f, &output));
    TEST_REQUIRE(output.target_speed_mps < 0.0f);
    TEST_REQUIRE(output.target_steering_rad > 0.05f);
    return true;
}

static bool test_14_reverse_tracker_right_curve_sign(void)
{
    ReverseTrackerOutput output;

    TEST_REQUIRE(tracker_at_path_end(-0.5f, &output));
    TEST_REQUIRE(output.target_speed_mps < 0.0f);
    TEST_REQUIRE(output.target_steering_rad < -0.05f);
    return true;
}

static bool test_15_sensor_fault_enters_fault_state(void)
{
    const uint64_t timestamp_us = UINT64_C(1000000);
    VehicleSafetyMonitor safety;
    VehicleFaultManager fault_manager;
    VehicleSafetyInputs inputs;
    VehicleStateMachine machine;
    VehicleStateMachineEvents events;
    VehicleActuatorCommand actuators;

    vehicle_safety_init(&safety, NULL, NULL, NULL);
    vehicle_fault_init(&fault_manager);
    memset(&inputs, 0, sizeof(inputs));
    inputs.timestamp_us = timestamp_us;
    inputs.state = VEHICLE_STATE_STAGE1_RECORD;
    inputs.left_wheel = make_wheel_sample(timestamp_us, 0, 0.0f);
    inputs.right_wheel = make_wheel_sample(timestamp_us, 0, 0.0f);
    inputs.mt6701_communication_ok = true;
    inputs.steering.timestamp_us = timestamp_us;
    inputs.steering.valid = true;
    inputs.imu = make_imu_sample(timestamp_us, 0.0f);
    inputs.imu_communication_ok = false;
    inputs.last_control_timestamp_us = timestamp_us;
    inputs.numeric_valid = true;
    inputs.path_capacity = PATH_MAX_POINTS;
    vehicle_safety_update(&safety, &fault_manager, &inputs);

    TEST_REQUIRE(vehicle_fault_has_active(&fault_manager));
    TEST_REQUIRE((fault_manager.active_flags & VEHICLE_FAULT_IMU_COMM) != 0U);
    TEST_REQUIRE(fault_manager.immediate_stop);

    vehicle_state_machine_init(&machine, 0U);
    memset(&events, 0, sizeof(events));
    events.boot_complete = true;
    events.critical_drivers_ready = true;
    TEST_REQUIRE(vehicle_state_machine_update(&machine, &events, 1U));
    memset(&events, 0, sizeof(events));
    events.sensors_calibrated = true;
    TEST_REQUIRE(vehicle_state_machine_update(&machine, &events, 2U));
    memset(&events, 0, sizeof(events));
    events.request_stage1_start = true;
    events.calibration_valid = true;
    TEST_REQUIRE(vehicle_state_machine_update(&machine, &events, 3U));
    TEST_REQUIRE(machine.state == VEHICLE_STATE_STAGE1_RECORD);
    memset(&events, 0, sizeof(events));
    events.fault_active = vehicle_fault_has_active(&fault_manager);
    TEST_REQUIRE(vehicle_state_machine_update(&machine, &events, 4U));
    TEST_REQUIRE(machine.state == VEHICLE_STATE_FAULT);

    actuators.left_motor_duty = 0.5f;
    actuators.right_motor_duty = -0.5f;
    actuators.steering_motor_duty = 0.2f;
    actuators.immediate_stop = false;
    vehicle_safety_gate_actuators(&safety, &fault_manager, &actuators);
    TEST_REQUIRE(nearly_equal(actuators.left_motor_duty, 0.0f, 0.0f));
    TEST_REQUIRE(nearly_equal(actuators.right_motor_duty, 0.0f, 0.0f));
    TEST_REQUIRE(nearly_equal(actuators.steering_motor_duty, 0.0f, 0.0f));
    TEST_REQUIRE(actuators.immediate_stop);
    return true;
}

static bool test_16_invalid_calibration_forces_zero_output(void)
{
    VehicleControl control;
    VehicleControlInput input;
    VehicleControlOutput output;
    VehicleSafetyMonitor safety;
    VehicleFaultManager fault_manager;
    VehicleActuatorCommand actuators;
    SteeringCalibrationTables tables =
        vehicle_calibration_get_steering_tables();

    TEST_REQUIRE(CALIBRATION_VALID == false);
    TEST_REQUIRE(!vehicle_calibration_is_valid(&g_vehicle_calibration,
                                               &tables));
    vehicle_control_init(&control, &g_vehicle_calibration, &tables);
    input.target_center_speed_mps = 1.0f;
    input.target_steering_rad = 0.2f;
    input.left_speed_mps = 0.0f;
    input.right_speed_mps = 0.0f;
    input.steering_continuous_count = 0;
    input.dt_s = 0.01f;
    vehicle_control_update(&control, &input, &output);
    TEST_REQUIRE(!output.valid);
    TEST_REQUIRE(nearly_equal(output.actuators.left_motor_duty, 0.0f, 0.0f));
    TEST_REQUIRE(nearly_equal(output.actuators.right_motor_duty, 0.0f, 0.0f));
    TEST_REQUIRE(nearly_equal(output.actuators.steering_motor_duty, 0.0f, 0.0f));

    vehicle_safety_init(&safety, NULL, &g_vehicle_calibration, &tables);
    vehicle_fault_init(&fault_manager);
    actuators.left_motor_duty = 0.4f;
    actuators.right_motor_duty = 0.4f;
    actuators.steering_motor_duty = -0.2f;
    actuators.immediate_stop = false;
    vehicle_safety_gate_actuators(&safety, &fault_manager, &actuators);
    TEST_REQUIRE(nearly_equal(actuators.left_motor_duty, 0.0f, 0.0f));
    TEST_REQUIRE(nearly_equal(actuators.right_motor_duty, 0.0f, 0.0f));
    TEST_REQUIRE(nearly_equal(actuators.steering_motor_duty, 0.0f, 0.0f));
    return true;
}

static bool test_17_mt6701_ssi_crc_and_magnetic_rejection(void)
{
    VehicleMt6701 sensor;
    VehicleMt6701Frame decoded;
    SteeringSample sample;
    uint32_t frame_before_zero = make_mt6701_ssi_frame(16380U, 0U);
    uint32_t frame_after_zero = make_mt6701_ssi_frame(3U, 0U);
    uint32_t bad_crc_frame = frame_after_zero ^ UINT32_C(1);
    uint32_t bad_magnetic_frame = make_mt6701_ssi_frame(4U, 1U);
    int64_t count_before_rejections;

    TEST_REQUIRE(vehicle_mt6701_parse_ssi_frame(frame_before_zero, &decoded));
    TEST_REQUIRE(decoded.crc_valid);
    TEST_REQUIRE(decoded.raw_angle == 16380U);
    TEST_REQUIRE(decoded.magnetic_status == 0U);
    TEST_REQUIRE(!vehicle_mt6701_parse_ssi_frame(bad_crc_frame, &decoded));
    TEST_REQUIRE(!decoded.crc_valid);

    vehicle_mt6701_init(&sensor);
    TEST_REQUIRE(vehicle_mt6701_update_ssi(&sensor, frame_before_zero,
                                           UINT64_C(1000), true, &sample));
    TEST_REQUIRE(vehicle_mt6701_update_ssi(&sensor, frame_after_zero,
                                           UINT64_C(2000), true, &sample));
    TEST_REQUIRE(sample.valid && sample.relative_count == 7);
    count_before_rejections = sample.continuous_count;

    TEST_REQUIRE(!vehicle_mt6701_update_ssi(&sensor, bad_crc_frame,
                                            UINT64_C(3000), true, &sample));
    TEST_REQUIRE(!sample.valid);
    TEST_REQUIRE(sensor.communication_error_count == 1U);
    TEST_REQUIRE(sensor.continuous_count == count_before_rejections);

    TEST_REQUIRE(vehicle_mt6701_parse_ssi_frame(bad_magnetic_frame,
                                                &decoded));
    TEST_REQUIRE(decoded.crc_valid && decoded.magnetic_status == 1U);
    TEST_REQUIRE(!vehicle_mt6701_update_ssi(&sensor, bad_magnetic_frame,
                                            UINT64_C(4000), true, &sample));
    TEST_REQUIRE(!sample.valid);
    TEST_REQUIRE(sensor.magnetic_error_count == 1U);
    TEST_REQUIRE(sensor.continuous_count == count_before_rejections);
    return true;
}

static bool test_18_online_path_recording_capacity_boundary(void)
{
    VehiclePathSample sample;
    VehiclePathInfo info;
    const PathPoint *points;
    VehiclePathResult result;
    uint32_t count;

    sample.x_m = 0.0f;
    sample.y_m = 0.0f;
    sample.yaw_rad = 0.0f;
    sample.speed_mps = 1.0f;
    TEST_REQUIRE(vehicle_path_start_recording(&sample) ==
                 VEHICLE_PATH_RESULT_OK);

    sample.x_m = RECORD_MAX_DISTANCE_M + 0.10f;
    result = vehicle_path_record_sample(&sample);
    TEST_REQUIRE(result == VEHICLE_PATH_RESULT_DISTANCE_LIMIT);
    info = vehicle_path_get_info();
    count = vehicle_path_get_count();
    points = vehicle_path_get_points();
    TEST_REQUIRE(info.distance_limit_reached);
    TEST_REQUIRE(!info.recording);
    TEST_REQUIRE(!info.overflowed);
    TEST_REQUIRE(count > 3900U);
    TEST_REQUIRE(count <= PATH_MAX_POINTS);
    TEST_REQUIRE(points != NULL);
    TEST_REQUIRE(points[count - 1U].s <= RECORD_MAX_DISTANCE_M + 0.001f);
    TEST_REQUIRE(points[count - 1U].s >=
                 RECORD_MAX_DISTANCE_M - PATH_RECORD_SPACING_M - 0.001f);
    TEST_REQUIRE(vehicle_path_record_sample(&sample) ==
                 VEHICLE_PATH_RESULT_NOT_RECORDING);
    return true;
}

static bool exercise_steering_soft_limits(
    const SteeringCalibrationPoint *points,
    size_t point_count,
    int64_t physical_left_limit_count,
    int64_t physical_right_limit_count)
{
    VehicleCalibration calibration = {0};
    SteeringCalibrationTables tables;
    VehicleSteeringControllerConfig config;
    VehicleSteeringController controller;
    VehicleSteeringControlOutput output;

    calibration.steering_left_soft_limit_count = physical_left_limit_count;
    calibration.steering_right_soft_limit_count = physical_right_limit_count;
    calibration.steering_start_duty_left = 0.0f;
    calibration.steering_start_duty_right = 0.0f;
    tables.from_left = points;
    tables.from_left_count = point_count;
    tables.from_right = points;
    tables.from_right_count = point_count;
    vehicle_steering_controller_default_config(&config);
    config.kp = 1.0f;
    config.ki = 0.0f;
    config.kd = 0.0f;
    config.integral_limit = 0.5f;
    config.output_limit = 0.5f;
    config.output_slew_per_s = 100.0f;
    config.position_deadband_rad = 0.001f;
    vehicle_steering_controller_init(&controller, &calibration, &tables,
                                     &config);

    /* Isolate the limiter from the global uncommissioned-calibration gate. */
    controller.calibration_valid = true;
    vehicle_steering_controller_update(&controller, 0.0f,
                                       physical_left_limit_count,
                                       0.01f, &output);
    TEST_REQUIRE(output.valid);
    TEST_REQUIRE(!output.at_soft_limit);
    TEST_REQUIRE(output.signed_duty < 0.0f);

    vehicle_steering_controller_reset(&controller);
    controller.integral = 0.4f;
    vehicle_steering_controller_update(&controller, 0.45f,
                                       physical_left_limit_count,
                                       0.01f, &output);
    TEST_REQUIRE(output.valid);
    TEST_REQUIRE(output.at_soft_limit);
    TEST_REQUIRE(nearly_equal(output.signed_duty, 0.0f, 0.0f));

    vehicle_steering_controller_reset(&controller);
    vehicle_steering_controller_update(&controller, 0.0f,
                                       physical_right_limit_count,
                                       0.01f, &output);
    TEST_REQUIRE(output.valid);
    TEST_REQUIRE(!output.at_soft_limit);
    TEST_REQUIRE(output.signed_duty > 0.0f);

    vehicle_steering_controller_reset(&controller);
    controller.integral = -0.4f;
    vehicle_steering_controller_update(&controller, -0.45f,
                                       physical_right_limit_count,
                                       0.01f, &output);
    TEST_REQUIRE(output.valid);
    TEST_REQUIRE(output.at_soft_limit);
    TEST_REQUIRE(nearly_equal(output.signed_duty, 0.0f, 0.0f));
    return true;
}

static bool test_19_steering_controller_soft_limit_direction_gate(void)
{
    static const SteeringCalibrationPoint positive_slope_points[] =
    {
        { -2000, -1.0f },
        { 0, 0.0f },
        { 2000, 1.0f }
    };
    static const SteeringCalibrationPoint negative_slope_points[] =
    {
        { -2000, 1.0f },
        { 0, 0.0f },
        { 2000, -1.0f }
    };

    TEST_REQUIRE(exercise_steering_soft_limits(
        positive_slope_points, ARRAY_COUNT(positive_slope_points),
        1000, -1000));
    TEST_REQUIRE(exercise_steering_soft_limits(
        negative_slope_points, ARRAY_COUNT(negative_slope_points),
        -1000, 1000));
    return true;
}

static bool test_20_s_curve_reverse_index_sign_and_feedback(void)
{
    enum { POINT_COUNT = 81 };
    static const uint32_t visit_indices[] =
    {
        80U, 72U, 64U, 56U, 48U, 40U, 32U, 24U, 16U, 8U, 4U
    };
    PathPoint path[POINT_COUNT];
    VehicleReverseTracker tracker;
    VehicleReverseTracker feedback_center_tracker;
    VehicleReverseTracker feedback_left_tracker;
    VehicleReverseTracker feedback_right_tracker;
    VehicleReverseTrackerConfig feedback_config;
    VehicleReverseTrackerInput input;
    ReverseTrackerOutput output;
    ReverseTrackerOutput center_output;
    ReverseTrackerOutput left_output;
    ReverseTrackerOutput right_output;
    uint32_t previous_index = POINT_COUNT - 1U;
    uint32_t visit;
    bool saw_left_turn = false;
    bool saw_right_turn = false;
    const uint32_t feedback_index = 40U;
    const float lateral_offset_m = 0.08f;
    float normal_x;
    float normal_y;

    build_s_path(path, POINT_COUNT, 0.05f, 0.6f);
    TEST_REQUIRE(vehicle_path_validate_points(path, POINT_COUNT,
                                              NULL, NULL) ==
                 VEHICLE_PATH_RESULT_OK);
    TEST_REQUIRE(vehicle_reverse_tracker_init(&tracker, NULL, path,
                                              POINT_COUNT) ==
                 VEHICLE_REVERSE_TRACKER_ACTIVE);
    for(visit = 0U; visit < ARRAY_COUNT(visit_indices); ++visit)
    {
        uint32_t index = visit_indices[visit];

        input.x_m = path[index].x;
        input.y_m = path[index].y;
        input.body_yaw_rad = path[index].yaw_body;
        input.speed_mps = (visit == 0U) ? 0.0f : -0.3f;
        input.dt_s = 0.01f;
        input.localization_valid = true;
        TEST_REQUIRE(vehicle_reverse_tracker_update(&tracker, &input,
                                                    &output) ==
                     VEHICLE_REVERSE_TRACKER_ACTIVE);
        TEST_REQUIRE(output.valid);
        TEST_REQUIRE(output.nearest_index <= previous_index);
        TEST_REQUIRE(output.target_index <= output.nearest_index);
        TEST_REQUIRE(output.target_speed_mps < 0.0f);
        if(path[output.nearest_index].curvature_forward > 0.35f)
        {
            TEST_REQUIRE(output.target_steering_rad > 0.02f);
            saw_left_turn = true;
        }
        if(path[output.nearest_index].curvature_forward < -0.35f)
        {
            TEST_REQUIRE(output.target_steering_rad < -0.02f);
            saw_right_turn = true;
        }
        previous_index = output.nearest_index;
    }
    TEST_REQUIRE(saw_left_turn && saw_right_turn);
    TEST_REQUIRE(previous_index <=
                 visit_indices[ARRAY_COUNT(visit_indices) - 1U]);

    vehicle_reverse_tracker_default_config(&feedback_config);
    feedback_config.curvature_feedforward_gain = 0.0f;
    feedback_config.heading_feedback_gain = 0.0f;
    TEST_REQUIRE(vehicle_reverse_tracker_init(&feedback_center_tracker,
                                              &feedback_config, path,
                                              POINT_COUNT) ==
                 VEHICLE_REVERSE_TRACKER_ACTIVE);
    TEST_REQUIRE(vehicle_reverse_tracker_init(&feedback_left_tracker,
                                              &feedback_config, path,
                                              POINT_COUNT) ==
                 VEHICLE_REVERSE_TRACKER_ACTIVE);
    TEST_REQUIRE(vehicle_reverse_tracker_init(&feedback_right_tracker,
                                              &feedback_config, path,
                                              POINT_COUNT) ==
                 VEHICLE_REVERSE_TRACKER_ACTIVE);

    input.x_m = path[feedback_index].x;
    input.y_m = path[feedback_index].y;
    input.body_yaw_rad = path[feedback_index].yaw_body;
    input.speed_mps = 0.0f;
    input.dt_s = 0.01f;
    input.localization_valid = true;
    TEST_REQUIRE(vehicle_reverse_tracker_update(&feedback_center_tracker,
                                                &input, &center_output) ==
                 VEHICLE_REVERSE_TRACKER_ACTIVE);
    normal_x = -sinf(path[feedback_index].yaw_body);
    normal_y = cosf(path[feedback_index].yaw_body);
    input.x_m = path[feedback_index].x + lateral_offset_m * normal_x;
    input.y_m = path[feedback_index].y + lateral_offset_m * normal_y;
    TEST_REQUIRE(vehicle_reverse_tracker_update(&feedback_left_tracker,
                                                &input, &left_output) ==
                 VEHICLE_REVERSE_TRACKER_ACTIVE);
    input.x_m = path[feedback_index].x - lateral_offset_m * normal_x;
    input.y_m = path[feedback_index].y - lateral_offset_m * normal_y;
    TEST_REQUIRE(vehicle_reverse_tracker_update(&feedback_right_tracker,
                                                &input, &right_output) ==
                 VEHICLE_REVERSE_TRACKER_ACTIVE);
    TEST_REQUIRE(center_output.valid && left_output.valid &&
                 right_output.valid);
    TEST_REQUIRE(left_output.cross_track_error_m < 0.0f);
    TEST_REQUIRE(right_output.cross_track_error_m > 0.0f);
    TEST_REQUIRE(left_output.target_steering_rad <
                 center_output.target_steering_rad);
    TEST_REQUIRE(right_output.target_steering_rad >
                 center_output.target_steering_rad);
    TEST_REQUIRE(left_output.target_steering_rad < -0.01f);
    TEST_REQUIRE(right_output.target_steering_rad > 0.01f);
    return true;
}

static bool test_21_near_closed_path_finish_requires_progress(void)
{
    enum { POINT_COUNT = 81 };
    static const uint32_t approach_indices[] = { 60U, 40U, 20U, 5U };
    PathPoint path[POINT_COUNT];
    VehicleReverseTracker tracker;
    VehicleReverseTracker off_finish_tracker;
    VehicleReverseTrackerInput input;
    ReverseTrackerOutput output;
    VehicleReverseTrackerStatus status;
    VehicleReverseTrackerConfig config;
    float endpoint_distance_m;
    uint32_t visit;

    build_near_closed_path(path, POINT_COUNT, 0.65f, 0.08f);
    TEST_REQUIRE(vehicle_path_validate_points(path, POINT_COUNT,
                                              NULL, NULL) ==
                 VEHICLE_PATH_RESULT_OK);
    vehicle_reverse_tracker_default_config(&config);
    endpoint_distance_m = hypotf(path[POINT_COUNT - 1U].x - path[0].x,
                                 path[POINT_COUNT - 1U].y - path[0].y);
    TEST_REQUIRE(endpoint_distance_m < config.finish_distance_m);

    TEST_REQUIRE(vehicle_reverse_tracker_init(&tracker, &config, path,
                                              POINT_COUNT) ==
                 VEHICLE_REVERSE_TRACKER_ACTIVE);
    input.x_m = path[POINT_COUNT - 1U].x;
    input.y_m = path[POINT_COUNT - 1U].y;
    input.body_yaw_rad = path[POINT_COUNT - 1U].yaw_body;
    input.speed_mps = 0.0f;
    input.dt_s = 0.01f;
    input.localization_valid = true;
    status = vehicle_reverse_tracker_update(&tracker, &input, &output);
    TEST_REQUIRE(status == VEHICLE_REVERSE_TRACKER_ACTIVE);
    TEST_REQUIRE(!output.finished);
    TEST_REQUIRE(output.nearest_index == POINT_COUNT - 1U);

    for(visit = 0U; visit < ARRAY_COUNT(approach_indices); ++visit)
    {
        uint32_t index = approach_indices[visit];

        input.x_m = path[index].x;
        input.y_m = path[index].y;
        input.body_yaw_rad = path[index].yaw_body;
        input.speed_mps = -0.3f;
        status = vehicle_reverse_tracker_update(&tracker, &input, &output);
        TEST_REQUIRE(status == VEHICLE_REVERSE_TRACKER_ACTIVE);
        TEST_REQUIRE(!output.finished);
        TEST_REQUIRE(output.nearest_index == index);
    }
    input.x_m = path[0].x;
    input.y_m = path[0].y;
    input.body_yaw_rad = path[0].yaw_body;
    input.speed_mps = 0.0f;
    status = vehicle_reverse_tracker_update(&tracker, &input, &output);
    TEST_REQUIRE(status == VEHICLE_REVERSE_TRACKER_FINISHED);
    TEST_REQUIRE(output.finished);
    TEST_REQUIRE(output.nearest_index == 0U);

    TEST_REQUIRE(vehicle_reverse_tracker_init(&off_finish_tracker, &config,
                                              path, POINT_COUNT) ==
                 VEHICLE_REVERSE_TRACKER_ACTIVE);
    input.x_m = path[0].x;
    input.y_m = path[0].y;
    input.body_yaw_rad = path[0].yaw_body;
    input.speed_mps = -0.5f;
    status = vehicle_reverse_tracker_update(&off_finish_tracker, &input,
                                            &output);
    TEST_REQUIRE(status == VEHICLE_REVERSE_TRACKER_ACTIVE);
    TEST_REQUIRE(off_finish_tracker.progress_index == 0U);
    TEST_REQUIRE(!output.finished);

    input.x_m = path[0].x + 0.30f;
    input.y_m = path[0].y;
    input.speed_mps = 0.0f;
    status = vehicle_reverse_tracker_update(&off_finish_tracker, &input,
                                            &output);
    TEST_REQUIRE(status == VEHICLE_REVERSE_TRACKER_TRACKING_ERROR);
    TEST_REQUIRE(status != VEHICLE_REVERSE_TRACKER_ACTIVE);
    TEST_REQUIRE(!output.finished);
    return true;
}

static bool test_22_invalid_encoder_faults_without_history(void)
{
    const uint64_t timestamp_us = UINT64_C(2000000);
    VehicleSafetyMonitor safety;
    VehicleFaultManager fault_manager;
    VehicleSafetyInputs inputs;

    vehicle_safety_init(&safety, NULL, NULL, NULL);
    vehicle_fault_init(&fault_manager);
    /* Isolate the runtime monitor from the commissioning gate. */
    safety.calibration_valid = true;
    safety.automatic_output_inhibited = false;
    safety.left_meter_per_count = 0.001f;
    safety.right_meter_per_count = 0.001f;

    memset(&inputs, 0, sizeof(inputs));
    inputs.timestamp_us = timestamp_us;
    inputs.state = VEHICLE_STATE_IDLE;
    inputs.left_wheel.timestamp_us = timestamp_us;
    inputs.left_wheel.valid = false;
    inputs.right_wheel = make_wheel_sample(timestamp_us, 0, 0.0f);
    inputs.mt6701_communication_ok = true;
    inputs.steering.timestamp_us = timestamp_us;
    inputs.steering.valid = true;
    inputs.imu_communication_ok = true;
    inputs.imu = make_imu_sample(timestamp_us, 0.0f);
    inputs.last_control_timestamp_us = timestamp_us;
    inputs.numeric_valid = true;
    inputs.path_capacity = PATH_MAX_POINTS;

    TEST_REQUIRE(!safety.previous_left_valid);
    vehicle_safety_update(&safety, &fault_manager, &inputs);
    TEST_REQUIRE((fault_manager.active_flags &
                  VEHICLE_FAULT_LEFT_ENCODER_JUMP) != 0U);
    TEST_REQUIRE((fault_manager.active_flags &
                  VEHICLE_FAULT_RIGHT_ENCODER_JUMP) == 0U);
    TEST_REQUIRE(fault_manager.active_flags ==
                 VEHICLE_FAULT_LEFT_ENCODER_JUMP);
    TEST_REQUIRE(fault_manager.immediate_stop);
    TEST_REQUIRE(safety.automatic_output_inhibited);
    return true;
}

static bool exercise_steering_approach_hysteresis(
    const SteeringCalibrationPoint *from_left_points,
    const SteeringCalibrationPoint *from_right_points,
    int64_t physical_left_limit_count,
    int64_t physical_right_limit_count,
    int64_t from_left_target_count,
    int64_t from_right_target_count,
    int64_t physical_left_start_count,
    int64_t just_past_from_left_count,
    int64_t crossed_to_physical_right_count,
    int64_t physical_right_start_count,
    int64_t just_past_from_right_count,
    int64_t crossed_to_physical_left_count)
{
    VehicleCalibration calibration = {0};
    SteeringCalibrationTables tables;
    VehicleSteeringControllerConfig config;
    VehicleSteeringController controller;
    VehicleSteeringControlOutput output;
    int64_t interpolated_count;

    calibration.steering_left_soft_limit_count = physical_left_limit_count;
    calibration.steering_right_soft_limit_count = physical_right_limit_count;
    calibration.steering_start_duty_left = 0.0f;
    calibration.steering_start_duty_right = 0.0f;
    tables.from_left = from_left_points;
    tables.from_left_count = 3U;
    tables.from_right = from_right_points;
    tables.from_right_count = 3U;
    vehicle_steering_controller_default_config(&config);
    config.approach_switch_hysteresis_count = 10;
    vehicle_steering_controller_init(&controller, &calibration, &tables,
                                     &config);
    controller.calibration_valid = true;

    TEST_REQUIRE(vehicle_steering_table_angle_to_count(
        from_left_points, 3U, 0.0f, &interpolated_count));
    TEST_REQUIRE(interpolated_count == from_left_target_count);
    TEST_REQUIRE(vehicle_steering_table_angle_to_count(
        from_right_points, 3U, 0.0f, &interpolated_count));
    TEST_REQUIRE(interpolated_count == from_right_target_count);

    controller.approach = VEHICLE_STEERING_APPROACH_FROM_LEFT;
    vehicle_steering_controller_update(&controller, 0.0f,
                                       physical_left_start_count,
                                       0.01f, &output);
    TEST_REQUIRE(output.valid);
    TEST_REQUIRE(output.approach == VEHICLE_STEERING_APPROACH_FROM_LEFT);
    vehicle_steering_controller_update(&controller, 0.0f,
                                       just_past_from_left_count,
                                       0.01f, &output);
    TEST_REQUIRE(output.valid);
    TEST_REQUIRE(output.approach == VEHICLE_STEERING_APPROACH_FROM_LEFT);
    vehicle_steering_controller_update(&controller, 0.0f,
                                       crossed_to_physical_right_count,
                                       0.01f, &output);
    TEST_REQUIRE(output.valid);
    TEST_REQUIRE(output.approach == VEHICLE_STEERING_APPROACH_FROM_RIGHT);

    vehicle_steering_controller_reset(&controller);
    controller.approach = VEHICLE_STEERING_APPROACH_FROM_RIGHT;
    vehicle_steering_controller_update(&controller, 0.0f,
                                       physical_right_start_count,
                                       0.01f, &output);
    TEST_REQUIRE(output.valid);
    TEST_REQUIRE(output.approach == VEHICLE_STEERING_APPROACH_FROM_RIGHT);
    vehicle_steering_controller_update(&controller, 0.0f,
                                       just_past_from_right_count,
                                       0.01f, &output);
    TEST_REQUIRE(output.valid);
    TEST_REQUIRE(output.approach == VEHICLE_STEERING_APPROACH_FROM_RIGHT);
    vehicle_steering_controller_update(&controller, 0.0f,
                                       crossed_to_physical_left_count,
                                       0.01f, &output);
    TEST_REQUIRE(output.valid);
    TEST_REQUIRE(output.approach == VEHICLE_STEERING_APPROACH_FROM_LEFT);
    return true;
}

static bool test_23_steering_approach_hysteresis_both_slopes(void)
{
    static const SteeringCalibrationPoint positive_from_left[] =
    {
        { -2000, -1.01f },
        { 0, -0.01f },
        { 2000, 0.99f }
    };
    static const SteeringCalibrationPoint positive_from_right[] =
    {
        { -2000, -0.99f },
        { 0, 0.01f },
        { 2000, 1.01f }
    };
    static const SteeringCalibrationPoint negative_from_left[] =
    {
        { -2000, 0.99f },
        { 0, -0.01f },
        { 2000, -1.01f }
    };
    static const SteeringCalibrationPoint negative_from_right[] =
    {
        { -2000, 1.01f },
        { 0, 0.01f },
        { 2000, -0.99f }
    };

    TEST_REQUIRE(exercise_steering_approach_hysteresis(
        positive_from_left, positive_from_right,
        1000, -1000, 20, -20,
        40, 15, 0, -40, -15, 0));
    TEST_REQUIRE(exercise_steering_approach_hysteresis(
        negative_from_left, negative_from_right,
        -1000, 1000, -20, 20,
        -40, -15, 0, 40, 15, 0));
    return true;
}

static bool test_24_wheel_mismatch_requests_controlled_stop(void)
{
    const uint64_t first_timestamp_us = UINT64_C(3000000);
    VehicleSafetyConfig config;
    VehicleSafetyMonitor safety;
    VehicleFaultManager fault_manager;
    VehicleSafetyInputs inputs;
    VehicleActuatorCommand actuators;
    unsigned int cycle;

    vehicle_safety_default_config(&config);
    config.encoder_max_acceleration_mps2 = 1000.0f;
    config.wheel_mismatch_min_speed_mps = 0.10f;
    config.wheel_mismatch_max_mps = 0.20f;
    config.wheel_mismatch_timeout_s = 0.015f;
    vehicle_safety_init(&safety, &config, NULL, NULL);
    vehicle_fault_init(&fault_manager);
    /* Isolate runtime routing from the uncommissioned calibration gate. */
    safety.calibration_valid = true;
    safety.automatic_output_inhibited = false;
    safety.left_meter_per_count = 0.001f;
    safety.right_meter_per_count = 0.001f;

    memset(&inputs, 0, sizeof(inputs));
    inputs.state = VEHICLE_STATE_STAGE1_RECORD;
    inputs.left_target_speed_mps = 0.5f;
    inputs.right_target_speed_mps = 0.5f;
    inputs.mt6701_communication_ok = true;
    inputs.steering.valid = true;
    inputs.imu_communication_ok = true;
    inputs.numeric_valid = true;
    inputs.path_capacity = PATH_MAX_POINTS;

    for(cycle = 0U; cycle < 3U; ++cycle)
    {
        uint64_t timestamp_us = first_timestamp_us +
            (uint64_t)cycle * UINT64_C(10000);

        inputs.timestamp_us = timestamp_us;
        inputs.left_wheel = make_wheel_sample(timestamp_us, 10, 1.0f);
        inputs.right_wheel = make_wheel_sample(timestamp_us, 0, 0.0f);
        inputs.steering.timestamp_us = timestamp_us;
        inputs.imu = make_imu_sample(timestamp_us, 0.0f);
        inputs.last_control_timestamp_us = timestamp_us;
        vehicle_safety_update(&safety, &fault_manager, &inputs);
    }

    TEST_REQUIRE((fault_manager.latched_flags &
                  VEHICLE_FAULT_WHEEL_MISMATCH) != 0U);
    TEST_REQUIRE(fault_manager.active_flags == VEHICLE_FAULT_NONE);
    TEST_REQUIRE(!fault_manager.immediate_stop);
    TEST_REQUIRE(vehicle_safety_get_controlled_stop_request_flags(&safety) ==
                 VEHICLE_FAULT_WHEEL_MISMATCH);
    TEST_REQUIRE((safety.monitored_active_flags &
                  VEHICLE_FAULT_WHEEL_MISMATCH) != 0U);
    TEST_REQUIRE(!safety.automatic_output_inhibited);
    TEST_REQUIRE(vehicle_safety_output_is_allowed(&safety, &fault_manager));

    actuators.left_motor_duty = 0.3f;
    actuators.right_motor_duty = 0.2f;
    actuators.steering_motor_duty = -0.1f;
    actuators.immediate_stop = false;
    vehicle_safety_gate_actuators(&safety, &fault_manager, &actuators);
    TEST_REQUIRE(nearly_equal(actuators.left_motor_duty, 0.3f, 0.0f));
    TEST_REQUIRE(nearly_equal(actuators.right_motor_duty, 0.2f, 0.0f));
    TEST_REQUIRE(nearly_equal(actuators.steering_motor_duty, -0.1f, 0.0f));
    TEST_REQUIRE(!actuators.immediate_stop);

    inputs.timestamp_us += UINT64_C(10000);
    inputs.left_target_speed_mps = 1.0f;
    inputs.right_target_speed_mps = 0.0f;
    inputs.left_wheel = make_wheel_sample(inputs.timestamp_us, 10, 1.0f);
    inputs.right_wheel = make_wheel_sample(inputs.timestamp_us, 0, 0.0f);
    inputs.steering.timestamp_us = inputs.timestamp_us;
    inputs.imu = make_imu_sample(inputs.timestamp_us, 0.0f);
    inputs.last_control_timestamp_us = inputs.timestamp_us;
    vehicle_safety_update(&safety, &fault_manager, &inputs);

    TEST_REQUIRE(vehicle_safety_get_controlled_stop_request_flags(&safety) ==
                 VEHICLE_FAULT_NONE);
    TEST_REQUIRE((safety.monitored_active_flags &
                  VEHICLE_FAULT_WHEEL_MISMATCH) == 0U);
    TEST_REQUIRE((fault_manager.latched_flags &
                  VEHICLE_FAULT_WHEEL_MISMATCH) != 0U);
    TEST_REQUIRE(fault_manager.active_flags == VEHICLE_FAULT_NONE);
    TEST_REQUIRE(!fault_manager.immediate_stop);
    TEST_REQUIRE(vehicle_safety_output_is_allowed(&safety, &fault_manager));
    return true;
}

static bool test_25_diagnostic_drive_parse_and_queue(void)
{
    static const char *const invalid_commands[] =
    {
        "DRIVE -0.01 0\r",
        "DRIVE 0.301 0\r",
        "DRIVE 0.1 0.351\r",
        "DRIVE 0.1 -0.351\r",
        "DRIVE NAN 0\r",
        "DRIVE 0 nan\r",
        "DRIVE 0.1\r",
        "DRIVE \r",
        "DRIVE 0.1 0.2 EXTRA\r"
    };
    VehicleDiagnostics diagnostics;
    VehicleDiagnosticRequest request;
    char response[VEHICLE_COMMAND_BUFFER_SIZE];
    size_t index;

    vehicle_diagnostics_init(&diagnostics);
    TEST_REQUIRE(!diagnostics.request_pending);
    TEST_REQUIRE(diagnostics.pending_request.action == VEHICLE_DIAG_NONE);

    feed_diagnostic_text(&diagnostics, "dRiVe 0.30 -0.35\r");
    TEST_REQUIRE(diagnostics.request_pending);
    TEST_REQUIRE(vehicle_diagnostics_take_response(
        &diagnostics, response, sizeof(response)));
    TEST_REQUIRE(strcmp(response, "OK\r\n") == 0);

    /* A full one-entry queue must retain the first command. */
    feed_diagnostic_text(&diagnostics, "DRIVE 0.10 0.20\n");
    TEST_REQUIRE(diagnostics.request_pending);
    TEST_REQUIRE(vehicle_diagnostics_take_response(
        &diagnostics, response, sizeof(response)));
    TEST_REQUIRE(strcmp(response, "ERR command queue busy\r\n") == 0);
    TEST_REQUIRE(vehicle_diagnostics_take_request(&diagnostics, &request));
    TEST_REQUIRE(request.action == VEHICLE_DIAG_STAGE1_DRIVE);
    TEST_REQUIRE(nearly_equal(request.signed_duty, 0.0f, 0.0f));
    TEST_REQUIRE(nearly_equal(request.target_speed_mps, 0.30f, 1.0e-6f));
    TEST_REQUIRE(nearly_equal(request.target_steering_rad, -0.35f,
                              1.0e-6f));
    TEST_REQUIRE(!vehicle_diagnostics_take_request(&diagnostics, &request));
    TEST_REQUIRE(!diagnostics.request_pending);
    TEST_REQUIRE(diagnostics.pending_request.action == VEHICLE_DIAG_NONE);
    TEST_REQUIRE(nearly_equal(
        diagnostics.pending_request.target_speed_mps, 0.0f, 0.0f));
    TEST_REQUIRE(nearly_equal(
        diagnostics.pending_request.target_steering_rad, 0.0f, 0.0f));

    /* A new mixed-case command is accepted after consuming the queue. */
    feed_diagnostic_text(&diagnostics, "DrIvE 0.125 +0.25\r");
    TEST_REQUIRE(vehicle_diagnostics_take_request(&diagnostics, &request));
    TEST_REQUIRE(request.action == VEHICLE_DIAG_STAGE1_DRIVE);
    TEST_REQUIRE(nearly_equal(request.target_speed_mps, 0.125f, 1.0e-6f));
    TEST_REQUIRE(nearly_equal(request.target_steering_rad, 0.25f,
                              1.0e-6f));
    TEST_REQUIRE(vehicle_diagnostics_take_response(
        &diagnostics, response, sizeof(response)));
    TEST_REQUIRE(strcmp(response, "OK\r\n") == 0);

    for(index = 0U; index < ARRAY_COUNT(invalid_commands); ++index)
    {
        request.action = VEHICLE_DIAG_PRINT_CONFIG;
        request.signed_duty = 1.0f;
        request.target_speed_mps = 1.0f;
        request.target_steering_rad = 1.0f;
        feed_diagnostic_text(&diagnostics, invalid_commands[index]);
        TEST_REQUIRE(!vehicle_diagnostics_take_request(&diagnostics,
                                                       &request));
        TEST_REQUIRE(request.action == VEHICLE_DIAG_PRINT_CONFIG);
        TEST_REQUIRE(nearly_equal(request.target_speed_mps, 1.0f, 0.0f));
        TEST_REQUIRE(nearly_equal(request.target_steering_rad, 1.0f, 0.0f));
        TEST_REQUIRE(vehicle_diagnostics_take_response(
            &diagnostics, response, sizeof(response)));
        TEST_REQUIRE(strncmp(response, "ERR ", 4U) == 0);
        TEST_REQUIRE(!diagnostics.request_pending);
    }
    return true;
}

static bool test_26_diagnostic_stop_preempts_motion(void)
{
    VehicleDiagnostics diagnostics;
    VehicleDiagnosticRequest request;
    char response[VEHICLE_COMMAND_BUFFER_SIZE];

    vehicle_diagnostics_init(&diagnostics);
    feed_diagnostic_text(&diagnostics,
                         "DRIVE 0.20 0.10\nSTOP\n");
    TEST_REQUIRE(vehicle_diagnostics_take_response(
        &diagnostics, response, sizeof(response)));
    TEST_REQUIRE(strcmp(response, "OK\r\n") == 0);
    TEST_REQUIRE(vehicle_diagnostics_take_request(&diagnostics, &request));
    TEST_REQUIRE(request.action == VEHICLE_DIAG_STOP);
    TEST_REQUIRE(nearly_equal(request.signed_duty, 0.0f, 0.0f));
    TEST_REQUIRE(nearly_equal(request.target_speed_mps, 0.0f, 0.0f));
    TEST_REQUIRE(nearly_equal(request.target_steering_rad, 0.0f, 0.0f));

    vehicle_diagnostics_init(&diagnostics);
    feed_diagnostic_text(&diagnostics,
                         "STOP\nDRIVE 0.20 0.10\n");
    TEST_REQUIRE(vehicle_diagnostics_take_response(
        &diagnostics, response, sizeof(response)));
    TEST_REQUIRE(strcmp(response, "ERR command queue busy\r\n") == 0);
    TEST_REQUIRE(vehicle_diagnostics_take_request(&diagnostics, &request));
    TEST_REQUIRE(request.action == VEHICLE_DIAG_STOP);

    vehicle_diagnostics_init(&diagnostics);
    feed_diagnostic_text(&diagnostics,
                         "DRIVE 0.20 0.10\nSTAGE1 STOP\n");
    TEST_REQUIRE(vehicle_diagnostics_take_response(
        &diagnostics, response, sizeof(response)));
    TEST_REQUIRE(strcmp(response, "OK\r\n") == 0);
    TEST_REQUIRE(vehicle_diagnostics_take_request(&diagnostics, &request));
    TEST_REQUIRE(request.action == VEHICLE_DIAG_STAGE1_STOP);
    TEST_REQUIRE(nearly_equal(request.target_speed_mps, 0.0f, 0.0f));
    TEST_REQUIRE(nearly_equal(request.target_steering_rad, 0.0f, 0.0f));

    vehicle_diagnostics_init(&diagnostics);
    feed_diagnostic_text(&diagnostics,
                         "STOP\nSTAGE1 STOP\n");
    TEST_REQUIRE(vehicle_diagnostics_take_response(
        &diagnostics, response, sizeof(response)));
    TEST_REQUIRE(strcmp(response, "ERR command queue busy\r\n") == 0);
    TEST_REQUIRE(vehicle_diagnostics_take_request(&diagnostics, &request));
    TEST_REQUIRE(request.action == VEHICLE_DIAG_STOP);
    TEST_REQUIRE(!vehicle_diagnostics_take_request(&diagnostics, &request));
    return true;
}

int main(void)
{
    static const TestCase tests[] =
    {
        { "01 encoder forward/reverse", test_01_encoder_forward_reverse },
        { "02 encoder 16-bit overflow", test_02_encoder_counter_overflow },
        { "03 MT6701 zero crossing", test_03_mt6701_cross_zero },
        { "04 MT6701 multi-turn return to zero", test_04_mt6701_multiturn_return_zero },
        { "05 steering interpolation/limits", test_05_steering_interpolation_and_limits },
        { "06 straight localization forward/reverse", test_06_localization_straight_forward_reverse },
        { "07 fixed-curvature localization forward/reverse", test_07_localization_fixed_curvature_forward_reverse },
        { "08 path static capacity", test_08_path_capacity },
        { "09 yaw crossing +/-pi", test_09_path_yaw_crosses_pi },
        { "10 path resampling", test_10_path_resampling },
        { "11 reverse path index traversal", test_11_reverse_path_index_traversal },
        { "12 reverse tracker straight steering sign", test_12_reverse_tracker_straight_sign },
        { "13 reverse tracker left-curve steering sign", test_13_reverse_tracker_left_curve_sign },
        { "14 reverse tracker right-curve steering sign", test_14_reverse_tracker_right_curve_sign },
        { "15 sensor fault transitions to FAULT", test_15_sensor_fault_enters_fault_state },
        { "16 CALIBRATION_VALID=false zero output", test_16_invalid_calibration_forces_zero_output },
        { "17 MT6701 SSI CRC/magnetic rejection", test_17_mt6701_ssi_crc_and_magnetic_rejection },
        { "18 online path capacity boundary", test_18_online_path_recording_capacity_boundary },
        { "19 steering soft-limit direction gate", test_19_steering_controller_soft_limit_direction_gate },
        { "20 S-curve reverse traversal/feedback", test_20_s_curve_reverse_index_sign_and_feedback },
        { "21 near-closed path finish/progress gate", test_21_near_closed_path_finish_requires_progress },
        { "22 invalid encoder immediate safety fault", test_22_invalid_encoder_faults_without_history },
        { "23 steering approach hysteresis both slopes", test_23_steering_approach_hysteresis_both_slopes },
        { "24 wheel mismatch controlled-stop routing", test_24_wheel_mismatch_requests_controlled_stop },
        { "25 diagnostic DRIVE parsing/queue", test_25_diagnostic_drive_parse_and_queue },
        { "26 diagnostic STOP preemption", test_26_diagnostic_stop_preempts_motion }
    };
    size_t index;
    unsigned int passed = 0U;

    printf("TC387 vehicle host tests (C99)\n");
    for(index = 0U; index < ARRAY_COUNT(tests); ++index)
    {
        bool ok = tests[index].function();

        printf("[%s] %s\n", ok ? "PASS" : "FAIL", tests[index].name);
        if(ok)
        {
            ++passed;
        }
    }
    printf("Result: %u/%u passed\n", passed,
           (unsigned int)ARRAY_COUNT(tests));
    return (passed == ARRAY_COUNT(tests)) ? 0 : 1;
}

#endif
