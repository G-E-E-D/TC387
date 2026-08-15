#include "vehicle_localization.h"

#include <math.h>
#include <stddef.h>

#include "vehicle_config.h"
#include "vehicle_math.h"

static void vehicle_localization_increment_error(
    VehicleLocalization *localization)
{
    if(localization->invalid_update_count < UINT32_MAX)
    {
        ++localization->invalid_update_count;
    }
}

static void vehicle_localization_clear_diagnostics(
    VehicleLocalizationDiagnostics *diagnostics)
{
    diagnostics->imu_weight = 0.0f;
    diagnostics->wheel_weight = 0.0f;
    diagnostics->steering_weight = 0.0f;
    diagnostics->wheel_speed_residual_mps = 0.0f;
    diagnostics->imu_wheel_residual_radps = 0.0f;
    diagnostics->imu_steering_residual_radps = 0.0f;
    diagnostics->model_disagreement = false;
    diagnostics->imu_suspect = false;
}

static bool vehicle_localization_sensor_time_valid(uint64_t cycle_timestamp_us,
                                                   uint64_t sensor_timestamp_us)
{
    return (sensor_timestamp_us <= cycle_timestamp_us) &&
           ((cycle_timestamp_us - sensor_timestamp_us) <=
            VEHICLE_SENSOR_TIMEOUT_US);
}

static bool vehicle_localization_inputs_valid(
    const VehicleLocalization *localization, uint64_t timestamp_us,
    const WheelEncoderSample *left_wheel,
    const WheelEncoderSample *right_wheel, const ImuSample *imu,
    float steering_angle_rad, bool steering_valid)
{
    if((localization == NULL) || !localization->configured ||
       (left_wheel == NULL) || (right_wheel == NULL) || (imu == NULL) ||
       !left_wheel->valid || !right_wheel->valid ||
       !imu->accel_gyro_valid ||
       !vehicle_float_is_finite(left_wheel->speed_mps) ||
       !vehicle_float_is_finite(right_wheel->speed_mps) ||
       !vehicle_float_is_finite(imu->angular_rate_radps[2]))
    {
        return false;
    }
    if(!vehicle_localization_sensor_time_valid(timestamp_us,
                                               left_wheel->timestamp_us) ||
       !vehicle_localization_sensor_time_valid(timestamp_us,
                                               right_wheel->timestamp_us) ||
       !vehicle_localization_sensor_time_valid(timestamp_us,
                                               imu->timestamp_us))
    {
        return false;
    }
    if(steering_valid &&
       (!vehicle_float_is_finite(steering_angle_rad) ||
        (fabsf(steering_angle_rad) >= (0.5f * VEHICLE_PI_F))))
    {
        return false;
    }
    return true;
}

static float vehicle_localization_confidence(float residual)
{
    float magnitude;

    magnitude = fabsf(residual);
    if(magnitude <= LOCALIZATION_OMEGA_DISAGREE_RADPS)
    {
        return 1.0f;
    }
    return LOCALIZATION_OMEGA_DISAGREE_RADPS / magnitude;
}

static void vehicle_localization_mark_invalid(
    VehicleLocalization *localization, uint64_t timestamp_us,
    VehiclePose *pose)
{
    vehicle_localization_increment_error(localization);
    localization->pose.timestamp_us = timestamp_us;
    localization->pose.valid = false;
    if(pose != NULL)
    {
        *pose = localization->pose;
    }
}

bool vehicle_localization_init(VehicleLocalization *localization,
                               float left_meter_per_count,
                               float right_meter_per_count,
                               float initial_gyro_z_bias_radps)
{
    bool valid;

    if(localization == NULL)
    {
        return false;
    }

    valid = vehicle_float_is_finite(left_meter_per_count) &&
            vehicle_float_is_finite(right_meter_per_count) &&
            vehicle_float_is_finite(initial_gyro_z_bias_radps) &&
            (left_meter_per_count > 0.0f) &&
            (right_meter_per_count > 0.0f);
    localization->left_meter_per_count = left_meter_per_count;
    localization->right_meter_per_count = right_meter_per_count;
    localization->configured = valid;
    localization->invalid_update_count = 0U;
    vehicle_localization_reset(localization, 0.0f, 0.0f, 0.0f,
                               valid ? initial_gyro_z_bias_radps : 0.0f);
    return valid;
}

void vehicle_localization_reset(VehicleLocalization *localization,
                                float x_m, float y_m, float yaw_rad,
                                float gyro_z_bias_radps)
{
    if(localization == NULL)
    {
        return;
    }

    localization->pose.timestamp_us = 0U;
    localization->pose.x_m = vehicle_float_is_finite(x_m) ? x_m : 0.0f;
    localization->pose.y_m = vehicle_float_is_finite(y_m) ? y_m : 0.0f;
    localization->pose.yaw_rad = vehicle_float_is_finite(yaw_rad) ?
                                  yaw_rad : 0.0f;
    localization->pose.vehicle_speed_mps = 0.0f;
    localization->pose.gyro_z_bias_radps =
        vehicle_float_is_finite(gyro_z_bias_radps) ?
        gyro_z_bias_radps : 0.0f;
    localization->pose.omega_imu_radps = 0.0f;
    localization->pose.omega_wheel_radps = 0.0f;
    localization->pose.omega_steering_radps = 0.0f;
    localization->pose.stationary = true;
    localization->pose.wheel_slip = false;
    localization->pose.valid = false;
    localization->last_timestamp_us = 0U;
    localization->initialized = false;
    vehicle_localization_clear_diagnostics(&localization->diagnostics);
}

bool vehicle_localization_update(VehicleLocalization *localization,
                                 uint64_t timestamp_us,
                                 const WheelEncoderSample *left_wheel,
                                 const WheelEncoderSample *right_wheel,
                                 const ImuSample *imu,
                                 float steering_angle_rad,
                                 bool steering_valid,
                                 VehiclePose *pose)
{
    uint64_t delta_time_us;
    float delta_time_s;
    float distance_left_m;
    float distance_right_m;
    float distance_center_m;
    float speed_from_distance_mps;
    float speed_from_samples_mps;
    float omega_imu_radps;
    float omega_wheel_radps;
    float omega_steering_radps;
    float expected_wheel_difference_mps;
    float wheel_speed_difference_mps;
    float wheel_speed_residual_mps;
    float wheel_imu_residual_radps;
    float steering_imu_residual_radps;
    float wheel_steering_residual_radps;
    float imu_weight;
    float wheel_weight;
    float steering_weight;
    float total_weight;
    float fused_omega_radps;
    float yaw_mid_rad;
    float next_x_m;
    float next_y_m;
    float next_yaw_rad;
    float next_speed_mps;
    float bias_candidate_radps;
    bool stationary;
    bool wheel_slip;
    bool model_disagreement;
    bool imu_suspect;

    if((localization == NULL) || (pose == NULL))
    {
        return false;
    }

    if(!vehicle_localization_inputs_valid(localization, timestamp_us,
                                           left_wheel, right_wheel, imu,
                                           steering_angle_rad,
                                           steering_valid))
    {
        vehicle_localization_mark_invalid(localization, timestamp_us, pose);
        return false;
    }

    if(!localization->initialized)
    {
        localization->last_timestamp_us = timestamp_us;
        localization->pose.timestamp_us = timestamp_us;
        localization->pose.vehicle_speed_mps =
            0.5f * (left_wheel->speed_mps + right_wheel->speed_mps);
        localization->pose.valid = true;
        localization->initialized = true;
        *pose = localization->pose;
        return true;
    }

    if(timestamp_us <= localization->last_timestamp_us)
    {
        vehicle_localization_mark_invalid(localization, timestamp_us, pose);
        return false;
    }

    delta_time_us = timestamp_us - localization->last_timestamp_us;
    delta_time_s = (float)delta_time_us * 1.0e-6f;
    if((delta_time_s < LOCALIZATION_MIN_DT_S) ||
       (delta_time_s > LOCALIZATION_MAX_DT_S))
    {
        if(delta_time_s > LOCALIZATION_MAX_DT_S)
        {
            localization->last_timestamp_us = timestamp_us;
        }
        vehicle_localization_mark_invalid(localization, timestamp_us, pose);
        return false;
    }

    distance_left_m = (float)left_wheel->delta_count *
                      localization->left_meter_per_count;
    distance_right_m = (float)right_wheel->delta_count *
                       localization->right_meter_per_count;
    distance_center_m = 0.5f * (distance_left_m + distance_right_m);
    speed_from_distance_mps = distance_center_m / delta_time_s;
    speed_from_samples_mps = 0.5f * (left_wheel->speed_mps +
                                     right_wheel->speed_mps);
    omega_wheel_radps = (right_wheel->speed_mps - left_wheel->speed_mps) /
                        VEHICLE_REAR_TRACK_M;
    omega_steering_radps = steering_valid ?
        (speed_from_distance_mps * tanf(steering_angle_rad) /
         VEHICLE_WHEELBASE_M) : 0.0f;
    omega_imu_radps = imu->angular_rate_radps[2] -
                      localization->pose.gyro_z_bias_radps;

    if(!vehicle_float_is_finite(distance_center_m) ||
       !vehicle_float_is_finite(speed_from_distance_mps) ||
       !vehicle_float_is_finite(speed_from_samples_mps) ||
       !vehicle_float_is_finite(omega_wheel_radps) ||
       !vehicle_float_is_finite(omega_steering_radps) ||
       !vehicle_float_is_finite(omega_imu_radps))
    {
        vehicle_localization_mark_invalid(localization, timestamp_us, pose);
        return false;
    }

    expected_wheel_difference_mps = steering_valid ?
        (omega_steering_radps * VEHICLE_REAR_TRACK_M) :
        (omega_imu_radps * VEHICLE_REAR_TRACK_M);
    wheel_speed_difference_mps = right_wheel->speed_mps -
                                 left_wheel->speed_mps;
    wheel_speed_residual_mps = wheel_speed_difference_mps -
                               expected_wheel_difference_mps;
    wheel_imu_residual_radps = omega_wheel_radps - omega_imu_radps;
    steering_imu_residual_radps = steering_valid ?
        (omega_steering_radps - omega_imu_radps) : 0.0f;
    wheel_steering_residual_radps = steering_valid ?
        (omega_wheel_radps - omega_steering_radps) : 0.0f;

    stationary = (fabsf(left_wheel->speed_mps) <=
                  LOCALIZATION_STATIC_SPEED_MPS) &&
                 (fabsf(right_wheel->speed_mps) <=
                  LOCALIZATION_STATIC_SPEED_MPS) &&
                 (fabsf(omega_imu_radps) <=
                  LOCALIZATION_STATIC_GYRO_RADPS);
    wheel_slip = (fabsf(wheel_speed_residual_mps) >
                  LOCALIZATION_SLIP_SPEED_MPS) ||
                 (fabsf(speed_from_distance_mps - speed_from_samples_mps) >
                  LOCALIZATION_SLIP_SPEED_MPS);
    model_disagreement =
        (fabsf(wheel_imu_residual_radps) >
         LOCALIZATION_OMEGA_DISAGREE_RADPS) ||
        (steering_valid &&
         ((fabsf(steering_imu_residual_radps) >
           LOCALIZATION_OMEGA_DISAGREE_RADPS) ||
          (fabsf(wheel_steering_residual_radps) >
           LOCALIZATION_OMEGA_DISAGREE_RADPS)));
    imu_suspect = steering_valid &&
        (fabsf(wheel_steering_residual_radps) <=
         (0.5f * LOCALIZATION_OMEGA_DISAGREE_RADPS)) &&
        (fabsf(wheel_imu_residual_radps) >
         LOCALIZATION_OMEGA_DISAGREE_RADPS) &&
        (fabsf(steering_imu_residual_radps) >
         LOCALIZATION_OMEGA_DISAGREE_RADPS);

    if(stationary)
    {
        localization->pose.gyro_z_bias_radps +=
            IMU_BIAS_STATIONARY_GAIN *
            (imu->angular_rate_radps[2] -
             localization->pose.gyro_z_bias_radps);
        distance_center_m = 0.0f;
        speed_from_distance_mps = 0.0f;
    }
    else if(!model_disagreement && !wheel_slip)
    {
        if(steering_valid)
        {
            bias_candidate_radps = imu->angular_rate_radps[2] -
                (0.67f * omega_wheel_radps +
                 0.33f * omega_steering_radps);
        }
        else
        {
            bias_candidate_radps = imu->angular_rate_radps[2] -
                                    omega_wheel_radps;
        }
        localization->pose.gyro_z_bias_radps +=
            IMU_BIAS_MOVING_GAIN *
            (bias_candidate_radps -
             localization->pose.gyro_z_bias_radps);
    }
    omega_imu_radps = imu->angular_rate_radps[2] -
                      localization->pose.gyro_z_bias_radps;

    imu_weight = imu_suspect ? 0.10f : 1.0f;
    wheel_weight = LOCALIZATION_WHEEL_OMEGA_GAIN *
                   vehicle_localization_confidence(
                       omega_wheel_radps - omega_imu_radps);
    if(wheel_slip)
    {
        wheel_weight *= 0.05f;
    }
    steering_weight = steering_valid ?
        (LOCALIZATION_STEER_OMEGA_GAIN *
         vehicle_localization_confidence(
             omega_steering_radps - omega_imu_radps)) : 0.0f;
    total_weight = imu_weight + wheel_weight + steering_weight;
    if(!(total_weight > 0.0f) || !vehicle_float_is_finite(total_weight))
    {
        vehicle_localization_mark_invalid(localization, timestamp_us, pose);
        return false;
    }
    imu_weight /= total_weight;
    wheel_weight /= total_weight;
    steering_weight /= total_weight;
    fused_omega_radps = imu_weight * omega_imu_radps +
                        wheel_weight * omega_wheel_radps +
                        steering_weight * omega_steering_radps;

    yaw_mid_rad = localization->pose.yaw_rad +
                  0.5f * fused_omega_radps * delta_time_s;
    next_x_m = localization->pose.x_m +
               distance_center_m * cosf(yaw_mid_rad);
    next_y_m = localization->pose.y_m +
               distance_center_m * sinf(yaw_mid_rad);
    next_yaw_rad = localization->pose.yaw_rad +
                   fused_omega_radps * delta_time_s;
    next_speed_mps = stationary ? 0.0f :
        (localization->pose.vehicle_speed_mps +
         LOCALIZATION_SPEED_FILTER_GAIN *
         (speed_from_distance_mps -
          localization->pose.vehicle_speed_mps));

    if(!vehicle_float_is_finite(next_x_m) ||
       !vehicle_float_is_finite(next_y_m) ||
       !vehicle_float_is_finite(next_yaw_rad) ||
       !vehicle_float_is_finite(next_speed_mps) ||
       !vehicle_float_is_finite(fused_omega_radps) ||
       !vehicle_float_is_finite(localization->pose.gyro_z_bias_radps))
    {
        vehicle_localization_mark_invalid(localization, timestamp_us, pose);
        return false;
    }

    localization->pose.timestamp_us = timestamp_us;
    localization->pose.x_m = next_x_m;
    localization->pose.y_m = next_y_m;
    localization->pose.yaw_rad = next_yaw_rad;
    localization->pose.vehicle_speed_mps = next_speed_mps;
    localization->pose.omega_imu_radps = omega_imu_radps;
    localization->pose.omega_wheel_radps = omega_wheel_radps;
    localization->pose.omega_steering_radps = omega_steering_radps;
    localization->pose.stationary = stationary;
    localization->pose.wheel_slip = wheel_slip;
    localization->pose.valid = true;
    localization->last_timestamp_us = timestamp_us;

    localization->diagnostics.imu_weight = imu_weight;
    localization->diagnostics.wheel_weight = wheel_weight;
    localization->diagnostics.steering_weight = steering_weight;
    localization->diagnostics.wheel_speed_residual_mps =
        wheel_speed_residual_mps;
    localization->diagnostics.imu_wheel_residual_radps =
        omega_imu_radps - omega_wheel_radps;
    localization->diagnostics.imu_steering_residual_radps =
        steering_valid ? (omega_imu_radps - omega_steering_radps) : 0.0f;
    localization->diagnostics.model_disagreement = model_disagreement;
    localization->diagnostics.imu_suspect = imu_suspect;

    *pose = localization->pose;
    return true;
}

const VehicleLocalizationDiagnostics *vehicle_localization_get_diagnostics(
    const VehicleLocalization *localization)
{
    return (localization != NULL) ? &localization->diagnostics : NULL;
}
