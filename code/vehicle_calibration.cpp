#include "vehicle_calibration.h"

#include <math.h>

/*
 * This is the only file that may contain unmeasured vehicle data.
 * Replace every MEASURE_REQUIRED value, populate both steering tables, then
 * set CALIBRATION_VALID to true in vehicle_calibration.h.
 */
const VehicleCalibration g_vehicle_calibration =
{
    CALIBRATION_VALID,

    MEASURE_REQUIRED 0,
    MEASURE_REQUIRED 0,
    MEASURE_REQUIRED 0.0f,
    MEASURE_REQUIRED 0.0f,
    MEASURE_REQUIRED 0,
    MEASURE_REQUIRED 0,

    MEASURE_REQUIRED 0,
    MEASURE_REQUIRED 0,
    MEASURE_REQUIRED 0.0f,
    MEASURE_REQUIRED 0.0f,
    MEASURE_REQUIRED 0.0f,
    MEASURE_REQUIRED 0.0f,

    MEASURE_REQUIRED 0,
    MEASURE_REQUIRED 0,
    MEASURE_REQUIRED 0.0f,
    MEASURE_REQUIRED 0.0f,
    MEASURE_REQUIRED 0,
    MEASURE_REQUIRED 0,

    MEASURE_REQUIRED 0.0f,
    MEASURE_REQUIRED 0.0f,
    MEASURE_REQUIRED 0.0f,
    MEASURE_REQUIRED 0.0f,
    MEASURE_REQUIRED 0.0f,
    MEASURE_REQUIRED 0.0f,
    MEASURE_REQUIRED 0.0f,
    MEASURE_REQUIRED 0.0f,
    MEASURE_REQUIRED 0.0f,
    MEASURE_REQUIRED 0.0f,
    MEASURE_REQUIRED 0.0f,
    MEASURE_REQUIRED 0.0f,
    MEASURE_REQUIRED 0.0f,

    MEASURE_REQUIRED 0.0f,
    MEASURE_REQUIRED 0.0f,
    MEASURE_REQUIRED 0.0f,
    { MEASURE_REQUIRED 0, MEASURE_REQUIRED 1, MEASURE_REQUIRED 2 },
    { MEASURE_REQUIRED 1, MEASURE_REQUIRED 1, MEASURE_REQUIRED 1 },

    MEASURE_REQUIRED 0.0f,
    MEASURE_REQUIRED 0.0f,
    MEASURE_REQUIRED 0.0f
};

const SteeringCalibrationPoint g_steering_calibration_from_left[] =
{
    { MEASURE_REQUIRED 0, MEASURE_REQUIRED 0.0f },
    { MEASURE_REQUIRED 0, MEASURE_REQUIRED 0.0f }
};

const size_t g_steering_calibration_from_left_count =
    sizeof(g_steering_calibration_from_left) /
    sizeof(g_steering_calibration_from_left[0]);

const SteeringCalibrationPoint g_steering_calibration_from_right[] =
{
    { MEASURE_REQUIRED 0, MEASURE_REQUIRED 0.0f },
    { MEASURE_REQUIRED 0, MEASURE_REQUIRED 0.0f }
};

const size_t g_steering_calibration_from_right_count =
    sizeof(g_steering_calibration_from_right) /
    sizeof(g_steering_calibration_from_right[0]);

static bool sign_is_valid(int8_t value)
{
    return (value == -1) || (value == 1);
}

static bool duty_is_valid(float value)
{
    return isfinite(value) && (value >= 0.0f) && (value <= 1.0f);
}

static bool guide_geometry_is_valid(const VehicleCalibration *calibration)
{
    return (calibration != nullptr) &&
           isfinite(calibration->guide_tag_width_m) &&
           isfinite(calibration->camera_fx_px) &&
           isfinite(calibration->camera_fy_px) &&
           isfinite(calibration->camera_cx_px) &&
           isfinite(calibration->camera_cy_px) &&
           isfinite(calibration->camera_position_x_m) &&
           isfinite(calibration->camera_position_y_m) &&
           isfinite(calibration->camera_yaw_rad) &&
           isfinite(calibration->guide_follow_distance_m) &&
           isfinite(calibration->guide_min_safe_distance_m) &&
           isfinite(calibration->stage1_max_speed_mps) &&
           isfinite(calibration->stage1_acceleration_mps2) &&
           isfinite(calibration->stage1_deceleration_mps2) &&
           (calibration->guide_tag_width_m > 0.0f) &&
           (calibration->camera_fx_px > 0.0f) &&
           (calibration->camera_fy_px > 0.0f) &&
           (calibration->guide_follow_distance_m > 0.0f) &&
           (calibration->guide_min_safe_distance_m > 0.0f) &&
           (calibration->stage1_max_speed_mps > 0.0f) &&
           (calibration->stage1_acceleration_mps2 > 0.0f) &&
           (calibration->stage1_deceleration_mps2 > 0.0f);
}

static bool axis_map_is_valid(const int8_t axis_map[3], const int8_t axis_sign[3])
{
    bool seen[3] = { false, false, false };
    size_t index;

    for(index = 0U; index < 3U; ++index)
    {
        if((axis_map[index] < 0) || (axis_map[index] > 2) ||
           seen[static_cast<uint8_t>(axis_map[index])] || !sign_is_valid(axis_sign[index]))
        {
            return false;
        }
        seen[static_cast<uint8_t>(axis_map[index])] = true;
    }
    return true;
}

static bool steering_table_is_valid(const SteeringCalibrationPoint *points,
                                    size_t count)
{
    size_t index;

    if((points == nullptr) || (count < 2U) ||
       (count > STEERING_CALIBRATION_MAX_POINTS))
    {
        return false;
    }

    for(index = 0U; index < count; ++index)
    {
        if(!isfinite(points[index].equivalent_steering_angle_rad))
        {
            return false;
        }
        if((index > 0U) &&
           (points[index].continuous_count <= points[index - 1U].continuous_count))
        {
            return false;
        }
    }
    return true;
}

static int steering_table_angle_direction(
    const SteeringCalibrationPoint *points, size_t count)
{
    size_t index;
    int direction = 0;

    if(!steering_table_is_valid(points, count))
    {
        return 0;
    }
    for(index = 1U; index < count; ++index)
    {
        float delta = points[index].equivalent_steering_angle_rad -
                      points[index - 1U].equivalent_steering_angle_rad;
        int current_direction;

        if(delta == 0.0f)
        {
            return 0;
        }
        current_direction = (delta > 0.0f) ? 1 : -1;
        if(direction == 0)
        {
            direction = current_direction;
        }
        else if(direction != current_direction)
        {
            return 0;
        }
    }
    return direction;
}

SteeringCalibrationTables vehicle_calibration_get_steering_tables()
{
    SteeringCalibrationTables tables;

    tables.from_left = g_steering_calibration_from_left;
    tables.from_left_count = g_steering_calibration_from_left_count;
    tables.from_right = g_steering_calibration_from_right;
    tables.from_right_count = g_steering_calibration_from_right_count;
    return tables;
}

bool vehicle_calibration_is_valid(const VehicleCalibration *calibration,
                                  const SteeringCalibrationTables *tables)
{
    int from_left_direction;
    int from_right_direction;

    if((calibration == nullptr) || (tables == nullptr) || !CALIBRATION_VALID ||
       !calibration->valid)
    {
        return false;
    }

    if(!sign_is_valid(calibration->left_encoder_forward_sign) ||
       !sign_is_valid(calibration->right_encoder_forward_sign) ||
       !sign_is_valid(calibration->left_motor_forward_direction) ||
       !sign_is_valid(calibration->right_motor_forward_direction) ||
       !sign_is_valid(calibration->steering_left_direction) ||
       (calibration->steering_encoder_center_raw_count > 4095U) ||
       !(calibration->left_meter_per_count > 0.0f) ||
       !(calibration->right_meter_per_count > 0.0f) ||
       (calibration->left_counts_per_wheel_rev <= 0) ||
       (calibration->right_counts_per_wheel_rev <= 0) ||
       !duty_is_valid(calibration->left_motor_start_duty_forward) ||
       !duty_is_valid(calibration->left_motor_start_duty_reverse) ||
       !duty_is_valid(calibration->right_motor_start_duty_forward) ||
       !duty_is_valid(calibration->right_motor_start_duty_reverse) ||
       !duty_is_valid(calibration->steering_start_duty_left) ||
       !duty_is_valid(calibration->steering_start_duty_right) ||
       !isfinite(calibration->imu_position_x_m) ||
       !isfinite(calibration->imu_position_y_m) ||
       !isfinite(calibration->imu_position_z_m) ||
       !(calibration->max_acceleration_mps2 > 0.0f) ||
       !(calibration->max_deceleration_mps2 > 0.0f) ||
       !(calibration->max_lateral_acceleration_mps2 > 0.0f) ||
       !guide_geometry_is_valid(calibration) ||
       (calibration->steering_left_soft_limit_count == 0) ||
       (calibration->steering_right_soft_limit_count == 0) ||
       ((calibration->steering_left_soft_limit_count < 0) ==
        (calibration->steering_right_soft_limit_count < 0)) ||
       !axis_map_is_valid(calibration->imu_axis_map,
                          calibration->imu_axis_sign) ||
       !steering_table_is_valid(tables->from_left, tables->from_left_count) ||
       !steering_table_is_valid(tables->from_right, tables->from_right_count))
    {
        return false;
    }
    from_left_direction = steering_table_angle_direction(
        tables->from_left, tables->from_left_count);
    from_right_direction = steering_table_angle_direction(
        tables->from_right, tables->from_right_count);
    if((from_left_direction == 0) ||
       (from_left_direction != from_right_direction) ||
       ((from_left_direction > 0) &&
        (calibration->steering_left_soft_limit_count <=
         calibration->steering_right_soft_limit_count)) ||
       ((from_left_direction < 0) &&
        (calibration->steering_left_soft_limit_count >=
         calibration->steering_right_soft_limit_count)))
    {
        return false;
    }
    return true;
}
