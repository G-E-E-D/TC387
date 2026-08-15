#ifndef VEHICLE_PATH_H
#define VEHICLE_PATH_H

#include <stdbool.h>
#include <stdint.h>

#include "vehicle_types.h"

typedef enum
{
    VEHICLE_PATH_RESULT_OK = 0,
    VEHICLE_PATH_RESULT_SAMPLE_SKIPPED,
    VEHICLE_PATH_RESULT_DISTANCE_LIMIT,
    VEHICLE_PATH_RESULT_NOT_RECORDING,
    VEHICLE_PATH_RESULT_INVALID_ARGUMENT,
    VEHICLE_PATH_RESULT_NUMERIC_ERROR,
    VEHICLE_PATH_RESULT_OVERFLOW,
    VEHICLE_PATH_RESULT_TOO_SHORT,
    VEHICLE_PATH_RESULT_DISCONTINUITY,
    VEHICLE_PATH_RESULT_INVALID_PATH
} VehiclePathResult;

typedef struct
{
    float x_m;
    float y_m;
    float yaw_rad;
    float speed_mps;
} VehiclePathSample;

typedef struct
{
    uint32_t point_count;
    float length_m;
    float largest_input_segment_m;
    float maximum_yaw_step_rad;
    bool recording;
    bool processed;
    bool valid;
    bool overflowed;
    bool distance_limit_reached;
    VehiclePathResult last_result;
} VehiclePathInfo;

void vehicle_path_reset(void);
VehiclePathResult vehicle_path_start_recording(const VehiclePathSample *sample);
VehiclePathResult vehicle_path_record_sample(const VehiclePathSample *sample);
VehiclePathResult vehicle_path_end_recording(void);

/* Copies test or imported data into the same static path buffer. */
VehiclePathResult vehicle_path_load_raw_points(const PathPoint *points,
                                               uint32_t point_count);

/* Deduplicates, resamples, smooths and computes forward curvature in place. */
VehiclePathResult vehicle_path_preprocess(void);

VehiclePathResult vehicle_path_validate_points(const PathPoint *points,
                                               uint32_t point_count,
                                               float *length_m,
                                               uint32_t *bad_index);

const PathPoint *vehicle_path_get_points(void);
const PathPoint *vehicle_path_get_point(uint32_t index);
uint32_t vehicle_path_get_count(void);
VehiclePathInfo vehicle_path_get_info(void);
bool vehicle_path_is_valid(void);
bool vehicle_path_result_is_error(VehiclePathResult result);

#endif
