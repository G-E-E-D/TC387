#include "vehicle_path.h"

#include <math.h>
#include <stddef.h>

#include "vehicle_config.h"
#include "vehicle_math.h"

typedef char VehiclePathPointSizeCheck[
    (sizeof(PathPoint) == PATH_POINT_SIZE_BYTES) ? 1 : -1];

typedef struct
{
    VehiclePathInfo info;
    VehiclePathSample last_input;
    float pending_distance_m;
    bool have_last_input;
} VehiclePathState;

static PathPoint g_vehicle_path_points[PATH_MAX_POINTS];
static VehiclePathState g_vehicle_path_state;

static VehiclePathResult vehicle_path_set_result(VehiclePathResult result)
{
    g_vehicle_path_state.info.last_result = result;
    return result;
}

static bool vehicle_path_sample_is_finite(const VehiclePathSample *sample)
{
    return (sample != NULL) && vehicle_float_is_finite(sample->x_m) &&
           vehicle_float_is_finite(sample->y_m) &&
           vehicle_float_is_finite(sample->yaw_rad) &&
           vehicle_float_is_finite(sample->speed_mps);
}

static bool vehicle_path_point_base_is_finite(const PathPoint *point)
{
    return (point != NULL) && vehicle_float_is_finite(point->x) &&
           vehicle_float_is_finite(point->y) &&
           vehicle_float_is_finite(point->yaw_body) &&
           vehicle_float_is_finite(point->recorded_speed);
}

static VehiclePathSample vehicle_path_interpolate_sample(
    const VehiclePathSample *start, const VehiclePathSample *end, float ratio)
{
    VehiclePathSample result;

    result.x_m = start->x_m + (end->x_m - start->x_m) * ratio;
    result.y_m = start->y_m + (end->y_m - start->y_m) * ratio;
    result.yaw_rad = start->yaw_rad + (end->yaw_rad - start->yaw_rad) * ratio;
    result.speed_mps = start->speed_mps +
                       (end->speed_mps - start->speed_mps) * ratio;
    return result;
}

static PathPoint vehicle_path_interpolate_point(const PathPoint *start,
                                                const PathPoint *end,
                                                float ratio,
                                                float arc_length_m)
{
    PathPoint result;

    result.x = start->x + (end->x - start->x) * ratio;
    result.y = start->y + (end->y - start->y) * ratio;
    result.yaw_body = start->yaw_body +
                      (end->yaw_body - start->yaw_body) * ratio;
    result.s = arc_length_m;
    result.curvature_forward = 0.0f;
    result.recorded_speed = start->recorded_speed +
                            (end->recorded_speed - start->recorded_speed) *
                                ratio;
    return result;
}

static VehiclePathResult vehicle_path_append_sample(
    const VehiclePathSample *sample, float arc_increment_m)
{
    PathPoint *point;
    const PathPoint *previous;

    if(!vehicle_path_sample_is_finite(sample) ||
       !vehicle_float_is_finite(arc_increment_m) ||
       (arc_increment_m < 0.0f))
    {
        return vehicle_path_set_result(VEHICLE_PATH_RESULT_NUMERIC_ERROR);
    }
    if(g_vehicle_path_state.info.point_count >= PATH_MAX_POINTS)
    {
        g_vehicle_path_state.info.overflowed = true;
        g_vehicle_path_state.info.recording = false;
        g_vehicle_path_state.info.valid = false;
        return vehicle_path_set_result(VEHICLE_PATH_RESULT_OVERFLOW);
    }

    point = &g_vehicle_path_points[g_vehicle_path_state.info.point_count];
    point->x = sample->x_m;
    point->y = sample->y_m;
    point->yaw_body = sample->yaw_rad;
    point->curvature_forward = 0.0f;
    point->recorded_speed = sample->speed_mps;

    if(g_vehicle_path_state.info.point_count == 0U)
    {
        point->s = 0.0f;
    }
    else
    {
        previous = &g_vehicle_path_points[
            g_vehicle_path_state.info.point_count - 1U];
        point->s = previous->s + arc_increment_m;
    }

    ++g_vehicle_path_state.info.point_count;
    g_vehicle_path_state.info.length_m = point->s;
    g_vehicle_path_state.info.processed = false;
    g_vehicle_path_state.info.valid = false;
    return VEHICLE_PATH_RESULT_OK;
}

static VehiclePathResult vehicle_path_compact_and_unwrap(void)
{
    uint32_t read_index;
    uint32_t write_index;
    float largest_segment_m = 0.0f;
    float maximum_yaw_step_rad = 0.0f;

    if(g_vehicle_path_state.info.point_count == 0U ||
       !vehicle_path_point_base_is_finite(&g_vehicle_path_points[0]))
    {
        return vehicle_path_set_result(VEHICLE_PATH_RESULT_NUMERIC_ERROR);
    }

    g_vehicle_path_points[0].s = 0.0f;
    g_vehicle_path_points[0].curvature_forward = 0.0f;
    write_index = 1U;

    for(read_index = 1U;
        read_index < g_vehicle_path_state.info.point_count;
        ++read_index)
    {
        PathPoint candidate = g_vehicle_path_points[read_index];
        PathPoint *previous = &g_vehicle_path_points[write_index - 1U];
        float delta_x;
        float delta_y;
        float segment_m;
        float yaw_step_rad;

        if(!vehicle_path_point_base_is_finite(&candidate))
        {
            return vehicle_path_set_result(VEHICLE_PATH_RESULT_NUMERIC_ERROR);
        }

        candidate.yaw_body = previous->yaw_body +
            vehicle_angle_difference(candidate.yaw_body,
                                     previous->yaw_body);
        yaw_step_rad = fabsf(candidate.yaw_body - previous->yaw_body);
        if(yaw_step_rad > maximum_yaw_step_rad)
        {
            maximum_yaw_step_rad = yaw_step_rad;
        }
        if(yaw_step_rad > PATH_MAX_YAW_JUMP_RAD)
        {
            return vehicle_path_set_result(
                VEHICLE_PATH_RESULT_DISCONTINUITY);
        }

        delta_x = candidate.x - previous->x;
        delta_y = candidate.y - previous->y;
        segment_m = hypotf(delta_x, delta_y);
        if(!vehicle_float_is_finite(segment_m))
        {
            return vehicle_path_set_result(VEHICLE_PATH_RESULT_NUMERIC_ERROR);
        }
        if(segment_m < PATH_MIN_POINT_DISTANCE_M)
        {
            continue;
        }
        if(segment_m > largest_segment_m)
        {
            largest_segment_m = segment_m;
        }

        candidate.s = previous->s + segment_m;
        candidate.curvature_forward = 0.0f;
        g_vehicle_path_points[write_index] = candidate;
        ++write_index;
    }

    g_vehicle_path_state.info.point_count = write_index;
    g_vehicle_path_state.info.length_m =
        g_vehicle_path_points[write_index - 1U].s;
    g_vehicle_path_state.info.largest_input_segment_m = largest_segment_m;
    g_vehicle_path_state.info.maximum_yaw_step_rad = maximum_yaw_step_rad;
    return VEHICLE_PATH_RESULT_OK;
}

static VehiclePathResult vehicle_path_densify_in_place(void)
{
    uint32_t read_index;
    uint32_t write_index;
    uint32_t expanded_count = 1U;
    PathPoint first_point;

    for(read_index = 1U;
        read_index < g_vehicle_path_state.info.point_count;
        ++read_index)
    {
        const PathPoint *start = &g_vehicle_path_points[read_index - 1U];
        const PathPoint *end = &g_vehicle_path_points[read_index];
        float segment_m = hypotf(end->x - start->x, end->y - start->y);
        float segment_count_f;
        uint32_t segment_count;

        if(!(segment_m > 0.0f) || !vehicle_float_is_finite(segment_m))
        {
            return vehicle_path_set_result(VEHICLE_PATH_RESULT_INVALID_PATH);
        }
        segment_count_f = ceilf(segment_m / PATH_RECORD_SPACING_M);
        if(!vehicle_float_is_finite(segment_count_f) ||
           (segment_count_f > (float)PATH_MAX_POINTS))
        {
            g_vehicle_path_state.info.overflowed = true;
            return vehicle_path_set_result(VEHICLE_PATH_RESULT_OVERFLOW);
        }
        segment_count = (uint32_t)segment_count_f;
        if(segment_count == 0U)
        {
            segment_count = 1U;
        }
        if(segment_count > (PATH_MAX_POINTS - expanded_count))
        {
            g_vehicle_path_state.info.overflowed = true;
            return vehicle_path_set_result(VEHICLE_PATH_RESULT_OVERFLOW);
        }
        expanded_count += segment_count;
    }

    if(expanded_count == g_vehicle_path_state.info.point_count)
    {
        return VEHICLE_PATH_RESULT_OK;
    }

    first_point = g_vehicle_path_points[0];
    write_index = expanded_count - 1U;
    read_index = g_vehicle_path_state.info.point_count - 1U;
    while(read_index > 0U)
    {
        PathPoint start = g_vehicle_path_points[read_index - 1U];
        PathPoint end = g_vehicle_path_points[read_index];
        float segment_m = hypotf(end.x - start.x, end.y - start.y);
        uint32_t segment_count =
            (uint32_t)ceilf(segment_m / PATH_RECORD_SPACING_M);
        uint32_t part;

        if(segment_count == 0U)
        {
            segment_count = 1U;
        }
        for(part = segment_count; part > 0U; --part)
        {
            float ratio = (float)part / (float)segment_count;
            float arc_length_m = start.s +
                                 (end.s - start.s) * ratio;

            g_vehicle_path_points[write_index] =
                vehicle_path_interpolate_point(&start, &end, ratio,
                                               arc_length_m);
            --write_index;
        }
        --read_index;
    }
    g_vehicle_path_points[0] = first_point;
    g_vehicle_path_state.info.point_count = expanded_count;
    return VEHICLE_PATH_RESULT_OK;
}

static VehiclePathResult vehicle_path_recompute_geometry(void)
{
    uint32_t index;
    float cumulative_s = 0.0f;

    if(g_vehicle_path_state.info.point_count == 0U ||
       !vehicle_path_point_base_is_finite(&g_vehicle_path_points[0]))
    {
        return vehicle_path_set_result(VEHICLE_PATH_RESULT_INVALID_PATH);
    }

    g_vehicle_path_points[0].s = 0.0f;
    g_vehicle_path_points[0].curvature_forward = 0.0f;
    for(index = 1U; index < g_vehicle_path_state.info.point_count; ++index)
    {
        PathPoint *current = &g_vehicle_path_points[index];
        const PathPoint *previous = &g_vehicle_path_points[index - 1U];
        float segment_m;

        if(!vehicle_path_point_base_is_finite(current))
        {
            return vehicle_path_set_result(VEHICLE_PATH_RESULT_NUMERIC_ERROR);
        }
        current->yaw_body = previous->yaw_body +
            vehicle_angle_difference(current->yaw_body,
                                     previous->yaw_body);
        if(fabsf(current->yaw_body - previous->yaw_body) >
           PATH_MAX_YAW_JUMP_RAD)
        {
            return vehicle_path_set_result(
                VEHICLE_PATH_RESULT_DISCONTINUITY);
        }

        segment_m = hypotf(current->x - previous->x,
                           current->y - previous->y);
        if(!vehicle_float_is_finite(segment_m) ||
           !(segment_m > (PATH_MIN_POINT_DISTANCE_M * 0.25f)))
        {
            return vehicle_path_set_result(VEHICLE_PATH_RESULT_INVALID_PATH);
        }
        cumulative_s += segment_m;
        current->s = cumulative_s;
        current->curvature_forward = 0.0f;
    }

    g_vehicle_path_state.info.length_m = cumulative_s;
    return VEHICLE_PATH_RESULT_OK;
}

static VehiclePathResult vehicle_path_resample_in_place(void)
{
    uint32_t input_count = g_vehicle_path_state.info.point_count;
    uint32_t interval_count;
    uint32_t output_count;
    uint32_t output_index;
    uint32_t source_index = 0U;
    float total_length_m = g_vehicle_path_state.info.length_m;
    float actual_spacing_m;

    if(!(total_length_m > 0.0f) ||
       !vehicle_float_is_finite(total_length_m))
    {
        return vehicle_path_set_result(VEHICLE_PATH_RESULT_TOO_SHORT);
    }

    interval_count = (uint32_t)floorf(total_length_m /
                                      PATH_RESAMPLE_SPACING_M);
    if(interval_count < (PATH_MIN_VALID_POINTS - 1U))
    {
        interval_count = PATH_MIN_VALID_POINTS - 1U;
    }
    if(interval_count == 0U)
    {
        interval_count = 1U;
    }
    output_count = interval_count + 1U;
    if(output_count > input_count || output_count > PATH_MAX_POINTS)
    {
        return vehicle_path_set_result(VEHICLE_PATH_RESULT_INVALID_PATH);
    }
    actual_spacing_m = total_length_m / (float)interval_count;

    for(output_index = 0U; output_index < output_count; ++output_index)
    {
        float target_s = (output_index == (output_count - 1U))
                             ? total_length_m
                             : actual_spacing_m * (float)output_index;
        PathPoint start;
        PathPoint end;
        float denominator;
        float ratio;

        while((source_index + 1U) < (input_count - 1U) &&
              g_vehicle_path_points[source_index + 1U].s < target_s)
        {
            ++source_index;
        }
        if(output_index > source_index &&
           target_s < g_vehicle_path_points[source_index + 1U].s)
        {
            return vehicle_path_set_result(VEHICLE_PATH_RESULT_INVALID_PATH);
        }

        start = g_vehicle_path_points[source_index];
        end = g_vehicle_path_points[source_index + 1U];
        denominator = end.s - start.s;
        if(!(denominator > 0.0f))
        {
            return vehicle_path_set_result(VEHICLE_PATH_RESULT_INVALID_PATH);
        }
        ratio = vehicle_clampf((target_s - start.s) / denominator,
                               0.0f, 1.0f);
        g_vehicle_path_points[output_index] =
            vehicle_path_interpolate_point(&start, &end, ratio, target_s);
    }

    g_vehicle_path_state.info.point_count = output_count;
    g_vehicle_path_state.info.length_m = total_length_m;
    return VEHICLE_PATH_RESULT_OK;
}

static void vehicle_path_smooth_in_place(void)
{
    enum
    {
        WINDOW_SIZE = (PATH_SMOOTH_HALF_WINDOW * 2U) + 1U
    };
    PathPoint window[WINDOW_SIZE];
    uint32_t read_index;
    const uint32_t half_window = PATH_SMOOTH_HALF_WINDOW;

    if(half_window == 0U ||
       g_vehicle_path_state.info.point_count <= (half_window * 2U))
    {
        return;
    }

    for(read_index = 0U;
        read_index < g_vehicle_path_state.info.point_count;
        ++read_index)
    {
        window[read_index % WINDOW_SIZE] =
            g_vehicle_path_points[read_index];
        if(read_index >= (half_window * 2U))
        {
            uint32_t center_index = read_index - half_window;

            if(center_index >= half_window &&
               (center_index + half_window) <
                   g_vehicle_path_state.info.point_count)
            {
                PathPoint smoothed = window[center_index % WINDOW_SIZE];
                float sum_x = 0.0f;
                float sum_y = 0.0f;
                float sum_yaw = 0.0f;
                float sum_speed = 0.0f;
                float sum_weight = 0.0f;
                int32_t offset;

                for(offset = -(int32_t)half_window;
                    offset <= (int32_t)half_window;
                    ++offset)
                {
                    uint32_t source =
                        (uint32_t)((int32_t)center_index + offset);
                    float weight =
                        (float)(half_window + 1U -
                                (uint32_t)((offset < 0) ? -offset : offset));
                    const PathPoint *original = &window[source % WINDOW_SIZE];

                    sum_x += original->x * weight;
                    sum_y += original->y * weight;
                    sum_yaw += original->yaw_body * weight;
                    sum_speed += original->recorded_speed * weight;
                    sum_weight += weight;
                }
                smoothed.x = sum_x / sum_weight;
                smoothed.y = sum_y / sum_weight;
                smoothed.yaw_body = sum_yaw / sum_weight;
                smoothed.recorded_speed = sum_speed / sum_weight;
                smoothed.curvature_forward = 0.0f;
                g_vehicle_path_points[center_index] = smoothed;
            }
        }
    }
}

static VehiclePathResult vehicle_path_compute_curvature(void)
{
    uint32_t index;

    for(index = 0U; index < g_vehicle_path_state.info.point_count; ++index)
    {
        uint32_t lower = (index > PATH_CURVATURE_WINDOW)
                             ? index - PATH_CURVATURE_WINDOW
                             : 0U;
        uint32_t upper = index + PATH_CURVATURE_WINDOW;
        float arc_span_m;
        float yaw_span_rad;

        if(upper >= g_vehicle_path_state.info.point_count)
        {
            upper = g_vehicle_path_state.info.point_count - 1U;
        }
        arc_span_m = g_vehicle_path_points[upper].s -
                     g_vehicle_path_points[lower].s;
        yaw_span_rad = g_vehicle_path_points[upper].yaw_body -
                       g_vehicle_path_points[lower].yaw_body;
        if(!(arc_span_m > 0.0f) || !vehicle_float_is_finite(yaw_span_rad))
        {
            return vehicle_path_set_result(VEHICLE_PATH_RESULT_INVALID_PATH);
        }
        g_vehicle_path_points[index].curvature_forward =
            yaw_span_rad / arc_span_m;
        if(!vehicle_float_is_finite(
               g_vehicle_path_points[index].curvature_forward))
        {
            return vehicle_path_set_result(VEHICLE_PATH_RESULT_NUMERIC_ERROR);
        }
    }
    return VEHICLE_PATH_RESULT_OK;
}

void vehicle_path_reset(void)
{
    g_vehicle_path_state.info.point_count = 0U;
    g_vehicle_path_state.info.length_m = 0.0f;
    g_vehicle_path_state.info.largest_input_segment_m = 0.0f;
    g_vehicle_path_state.info.maximum_yaw_step_rad = 0.0f;
    g_vehicle_path_state.info.recording = false;
    g_vehicle_path_state.info.processed = false;
    g_vehicle_path_state.info.valid = false;
    g_vehicle_path_state.info.overflowed = false;
    g_vehicle_path_state.info.distance_limit_reached = false;
    g_vehicle_path_state.info.last_result = VEHICLE_PATH_RESULT_OK;
    g_vehicle_path_state.last_input.x_m = 0.0f;
    g_vehicle_path_state.last_input.y_m = 0.0f;
    g_vehicle_path_state.last_input.yaw_rad = 0.0f;
    g_vehicle_path_state.last_input.speed_mps = 0.0f;
    g_vehicle_path_state.pending_distance_m = 0.0f;
    g_vehicle_path_state.have_last_input = false;
}

VehiclePathResult vehicle_path_start_recording(const VehiclePathSample *sample)
{
    VehiclePathResult result;

    if(!vehicle_path_sample_is_finite(sample))
    {
        return VEHICLE_PATH_RESULT_INVALID_ARGUMENT;
    }

    vehicle_path_reset();
    result = vehicle_path_append_sample(sample, 0.0f);
    if(result != VEHICLE_PATH_RESULT_OK)
    {
        return result;
    }
    g_vehicle_path_state.last_input = *sample;
    g_vehicle_path_state.have_last_input = true;
    g_vehicle_path_state.info.recording = true;
    return vehicle_path_set_result(VEHICLE_PATH_RESULT_OK);
}

VehiclePathResult vehicle_path_record_sample(const VehiclePathSample *sample)
{
    VehiclePathSample current;
    VehiclePathSample cursor;
    float segment_m;
    float current_distance_m;
    float remaining_limit_m;
    bool appended = false;
    bool hit_distance_limit = false;

    if(!g_vehicle_path_state.info.recording ||
       !g_vehicle_path_state.have_last_input)
    {
        return vehicle_path_set_result(VEHICLE_PATH_RESULT_NOT_RECORDING);
    }
    if(!vehicle_path_sample_is_finite(sample))
    {
        g_vehicle_path_state.info.recording = false;
        return vehicle_path_set_result(VEHICLE_PATH_RESULT_NUMERIC_ERROR);
    }

    current = *sample;
    current.yaw_rad = g_vehicle_path_state.last_input.yaw_rad +
        vehicle_angle_difference(current.yaw_rad,
                                 g_vehicle_path_state.last_input.yaw_rad);
    segment_m = hypotf(current.x_m - g_vehicle_path_state.last_input.x_m,
                       current.y_m - g_vehicle_path_state.last_input.y_m);
    if(!vehicle_float_is_finite(segment_m))
    {
        g_vehicle_path_state.info.recording = false;
        return vehicle_path_set_result(VEHICLE_PATH_RESULT_NUMERIC_ERROR);
    }

    if(segment_m > g_vehicle_path_state.info.largest_input_segment_m)
    {
        g_vehicle_path_state.info.largest_input_segment_m = segment_m;
    }
    if(fabsf(current.yaw_rad - g_vehicle_path_state.last_input.yaw_rad) >
       g_vehicle_path_state.info.maximum_yaw_step_rad)
    {
        g_vehicle_path_state.info.maximum_yaw_step_rad =
            fabsf(current.yaw_rad -
                  g_vehicle_path_state.last_input.yaw_rad);
    }

    if(segment_m < (PATH_MIN_POINT_DISTANCE_M * 0.25f))
    {
        g_vehicle_path_state.last_input = current;
        return vehicle_path_set_result(VEHICLE_PATH_RESULT_SAMPLE_SKIPPED);
    }

    current_distance_m = g_vehicle_path_points[
        g_vehicle_path_state.info.point_count - 1U].s +
        g_vehicle_path_state.pending_distance_m;
    remaining_limit_m = RECORD_MAX_DISTANCE_M - current_distance_m;
    if(!(remaining_limit_m > 0.0f))
    {
        g_vehicle_path_state.info.recording = false;
        g_vehicle_path_state.info.distance_limit_reached = true;
        return vehicle_path_set_result(VEHICLE_PATH_RESULT_DISTANCE_LIMIT);
    }
    if(segment_m > remaining_limit_m)
    {
        float ratio = remaining_limit_m / segment_m;

        current = vehicle_path_interpolate_sample(
            &g_vehicle_path_state.last_input, &current, ratio);
        segment_m = remaining_limit_m;
        hit_distance_limit = true;
    }

    cursor = g_vehicle_path_state.last_input;
    while((g_vehicle_path_state.pending_distance_m + segment_m) >=
          PATH_RECORD_SPACING_M)
    {
        float needed_m = PATH_RECORD_SPACING_M -
                         g_vehicle_path_state.pending_distance_m;
        float ratio = needed_m / segment_m;
        VehiclePathSample placed =
            vehicle_path_interpolate_sample(&cursor, &current, ratio);
        VehiclePathResult result = vehicle_path_append_sample(
            &placed, PATH_RECORD_SPACING_M);

        if(result != VEHICLE_PATH_RESULT_OK)
        {
            return result;
        }
        appended = true;
        cursor = placed;
        segment_m -= needed_m;
        g_vehicle_path_state.pending_distance_m = 0.0f;
        if(!(segment_m > 0.0f))
        {
            segment_m = 0.0f;
            break;
        }
    }
    g_vehicle_path_state.pending_distance_m += segment_m;
    g_vehicle_path_state.last_input = current;

    if(g_vehicle_path_state.pending_distance_m >= PATH_MIN_POINT_DISTANCE_M &&
       (hit_distance_limit ||
        fabsf(current.yaw_rad -
              g_vehicle_path_points[
                  g_vehicle_path_state.info.point_count - 1U].yaw_body) >=
            PATH_YAW_SAMPLE_THRESHOLD_RAD))
    {
        VehiclePathResult result = vehicle_path_append_sample(
            &current, g_vehicle_path_state.pending_distance_m);

        if(result != VEHICLE_PATH_RESULT_OK)
        {
            return result;
        }
        g_vehicle_path_state.pending_distance_m = 0.0f;
        appended = true;
    }

    if(hit_distance_limit)
    {
        g_vehicle_path_state.info.recording = false;
        g_vehicle_path_state.info.distance_limit_reached = true;
        return vehicle_path_set_result(VEHICLE_PATH_RESULT_DISTANCE_LIMIT);
    }
    return vehicle_path_set_result(appended ? VEHICLE_PATH_RESULT_OK
                                            : VEHICLE_PATH_RESULT_SAMPLE_SKIPPED);
}

VehiclePathResult vehicle_path_end_recording(void)
{
    if(!g_vehicle_path_state.have_last_input ||
       g_vehicle_path_state.info.point_count == 0U)
    {
        return vehicle_path_set_result(VEHICLE_PATH_RESULT_INVALID_PATH);
    }
    if(g_vehicle_path_state.info.overflowed)
    {
        g_vehicle_path_state.info.recording = false;
        return vehicle_path_set_result(VEHICLE_PATH_RESULT_OVERFLOW);
    }
    if(g_vehicle_path_state.info.recording &&
       g_vehicle_path_state.pending_distance_m >= PATH_MIN_POINT_DISTANCE_M)
    {
        VehiclePathResult result = vehicle_path_append_sample(
            &g_vehicle_path_state.last_input,
            g_vehicle_path_state.pending_distance_m);

        if(result != VEHICLE_PATH_RESULT_OK)
        {
            return result;
        }
        g_vehicle_path_state.pending_distance_m = 0.0f;
    }
    g_vehicle_path_state.info.recording = false;
    g_vehicle_path_state.info.processed = false;
    g_vehicle_path_state.info.valid = false;
    return vehicle_path_set_result(VEHICLE_PATH_RESULT_OK);
}

VehiclePathResult vehicle_path_load_raw_points(const PathPoint *points,
                                               uint32_t point_count)
{
    uint32_t index;

    if(points == NULL || point_count == 0U)
    {
        return VEHICLE_PATH_RESULT_INVALID_ARGUMENT;
    }
    if(point_count > PATH_MAX_POINTS)
    {
        vehicle_path_reset();
        g_vehicle_path_state.info.overflowed = true;
        return vehicle_path_set_result(VEHICLE_PATH_RESULT_OVERFLOW);
    }

    vehicle_path_reset();
    for(index = 0U; index < point_count; ++index)
    {
        if(!vehicle_path_point_base_is_finite(&points[index]))
        {
            vehicle_path_reset();
            return vehicle_path_set_result(VEHICLE_PATH_RESULT_NUMERIC_ERROR);
        }
        g_vehicle_path_points[index] = points[index];
        g_vehicle_path_points[index].curvature_forward = 0.0f;
    }
    g_vehicle_path_state.info.point_count = point_count;
    g_vehicle_path_state.last_input.x_m = points[point_count - 1U].x;
    g_vehicle_path_state.last_input.y_m = points[point_count - 1U].y;
    g_vehicle_path_state.last_input.yaw_rad =
        points[point_count - 1U].yaw_body;
    g_vehicle_path_state.last_input.speed_mps =
        points[point_count - 1U].recorded_speed;
    g_vehicle_path_state.have_last_input = true;
    return vehicle_path_set_result(VEHICLE_PATH_RESULT_OK);
}

VehiclePathResult vehicle_path_preprocess(void)
{
    VehiclePathResult result;
    uint32_t bad_index;
    float validated_length_m;

    if(g_vehicle_path_state.info.recording)
    {
        result = vehicle_path_end_recording();
        if(result != VEHICLE_PATH_RESULT_OK)
        {
            return result;
        }
    }
    if(g_vehicle_path_state.info.overflowed)
    {
        return vehicle_path_set_result(VEHICLE_PATH_RESULT_OVERFLOW);
    }

    result = vehicle_path_compact_and_unwrap();
    if(result != VEHICLE_PATH_RESULT_OK)
    {
        return result;
    }
    if(g_vehicle_path_state.info.point_count < 2U ||
       g_vehicle_path_state.info.length_m < PATH_MIN_VALID_LENGTH_M)
    {
        return vehicle_path_set_result(VEHICLE_PATH_RESULT_TOO_SHORT);
    }

    result = vehicle_path_densify_in_place();
    if(result != VEHICLE_PATH_RESULT_OK)
    {
        return result;
    }
    result = vehicle_path_recompute_geometry();
    if(result != VEHICLE_PATH_RESULT_OK)
    {
        return result;
    }
    result = vehicle_path_resample_in_place();
    if(result != VEHICLE_PATH_RESULT_OK)
    {
        return result;
    }

    vehicle_path_smooth_in_place();
    result = vehicle_path_recompute_geometry();
    if(result != VEHICLE_PATH_RESULT_OK)
    {
        return result;
    }
    result = vehicle_path_compute_curvature();
    if(result != VEHICLE_PATH_RESULT_OK)
    {
        return result;
    }
    result = vehicle_path_validate_points(
        g_vehicle_path_points, g_vehicle_path_state.info.point_count,
        &validated_length_m, &bad_index);
    if(result != VEHICLE_PATH_RESULT_OK)
    {
        (void)bad_index;
        return vehicle_path_set_result(result);
    }

    g_vehicle_path_state.info.length_m = validated_length_m;
    g_vehicle_path_state.info.processed = true;
    g_vehicle_path_state.info.valid = true;
    return vehicle_path_set_result(VEHICLE_PATH_RESULT_OK);
}

VehiclePathResult vehicle_path_validate_points(const PathPoint *points,
                                               uint32_t point_count,
                                               float *length_m,
                                               uint32_t *bad_index)
{
    uint32_t index;

    if(length_m != NULL)
    {
        *length_m = 0.0f;
    }
    if(bad_index != NULL)
    {
        *bad_index = 0U;
    }
    if(points == NULL || point_count > PATH_MAX_POINTS)
    {
        return VEHICLE_PATH_RESULT_INVALID_ARGUMENT;
    }
    if(point_count < PATH_MIN_VALID_POINTS)
    {
        return VEHICLE_PATH_RESULT_TOO_SHORT;
    }
    if(!vehicle_path_point_base_is_finite(&points[0]) ||
       !vehicle_float_is_finite(points[0].s) ||
       !vehicle_float_is_finite(points[0].curvature_forward) ||
       fabsf(points[0].s) > PATH_MIN_POINT_DISTANCE_M)
    {
        return VEHICLE_PATH_RESULT_INVALID_PATH;
    }

    for(index = 1U; index < point_count; ++index)
    {
        float segment_m;
        float delta_s;

        if(!vehicle_path_point_base_is_finite(&points[index]) ||
           !vehicle_float_is_finite(points[index].s) ||
           !vehicle_float_is_finite(points[index].curvature_forward))
        {
            if(bad_index != NULL)
            {
                *bad_index = index;
            }
            return VEHICLE_PATH_RESULT_NUMERIC_ERROR;
        }
        segment_m = hypotf(points[index].x - points[index - 1U].x,
                           points[index].y - points[index - 1U].y);
        delta_s = points[index].s - points[index - 1U].s;
        if(!(segment_m > (PATH_MIN_POINT_DISTANCE_M * 0.25f)) ||
           segment_m > PATH_MAX_INPUT_SEGMENT_M || !(delta_s > 0.0f))
        {
            if(bad_index != NULL)
            {
                *bad_index = index;
            }
            return VEHICLE_PATH_RESULT_INVALID_PATH;
        }
        if(fabsf(points[index].yaw_body -
                 points[index - 1U].yaw_body) > PATH_MAX_YAW_JUMP_RAD)
        {
            if(bad_index != NULL)
            {
                *bad_index = index;
            }
            return VEHICLE_PATH_RESULT_DISCONTINUITY;
        }
    }

    if(points[point_count - 1U].s < PATH_MIN_VALID_LENGTH_M)
    {
        return VEHICLE_PATH_RESULT_TOO_SHORT;
    }
    if(length_m != NULL)
    {
        *length_m = points[point_count - 1U].s;
    }
    return VEHICLE_PATH_RESULT_OK;
}

const PathPoint *vehicle_path_get_points(void)
{
    return g_vehicle_path_points;
}

const PathPoint *vehicle_path_get_point(uint32_t index)
{
    if(index >= g_vehicle_path_state.info.point_count)
    {
        return NULL;
    }
    return &g_vehicle_path_points[index];
}

uint32_t vehicle_path_get_count(void)
{
    return g_vehicle_path_state.info.point_count;
}

VehiclePathInfo vehicle_path_get_info(void)
{
    return g_vehicle_path_state.info;
}

bool vehicle_path_is_valid(void)
{
    return g_vehicle_path_state.info.valid;
}

bool vehicle_path_result_is_error(VehiclePathResult result)
{
    return result != VEHICLE_PATH_RESULT_OK &&
           result != VEHICLE_PATH_RESULT_SAMPLE_SKIPPED &&
           result != VEHICLE_PATH_RESULT_DISTANCE_LIMIT;
}
