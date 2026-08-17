#ifndef VEHICLE_MATH_H
#define VEHICLE_MATH_H

#include <stdbool.h>

constexpr auto VEHICLE_PI_F = 3.14159265358979323846f;
constexpr auto VEHICLE_TWO_PI_F = 6.28318530717958647692f;

float vehicle_clampf(float value, float minimum, float maximum);
float vehicle_wrap_pi(float angle_rad);
float vehicle_angle_difference(float target_rad, float current_rad);
float vehicle_rate_limit(float target, float previous, float rise_per_s,
                         float fall_per_s, float dt_s);
bool vehicle_float_is_finite(float value);
bool vehicle_float_array_is_finite(const float *values, unsigned int count);

#endif
