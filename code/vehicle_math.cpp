#include "vehicle_math.h"

#include <math.h>
#include <stddef.h>

float vehicle_clampf(float value, float minimum, float maximum)
{
    if(value < minimum)
    {
        return minimum;
    }
    if(value > maximum)
    {
        return maximum;
    }
    return value;
}

float vehicle_wrap_pi(float angle_rad)
{
    if(!isfinite(angle_rad))
    {
        return 0.0f;
    }
    while(angle_rad > VEHICLE_PI_F)
    {
        angle_rad -= VEHICLE_TWO_PI_F;
    }
    while(angle_rad <= -VEHICLE_PI_F)
    {
        angle_rad += VEHICLE_TWO_PI_F;
    }
    return angle_rad;
}

float vehicle_angle_difference(float target_rad, float current_rad)
{
    return vehicle_wrap_pi(target_rad - current_rad);
}

float vehicle_rate_limit(float target, float previous, float rise_per_s,
                         float fall_per_s, float dt_s)
{
    float delta;
    float limit;

    if(!(dt_s > 0.0f) || !isfinite(target) || !isfinite(previous))
    {
        return previous;
    }

    delta = target - previous;
    limit = ((target * previous) < 0.0f || fabsf(target) < fabsf(previous))
                ? fall_per_s * dt_s
                : rise_per_s * dt_s;
    limit = fabsf(limit);
    return previous + vehicle_clampf(delta, -limit, limit);
}

bool vehicle_float_is_finite(float value)
{
    return isfinite(value) != 0;
}

bool vehicle_float_array_is_finite(const float *values, unsigned int count)
{
    unsigned int index;

    if(values == nullptr)
    {
        return false;
    }
    for(index = 0U; index < count; ++index)
    {
        if(!isfinite(values[index]))
        {
            return false;
        }
    }
    return true;
}
