#include "mpu_nav.h"

static float yaw_ref;
static float arc_start_yaw;

float mpu_normalize(float a)
{
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

void mpu_set_ref(float yaw)
{
    yaw_ref = yaw;
}

float mpu_get_ref(void)
{
    return yaw_ref;
}

float mpu_heading_error(float yaw)
{
    return mpu_normalize(yaw - yaw_ref);
}

void mpu_arc_begin(float yaw)
{
    arc_start_yaw = yaw;
}

float mpu_arc_progress(float yaw)
{
    float d = mpu_normalize(yaw - arc_start_yaw);
    return d < 0 ? -d : d;
}

float mpu_clamp_speed(float s)
{
    if (s > 600.0f) return 600.0f;
    if (s < 0.0f)   return 0.0f;
    return s;
}
