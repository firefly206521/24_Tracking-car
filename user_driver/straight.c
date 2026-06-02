#include "straight.h"
#include "tracker.h"
#include "motor.h"

volatile int32_t straight_enc_acc = 0;
volatile float  g_yaw = 0;

static uint8_t  straight_active = 0;
static float    straight_ref_yaw;
static int32_t  straight_enc_start;

#define STRAIGHT_KP       1.0f
#define SPEED_MAX         1000.0f

static float normalize(float a)
{
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

static float clamp_speed(float s)
{
    if (s > SPEED_MAX) return SPEED_MAX;
    if (s < 0.0f)      return 0.0f;
    return s;
}

void straight_begin(float yaw)
{
    straight_active    = 1;
    straight_ref_yaw   = yaw;
    straight_enc_start = straight_enc_acc;
}

uint8_t straight_is_active(void)
{
    return straight_active;
}

uint8_t straight_run(float yaw)
{
    float err  = normalize(yaw - straight_ref_yaw);
    float corr = STRAIGHT_KP * err;

    target_speed_2 = clamp_speed(STRAIGHT_BASE_SPEED - corr);
    target_speed_1 = clamp_speed(STRAIGHT_BASE_SPEED + corr);

    if (straight_get_distance() > STRAIGHT_DIST_MAX && straight_line_detected()) {
        straight_active = 0;
        return 1;
    }
    return 0;
}

int32_t straight_get_distance(void)
{
    return straight_enc_acc - straight_enc_start;
}

uint8_t straight_line_detected(void)
{
    for (int i = 0; i < 7; i++) {
        if (tracker_value[i] == 1) return 1;
    }
    return 0;
}
