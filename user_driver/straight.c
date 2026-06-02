#include "straight.h"
#include "tracker.h"
#include "motor.h"
#include "pid_utils.h"

volatile int32_t straight_enc_acc = 0;
volatile float  g_yaw = 0;

static uint8_t  straight_active = 0;
static float    straight_ref_yaw;
static int32_t  straight_enc_start;

static pid_ctrl_t straight_pid = {
    .Kp = 1.0f,
    .Ki = 0.0f,
    .Kd = 0.0f,
    .integral_max = 0.0f
};

#define SPEED_MAX 1000.0f

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
    float err  = normalize_angle(yaw - straight_ref_yaw);
    float corr = pid_compute(&straight_pid, err);

    target_speed_2 = clamp_value(STRAIGHT_BASE_SPEED - corr, 0.0f, SPEED_MAX);
    target_speed_1 = clamp_value(STRAIGHT_BASE_SPEED + corr, 0.0f, SPEED_MAX);

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
