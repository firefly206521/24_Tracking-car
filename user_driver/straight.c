#include "straight.h"
#include "tracker.h"
#include "motor.h"
#include "pid_utils.h"

volatile int32_t straight_enc_acc = 0;
volatile float  g_yaw = 0;

#define STRAIGHT_RAMP_STEP 10.0f
#define SPEED_MAX          1000.0f

static uint8_t  straight_active = 0;
static float    straight_ref_yaw;
static int32_t  straight_enc_start;
static float    straight_ramp;

static pid_ctrl_t straight_pid = {
    .Kp = 2.0f,
    .Ki = 0.3f,
    .Kd = 0.0f,
    .integral_max = 100.0f
};

// 核心航向修正，ISR 和 MPU_NAV 共用
static void straight_steer(float yaw)
{
    if (straight_ramp < STRAIGHT_BASE_SPEED) {
        straight_ramp += STRAIGHT_RAMP_STEP;
        if (straight_ramp > STRAIGHT_BASE_SPEED) straight_ramp = STRAIGHT_BASE_SPEED;
    }
    float err  = -normalize_angle(yaw - straight_ref_yaw);
    float corr = pid_compute(&straight_pid, err);
    target_speed_2 = clamp_value(straight_ramp - corr, 0.0f, SPEED_MAX);
    target_speed_1 = clamp_value(straight_ramp + corr, 0.0f, SPEED_MAX);
}

// ===== ISR 用（丢线后直行，有退出条件）=====

void straight_begin(float yaw)
{
    straight_active    = 1;
    straight_ref_yaw   = yaw;
    straight_enc_start = straight_enc_acc;
    straight_ramp      = 0.0f;
    pid_reset(&straight_pid);
    motor_set_direction(MOTOR_RIGHT, 1);
    motor_set_direction(MOTOR_LEFT, 1);
}

uint8_t straight_is_active(void)
{
    return straight_active;
}

uint8_t straight_run(float yaw)
{
    straight_steer(yaw);
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

// ===== STATUS_MPU_NAV 专用（无退出条件，不设 straight_active）=====

void straight_nav_begin(float yaw)
{
    straight_ref_yaw = yaw;
    straight_ramp    = 0.0f;   // 第一次直行：从 0 加速
    pid_reset(&straight_pid);
    motor_set_direction(MOTOR_RIGHT, 1);
    motor_set_direction(MOTOR_LEFT, 1);
}

void straight_nav_run(float yaw)
{
    straight_steer(yaw);
}

void straight_nav_resume(float yaw)
{
    straight_ref_yaw = yaw;
    straight_ramp    = STRAIGHT_BASE_SPEED;  // 第二次直行：跳过斜坡，直接满速
    motor_set_direction(MOTOR_RIGHT, 1);
    motor_set_direction(MOTOR_LEFT, 1);
    // 不调 pid_reset — 积分全程保持
}

void straight_force_stop(void)
{
    straight_active = 0;
}
