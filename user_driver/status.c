#include "status.h"
#include "motor.h"
#include "tracker.h"
#include "oled.h"
#include "idle.h"
#include "mpu_nav.h"
#include "pid_utils.h"

system_status_t sys_status = STATUS_IDLE;
int              start_flag = 0;

#define M3_BASE_SPEED 800.0f

static uint8_t    m3_init    = 0;
static float      m3_ref_yaw = 0.0f;
static pid_ctrl_t m3_heading_pid = {
    .Kp = 3.0f,
    .Ki = 0.0f,
    .Kd = 0.0f,
    .integral_max = 0.0f
};

void status_cycle_next(void)
{
    sys_status = (sys_status + 1) % STATUS_COUNT;
    start_flag = 0;
    m3_init    = 0;
}

void status_toggle_start(void)
{
    start_flag ^= 1;
    pid_line.integral = 0;
    mpu_reset_zero(g_raw_yaw);
    m3_init = 0;
}

void status_run(float yaw)
{
    switch (sys_status) {
    case STATUS_IDLE:
        stay_idle();
        break;

    case STATUS_TRACKING:
        if (start_flag == 0) {
            tracking_active = 0;
            stay_idle();
        } else {
            tracking_active = 1;
            motor_set_direction(MOTOR_LEFT, 1);
            motor_set_direction(MOTOR_RIGHT, 1);
        }
        break;

    case STATUS_MPU_NAV: {
        if (start_flag == 0) {
            stay_idle();
        }
        else if (!m3_init) {
            m3_ref_yaw = yaw;
            motor_set_direction(MOTOR_RIGHT, 1);
            motor_set_direction(MOTOR_LEFT, 1);
            target_speed_1 = M3_BASE_SPEED;
            target_speed_2 = M3_BASE_SPEED;
            m3_init = 1;
        }
        float err  = normalize_angle(yaw - m3_ref_yaw);
        float corr = pid_compute(&m3_heading_pid, err);
        target_speed_2 = clamp_value(M3_BASE_SPEED - corr, 0.0f, 1000.0f);
        target_speed_1 = clamp_value(M3_BASE_SPEED + corr, 0.0f, 1000.0f);
        break;
    }

    default:
        break;
    }
}
