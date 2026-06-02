#include "status.h"
#include "motor.h"
#include "tracker.h"
#include "oled.h"
#include "idle.h"
#include "mpu_nav.h"
#include "pid_utils.h"
#include "straight.h"

#define S1_DIST_PULSES   1750   // status1 目标脉冲数

system_status_t sys_status = STATUS_IDLE;
int              start_flag = 0;

static uint8_t s1_init = 0;
static uint8_t m3_init = 0;

void status_cycle_next(void)
{
    sys_status = (sys_status + 1) % STATUS_COUNT;
    start_flag = 0;
    s1_init    = 0;
    m3_init    = 0;
}

void status_toggle_start(void)
{
    start_flag ^= 1;
    pid_line.integral = 0;
    mpu_reset_zero(g_raw_yaw);
    s1_init = 0;
    m3_init = 0;
}

void status_run(float yaw)
{
    switch (sys_status) {
    case STATUS_IDLE:
        stay_idle();
        break;

    case STATUS_DIST:
        if (start_flag == 0) {
            stay_idle();
            s1_init = 0;
        }
        else if (!s1_init) {
            straight_begin(yaw);
            s1_init = 1;
        }
        // straight_run 由 ISR 驱动，这里只检查是否到位
        if (straight_get_distance() / 2 >= S1_DIST_PULSES) {
            start_flag = 0;
            straight_force_stop();
            target_speed_1 = 0;
            target_speed_2 = 0;
            motor_brake(MOTOR_RIGHT);
            motor_brake(MOTOR_LEFT);
        }
        break;

    case STATUS_MPU_NAV:
        if (start_flag == 0) {
            stay_idle();
        }
        else {
            if (!m3_init) {
                straight_nav_begin(yaw);
                m3_init = 1;
            }
            straight_nav_run(yaw);
        }
        break;

    case STATUS_LINE_TRACK:
        if (start_flag == 0) {
            tracking_active = 0;
            stay_idle();
        } else {
            tracking_active = 1;
            motor_set_direction(MOTOR_LEFT, 1);
            motor_set_direction(MOTOR_RIGHT, 1);
        }
        break;

    default:
        break;
    }
}
