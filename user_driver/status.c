#include "status.h"
#include "motor.h"
#include "tracker.h"
#include "oled.h"
#include "idle.h"
#include "mpu_nav.h"
#include "straight.h"
#include "status2.h"

system_status_t sys_status = STATUS_IDLE;
int              start_flag = 0;

void status_cycle_next(void)
{
    sys_status = (sys_status + 1) % STATUS_COUNT;
    start_flag = 0;
    status2_reset();
}

void status_toggle_start(void)
{
    start_flag ^= 1;
    pid_line.integral = 0;
    mpu_reset_zero(g_raw_yaw);
    status2_reset();
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

    case STATUS_MPU_NAV:   // 第二题
        status2_run(yaw, start_flag);
        break;

    default:
        break;
    }
}
