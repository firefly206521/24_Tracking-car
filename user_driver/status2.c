/**
 * 第二题：400m 跑道 —— 直道空白，弯道有黑线
 * 小车从 0 加速直行 → 遇线循迹 → 出线后以初始+180° 直行 → 再遇线循迹 → 出线停车
 */

#include "status2.h"
#include "motor.h"
#include "tracker.h"
#include "straight.h"
#include "idle.h"
#include "pid_utils.h"

static uint8_t s2_init         = 0;
static uint8_t s2_phase        = S2_PHASE_STRAIGHT1;
static float   s2_init_yaw     = 0.0f;
static uint8_t s2_prev_on_line = 0;

void status2_reset(void)
{
    s2_init = 0;
    s2_phase = S2_PHASE_STRAIGHT1;
}

void status2_run(float yaw, int start_flag)
{
    if (start_flag == 0) {
        stay_idle();
        s2_phase = S2_PHASE_STRAIGHT1;
        return;
    }

    // 首次进入：保存初始角度，斜坡启动
    if (!s2_init) {
        s2_init_yaw     = yaw;
        s2_phase        = S2_PHASE_STRAIGHT1;
        s2_prev_on_line = 0;
        straight_nav_begin(yaw);
        s2_init = 1;
    }

    // 边缘检测：进入 / 离开黑线
    uint8_t on_line      = straight_line_detected();
    uint8_t line_rising  = on_line && !s2_prev_on_line;
    uint8_t line_falling = !on_line && s2_prev_on_line;
    s2_prev_on_line      = on_line;

    switch (s2_phase) {

    case S2_PHASE_STRAIGHT1:
        if (line_rising) {
            nav_curve_mode   = 1;
            tracking_active  = 1;
            pid_line.integral = 0;
            s2_phase = S2_PHASE_CURVE1;
        } else {
            straight_nav_run(yaw);
        }
        break;

    case S2_PHASE_CURVE1:
        if (line_falling) {
            tracking_active = 0;
            nav_curve_mode  = 0;
            straight_force_stop();
            float target = normalize_angle(s2_init_yaw + 180.0f);
            straight_nav_resume(target);
            s2_phase = S2_PHASE_STRAIGHT2;
        }
        break;

    case S2_PHASE_STRAIGHT2:
        if (line_rising) {
            nav_curve_mode   = 1;
            tracking_active  = 1;
            pid_line.integral = 0;
            s2_phase = S2_PHASE_CURVE2;
        } else {
            straight_nav_run(yaw);
        }
        break;

    case S2_PHASE_CURVE2:
        if (line_falling) {
            tracking_active = 0;
            nav_curve_mode  = 0;
            straight_force_stop();
            stay_idle();
            s2_phase = S2_PHASE_DONE;
        }
        break;

    case S2_PHASE_DONE:
        stay_idle();
        break;
    }
}
