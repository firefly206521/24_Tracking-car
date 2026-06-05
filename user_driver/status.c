#include "status.h"
#include "motor.h"
#include "tracker.h"
#include "oled.h"
#include "idle.h"
#include "mpu_nav.h"
#include "pid_utils.h"
#include "straight.h"
#include "buzzer.h"

#define S1_DIST_PULSES   1650

system_status_t sys_status = STATUS_IDLE;
int              start_flag = 0;

static uint8_t s1_init    = 0;
static uint8_t m3_init    = 0;
static uint8_t track_init = 0;
extern volatile int encoder_motor1;
extern volatile int encoder_motor2;

// ===== 第三问状态机 =====
#define S3_BASE_SPEED     600.0f
#define S3_RAMP_STEP      20.0f
#define S3_SPEED_MAX      700.0f
#define S3_PRE_TURN_1     42.0f
#define S3_PRE_TURN_2     39.0f
#define S3_ALIGN_BOOST    1.5f
#define S3_TRACK_I_INIT   -20.0f  // 巡线积分初始值
#define S3_LINE_DEBOUNCE  3
#define S3_FORCE_TURN_DELAY_1    400  // 降速后强制转弯延时（第一段直道，ms）
#define S3_FORCE_TURN_DELAY_2    400  // 降速后强制转弯延时（第二段直道，ms）
#define S3_FORCE_TURN_EXTRA      5.0f // 强制转弯超出水平的角度（度）

enum { S3_STRAIGHT1=0, S3_ALIGN1, S3_CURVE1, S3_STRAIGHT2, S3_ALIGN2, S3_CURVE2, S3_DONE };
static uint8_t  s3_state = S3_STRAIGHT1;
static uint8_t  s3_init  = 0;
static float    s3_init_yaw;
static float    s3_ref_yaw;
static float    s3_ramp;
static uint8_t  s3_line_cnt, s3_on_line, s3_on_line_prev;
static uint32_t s3_track_ms;
static uint8_t  s3_track_ok;
static uint16_t s3_timeout;
static int8_t   s3_turn_dir;  // 1=右转, -1=左转, 交替
static uint16_t s3_curve_timer;
static int32_t s3_straight_start_enc;  // 刚进入直道时的编码器总和
static uint8_t s3_force_turn_active;   // 强制转弯已触发标记
static uint32_t s3_slowdown_ms;        // 降速开始时刻 (sys_tick_ms)
static uint8_t s3_line_prev;        // s3_on_line 前一拍，用于边沿检测
volatile float s3_dbg_err, s3_dbg_corr, s3_dbg_ramp;  // debug
volatile float s3_dbg_t1, s3_dbg_t2;                   // debug: 刚设完的 target
volatile float s3_dbg_t1_end, s3_dbg_t2_end;            // debug: case 末尾的 target

// 第三问独立 PID
static pid_ctrl_t s3_pid;

static void s3_pid_init(void)
{
    s3_pid.Kp = 3.0f; s3_pid.Ki = 0.3f; s3_pid.Kd = 0.5f;
    s3_pid.integral_max = 100.0f; s3_pid.error = 0; s3_pid.last_error = 0;
    s3_pid.integral = 0; s3_pid.output = 0;
}

void status_cycle_next(void)
{
    sys_status = (sys_status + 1) % STATUS_COUNT;
    start_flag  = 0;
    s1_init     = 0;
    m3_init     = 0;
    track_init  = 0;
    s3_init     = 0;
    change      = 0;
    motor_hard_brake_reset();
    stay_idle();
}

void status_toggle_start(void)
{
    start_flag ^= 1;
    pid_line.integral = 0;
    pid_line_q4.integral = 0;
    mpu_reset_zero(g_raw_yaw);
    s1_init    = 0;
    m3_init    = 0;
    track_init = 0;
    s3_init    = 0;
    change     = 0;
    tracking_active = 0;
    straight_force_stop();
    motor_hard_brake_reset();
}

// 第三问：线检测去抖
static void s3_update_line(void)
{
    uint8_t raw = straight_line_detected();
    if (raw == s3_on_line_prev) s3_line_cnt++;
    else { s3_line_cnt = 0; s3_on_line_prev = raw; }
    if (s3_line_cnt >= S3_LINE_DEBOUNCE) s3_on_line = raw;
}

void status_run(float yaw)
{
    switch (sys_status) {

    case STATUS_IDLE:
        stay_idle();
        break;

    case STATUS_DIST:
        if (start_flag == 0) { stay_idle(); s1_init = 0; }
        else if (!s1_init)   { straight_begin(yaw); s1_init = 1; }
        else if (straight_get_distance() / 2 >= S1_DIST_PULSES) {
            start_flag = 0; straight_force_stop();
            motor_hard_brake(MOTOR_RIGHT); motor_hard_brake(MOTOR_LEFT);
            buzzer_beep();
        }
        break;

    case STATUS_LINE_TRACK_2:
        if (start_flag == 0) {
            tracking_active = 0;
            stay_idle();
            track_init = 0;
        } else {
            if (!track_init) {
                float snapped = (yaw > 90.0f || yaw < -90.0f) ? 180.0f : 0.0f;
                straight_begin(snapped);
                tracking_active = 1;
                motor_set_direction(MOTOR_LEFT, 1);
                motor_set_direction(MOTOR_RIGHT, 1);
                track_init = 1;
            }
        }
        break;

    // ===== 第三问 / 第四问 =====
    case STATUS_LINE_TRACK_2ND:
    case STATUS_LINE_TRACK:
        if (start_flag == 0) {
            stay_idle(); s3_init = 0; s3_state = S3_STRAIGHT1; return;
        }
        if (!s3_init) {
            s3_init_yaw = yaw;
            s3_turn_dir = -1;  // 第一弯右转 (yaw - angle)
            s3_ref_yaw = normalize_angle(s3_init_yaw - S3_PRE_TURN_1);
            s3_ramp    = S3_BASE_SPEED * 0.5f;
            s3_pid_init();
            s3_state   = S3_STRAIGHT1;
            s3_on_line = 0; s3_on_line_prev = 0; s3_line_cnt = 0;
            s3_track_ok = 0; s3_timeout = 0;
            motor_set_direction(MOTOR_RIGHT, 1);
            motor_set_direction(MOTOR_LEFT, 1);
            s3_straight_start_enc = straight_enc_acc;
            s3_slowdown_ms = 0;
            s3_force_turn_active = 0;
            s3_line_prev = 0;
            change = 0;
            s3_init = 1;
        }
        {
            uint8_t raw = 0;
            for (int i = 0; i < 7; i++) {
                if (tracker_value[i] == 0) { raw = 1; break; }
            }
            if (raw == s3_on_line_prev) s3_line_cnt++;
            else { s3_line_cnt = 0; s3_on_line_prev = raw; }
            if (s3_line_cnt >= S3_LINE_DEBOUNCE) s3_on_line = raw;
        }
        pid_ctrl_t *pid_line_cur = (sys_status == STATUS_LINE_TRACK_2ND) ? &pid_line_q4 : &pid_line;
        extern volatile uint32_t sys_tick_ms;
        if (sys_tick_ms - s3_track_ms >= 30) { s3_track_ms = sys_tick_ms; s3_track_ok = 0; }

        if (s3_on_line != s3_line_prev) {
            s3_line_prev = s3_on_line;
            change++;
            buzzer_beep();
            if (change >= (sys_status == STATUS_LINE_TRACK_2ND ? 16 : 4)) {
                start_flag = 0;
                motor_hard_brake(MOTOR_RIGHT);
                motor_hard_brake(MOTOR_LEFT);
                break;
            }
        }

        if (s3_state == S3_STRAIGHT1) {
            if (s3_ramp < S3_BASE_SPEED && s3_slowdown_ms == 0) {
                s3_ramp += S3_RAMP_STEP;
                if (s3_ramp > S3_BASE_SPEED) s3_ramp = S3_BASE_SPEED;
            }
            float err  = -normalize_angle(yaw - s3_ref_yaw);
            float corr = pid_compute(&s3_pid, err);
            target_speed_1 = clamp_value(s3_ramp + corr, 0.0f, S3_SPEED_MAX);
            target_speed_2 = clamp_value(s3_ramp - corr, 0.0f, S3_SPEED_MAX);

            if (s3_ramp >= S3_BASE_SPEED && !s3_on_line) s3_timeout++;
            if (s3_timeout > 190 && s3_slowdown_ms == 0) { s3_ramp = 200.0f; s3_slowdown_ms = sys_tick_ms; }

            // 降速后延时强制转弯
            if (!s3_on_line && s3_slowdown_ms != 0 && sys_tick_ms - s3_slowdown_ms >= S3_FORCE_TURN_DELAY_1) {
                s3_ref_yaw = normalize_angle(s3_init_yaw + s3_turn_dir * S3_FORCE_TURN_EXTRA);
                s3_force_turn_active = 1;
            }

            if (s3_on_line) { s3_ramp = 300.0f; s3_state = S3_ALIGN1; }
        }
        else if (s3_state == S3_ALIGN1) {
            if (!s3_force_turn_active) s3_ref_yaw = s3_init_yaw;  // 左转回 0°
            float err_a = -normalize_angle(yaw - s3_ref_yaw);
            float corr_a = pid_compute(&s3_pid, err_a) * S3_ALIGN_BOOST;
            target_speed_1 = clamp_value(s3_ramp + corr_a, 0.0f, S3_SPEED_MAX);
            target_speed_2 = clamp_value(s3_ramp - corr_a, 0.0f, S3_SPEED_MAX);
            float d = normalize_angle(yaw - s3_ref_yaw);
            if (d < 0) d = -d;
            if (d < 5.0f) { pid_line_cur->integral = -s3_turn_dir * S3_TRACK_I_INIT; s3_force_turn_active = 0; s3_state = S3_CURVE1; }
        }
        else if (s3_state == S3_CURVE1) {
            if (!s3_track_ok) { s3_track_ok = 1; tracker_pid(s3_ramp, pid_line_cur); }
            if (s3_ramp < 500.0f) s3_ramp += 10.0f;
            if (!s3_on_line) {
                s3_turn_dir = -s3_turn_dir;
                s3_ref_yaw = normalize_angle(s3_init_yaw + 180.0f + s3_turn_dir * S3_PRE_TURN_2);
                s3_ramp  = S3_BASE_SPEED;
                s3_timeout = 0;
                s3_straight_start_enc = straight_enc_acc;
                s3_slowdown_ms = 0;
                pid_reset(&s3_pid);
                s3_state = S3_STRAIGHT2;
            }
        }
        else if (s3_state == S3_STRAIGHT2) {
            if (s3_ramp < S3_BASE_SPEED && s3_slowdown_ms == 0) { s3_ramp = S3_BASE_SPEED / 2.0f; }
            if (s3_ramp < S3_BASE_SPEED && s3_slowdown_ms == 0) {
                s3_ramp += S3_RAMP_STEP;
                if (s3_ramp > S3_BASE_SPEED) s3_ramp = S3_BASE_SPEED;
            }
            float err  = -normalize_angle(yaw - s3_ref_yaw);
            float corr = pid_compute(&s3_pid, err);
            target_speed_1 = clamp_value(s3_ramp + corr, 0.0f, S3_SPEED_MAX);
            target_speed_2 = clamp_value(s3_ramp - corr, 0.0f, S3_SPEED_MAX);

            if (s3_ramp >= S3_BASE_SPEED && !s3_on_line) s3_timeout++;
            if (s3_timeout > 190 && s3_slowdown_ms == 0) { s3_ramp = 200.0f; s3_slowdown_ms = sys_tick_ms; }

            // 降速后延时强制转弯
            if (!s3_on_line && s3_slowdown_ms != 0 && sys_tick_ms - s3_slowdown_ms >= S3_FORCE_TURN_DELAY_2) {
                float base = normalize_angle(s3_init_yaw + 180.0f);
                s3_ref_yaw = normalize_angle(base + s3_turn_dir * S3_FORCE_TURN_EXTRA);
                s3_force_turn_active = 1;
            }

            if (s3_on_line) { s3_ramp = 300.0f; s3_state = S3_ALIGN2; }
        }
        else if (s3_state == S3_ALIGN2) {
            float ref2 = normalize_angle(s3_init_yaw + 180.0f);
            if (!s3_force_turn_active) s3_ref_yaw = ref2;  // 右转回 180°
            float err_a = -normalize_angle(yaw - s3_ref_yaw);
            float corr_a = pid_compute(&s3_pid, err_a) * S3_ALIGN_BOOST;
            target_speed_1 = clamp_value(s3_ramp + corr_a, 0.0f, S3_SPEED_MAX);
            target_speed_2 = clamp_value(s3_ramp - corr_a, 0.0f, S3_SPEED_MAX);
            float d = normalize_angle(yaw - s3_ref_yaw);
            if (d < 0) d = -d;
            if (d < 5.0f) { pid_line_cur->integral = -s3_turn_dir * S3_TRACK_I_INIT; s3_force_turn_active = 0; s3_state = S3_CURVE2; }
        }
        else if (s3_state == S3_CURVE2) {
            if (!s3_track_ok) { s3_track_ok = 1; tracker_pid(s3_ramp, pid_line_cur); }
            if (s3_ramp < 500.0f) s3_ramp += 5.0f;
            if (!s3_on_line) {
                s3_turn_dir = -s3_turn_dir;
                {
                    float base, pre;
                    if (s3_turn_dir == -1) {
                        base = s3_init_yaw;
                        pre  = S3_PRE_TURN_1;  // STRAIGHT1 用 PRE_TURN_1
                    } else {
                        base = normalize_angle(s3_init_yaw + 180.0f);
                        pre  = S3_PRE_TURN_2;  // STRAIGHT2 用 PRE_TURN_2
                    }
                    s3_ref_yaw = normalize_angle(base + s3_turn_dir * pre);
                }
                s3_ramp  = S3_BASE_SPEED;
                s3_timeout = 0;
                s3_straight_start_enc = straight_enc_acc;
                s3_slowdown_ms = 0;
                pid_reset(&s3_pid);
                s3_state = (s3_turn_dir == -1) ? S3_STRAIGHT1 : S3_STRAIGHT2;
            }
        }
        else { stay_idle(); }
        break;

    default:
        break;
    }
}
