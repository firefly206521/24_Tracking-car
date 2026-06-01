#include "tracker.h"
#include "motor.h"

uint8_t tracker_value[] = {0,0,0,0,0,0,0};
volatile uint8_t tracking_active = 0;

TrackLevelConfig track_levels[TRACK_NUM_LEVELS] = {
    // L1: 低速找线，纯P稳为主，不给积分
    { 90.0f, 0.0f, 0.0f, 150, 0 },
    // L2: 过渡加速，加入积分
    { 320.0f, 1.6f, 0.0f, 350, 4 },
    // L3: 高速巡航
    { 480.0f, 4.0f, 0.0f, 550, 0 },
};

TrackState track_state = {0};

extern uint8_t get_gpio_value(GPIO_Regs* gpio_port, uint32_t gpio)
{
    uint32_t high_bits = DL_GPIO_readPins(gpio_port, gpio);
    if ((high_bits & gpio) == 0) {
        return 1;
    } else {
        return 0;
    }
}

void tracker_get_value(void)
{
    tracker_value[0] = get_gpio_value(tracker_L0_PORT, tracker_L0_PIN);
    tracker_value[1] = get_gpio_value(tracker_L1_PORT, tracker_L1_PIN);
    tracker_value[2] = get_gpio_value(tracker_L2_PORT, tracker_L2_PIN);
    tracker_value[3] = get_gpio_value(tracker_MID_PORT, tracker_MID_PIN);
    tracker_value[4] = get_gpio_value(tracker_R1_PORT, tracker_R1_PIN);
    tracker_value[5] = get_gpio_value(tracker_R2_PORT, tracker_R2_PIN);
    tracker_value[6] = get_gpio_value(tracker_R0_PORT, tracker_R0_PIN);
}

static float compute_error(uint8_t *val, TrackState *state)
{
    int sum = 0;
    int weighted_sum = 0;
    for (int i = 0; i < 7; i++) {
        if (val[i] == 1) {
            sum += 1;
            weighted_sum += i * 100;
        }
    }
    if (sum == 0) {
        return state->last_error;
    }
    float position = (float)weighted_sum / sum;
    return (position - 300.0f) / 300.0f;
}

void track_line(void)
{
    TrackLevelConfig *cfg = &track_levels[track_state.level];

    tracker_get_value();
    track_state.error = compute_error(tracker_value, &track_state);
    track_state.integral += track_state.error;

    if (track_state.integral > 100.0f)  track_state.integral = 100.0f;
    if (track_state.integral < -100.0f) track_state.integral = -100.0f;

    float derivative = track_state.error - track_state.last_error;
    float pid_output = cfg->Kp * track_state.error
                     + cfg->Ki * track_state.integral
                     + cfg->Kd * derivative;
    track_state.last_error = track_state.error;

    int16_t left_speed  = cfg->base_speed + (int16_t)pid_output;
    int16_t right_speed = cfg->base_speed - (int16_t)pid_output;

    #define MAX_SPEED 800
    #define MIN_SPEED 50
    if (left_speed  > MAX_SPEED) left_speed  = MAX_SPEED;
    if (left_speed  < MIN_SPEED) left_speed  = MIN_SPEED;
    if (right_speed > MAX_SPEED) right_speed = MAX_SPEED;
    if (right_speed < MIN_SPEED) right_speed = MIN_SPEED;

    target_speed_2 = left_speed;
    target_speed_1 = right_speed;

    // 速度级别状态机
    track_state.level_timer++;

    if (track_state.level == 0) {
        // L1→L2: 中间传感器(L1/MID/R1)至少2个看到线,连续确认
        uint8_t center_cnt = tracker_value[2] + tracker_value[3] + tracker_value[4];
        if (center_cnt >= 2) {
            track_state.lock_cnt++;
        } else {
            track_state.lock_cnt = 0;
        }
        if (track_state.lock_cnt >= TRACK_LOCK_CYCLES) {
            track_state.level = 1;
            track_state.level_timer = 0;
        }
    } else if (track_state.level == 1) {
        // L2→L3: 定时升级
        if (track_state.level_timer >= cfg->hold_cycles) {
            track_state.level = 2;
            track_state.level_timer = 0;
        }
    }
}

void track_set_direction(TrackDirection dir)
{
    if (dir != track_state.direction) {
        track_state.integral = -track_state.integral;
        track_state.direction = dir;
        track_reset_level();
    }
}

void track_reset_level(void)
{
    track_state.level = 0;
    track_state.lock_cnt = 0;
    track_state.level_timer = 0;
}
