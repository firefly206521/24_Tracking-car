#include "tracker.h"
#include "motor.h"

uint8_t tracker_value[] = {0,0,0,0,0,0,0};
volatile uint8_t tracking_active = 0;

pid_ctrl_t pid_line = {
    .Kp = 900.0f,
    .Ki = 10.0f,
    .Kd = 0.0f,
    .integral_max = 30.0f
};

#define BASE_SPEED  500
#define MAX_SPEED   1200
#define MIN_SPEED   50

extern uint8_t get_gpio_value(GPIO_Regs* gpio_port, uint32_t gpio)
{
    uint32_t high_bits = DL_GPIO_readPins(gpio_port, gpio);
    if ((high_bits & gpio) == 0) {
        return 1;
    } else {
        return 0;
    }
}

void tracker_get_value()
{
    tracker_value[0] = get_gpio_value(tracker_L0_PORT, tracker_L0_PIN);
    tracker_value[1] = get_gpio_value(tracker_L1_PORT, tracker_L1_PIN);
    tracker_value[2] = get_gpio_value(tracker_L2_PORT, tracker_L2_PIN);
    tracker_value[3] = get_gpio_value(tracker_MID_PORT, tracker_MID_PIN);
    tracker_value[4] = get_gpio_value(tracker_R1_PORT, tracker_R1_PIN);
    tracker_value[5] = get_gpio_value(tracker_R2_PORT, tracker_R2_PIN);
    tracker_value[6] = get_gpio_value(tracker_R0_PORT, tracker_R0_PIN);
}

float compute_error(uint8_t *val, pid_ctrl_t *pid)
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
        return pid->last_error;
    }
    float position = (float)weighted_sum / sum;
    return (position - 300.0f) / 300.0f;
}

void tracker_pid(float base_speed, pid_ctrl_t *pid)
{
    float error = compute_error(tracker_value, pid);
    float output = pid_compute(pid, error);
    int16_t left_speed  = base_speed + (int16_t)output;
    int16_t right_speed = base_speed - (int16_t)output;
    left_speed  = clamp_value(left_speed,  MIN_SPEED, MAX_SPEED);
    right_speed = clamp_value(right_speed, MIN_SPEED, MAX_SPEED);
    target_speed_2 = left_speed;
    target_speed_1 = right_speed;
}

void track_line()
{
    tracker_pid(BASE_SPEED, &pid_line);
}
