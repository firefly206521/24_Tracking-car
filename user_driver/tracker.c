#include "tracker.h"
#include "motor.h"
uint8_t tracker_value[]={0,0,0,0,0,0,0};
volatile uint8_t tracking_active = 0;
float Kp = 80.0f;
float Ki = 0.0f;
float Kd = 0.0f;
tracker_pid_t pid_line = {0};
tracker_pid_t pid_arc  = {0};
#define BASE_SPEED  800   // 直道的基准速度（左右轮共同的基础值）
#define MAX_SPEED   1200  // 最高速度，防止超速冲出赛道
#define MIN_SPEED   50  // 最低速度，防止电机停转或无力


extern uint8_t get_gpio_value(GPIO_Regs* gpio_port, uint32_t gpio)
{
    uint32_t high_bits = DL_GPIO_readPins(gpio_port, gpio); // 必须用32位接！
    // 根据你的传感器特性选择逻辑（黑线=低电平→返回1 ，或反过来）
    if ((high_bits & gpio) == 0) {
        return 1;   // 检测到黑线
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
float compute_error(uint8_t *val, tracker_pid_t *pid)
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

void tracker_pid(float base_speed, tracker_pid_t *pid)
{
    pid->error = compute_error(tracker_value, pid);
    pid->integral += pid->error;
    if (pid->integral > 30.0f) pid->integral = 30.0f;
    if (pid->integral < -30.0f) pid->integral = -30.0f;
    float derivative = pid->error - pid->last_error;
    pid->pid_output = Kp * pid->error + Ki * pid->integral + Kd * derivative;
    pid->last_error = pid->error;
    int16_t left_speed  = base_speed + (int16_t)pid->pid_output;
    int16_t right_speed = base_speed - (int16_t)pid->pid_output;
    if (left_speed > MAX_SPEED)  left_speed = MAX_SPEED;
    if (left_speed < MIN_SPEED)  left_speed = MIN_SPEED;
    if (right_speed > MAX_SPEED) right_speed = MAX_SPEED;
    if (right_speed < MIN_SPEED) right_speed = MIN_SPEED;
    target_speed_2 = left_speed;
    target_speed_1 = right_speed;
}

void track_line(){
    tracker_pid(BASE_SPEED, &pid_line);
}







