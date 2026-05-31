#include "tracker.h"
#include "motor.h"
uint8_t tracker_value[]={0,0,0,0,0,0,0};
volatile uint8_t tracking_active = 0;  // 循迹模式激活标志
float Kp = 320.0f;    // 比例系数
float Ki = 1.6f;    // 积分系数
float Kd = 0.0f;     // 微分系数，阻尼振荡
float error = 0;         // 当前偏差
float last_error = 0;    // 上一次偏差，用于微分项+脱线保持
float integral = 0;      // 积分累加和
float pid_output = 0;     // PID 控制器的输出值
#define BASE_SPEED  300   // 直道的基准速度（左右轮共同的基础值）
#define MAX_SPEED   800  // 最高速度，防止超速冲出赛道
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
float compute_error(uint8_t *val)
{
    int sum = 0;          // 记录有几个传感器检测到黑线
    int weighted_sum = 0; // 记录这些传感器位置（索引）的和，乘以100避免浮点
    for (int i = 0; i < 7; i++) {
        if (val[i] == 1) {       // 假设1代表黑线
            sum += 1;
            weighted_sum += i * 100;   // 用 i * 100 放大，提高精度
        }
    }
    // 如果全部是白线（小车完全脱线了）
    if (sum == 0) {
        return last_error;   // 返回上一次的误差，让小车维持原方向（保守策略）
    }
    // 计算黑线的加权平均位置（值在 0 ~ 600 之间，中心是 300）
    float position = (float)weighted_sum / sum;
    // 映射到 [-1, 1] 区间，0 表示正好居中
    float err = (position - 300.0f) / 300.0f;
    return err;
}

void track_line(){
    // ① 读取传感器数据到 tracker_value 数组
    tracker_get_value();
    // ② 计算当前偏差
    error = compute_error(tracker_value);
    // ③ 累加积分
    integral += error;
    if (integral > 30.0f) integral = 30.0f;
    if (integral < -30.0f) integral = -30.0f;
    // ④ PID 控制量 = P + I + D（由 50ms ISR 定时调用）
    float derivative = error - last_error;
    pid_output = Kp * error + Ki * integral + Kd * derivative;
    last_error = error;
    // ⑤ 转化为左右轮速度差
    int16_t left_speed  = BASE_SPEED + (int16_t)pid_output;
    int16_t right_speed = BASE_SPEED - (int16_t)pid_output;
    // ⑥ 限幅
    if (left_speed > MAX_SPEED)  left_speed = MAX_SPEED;
    if (left_speed < MIN_SPEED)  left_speed = MIN_SPEED;
    if (right_speed > MAX_SPEED) right_speed = MAX_SPEED;
    if (right_speed < MIN_SPEED) right_speed = MIN_SPEED;
    // ⑦ 输出目标速度
    target_speed_2 = left_speed;     // MOTOR_LEFT=2 → 左电机
    target_speed_1 = right_speed;    // MOTOR_RIGHT=1 → 右电机
}







