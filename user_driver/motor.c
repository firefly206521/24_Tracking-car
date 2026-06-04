#include "motor.h"
#include "tracker.h"
#include "straight.h"
#include "status.h"

volatile int encoder_motor1;
volatile int encoder_motor2;
volatile float speed_1=0;
volatile float speed_2=0;
uint8_t all_lost =0;

//pid所用数量 — 位置式 PID
int32_t PWM_1_duty=0;
int32_t PWM_2_duty=0;
float integral_1=0;    // 右电机积分累加
float integral_2=0;    // 左电机积分累加
//电机速度
volatile float target_speed_1;
volatile float target_speed_2;

//PID分频：硬件定时器10ms，每5次(50ms)执行一次PID
#define PID_DECIMATION    1
//速度低通滤波系数 (0~1，越小越平滑)
#define SPEED_FILTER_ALPHA 0.7f
static uint8_t pid_divider = 0;

//pid参数 — 位置式 PID: PWM = Kp×error + Ki×integral
float Kp1=12.0;//比例系数
float Ki1=2.0;//积分系数

uint8_t last_status;
uint8_t change = 0;
#define STRAIGHT 0
#define TRACK 1

// 硬刹停：反转 PWM 维持计数器
#define HARD_BRAKE_TICKS 8   // 8 * 30ms = 240ms 反转制动
#define HARD_BRAKE_PWM   4000
static int8_t hard_brake_1 = -1;  // RIGHT, -1=空闲
static int8_t hard_brake_2 = -1;  // LEFT

//初始化电机
void motor_init(uint8_t motor_id)
{
    DL_GPIO_setPins(MOTOR_STBY_PORT,MOTOR_STBY_PIN);
    
    if (motor_id==MOTOR_RIGHT){
        DL_GPIO_setPins(MOTOR_AIN1_PORT,MOTOR_AIN1_PIN);
        DL_GPIO_setPins(MOTOR_AIN2_PORT,MOTOR_AIN2_PIN);
        DL_Timer_setCaptureCompareValue(PWMAB_INST,0,GPIO_PWMAB_C0_IDX);
    }

    else if(motor_id==MOTOR_LEFT){
        DL_GPIO_setPins(MOTOR_BIN1_PORT,MOTOR_BIN1_PIN);
        DL_GPIO_setPins(MOTOR_BIN2_PORT,MOTOR_BIN2_PIN);
        DL_Timer_setCaptureCompareValue(PWMAB_INST,0,GPIO_PWMAB_C1_IDX);
    }
    DL_Timer_startCounter(MOTOR_PID_INST);
    DL_TimerA_startCounter(PWMAB_INST);
    //开始电机定时器
    NVIC_EnableIRQ(MOTOR_PID_INST_INT_IRQN);
    //开启定时器中断，然后写MOTOR_PID_INST_IRQHandler函数来实现PID控制

}


//电机速度设置
void motor_set_duty(uint8_t motor_id,uint32_t duty)
{
    if (duty>4000){
        duty=4000;
    }

    if (motor_id==MOTOR_RIGHT){
        DL_Timer_setCaptureCompareValue(PWMAB_INST,duty,GPIO_PWMAB_C0_IDX);
    }
    else if(motor_id==MOTOR_LEFT){
        DL_Timer_setCaptureCompareValue(PWMAB_INST,duty,GPIO_PWMAB_C1_IDX);
    }
}

//电机正反转设置
//0:停止 1:正转 2:反转s
void motor_set_direction(uint8_t motor_id,uint8_t direction)
{
    if (motor_id==MOTOR_RIGHT){
        if (direction==0){
            DL_GPIO_clearPins(MOTOR_AIN1_PORT,MOTOR_AIN1_PIN);
            DL_GPIO_clearPins(MOTOR_AIN2_PORT,MOTOR_AIN2_PIN);
        }
        else if(direction==1){
            DL_GPIO_clearPins(MOTOR_AIN1_PORT,MOTOR_AIN1_PIN);
            DL_GPIO_setPins(MOTOR_AIN2_PORT,MOTOR_AIN2_PIN);
        }
        else if(direction==2){
            DL_GPIO_setPins(MOTOR_AIN1_PORT,MOTOR_AIN1_PIN);
            DL_GPIO_clearPins(MOTOR_AIN2_PORT,MOTOR_AIN2_PIN);
        }
    }
    else if(motor_id==MOTOR_LEFT){
        if (direction==0){
            DL_GPIO_clearPins(MOTOR_BIN1_PORT,MOTOR_BIN1_PIN);
            DL_GPIO_clearPins(MOTOR_BIN2_PORT,MOTOR_BIN2_PIN);
        }
        else if(direction==1){
            DL_GPIO_clearPins(MOTOR_BIN1_PORT,MOTOR_BIN1_PIN);
            DL_GPIO_setPins(MOTOR_BIN2_PORT,MOTOR_BIN2_PIN);
        }
        else if(direction==2){
            DL_GPIO_setPins(MOTOR_BIN1_PORT,MOTOR_BIN1_PIN);
            DL_GPIO_clearPins(MOTOR_BIN2_PORT,MOTOR_BIN2_PIN);
        }
    }
}

// TB6612 制动：IN1 和 IN2 同时 HIGH，电机端子短路制动
void motor_brake(uint8_t motor_id)
{
    if (motor_id == MOTOR_RIGHT) {
        DL_Timer_setCaptureCompareValue(PWMAB_INST, 4000, GPIO_PWMAB_C0_IDX); // PWM 拉满
        DL_GPIO_setPins(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN);
        DL_GPIO_setPins(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN);
    } else if (motor_id == MOTOR_LEFT) {
        DL_Timer_setCaptureCompareValue(PWMAB_INST, 4000, GPIO_PWMAB_C1_IDX); // PWM 拉满
        DL_GPIO_setPins(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN);
        DL_GPIO_setPins(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN);
    }
}

float speed_calculate(int motor_id){
    float speed=0.0;
    if (motor_id==MOTOR_RIGHT){
        speed=(float)encoder_motor1*PI*tire_R/encoder_k*33.33f;
        //单位mm/s，PID有效周期30ms，系数=1000/30≈33.33
        straight_enc_acc += encoder_motor1;
        encoder_motor1=0;
    }
    else if(motor_id==MOTOR_LEFT){
        speed=(float)encoder_motor2*PI*tire_R/encoder_k*33.33f;
        straight_enc_acc += encoder_motor2;
        encoder_motor2=0;
    }
    return speed;
}

void question2_run()
{

    if (change < 4) {
        if (tracking_active) {
            all_lost = 0;
            for (int i = 0; i < 7; i++) {
                all_lost+= tracker_value[i];
            }
            if(all_lost==7){
                all_lost=1;
            }
            else{
                all_lost=0;
            }
            if (all_lost) {
                float snapped = (g_yaw > 90.0f || g_yaw < -90.0f) ? 180.0f : 0.0f;
                straight_nav_update_ref(snapped);
                straight_nav_run(g_yaw);
                if(last_status != STRAIGHT){
                    change++;
                    last_status = STRAIGHT;
                }
            }
            else {
                if(last_status != TRACK){
                    change++;
                    last_status = TRACK;
                }
                track_line();
            }
        }
        else if (straight_is_active()) {

            if (straight_run(g_yaw)) {
                tracking_active = 1;
                pid_line.integral = 0;
            }
        }
    }
    else if (change == 4) {
        motor_hard_brake(MOTOR_RIGHT);
        motor_hard_brake(MOTOR_LEFT);
        change = 5;
    }
}


// ISR 每 30ms 调用一次，倒计时并维持反转方向+PWM
static void hard_brake_tick(void)
{
    if (hard_brake_1 > 0) {
        hard_brake_1--;
        motor_set_direction(MOTOR_RIGHT, 2);
        motor_set_duty(MOTOR_RIGHT, HARD_BRAKE_PWM);
        if (hard_brake_1 == 0) {
            motor_brake(MOTOR_RIGHT);
            hard_brake_1 = -1;
        }
    }
    if (hard_brake_2 > 0) {
        hard_brake_2--;
        motor_set_direction(MOTOR_LEFT, 2);
        motor_set_duty(MOTOR_LEFT, HARD_BRAKE_PWM);
        if (hard_brake_2 == 0) {
            motor_brake(MOTOR_LEFT);
            hard_brake_2 = -1;
        }
    }
}

void motor_hard_brake(uint8_t motor_id)
{
    if (motor_id == MOTOR_RIGHT) {
        if (hard_brake_1 >= 0) return;  // 正在刹停或已完成，不重复触发
        hard_brake_1    = HARD_BRAKE_TICKS;
        integral_1      = 0;
        target_speed_1  = 0;
        motor_set_direction(MOTOR_RIGHT, 2);
        motor_set_duty(MOTOR_RIGHT, HARD_BRAKE_PWM);
    } else if (motor_id == MOTOR_LEFT) {
        if (hard_brake_2 >= 0) return;
        hard_brake_2    = HARD_BRAKE_TICKS;
        integral_2      = 0;
        target_speed_2  = 0;
        motor_set_direction(MOTOR_LEFT, 2);
        motor_set_duty(MOTOR_LEFT, HARD_BRAKE_PWM);
    }
}

void motor_hard_brake_reset(void)
{
    hard_brake_1 = -1;
    hard_brake_2 = -1;
}

void MOTOR_PID_INST_IRQHandler()
{
    switch (DL_Timer_getPendingInterrupt(MOTOR_PID_INST))
    {
    //因为有很多个定时器的选项，例如load event、zero event、capture compare event等，所以需要判断是哪一个事件触发了中断
    case DL_TIMER_IIDX_LOAD:
        pid_divider++;
        if (pid_divider >= PID_DECIMATION) {
            pid_divider = 0;
            float raw_1 = speed_calculate(MOTOR_RIGHT);
            float raw_2 = speed_calculate(MOTOR_LEFT);
            speed_1 = speed_1 * (1.0f - SPEED_FILTER_ALPHA) + raw_1 * SPEED_FILTER_ALPHA;
            speed_2 = speed_2 * (1.0f - SPEED_FILTER_ALPHA) + raw_2 * SPEED_FILTER_ALPHA;
            // 统一读取传感器，20Hz 路由：循迹 / 直行
            tracker_get_value();
            if (sys_status == STATUS_LINE_TRACK_2) {
                question2_run();
            }
            hard_brake_tick();
            if (hard_brake_1 <= 0) MOTOR_PID(MOTOR_RIGHT, target_speed_1);
            if (hard_brake_2 <= 0) MOTOR_PID(MOTOR_LEFT,  target_speed_2);
        }
        break;

    default:
        break;
    }
}

//调速,位置式 PID: PWM = Kp×error + Ki×integral

int32_t limit_duty(int32_t duty){
    if (duty>4000){
        duty=4000;
    }
    else if(duty<0){
        duty=0;
    }
    return duty;
}

// 差速追赶：右轮加速左轮减速，追 220 个脉冲差后同速
// 主循环调用 motor_align_pulses()，返 1 完成
uint8_t motor_align_pulses(void)
{
    #define ALIGN_GAP 220
    static int32_t enc_r0 = 0, enc_l0 = 0;
    static uint8_t active = 0;

    if (!active) {
        enc_r0 = encoder_motor1;
        enc_l0 = encoder_motor2;
        active  = 1;
    }

    int32_t dr = encoder_motor1 - enc_r0;
    int32_t dl = encoder_motor2 - enc_l0;

    if (dr - dl >= ALIGN_GAP) {
        target_speed_1 = STRAIGHT_BASE_SPEED;
        target_speed_2 = STRAIGHT_BASE_SPEED;
        active = 0;
        return 1;
    }

    target_speed_1 = STRAIGHT_BASE_SPEED + 300.0f;
    target_speed_2 = STRAIGHT_BASE_SPEED - 300.0f;
    return 0;
}

void MOTOR_PID(uint8_t motor_id,float target_speed){
    float error;
    if(motor_id==MOTOR_RIGHT){
        error=target_speed-speed_1;
        integral_1 += error;
        if (integral_1 > 1500.0f) integral_1 = 1500.0f;
        if (integral_1 < -1500.0f) integral_1 = -1500.0f;
        PWM_1_duty=limit_duty((int32_t)(Kp1*error + Ki1*integral_1));
        motor_set_duty(MOTOR_RIGHT,(uint32_t)PWM_1_duty);
    }
    if(motor_id==MOTOR_LEFT){
        error=target_speed-speed_2;
        integral_2 += error;
        if (integral_2 > 1500.0f) integral_2 = 1500.0f;
        if (integral_2 < -1500.0f) integral_2 = -1500.0f;
        PWM_2_duty=limit_duty((int32_t)(Kp1*error + Ki1*integral_2));
        motor_set_duty(MOTOR_LEFT,(uint32_t)PWM_2_duty);
    }
}




