#include "motor.h"

volatile int encoder_motor1;
volatile int encoder_motor2;
volatile float speed_1=0;
volatile float speed_2=0;

//pid所用数量 — 位置式 PID
int32_t PWM_1_duty=0;
int32_t PWM_2_duty=0;
float integral_1=0;    // 右电机积分累加
float integral_2=0;    // 左电机积分累加
//电机速度
float target_speed_1;
float target_speed_2;

//PID分频：硬件定时器10ms，每5次(50ms)执行一次PID
#define PID_DECIMATION    5
//速度低通滤波系数 (0~1，越小越平滑)
#define SPEED_FILTER_ALPHA 0.3f
static uint8_t pid_divider = 0;

//pid参数 — 位置式 PID: PWM = Kp×error + Ki×integral
float Kp1=8.0;//比例系数
float Ki1=2.0;//积分系数



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

float speed_calculate(int motor_id){
    float speed=0.0;
    if (motor_id==MOTOR_RIGHT){
        speed=(float)encoder_motor1*PI*tire_R/encoder_k*20;
        //单位mm/s，PID有效周期50ms，系数=1000/50=20
        encoder_motor1=0;
    }
    else if(motor_id==MOTOR_LEFT){
        speed=(float)encoder_motor2*PI*tire_R/encoder_k*20;
        encoder_motor2=0;
    }
    return speed;
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
            MOTOR_PID(MOTOR_RIGHT,target_speed_1);
            MOTOR_PID(MOTOR_LEFT,target_speed_2);
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




