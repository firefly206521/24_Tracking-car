#include "motor.h"

int encoder_motor1;
int encoder_motor2;
volatile float speed_1=0;
volatile float speed_2=0;

//pid所用数量
int16_t PWM_1_duty=0;
int16_t PWM_2_duty=0;
float last_error_1=0;
float current_error_1=0;
float last_error_2=0;
float current_error_2=0;
//电机速度
volatile float speed_1;
volatile float speed_2;
float target_speed_1;
float target_speed_2;

//pid参数
float Kp1=0.5;//比例系数
float Ki1=0.4;//积分系数
float Kp2=0.5;//比例系数
float Ki2=0.4;//积分系数


//初始化电机
void motor_init(uint8_t motor_id)
{
    DL_GPIO_setPins(MOTOR_STBY_PORT,MOTOR_STBY_PIN);
    
    if (motor_id==1){
        DL_GPIO_setPins(MOTOR_AIN1_PORT,MOTOR_AIN1_PIN);
        DL_GPIO_setPins(MOTOR_AIN2_PORT,MOTOR_AIN2_PIN);
        DL_Timer_setCaptureCompareValue(PWMAB_INST,0,GPIO_PWMAB_C0_IDX);
    }

    else if(motor_id==2){
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

    if (motor_id==1){
        DL_Timer_setCaptureCompareValue(PWMAB_INST,duty,GPIO_PWMAB_C0_IDX);
    }
    else if(motor_id==2){
        DL_Timer_setCaptureCompareValue(PWMAB_INST,duty,GPIO_PWMAB_C1_IDX);
    }
}

//电机正反转设置
//0:停止 1:正转 2:反转s
void motor_set_direction(uint8_t motor_id,uint8_t direction)
{
    if (motor_id==1){
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
    else if(motor_id==2){
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
    if (motor_id==1){
        speed=(float)encoder_motor1*PI*tire_R/encoder_k*100;
        //单位mm/s
        //timer触发的周期是25ms，所以40是1000ms/25ms
        encoder_motor1=0;
    }
    else if(motor_id==2){
        speed=(float)encoder_motor2*PI*tire_R/encoder_k*100;
        encoder_motor2=0;
    }
    return speed;
    //Add similar logic for motor_id==2 if needed
}


void MOTOR_PID_INST_IRQHandler()
{
    switch (DL_Timer_getPendingInterrupt(MOTOR_PID_INST))
    {
    //因为有很多个定时器的选项，例如load event、zero event、capture compare event等，所以需要判断是哪一个事件触发了中断
    case DL_TIMER_IIDX_LOAD:
        speed_1=speed_calculate(1);
        speed_2=speed_calculate(2);
        MOTOR_PID(1,target_speed_1);
        MOTOR_PID(2,target_speed_2);
        break;

    default:
        break;
    }
}

//调速,增量式pid

uint32_t limit_duty(uint32_t duty){
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
    if(motor_id==1){
        error=target_speed-speed_1;
        current_error_1=error;
        PWM_1_duty=PWM_1_duty+Kp1*(current_error_1-last_error_1)+Ki1*current_error_1;
        last_error_1=current_error_1;
        motor_set_duty(1,PWM_1_duty);
    }
    if(motor_id==2){
        error=target_speed-speed_2;
        current_error_2=error;
        PWM_2_duty=PWM_2_duty+Kp2*(current_error_2-last_error_2)+Ki2*current_error_2;
        last_error_2=current_error_2;
        motor_set_duty(2,PWM_2_duty);
    }
}




