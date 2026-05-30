/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include <stdio.h>
#include "delay.h"
#include "motor.h"
#include "key.h"
#include "oled.h"
#include "tracker.h"
#include "idle.h"
#include "uart.h"
#include "mpu_nav.h"
#include "mpu_port.h"

int status=0;//改变状态机
int start_flag=0;//确定改变状态
int encoder_motor1=0;
int encoder_motor2=0;
float target_speed_1=0;
float target_speed_2=0;
extern int16_t PWM_1_duty;
extern volatile float speed_1;
extern volatile float speed_2;
extern uint8_t tracker_value[];

extern volatile uint32_t sys_tick_ms;

// SysTick 中断服务函数 (每 1ms 触发一次)
void SysTick_Handler(void) {
    sys_tick_ms++;
}

int main(void)
{
    int cnt = 0;
    delay_ms(1000);
    SYSCFG_DL_init();
    delay_ms(1000);//等待电源稳定，保证IIC上电成功
    //OLED初始化
    OLED_Init();
    OLED_ColorTurn(0);//0正常显示，1反色显示
    OLED_DisplayTurn(0);//0正常显示，1屏幕旋转180度
    OLED_Clear();

    //电机初始化
    motor_init(1);
    motor_init(2);

    //陀螺仪初始化
    //while(DMP_Init());
    //float pitch = 0, roll = 0, yaw = 0;


    char yaw_buf[40];
    char line0[20], line1[20], line2[20], line3[20];
    int oled_refresh_cnt = 0;

    //给电机和案件引脚设置为开启中断功能
    NVIC_EnableIRQ(MOTOR_EC1A_IIDX);
    NVIC_EnableIRQ(MOTOR_EC2A_IIDX);
    NVIC_EnableIRQ(KEY_KEY_1_IIDX);
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    NVIC_EnableIRQ(MOTOR_GPIOA_INT_IRQN);
    motor_set_direction(1, 0);
    motor_set_direction(2, 0);

    while (1) {

        tracker_get_value();
        //while(DMP_Read_Data(&pitch,&roll,&yaw));
        // 调试时注释掉上面陀螺仪

        

        // OLED 每 50 次 (约50ms) 刷新一次
        // oled_refresh_cnt++;
        // if (oled_refresh_cnt >= 50) {
        //     oled_refresh_cnt = 0;
            snprintf(line0, sizeof(line0), "S:%d", status);
            //snprintf(line1, sizeof(line1), "Y:%.1f", yaw);
            snprintf(line3, sizeof(line3), "s1:%.0f s2:%.0f", speed_1, speed_2);
            OLED_ShowString(0, 0, (u8*)line0, 16);
            OLED_ShowString(0, 16, (u8*)line1, 16);
            OLED_ShowString(0, 32, (u8*)line2, 16);
            OLED_ShowString(0, 48, (u8*)line3, 16);
            OLED_Refresh();
        // }

        delay_ms(100);

        if(status==0){
            stay_idle();
        }
        else if(status==1){
            if(start_flag==0){
                stay_idle();
                cnt = 0;
            }
            else if(start_flag==1)
            {        
            motor_set_direction(1, 1);
            motor_set_direction(2, 1);
            target_speed_1=800;
            target_speed_2=800;
            OLED_ShowStatusAndSpeeds(status, speed_1, speed_2);
            OLED_Refresh();
            // 1ms 发送一次电机实际速度
            snprintf(yaw_buf, sizeof(yaw_buf), "speed_1:%d,speed_2:%d,cnt:%d\r\n", (int)speed_1, (int)speed_2,cnt);
            UART_send_string(Yaw_INST, yaw_buf);
            cnt++;
            }
        }
        else if(status==2){
            if(start_flag==0){
                stay_idle();
                cnt = 0;
            }
            else if(start_flag==1)
            {
            track_line(); 
            OLED_ShowStatusAndSpeeds(status, speed_1, speed_2);
            OLED_Refresh();
            snprintf(yaw_buf, sizeof(yaw_buf), "speed_1:%d,speed_2:%d,cnt:%d\r\n", (int)speed_1, (int)speed_2,cnt);
            UART_send_string(Yaw_INST, yaw_buf);
            cnt++;
            }
        }
        



        //     // OLED_ShowStatusAndSpeeds(status, speed_1, speed_2);
        //     // DL_GPIO_setPins(LED_GRP_0_PORT, LED_GRP_0_LED_1_PIN);
        //     // delay_cycles(500);
        //     // DL_GPIO_clearPins(LED_GRP_0_PORT, LED_GRP_0_LED_1_PIN);
        //     // delay_cycles(500);
        // }
        // else if(status==1){
        //     motor_set_direction(1, 1);
        //     motor_set_direction(2, 1);
        //     target_speed_1=800;
        //     target_speed_2=800;
        //     OLED_ShowStatusAndSpeeds(status, speed_1, speed_2);
        // }
    

        // OLED_Refresh();

        // DL_GPIO_setPins(LED_GRP_0_PORT, LED_GRP_0_LED_1_PIN);
        // delay_ms(500);
        // DL_GPIO_clearPins(LED_GRP_0_PORT, LED_GRP_0_LED_1_PIN);
        // delay_ms(500);
        // motor_set_direction(1,1);
        // motor_set_duty(1,2000);
        // motor_set_direction(2,1);
        // motor_set_duty(2,2000);
        
    }
}
