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

extern volatile uint32_t sys_tick_ms;

// ====== 模式3 状态机 ======
typedef enum {
    M3_IDLE = 0,
    M3_AC_DIAG,
    M3_CB_ARC,
    M3_BD_DIAG,
    M3_DA_ARC,
    M3_DONE
} m3_state_t;

static m3_state_t m3_state = M3_IDLE;

#define M3_KP_HEADING   2.0f
#define M3_KP_TRACKER   80.0f
#define M3_BASE_SPEED   300.0f
#define M3_ARC_DONE_DEG 170.0f

// SysTick 中断服务函数 (每 1ms 触发一次)
void SysTick_Handler(void) {
    sys_tick_ms++;
}

int main(void)
{
    int cnt = 0;
    
    SYSCFG_DL_init();
    delay_ms(1000);//等待电源稳定，保证IIC上电成功
    //OLED初始化
    OLED_Init();
    OLED_ColorTurn(0);//0正常显示，1反色显示
    OLED_DisplayTurn(0);//0正常显示，1屏幕旋转180度
    OLED_Clear();

    //电机初始化
    motor_init(MOTOR_LEFT);
    motor_init(MOTOR_RIGHT);

    //陀螺仪初始化
    //while(DMP_Init());
    float pitch = 0, roll = 0, yaw = 0;


    char yaw_buf[50];
    char line0[20], line1[20], line2[20], line3[20];

    //给电机和案件引脚设置为开启中断功能
    NVIC_EnableIRQ(MOTOR_EC1A_IIDX);
    NVIC_EnableIRQ(MOTOR_EC2A_IIDX);
    NVIC_EnableIRQ(KEY_KEY_1_IIDX);
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    NVIC_EnableIRQ(MOTOR_GPIOA_INT_IRQN);
    motor_set_direction(MOTOR_RIGHT, 0);
    motor_set_direction(MOTOR_LEFT, 0);

    while (1) {

        tracker_get_value();
        //while(DMP_Read_Data(&pitch,&roll,&yaw));

        snprintf(line0, sizeof(line0), "S:%d", status);
        snprintf(line1, sizeof(line1), "Y:%.1f P:%.1f", yaw, pitch);
        snprintf(line3, sizeof(line3), "s1:%.0f s2:%.0f", speed_1, speed_2);
        OLED_ShowString(0, 48, (u8*)line3, 16);
        OLED_Refresh();

        if(status==0){
            stay_idle();
        }
        else if(status==1){
            if(start_flag==0){
                stay_idle();
            }
            else if(start_flag==1)
            {        
            motor_set_direction(MOTOR_RIGHT, 1);
            motor_set_direction(MOTOR_LEFT, 1);
            target_speed_1=800;
            target_speed_2=800;
            OLED_ShowStatusAndSpeeds(status, speed_1, speed_2);
            OLED_Refresh();
            }
        }
        else if(status==2){
            if(start_flag==0){
                stay_idle();
            }
            else if(start_flag==1)
            {
            track_line(); 
            OLED_ShowStatusAndSpeeds(status, speed_1, speed_2);
            OLED_Refresh();
            }
        }
        else if (status == 3) {
            switch (m3_state) {
            case M3_IDLE:
                mpu_set_ref(yaw);
                motor_set_direction(MOTOR_RIGHT, 1);
                motor_set_direction(MOTOR_LEFT, 1);
                target_speed_1 = M3_BASE_SPEED;
                target_speed_2 = M3_BASE_SPEED;
                m3_state = M3_AC_DIAG;
                break;
            case M3_AC_DIAG: {
                float err  = mpu_heading_error(yaw);
                float corr = M3_KP_HEADING * err;
                target_speed_2 = mpu_clamp_speed(M3_BASE_SPEED - corr);
                target_speed_1 = mpu_clamp_speed(M3_BASE_SPEED + corr);
                if (tracker_value[2]) {
                    mpu_arc_begin(yaw);
                    m3_state = M3_CB_ARC;
                }
                break;
            }
            case M3_CB_ARC: {
                int8_t pos = 0;
                if (tracker_value[0]) pos -= 2;
                if (tracker_value[1]) pos -= 1;
                if (tracker_value[3]) pos += 1;
                if (tracker_value[4]) pos += 2;
                float corr = M3_KP_TRACKER * pos;
                target_speed_2 = mpu_clamp_speed(M3_BASE_SPEED - corr);
                target_speed_1 = mpu_clamp_speed(M3_BASE_SPEED + corr);
                if (mpu_arc_progress(yaw) > M3_ARC_DONE_DEG) {
                    m3_state = M3_BD_DIAG;
                }
                break;
            }
            case M3_BD_DIAG: {
                float target = mpu_normalize(mpu_get_ref() + 180.0f);
                float err    = mpu_normalize(yaw - target);
                float corr   = M3_KP_HEADING * err;
                target_speed_2 = mpu_clamp_speed(M3_BASE_SPEED - corr);
                target_speed_1 = mpu_clamp_speed(M3_BASE_SPEED + corr);
                if (tracker_value[2]) {
                    mpu_arc_begin(yaw);
                    m3_state = M3_DA_ARC;
                }
                break;
            }
            case M3_DA_ARC: {
                int8_t pos = 0;
                if (tracker_value[0]) pos -= 2;
                if (tracker_value[1]) pos -= 1;
                if (tracker_value[3]) pos += 1;
                if (tracker_value[4]) pos += 2;
                float corr = M3_KP_TRACKER * pos;
                target_speed_2 = mpu_clamp_speed(M3_BASE_SPEED - corr);
                target_speed_1 = mpu_clamp_speed(M3_BASE_SPEED + corr);
                if (mpu_arc_progress(yaw) > M3_ARC_DONE_DEG) {
                    m3_state = M3_DONE;
                }
                break;
            }
            case M3_DONE:
                target_speed_1 = 0;
                target_speed_2 = 0;
                motor_set_direction(MOTOR_RIGHT, 0);
                motor_set_direction(MOTOR_LEFT, 0);
                break;
            }
        }



        //     // OLED_ShowStatusAndSpeeds(status, speed_1, speed_2);
        //     // DL_GPIO_setPins(LED_GRP_0_PORT, LED_GRP_0_LED_1_PIN);
        //     // delay_cycles(500);
        //     // DL_GPIO_clearPins(LED_GRP_0_PORT, LED_GRP_0_LED_1_PIN);
        //     // delay_cycles(500);
        // }
        // else if(status==1){
        //     motor_set_direction(MOTOR_RIGHT, 1);
        //     motor_set_direction(MOTOR_LEFT, 1);
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
