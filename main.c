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
int status=0;
int encoder_motor1=0;
int encoder_motor2=0;
float target_speed_1=0;
float target_speed_2=0;
extern int16_t PWM_1_duty;
extern volatile float speed_1;
extern volatile float speed_2;
extern uint8_t tracker_value[];
int main(void)
{
    SYSCFG_DL_init();

    //OLED初始化
    OLED_Init();
    OLED_ColorTurn(0);//0正常显示，1反色显示
    OLED_DisplayTurn(0);//0正常显示，1屏幕旋转180度
    OLED_Clear();

    //电机初始化
    motor_init(1);
    motor_init(2);

    //给电机和案件引脚设置为开启中断s功能
    NVIC_EnableIRQ(MOTOR_EC1A_IIDX);
    NVIC_EnableIRQ(MOTOR_EC2A_IIDX);
    NVIC_EnableIRQ(KEY_KEY_1_IIDX);
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    NVIC_EnableIRQ(MOTOR_GPIOA_INT_IRQN);
    motor_set_direction(1, 1);
    motor_set_direction(2, 1);





    while (1) {
        tracker_get_value();
        char tracker_buf[] = "0000000\n";
        sprintf(tracker_buf, "%d%d%d%d%d%d%d\n", 
        tracker_value[0], tracker_value[1], tracker_value[2], 
        tracker_value[3], tracker_value[4], tracker_value[5], 
        tracker_value[6]);
        OLED_ShowString(0, 0, (u8*)tracker_buf,16);
        if(status==0){
            encoder_motor1=0;
            encoder_motor2=0;
            motor_set_direction(1,0);
            motor_set_direction(2,0);
            target_speed_1=0;
            target_speed_2=0;
            OLED_ShowStatusAndSpeeds(status, speed_1, speed_2);
        }
        else if(status==1){
            motor_set_direction(1, 1);
            motor_set_direction(2, 1);
            target_speed_1=800;
            target_speed_2=800;
            OLED_ShowStatusAndSpeeds(status, speed_1, speed_2);
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
