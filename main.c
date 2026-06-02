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
#include "status.h"
#include "straight.h"
#include "mpu_nav.h"

extern volatile uint32_t sys_tick_ms;

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
    while(DMP_Init());
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

        while(DMP_Read_Data(&pitch,&roll,&yaw));
        g_raw_yaw = yaw;
        yaw       = mpu_apply_calib(yaw);
        g_yaw     = yaw;

        // snprintf(yaw_buf, sizeof(yaw_buf), "Yaw: %.2f cnt:%d\r\n", yaw,cnt);
        // UART_send_string(Yaw_INST, yaw_buf);
        // cnt++;
        
        snprintf(line3, sizeof(line3), "Y:%.1f", yaw);
        OLED_ShowString(0, 48, (u8*)line3, 16);

        status_run(yaw);

    }
}
