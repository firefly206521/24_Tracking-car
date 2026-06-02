#include "key.h"
#include "oled.h"
#include "motor.h"
#include "tracker.h"
#include "status.h"

extern volatile uint32_t sys_tick_ms;

uint8_t get_key_value(uint32_t key)
{
uint8_t high_bits = DL_GPIO_readPins(KEY_PORT, key);
if((high_bits & key)!=0){
    return 1;
}
else return 0;
}

//按键消抖时间窗口 (ms)
#define KEY_DEBOUNCE_MS 50

//GPIO的中断是很多GPIO共享的，所以具体哪个GPIO触发了中断需要在一个函数里判断
void GROUP1_IRQHandler(void)
{
    static uint32_t last_key1_ms = 0;
    static uint32_t last_key4_ms = 0;

    DL_GPIO_setPins(LED_GRP_0_PORT, LED_GRP_0_LED_1_PIN);
    switch (DL_GPIO_getPendingInterrupt(GPIOB))
    {
    case KEY_KEY_1_IIDX:
        if (sys_tick_ms - last_key1_ms >= KEY_DEBOUNCE_MS) {
            last_key1_ms = sys_tick_ms;
            status_cycle_next();
        }
        break;
    case KEY_KEY_4_IIDX:
        if (sys_tick_ms - last_key4_ms >= KEY_DEBOUNCE_MS) {
            last_key4_ms = sys_tick_ms;
            status_toggle_start();
        }
        break;
    case MOTOR_EC2A_IIDX:
        encoder_motor2++;
        break;
    default:
        break;
    }

    //电机的上升沿触发中断也在这里写
    //并且我们观察头文件可以得知，只有种GPIO：GPIOA和GPIOB。所以我们只能写两个switch
switch (DL_GPIO_getPendingInterrupt(GPIOA))
    {
        case MOTOR_EC1A_IIDX:
            encoder_motor1++;
            break;

        default:
            break;

    }

}    