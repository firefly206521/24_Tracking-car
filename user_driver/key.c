#include "key.h"
#include "oled.h"
extern int status;
extern int encoder_motor1;
extern int encoder_motor2;
uint8_t get_key_value(uint32_t key)
{
uint8_t high_bits = DL_GPIO_readPins(KEY_PORT, key);
if((high_bits & key)!=0){
    return 1;
}
else return 0;
}

//GPIO的中断是很多GPIO共享的，所以具体哪个GPIO触发了中断需要在一个函数里判断
void GROUP1_IRQHandler(void)
{
    DL_GPIO_setPins(LED_GRP_0_PORT, LED_GRP_0_LED_1_PIN);
    switch (DL_GPIO_getPendingInterrupt(GPIOB))
    {
    case KEY_KEY_1_IIDX:
        status=(status+1)%2;
        break;
    case KEY_KEY_4_IIDX:
        break;
    case MOTOR_EC2A_IIDX:
        encoder_motor2++;
        break;
    default:
        break;
        OLED_Init();
        OLED_ColorTurn(0);//0正常显示，1反色显示
        OLED_DisplayTurn(0);//0正常显示，1屏幕旋转180度
        OLED_Clear();
    }

    //电机的上升沿触发中断也在这里写
    //并且我们观察头文件可以得知，只有种GPIO：GPIOA和GPIOB。所以我们只能写两个switch
switch (DL_GPIO_getPendingInterrupt(GPIOA))
    {
        case MOTOR_EC1A_IIDX:
            encoder_motor1++;
            DL_GPIO_setPins(LED_GRP_0_PORT, LED_GRP_0_LED_1_PIN);
            break;

        default:
            break;

    }

}    