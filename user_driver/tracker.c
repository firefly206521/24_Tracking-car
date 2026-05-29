#include "tracker.h"

uint8_t tracker_value[]={0,0,0,0,0,0,0};

uint8_t get_gpio_value(GPIO_Regs* gpio_port, uint32_t gpio)
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







