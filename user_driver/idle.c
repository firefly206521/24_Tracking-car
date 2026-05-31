#include "motor.h"
#include "oled.h"
#include "key.h"

void stay_idle(void)
{   
encoder_motor1=0;
encoder_motor2=0;
motor_set_direction(MOTOR_LEFT,0);
motor_set_direction(MOTOR_RIGHT,0);
target_speed_1=0;
target_speed_2=0;
OLED_ShowStatusAndSpeeds(status, speed_1, speed_2);
OLED_Refresh();
}
