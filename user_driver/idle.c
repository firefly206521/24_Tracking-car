#include "motor.h"
#include "oled.h"
extern int encoder_motor1;
extern int encoder_motor2;
extern int target_speed_1;
extern int target_speed_2;
extern int status;
extern int speed_1;
extern int speed_2;
void stay_idle(void)
{   
encoder_motor1=0;
encoder_motor2=0;
motor_set_direction(1,0);
motor_set_direction(2,0);
target_speed_1=0;
target_speed_2=0;
OLED_ShowStatusAndSpeeds(status, speed_1, speed_2);
OLED_Refresh();
}
