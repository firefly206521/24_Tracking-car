#include "motor.h"
#include "oled.h"
#include "status.h"
#include "tracker.h"

void stay_idle(void)
{
tracking_active = 0;
encoder_motor1=0;
encoder_motor2=0;
integral_1=0;
integral_2=0;
motor_set_direction(MOTOR_LEFT,0);
motor_set_direction(MOTOR_RIGHT,0);
target_speed_1=0;
target_speed_2=0;
OLED_ShowStatusAndSpeeds(sys_status, speed_1, speed_2);
}
