#ifndef MOTOR_H
#define MOTOR_H

#include "ti_msp_dl_config.h"
//接线注释：TB6612 MSPM0G35073507
//PB24<-->STBY
//PA8<-->AIN1
//PA9<-->AIN2
//GND都要接上
//3V3 <-->VCC
//VM 7.4V
//直流电机：
//M+<-->A01
//M-<-->A02
//编码器：
//1A<-->PA21
//1B<-->PA22
//VCC<-->3V3
//GND<-->GND

#define PI 3.141
#define encoder_k 260//编码器线数
#define tire_R 48//mm
void motor_set_direction(uint8_t motor_id,uint8_t direction);
void motor_init(uint8_t motor_id);
void motor_set_duty(uint8_t motor_id,uint32_t duty);
void MOTOR_PID(uint8_t motor_id,float target_speed);

#endif /* MOTOR_H */