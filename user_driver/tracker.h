#ifndef TRACKER_H
#define TRACKER_H

#include "ti_msp_dl_config.h"

typedef struct {
    float error;
    float last_error;
    float integral;
    float pid_output;
} tracker_pid_t;

extern uint8_t tracker_value[];
extern volatile uint8_t tracking_active;
extern float Kp, Ki, Kd;
extern tracker_pid_t pid_line;   // 直线循迹 PID 状态
extern tracker_pid_t pid_arc;    // 圆弧循迹 PID 状态（CB/DA 共享）

void tracker_get_value();
void track_line();
void tracker_pid(float base_speed, tracker_pid_t *pid);
//VCC<-->5V
//GND
//L1<-->PB8
//L2<-->PA17
//MID<-->PB9
//R1<-->PA2
//R2<-->PA24
//RO<-->PA25
//L0<-->PA15
#endif /* TRACKER_H */