#ifndef STATUS_H
#define STATUS_H

#include <stdint.h>

typedef enum {
    STATUS_IDLE        = 0,
    STATUS_DIST        = 1,   // 直行 1750 脉冲后停车
    STATUS_MPU_NAV     = 2,   // 陀螺仪导航
    STATUS_LINE_TRACK  = 3,   // 循迹模式
    STATUS_COUNT
} system_status_t;

extern system_status_t sys_status;
extern int              start_flag;

void status_cycle_next(void);
void status_toggle_start(void);
void status_run(float yaw);

#endif
