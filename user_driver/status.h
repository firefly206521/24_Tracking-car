#ifndef STATUS_H
#define STATUS_H

#include <stdint.h>

typedef enum {
    STATUS_IDLE       = 0,
    STATUS_TRACKING   = 1,   // 循迹模式 (原 status 2)
    STATUS_MPU_NAV    = 2,   // 陀螺仪导航模式 (原 status 3)
    STATUS_COUNT
} system_status_t;

extern system_status_t sys_status;
extern int              start_flag;

void status_cycle_next(void);
void status_toggle_start(void);
void status_run(float yaw);

#endif
