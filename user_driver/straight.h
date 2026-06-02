#ifndef STRAIGHT_H
#define STRAIGHT_H

#include <stdint.h>

#define STRAIGHT_DIST_MAX   1750
#define STRAIGHT_BASE_SPEED 800.0f

// 不清零的编码器脉冲累加值（每 50ms 在 speed_calculate 中累加）
extern volatile int32_t straight_enc_acc;
// 全局 yaw，主循环更新，ISR 读取
extern volatile float g_yaw;

void    straight_begin(float yaw);
uint8_t straight_run(float yaw);
uint8_t straight_is_active(void);
int32_t straight_get_distance(void);
uint8_t straight_line_detected(void);

// STATUS_MPU_NAV 专用：不走 active 标志，无退出条件
void    straight_nav_begin(float yaw);
void    straight_nav_run(float yaw);
void    straight_nav_resume(float yaw);    // 换目标角度，不重置 PID 积分
void    straight_force_stop(void);          // 强制退出 straight（清除 active 标志）

#endif
