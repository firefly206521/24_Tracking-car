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
void    straight_force_stop(void);

// 主循环直行（不设 straight_active，无退出条件，给 status2 用）
void    straight_nav_begin(float yaw);
void    straight_nav_run(float yaw);

#endif
