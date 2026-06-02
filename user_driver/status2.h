#ifndef STATUS2_H
#define STATUS2_H

#include <stdint.h>

// 状态机阶段
#define S2_PHASE_STRAIGHT1  0   // 第一阶段直行（初始方向）
#define S2_PHASE_CURVE1     1   // 第一个弯道（循迹）
#define S2_PHASE_STRAIGHT2  2   // 第二阶段直行（初始+180°）
#define S2_PHASE_CURVE2     3   // 第二个弯道（循迹）
#define S2_PHASE_DONE       4   // 完成，停车

void status2_run(float yaw, int start_flag);
void status2_reset(void);

#endif
