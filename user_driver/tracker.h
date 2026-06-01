#ifndef TRACKER_H
#define TRACKER_H

#include "ti_msp_dl_config.h"

#define TRACK_NUM_LEVELS    3
#define TRACK_LOCK_CYCLES   10   // L1→L2 锁线确认周期数 (500ms @ 20Hz)

typedef enum {
    TRACK_DIR_CW  = 1,
    TRACK_DIR_CCW = -1
} TrackDirection;

typedef struct {
    float    Kp, Ki, Kd;
    uint16_t base_speed;
    uint16_t hold_cycles;       // 0=需传感器锁线确认, >0=定时升级
} TrackLevelConfig;

typedef struct {
    float          error;
    float          last_error;
    float          integral;
    uint8_t        level;       // 0=L1, 1=L2, 2=L3
    uint16_t       lock_cnt;    // L1 锁线连续计数
    uint16_t       level_timer; // 当前级已运行周期
    TrackDirection direction;
} TrackState;

extern uint8_t tracker_value[];
extern volatile uint8_t tracking_active;
extern TrackLevelConfig track_levels[TRACK_NUM_LEVELS];
extern TrackState track_state;

void tracker_get_value(void);
void track_line(void);
void track_set_direction(TrackDirection dir);
void track_reset_level(void);

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
