#ifndef TRACKER_H
#define TRACKER_H

#include "ti_msp_dl_config.h"

extern uint8_t tracker_value[];
extern volatile uint8_t tracking_active;

void tracker_get_value();
void track_line();
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