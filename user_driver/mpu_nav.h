#ifndef MPU_NAV_H
#define MPU_NAV_H

#include "ti_msp_dl_config.h"

float   mpu_apply_calib(float raw_yaw);
void    mpu_reset_zero(float raw_yaw);
uint8_t mpu_calib_done(void);

extern volatile float g_raw_yaw;

#endif
