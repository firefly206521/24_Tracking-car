#include "mpu_nav.h"
#include "pid_utils.h"

#define CALIB_SAMPLES 50

static float    mpu_zero     = 0.0f;
static float    calib_sum    = 0.0f;
static uint16_t calib_cnt    = 0;
static uint8_t  calib_done   = 0;

float mpu_apply_calib(float raw_yaw)
{
    if (!calib_done) {
        calib_sum += raw_yaw;
        calib_cnt++;
        if (calib_cnt >= CALIB_SAMPLES) {
            mpu_zero   = calib_sum / (float)CALIB_SAMPLES;
            calib_done = 1;
        }
    }
    return normalize_angle(raw_yaw - mpu_zero);
}

void mpu_reset_zero(float raw_yaw)
{
    mpu_zero   = raw_yaw;
    calib_done = 1;
}

uint8_t mpu_calib_done(void)
{
    return calib_done;
}

volatile float g_raw_yaw = 0;
