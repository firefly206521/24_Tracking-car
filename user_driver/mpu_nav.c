#include "mpu_nav.h"

#define CALIB_SAMPLES 50

static float    mpu_zero     = 0.0f;
static float    calib_sum    = 0.0f;
static uint16_t calib_cnt    = 0;
static uint8_t  calib_done   = 0;

float mpu_normalize(float a)
{
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

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
    return mpu_normalize(raw_yaw - mpu_zero);
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

float mpu_clamp_speed(float s)
{
    if (s > 600.0f) return 600.0f;
    if (s < 0.0f)   return 0.0f;
    return s;
}
