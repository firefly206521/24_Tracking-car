#include "pid_utils.h"

float pid_compute(pid_ctrl_t *pid, float error)
{
    pid->error = error;
    pid->integral += error;
    if (pid->integral >  pid->integral_max) pid->integral =  pid->integral_max;
    if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
    float derivative = error - pid->last_error;
    pid->output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    pid->last_error = error;
    return pid->output;
}

float normalize_angle(float a)
{
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

float clamp_value(float val, float min, float max)
{
    if (val > max) return max;
    if (val < min) return min;
    return val;
}
