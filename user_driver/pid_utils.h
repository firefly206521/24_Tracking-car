#ifndef PID_UTILS_H
#define PID_UTILS_H

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float error;
    float last_error;
    float integral;
    float output;
    float integral_max;
} pid_ctrl_t;

float pid_compute(pid_ctrl_t *pid, float error);
void   pid_reset(pid_ctrl_t *pid);
float normalize_angle(float a);
float clamp_value(float val, float min, float max);

#endif
