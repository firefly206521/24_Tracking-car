#ifndef MPU_NAV_H
#define MPU_NAV_H

float mpu_normalize(float angle);
void  mpu_set_ref(float yaw);
float mpu_get_ref(void);
float mpu_heading_error(float yaw);
void  mpu_arc_begin(float yaw);
float mpu_arc_progress(float yaw);
float mpu_clamp_speed(float s);

#endif
