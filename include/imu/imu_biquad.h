#ifndef IMU_BIQUAD_H
#define IMU_BIQUAD_H

#include "imu_types.h"

typedef struct {
    /* A zero-initialized filter is a bitwise-transparent bypass. */
    bool enabled;
    float b0, b1, b2;
    float a1, a2;
    imu_vec3f_t z1;
    imu_vec3f_t z2;
} imu_biquad_t;

bool imu_biquad_configure_notch(imu_biquad_t *filter, float sample_rate_hz,
                                float frequency_hz, float q);
/* Reset delay state without changing the current coefficients or enable bit. */
void imu_biquad_reset(imu_biquad_t *filter);
bool imu_biquad_process(imu_biquad_t *filter, imu_vec3f_t input,
                        imu_vec3f_t *out);

#endif
