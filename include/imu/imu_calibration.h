#ifndef IMU_CALIBRATION_H
#define IMU_CALIBRATION_H

#include "imu_types.h"

typedef struct {
    uint64_t timestamp_ns;
    imu_vec3f_t gyro_rad_s;
    imu_vec3f_t accel_m_s2;
    float gyro_temperature_c[3];
    float accel_temperature_c;
} imu_calibrated_sample_t;

/* Applies body_rotation * (scale_alignment * raw - bias(T)). The calibration
 * record, board rotation, input sample, temperature policy, and every computed
 * value must be valid and finite. On failure, *out is unchanged. */
bool imu_calibrate_sample(const imu_calibration_t *calibration,
                          const imu_sample_t *input,
                          imu_calibrated_sample_t *out);

#endif
