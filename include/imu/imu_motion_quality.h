#ifndef IMU_MOTION_QUALITY_H
#define IMU_MOTION_QUALITY_H

#include "imu_types.h"

typedef enum {
    IMU_MOTION_REASON_NONE = 0,
    IMU_MOTION_REASON_ACCEL_STATUS = (UINT32_C(1) << 0),
    IMU_MOTION_REASON_GYRO_STATUS = (UINT32_C(1) << 1),
    IMU_MOTION_REASON_ACCEL_NORM = (UINT32_C(1) << 2),
    IMU_MOTION_REASON_VIBRATION = (UINT32_C(1) << 3),
    IMU_MOTION_REASON_JERK = (UINT32_C(1) << 4),
    IMU_MOTION_REASON_INNOVATION = (UINT32_C(1) << 5),
    IMU_MOTION_REASON_GYRO_MOTION = (UINT32_C(1) << 6),
    IMU_MOTION_REASON_GYRO_VARIANCE = (UINT32_C(1) << 7),
    IMU_MOTION_REASON_ACCEL_VARIANCE = (UINT32_C(1) << 8)
} imu_motion_reason_t;

typedef struct {
    float accel_weight;
    float accel_norm_residual;
    float vibration_rms;
    float jerk;
    float innovation_angle_rad;
    float innovation_persistence_s;
    bool static_detected;
    uint32_t mode_reasons;

    uint32_t static_enter_count;
    uint32_t static_exit_count;
    uint32_t window_count;
    uint32_t window_index;
    uint32_t window_length;
    uint64_t last_timestamp_ns;
    bool has_previous_sample;
    imu_vec3f_t previous_accel_m_s2;
    imu_vec3f_t accel_window[IMU_QUALITY_WINDOW_MAX];
    imu_vec3f_t gyro_window[IMU_QUALITY_WINDOW_MAX];
    bool initialized;
} imu_motion_quality_t;

/* Initializes fixed storage. Invalid configuration leaves *quality unchanged. */
bool imu_motion_quality_init(imu_motion_quality_t *quality,
                             const imu_config_t *config);

/* Updates quality from one calibrated SI-unit sample. The innovation angle is
 * the measured-specific-force versus predicted-gravity disagreement. Invalid
 * input, configuration, time, or arithmetic leaves *quality unchanged. */
bool imu_motion_quality_update(imu_motion_quality_t *quality,
                               const imu_config_t *config,
                               const imu_sample_t *sample,
                               float innovation_angle_rad);

#endif
