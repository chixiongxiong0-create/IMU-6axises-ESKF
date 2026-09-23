#ifndef IMU_COMPLEMENTARY_H
#define IMU_COMPLEMENTARY_H

#include "imu_motion_quality.h"

/* Caller-owned benchmark state. q_nb maps FRD body vectors into NED. */
typedef struct {
    imu_quatf_t q_nb;
    imu_vec3f_t gyro_bias_rad_s;
    imu_vec3f_t initial_gyro_bias_rad_s;
    imu_vec3f_t angular_rate_rad_s;
    uint64_t last_timestamp_ns;
    bool has_timestamp;
    bool initialized;
} imu_complementary_t;

/* Transactional initialization. Gains and timing limits come from config.
 * The integral correction is bounded in norm by max_bias_step_rad_s relative
 * to the supplied initial bias, preventing long rejected-motion windup. */
imu_status_t imu_complementary_init(imu_complementary_t *filter,
                                    const imu_config_t *config,
                                    imu_quatf_t q_nb,
                                    imu_vec3f_t gyro_bias_rad_s);

/* The sample is already calibrated. quality is the result for this same
 * sample, obtained by the common motion-quality path. The first frame sets
 * the epoch without integrating. Nonfinite and invalid time leave all caller
 * storage unchanged. Gaps and missing gyros consume time but freeze attitude.
 * Stale/faulted acceleration receives zero correction. Yaw is relative only. */
imu_status_t imu_complementary_process(imu_complementary_t *filter,
                                       const imu_config_t *config,
                                       const imu_sample_t *sample,
                                       const imu_motion_quality_t *quality,
                                       imu_output_t *output);

#endif
