#ifndef IMU_ESEKF_H
#define IMU_ESEKF_H

#include "imu_types.h"

/* Right multiplicative body-frame error: q_true = q_nb * Exp(delta_theta).
 * Exp denotes the unit quaternion of the rotation vector (half-angle inside
 * the quaternion exponential). P order is [body angle, body gyro bias]. */
typedef struct {
    imu_quatf_t q_nb;
    imu_vec3f_t gyro_bias_rad_s;
    float P[IMU_ERROR_STATE_DIM][IMU_ERROR_STATE_DIM];
    imu_vec3f_t angular_rate_rad_s;
    bool initialized;
} imu_esekf_t;

/* All operations are transactional: any non-OK result leaves state/output
 * byte-identical. Configuration is supplied by the caller, not retained.
 * Each operation validates only the configuration fields that it consumes.
 * Initialization normalizes q and uses initial_covariance_diag. */
imu_status_t imu_esekf_init(imu_esekf_t *filter, const imu_config_t *config,
                          imu_quatf_t q_nb, imu_vec3f_t gyro_bias_rad_s);
imu_status_t imu_esekf_predict(imu_esekf_t *filter, const imu_config_t *config,
                             imu_vec3f_t gyro_calibrated_rad_s, float dt_s);
/* Caller supplies a fresh, calibrated, qualified acceleration. weight in [0,1]
 * scales R = accel_direction_noise_variance/weight; zero is a bitwise no-op.
 * Quality threshold/freshness/status gating belongs to the lifecycle caller.
 * Cross residual z cross h, H = [I-hh',0], h = R(q_nb)'*[0,0,-1].
 * Hemisphere and NIS rejection return AIDING_UNAVAILABLE. A poorly conditioned
 * innovation returns NUMERICAL_FAULT. Both leave the filter unchanged. */
imu_status_t imu_esekf_update_gravity(imu_esekf_t *filter,
                                    const imu_config_t *config,
                                    imu_vec3f_t specific_force_m_s2,
                                    float weight);
/* Caller qualifies stationarity, freshness, dwell, and update cadence. All
 * three statuses must set DATA_VALID with FAULT_MASK clear. H=[0,I] directly
 * observes bias. Both observation APIs bound the norm of the bias increment
 * by max_bias_step_rad_s, scaling the whole gain before Joseph covariance.
 * Injection uses q <- q*Exp(dtheta), followed by covariance reset
 * G=diag(I-.5*skew(dtheta), I). No absolute heading is observed. */
imu_status_t imu_esekf_update_zero_rate(imu_esekf_t *filter,
                                      const imu_config_t *config,
                                      imu_vec3f_t gyro_calibrated_rad_s,
                                      const uint32_t gyro_status[3]);
/* Read-only finite/symmetry/positive-diagonal and scale-aware PSD check.
 * Tolerates only float roundoff: at most 32*FLT_EPSILON diagonal loading in
 * correlation coordinates. Predictions and measurement updates regularize
 * near-zero pivots within that budget before committing; material negative
 * modes return NUMERICAL_FAULT transactionally. */
bool imu_esekf_covariance_valid(const imu_esekf_t *filter);
/* Populates attitude, bias, last corrected rate and Euler-angle uncertainty.
 * Returns NUMERICAL_FAULT at the Euler pitch singularity (|cos(pitch)|<1e-5).
 * Timestamp, quality/mode, calibration, and sensor diagnostics are lifecycle
 * caller fields: initially zero/GYRO_ONLY here. Heading-unobservable is set. */
imu_status_t imu_esekf_get_output(const imu_esekf_t *filter, imu_output_t *out);

#endif
