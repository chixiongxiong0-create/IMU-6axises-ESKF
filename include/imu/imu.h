#ifndef IMU_H
#define IMU_H

#include "imu_biquad.h"
#include "imu_calibration.h"
#include "imu_esekf.h"
#include "imu_motion_quality.h"

typedef enum {
    IMU_STATE_UNINITIALIZED = 0, IMU_STATE_ALIGNING, IMU_STATE_RUNNING,
    IMU_STATE_DEGRADED, IMU_STATE_FAULT
} imu_state_t;

/* Caller-owned fixed storage. Treat members as private after initialization. */
typedef struct {
    imu_config_t config;
    imu_calibration_t calibration;
    imu_esekf_t esekf;
    imu_motion_quality_t quality;
    imu_biquad_t gyro_notches[IMU_NOTCH_MAX];
    imu_biquad_t accel_notches[IMU_NOTCH_MAX];
    imu_output_t output;
    imu_state_t state;
    imu_vec3f_t alignment_accel, alignment_gyro;
    uint32_t alignment_count;
    uint64_t last_timestamp_ns, last_zero_rate_ns;
    bool configured, has_timestamp, calibration_valid;
} imu_t;

/* Conservative starting values, SI units, 485 Hz; tune against hardware logs.
 * Notches and signal delays default to zero/bypass. Set measured delays from
 * the acquisition chain; all gyro delays must be exactly equal and
 * |accel_delay - gyro_delay[0]| must be bounded by
 * max_delay_compensation_s. Compensation is a constant-rate approximation,
 * using a common gyro epoch, not clock synchronization. */
void imu_config_defaults(imu_config_t *config);
/* Transactional validation. NULL calibration is allowed only in explicit
 * diagnostic allow_uncalibrated mode. Even known-q initialization waits for a
 * healthy fresh frame before reporting initialized. Stationary startup needs
 * a static dwell followed by alignment_samples consecutive qualified frames. */
bool imu_init(imu_t *imu, const imu_config_t *config,
              const imu_calibration_t *calibration);
/* First accepted frame establishes the epoch; no nominal dt is integrated.
 * Nonfinite input, reversed/equal time, or dt below min leave both state and
 * output byte-identical (NONFINITE/TIME_REVERSED/BAD_ARGUMENT respectively).
 * Missing gyro axes or dt above max consume the reported interval once,
 * freeze attitude, grow covariance, and report SENSOR_DEGRADED/TIME_GAP.
 * The next fresh frame integrates only its own interval, never fills missing
 * motion with a reused reading. Stale acceleration never updates quality or
 * gravity. Observation rejection preserves successful propagation. A hard
 * numerical fault latches invalid output until reset/reinitialization.
 * timestamp_ns remains the input host latch/publication reference;
 * attitude_epoch_ns subtracts the common gyro delay. No prediction to the
 * host epoch is performed. If this subtraction would underflow, epoch is 0,
 * TIMING_DEGRADED is set, timing_status is BAD_ARGUMENT (TIME_GAP takes
 * precedence), and an otherwise OK frame returns SENSOR_DEGRADED. A known
 * initial quaternion must describe the first frame's physical gyro epoch.
 * Valid calls write output, including NOT_INITIALIZED during alignment.
 * Buffers must not overlap the imu object. No dynamic allocation is used. */
imu_status_t imu_process(imu_t *imu, const imu_sample_t *sample,
                         imu_output_t *output);
/* Restart alignment using the retained config/calibration (no timestamp). */
bool imu_reset(imu_t *imu);
/* Copy the last committed output without advancing any estimator state. */
imu_status_t imu_get_output(const imu_t *imu, imu_output_t *output);
/* Always a state-preserving no-op in this version. Valid finite arguments
 * return AIDING_UNAVAILABLE; malformed arguments return BAD_ARGUMENT/NONFINITE.
 * Neither freshness, timestamp nor status can enable heading aiding. */
imu_status_t imu_update_magnetometer(imu_t *imu, const imu_mag_sample_t *sample);

#endif
