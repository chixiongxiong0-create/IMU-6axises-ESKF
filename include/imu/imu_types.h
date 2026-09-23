#ifndef IMU_TYPES_H
#define IMU_TYPES_H

#include <stdbool.h>
#include <stdint.h>

/* Body: forward/right/down. Navigation: north/east/down. q_nb maps body to NED. */
#define IMU_FRAME_FRD_NED 1
#define IMU_AXIS_COUNT 3U
#define IMU_ERROR_STATE_DIM 6U
#define IMU_QUALITY_WINDOW_MAX 128U
#define IMU_NOTCH_MAX 4U
#define IMU_TEMP_POLY_COEFFICIENTS 3U
#define IMU_CALIBRATION_VERSION 1U

typedef struct { float x, y, z; } imu_vec3f_t;
/* Scalar-first Hamilton quaternion. */
typedef struct { float w, x, y, z; } imu_quatf_t;

typedef enum {
    IMU_STATUS_OK = 0,
    IMU_STATUS_NOT_INITIALIZED,
    IMU_STATUS_BAD_ARGUMENT,
    IMU_STATUS_NONFINITE,
    IMU_STATUS_TIME_REVERSED,
    IMU_STATUS_TIME_GAP,
    IMU_STATUS_SENSOR_DEGRADED,
    IMU_STATUS_NUMERICAL_FAULT,
    IMU_STATUS_AIDING_UNAVAILABLE
} imu_status_t;

/* DATA_VALID is positive; all remaining bits indicate faults. Healthy data
 * requires DATA_VALID set AND FAULT_MASK clear, independently of freshness. */
#define IMU_SENSOR_DATA_VALID      (UINT32_C(1) << 0)
#define IMU_SENSOR_PARITY_ERROR    (UINT32_C(1) << 1)
#define IMU_SENSOR_SEQUENCE_ERROR  (UINT32_C(1) << 2)
#define IMU_SENSOR_PLL_ERROR       (UINT32_C(1) << 3)
#define IMU_SENSOR_POWER_ERROR     (UINT32_C(1) << 4)
#define IMU_SENSOR_SELF_TEST_ERROR (UINT32_C(1) << 5)
#define IMU_SENSOR_FIFO_ERROR      (UINT32_C(1) << 6)
#define IMU_SENSOR_OVERRANGE       (UINT32_C(1) << 7)
#define IMU_SENSOR_FAULT_MASK      UINT32_C(0xFE)

typedef struct {
    /* Common monotonic frame epoch in ns; never wall-clock time. */
    uint64_t timestamp_ns;
    float gyro_rad_s[3];
    float gyro_temperature_c[3];
    uint32_t gyro_status[3];
    /* Specific force: a stationary, level FRD sensor reports [0, 0, -g]. */
    float accel_m_s2[3];
    float accel_temperature_c;
    uint32_t accel_status;
    /* False means this value MUST NOT be used as an independent new sample. */
    bool gyro_sample_fresh[3];
    bool accel_sample_fresh;
} imu_sample_t;

typedef struct {
    uint64_t timestamp_ns;
    /* Calibrated body-frame magnetic field, microtesla. */
    imu_vec3f_t field_ut;
    uint32_t status;
    bool sample_fresh;
} imu_mag_sample_t;

typedef struct {
    bool enabled;
    float frequency_hz;
    float q;
} imu_notch_config_t;

typedef enum {
    IMU_INIT_STATIONARY = 0,
    IMU_INIT_QUATERNION
} imu_init_mode_t;

/* Fixed storage only. A zero-initialized configuration is NOT a valid setup;
 * lifecycle validation and defaults are supplied by the estimator API. */
typedef struct {
    float gravity_m_s2;
    float nominal_dt_s;
    float min_dt_s;
    float max_dt_s;
    float gyro_delay_s[3];
    float accel_delay_s;
    float max_delay_compensation_s;

    /* Continuous-time densities: rad/s/sqrt(Hz), rad/s^2/sqrt(Hz). */
    float gyro_noise_density;
    float gyro_bias_rw;
    /* Direction variance (dimensionless) and zero-rate variance (rad/s)^2. */
    float accel_direction_noise_variance;
    float zero_rate_noise_variance;
    float gravity_nis_limit;
    float max_bias_step_rad_s;

    uint32_t quality_window_samples;
    /* Soft gate: full confidence below soft, zero at/above hard. */
    float accel_norm_soft_m_s2;
    float accel_norm_hard_m_s2;
    float vibration_soft_m_s2;
    float vibration_hard_m_s2;
    float jerk_soft_m_s3;
    float jerk_hard_m_s3;
    float innovation_soft_rad;
    float innovation_hard_rad;
    float innovation_persistence_s;
    float accel_weight_min;
    float weight_rise_per_s;
    float weight_fall_per_s;

    float static_gyro_max_rad_s;
    float static_gyro_variance_max;
    float static_accel_norm_max_m_s2;
    float static_accel_variance_max;
    float static_jerk_max_m_s3;
    float static_vibration_max_m_s2;
    uint32_t static_enter_samples;
    uint32_t static_exit_samples;
    float zero_rate_update_interval_s;

    uint32_t gyro_notch_count;
    uint32_t accel_notch_count;
    imu_notch_config_t gyro_notches[IMU_NOTCH_MAX];
    imu_notch_config_t accel_notches[IMU_NOTCH_MAX];

    /* Diagonal variances: [attitude error rad^2, bias error (rad/s)^2]. */
    float initial_covariance_diag[IMU_ERROR_STATE_DIM];
    imu_init_mode_t init_mode;
    imu_quatf_t initial_q_nb;
    float initial_yaw_rad;
    uint32_t alignment_samples;
    bool allow_uncalibrated;

    /* Benchmark gains: proportional 1/s, integral 1/s^2. */
    float complementary_kp;
    float complementary_ki;
} imu_config_t;

/* Row-major matrices act on column vectors. Operation order for each sensor:
 * body_rotation * (scale_alignment * raw - bias(T)). Gyro scalar scale and
 * cross-axis corrections are folded into gyro_scale_alignment.
 * bias(T) = c[0] + c[1]*(T-Tref) + c[2]*(T-Tref)^2, per axis. */
typedef struct {
    uint32_t version;
    bool valid;
    uint32_t checksum;
    /* Storage layer validates its serialized record checksum, not C padding. */
    bool checksum_valid;
    float temperature_min_c;
    float temperature_max_c;
    float temperature_reference_c;
    bool allow_temperature_extrapolation;
    float gyro_scale_alignment[3][3];
    float accel_scale_alignment[3][3];
    float body_rotation[3][3];
    /* Bias units: rad/s and m/s^2; higher coefficients include /degC^n. */
    float gyro_bias_poly[3][IMU_TEMP_POLY_COEFFICIENTS];
    float accel_bias_poly[3][IMU_TEMP_POLY_COEFFICIENTS];
} imu_calibration_t;

typedef enum {
    IMU_FUSION_INVALID = 0,
    IMU_FUSION_ALIGNING,
    IMU_FUSION_GYRO_ONLY,
    IMU_FUSION_GRAVITY,
    IMU_FUSION_STATIC
} imu_fusion_mode_t;

#define IMU_HEALTH_GYRO_DEGRADED          (UINT32_C(1) << 0)
#define IMU_HEALTH_ACCEL_DEGRADED         (UINT32_C(1) << 1)
#define IMU_HEALTH_TIMING_DEGRADED        (UINT32_C(1) << 2)
#define IMU_HEALTH_UNCALIBRATED           (UINT32_C(1) << 3)
#define IMU_HEALTH_NUMERICAL_FAULT        (UINT32_C(1) << 4)
#define IMU_HEALTH_HEADING_UNOBSERVABLE   (UINT32_C(1) << 5)
#define IMU_HEALTH_TEMPERATURE_EXTRAPOLATED (UINT32_C(1) << 6)
#define IMU_HEALTH_UNCERTAINTY_UNAVAILABLE (UINT32_C(1) << 7)

typedef struct {
    uint64_t timestamp_ns;
    imu_quatf_t q_nb;
    float roll_rad;
    float pitch_rad;
    /* Relative yaw; no absolute heading is claimed without external aiding. */
    float yaw_rad;
    imu_vec3f_t angular_rate_rad_s;
    imu_vec3f_t gyro_bias_rad_s;
    float accel_weight;
    /* One-sigma attitude uncertainty, radians, ordered roll/pitch/yaw.
     * If IMU_HEALTH_UNCERTAINTY_UNAVAILABLE is set, all three values are
     * finite FLT_MAX sentinels and are not estimated standard deviations. */
    imu_vec3f_t attitude_std_rad;
    imu_fusion_mode_t fusion_mode;
    imu_status_t status;
    imu_status_t timing_status;
    bool calibration_valid;
    bool initialized;
    uint32_t health_flags;
    uint32_t gyro_status[3];
    uint32_t accel_status;
} imu_output_t;

#endif
