#include "imu/imu_motion_quality.h"

#include <math.h>
#include <string.h>

#define NANOSECONDS_PER_SECOND UINT64_C(1000000000)
#define SECONDS_PER_NANOSECOND 1.0e-9f

static bool finite_nonnegative(float value)
{
    return isfinite(value) && value >= 0.0f;
}

static bool finite_positive(float value)
{
    return isfinite(value) && value > 0.0f;
}

static bool gate_valid(float soft, float hard)
{
    return finite_nonnegative(soft) && isfinite(hard) && hard > soft;
}

static bool config_valid(const imu_config_t *config)
{
    return config != NULL &&
           finite_positive(config->gravity_m_s2) &&
           finite_positive(config->nominal_dt_s) &&
           config->quality_window_samples > 0U &&
           config->quality_window_samples <= IMU_QUALITY_WINDOW_MAX &&
           gate_valid(config->accel_norm_soft_m_s2,
                      config->accel_norm_hard_m_s2) &&
           gate_valid(config->vibration_soft_m_s2,
                      config->vibration_hard_m_s2) &&
           gate_valid(config->jerk_soft_m_s3, config->jerk_hard_m_s3) &&
           gate_valid(config->innovation_soft_rad,
                      config->innovation_hard_rad) &&
           finite_positive(config->innovation_persistence_s) &&
           finite_nonnegative(config->accel_weight_min) &&
           config->accel_weight_min <= 1.0f &&
           finite_nonnegative(config->weight_rise_per_s) &&
           finite_nonnegative(config->weight_fall_per_s) &&
           finite_nonnegative(config->static_gyro_max_rad_s) &&
           finite_nonnegative(config->static_gyro_variance_max) &&
           finite_nonnegative(config->static_accel_norm_max_m_s2) &&
           finite_nonnegative(config->static_accel_variance_max) &&
           finite_nonnegative(config->static_jerk_max_m_s3) &&
           finite_nonnegative(config->static_vibration_max_m_s2) &&
           config->static_enter_samples > 0U &&
           config->static_exit_samples > 0U;
}

static imu_vec3f_t sample_accel(const imu_sample_t *sample)
{
    imu_vec3f_t value;

    value.x = sample->accel_m_s2[0];
    value.y = sample->accel_m_s2[1];
    value.z = sample->accel_m_s2[2];
    return value;
}

static imu_vec3f_t sample_gyro(const imu_sample_t *sample)
{
    imu_vec3f_t value;

    value.x = sample->gyro_rad_s[0];
    value.y = sample->gyro_rad_s[1];
    value.z = sample->gyro_rad_s[2];
    return value;
}

static bool vector_finite(imu_vec3f_t value)
{
    return isfinite(value.x) && isfinite(value.y) && isfinite(value.z);
}

static bool sample_floats_finite(const imu_sample_t *sample)
{
    unsigned axis;

    if (!isfinite(sample->accel_temperature_c)) {
        return false;
    }
    for (axis = 0U; axis < IMU_AXIS_COUNT; ++axis) {
        if (!isfinite(sample->gyro_rad_s[axis]) ||
            !isfinite(sample->gyro_temperature_c[axis]) ||
            !isfinite(sample->accel_m_s2[axis])) {
            return false;
        }
    }
    return true;
}

static float vector_norm(imu_vec3f_t value)
{
    return sqrtf(value.x * value.x + value.y * value.y + value.z * value.z);
}

static float smooth_quality(float value, float soft, float hard)
{
    float position;
    float smooth;

    if (value <= soft) {
        return 1.0f;
    }
    if (value >= hard) {
        return 0.0f;
    }
    position = (value - soft) / (hard - soft);
    smooth = position * position * (3.0f - 2.0f * position);
    return 1.0f - smooth;
}

static uint32_t increment_saturated(uint32_t value)
{
    return value == UINT32_MAX ? UINT32_MAX : value + 1U;
}

static bool sensor_healthy(uint32_t status, bool fresh)
{
    return fresh && (status & IMU_SENSOR_DATA_VALID) != 0U &&
           (status & IMU_SENSOR_FAULT_MASK) == 0U;
}

static float clamp01(float value)
{
    if (value < 0.0f) {
        return 0.0f;
    }
    if (value > 1.0f) {
        return 1.0f;
    }
    return value;
}

static float elapsed_ns_to_seconds(uint64_t elapsed_ns)
{
    const uint64_t whole_seconds = elapsed_ns / NANOSECONDS_PER_SECOND;
    const uint64_t remainder_ns = elapsed_ns % NANOSECONDS_PER_SECOND;

    return (float)whole_seconds +
           (float)remainder_ns * SECONDS_PER_NANOSECOND;
}

static bool window_statistics(const imu_motion_quality_t *quality,
                              float *accel_variance,
                              float *gyro_variance)
{
    imu_vec3f_t accel_mean = {0.0f, 0.0f, 0.0f};
    imu_vec3f_t gyro_mean = {0.0f, 0.0f, 0.0f};
    float accel_energy = 0.0f;
    float gyro_energy = 0.0f;
    float inverse_count;
    uint32_t i;

    if (quality->window_count == 0U) {
        return false;
    }
    inverse_count = 1.0f / (float)quality->window_count;
    for (i = 0U; i < quality->window_count; ++i) {
        accel_mean.x += quality->accel_window[i].x;
        accel_mean.y += quality->accel_window[i].y;
        accel_mean.z += quality->accel_window[i].z;
        gyro_mean.x += quality->gyro_window[i].x;
        gyro_mean.y += quality->gyro_window[i].y;
        gyro_mean.z += quality->gyro_window[i].z;
    }
    accel_mean.x *= inverse_count;
    accel_mean.y *= inverse_count;
    accel_mean.z *= inverse_count;
    gyro_mean.x *= inverse_count;
    gyro_mean.y *= inverse_count;
    gyro_mean.z *= inverse_count;
    if (!vector_finite(accel_mean) || !vector_finite(gyro_mean)) {
        return false;
    }
    for (i = 0U; i < quality->window_count; ++i) {
        const float accel_x = quality->accel_window[i].x - accel_mean.x;
        const float accel_y = quality->accel_window[i].y - accel_mean.y;
        const float accel_z = quality->accel_window[i].z - accel_mean.z;
        const float gyro_x = quality->gyro_window[i].x - gyro_mean.x;
        const float gyro_y = quality->gyro_window[i].y - gyro_mean.y;
        const float gyro_z = quality->gyro_window[i].z - gyro_mean.z;

        accel_energy += accel_x * accel_x + accel_y * accel_y + accel_z * accel_z;
        gyro_energy += gyro_x * gyro_x + gyro_y * gyro_y + gyro_z * gyro_z;
    }
    *accel_variance = accel_energy * inverse_count;
    *gyro_variance = gyro_energy * inverse_count;
    return isfinite(*accel_variance) && isfinite(*gyro_variance) &&
           *accel_variance >= 0.0f && *gyro_variance >= 0.0f;
}

bool imu_motion_quality_init(imu_motion_quality_t *quality,
                             const imu_config_t *config)
{
    imu_motion_quality_t initialized;

    if (quality == NULL || !config_valid(config)) {
        return false;
    }
    memset(&initialized, 0, sizeof(initialized));
    initialized.window_length = config->quality_window_samples;
    initialized.initialized = true;
    *quality = initialized;
    return true;
}

bool imu_motion_quality_update(imu_motion_quality_t *quality,
                               const imu_config_t *config,
                               const imu_sample_t *sample,
                               float innovation_angle_rad)
{
    imu_motion_quality_t next;
    imu_vec3f_t accel;
    imu_vec3f_t gyro;
    float dt_s;
    float accel_norm;
    float gyro_norm;
    float accel_variance;
    float gyro_variance;
    float norm_quality;
    float vibration_quality;
    float jerk_quality;
    float innovation_quality;
    float persistence_ratio;
    float effective_innovation;
    float target_weight;
    float slew_step;
    bool accel_healthy;
    bool gyros_healthy = true;
    bool static_candidate;
    unsigned axis;

    if (quality == NULL || !quality->initialized || !config_valid(config) ||
        sample == NULL || !isfinite(innovation_angle_rad) ||
        innovation_angle_rad < 0.0f ||
        quality->window_length != config->quality_window_samples) {
        return false;
    }
    if (!sample_floats_finite(sample)) {
        return false;
    }
    accel = sample_accel(sample);
    gyro = sample_gyro(sample);
    if (!vector_finite(accel) || !vector_finite(gyro)) {
        return false;
    }
    if (quality->has_previous_sample) {
        uint64_t elapsed_ns;

        if (sample->timestamp_ns <= quality->last_timestamp_ns) {
            return false;
        }
        elapsed_ns = sample->timestamp_ns - quality->last_timestamp_ns;
        dt_s = elapsed_ns_to_seconds(elapsed_ns);
    } else {
        dt_s = config->nominal_dt_s;
    }
    if (!finite_positive(dt_s)) {
        return false;
    }

    next = *quality;
    accel_norm = vector_norm(accel);
    gyro_norm = vector_norm(gyro);
    if (!isfinite(accel_norm) || !isfinite(gyro_norm)) {
        return false;
    }
    next.accel_norm_residual = fabsf(accel_norm - config->gravity_m_s2);
    if (quality->has_previous_sample) {
        imu_vec3f_t delta;

        delta.x = accel.x - quality->previous_accel_m_s2.x;
        delta.y = accel.y - quality->previous_accel_m_s2.y;
        delta.z = accel.z - quality->previous_accel_m_s2.z;
        next.jerk = vector_norm(delta) / dt_s;
    } else {
        next.jerk = 0.0f;
    }
    next.innovation_angle_rad = innovation_angle_rad;
    if (!isfinite(next.accel_norm_residual) || !isfinite(next.jerk)) {
        return false;
    }

    next.accel_window[next.window_index] = accel;
    next.gyro_window[next.window_index] = gyro;
    next.window_index = (next.window_index + 1U) % next.window_length;
    if (next.window_count < next.window_length) {
        ++next.window_count;
    }
    if (!window_statistics(&next, &accel_variance, &gyro_variance)) {
        return false;
    }
    next.vibration_rms = sqrtf(accel_variance);
    if (!isfinite(next.vibration_rms)) {
        return false;
    }

    if (innovation_angle_rad > config->innovation_soft_rad) {
        next.innovation_persistence_s += dt_s;
        if (next.innovation_persistence_s > config->innovation_persistence_s) {
            next.innovation_persistence_s = config->innovation_persistence_s;
        }
    } else if (next.innovation_persistence_s > dt_s) {
        next.innovation_persistence_s -= dt_s;
    } else {
        next.innovation_persistence_s = 0.0f;
    }
    persistence_ratio = clamp01(next.innovation_persistence_s /
                                config->innovation_persistence_s);
    effective_innovation = innovation_angle_rad * persistence_ratio;
    norm_quality = smooth_quality(next.accel_norm_residual,
                                  config->accel_norm_soft_m_s2,
                                  config->accel_norm_hard_m_s2);
    vibration_quality = smooth_quality(next.vibration_rms,
                                       config->vibration_soft_m_s2,
                                       config->vibration_hard_m_s2);
    jerk_quality = smooth_quality(next.jerk, config->jerk_soft_m_s3,
                                  config->jerk_hard_m_s3);
    innovation_quality = smooth_quality(effective_innovation,
                                        config->innovation_soft_rad,
                                        config->innovation_hard_rad);
    target_weight = norm_quality * vibration_quality * jerk_quality *
                    innovation_quality;

    next.mode_reasons = IMU_MOTION_REASON_NONE;
    accel_healthy = sensor_healthy(sample->accel_status,
                                   sample->accel_sample_fresh);
    for (axis = 0U; axis < IMU_AXIS_COUNT; ++axis) {
        if (!sensor_healthy(sample->gyro_status[axis],
                            sample->gyro_sample_fresh[axis])) {
            gyros_healthy = false;
        }
    }
    if (norm_quality < 1.0f) {
        next.mode_reasons |= IMU_MOTION_REASON_ACCEL_NORM;
    }
    if (vibration_quality < 1.0f) {
        next.mode_reasons |= IMU_MOTION_REASON_VIBRATION;
    }
    if (jerk_quality < 1.0f) {
        next.mode_reasons |= IMU_MOTION_REASON_JERK;
    }
    if (innovation_quality < 1.0f) {
        next.mode_reasons |= IMU_MOTION_REASON_INNOVATION;
    }
    if (!accel_healthy) {
        next.mode_reasons |= IMU_MOTION_REASON_ACCEL_STATUS;
        target_weight = 0.0f;
    }
    if (!gyros_healthy) {
        next.mode_reasons |= IMU_MOTION_REASON_GYRO_STATUS;
    }

    if (!accel_healthy) {
        next.accel_weight = 0.0f;
    } else if (target_weight > next.accel_weight) {
        slew_step = config->weight_rise_per_s * dt_s;
        if (!isfinite(slew_step)) {
            return false;
        }
        next.accel_weight += slew_step;
        if (next.accel_weight > target_weight) {
            next.accel_weight = target_weight;
        }
    } else {
        slew_step = config->weight_fall_per_s * dt_s;
        if (!isfinite(slew_step)) {
            return false;
        }
        next.accel_weight -= slew_step;
        if (next.accel_weight < target_weight) {
            next.accel_weight = target_weight;
        }
    }
    next.accel_weight = clamp01(next.accel_weight);
    if (!isfinite(next.accel_weight)) {
        return false;
    }

    static_candidate = accel_healthy && gyros_healthy &&
                       gyro_norm <= config->static_gyro_max_rad_s &&
                       gyro_variance <= config->static_gyro_variance_max &&
                       next.accel_norm_residual <=
                           config->static_accel_norm_max_m_s2 &&
                       accel_variance <= config->static_accel_variance_max &&
                       next.jerk <= config->static_jerk_max_m_s3 &&
                       next.vibration_rms <=
                           config->static_vibration_max_m_s2;
    if (gyro_norm > config->static_gyro_max_rad_s) {
        next.mode_reasons |= IMU_MOTION_REASON_GYRO_MOTION;
    }
    if (gyro_variance > config->static_gyro_variance_max) {
        next.mode_reasons |= IMU_MOTION_REASON_GYRO_VARIANCE;
    }
    if (accel_variance > config->static_accel_variance_max) {
        next.mode_reasons |= IMU_MOTION_REASON_ACCEL_VARIANCE;
    }
    if (static_candidate) {
        next.static_enter_count = increment_saturated(next.static_enter_count);
        next.static_exit_count = 0U;
        if (!next.static_detected &&
            next.static_enter_count >= config->static_enter_samples) {
            next.static_detected = true;
        }
    } else {
        next.static_exit_count = increment_saturated(next.static_exit_count);
        next.static_enter_count = 0U;
        if (next.static_detected &&
            next.static_exit_count >= config->static_exit_samples) {
            next.static_detected = false;
        }
    }

    next.previous_accel_m_s2 = accel;
    next.last_timestamp_ns = sample->timestamp_ns;
    next.has_previous_sample = true;
    *quality = next;
    return true;
}
