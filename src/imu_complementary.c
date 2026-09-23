#include "imu/imu_complementary.h"

#include "imu/imu_math.h"
#include "imu_timing.h"

#include <float.h>
#include <math.h>
#include <string.h>

#define NS_PER_SECOND UINT64_C(1000000000)

static bool finite_vec(imu_vec3f_t v)
{
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

static bool finite_quat(imu_quatf_t q)
{
    return isfinite(q.w) && isfinite(q.x) && isfinite(q.y) && isfinite(q.z);
}

static bool config_valid(const imu_config_t *c)
{
    uint64_t delay_ns;
    return c != NULL && imu_common_gyro_delay_ns(c,&delay_ns) &&
           isfinite(c->min_dt_s) && c->min_dt_s > 0.0f &&
           isfinite(c->max_dt_s) && c->max_dt_s >= c->min_dt_s &&
           isfinite(c->complementary_kp) && c->complementary_kp >= 0.0f &&
           isfinite(c->complementary_ki) && c->complementary_ki >= 0.0f &&
           isfinite(c->max_bias_step_rad_s) &&
           c->max_bias_step_rad_s > 0.0f &&
           isfinite(c->accel_weight_min) &&
           c->accel_weight_min >= 0.0f && c->accel_weight_min <= 1.0f;
}

static bool sample_finite(const imu_sample_t *s)
{
    unsigned i;
    if (!isfinite(s->accel_temperature_c)) return false;
    for (i = 0U; i < IMU_AXIS_COUNT; ++i) {
        if (!isfinite(s->gyro_rad_s[i]) ||
            !isfinite(s->gyro_temperature_c[i]) ||
            !isfinite(s->accel_m_s2[i])) return false;
    }
    return true;
}

static bool healthy(uint32_t status, bool fresh)
{
    return fresh && (status & IMU_SENSOR_DATA_VALID) != 0U &&
           (status & IMU_SENSOR_FAULT_MASK) == 0U;
}

static float elapsed_seconds(uint64_t elapsed_ns)
{
    return (float)(elapsed_ns / NS_PER_SECOND) +
           (float)(elapsed_ns % NS_PER_SECOND) * 1.0e-9f;
}

static imu_status_t fill_output(const imu_complementary_t *f,
                                const imu_config_t *c, const imu_sample_t *s, float shared_weight,
                                bool corrected,
                                imu_status_t status, bool gyro_good,
                                bool accel_good, imu_output_t *out)
{
    imu_vec3f_t euler;
    uint64_t delay_ns;
    unsigned i;
    if (!imu_quat_to_euler_ned(f->q_nb, &euler))
        return IMU_STATUS_NUMERICAL_FAULT;
    memset(out, 0, sizeof(*out));
    out->timestamp_ns = s->timestamp_ns;
    out->q_nb = f->q_nb;
    out->roll_rad = euler.x;
    out->pitch_rad = euler.y;
    out->yaw_rad = euler.z;
    out->angular_rate_rad_s = f->angular_rate_rad_s;
    out->gyro_bias_rad_s = f->gyro_bias_rad_s;
    /* Benchmark has no covariance: flag distinguishes sentinel from sigma. */
    out->attitude_std_rad = (imu_vec3f_t){IMU_UNCERTAINTY_UNAVAILABLE_RAD,
        IMU_UNCERTAINTY_UNAVAILABLE_RAD, IMU_UNCERTAINTY_UNAVAILABLE_RAD};
    out->accel_weight = accel_good ? shared_weight : 0.0f;
    out->fusion_mode = corrected &&
                       status == IMU_STATUS_OK ? IMU_FUSION_GRAVITY :
                                                 IMU_FUSION_GYRO_ONLY;
    out->status = status;
    out->timing_status = status == IMU_STATUS_TIME_GAP ?
                         IMU_STATUS_TIME_GAP : IMU_STATUS_OK;
    out->initialized = true;
    out->calibration_valid = true;
    out->health_flags = IMU_HEALTH_HEADING_UNOBSERVABLE |
                        IMU_HEALTH_UNCERTAINTY_UNAVAILABLE;
    if (!gyro_good) out->health_flags |= IMU_HEALTH_GYRO_DEGRADED;
    if (!accel_good) out->health_flags |= IMU_HEALTH_ACCEL_DEGRADED;
    if (status == IMU_STATUS_TIME_GAP)
        out->health_flags |= IMU_HEALTH_TIMING_DEGRADED;
    for (i = 0U; i < IMU_AXIS_COUNT; ++i)
        out->gyro_status[i] = s->gyro_status[i];
    out->accel_status = s->accel_status;
    if(!imu_common_gyro_delay_ns(c,&delay_ns)) return IMU_STATUS_BAD_ARGUMENT;
    imu_output_epoch(out,delay_ns);
    return IMU_STATUS_OK;
}

imu_status_t imu_complementary_init(imu_complementary_t *f,
                                    const imu_config_t *c,
                                    imu_quatf_t q_nb,
                                    imu_vec3f_t bias)
{
    imu_complementary_t next;
    if (f == NULL || c == NULL) return IMU_STATUS_BAD_ARGUMENT;
    if (!finite_quat(q_nb) || !finite_vec(bias)) return IMU_STATUS_NONFINITE;
    if (!config_valid(c)) return IMU_STATUS_BAD_ARGUMENT;
    if (!imu_quat_normalize(&q_nb)) return IMU_STATUS_BAD_ARGUMENT;
    memset(&next, 0, sizeof(next));
    next.q_nb = q_nb;
    next.gyro_bias_rad_s = bias;
    next.initialized = true;
    *f = next;
    return IMU_STATUS_OK;
}

imu_status_t imu_complementary_process(imu_complementary_t *f,
                                       const imu_config_t *c,
                                       const imu_sample_t *s,
                                       const imu_motion_quality_t *quality,
                                       imu_output_t *out)
{
    imu_complementary_t next;
    imu_output_t result;
    imu_vec3f_t measured, predicted, residual, correction, integral, rate;
    float dt = 0.0f, weight = 0.0f, norm, integral_norm;
    imu_status_t status = IMU_STATUS_OK;
    bool gyro_good = true, accel_good;
    unsigned i;

    if (f == NULL || c == NULL || s == NULL || quality == NULL || out == NULL)
        return IMU_STATUS_BAD_ARGUMENT;
    if (!f->initialized) return IMU_STATUS_NOT_INITIALIZED;
    if (!finite_quat(f->q_nb) || !finite_vec(f->gyro_bias_rad_s) ||
        !finite_vec(f->angular_rate_rad_s) || !sample_finite(s) ||
        !isfinite(quality->accel_weight)) return IMU_STATUS_NONFINITE;
    if (!config_valid(c) || quality->accel_weight < 0.0f ||
        quality->accel_weight > 1.0f) return IMU_STATUS_BAD_ARGUMENT;
    if (f->has_timestamp) {
        if (s->timestamp_ns <= f->last_timestamp_ns)
            return IMU_STATUS_TIME_REVERSED;
        dt = elapsed_seconds(s->timestamp_ns - f->last_timestamp_ns);
        if (!isfinite(dt)) return IMU_STATUS_NONFINITE;
        if (dt < c->min_dt_s) return IMU_STATUS_BAD_ARGUMENT;
        if (dt > c->max_dt_s) status = IMU_STATUS_TIME_GAP;
    }
    for (i = 0U; i < IMU_AXIS_COUNT; ++i)
        gyro_good = gyro_good && healthy(s->gyro_status[i],
                                         s->gyro_sample_fresh[i]);
    accel_good = healthy(s->accel_status, s->accel_sample_fresh) &&
                 quality->initialized &&
                 quality->last_timestamp_ns == s->timestamp_ns;
    if (!gyro_good && status == IMU_STATUS_OK)
        status = IMU_STATUS_SENSOR_DEGRADED;
    if (!accel_good && status == IMU_STATUS_OK)
        status = IMU_STATUS_SENSOR_DEGRADED;

    next = *f;
    next.last_timestamp_ns = s->timestamp_ns;
    next.has_timestamp = true;
    if (f->has_timestamp && gyro_good && status != IMU_STATUS_TIME_GAP) {
        rate = (imu_vec3f_t){s->gyro_rad_s[0] - f->gyro_bias_rad_s.x,
                             s->gyro_rad_s[1] - f->gyro_bias_rad_s.y,
                             s->gyro_rad_s[2] - f->gyro_bias_rad_s.z};
        if (!finite_vec(rate) || !imu_quat_integrate(&next.q_nb, rate, dt))
            return IMU_STATUS_NUMERICAL_FAULT;
        if (accel_good && quality->accel_weight >= c->accel_weight_min &&
            quality->accel_weight > 0.0f) {
            measured = (imu_vec3f_t){s->accel_m_s2[0], s->accel_m_s2[1],
                                      s->accel_m_s2[2]};
            norm = imu_vec3_norm(measured);
            if (!isfinite(norm)) return IMU_STATUS_NUMERICAL_FAULT;
            if (norm > 1.0e-12f) {
                measured.x /= norm; measured.y /= norm; measured.z /= norm;
                if (!imu_quat_rotate(imu_quat_conjugate(next.q_nb),
                                     (imu_vec3f_t){0,0,-1}, &predicted))
                    return IMU_STATUS_NUMERICAL_FAULT;
                /* Same z cross h residual and hemisphere guard as the ESKF. */
                if (imu_vec3_dot(measured, predicted) > 0.0f) {
                    residual = imu_vec3_cross(measured, predicted);
                    weight = quality->accel_weight;
                    correction = (imu_vec3f_t){c->complementary_kp*weight*residual.x,
                                                c->complementary_kp*weight*residual.y,
                                                c->complementary_kp*weight*residual.z};
                    integral = (imu_vec3f_t){
                        -c->complementary_ki*weight*residual.x*dt,
                        -c->complementary_ki*weight*residual.y*dt,
                        -c->complementary_ki*weight*residual.z*dt};
                    integral_norm = imu_vec3_norm(integral);
                    if (!finite_vec(correction) || !finite_vec(integral) ||
                        !isfinite(integral_norm)) return IMU_STATUS_NUMERICAL_FAULT;
                    if (integral_norm > c->max_bias_step_rad_s) {
                        const float scale = c->max_bias_step_rad_s / integral_norm;
                        integral.x *= scale; integral.y *= scale; integral.z *= scale;
                    }
                    next.gyro_bias_rad_s = (imu_vec3f_t){
                        f->gyro_bias_rad_s.x + integral.x,
                        f->gyro_bias_rad_s.y + integral.y,
                        f->gyro_bias_rad_s.z + integral.z};
                    if (!imu_quat_integrate(&next.q_nb, correction, dt))
                        return IMU_STATUS_NUMERICAL_FAULT;
                } else {
                    accel_good = false;
                }
            } else {
                accel_good = false;
            }
            if (!accel_good && status == IMU_STATUS_OK)
                status = IMU_STATUS_SENSOR_DEGRADED;
        }
        if (!finite_vec(rate) || !finite_vec(next.gyro_bias_rad_s))
            return IMU_STATUS_NUMERICAL_FAULT;
        next.angular_rate_rad_s = (imu_vec3f_t){
            s->gyro_rad_s[0] - next.gyro_bias_rad_s.x,
            s->gyro_rad_s[1] - next.gyro_bias_rad_s.y,
            s->gyro_rad_s[2] - next.gyro_bias_rad_s.z};
        if (!finite_vec(next.angular_rate_rad_s))
            return IMU_STATUS_NUMERICAL_FAULT;
    } else if (gyro_good && status != IMU_STATUS_TIME_GAP) {
        next.angular_rate_rad_s = (imu_vec3f_t){
            s->gyro_rad_s[0] - next.gyro_bias_rad_s.x,
            s->gyro_rad_s[1] - next.gyro_bias_rad_s.y,
            s->gyro_rad_s[2] - next.gyro_bias_rad_s.z};
        if (!finite_vec(next.angular_rate_rad_s))
            return IMU_STATUS_NUMERICAL_FAULT;
    } else {
        next.angular_rate_rad_s = (imu_vec3f_t){0,0,0};
    }
    if (fill_output(&next, c, s, quality->accel_weight, weight > 0.0f,
                    status, gyro_good, accel_good,
                    &result) != IMU_STATUS_OK)
        return IMU_STATUS_NUMERICAL_FAULT;
    *f = next;
    *out = result;
    return result.status;
}
