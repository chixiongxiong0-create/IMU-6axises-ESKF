#include "imu/imu_math.h"

#include <math.h>
#include <string.h>

#define IMU_QUAT_NORM_MIN 1.0e-12f
#define IMU_SMALL_ANGLE_MAX 1.0e-4f

static bool vec3_is_finite(imu_vec3f_t v)
{
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

static bool quat_is_finite(imu_quatf_t q)
{
    return isfinite(q.w) && isfinite(q.x) && isfinite(q.y) && isfinite(q.z);
}

static imu_vec3f_t invalid_vec3(void)
{
    const imu_vec3f_t invalid = {NAN, NAN, NAN};
    return invalid;
}

static imu_quatf_t invalid_quat(void)
{
    const imu_quatf_t invalid = {NAN, NAN, NAN, NAN};
    return invalid;
}

float imu_vec3_dot(imu_vec3f_t a, imu_vec3f_t b)
{
    float result;

    if (!vec3_is_finite(a) || !vec3_is_finite(b)) {
        return NAN;
    }
    result = a.x * b.x + a.y * b.y + a.z * b.z;
    return isfinite(result) ? result : NAN;
}

imu_vec3f_t imu_vec3_cross(imu_vec3f_t a, imu_vec3f_t b)
{
    imu_vec3f_t result;

    if (!vec3_is_finite(a) || !vec3_is_finite(b)) {
        return invalid_vec3();
    }
    result.x = a.y * b.z - a.z * b.y;
    result.y = a.z * b.x - a.x * b.z;
    result.z = a.x * b.y - a.y * b.x;
    return vec3_is_finite(result) ? result : invalid_vec3();
}

float imu_vec3_norm(imu_vec3f_t v)
{
    float result;

    if (!vec3_is_finite(v)) {
        return NAN;
    }
    result = hypotf(hypotf(v.x, v.y), v.z);
    return isfinite(result) ? result : NAN;
}

bool imu_mat3_multiply(float a[3][3], float b[3][3], float out[3][3])
{
    float result[3][3];
    unsigned i;
    unsigned j;
    unsigned k;

    if (a == NULL || b == NULL || out == NULL) {
        return false;
    }
    for (i = 0U; i < 3U; ++i) {
        for (j = 0U; j < 3U; ++j) {
            result[i][j] = 0.0f;
            for (k = 0U; k < 3U; ++k) {
                if (!isfinite(a[i][k]) || !isfinite(b[k][j])) {
                    return false;
                }
                result[i][j] += a[i][k] * b[k][j];
            }
            if (!isfinite(result[i][j])) {
                return false;
            }
        }
    }
    memcpy(out, result, sizeof(result));
    return true;
}

void imu_mat6_zero(float out[IMU_ERROR_STATE_DIM][IMU_ERROR_STATE_DIM])
{
    if (out != NULL) {
        memset(out, 0, sizeof(float) * IMU_ERROR_STATE_DIM * IMU_ERROR_STATE_DIM);
    }
}

void imu_mat6_identity(float out[IMU_ERROR_STATE_DIM][IMU_ERROR_STATE_DIM])
{
    unsigned i;

    if (out == NULL) {
        return;
    }
    imu_mat6_zero(out);
    for (i = 0U; i < IMU_ERROR_STATE_DIM; ++i) {
        out[i][i] = 1.0f;
    }
}

bool imu_mat6_multiply(float a[IMU_ERROR_STATE_DIM][IMU_ERROR_STATE_DIM],
                       float b[IMU_ERROR_STATE_DIM][IMU_ERROR_STATE_DIM],
                       float out[IMU_ERROR_STATE_DIM][IMU_ERROR_STATE_DIM])
{
    float result[IMU_ERROR_STATE_DIM][IMU_ERROR_STATE_DIM];
    unsigned i;
    unsigned j;
    unsigned k;

    if (a == NULL || b == NULL || out == NULL) {
        return false;
    }
    for (i = 0U; i < IMU_ERROR_STATE_DIM; ++i) {
        for (j = 0U; j < IMU_ERROR_STATE_DIM; ++j) {
            result[i][j] = 0.0f;
            for (k = 0U; k < IMU_ERROR_STATE_DIM; ++k) {
                if (!isfinite(a[i][k]) || !isfinite(b[k][j])) {
                    return false;
                }
                result[i][j] += a[i][k] * b[k][j];
            }
            if (!isfinite(result[i][j])) {
                return false;
            }
        }
    }
    memcpy(out, result, sizeof(result));
    return true;
}

bool imu_mat6_transpose(float in[IMU_ERROR_STATE_DIM][IMU_ERROR_STATE_DIM],
                        float out[IMU_ERROR_STATE_DIM][IMU_ERROR_STATE_DIM])
{
    float result[IMU_ERROR_STATE_DIM][IMU_ERROR_STATE_DIM];
    unsigned i;
    unsigned j;

    if (in == NULL || out == NULL) {
        return false;
    }
    for (i = 0U; i < IMU_ERROR_STATE_DIM; ++i) {
        for (j = 0U; j < IMU_ERROR_STATE_DIM; ++j) {
            if (!isfinite(in[j][i])) {
                return false;
            }
            result[i][j] = in[j][i];
        }
    }
    memcpy(out, result, sizeof(result));
    return true;
}

bool imu_mat6_symmetrize(float in_out[IMU_ERROR_STATE_DIM][IMU_ERROR_STATE_DIM])
{
    float result[IMU_ERROR_STATE_DIM][IMU_ERROR_STATE_DIM];
    unsigned i;
    unsigned j;

    if (in_out == NULL) {
        return false;
    }
    for (i = 0U; i < IMU_ERROR_STATE_DIM; ++i) {
        for (j = 0U; j < IMU_ERROR_STATE_DIM; ++j) {
            if (!isfinite(in_out[i][j]) || !isfinite(in_out[j][i])) {
                return false;
            }
            result[i][j] = 0.5f * (in_out[i][j] + in_out[j][i]);
            if (!isfinite(result[i][j])) {
                return false;
            }
        }
    }
    memcpy(in_out, result, sizeof(result));
    return true;
}

imu_quatf_t imu_quat_multiply(imu_quatf_t a, imu_quatf_t b)
{
    imu_quatf_t result;

    if (!quat_is_finite(a) || !quat_is_finite(b)) {
        return invalid_quat();
    }
    result.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    result.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    result.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    result.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
    return quat_is_finite(result) ? result : invalid_quat();
}

imu_quatf_t imu_quat_conjugate(imu_quatf_t q)
{
    if (!quat_is_finite(q)) {
        return invalid_quat();
    }
    q.x = -q.x;
    q.y = -q.y;
    q.z = -q.z;
    return q;
}

float imu_quat_norm(imu_quatf_t q)
{
    float result;

    if (!quat_is_finite(q)) {
        return NAN;
    }
    result = hypotf(hypotf(q.w, q.x), hypotf(q.y, q.z));
    return isfinite(result) ? result : NAN;
}

bool imu_quat_normalize(imu_quatf_t *q)
{
    imu_quatf_t result;
    float norm;

    if (q == NULL || !quat_is_finite(*q)) {
        return false;
    }
    norm = imu_quat_norm(*q);
    if (!isfinite(norm) || norm < IMU_QUAT_NORM_MIN) {
        return false;
    }
    result.w = q->w / norm;
    result.x = q->x / norm;
    result.y = q->y / norm;
    result.z = q->z / norm;
    if (!quat_is_finite(result)) {
        return false;
    }
    *q = result;
    return true;
}

bool imu_quat_rotate(imu_quatf_t q, imu_vec3f_t in, imu_vec3f_t *out)
{
    const imu_quatf_t vector_q = {0.0f, in.x, in.y, in.z};
    imu_quatf_t rotated;
    imu_vec3f_t result;

    if (out == NULL || !vec3_is_finite(in) || !imu_quat_normalize(&q)) {
        return false;
    }
    rotated = imu_quat_multiply(imu_quat_multiply(q, vector_q),
                                imu_quat_conjugate(q));
    result.x = rotated.x;
    result.y = rotated.y;
    result.z = rotated.z;
    if (!vec3_is_finite(result)) {
        return false;
    }
    *out = result;
    return true;
}

bool imu_quat_integrate(imu_quatf_t *q_nb, imu_vec3f_t omega_body_rad_s,
                        float dt_s)
{
    imu_quatf_t base;
    imu_quatf_t delta;
    imu_quatf_t result;
    float rate_norm;
    float angle;
    float scale;

    if (q_nb == NULL || !vec3_is_finite(omega_body_rad_s) ||
        !isfinite(dt_s) || dt_s <= 0.0f) {
        return false;
    }
    base = *q_nb;
    if (!imu_quat_normalize(&base)) {
        return false;
    }
    rate_norm = imu_vec3_norm(omega_body_rad_s);
    angle = rate_norm * dt_s;
    if (!isfinite(angle)) {
        return false;
    }
    if (angle < IMU_SMALL_ANGLE_MAX) {
        scale = 0.5f * dt_s;
        delta.w = 1.0f;
    } else {
        const float half_angle = 0.5f * angle;
        scale = sinf(half_angle) / rate_norm;
        delta.w = cosf(half_angle);
    }
    delta.x = omega_body_rad_s.x * scale;
    delta.y = omega_body_rad_s.y * scale;
    delta.z = omega_body_rad_s.z * scale;
    result = imu_quat_multiply(base, delta);
    if (!imu_quat_normalize(&result)) {
        return false;
    }
    *q_nb = result;
    return true;
}

bool imu_quat_to_euler_ned(imu_quatf_t q_nb, imu_vec3f_t *roll_pitch_yaw_rad)
{
    imu_vec3f_t result;
    float pitch_sine;

    if (roll_pitch_yaw_rad == NULL || !imu_quat_normalize(&q_nb)) {
        return false;
    }
    result.x = atan2f(2.0f * (q_nb.w * q_nb.x + q_nb.y * q_nb.z),
                      1.0f - 2.0f * (q_nb.x * q_nb.x + q_nb.y * q_nb.y));
    pitch_sine = 2.0f * (q_nb.w * q_nb.y - q_nb.z * q_nb.x);
    if (pitch_sine > 1.0f) {
        pitch_sine = 1.0f;
    } else if (pitch_sine < -1.0f) {
        pitch_sine = -1.0f;
    }
    result.y = asinf(pitch_sine);
    result.z = atan2f(2.0f * (q_nb.w * q_nb.z + q_nb.x * q_nb.y),
                      1.0f - 2.0f * (q_nb.y * q_nb.y + q_nb.z * q_nb.z));
    if (!vec3_is_finite(result)) {
        return false;
    }
    *roll_pitch_yaw_rad = result;
    return true;
}

float imu_quat_yaw_ned(imu_quatf_t q_nb)
{
    imu_vec3f_t euler;

    return imu_quat_to_euler_ned(q_nb, &euler) ? euler.z : NAN;
}
