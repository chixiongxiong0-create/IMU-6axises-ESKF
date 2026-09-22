#include "imu/imu_esekf.h"
#include "imu/imu_math.h"

#include <math.h>
#include <string.h>

static bool finite_vec(imu_vec3f_t v)
{
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

/* Finite positive diagonals and symmetry. This is
 * not a full PSD proof: long unaided single-precision propagation can become
 * ill-conditioned. Innovation inversion separately checks positive pivots. */
bool imu_esekf_covariance_valid(const imu_esekf_t *f)
{
    unsigned i, j;
    if (f == NULL) return false;
    for (i = 0; i < 6; ++i)
        if (!isfinite(f->P[i][i]) || f->P[i][i] <= 0) return false;
    for (i = 0; i < 6; ++i) {
        for (j = 0; j < 6; ++j) {
            const float a = f->P[i][j], b = f->P[j][i];
            if (!isfinite(a) || !isfinite(b) ||
                fabsf(a-b) > 1e-5f*fmaxf(fabsf(a), fabsf(b))) return false;
        }
    }
    return true;
}

static imu_status_t state_valid(const imu_esekf_t *f)
{
    float norm;
    if (f == NULL) return IMU_STATUS_BAD_ARGUMENT;
    if (!f->initialized) return IMU_STATUS_NOT_INITIALIZED;
    norm = imu_quat_norm(f->q_nb);
    if (!isfinite(norm) || fabsf(norm-1.0f) > 1e-4f ||
        !finite_vec(f->gyro_bias_rad_s) || !finite_vec(f->angular_rate_rad_s) ||
        !imu_esekf_covariance_valid(f)) return IMU_STATUS_NUMERICAL_FAULT;
    return IMU_STATUS_OK;
}

static void skew(imu_vec3f_t v, float s[3][3])
{
    s[0][0] = 0;    s[0][1] = -v.z; s[0][2] = v.y;
    s[1][0] = v.z;  s[1][1] = 0;    s[1][2] = -v.x;
    s[2][0] = -v.y; s[2][1] = v.x;  s[2][2] = 0;
}

static bool transform_covariance(float p[6][6], float transform[6][6])
{
    float temp[6][6], transpose[6][6];
    return imu_mat6_transpose(transform, transpose) &&
           imu_mat6_multiply(transform, p, temp) &&
           imu_mat6_multiply(temp, transpose, p);
}

imu_status_t imu_esekf_init(imu_esekf_t *f, const imu_config_t *c,
                          imu_quatf_t q, imu_vec3f_t bias)
{
    imu_esekf_t next;
    unsigned i;
    if (f == NULL || c == NULL) return IMU_STATUS_BAD_ARGUMENT;
    if (!finite_vec(bias) || !isfinite(imu_quat_norm(q)))
        return IMU_STATUS_NONFINITE;
    if (!imu_quat_normalize(&q)) return IMU_STATUS_BAD_ARGUMENT;
    memset(&next, 0, sizeof(next));
    for (i = 0; i < 6; ++i) {
        if (!isfinite(c->initial_covariance_diag[i])) return IMU_STATUS_NONFINITE;
        if (c->initial_covariance_diag[i] <= 0) return IMU_STATUS_BAD_ARGUMENT;
        next.P[i][i] = c->initial_covariance_diag[i];
    }
    next.q_nb = q;
    next.gyro_bias_rad_s = bias;
    next.initialized = true;
    *f = next;
    return IMU_STATUS_OK;
}

imu_status_t imu_esekf_predict(imu_esekf_t *f, const imu_config_t *c,
                             imu_vec3f_t gyro, float dt)
{
    imu_esekf_t next;
    float transition[6][6], s[3][3], qtheta, qbias;
    unsigned i, j;
    imu_status_t status;
    if (c == NULL) return IMU_STATUS_BAD_ARGUMENT;
    status = state_valid(f);
    if (status != IMU_STATUS_OK) return status;
    if (!finite_vec(gyro) || !isfinite(dt) ||
        !isfinite(c->gyro_noise_density) || !isfinite(c->gyro_bias_rw))
        return IMU_STATUS_NONFINITE;
    if (dt <= 0 || c->gyro_noise_density < 0 || c->gyro_bias_rw < 0)
        return IMU_STATUS_BAD_ARGUMENT;
    next = *f;
    next.angular_rate_rad_s = (imu_vec3f_t){gyro.x-f->gyro_bias_rad_s.x,
        gyro.y-f->gyro_bias_rad_s.y, gyro.z-f->gyro_bias_rad_s.z};
    if (!imu_quat_integrate(&next.q_nb, next.angular_rate_rad_s, dt))
        return IMU_STATUS_NUMERICAL_FAULT;
    imu_mat6_identity(transition);
    skew(next.angular_rate_rad_s, s);
    for (i = 0; i < 3; ++i) {
        for (j = 0; j < 3; ++j) transition[i][j] -= s[i][j]*dt;
        transition[i][i+3] = -dt;
    }
    if (!transform_covariance(next.P, transition)) return IMU_STATUS_NUMERICAL_FAULT;
    qtheta = c->gyro_noise_density*c->gyro_noise_density*dt;
    qbias = c->gyro_bias_rw*c->gyro_bias_rw*dt;
    for (i = 0; i < 3; ++i) {
        next.P[i][i] += qtheta;
        next.P[i+3][i+3] += qbias;
    }
    if (!imu_mat6_symmetrize(next.P) || !imu_esekf_covariance_valid(&next))
        return IMU_STATUS_NUMERICAL_FAULT;
    *f = next;
    return IMU_STATUS_OK;
}

/* Scale before cofactors so the determinant test measures conditioning rather
 * than units. Positive leading minors enforce SPD; tiny determinants reject
 * near-singular inverses before they can produce enormous corrections. */
static bool inverse_symmetric3(float s[3][3], float inverse[3][3])
{
    float a[3][3], cofactor[3][3], scale = 0, det;
    unsigned i, j;
    for (i = 0; i < 3; ++i) {
        if (!isfinite(s[i][i]) || s[i][i] <= 0) return false;
        scale = fmaxf(scale, s[i][i]);
    }
    for (i = 0; i < 3; ++i)
        for (j = 0; j < 3; ++j) {
            if (!isfinite(s[i][j])) return false;
            a[i][j] = (0.5f*s[i][j] + 0.5f*s[j][i])/scale;
        }
    cofactor[0][0] = a[1][1]*a[2][2]-a[1][2]*a[1][2];
    cofactor[0][1] = a[0][2]*a[1][2]-a[0][1]*a[2][2];
    cofactor[0][2] = a[0][1]*a[1][2]-a[0][2]*a[1][1];
    cofactor[1][1] = a[0][0]*a[2][2]-a[0][2]*a[0][2];
    cofactor[1][2] = a[0][1]*a[0][2]-a[0][0]*a[1][2];
    cofactor[2][2] = a[0][0]*a[1][1]-a[0][1]*a[0][1];
    cofactor[1][0] = cofactor[0][1];
    cofactor[2][0] = cofactor[0][2];
    cofactor[2][1] = cofactor[1][2];
    det = a[0][0]*cofactor[0][0] + a[0][1]*cofactor[0][1] +
          a[0][2]*cofactor[0][2];
    if (a[0][0] <= 1e-7f || cofactor[2][2] <= 1e-7f ||
        !isfinite(det) || det <= 1e-7f) return false;
    for (i = 0; i < 3; ++i)
        for (j = 0; j < 3; ++j) {
            inverse[i][j] = (cofactor[i][j]/det)/scale;
            if (!isfinite(inverse[i][j])) return false;
        }
    return true;
}

/* H and residual share coordinates; R = variance*I. A zero nis_limit disables
 * the innovation gate (used only by the separately qualified static update). */
static imu_status_t measurement_update(imu_esekf_t *f, float h[3][6],
                                       const float residual[3], float variance,
                                       float nis_limit, float max_bias_step)
{
    imu_esekf_t next = *f;
    float pht[6][3] = {{0}}, s[3][3] = {{0}}, inverse[3][3];
    float gain[6][3] = {{0}}, delta[6] = {0}, transform[6][6], skew_delta[3][3];
    float nis = 0;
    imu_vec3f_t angle;
    unsigned i, j, k;
    if (!isfinite(variance) || variance <= 0) return IMU_STATUS_NUMERICAL_FAULT;
    for (i = 0; i < 6; ++i)
        for (j = 0; j < 3; ++j)
            for (k = 0; k < 6; ++k) pht[i][j] += f->P[i][k]*h[j][k];
    for (i = 0; i < 3; ++i)
        for (j = 0; j < 3; ++j) {
            for (k = 0; k < 6; ++k) s[i][j] += h[i][k]*pht[k][j];
            if (i == j) s[i][j] += variance;
        }
    if (!inverse_symmetric3(s, inverse)) return IMU_STATUS_NUMERICAL_FAULT;
    for (i = 0; i < 3; ++i)
        for (j = 0; j < 3; ++j) nis += residual[i]*inverse[i][j]*residual[j];
    if (!isfinite(nis) || nis < 0) return IMU_STATUS_NUMERICAL_FAULT;
    if (nis_limit > 0 && nis > nis_limit) return IMU_STATUS_AIDING_UNAVAILABLE;
    for (i = 0; i < 6; ++i) {
        for (j = 0; j < 3; ++j) {
            for (k = 0; k < 3; ++k) gain[i][j] += pht[i][k]*inverse[k][j];
            delta[i] += gain[i][j]*residual[j];
        }
        if (!isfinite(delta[i])) return IMU_STATUS_NUMERICAL_FAULT;
    }
    {
        const float norm = imu_vec3_norm((imu_vec3f_t){delta[3], delta[4], delta[5]});
        if (!isfinite(norm)) return IMU_STATUS_NUMERICAL_FAULT;
        if (norm > max_bias_step) {
            const float scale = max_bias_step/norm;
            /* Limit the whole gain consistently, not only the nominal bias;
             * Joseph then reflects the actual reduced information used. */
            for (i = 0; i < 6; ++i) {
                delta[i] *= scale;
                for (j = 0; j < 3; ++j) gain[i][j] *= scale;
            }
        }
    }
    imu_mat6_identity(transform);
    for (i = 0; i < 6; ++i)
        for (j = 0; j < 6; ++j)
            for (k = 0; k < 3; ++k) transform[i][j] -= gain[i][k]*h[k][j];
    /* Joseph: (I-KH)P(I-KH)' + KRK', including when gain is limited. */
    if (!transform_covariance(next.P, transform)) return IMU_STATUS_NUMERICAL_FAULT;
    for (i = 0; i < 6; ++i)
        for (j = 0; j < 6; ++j)
            for (k = 0; k < 3; ++k)
                next.P[i][j] += gain[i][k]*variance*gain[j][k];
    angle = (imu_vec3f_t){delta[0], delta[1], delta[2]};
    if (!imu_quat_integrate(&next.q_nb, angle, 1.0f)) return IMU_STATUS_NUMERICAL_FAULT;
    next.gyro_bias_rad_s.x += delta[3];
    next.gyro_bias_rad_s.y += delta[4];
    next.gyro_bias_rad_s.z += delta[5];
    next.angular_rate_rad_s.x -= delta[3];
    next.angular_rate_rad_s.y -= delta[4];
    next.angular_rate_rad_s.z -= delta[5];
    /* After right injection reset error coordinates with G_theta = I-.5*[d]x.
     * Bias coordinates remain body-fixed, so their reset block is identity. */
    imu_mat6_identity(transform);
    skew(angle, skew_delta);
    for (i = 0; i < 3; ++i)
        for (j = 0; j < 3; ++j) transform[i][j] -= 0.5f*skew_delta[i][j];
    if (!transform_covariance(next.P, transform) || !imu_mat6_symmetrize(next.P) ||
        state_valid(&next) != IMU_STATUS_OK) return IMU_STATUS_NUMERICAL_FAULT;
    *f = next;
    return IMU_STATUS_OK;
}

imu_status_t imu_esekf_update_gravity(imu_esekf_t *f, const imu_config_t *c,
                                    imu_vec3f_t accel, float weight)
{
    imu_vec3f_t predicted, measured, cross;
    float norm, h[3][6] = {{0}}, direction[3], residual[3];
    unsigned i, j;
    imu_status_t status;
    if (c == NULL) return IMU_STATUS_BAD_ARGUMENT;
    status = state_valid(f);
    if (status != IMU_STATUS_OK) return status;
    if (!finite_vec(accel) || !isfinite(weight) ||
        !isfinite(c->accel_direction_noise_variance) || !isfinite(c->gravity_nis_limit) ||
        !isfinite(c->max_bias_step_rad_s))
        return IMU_STATUS_NONFINITE;
    if (weight < 0 || weight > 1 || c->accel_direction_noise_variance <= 0 ||
        c->gravity_nis_limit <= 0 || c->max_bias_step_rad_s <= 0)
        return IMU_STATUS_BAD_ARGUMENT;
    if (weight == 0) return IMU_STATUS_OK;
    norm = imu_vec3_norm(accel);
    if (!isfinite(norm)) return IMU_STATUS_NUMERICAL_FAULT;
    if (norm < 1e-12f) return IMU_STATUS_BAD_ARGUMENT;
    measured = (imu_vec3f_t){accel.x/norm, accel.y/norm, accel.z/norm};
    if (!imu_quat_rotate(imu_quat_conjugate(f->q_nb),
                        (imu_vec3f_t){0, 0, -1}, &predicted))
        return IMU_STATUS_NUMERICAL_FAULT;
    /* Cross residual is locally valid only in the same hemisphere. Reject
     * antiparallel measurements, whose cross product misleadingly vanishes. */
    if (imu_vec3_dot(measured, predicted) <= 0) return IMU_STATUS_AIDING_UNAVAILABLE;
    cross = imu_vec3_cross(measured, predicted);
    residual[0] = cross.x; residual[1] = cross.y; residual[2] = cross.z;
    direction[0] = predicted.x; direction[1] = predicted.y; direction[2] = predicted.z;
    /* z = h + [h]x*dtheta; z cross h = (I-hh')*dtheta.
     * H's nullspace is body gravity: no heading observation is introduced.
     * R uses isotropic direction noise, conservatively regularizing the unused
     * radial residual component for a guarded full 3x3 solve. */
    for (i = 0; i < 3; ++i)
        for (j = 0; j < 3; ++j)
            h[i][j] = (i == j ? 1.0f : 0.0f)-direction[i]*direction[j];
    return measurement_update(f, h, residual,
        c->accel_direction_noise_variance/weight, c->gravity_nis_limit,
        c->max_bias_step_rad_s);
}

imu_status_t imu_esekf_update_zero_rate(imu_esekf_t *f, const imu_config_t *c,
                                      imu_vec3f_t gyro, const uint32_t flags[3])
{
    imu_esekf_t next;
    float h[3][6] = {{0}}, residual[3];
    unsigned i;
    imu_status_t status;
    if (c == NULL || flags == NULL) return IMU_STATUS_BAD_ARGUMENT;
    status = state_valid(f);
    if (status != IMU_STATUS_OK) return status;
    if (!finite_vec(gyro) || !isfinite(c->zero_rate_noise_variance) ||
        !isfinite(c->max_bias_step_rad_s)) return IMU_STATUS_NONFINITE;
    if (c->zero_rate_noise_variance <= 0 || c->max_bias_step_rad_s <= 0)
        return IMU_STATUS_BAD_ARGUMENT;
    for (i = 0; i < 3; ++i) {
        if ((flags[i] & IMU_SENSOR_DATA_VALID) == 0 ||
            (flags[i] & IMU_SENSOR_FAULT_MASK) != 0) return IMU_STATUS_SENSOR_DEGRADED;
        h[i][i+3] = 1;
    }
    next = *f;
    residual[0] = gyro.x-f->gyro_bias_rad_s.x;
    residual[1] = gyro.y-f->gyro_bias_rad_s.y;
    residual[2] = gyro.z-f->gyro_bias_rad_s.z;
    next.angular_rate_rad_s = (imu_vec3f_t){residual[0], residual[1], residual[2]};
    status = measurement_update(&next, h, residual, c->zero_rate_noise_variance,
                                0, c->max_bias_step_rad_s);
    if (status == IMU_STATUS_OK) *f = next;
    return status;
}

imu_status_t imu_esekf_get_output(const imu_esekf_t *f, imu_output_t *out)
{
    imu_output_t next;
    imu_vec3f_t euler;
    float jacobian[3][3], variance[3] = {0};
    float sr, cr, cp, tp;
    unsigned i, j, k;
    imu_status_t status;
    if (out == NULL) return IMU_STATUS_BAD_ARGUMENT;
    status = state_valid(f);
    if (status != IMU_STATUS_OK) return status;
    if (!imu_quat_to_euler_ned(f->q_nb, &euler)) return IMU_STATUS_NUMERICAL_FAULT;
    sr = sinf(euler.x); cr = cosf(euler.x); cp = cosf(euler.y);
    /* Euler uncertainty is singular at pitch +/-pi/2. Quaternion remains valid. */
    if (fabsf(cp) < 1e-5f) return IMU_STATUS_NUMERICAL_FAULT;
    tp = sinf(euler.y)/cp;
    jacobian[0][0] = 1; jacobian[0][1] = sr*tp; jacobian[0][2] = cr*tp;
    jacobian[1][0] = 0; jacobian[1][1] = cr; jacobian[1][2] = -sr;
    jacobian[2][0] = 0; jacobian[2][1] = sr/cp; jacobian[2][2] = cr/cp;
    for (i = 0; i < 3; ++i) {
        for (j = 0; j < 3; ++j)
            for (k = 0; k < 3; ++k)
                variance[i] += jacobian[i][j]*f->P[j][k]*jacobian[i][k];
        if (!isfinite(variance[i]) || variance[i] <= 0)
            return IMU_STATUS_NUMERICAL_FAULT;
    }
    memset(&next, 0, sizeof(next));
    next.q_nb = f->q_nb;
    next.gyro_bias_rad_s = f->gyro_bias_rad_s;
    next.angular_rate_rad_s = f->angular_rate_rad_s;
    next.roll_rad = euler.x; next.pitch_rad = euler.y; next.yaw_rad = euler.z;
    next.attitude_std_rad = (imu_vec3f_t){sqrtf(variance[0]), sqrtf(variance[1]),
                                       sqrtf(variance[2])};
    next.initialized = true;
    next.health_flags = IMU_HEALTH_HEADING_UNOBSERVABLE;
    next.status = IMU_STATUS_OK;
    next.fusion_mode = IMU_FUSION_GYRO_ONLY;
    *out = next;
    return IMU_STATUS_OK;
}
