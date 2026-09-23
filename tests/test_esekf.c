#include "test_framework.h"
#include "imu/imu_esekf.h"
#include "imu/imu_math.h"

#include <float.h>
#include <string.h>

static imu_config_t config(void)
{
    imu_config_t c = {0};
    unsigned i;
    c.gyro_noise_density = 0.002f;
    c.gyro_bias_rw = 0.0001f;
    c.accel_direction_noise_variance = 0.0025f;
    c.zero_rate_noise_variance = 0.0001f;
    c.gravity_nis_limit = 25.0f;
    c.max_bias_step_rad_s = 0.01f;
    for (i = 0; i < 6; ++i)
        c.initial_covariance_diag[i] = i < 3 ? 0.04f : 0.001f;
    return c;
}

static imu_esekf_t identity_filter(const imu_config_t *c)
{
    imu_esekf_t f = {0};
    TEST_ASSERT(imu_esekf_init(&f, c, (imu_quatf_t){1, 0, 0, 0},
                             (imu_vec3f_t){0, 0, 0}) == IMU_STATUS_OK);
    return f;
}

/* Independent double Jacobi eigenvalues of the correlation matrix. This is
 * deliberately not the production factorization or its validity predicate. */
static double minimum_correlation_eigenvalue(const imu_esekf_t *f)
{
    double a[6][6], smallest;
    unsigned i, j, iteration;
    for (i = 0; i < 6; ++i)
        for (j = 0; j < 6; ++j)
            a[i][j] = (double)f->P[i][j]/sqrt((double)f->P[i][i]*f->P[j][j]);
    for (iteration = 0; iteration < 200; ++iteration) {
        unsigned p = 0, q = 1;
        double largest = 0, angle, c, s, app, aqq, apq;
        for (i = 0; i < 6; ++i)
            for (j = i+1; j < 6; ++j)
                if (fabs(a[i][j]) > largest) {
                    largest = fabs(a[i][j]); p = i; q = j;
                }
        if (largest < 1e-14) break;
        angle = 0.5*atan2(2*a[p][q], a[q][q]-a[p][p]);
        c = cos(angle); s = sin(angle);
        app = a[p][p]; aqq = a[q][q]; apq = a[p][q];
        for (i = 0; i < 6; ++i) {
            if (i == p || i == q) continue;
            const double aip = a[i][p], aiq = a[i][q];
            a[i][p] = a[p][i] = c*aip-s*aiq;
            a[i][q] = a[q][i] = s*aip+c*aiq;
        }
        a[p][p] = c*c*app-2*c*s*apq+s*s*aqq;
        a[q][q] = s*s*app+2*c*s*apq+c*c*aqq;
        a[p][q] = a[q][p] = 0;
    }
    TEST_ASSERT(iteration < 200);
    smallest = a[0][0];
    for (i = 1; i < 6; ++i) smallest = fmin(smallest, a[i][i]);
    return smallest;
}

/* The [1,2;2,1] block has eigenvalues 3 and -1 despite positive diagonals.
 * Removing PSD rejection must fail validation and every transactional API. */
static void test_indefinite_covariance(void)
{
    imu_config_t c = config();
    const float scales[] = {1e-12f, 1.0f, 1e12f};
    const uint32_t flags[3] = {1, 1, 1};
    unsigned i, trial;
    for (trial = 0; trial < 3; ++trial) {
        imu_esekf_t f = identity_filter(&c), before;
        imu_output_t out, saved;
        for (i = 0; i < 6; ++i) f.P[i][i] = scales[trial];
        f.P[0][1] = f.P[1][0] = 2*scales[trial];
        before = f;
        TEST_ASSERT_NEAR(minimum_correlation_eigenvalue(&f), -1, 1e-12);
        TEST_ASSERT(!imu_esekf_covariance_valid(&f));
        TEST_ASSERT(imu_esekf_predict(&f, &c, (imu_vec3f_t){0, 0, 0}, 0.01f)
                    == IMU_STATUS_NUMERICAL_FAULT);
        TEST_ASSERT(memcmp(&f, &before, sizeof(f)) == 0);
        f = before;
        TEST_ASSERT(imu_esekf_update_gravity(&f, &c, (imu_vec3f_t){0, 0, -9.8f}, 1)
                    == IMU_STATUS_NUMERICAL_FAULT);
        TEST_ASSERT(memcmp(&f, &before, sizeof(f)) == 0);
        f = before;
        TEST_ASSERT(imu_esekf_update_zero_rate(&f, &c, (imu_vec3f_t){0, 0, 0}, flags)
                    == IMU_STATUS_NUMERICAL_FAULT);
        TEST_ASSERT(memcmp(&f, &before, sizeof(f)) == 0);
        f = before;
        memset(&out, 0x5a, sizeof(out)); saved = out;
        TEST_ASSERT(imu_esekf_get_output(&f, &out) == IMU_STATUS_NUMERICAL_FAULT);
        TEST_ASSERT(memcmp(&out, &saved, sizeof(out)) == 0);
    }
}

/* A two-ulp negative mode is rounding scale, not a corrupt prior. Each
 * committing API must regularize it; a no-op/validator must not mutate it. */
static void test_roundoff_covariance(void)
{
    imu_config_t c = config();
    const float scales[] = {1e-12f, 1.0f, 1e12f};
    const uint32_t flags[3] = {1, 1, 1};
    unsigned trial, operation, i;
    c.gyro_noise_density = c.gyro_bias_rw = 0;
    for (trial = 0; trial < 3; ++trial) {
        c.accel_direction_noise_variance = 0.0025f*scales[trial];
        c.zero_rate_noise_variance = 0.0001f*scales[trial];
        for (operation = 0; operation < 3; ++operation) {
            imu_esekf_t f = identity_filter(&c), before;
            imu_status_t status;
            for (i = 0; i < 6; ++i) f.P[i][i] = scales[trial];
            /* An unobserved attitude block for zero-rate and yaw/bias block
             * for gravity; the negative mode survives a zero residual. */
            i = operation == 1 ? 5 : 0;
            f.P[2][i] = f.P[i][2] = scales[trial]*(1+2*FLT_EPSILON);
            before = f;
            TEST_ASSERT(minimum_correlation_eigenvalue(&f) < -FLT_EPSILON);
            TEST_ASSERT(imu_esekf_covariance_valid(&f));
            TEST_ASSERT(memcmp(&f, &before, sizeof(f)) == 0);
            if (operation == 0)
                status = imu_esekf_predict(&f, &c, (imu_vec3f_t){0, 0, 0}, 1e-8f);
            else if (operation == 1)
                status = imu_esekf_update_gravity(&f, &c, (imu_vec3f_t){0, 0, -9.8f}, 1);
            else
                status = imu_esekf_update_zero_rate(&f, &c, (imu_vec3f_t){0, 0, 0}, flags);
            TEST_ASSERT(status == IMU_STATUS_OK);
            TEST_ASSERT(minimum_correlation_eigenvalue(&f) >= -1e-12);
            for (i = 0; i < 6; ++i)
                TEST_ASSERT(f.P[i][i] <= before.P[i][i]*(1+64*FLT_EPSILON));
        }
    }
}

/* Independent double RK4 quaternion ODE truth; catches the multiplication side,
 * axes, sign, dt, and bias-subtraction bugs without using production helpers. */
static void derivative(const double q[4], const double w[3], double d[4])
{
    d[0] = -0.5 * (q[1]*w[0] + q[2]*w[1] + q[3]*w[2]);
    d[1] =  0.5 * (q[0]*w[0] + q[2]*w[2] - q[3]*w[1]);
    d[2] =  0.5 * (q[0]*w[1] + q[3]*w[0] - q[1]*w[2]);
    d[3] =  0.5 * (q[0]*w[2] + q[1]*w[1] - q[2]*w[0]);
}

static void truth_step(double q[4], const double w[3], double dt)
{
    double a[4], b[4], c[4], d[4], t[4];
    unsigned i;
    derivative(q, w, a);
    for (i = 0; i < 4; ++i) t[i] = q[i] + 0.5*dt*a[i];
    derivative(t, w, b);
    for (i = 0; i < 4; ++i) t[i] = q[i] + 0.5*dt*b[i];
    derivative(t, w, c);
    for (i = 0; i < 4; ++i) t[i] = q[i] + dt*c[i];
    derivative(t, w, d);
    for (i = 0; i < 4; ++i)
        q[i] += dt*(a[i] + 2*b[i] + 2*c[i] + d[i])/6.0;
}

static void test_propagation(void)
{
    imu_config_t c = config();
    imu_esekf_t f = identity_filter(&c);
    imu_output_t out = {0};
    double truth[4] = {1, 0, 0, 0};
    unsigned i, j;
    TEST_ASSERT(imu_esekf_predict(&f, &c, (imu_vec3f_t){0, 0, 0}, 0.01f)
                == IMU_STATUS_OK);
    TEST_ASSERT_NEAR(f.q_nb.w, 1, 0);
    TEST_ASSERT_NEAR(f.q_nb.x, 0, 0);
    TEST_ASSERT_NEAR(f.q_nb.y, 0, 0);
    TEST_ASSERT_NEAR(f.q_nb.z, 0, 0);
    /* Hand-calculated FPF' cross term and density-squared continuous Q. */
    TEST_ASSERT_NEAR(f.P[0][0], 0.04000014f, 5e-9f);
    TEST_ASSERT_NEAR(f.P[0][3], -0.00001f, 1e-11f);
    TEST_ASSERT_NEAR(f.P[3][3], 0.0010000001f, 2e-10f);
    TEST_ASSERT(f.P[2][2] > c.initial_covariance_diag[2]);
    f = identity_filter(&c);
    f.gyro_bias_rad_s = (imu_vec3f_t){0.02f, -0.03f, 0.04f};
    for (i = 0; i < 1500; ++i) {
        double w[3] = {0.1 + 0.2*sin(i*0.003), -0.3*cos(i*0.002), 0.4};
        imu_vec3f_t measured = {(float)w[0]+0.02f, (float)w[1]-0.03f, 0.44f};
        TEST_ASSERT(imu_esekf_predict(&f, &c, measured, 1.0f/485.0f)
                    == IMU_STATUS_OK);
        for (j = 0; j < 10; ++j) truth_step(truth, w, 1.0/4850.0);
    }
    TEST_ASSERT_NEAR(f.q_nb.w, truth[0], 2e-5f);
    TEST_ASSERT_NEAR(f.q_nb.x, truth[1], 2e-5f);
    TEST_ASSERT_NEAR(f.q_nb.y, truth[2], 2e-5f);
    TEST_ASSERT_NEAR(f.q_nb.z, truth[3], 2e-5f);
    TEST_ASSERT(imu_esekf_get_output(&f, &out) == IMU_STATUS_OK);
    TEST_ASSERT_NEAR(out.angular_rate_rad_s.z, 0.4f, 1e-7f);
    TEST_ASSERT(out.initialized);
    TEST_ASSERT(out.health_flags & IMU_HEALTH_HEADING_UNOBSERVABLE);
    TEST_ASSERT(imu_esekf_covariance_valid(&f));
}

/* Catches accumulated quaternion drift and missing covariance process noise. */
static void test_ten_minutes(void)
{
    imu_config_t c = config();
    imu_esekf_t f = identity_filter(&c);
    unsigned i;
    for (i = 0; i < 485U*600U; ++i) {
        imu_status_t status = imu_esekf_predict(&f, &c,
            (imu_vec3f_t){0.1f, -0.2f, 0.3f}, 1.0f/485.0f);
        TEST_ASSERT(status == IMU_STATUS_OK);
        if (status != IMU_STATUS_OK) {
            fprintf(stderr, "propagation rejected at step %u\n", i);
            return;
        }
        TEST_ASSERT_NEAR(imu_quat_norm(f.q_nb), 1, 2e-5f);
        if (i % 1000 == 0 || i == 485U*600U-1) {
            const double eigenvalue = minimum_correlation_eigenvalue(&f);
            TEST_ASSERT(eigenvalue >= -8*FLT_EPSILON);
            if (eigenvalue < -8*FLT_EPSILON) {
                fprintf(stderr, "non-PSD covariance at step %u: normalized eigenvalue %.9g\n",
                        i, eigenvalue);
                return;
            }
        }
    }
    TEST_ASSERT(imu_esekf_covariance_valid(&f));
    TEST_ASSERT(f.P[2][2] > c.initial_covariance_diag[2]);
}

/* Catches partial state writes on invalid configuration/input and corruption. */
static void test_prediction_guards(void)
{
    imu_config_t c = config();
    imu_esekf_t f = identity_filter(&c), before = f;
    TEST_ASSERT(imu_esekf_predict(&f, &c, (imu_vec3f_t){NAN, 0, 0}, 0.01f)
                == IMU_STATUS_NONFINITE);
    TEST_ASSERT(imu_esekf_predict(&f, &c, (imu_vec3f_t){0, 0, 0}, 0)
                == IMU_STATUS_BAD_ARGUMENT);
    TEST_ASSERT(memcmp(&f, &before, sizeof(f)) == 0);
    c.gyro_noise_density = -1;
    TEST_ASSERT(imu_esekf_predict(&f, &c, (imu_vec3f_t){0, 0, 0}, 0.01f)
                == IMU_STATUS_BAD_ARGUMENT);
    TEST_ASSERT(memcmp(&f, &before, sizeof(f)) == 0);
    c = config();
    c.initial_covariance_diag[2] = -1;
    TEST_ASSERT(imu_esekf_init(&f, &c, (imu_quatf_t){1, 0, 0, 0},
                             (imu_vec3f_t){0, 0, 0}) == IMU_STATUS_BAD_ARGUMENT);
    TEST_ASSERT(memcmp(&f, &before, sizeof(f)) == 0);
    c = config();
    f.P[0][0] = -1;
    before = f;
    TEST_ASSERT(!imu_esekf_covariance_valid(&f));
    TEST_ASSERT(imu_esekf_predict(&f, &c, (imu_vec3f_t){0, 0, 0}, 0.01f)
                == IMU_STATUS_NUMERICAL_FAULT);
    TEST_ASSERT(memcmp(&f, &before, sizeof(f)) == 0);
}

/* Catches gravity sign/side errors, noise/weight inversion, fictitious yaw
 * observations, omitted Joseph noise, and omitted error-coordinate reset. */
static void test_gravity(void)
{
    imu_config_t c = config(), noisy = c;
    imu_esekf_t f = identity_filter(&c), weak, before;
    imu_vec3f_t euler;
    unsigned axis, i, j;
    noisy.accel_direction_noise_variance = 0.2f;
    for (axis = 0; axis < 2; ++axis) {
        /* Yaw=0.8 followed by a body roll OR pitch of 0.2 radians. */
        f = identity_filter(&c);
        f.q_nb = axis == 0 ?
            (imu_quatf_t){0.9164595f, 0.09195267f, 0.03887696f, 0.3874729f} :
            (imu_quatf_t){0.9164595f, -0.03887696f, 0.09195267f, 0.3874729f};
        TEST_ASSERT(imu_quat_normalize(&f.q_nb));
        weak = f;
        TEST_ASSERT(imu_esekf_update_gravity(&weak, &noisy,
                    (imu_vec3f_t){0, 0, -9.80665f}, 1) == IMU_STATUS_OK);
        TEST_ASSERT(imu_esekf_update_gravity(&f, &c,
                    (imu_vec3f_t){0, 0, -9.80665f}, 1) == IMU_STATUS_OK);
        TEST_ASSERT(imu_quat_to_euler_ned(f.q_nb, &euler));
        TEST_ASSERT(fabsf(euler.x) < 0.014f && fabsf(euler.y) < 0.014f);
        TEST_ASSERT_NEAR(euler.z, 0.8f, 2e-6f);
        TEST_ASSERT(imu_quat_to_euler_ned(weak.q_nb, &euler));
        TEST_ASSERT(fabsf(euler.x) + fabsf(euler.y) > 0.15f);
        TEST_ASSERT(imu_esekf_covariance_valid(&f));
        for (i = 0; i < 6; ++i)
            for (j = 0; j < 6; ++j) TEST_ASSERT_NEAR(f.P[i][j], f.P[j][i], 0);
    }
    f = identity_filter(&c);
    before = f;
    TEST_ASSERT(imu_esekf_update_gravity(&f, &c, (imu_vec3f_t){0, 0, -9.8f}, 0)
                == IMU_STATUS_OK);
    TEST_ASSERT(memcmp(&f, &before, sizeof(f)) == 0);
    TEST_ASSERT(imu_esekf_update_gravity(&f, &c,
                (imu_vec3f_t){0, -1.9476176f, -9.607931f}, 1) == IMU_STATUS_OK);
    TEST_ASSERT_NEAR(f.P[0][0], 0.0023529412f, 1e-8f);
    /* dtheta_x=(.04/.0425)*sin(.2); G=I-.5*[dtheta]x. */
    TEST_ASSERT_NEAR(f.P[1][2], 0.003519678f, 2e-6f);
    TEST_ASSERT_NEAR(f.P[2][2], 0.04002057f, 2e-7f);
    weak = identity_filter(&c);
    TEST_ASSERT(imu_esekf_update_gravity(&weak, &c,
                (imu_vec3f_t){0, -1.9476176f, -9.607931f}, 0.1f) == IMU_STATUS_OK);
    TEST_ASSERT(fabsf(weak.q_nb.x) < fabsf(f.q_nb.x));
    /* Gravity alone must never tighten the level heading uncertainty. */
    f = identity_filter(&c);
    f.q_nb = (imu_quatf_t){0.921060994f, 0, 0, 0.389418342f};
    for (i = 0; i < 500; ++i)
        TEST_ASSERT(imu_esekf_update_gravity(&f, &c,
            (imu_vec3f_t){0, 0, -9.8f}, 1) == IMU_STATUS_OK);
    TEST_ASSERT_NEAR(imu_quat_yaw_ned(f.q_nb), 0.8f, 2e-6f);
    TEST_ASSERT_NEAR(f.P[2][2], 0.04f, 1e-7f);
    TEST_ASSERT_NEAR(f.P[5][5], 0.001f, 1e-9f);
}

/* Cross residual must use a body-frame nullspace independent of navigation
 * yaw. Left-multiplying identical heading offsets commutes with the update. */
static void test_gravity_yaw_equivariance(void)
{
    imu_config_t c = config();
    imu_esekf_t a = identity_filter(&c), b;
    const imu_quatf_t heading = {0.825335615f, 0, 0, 0.564642473f};
    imu_quatf_t expected;
    unsigned i, j;
    a.q_nb = (imu_quatf_t){0.983831341f, 0.148691564f, 0.098712395f, -0.014918919f};
    b = a;
    b.q_nb = imu_quat_multiply(heading, a.q_nb);
    TEST_ASSERT(imu_esekf_update_gravity(&a, &c, (imu_vec3f_t){0, 0, -9.8f}, 1)
                == IMU_STATUS_OK);
    TEST_ASSERT(imu_esekf_update_gravity(&b, &c, (imu_vec3f_t){0, 0, -9.8f}, 1)
                == IMU_STATUS_OK);
    expected = imu_quat_multiply(heading, a.q_nb);
    TEST_ASSERT_NEAR(b.q_nb.w, expected.w, 2e-6f);
    TEST_ASSERT_NEAR(b.q_nb.x, expected.x, 2e-6f);
    TEST_ASSERT_NEAR(b.q_nb.y, expected.y, 2e-6f);
    TEST_ASSERT_NEAR(b.q_nb.z, expected.z, 2e-6f);
    for (i = 0; i < 6; ++i)
        for (j = 0; j < 6; ++j) TEST_ASSERT_NEAR(a.P[i][j], b.P[i][j], 1e-7f);
}

/* Catches NIS acceptance, antiparallel cross-product ambiguity, unchecked
 * ill-conditioned innovation inversion, and any partial mutation on failure. */
static void test_gravity_guards(void)
{
    imu_config_t c = config();
    imu_esekf_t f = identity_filter(&c), before = f;
    c.gravity_nis_limit = 0.01f;
    TEST_ASSERT(imu_esekf_update_gravity(&f, &c,
        (imu_vec3f_t){0, -1.9476176f, -9.607931f}, 1) == IMU_STATUS_AIDING_UNAVAILABLE);
    TEST_ASSERT(memcmp(&f, &before, sizeof(f)) == 0);
    c = config();
    c.accel_direction_noise_variance = 1e-15f;
    TEST_ASSERT(imu_esekf_update_gravity(&f, &c,
        (imu_vec3f_t){0, 0, -9.8f}, 1) == IMU_STATUS_NUMERICAL_FAULT);
    TEST_ASSERT(memcmp(&f, &before, sizeof(f)) == 0);
    c = config();
    TEST_ASSERT(imu_esekf_update_gravity(&f, &c,
        (imu_vec3f_t){0, 0, 9.8f}, 1) == IMU_STATUS_AIDING_UNAVAILABLE);
    TEST_ASSERT(imu_esekf_update_gravity(&f, &c,
        (imu_vec3f_t){0, 0, 0}, 1) == IMU_STATUS_BAD_ARGUMENT);
    TEST_ASSERT(imu_esekf_update_gravity(&f, &c,
        (imu_vec3f_t){NAN, 0, -9.8f}, 1) == IMU_STATUS_NONFINITE);
    TEST_ASSERT(imu_esekf_update_gravity(&f, &c,
        (imu_vec3f_t){0, 0, -9.8f}, 1.1f) == IMU_STATUS_BAD_ARGUMENT);
    TEST_ASSERT(memcmp(&f, &before, sizeof(f)) == 0);
}

/* Catches missing bias axes, wrong correction sign, unbounded bias steps,
 * covariance overconfidence when clipping, and failure to reuse cross-P. */
static void test_zero_rate(void)
{
    imu_config_t c = config();
    imu_esekf_t f = identity_filter(&c), before;
    const uint32_t healthy[3] = {IMU_SENSOR_DATA_VALID, IMU_SENSOR_DATA_VALID,
                                 IMU_SENSOR_DATA_VALID};
    const imu_vec3f_t measured = {0.02f, -0.03f, 0.04f};
    imu_vec3f_t step;
    unsigned i;
    TEST_ASSERT(imu_esekf_update_zero_rate(&f, &c, measured, healthy) == IMU_STATUS_OK);
    TEST_ASSERT_NEAR(imu_vec3_norm(f.gyro_bias_rad_s), 0.01f, 1e-7f);
    TEST_ASSERT_NEAR(f.P[3][3], 0.000666540f, 2e-8f);
    for (i = 0; i < 200; ++i) {
        before = f;
        TEST_ASSERT(imu_esekf_update_zero_rate(&f, &c, measured, healthy)
                    == IMU_STATUS_OK);
        step = (imu_vec3f_t){f.gyro_bias_rad_s.x-before.gyro_bias_rad_s.x,
            f.gyro_bias_rad_s.y-before.gyro_bias_rad_s.y,
            f.gyro_bias_rad_s.z-before.gyro_bias_rad_s.z};
        TEST_ASSERT(imu_vec3_norm(step) <= c.max_bias_step_rad_s+1e-7f);
        TEST_ASSERT(imu_esekf_covariance_valid(&f));
    }
    TEST_ASSERT_VEC3_NEAR(f.gyro_bias_rad_s, measured, 0.0001f);
    TEST_ASSERT(f.P[3][3] < 1e-6f && f.P[4][4] < 1e-6f && f.P[5][5] < 1e-6f);
    /* A prior attitude/bias correlation lets rate aiding remove accumulated
     * drift. This must apply to yaw as well, without claiming absolute yaw. */
    f = identity_filter(&c);
    TEST_ASSERT(imu_esekf_predict(&f, &c, (imu_vec3f_t){0, 0, 0.005f}, 0.1f)
                == IMU_STATUS_OK);
    before = f;
    TEST_ASSERT(imu_esekf_update_zero_rate(&f, &c, (imu_vec3f_t){0, 0, 0.005f},
                                         healthy) == IMU_STATUS_OK);
    TEST_ASSERT(f.q_nb.z < before.q_nb.z);
    TEST_ASSERT(f.P[2][2] < before.P[2][2]);
}

#define ASSERT_UNCHANGED_CALL(expression, expected) do { \
    imu_esekf_t saved_ = f; \
    TEST_ASSERT((expression) == (expected)); \
    TEST_ASSERT(memcmp(&f, &saved_, sizeof(f)) == 0); \
} while (0)

/* At the smallest float scale, averaging raw entries first erases every
 * off-diagonal. The true correlation is 2I-11', with eigenvalues -4 and 2. */
static void test_subnormal_indefinite_covariance(void)
{
    imu_config_t c = config();
    imu_esekf_t f = identity_filter(&c);
    const float smallest = nextafterf(0.0f, 1.0f); /* C99 FLT_TRUE_MIN. */
    const uint32_t flags[3] = {1, 1, 1};
    imu_output_t out, saved_out;
    unsigned i, j;
    TEST_ASSERT(smallest > 0 && smallest < FLT_MIN);
    for (i = 0; i < 6; ++i)
        for (j = 0; j < 6; ++j)
            f.P[i][j] = i == j ? smallest : -smallest;
    TEST_ASSERT_NEAR(minimum_correlation_eigenvalue(&f), -4, 1e-12);
    TEST_ASSERT(!imu_esekf_covariance_valid(&f));
    ASSERT_UNCHANGED_CALL(imu_esekf_predict(&f, &c, ((imu_vec3f_t){0, 0, 0}), 0.01f),
                          IMU_STATUS_NUMERICAL_FAULT);
    /* Restore the fixture after a failed rollback assertion in a RED run. */
    for (i = 0; i < 6; ++i)
        for (j = 0; j < 6; ++j)
            f.P[i][j] = i == j ? smallest : -smallest;
    ASSERT_UNCHANGED_CALL(imu_esekf_update_gravity(&f, &c,
                          ((imu_vec3f_t){0, 0, -9.8f}), 0),
                          IMU_STATUS_NUMERICAL_FAULT);
    ASSERT_UNCHANGED_CALL(imu_esekf_update_gravity(&f, &c,
                          ((imu_vec3f_t){0, 0, -9.8f}), 1),
                          IMU_STATUS_NUMERICAL_FAULT);
    ASSERT_UNCHANGED_CALL(imu_esekf_update_zero_rate(&f, &c,
                          ((imu_vec3f_t){0, 0, 0}), flags),
                          IMU_STATUS_NUMERICAL_FAULT);
    memset(&out, 0x5a, sizeof(out));
    memcpy(&saved_out, &out, sizeof(out));
    TEST_ASSERT(imu_esekf_get_output(&f, &out) == IMU_STATUS_NUMERICAL_FAULT);
    TEST_ASSERT(memcmp(&out, &saved_out, sizeof(out)) == 0);
}

/* All failing calls protect q, bias, P, rate and lifecycle byte-for-byte,
 * including failures reached only after intermediate arithmetic overflow. */
static void test_update_rollback(void)
{
    imu_config_t c = config();
    imu_esekf_t f = identity_filter(&c);
    const imu_vec3f_t gyro = {0.02f, -0.03f, 0.04f};
    const imu_vec3f_t gravity = {0, 0, -9.8f};
    uint32_t flags[3] = {1, 1, 1};
    imu_output_t out, saved_out;
    unsigned axis, bit;
    for (axis = 0; axis < 3; ++axis) {
        flags[axis] = 0;
        ASSERT_UNCHANGED_CALL(imu_esekf_update_zero_rate(&f, &c, gyro, flags),
                              IMU_STATUS_SENSOR_DEGRADED);
        for (bit = 1; bit < 8; ++bit) {
            flags[axis] = IMU_SENSOR_DATA_VALID | (UINT32_C(1) << bit);
            ASSERT_UNCHANGED_CALL(imu_esekf_update_zero_rate(&f, &c, gyro, flags),
                                  IMU_STATUS_SENSOR_DEGRADED);
        }
        flags[axis] = 1;
    }
    ASSERT_UNCHANGED_CALL(imu_esekf_update_zero_rate(&f, &c, gyro, NULL),
                          IMU_STATUS_BAD_ARGUMENT);
    ASSERT_UNCHANGED_CALL(imu_esekf_update_zero_rate(&f, NULL, gyro, flags),
                          IMU_STATUS_BAD_ARGUMENT);
    ASSERT_UNCHANGED_CALL(imu_esekf_update_zero_rate(&f, &c,
                          ((imu_vec3f_t){NAN, 0, 0}), flags), IMU_STATUS_NONFINITE);
    c.zero_rate_noise_variance = 0;
    ASSERT_UNCHANGED_CALL(imu_esekf_update_zero_rate(&f, &c, gyro, flags),
                          IMU_STATUS_BAD_ARGUMENT);
    c.zero_rate_noise_variance = NAN;
    ASSERT_UNCHANGED_CALL(imu_esekf_update_zero_rate(&f, &c, gyro, flags),
                          IMU_STATUS_NONFINITE);
    c = config();
    c.max_bias_step_rad_s = -1;
    ASSERT_UNCHANGED_CALL(imu_esekf_update_zero_rate(&f, &c, gyro, flags),
                          IMU_STATUS_BAD_ARGUMENT);
    ASSERT_UNCHANGED_CALL(imu_esekf_update_gravity(&f, &c, gravity, 1),
                          IMU_STATUS_BAD_ARGUMENT);
    c.max_bias_step_rad_s = INFINITY;
    ASSERT_UNCHANGED_CALL(imu_esekf_update_zero_rate(&f, &c, gyro, flags),
                          IMU_STATUS_NONFINITE);
    ASSERT_UNCHANGED_CALL(imu_esekf_update_gravity(&f, &c, gravity, 1),
                          IMU_STATUS_NONFINITE);
    c = config();
    ASSERT_UNCHANGED_CALL(imu_esekf_predict(&f, &c, gyro, FLT_MAX),
                          IMU_STATUS_NUMERICAL_FAULT);
    ASSERT_UNCHANGED_CALL(imu_esekf_update_zero_rate(&f, &c,
                          ((imu_vec3f_t){FLT_MAX, 0, 0}), flags),
                          IMU_STATUS_NUMERICAL_FAULT);
    c.accel_direction_noise_variance = FLT_MAX;
    ASSERT_UNCHANGED_CALL(imu_esekf_update_gravity(&f, &c, gravity, 0.01f),
                          IMU_STATUS_NUMERICAL_FAULT);
    c = config();
    f.P[5][5] = 1e-15f;
    c.zero_rate_noise_variance = 1e-15f;
    ASSERT_UNCHANGED_CALL(imu_esekf_update_zero_rate(&f, &c, gyro, flags),
                          IMU_STATUS_NUMERICAL_FAULT);
    c = config();
    f = identity_filter(&c);
    f.P[0][3] = NAN;
    ASSERT_UNCHANGED_CALL(imu_esekf_update_zero_rate(&f, &c, gyro, flags),
                          IMU_STATUS_NUMERICAL_FAULT);
    ASSERT_UNCHANGED_CALL(imu_esekf_update_gravity(&f, &c, gravity, 1),
                          IMU_STATUS_NUMERICAL_FAULT);
    f = identity_filter(&c);
    f.q_nb = (imu_quatf_t){0, 0, 0, 0};
    ASSERT_UNCHANGED_CALL(imu_esekf_update_zero_rate(&f, &c, gyro, flags),
                          IMU_STATUS_NUMERICAL_FAULT);
    f = identity_filter(&c);
    f.initialized = false;
    ASSERT_UNCHANGED_CALL(imu_esekf_predict(&f, &c, gyro, 0.01f),
                          IMU_STATUS_NOT_INITIALIZED);
    ASSERT_UNCHANGED_CALL(imu_esekf_update_gravity(&f, &c, gravity, 1),
                          IMU_STATUS_NOT_INITIALIZED);
    ASSERT_UNCHANGED_CALL(imu_esekf_update_zero_rate(&f, &c, gyro, flags),
                          IMU_STATUS_NOT_INITIALIZED);
    memset(&out, 0x5a, sizeof(out));
    memcpy(&saved_out, &out, sizeof(out));
    TEST_ASSERT(imu_esekf_get_output(&f, &out) == IMU_STATUS_NOT_INITIALIZED);
    TEST_ASSERT(memcmp(&out, &saved_out, sizeof(out)) == 0);
    TEST_ASSERT(imu_esekf_init(NULL, &c, (imu_quatf_t){1, 0, 0, 0}, gyro)
                == IMU_STATUS_BAD_ARGUMENT);
    TEST_ASSERT(imu_esekf_predict(NULL, &c, gyro, 1) == IMU_STATUS_BAD_ARGUMENT);
    TEST_ASSERT(imu_esekf_update_gravity(NULL, &c, gravity, 1) == IMU_STATUS_BAD_ARGUMENT);
    TEST_ASSERT(imu_esekf_update_zero_rate(NULL, &c, gyro, flags) == IMU_STATUS_BAD_ARGUMENT);
    TEST_ASSERT(imu_esekf_get_output(NULL, &out) == IMU_STATUS_BAD_ARGUMENT);
}

/* Gyro-bias step limits also constrain gravity updates through attitude/bias
 * cross covariance, and the same limited gain must drive Joseph covariance. */
static void test_gravity_bias_limit(void)
{
    imu_config_t c = config();
    imu_esekf_t f = identity_filter(&c);
    c.max_bias_step_rad_s = 0.0001f;
    TEST_ASSERT(imu_esekf_predict(&f, &c, (imu_vec3f_t){0, 0, 0}, 1)
                == IMU_STATUS_OK);
    TEST_ASSERT(imu_esekf_update_gravity(&f, &c,
        (imu_vec3f_t){0, -1.9476176f, -9.607931f}, 1) == IMU_STATUS_OK);
    TEST_ASSERT_NEAR(imu_vec3_norm(f.gyro_bias_rad_s), c.max_bias_step_rad_s, 1e-8f);
    TEST_ASSERT(f.P[0][0] > 0.03f);
}

int main(int argc, char **argv)
{
    if (argc == 1 || strcmp(argv[1], "psd") == 0) {
        test_indefinite_covariance();
        test_roundoff_covariance();
        test_subnormal_indefinite_covariance();
    }
    if (argc == 1 || strcmp(argv[1], "propagation") == 0) {
        test_propagation();
        test_prediction_guards();
        test_ten_minutes();
    }
    if (argc == 1 || strcmp(argv[1], "gravity") == 0) {
        test_gravity();
        test_gravity_yaw_equivariance();
        test_gravity_guards();
    }
    if (argc == 1 || strcmp(argv[1], "zero") == 0) {
        test_zero_rate();
        test_update_rollback();
        test_gravity_bias_limit();
    }
    return test_summary();
}
