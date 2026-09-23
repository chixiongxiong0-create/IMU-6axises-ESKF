#include "test_framework.h"
#include "imu/imu_complementary.h"
#include "imu/imu_math.h"

#include <math.h>
#include <string.h>

#define STEP_NS UINT64_C(100000000)

static imu_config_t config(void)
{
    imu_config_t c = {0};
    c.min_dt_s = 0.001f;
    c.max_dt_s = 1.0f;
    c.complementary_kp = 2.0f;
    c.complementary_ki = 0.5f;
    c.max_bias_step_rad_s = 0.01f;
    c.accel_weight_min = 0.0f;
    return c;
}

static imu_sample_t sample_at(uint64_t time_ns)
{
    imu_sample_t s = {0};
    unsigned i;
    s.timestamp_ns = time_ns;
    s.accel_m_s2[2] = -9.80665f;
    s.accel_status = IMU_SENSOR_DATA_VALID;
    s.accel_sample_fresh = true;
    for (i = 0U; i < IMU_AXIS_COUNT; ++i) {
        s.gyro_status[i] = IMU_SENSOR_DATA_VALID;
        s.gyro_sample_fresh[i] = true;
    }
    return s;
}

static imu_motion_quality_t quality_at(uint64_t time_ns, float weight)
{
    imu_motion_quality_t q = {0};
    q.initialized = true;
    q.last_timestamp_ns = time_ns;
    q.accel_weight = weight;
    return q;
}

static imu_complementary_t identity_filter(const imu_config_t *c)
{
    imu_complementary_t f;
    TEST_ASSERT(imu_complementary_init(&f, c, (imu_quatf_t){1,0,0,0},
                                      (imu_vec3f_t){0,0,0}) == IMU_STATUS_OK);
    return f;
}

static void process_at(imu_complementary_t *f, const imu_config_t *c,
                       imu_sample_t *s, float weight, imu_output_t *out)
{
    const imu_motion_quality_t q = quality_at(s->timestamp_ns, weight);
    TEST_ASSERT(imu_complementary_process(f, c, s, &q, out) == IMU_STATUS_OK);
}

/* Catches spurious quaternion movement at exact zero angular rate. */
static void test_zero_rate_stability(void)
{
    const imu_config_t c = config();
    imu_complementary_t f = identity_filter(&c);
    imu_output_t out;
    imu_sample_t s = sample_at(STEP_NS);
    process_at(&f, &c, &s, 1.0f, &out);
    s.timestamp_ns += STEP_NS;
    process_at(&f, &c, &s, 1.0f, &out);
    TEST_ASSERT(f.q_nb.w == 1.0f && f.q_nb.x == 0.0f &&
                f.q_nb.y == 0.0f && f.q_nb.z == 0.0f);
    TEST_ASSERT(out.initialized);
    TEST_ASSERT((out.health_flags & IMU_HEALTH_HEADING_UNOBSERVABLE) != 0U);
}

/* Catches a fixed nominal period or the wrong quaternion multiplication. */
static void test_known_rate_propagation(void)
{
    imu_config_t c = config();
    imu_complementary_t f = identity_filter(&c);
    imu_output_t out;
    imu_sample_t s = sample_at(STEP_NS);
    c.complementary_kp = 0.0f;
    c.complementary_ki = 0.0f;
    process_at(&f, &c, &s, 0.0f, &out);
    s.timestamp_ns += STEP_NS;
    s.gyro_rad_s[2] = 1.0f;
    process_at(&f, &c, &s, 0.0f, &out);
    TEST_ASSERT_NEAR(f.q_nb.w, cosf(0.05f), 1.0e-6f);
    TEST_ASSERT_NEAR(f.q_nb.z, sinf(0.05f), 1.0e-6f);
    TEST_ASSERT_NEAR(out.yaw_rad, 0.1f, 1.0e-6f);
}

/* Catches missing quality scaling, a sign reversal, and corrections at zero
 * weight. A horizontal specific-force component produces a body-y residual. */
static void test_gravity_correction_uses_shared_weight(void)
{
    imu_config_t c = config();
    imu_complementary_t full = identity_filter(&c);
    imu_complementary_t half = identity_filter(&c);
    imu_complementary_t none = identity_filter(&c);
    imu_output_t out;
    imu_sample_t s = sample_at(STEP_NS);
    c.complementary_ki = 0.0f;
    process_at(&full, &c, &s, 1.0f, &out);
    process_at(&half, &c, &s, 0.5f, &out);
    process_at(&none, &c, &s, 0.0f, &out);
    s.timestamp_ns += STEP_NS;
    s.accel_m_s2[0] = 9.80665f * sinf(0.2f);
    s.accel_m_s2[2] = -9.80665f * cosf(0.2f);
    process_at(&full, &c, &s, 1.0f, &out);
    process_at(&half, &c, &s, 0.5f, &out);
    process_at(&none, &c, &s, 0.0f, &out);
    TEST_ASSERT_NEAR(full.angular_rate_rad_s.y, 2.0f*sinf(0.2f), 1.0e-6f);
    TEST_ASSERT_NEAR(half.angular_rate_rad_s.y, sinf(0.2f), 1.0e-6f);
    TEST_ASSERT_NEAR(none.q_nb.y, 0.0f, 0.0f);
    TEST_ASSERT_NEAR(none.gyro_bias_rad_s.y, 0.0f, 0.0f);
}

/* Catches a vertical-axis gravity correction masquerading as heading aiding. */
static void test_yaw_is_relative(void)
{
    const imu_config_t c = config();
    const imu_quatf_t yawed = {0.87758255f,0,0,0.47942555f};
    imu_complementary_t f;
    imu_output_t out;
    imu_sample_t s = sample_at(STEP_NS);
    TEST_ASSERT(imu_complementary_init(&f, &c, yawed,
                                      (imu_vec3f_t){0,0,0}) == IMU_STATUS_OK);
    process_at(&f, &c, &s, 1.0f, &out);
    s.timestamp_ns += STEP_NS;
    process_at(&f, &c, &s, 1.0f, &out);
    TEST_ASSERT_NEAR(out.yaw_rad, 1.0f, 1.0e-6f);
    TEST_ASSERT_NEAR(out.gyro_bias_rad_s.z, 0.0f, 0.0f);
    TEST_ASSERT((out.health_flags & IMU_HEALTH_HEADING_UNOBSERVABLE) != 0U);
}

/* Catches unbounded integral windup under a persistent tilted observation. */
static void test_integral_bias_is_clamped(void)
{
    imu_config_t c = config();
    imu_complementary_t f = identity_filter(&c);
    imu_output_t out;
    imu_sample_t s = sample_at(STEP_NS);
    unsigned i;
    c.complementary_kp = 0.0f;
    c.complementary_ki = 10.0f;
    process_at(&f, &c, &s, 1.0f, &out);
    for (i = 0U; i < 20U; ++i) {
        s.timestamp_ns += STEP_NS;
        s.accel_m_s2[0] = 9.80665f * sinf(0.2f);
        s.accel_m_s2[2] = -9.80665f * cosf(0.2f);
        process_at(&f, &c, &s, 1.0f, &out);
    }
    TEST_ASSERT(imu_vec3_norm(f.gyro_bias_rad_s) <=
                c.max_bias_step_rad_s + 1.0e-7f);
    TEST_ASSERT(f.gyro_bias_rad_s.y < 0.0f);
}

/* Catches zero gain acting on gravity or rejecting valid gain boundaries. */
static void test_zero_and_negative_gains(void)
{
    imu_config_t c = config();
    imu_complementary_t f;
    imu_complementary_t before;
    imu_output_t out;
    imu_sample_t s = sample_at(STEP_NS);
    c.complementary_kp = 0.0f;
    c.complementary_ki = 0.0f;
    f = identity_filter(&c);
    process_at(&f, &c, &s, 1.0f, &out);
    s.timestamp_ns += STEP_NS;
    s.accel_m_s2[0] = 1.0f;
    process_at(&f, &c, &s, 1.0f, &out);
    TEST_ASSERT_NEAR(f.q_nb.y, 0.0f, 0.0f);
    TEST_ASSERT_NEAR(f.gyro_bias_rad_s.y, 0.0f, 0.0f);
    before = f;
    c.complementary_kp = -0.1f;
    TEST_ASSERT(imu_complementary_init(&f, &c, (imu_quatf_t){1,0,0,0},
                                      (imu_vec3f_t){0,0,0}) == IMU_STATUS_BAD_ARGUMENT);
    TEST_ASSERT(memcmp(&f, &before, sizeof(f)) == 0);
    c.complementary_kp = 0.0f;
    c.complementary_ki = -0.1f;
    TEST_ASSERT(imu_complementary_init(&f, &c, (imu_quatf_t){1,0,0,0},
                                      (imu_vec3f_t){0,0,0}) == IMU_STATUS_BAD_ARGUMENT);
    TEST_ASSERT(memcmp(&f, &before, sizeof(f)) == 0);
}

/* Catches lagged quality being reused when acceleration is stale. */
static void test_stale_acceleration_and_recovery(void)
{
    const imu_config_t c = config();
    imu_complementary_t f = identity_filter(&c);
    imu_motion_quality_t q;
    imu_output_t out;
    imu_sample_t s = sample_at(STEP_NS);
    process_at(&f, &c, &s, 1.0f, &out);
    TEST_ASSERT_NEAR(out.accel_weight, 1.0f, 0.0f);
    s.timestamp_ns += STEP_NS;
    s.accel_m_s2[0] = 2.0f;
    s.accel_sample_fresh = false;
    q = quality_at(s.timestamp_ns - STEP_NS, 1.0f);
    TEST_ASSERT(imu_complementary_process(&f, &c, &s, &q, &out)
                == IMU_STATUS_SENSOR_DEGRADED);
    TEST_ASSERT_NEAR(f.q_nb.y, 0.0f, 0.0f);
    TEST_ASSERT_NEAR(out.accel_weight, 0.0f, 0.0f);
    s.timestamp_ns += STEP_NS;
    s.accel_sample_fresh = true;
    process_at(&f, &c, &s, 0.5f, &out);
    TEST_ASSERT(f.q_nb.y > 0.0f);
    TEST_ASSERT_NEAR(out.accel_weight, 0.5f, 0.0f);
}

static void assert_rollback(imu_complementary_t *f, const imu_config_t *c,
                            imu_sample_t *s, const imu_motion_quality_t *q,
                            imu_output_t *out, imu_status_t expected)
{
    const imu_complementary_t before = *f;
    const imu_output_t prior = *out;
    TEST_ASSERT(imu_complementary_process(f, c, s, q, out) == expected);
    TEST_ASSERT(memcmp(f, &before, sizeof(*f)) == 0);
    TEST_ASSERT(memcmp(out, &prior, sizeof(*out)) == 0);
}

/* Catches partial timestamp, output, or bias writes on invalid input. */
static void test_nonfinite_and_time_guards(void)
{
    const imu_config_t c = config();
    imu_complementary_t f = identity_filter(&c);
    imu_output_t out;
    imu_sample_t s = sample_at(STEP_NS);
    imu_motion_quality_t q;
    process_at(&f, &c, &s, 1.0f, &out);
    s.timestamp_ns += STEP_NS;
    q = quality_at(s.timestamp_ns, 1.0f);
    s.gyro_rad_s[0] = NAN;
    assert_rollback(&f, &c, &s, &q, &out, IMU_STATUS_NONFINITE);
    s.gyro_rad_s[0] = 0.0f;
    s.accel_temperature_c = INFINITY;
    assert_rollback(&f, &c, &s, &q, &out, IMU_STATUS_NONFINITE);
    s.accel_temperature_c = 0.0f;
    q.accel_weight = NAN;
    assert_rollback(&f, &c, &s, &q, &out, IMU_STATUS_NONFINITE);
    q.accel_weight = 1.0f;
    s.timestamp_ns = STEP_NS;
    assert_rollback(&f, &c, &s, &q, &out, IMU_STATUS_TIME_REVERSED);
    s.timestamp_ns = STEP_NS + UINT64_C(100);
    assert_rollback(&f, &c, &s, &q, &out, IMU_STATUS_BAD_ARGUMENT);
}

/* Catches integrating across a missing interval or replaying that interval
 * at recovery instead of consuming it exactly once. */
static void test_gap_consumption_and_recovery(void)
{
    const imu_config_t c = config();
    imu_complementary_t f = identity_filter(&c);
    imu_motion_quality_t q;
    imu_output_t out;
    imu_sample_t s = sample_at(STEP_NS);
    process_at(&f, &c, &s, 0.0f, &out);
    s.timestamp_ns += UINT64_C(2000000000);
    s.gyro_rad_s[2] = 1.0f;
    q = quality_at(s.timestamp_ns, 0.0f);
    TEST_ASSERT(imu_complementary_process(&f, &c, &s, &q, &out)
                == IMU_STATUS_TIME_GAP);
    TEST_ASSERT_NEAR(out.yaw_rad, 0.0f, 0.0f);
    TEST_ASSERT((out.health_flags & IMU_HEALTH_TIMING_DEGRADED) != 0U);
    s.timestamp_ns += STEP_NS;
    process_at(&f, &c, &s, 0.0f, &out);
    TEST_ASSERT_NEAR(out.yaw_rad, 0.1f, 1.0e-6f);
}

/* Catches a vanishing cross product from antiparallel gravity being reported
 * as a valid gravity observation. Propagation still succeeds. */
static void test_antiparallel_gravity_is_rejected(void)
{
    const imu_config_t c = config();
    imu_complementary_t f = identity_filter(&c);
    imu_motion_quality_t q;
    imu_output_t out;
    imu_sample_t s = sample_at(STEP_NS);
    process_at(&f, &c, &s, 1.0f, &out);
    s.timestamp_ns += STEP_NS;
    s.accel_m_s2[2] = 9.80665f;
    s.gyro_rad_s[2] = 1.0f;
    q = quality_at(s.timestamp_ns, 1.0f);
    TEST_ASSERT(imu_complementary_process(&f, &c, &s, &q, &out)
                == IMU_STATUS_SENSOR_DEGRADED);
    TEST_ASSERT_NEAR(out.accel_weight, 0.0f, 0.0f);
    TEST_ASSERT(out.fusion_mode == IMU_FUSION_GYRO_ONLY);
    TEST_ASSERT_NEAR(out.yaw_rad, 0.1f, 1.0e-6f);
}

int main(void)
{
    test_zero_rate_stability();
    test_known_rate_propagation();
    test_gravity_correction_uses_shared_weight();
    test_yaw_is_relative();
    test_integral_bias_is_clamped();
    test_zero_and_negative_gains();
    test_stale_acceleration_and_recovery();
    test_nonfinite_and_time_guards();
    test_gap_consumption_and_recovery();
    test_antiparallel_gravity_is_rejected();
    return test_summary();
}
